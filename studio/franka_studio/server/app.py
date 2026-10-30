"""aiohttp server: the single-page UI, a JSON API, a status WebSocket and camera snapshots.

Every action is a ``POST /api/<action>`` with a JSON body, answered with ``{ok, message}``.
Blocking work (ROS service calls, file moves, ssh) runs in the default thread pool.
The WebSocket pushes ``status`` every 250 ms and new log lines of the component the page follows.
"""

from __future__ import annotations

import asyncio
import functools
import json
import logging

import cv2
import numpy as np
from aiohttp import WSMsgType, web

from franka_studio.paths import STATIC_DIR
from franka_studio.studio import Studio

log = logging.getLogger(__name__)
STATUS_PERIOD_S = 0.25


def _json(data: object) -> web.Response:
    return web.json_response(data, dumps=functools.partial(json.dumps, default=str))


@web.middleware
async def revalidate_static(request: web.Request, handler: object) -> web.StreamResponse:
    """Browsers must revalidate app.js / app.css, or they keep running an old UI after an update."""
    response = await handler(request)
    if request.path.startswith("/static/"):
        response.headers["Cache-Control"] = "no-cache"
    return response


def build_app(studio: Studio) -> web.Application:
    app = web.Application(client_max_size=4 * 1024 * 1024, middlewares=[revalidate_static])

    async def call(fn: object, *args: object) -> object:
        return await asyncio.get_running_loop().run_in_executor(None, fn, *args)

    # Action name -> (studio method, body keys passed positionally)
    actions = {
        "project": (studio.select_project, ["project"]),
        "component/start": (studio.start_component, ["name"]),
        "component/stop": (studio.stop_component, ["name"]),
        "policies/refresh": (studio.refresh_policies, []),
        "policy/load": (studio.load_policy, ["id"]),
        "policy/unload": (studio.unload_policy, []),
        "policy/run": (studio.run_policy, ["prompt"]),
        "policy/stop": (studio.stop_policy, []),
        "home": (studio.home, []),
        "collection/select": (studio.select_dataset, ["dataset", "task", "instruction"]),
        "collection/start": (studio.start_recording, ["dataset", "task", "instruction", "operator"]),
        "collection/stop": (studio.stop_recording, []),
        "collection/discard": (studio.discard_last, []),
        "cluster/sync": (studio.sync_to_cluster, ["dataset", "task", "convert_config", "convert"]),
        "cluster/train": (studio.train_on_cluster, ["config", "dataset", "task"]),
        "cluster/refresh": (studio.refresh_jobs, []),
        "modality": (studio.set_modality, ["id", "on"]),
        "camera/save": (studio.save_camera_pose, ["name", "position", "wxyz"]),   # board_calibration.py
        "component/restart": (studio.restart_component, ["name"]),
        "halt": (studio.halt, []),
        "resume": (studio.resume, []),
        "calibration/watch": (studio.calibration_watch, ["on"]),
        "calibration/start": (studio.calibration_start, ["arms"]),
        "calibration/stop": (studio.halt, []),          # stopping a calibration is a Stop
        "calibration/save": (studio.calibration_save, ["name"]),
        "calibration/apply": (studio.calibration_apply, ["name"]),
        "objects/set": (studio.objects_set, ["objects"]),
        "objects/scan": (studio.objects_scan, []),
        "objects/stop": (studio.halt, []),               # stopping a scan is a Stop, as for calibration
    }

    async def action(request: web.Request) -> web.Response:
        name = request.match_info["action"]
        if name not in actions:
            raise web.HTTPNotFound()
        try:
            body = await request.json() if request.can_read_body else {}
        except ValueError:
            body = {}
        fn, keys = actions[name]
        args = [body.get(k) if body.get(k) is not None else "" for k in keys]
        if name == "cluster/sync":
            args[2] = body.get("convert_config") or None
            args[3] = bool(body.get("convert", True))
        try:
            result = await call(fn, *args)
        except Exception as e:
            log.exception("action %s failed", name)
            studio.notify(f"{name}: {e}", "error")
            result = {"ok": False, "message": str(e)}
        return _json(result)

    async def status(request: web.Request) -> web.Response:
        return _json(await call(studio.status, 0))

    async def logs(request: web.Request) -> web.Response:
        since = int(request.query.get("since", 0))
        return _json(studio.component_logs(request.match_info["name"], since))

    async def frame(request: web.Request) -> web.Response:
        """JPEG of one image feature of the newest observation (what the policy sees)."""
        key = request.match_info["key"]
        record = studio.latest_record()
        obs = getattr(record, "observation", None) or {}
        img = obs.get(key)
        if not isinstance(img, np.ndarray) or img.ndim != 3:
            raise web.HTTPNotFound()

        def encode() -> bytes:
            arr = img.transpose(1, 2, 0) if img.shape[0] in (1, 3, 4) else img
            arr = np.ascontiguousarray(arr[..., :3])
            if arr.dtype != np.uint8:
                arr = np.clip(arr * (255 if arr.max() <= 1.0 else 1), 0, 255).astype(np.uint8)
            if arr.shape[-1] == 3:
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            return cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 82])[1].tobytes()

        data = await call(encode)
        return web.Response(body=data, content_type="image/jpeg", headers={"Cache-Control": "no-store"})

    async def camera(request: web.Request) -> web.Response:
        """JPEG of a scene camera (every camera of the rig, not only the policy's)."""
        img = studio.camera_image(request.match_info["name"])
        if img is None:
            raise web.HTTPNotFound()

        def encode() -> bytes:
            return cv2.imencode(".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()

        return web.Response(body=await call(encode), content_type="image/jpeg", headers={"Cache-Control": "no-store"})

    async def websocket(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        follow = {"name": None, "seq": 0}
        last_event = 0

        async def reader() -> None:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                    except ValueError:
                        continue
                    if data.get("type") == "follow":
                        follow["name"], follow["seq"] = data.get("name"), int(data.get("since", 0))
                elif msg.type in (WSMsgType.ERROR, WSMsgType.CLOSE):
                    break

        task = asyncio.create_task(reader())
        try:
            while not ws.closed:
                snap = await call(studio.status, last_event)
                if snap["events"]:
                    last_event = snap["events"][-1]["id"]
                await ws.send_str(json.dumps({"type": "status", "data": snap}, default=str))
                if follow["name"]:
                    lines = studio.component_logs(follow["name"], follow["seq"])
                    if lines:
                        follow["seq"] = lines[-1][0]
                        await ws.send_str(json.dumps({"type": "logs", "name": follow["name"], "lines": lines}))
                await asyncio.sleep(STATUS_PERIOD_S)
        except (ConnectionResetError, RuntimeError):
            pass
        finally:
            task.cancel()
        return ws

    async def index(request: web.Request) -> web.Response:
        # Versioned asset URLs: a changed app.js / app.css is a new URL, so no browser keeps the old one.
        html = (STATIC_DIR / "index.html").read_text()
        for name in ("app.js", "app.css"):
            version = int((STATIC_DIR / name).stat().st_mtime)
            html = html.replace(f'"/static/{name}"', f'"/static/{name}?v={version}"')
        return web.Response(text=html, content_type="text/html", headers={"Cache-Control": "no-cache"})

    app.router.add_get("/", index)
    app.router.add_get("/api/status", status)
    app.router.add_get("/api/logs/{name}", logs)
    app.router.add_post("/api/{action:.+}", action)
    app.router.add_get("/frame/{key}", frame)
    app.router.add_get("/camera/{name}", camera)
    app.router.add_get("/ws", websocket)
    app.router.add_static("/static", STATIC_DIR, append_version=True)
    return app
