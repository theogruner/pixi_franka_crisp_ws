"""``python -m franka_studio [--config config/studio.yaml]`` (or ``pixi run studio``)."""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal

from aiohttp import web

from franka_studio.config import load_config


def main() -> None:
    ap = argparse.ArgumentParser(prog="franka_studio", description=__doc__)
    ap.add_argument("--config", help="studio config (default: config/studio.yaml)")
    ap.add_argument("--host", help="override server.host (0.0.0.0 = reachable from the network)")
    ap.add_argument("--port", type=int, help="override server.port")
    ap.add_argument("--no-viewer", action="store_true", help="do not start the 3D viewer")
    ap.add_argument("--keep-components", action="store_true",
                    help="leave the components the studio started running when it exits")
    ap.add_argument("--debug", action="store_true",
                    help="verbose logging, including one access-log line per HTTP request")
    args = ap.parse_args()

    # Quiet by default: the UI polls camera frames ~10x/s, so INFO + access log floods the terminal
    logging.basicConfig(level=logging.INFO if args.debug else logging.WARNING,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    if args.debug:
        logging.getLogger("franka_studio").setLevel(logging.DEBUG)
    cfg = load_config(args.config)
    host = args.host or cfg["server"]["host"]
    port = args.port or int(cfg["server"]["port"])

    from franka_studio.server.app import build_app
    from franka_studio.studio import Studio

    studio = Studio(cfg, with_viewer=not args.no_viewer)
    app = build_app(studio)

    async def on_shutdown(_app: web.Application) -> None:
        studio.shutdown(stop_components=not args.keep_components)

    async def on_startup(_app: web.Application) -> None:
        # a closed terminal (SIGHUP) shuts down like Ctrl-C / SIGTERM, stopping the components
        def hangup() -> None:
            raise web.GracefulExit()

        asyncio.get_running_loop().add_signal_handler(signal.SIGHUP, hangup)

    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    shown = "localhost" if host in ("0.0.0.0", "127.0.0.1") else host
    print(f"\n  Franka Studio  →  http://{shown}:{port}\n", flush=True)
    web.run_app(app, host=host, port=port, print=None, shutdown_timeout=5,
                access_log=logging.getLogger("aiohttp.access") if args.debug else None)


if __name__ == "__main__":
    main()
