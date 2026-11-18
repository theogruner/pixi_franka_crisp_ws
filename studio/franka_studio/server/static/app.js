// Franka Studio front end. All state comes from the server's `status` message; the page keeps
// only view state (selection, filters, followed log, layout) and the form inputs.
//
// Layout: every widget is a `.panel` living in one of three docks — the sidebar (stacked,
// collapsible), the inspector and the bottom area (tabbed). Panels are dragged between docks by
// their tab / sidebar title; the arrangement, sizes and visibility are remembered per browser.
// A new widget (e.g. an agent panel) is one more <section class="panel" data-dock=… data-title=…
// data-icon=…> plus its render function.

const $ = (sel) => document.querySelector(sel);
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const PAGE_T0 = Date.now() / 1000;
const icon = (id, cls = "i") => `<svg class="${cls}"><use href="#${id}"/></svg>`;

const ui = {
  status: null,
  ws: null,
  selectedPolicy: null,
  kindFilter: "",
  logName: null,
  logLines: [],
  formLoaded: null,        // project whose remembered values were put into the form
  camTimers: {},
  lastSig: {},
};

const store = {
  get(key, fallback = null) { try { const v = localStorage.getItem(key); return v == null ? fallback : JSON.parse(v); } catch (e) { return fallback; } },
  set(key, value) { try { localStorage.setItem(key, JSON.stringify(value)); } catch (e) {} },
  del(key) { try { localStorage.removeItem(key); } catch (e) {} },
};

// ---------------------------------------------------------------- helpers

const ICONS = { robot: "i-robot", camera: "i-camera", teleop: "i-gamepad", gamepad: "i-gamepad", record: "i-record", server: "i-server", brain: "i-brain", cloud: "i-points", points: "i-points", cube: "i-cube" };
const MODALITY_ICONS = { state: "i-robot", rgb: "i-camera", pointcloud: "i-points", depth: "i-layers", object_poses: "i-cube" };
const TOAST_ICONS = { success: "i-check", error: "i-alert", warn: "i-alert", info: "i-info" };

function fmtDuration(s) {
  if (s == null) return "";
  s = Math.max(0, Math.floor(s));
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), sec = s % 60;
  return h ? `${h}h ${String(m).padStart(2, "0")}m` : `${String(m).padStart(2, "0")}:${String(sec).padStart(2, "0")}`;
}

function changed(key, value) {
  const sig = JSON.stringify(value);
  if (ui.lastSig[key] === sig) return false;
  ui.lastSig[key] = sig;
  return true;
}

async function post(action, body = {}) {
  try {
    const r = await fetch(`/api/${action}`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
    const data = await r.json();
    if (data && data.message && !data.ok) toast(data.message, "error", true);
    return data;
  } catch (e) {
    toast(`Request failed: ${e}`, "error", true);
    return { ok: false };
  }
}

const shownToasts = new Set();
function toast(text, level = "info", local = false) {
  const key = `${level}:${text}`;
  if (local) { shownToasts.add(key); setTimeout(() => shownToasts.delete(key), 1500); }
  else if (shownToasts.has(key)) return;
  const el = document.createElement("div");
  el.className = `toast ${level}`;
  el.innerHTML = `${icon(TOAST_ICONS[level] || "i-info")}<div>${esc(text)}</div>`;
  el.onclick = () => el.remove();
  $("#toasts").appendChild(el);
  setTimeout(() => { el.classList.add("out"); setTimeout(() => el.remove(), 350); }, level === "error" ? 7000 : 4000);
  while ($("#toasts").children.length > 4) $("#toasts").firstChild.remove();
}

// Confirmation sheet (replaces window.confirm). `motion` adds the "the arm moves" callout.
function ask({ title, body = "", ok = "OK", tone = "", iconId = "i-alert", motion = "" }) {
  const dlg = $("#sheet");
  dlg.className = `sheet ${tone}`;
  $("#sheet-title").textContent = title;
  $("#sheet-icon").innerHTML = icon(iconId);
  $("#sheet-body").innerHTML = (body ? body.split("\n\n").map((p) => `<p>${esc(p)}</p>`).join("") : "")
    + (motion ? `<div class="callout">${icon("i-motion")}<span>${esc(motion)}</span></div>` : "");
  $("#sheet-ok").textContent = ok;
  return new Promise((resolve) => {
    dlg.returnValue = "";
    dlg.onclose = () => resolve(dlg.returnValue === "ok");
    dlg.showModal();
    $("#sheet-ok").focus();
  });
}

// ---------------------------------------------------------------- connection

function connect() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  ui.ws = ws;
  ws.onopen = () => { setConn(true); if (ui.logName) follow(ui.logName, true); };
  ws.onclose = () => { setConn(false); setTimeout(connect, 1500); };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "status") render(msg.data);
    else if (msg.type === "logs" && msg.name === ui.logName) appendLogs(msg.lines);
  };
}

function setConn(online) {
  const el = $("#conn");
  el.classList.toggle("online", online);
  el.title = online ? "Connected to the studio" : "Not connected to the studio — reconnecting";
  el.querySelector(".conn-text").textContent = online ? "live" : "Reconnecting…";
}

// ---------------------------------------------------------------- render

function render(s) {
  ui.status = s;
  renderTop(s);
  renderComponents(s);
  renderSpace(s);
  renderPolicies(s);
  renderPolicyActive(s);
  renderCollection(s);
  renderObservation(s);
  renderRigCameras(s);
  renderCluster(s);
  renderCalibration(s);
  renderObjects(s);
  renderViewer(s);
  renderBadges(s);
  syncDocks();
  for (const e of s.events || []) if (e.t > PAGE_T0 - 1) toast(e.text, e.level);
}

function renderTop(s) {
  if (changed("projects", [s.projects, s.project?.id])) {
    $("#project-select").innerHTML = (s.projects.length ? s.projects : ["(no projects)"])
      .map((p) => `<option ${p === s.project?.id ? "selected" : ""}>${esc(p)}</option>`).join("");
  }
  $("#env-name").textContent = s.env ? s.env.name : "no scene env";
  const rec = s.collection.recording;
  const p = s.policy;
  let cls = "", text = "Idle", target = "";
  if (s.halted) { cls = "halted"; text = "Stopped"; }
  else if (rec) { cls = "recording"; text = `Recording${s.collection.current ? " " + fmtDuration(s.collection.current.duration) : ""}${p ? " · DAgger" : ""}`; target = "collect-card"; }
  else if (s.calibration?.state === "running") { cls = "busy"; text = "Calibrating cameras"; target = "calib-card"; }
  else if (s.object_scan?.state === "running") { cls = "busy"; text = "Scanning objects"; target = "objects-card"; }
  else if (p?.state === "running") { cls = "running"; text = "Policy running"; target = "policy-card"; }
  else if (s.loading) { cls = "busy"; text = "Loading policy…"; target = "policy-card"; }
  else if (p) { cls = "ready"; text = "Policy ready"; target = "policy-card"; }
  const chip = $("#mode-chip");
  chip.className = `status ${cls}`;
  chip.dataset.target = target;
  $("#mode-text").textContent = text;
  $("#halt-banner").hidden = !s.halted;
  $("#home-top-btn").disabled = !!s.halted;
  $("#home-top-btn").title = s.halted ? "Resume first" : "Drive the arms to the home joints (home_pose of the env)";
  document.body.classList.toggle("is-halted", !!s.halted);
}

// Dots on the tabs / sidebar titles, so a panel's state is visible while another one is open.
function renderBadges(s) {
  const set = (id, b) => {
    const p = document.getElementById(id);
    if (!p) return;
    if (b) p.dataset.badge = b; else delete p.dataset.badge;
  };
  const failed = s.components.some((c) => c.state === "failed");
  set("collect-card", s.collection.recording ? "rec" : "");
  set("policy-card", s.policy?.state === "running" ? "run" : s.loading ? "busy" : s.policy ? "ready" : "");
  set("calib-card", s.calibration?.state === "running" ? "busy" : "");
  set("objects-card", s.object_scan?.state === "running" ? "busy" : "");
  set("console-card", failed ? "err" : "");
  set("stack-card", failed ? "err" : "");
  set("space-card", (s.modalities || []).some((m) => m.enabled && m.component && !m.component_up) ? "warn" : "");
  set("cluster-card", (s.jobs || []).some((j) => ["running", "PENDING", "RUNNING"].includes(j.slurm_state || j.status)) ? "busy" : "");
}

function renderComponents(s) {
  const comps = s.components;
  const box = $("#components");
  if (changed("comp-list", comps.map((c) => c.name))) {
    box.innerHTML = comps.map((c) => `
      <div class="row comp" data-name="${esc(c.name)}">
        <div class="row-icon">${icon(ICONS[c.icon] || "i-box")}</div>
        <div class="row-main">
          <div class="row-title"><span>${esc(c.label)}</span></div>
          <div class="row-sub"><span class="dot"></span><span class="txt"></span></div>
        </div>
        <div class="row-trail">
          <button class="icon-btn comp-log" title="Show its output in the console">${icon("i-terminal")}</button>
          <button class="btn btn-sm comp-toggle"></button>
        </div>
      </div>`).join("") || `<div class="empty">No components configured.</div>`;
    box.querySelectorAll(".comp").forEach((el) => {
      const name = el.dataset.name;
      el.querySelector(".comp-toggle").onclick = async (ev) => {
        const c = ui.status.components.find((x) => x.name === name);
        const btn = ev.currentTarget;
        btn.disabled = true;
        await post(c.owned ? "component/stop" : "component/start", { name });
        if (!c.owned) follow(name);
        setTimeout(() => (btn.disabled = false), 600);
      };
      el.querySelector(".comp-log").onclick = () => { follow(name); revealPanel("console-card"); };
    });
    renderTabs(s);
  }
  for (const c of comps) {
    const el = box.querySelector(`.comp[data-name="${CSS.escape(c.name)}"]`);
    if (!el) continue;
    const st = c.owned ? c.state : c.detected ? "running" : c.state;
    el.className = `row comp is-${st}`;
    let txt = { running: "Running", starting: "Starting…", stopping: "Stopping…", stopped: "Stopped", exited: "Exited", failed: `Failed (${c.exit_code})` }[st] || st;
    if (c.owned && c.uptime != null) txt += ` · ${fmtDuration(c.uptime)}`;
    if (!c.owned && c.detected) txt = "Running · external";
    if (!c.configured) txt = "Not configured";
    el.querySelector(".txt").textContent = txt;
    const btn = el.querySelector(".comp-toggle");
    btn.textContent = c.owned ? "Stop" : "Start";
    btn.className = `btn btn-sm comp-toggle ${c.owned ? "btn-secondary" : "btn-tinted"}`;
    btn.disabled = !c.configured || st === "stopping" || (!c.owned && c.detected);
    btn.title = !c.owned && c.detected ? "Started outside the studio" : c.cmd || "Set a command in config/studio.yaml";
  }
  renderTabStates(s);
}

function renderTabs(s) {
  $("#log-tabs").innerHTML = s.components.map((c) => `<button class="log-tab" role="tab" data-name="${esc(c.name)}"><span class="dot"></span>${esc(c.label)}</button>`).join("");
  $("#log-tabs").querySelectorAll(".log-tab").forEach((t) => (t.onclick = () => follow(t.dataset.name)));
  if (!ui.logName && s.components.length) follow(s.components[0].name);
  else highlightTab();
}

function renderTabStates(s) {
  for (const c of s.components) {
    const t = $(`#log-tabs .log-tab[data-name="${CSS.escape(c.name)}"]`);
    if (t) t.classList.toggle("running", (c.owned && c.state === "running") || (!c.owned && !!c.detected)), t.classList.toggle("failed", c.state === "failed");
  }
}

function highlightTab() {
  document.querySelectorAll("#log-tabs .log-tab").forEach((t) => {
    const on = t.dataset.name === ui.logName;
    t.classList.toggle("active", on);
    t.setAttribute("aria-selected", String(on));
  });
}

function follow(name, reconnect = false) {
  if (name !== ui.logName || reconnect) {
    if (name !== ui.logName) { ui.logName = name; ui.logLines = []; $("#console").innerHTML = ""; }
    if (ui.ws && ui.ws.readyState === 1) ui.ws.send(JSON.stringify({ type: "follow", name, since: ui.logLines.length ? ui.logLines[ui.logLines.length - 1][0] : 0 }));
  }
  highlightTab();
}

function lineClass(l) {
  if (l.startsWith("$ ")) return "l-cmd";
  if (l.startsWith("[studio]")) return "l-studio";
  if (/\b(ERROR|Error|error|FATAL|Traceback|failed)\b/.test(l)) return "l-err";
  if (/\b(WARN|WARNING|Warning)\b/.test(l)) return "l-warn";
  return "";
}

function appendLogs(lines) {
  const con = $("#console");
  const atBottom = con.scrollHeight - con.scrollTop - con.clientHeight < 40;
  const frag = document.createDocumentFragment();
  for (const [seq, text] of lines) {
    ui.logLines.push([seq, text]);
    const div = document.createElement("div");
    const cls = lineClass(text);
    if (cls) div.className = cls;
    div.textContent = text;
    frag.appendChild(div);
  }
  con.appendChild(frag);
  while (con.childElementCount > 2000) con.firstChild.remove();
  if (ui.logLines.length > 2000) ui.logLines.splice(0, ui.logLines.length - 2000);
  if ($("#log-follow").checked && atBottom) con.scrollTop = con.scrollHeight;
  else if ($("#log-follow").checked && lines.length && con.childElementCount === lines.length) con.scrollTop = con.scrollHeight;
}

// ---- observation space

function featureShape(s, key) {
  const f = (s.env?.features || []).find((x) => x.key === key);
  const sp = s.env?.observation_space?.spaces?.[key];
  const shape = f?.shape || sp?.shape;
  return shape && shape.length ? shape.join("×") : "";
}

function renderSpace(s) {
  const mods = s.modalities || [];
  $("#space-card").hidden = !mods.length;
  if (!mods.length) return;
  const box = $("#modalities");
  if (changed("mod-list", mods.map((m) => m.id))) {
    box.innerHTML = mods.map((m) => `
      <div class="row mod" data-id="${esc(m.id)}">
        <div class="row-icon">${icon(MODALITY_ICONS[m.id] || (m.component ? "i-points" : "i-eye"))}</div>
        <div class="row-main">
          <div class="row-title"><span>${esc(m.label)}</span><span class="tags"></span></div>
          <div class="row-sub block mod-desc"></div>
          <div class="row-mono mod-keys"></div>
        </div>
        <label class="switch" title=""><input type="checkbox" aria-label="${esc(m.label)}"><span></span></label>
      </div>`).join("");
    box.querySelectorAll(".mod").forEach((el) => {
      const input = el.querySelector("input");
      input.onchange = async () => {
        input.disabled = true;
        await post("modality", { id: el.dataset.id, on: input.checked });
        setTimeout(() => (input.disabled = false), 400);
      };
    });
  }
  const recording = s.collection.recording;
  for (const m of mods) {
    const el = box.querySelector(`.mod[data-id="${CSS.escape(m.id)}"]`);
    if (!el) continue;
    el.classList.toggle("on", m.enabled);
    const input = el.querySelector("input");
    if (document.activeElement !== input) input.checked = m.enabled;
    input.disabled = m.required || recording;
    el.querySelector(".switch").title = m.required ? "Always part of the observation" : recording ? "Stop the recording first" : m.enabled ? "Switch off" : "Switch on";
    const tags = [];
    if (m.required) tags.push(`<span class="tag lock" title="Always part of the observation">${icon("i-lock", "i inline")}</span>`);
    if (m.enabled && m.record.length) tags.push(`<span class="tag rec" title="${esc(m.record.join("\n"))}">REC</span>`);
    if (m.component && m.enabled && !m.component_up) tags.push(`<span class="tag err" title="${esc(m.component)}">not running</span>`);
    el.querySelector(".tags").innerHTML = tags.join("");
    el.querySelector(".mod-desc").textContent = m.description || (m.component ? `via ${m.component}` : "");
    const keys = m.enabled ? (s.env?.features || []).filter((f) => m.keys.some((k) => globMatch(k, f.key))) : [];
    el.querySelector(".mod-keys").textContent = m.enabled
      ? (keys.length ? keys.map((f) => `${f.key.replace("observation.", "")}${featureShape(s, f.key) ? " " + featureShape(s, f.key) : ""}`).join(" · ") : m.keys.join(" · "))
      : "off";
  }
  const on = mods.filter((m) => m.enabled).length;
  $("#space-sub").textContent = `${on} of ${mods.length} on`;
  renderPointCloud(s);
  renderPoseTracking(s);
  const extra = s.record_topics || [];
  $("#record-line").innerHTML = extra.length ? `Recorded with each episode: ${extra.map((t) => `<code>${esc(t)}</code>`).join(" ")}` : "Recorded: the collector's topics and RGB videos.";
}

function globMatch(pattern, key) {
  const re = new RegExp("^" + pattern.replace(/[.+^${}()|[\]\\]/g, "\\$&").replace(/\*/g, ".*").replace(/\?/g, ".") + "$");
  return re.test(key);
}

function renderPointCloud(s) {
  const pcMod = (s.modalities || []).find((m) => m.component === "pointcloud" && m.enabled);
  const pc = s.pointcloud;
  const el = $("#pc-status");
  el.hidden = !pcMod;
  if (!pcMod) return;
  el.className = `live-box ${pc?.live ? "live" : "stale"}`;
  if (!pc || !pc.live) {
    el.innerHTML = `<div class="live-head"><span class="dot"></span><span>${pcMod.component_up ? "Waiting for the point cloud node…" : `Starting ${esc(pcMod.component)}… (needs the cameras)`}</span></div>`;
    return;
  }
  const cams = Object.entries(pc.cameras || {});
  el.innerHTML = `<div class="live-head"><span class="dot"></span><span>Point cloud</span><b>${pc.hz.toFixed(1)} Hz</b><span><b>${(pc.points / 1000).toFixed(1)}k</b> pts</span><span class="muted">${esc(pc.engine)}</span></div>
    <div class="live-grid">${cams.map(([n, c]) => `<span>${esc(n)}</span><span>${c.ms ? `${c.ms.toFixed(0)} ms` : "—"}</span>${c.error ? `<span class="err">${esc(c.error)}</span>` : ""}`).join("")}</div>`;
}

function renderPoseTracking(s) {
  const mod = (s.modalities || []).find((m) => m.component === "pose_tracking" && m.enabled);
  const pt = s.pose_tracking;
  const el = $("#pt-status");
  el.hidden = !mod;
  if (!mod) return;
  const st = pt?.status;
  el.className = `live-box ${pt?.live ? "live" : "stale"}`;
  if (!pt || !pt.live) {
    const none = !(pt?.objects || []).length ? " — no <code>objects</code> in env.yaml" : "";
    el.innerHTML = `<div class="live-head"><span class="dot"></span><span>${mod.component_up ? "Waiting for the pose tracking node…" : `Starting ${esc(mod.component)}… (needs the point cloud)`}${none}</span></div>`;
    return;
  }
  const objs = (st.order || []).map((n) => [n, st.objects?.[n] || {}]);
  el.innerHTML = `<div class="live-head"><span class="dot"></span><span>Object poses</span><b>${st.hz.toFixed(1)} Hz</b><span>${esc(st.camera)}</span>${st.seg_ms != null ? `<span class="muted">seg ${st.seg_ms.toFixed(0)} ms</span>` : ""}</div>
    ${st.error ? `<div class="err">${esc(st.error)}</div>` : ""}
    <div class="live-grid">${objs.map(([n, o]) => `<span>${esc(n)}</span><span>${esc(o.status || "—")}${o.iou != null ? ` · IoU ${o.iou.toFixed(2)}` : ""}</span>${o.message && o.status !== "tracking" ? `<span class="err">${esc(o.message)}</span>` : ""}`).join("")}</div>`;
}

// ---- objects (pose tracking targets, live scan)

function objectRows() {
  return [...$("#obj-list").querySelectorAll(".obj-row")].map((r) => ({ name: r.dataset.name || "", prompt: r.querySelector("input").value.trim() }))
    .filter((o) => o.prompt);
}

function saveObjects(objects) {
  ui.lastSig["obj-list"] = null;   // redraw from what the studio stored
  return post("objects/set", { objects });
}

function objectState(name, s) {
  const scan = s.object_scan || {};
  const node = scan.node;
  if (scan.state === "running" && node) {
    if (node.state === "fusing" || (node.views && name in node.views)) return ["", `${node.views?.[name] ?? 0} views`];
    if (node.located && Object.keys(node.located).length) return name in node.located ? ["mesh", "located"] : ["lost", "not seen"];
    return ["", "…"];
  }
  const res = scan.results?.[name];
  const st = s.pose_tracking?.live ? s.pose_tracking.status?.objects?.[name] : null;
  if (st?.status) return [st.status === "tracking" ? "tracking" : st.status === "error" ? "error" : "lost", st.status];
  if (res?.error) return ["error", "scan failed"];
  const spec = (s.pose_tracking?.specs || []).find((o) => o.name === name);
  return spec?.mesh ? ["mesh", "mesh"] : ["", "no mesh"];
}

function renderObjects(s) {
  const specs = s.pose_tracking?.specs || [];
  const scan = s.object_scan || {};
  const running = scan.state === "running";
  const list = $("#obj-list");
  if (!list.contains(document.activeElement) && changed("obj-list", specs.map((o) => [o.name, o.prompt, o.mesh]))) {
    list.innerHTML = specs.length ? specs.map((o) => `<div class="row obj-row" data-name="${esc(o.name)}">
        <input value="${esc(o.prompt)}" title="${esc(o.name)}: prompt for the segmenter" spellcheck="false" aria-label="Object prompt">
        <span class="pill obj-state"></span>
        <button class="icon-btn" title="Scan again (new mesh)" data-rescan ${o.mesh ? "" : "hidden"}>${icon("i-refresh")}</button>
        <button class="icon-btn" title="Remove" data-remove>${icon("i-trash")}</button></div>`).join("")
      : "";
  }
  for (const row of list.querySelectorAll(".obj-row")) {
    const [cls, text] = objectState(row.dataset.name, s);
    const chip = row.querySelector(".obj-state");
    chip.className = `pill obj-state ${cls}`;
    chip.textContent = text;
    row.querySelector("input").disabled = running;
    row.querySelector("[data-remove]").disabled = running;
    row.querySelector("[data-rescan]").disabled = running || s.halted;
  }
  const missing = specs.filter((o) => !o.mesh).length;
  $("#objects-sub").textContent = specs.length ? `${specs.length} objects${missing ? ` · ${missing} without mesh` : ""}` : "";
  const manual = running && scan.manual;
  const capturing = manual && String(scan.phase || "").startsWith("capturing");
  $("#obj-scan").hidden = running;
  $("#obj-auto-label").hidden = running;
  $("#obj-capture").hidden = !manual;
  $("#obj-finish").hidden = !manual;
  $("#obj-capture").disabled = capturing || !String(scan.phase || "").includes("Capture view");
  $("#obj-finish").disabled = capturing || !scan.captures;
  $("#obj-capture-text").textContent = capturing ? "Capturing…" : `Capture view ${(scan.captures || 0) + 1}`;
  $("#obj-stop").hidden = !running;
  $("#obj-scan").disabled = !specs.length || s.halted || s.calibration?.state === "running";
  $("#obj-scan").title = s.halted ? "Resume first" : !specs.length ? "Add objects first"
    : missing ? "Locate the objects without a mesh, drive the wrist cameras around them, build their meshes and start tracking (objects with a mesh are not scanned again)"
    : "Every object has a mesh: start the pose tracking, which locates them (no arm motion)";
  $("#obj-scan-text").textContent = missing ? `Scan ${missing} & estimate poses` : "Estimate poses";
  $("#obj-auto-label").hidden = running || !missing;     // automatic only matters when something gets scanned
  $("#obj-new").disabled = running;
  const prog = $("#obj-progress");
  const lines = [...(scan.progress || [])];
  if (running && scan.node?.message) lines.push(`node: ${scan.node.message}`);
  prog.hidden = !lines.length;
  if (!prog.hidden && changed("obj-prog", [scan.state, scan.phase, lines])) {
    prog.innerHTML = `<div class="phase ${esc(scan.state)}">${esc(running ? scan.phase : scan.state)}</div>${esc(lines.join("\n"))}`;
    prog.scrollTop = prog.scrollHeight;
  }
}

// ---- policies

function policyMatches(p) {
  const words = $("#policy-filter").value.toLowerCase().split(/\s+/).filter(Boolean);
  const hay = `${p.name} ${p.file} ${p.policy_type} ${p.description || ""}`.toLowerCase();
  return (!ui.kindFilter || p.kind === ui.kindFilter) && words.every((w) => hay.includes(w));
}

function renderPolicies(s) {
  const list = s.policies.filter(policyMatches);
  const loadedId = s.policy?.id;
  if (!ui.selectedPolicy || !s.policies.some((p) => p.id === ui.selectedPolicy)) {
    ui.selectedPolicy = s.remembered?.policy && s.policies.some((p) => p.id === s.remembered.policy) ? s.remembered.policy : s.policies[0]?.id || null;
  }
  $("#policy-sub").textContent = s.policies.length ? `${s.policies.length} in project` : "";
  if (changed("policies", [list, loadedId, ui.selectedPolicy, s.loading])) {
    $("#policy-list").innerHTML = list.length ? list.map((p) => {
      const kind = p.kind === "vision_language" ? `<span class="tag vl">Vision + Language</span>` : `<span class="tag v">Vision</span>`;
      const step = p.step ? `step ${esc(p.step.replace(/^0+/, "") || p.step)}` : "";
      const trained = p.trained ? esc(p.trained.replace("T", " ")) : "";
      return `<button class="row pol ${p.id === ui.selectedPolicy ? "selected" : ""} ${p.id === loadedId ? "loaded" : ""}" data-id="${p.id}" title="${esc(p.file)}${p.description ? "\n" + esc(p.description) : ""}">
        <span class="row-main">
          <span class="row-title"><span>${esc(p.name)}</span>${p.id === loadedId ? `<span class="loaded-dot" title="Loaded"></span>` : ""}${s.loading === p.id ? `<span class="muted">loading…</span>` : ""}</span>
          <span class="row-sub">${kind}${p.policy_type ? `<span class="tag">${esc(p.policy_type)}</span>` : ""}${p.error ? `<span class="tag err" title="${esc(p.error)}">problem</span>` : ""}<span>${[step, trained].filter(Boolean).join(" · ")}</span></span>
          ${p.description ? `<span class="row-sub block">${esc(p.description)}</span>` : ""}
        </span>
        ${icon("i-check", "i check")}
      </button>`;
    }).join("") : `<div class="empty">${s.policies.length ? "No policy matches the filter." : "No policies yet — add YAML files to the project's <code>policies/</code> folder and press refresh."}</div>`;
    $("#policy-list").querySelectorAll(".pol").forEach((b) => {
      b.onclick = () => { ui.selectedPolicy = b.dataset.id; ui.lastSig.policies = null; renderPolicies(ui.status); };
      b.ondblclick = () => loadSelected();
    });
  }
  const sel = s.policies.find((p) => p.id === ui.selectedPolicy);
  const btn = $("#load-btn");
  const busy = !!s.loading || s.policy?.state === "running" || s.collection.recording;
  btn.disabled = !sel || busy || !!sel.error || sel.id === loadedId;
  btn.innerHTML = `${icon("i-download")}${s.loading ? "Loading…" : sel && sel.id === loadedId ? "Loaded" : loadedId ? "Switch to this policy" : "Load policy"}`;
  btn.title = sel?.error || (s.policy?.state === "running" ? "Stop the policy first" : s.collection.recording ? "Stop the recording first" : "");
}

function loadSelected() {
  if (ui.selectedPolicy) post("policy/load", { id: ui.selectedPolicy });
}

function renderPolicyActive(s) {
  const p = s.policy;
  $("#policy-active").hidden = !p;
  if (!p) return;
  const spec = s.policies.find((x) => x.id === p.id) || {};
  const running = p.state === "running";
  $("#policy-active").classList.toggle("running", running);
  $("#pa-name").textContent = p.name;
  $("#pa-meta").textContent = [spec.policy_type, spec.server_address, p.frequency ? `${p.frequency} Hz` : ""].filter(Boolean).join(" · ");
  const pill = $("#pa-state");
  pill.textContent = p.state;
  pill.className = `pill ${p.state}`;
  const frac = Math.min(1, p.queued / Math.max(1, p.actions_per_chunk));
  $("#queue-fill").style.width = `${frac * 100}%`;
  $("#queue-threshold").style.left = `${Math.min(100, (p.threshold / Math.max(1, p.actions_per_chunk)) * 100)}%`;
  const stat = (label, value) => `<div class="stat"><span>${label}</span><b>${value}</b></div>`;
  $("#pa-stats").innerHTML = stat("Queue", `${p.queued}<small>/${p.actions_per_chunk}</small>`)
    + stat("Control", `${p.control_hz ?? "—"}<small> Hz</small>`)
    + stat("Chunks", `${p.inference_hz ?? "—"}<small>/s</small>`)
    + stat(p.run_time != null ? `Steps · ${fmtDuration(p.run_time)}` : "Steps", `${p.steps}`);
  const lang = p.kind === "vision_language";
  $("#prompt-box").hidden = !lang;
  if (lang && changed("prompt-chips", [p.id, spec.prompts])) {
    const prompt = $("#prompt");
    if (!prompt.value) prompt.value = s.remembered?.prompt && (spec.prompts || []).length === 0 ? s.remembered.prompt : (spec.prompts || [])[0] || s.remembered?.prompt || "";
    $("#prompt-chips").innerHTML = (spec.prompts || []).map((t) => `<button title="${esc(t)}">${esc(t)}</button>`).join("");
    $("#prompt-chips").querySelectorAll("button").forEach((b) => (b.onclick = () => ($("#prompt").value = b.title)));
  }
  // one primary action at a time: Run while ready, Stop policy while running
  $("#run-btn").hidden = running;
  $("#stop-btn").hidden = !running;
  $("#run-btn").disabled = p.state !== "ready" || s.collection.recording;
  $("#run-btn").title = s.collection.recording ? "Stop the recording first" : s.halted ? "Resumes the arms and runs" : "Run the policy (Ctrl+Enter in the prompt)";
  $("#unload-btn").disabled = running;
  if (p.error && changed("policy-error", p.error)) toast(p.error, "error");
}

// ---- collection

function fillForm(s) {
  if (!s.project || ui.formLoaded === s.project.id) return;
  ui.formLoaded = s.project.id;
  const r = s.remembered || {};
  $("#dataset").value = r.dataset || "";
  $("#task").value = r.task || "";
  $("#instruction").value = r.instruction || (r.task ? s.tasks[r.task] || "" : "");
  $("#operator").value = r.operator || "";
}

function renderCollection(s) {
  fillForm(s);
  const c = s.collection;
  if (changed("datasets", s.datasets)) {
    $("#dataset-list").innerHTML = Object.keys(s.datasets).map((d) => `<option value="${esc(d)}">`).join("");
  }
  const ds = $("#dataset").value.trim();
  const dsTasks = s.datasets[ds] || {};
  const taskNames = [...new Set([...Object.keys(dsTasks), ...Object.keys(s.tasks || {})])];
  if (changed("tasks", [ds, taskNames])) {
    $("#task-list").innerHTML = taskNames.map((t) => `<option value="${esc(t)}">${esc(s.tasks?.[t] || "")}</option>`).join("");
  }
  const task = $("#task").value.trim();
  const n = dsTasks[task];
  $("#ds-count").textContent = ds && task ? (n != null ? `${n} episode${n === 1 ? "" : "s"} in ${ds}/${task}` : `New task — ${ds}/${task} will be created`) : "Choose a dataset and task.";
  const badge = $("#collect-kind");
  badge.textContent = s.policy ? "DAgger" : "Demo";
  badge.className = `badge ${s.policy ? "dagger" : ""}`;
  badge.title = s.policy ? `Episodes are tagged with ${s.policy.name}` : "Teleoperated demonstrations";

  const rec = $("#rec-btn");
  const collectorUp = s.components.some((x) => x.name === s.data_collector && (x.owned && x.state !== "stopping" || x.detected));
  const policyRunning = s.policy?.state === "running";
  rec.classList.toggle("on", c.recording);
  rec.disabled = !c.recording && (policyRunning || !collectorUp || !ds || !task);
  rec.title = policyRunning ? "Stop the policy first" : !collectorUp ? "Start the data collector" : !ds || !task ? "Choose dataset and task" : c.recording ? "Stop recording" : "Start recording";
  rec.setAttribute("aria-pressed", String(c.recording));
  $("#rec-time").textContent = c.recording && c.current ? fmtDuration(c.current.duration) : "00:00";
  $("#rec-sub").textContent = c.recording && c.current ? `→ ${c.current.dataset}/${c.current.task}` : policyRunning ? "Policy running — stop it to record" : !collectorUp ? "Data collector not running" : !ds || !task ? "Choose a dataset and task" : c.message || "Ready";
  $("#discard-btn").disabled = !c.can_discard || c.recording;
  for (const id of ["#dataset", "#task"]) $(id).disabled = c.recording;

  if (changed("episodes", c.history)) {
    $("#episodes").innerHTML = c.history.length ? c.history.map((e) => `<div class="row ep" title="${esc(e.path)}${e.error ? "\n" + esc(e.error) : ""}">
      <span class="row-main">${esc(e.name)}</span><span class="ep-time">${fmtDuration(e.duration)}</span>
      ${e.kind === "dagger" ? `<span class="tag vl">DAgger</span>` : ""}
      <span class="pill ${esc(e.status)}">${esc(e.status)}</span></div>`).join("")
      : `<div class="empty">No episodes recorded yet.</div>`;
  }
  $("#sync-target").textContent = ds && task ? `${ds}/${task}` : "—";
}

// ---- observation

function renderObservation(s) {
  const feats = s.env?.features || [];
  const imgs = feats.filter((f) => f.image);
  const cams = $("#cams");
  if (changed("cams", [s.env?.name, imgs.map((f) => f.key)])) {
    cams.innerHTML = imgs.length ? imgs.map((f) => `<div class="cam stale" data-key="${esc(f.key)}"><img alt=""><div class="cam-label"><span>${esc(f.key.replace("observation.images.", ""))}</span><span class="shape"></span></div></div>`).join("") : `<div class="empty">No image features in the active env.</div>`;
    cams.querySelectorAll(".cam").forEach((el) => pollCam(el, (k) => `/frame/${encodeURIComponent(k)}`));
  }
  for (const f of imgs) {
    const el = cams.querySelector(`.cam[data-key="${CSS.escape(f.key)}"]`);
    if (!el) continue;
    el.classList.toggle("stale", !f.ok);
    el.querySelector(".shape").textContent = f.shape ? f.shape.join("×") : "";
  }
  const vecs = feats.filter((f) => !f.image);
  if (changed("features", vecs.map((f) => [f.key, f.ok, f.shape, f.age != null ? Math.round(f.age * 100) : null]))) {
    $("#features").innerHTML = vecs.map((f) => `<span class="feat ${f.ok ? "ok" : "bad"}" title="${esc(f.topics.join("\n"))}"><span class="dot"></span><b>${esc(f.key.replace("observation.", ""))}</b>${f.shape ? ` [${f.shape.join("×")}]` : ""}${f.age != null ? ` · ${(f.age * 1000).toFixed(0)} ms` : " · waiting"}</span>`).join("");
  }
  const missing = feats.filter((f) => !f.ok).length;
  $("#obs-missing").textContent = feats.length ? (missing ? `· ${missing} of ${feats.length} not live` : `· ${feats.length} live`) : "";
  const p = s.policy;
  $("#obs-rate").textContent = p?.state === "running" && p.control_hz ? `${p.control_hz} Hz · policy` : s.env ? "Monitor" : "— Hz";
}

function renderRigCameras(s) {
  const cams = s.env?.cameras || [];
  $("#rig-card").hidden = !cams.length;
  const box = $("#rig-cams");
  if (changed("rig-cams", [s.env?.name, cams.map((c) => c.name)])) {
    box.innerHTML = cams.map((c) => `<div class="cam stale" data-key="${esc(c.name)}"><img alt=""><div class="cam-label"><span>${esc(c.name)}</span><span class="rate"></span></div></div>`).join("");
    box.querySelectorAll(".cam").forEach((el) => pollCam(el, (k) => `/camera/${encodeURIComponent(k)}`));
  }
  for (const c of cams) {
    const el = box.querySelector(`.cam[data-key="${CSS.escape(c.name)}"]`);
    if (!el) continue;
    el.classList.toggle("stale", !c.ok);
    el.querySelector(".rate").textContent = c.ok ? `${(c.age * 1000).toFixed(0)} ms` : "";
  }
  const live = cams.filter((c) => c.ok).length;
  $("#rig-status").textContent = cams.length ? `· ${live} of ${cams.length} live` : "";
}

function pollCam(el, urlFor) {
  const img = el.querySelector("img");
  const key = el.dataset.key;
  const next = () => {
    if (!el.isConnected) return;
    // panels that are not shown (inactive tab, hidden dock) poll slowly
    const shown = el.offsetParent !== null;
    ui.camTimers[urlFor(key)] = setTimeout(load, el.classList.contains("stale") || !shown ? 1000 : 110);
  };
  const load = () => {
    if (!el.isConnected) return;
    const probe = new Image();
    probe.onload = () => { img.src = probe.src; next(); };
    probe.onerror = next;
    probe.src = `${urlFor(key)}?t=${Date.now()}`;
  };
  load();
}

// ---- cluster

function renderCluster(s) {
  $("#cluster-host").textContent = s.cluster?.host ? `ssh ${s.cluster.host}` : "not configured";
  if (changed("convert-configs", s.conversion_configs)) {
    $("#convert-config").innerHTML = (s.conversion_configs.length ? [""].concat(s.conversion_configs) : [""]).map((c) => `<option value="${esc(c)}">${c ? esc(c) : "None (command default)"}</option>`).join("");
  }
  if (changed("train-configs", s.train_configs)) {
    $("#train-config").innerHTML = s.train_configs.length ? s.train_configs.map((c) => `<option>${esc(c)}</option>`).join("") : `<option value="">Add configs to train_configs/</option>`;
  }
  const ds = $("#dataset").value.trim(), task = $("#task").value.trim();
  const ready = s.cluster?.configured && ds && task && !s.collection.recording;
  $("#sync-only").disabled = !ready;
  $("#sync-convert").disabled = !ready;
  const why = !s.cluster?.configured ? "Set cluster.host in project.yaml" : !ds || !task ? "Choose dataset and task in Record" : s.collection.recording ? "Stop the recording first" : "";
  $("#sync-only").title = why || "rsync the dataset/task to the cluster";
  $("#sync-convert").title = why || "rsync, then run the project's conversion command";
  $("#train-btn").disabled = !s.cluster?.configured || !s.train_configs.length;
  if (changed("jobs", s.jobs)) {
    const open = new Set([...document.querySelectorAll(".job[open]")].map((d) => d.dataset.id));
    $("#jobs").innerHTML = s.jobs.length ? s.jobs.map((j) => {
      const state = j.slurm_state || j.status;
      const when = new Date(j.created * 1000).toLocaleString([], { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" });
      return `<details class="job" data-id="${esc(j.id)}" ${open.has(j.id) ? "open" : ""}>
        <summary>${icon("i-chevron-right")}<span class="job-title" title="${esc(j.title)}">${esc(j.title)}</span><span class="job-when">${when}</span>
        <span class="pill ${state === "running" ? "job-running" : esc(state)}">${esc(state)}${j.slurm_id ? ` #${esc(j.slurm_id)}` : ""}</span></summary>
        <pre>${esc(j.lines.join("\n"))}</pre></details>`;
    }).join("") : `<div class="empty">No cluster jobs yet.</div>`;
  }
}

// ---- camera calibration

function renderCalibration(s) {
  const c = s.calibration;
  if (!c) return;
  const watch = $("#calib-watch");
  if (document.activeElement !== watch) watch.checked = c.watching;
  const vis = c.visibility || {};
  const visHtml = Object.entries(vis).map(([cam, n]) => {
    const cls = n == null ? "" : n >= 4 ? "ok" : "warn";
    const txt = n == null ? (c.watching || c.state === "running" ? "…" : "off") : `${n} markers`;
    return `<span class="chip ${cls}" title="${esc(cam)}"><span class="dot"></span>${esc(cam.replace("zed_", ""))} · ${txt}</span>`;
  }).join("");
  if (changed("calib-vis", visHtml)) $("#calib-vis").innerHTML = visHtml || `<span class="muted">No cameras.</span>`;
  const arms = Object.keys(c.arms || {});
  if (changed("calib-arms", arms)) {
    $("#calib-arms").innerHTML = arms.map((a) => `<label class="chip"><input type="checkbox" value="${esc(a)}" checked>${icon("i-check")}${esc(a.replace("zed_", ""))}</label>`).join("")
      || `<span class="muted">No wrist cameras.</span>`;
  }
  const runningCal = c.state === "running";
  $("#calib-start").disabled = runningCal || s.halted;
  $("#calib-start").title = s.halted ? "Resume first" : runningCal ? "Running — Stop aborts it" : "Probe and drive to the views with the checked arms, one after the other";
  $("#calib-add").disabled = !c.can_add || runningCal || s.halted;
  $("#calib-add").title = c.can_add ? "Board moved: capture it at its new place with the checked arms and solve every placement together" : "Calibrate first";
  const prog = $("#calib-progress");
  prog.hidden = !(c.progress || []).length;
  if (!prog.hidden && changed("calib-prog", [c.state, c.phase, c.progress])) {
    prog.innerHTML = `<div class="phase ${esc(c.state)}">${esc(c.state === "running" ? c.phase : c.state)}</div>${esc((c.progress || []).join("\n"))}`;
    prog.scrollTop = prog.scrollHeight;
  }
  const res = $("#calib-result");
  res.hidden = !c.result;
  $("#calib-save-row").hidden = !c.result || (c.result.weak || []).length > 0;
  if (c.result && changed("calib-res", c.result)) {
    const r = c.result;
    const rows = Object.entries(r.rms).map(([cam, v]) => {
      const before = r.rms_before?.[cam];
      return `<span>${esc(cam)}</span><span class="${v < 2 ? "good" : "bad"}">${v.toFixed(2)} px${before != null ? ` <span class="muted">(was ${before.toFixed(1)})</span>` : ""}</span>`;
    }).join("");
    const robots = Object.entries(r.values?.robots || {}).map(([robot, vals]) =>
      `<span>${esc(robot)}</span><span class="muted">${esc(Object.keys(vals).some((k) => k.startsWith("cam_")) ? "camera mount" : "")}${vals.xyz ? (Object.keys(vals).length > 2 ? " + base" : "base") : ""}</span>`).join("");
    const statics = Object.keys(r.values?.static || {}).map((cam) => `<span>${esc(cam)}</span><span class="muted">pose</span>`).join("");
    const cl = r.cloud;
    const clouds = !cl ? "" : cl.used
      ? Object.entries(cl.mm).map(([cam, [a, b]]) => `<span>${esc(cam)}</span><span class="${b != null && b < 3 ? "good" : "bad"}">${b} mm <span class="muted">(was ${a})</span></span>`).join("")
      : `<span class="muted wide">${esc(cl.message)}</span>`;
    const weak = (r.weak || []).length
      ? `<span class="head bad">Weak — not to be saved: add a placement or calibrate again</span>${r.weak.map((w) => `<span class="bad wide">${esc(w)}</span>`).join("")}`
      : "";
    res.innerHTML = `${weak}<span class="head">Reprojection error · ${r.placements || 1} board placement${(r.placements || 1) > 1 ? "s" : ""}</span>${rows}`
      + `<span class="head">Distance to the static camera's cloud</span>${clouds}`
      + `<span class="head">Changes when applied</span>${robots}${statics}`;
  }
  const saved = c.saved || [];
  if (changed("calib-saved", saved.map((x) => x.name))) {
    $("#calib-saved").innerHTML = saved.length
      ? saved.map((x) => `<option value="${esc(x.name)}">${esc(x.name)}${x.created ? ` · ${esc(x.created.replace("T", " ").slice(0, 16))}` : ""}</option>`).join("")
      : `<option value="">None yet</option>`;
  }
  $("#calib-apply").disabled = !saved.length || runningCal;
}

// ---- viewer

function viewerUrl(s) {
  return s.viewer.public_url || `${location.protocol}//${location.hostname}:${s.viewer.port}`;
}

function renderViewer(s) {
  if (!changed("viewer", s.viewer)) return;
  const frame = $("#viewer-frame");
  if (s.viewer.enabled) {
    const url = viewerUrl(s);
    frame.src = url;
    $("#viewer-open").href = url;
    $("#viewer").classList.add("live");
  } else {
    frame.removeAttribute("src");
    $("#viewer").classList.remove("live");
  }
}

// ---------------------------------------------------------------- docks & layout

const LAYOUT_KEY = "studio-docks-v1";
const VIEW_KEY = "studio-view-v1";
const SIZES_KEY = "studio-sizes-v1";
const MEDIUM = matchMedia("(max-width: 1279px)");
const NARROW = matchMedia("(max-width: 860px)");
const layout = { active: {}, collapsed: [] };
const view = { sidebar: true, inspector: true, bottom: true, sidebarOverlay: false };

const docks = () => [...document.querySelectorAll(".dock[data-dock]")];
const panelBox = (dock) => dock.querySelector(".dock-panels");
const panelsIn = (dock) => [...panelBox(dock).children].filter((p) => p.classList.contains("panel"));
const isTabbed = (dock) => dock.classList.contains("dock-tabs");
const dockOf = (panel) => panel.closest(".dock");

function saveLayout() {
  store.set(LAYOUT_KEY, {
    docks: Object.fromEntries(docks().map((d) => [d.dataset.dock, panelsIn(d).map((p) => p.id)])),
    active: layout.active,
    collapsed: layout.collapsed,
  });
}

function buildTabs(dock) {
  const bar = dock.querySelector(".tabbar");
  bar.innerHTML = panelsIn(dock).map((p) => `<button class="tab" role="tab" draggable="true" data-panel="${p.id}" title="${esc(p.dataset.title)}">
      ${icon(p.dataset.icon || "i-box")}<span>${esc(p.dataset.tab || p.dataset.title)}</span><span class="badge-dot"></span></button>`).join("");
  bar.querySelectorAll(".tab").forEach((t) => (t.onclick = () => activate(dock, t.dataset.panel)));
}

function activate(dock, id) {
  layout.active[dock.dataset.dock] = id;
  saveLayout();
  syncDock(dock);
  // tiles of a panel that just became visible should refresh at once
  document.getElementById(id)?.querySelectorAll(".console").forEach((c) => { if ($("#log-follow").checked) c.scrollTop = c.scrollHeight; });
}

function syncDock(dock) {
  const panels = panelsIn(dock);
  const visible = panels.filter((p) => !p.hidden);
  dock.querySelector(".dock-empty").hidden = visible.length > 0;
  if (isTabbed(dock)) {
    let active = layout.active[dock.dataset.dock];
    if (!visible.some((p) => p.id === active)) active = visible[0]?.id;
    for (const p of panels) p.classList.toggle("is-active", p.id === active);
    for (const t of dock.querySelectorAll(".tab")) {
      const p = document.getElementById(t.dataset.panel);
      t.hidden = !p || p.hidden;
      t.setAttribute("aria-selected", String(t.dataset.panel === active));
      const b = p?.dataset.badge || "";
      if ((t.dataset.badge || "") !== b) { if (b) t.dataset.badge = b; else delete t.dataset.badge; }
    }
  }
}

function syncDocks() { docks().forEach(syncDock); }

function setCollapsed(panel, collapsed) {
  panel.classList.toggle("collapsed", collapsed);
  layout.collapsed = layout.collapsed.filter((id) => id !== panel.id).concat(collapsed ? [panel.id] : []);
  saveLayout();
}

// Bring a panel into view: open its dock, select its tab / expand it.
function revealPanel(id) {
  const p = document.getElementById(id);
  if (!p || p.hidden) return;
  const dock = dockOf(p);
  if (dock.id === "dock-sidebar") setView("sidebar", true);
  if (dock.id === "dock-inspector") setView("inspector", true);
  if (dock.id === "dock-bottom") setView("bottom", true);
  if (isTabbed(dock)) activate(dock, id);
  else if (p.classList.contains("collapsed")) setCollapsed(p, false);
  requestAnimationFrame(() => p.scrollIntoView({ block: "nearest", behavior: "smooth" }));
}

function applyView() {
  const ws = $("#workspace");
  const medium = MEDIUM.matches;
  ws.classList.toggle("no-sidebar", !medium && !view.sidebar);
  ws.classList.toggle("sidebar-open", medium && view.sidebarOverlay);
  ws.classList.toggle("no-inspector", !view.inspector);
  $(".stage").classList.toggle("no-bottom", !view.bottom);
  $("#toggle-sidebar").classList.toggle("on", medium ? view.sidebarOverlay : view.sidebar);
  $("#toggle-inspector").classList.toggle("on", view.inspector);
  $("#toggle-bottom").classList.toggle("on", view.bottom);
  for (const [k, id] of [["sidebar", "#toggle-sidebar"], ["inspector", "#toggle-inspector"], ["bottom", "#toggle-bottom"]]) {
    $(id).setAttribute("aria-pressed", String(k === "sidebar" && medium ? view.sidebarOverlay : view[k]));
  }
}

function setView(key, on) {
  if (key === "sidebar" && MEDIUM.matches) view.sidebarOverlay = on;   // floats over the scene, not remembered
  else view[key] = on;
  store.set(VIEW_KEY, { sidebar: view.sidebar, inspector: view.inspector, bottom: view.bottom });
  applyView();
}

function wireLayout() {
  // restore: panels into their remembered docks (unknown / new panels stay in their default dock)
  const saved = store.get(LAYOUT_KEY);
  for (const [name, ids] of Object.entries(saved?.docks || {})) {
    const dock = document.querySelector(`.dock[data-dock="${CSS.escape(name)}"]`);
    if (dock) for (const id of ids) {
      const p = document.getElementById(id);
      if (p?.classList.contains("panel")) panelBox(dock).appendChild(p);
    }
  }
  layout.active = saved?.active || {};
  layout.collapsed = saved?.collapsed || [];
  Object.assign(view, store.get(VIEW_KEY, {}));

  for (const dock of docks()) {
    const empty = document.createElement("div");
    empty.className = "dock-empty";
    empty.textContent = "Drag a tab or a sidebar title here.";
    panelBox(dock).prepend(empty);
  }
  for (const p of document.querySelectorAll(".panel")) {
    const title = p.querySelector(".panel-title");
    if (!title) continue;
    title.insertAdjacentHTML("afterbegin", icon("i-chevron", "i disc"));
    title.classList.add("drag-handle");
    title.title = "Click to collapse · drag to move";
    if (layout.collapsed.includes(p.id)) p.classList.add("collapsed");
    // only the title starts a drag, so inputs, sliders and text selection inside keep working
    title.addEventListener("pointerdown", () => { if (!isTabbed(dockOf(p))) p.draggable = true; });
    title.addEventListener("click", () => { if (!isTabbed(dockOf(p))) setCollapsed(p, !p.classList.contains("collapsed")); });
  }
  document.addEventListener("pointerup", () => document.querySelectorAll(".panel[draggable=true]").forEach((p) => { if (!drag) p.draggable = false; }));
  docks().filter(isTabbed).forEach(buildTabs);
  syncDocks();
  wireDrag();
  wireSplitters();

  $("#toggle-sidebar").onclick = () => setView("sidebar", MEDIUM.matches ? !view.sidebarOverlay : !view.sidebar);
  $("#toggle-inspector").onclick = () => setView("inspector", !view.inspector);
  $("#toggle-bottom").onclick = () => setView("bottom", !view.bottom);
  MEDIUM.addEventListener("change", () => { view.sidebarOverlay = false; applyView(); });
  // the floating sidebar closes when you click elsewhere
  document.addEventListener("pointerdown", (e) => {
    if (MEDIUM.matches && !NARROW.matches && view.sidebarOverlay && !e.target.closest("#dock-sidebar, #toggle-sidebar, .sheet")) setView("sidebar", false);
  });
  applyView();

  $("#layout-reset").onclick = () => {
    for (const k of [LAYOUT_KEY, VIEW_KEY, SIZES_KEY, "studio-layout"]) store.del(k);
    location.reload();
  };
}

// Drag a tab (tabbed docks) or a sidebar title into any dock.
let drag = null;
function wireDrag() {
  const marker = document.createElement("div");
  const clearTargets = () => docks().forEach((d) => d.classList.remove("drop-target"));
  const end = () => {
    if (!drag) return;
    drag.source.classList.remove("dragging");
    drag.panel.draggable = false;
    document.body.classList.remove("dragging-panel");
    marker.remove();
    clearTargets();
    drag = null;
  };

  document.addEventListener("dragstart", (e) => {
    const tab = e.target.closest?.(".tab");
    const panel = tab ? document.getElementById(tab.dataset.panel) : e.target.classList?.contains("panel") ? e.target : null;
    if (!panel) return;
    drag = { panel, source: tab || panel };
    e.dataTransfer.effectAllowed = "move";
    e.dataTransfer.setData("text/plain", panel.id);
    requestAnimationFrame(() => { drag?.source.classList.add("dragging"); document.body.classList.add("dragging-panel"); });
  });
  document.addEventListener("dragend", end);

  for (const dock of docks()) {
    dock.addEventListener("dragover", (e) => {
      if (!drag) return;
      e.preventDefault();
      e.dataTransfer.dropEffect = "move";
      if (!dock.classList.contains("drop-target")) { clearTargets(); dock.classList.add("drop-target"); }
      if (isTabbed(dock)) {
        marker.className = "tab-drop";
        const bar = dock.querySelector(".tabbar");
        const tabs = [...bar.querySelectorAll(".tab:not([hidden]):not(.dragging)")];
        const before = e.target.closest(".tabbar") ? tabs.find((t) => { const r = t.getBoundingClientRect(); return e.clientX < r.left + r.width / 2; }) : null;
        if (before) { if (marker.nextElementSibling !== before) bar.insertBefore(marker, before); }
        else if (bar.lastElementChild !== marker) bar.appendChild(marker);
      } else {
        marker.className = "drop-slot";
        const box = panelBox(dock);
        const before = panelsIn(dock).filter((p) => !p.hidden && p !== drag.panel)
          .find((p) => { const r = p.getBoundingClientRect(); return e.clientY < r.top + r.height / 2; });
        if (before) { if (marker.nextElementSibling !== before) box.insertBefore(marker, before); }
        else if (box.lastElementChild !== marker) box.appendChild(marker);
      }
    });
    dock.addEventListener("dragleave", (e) => { if (drag && !dock.contains(e.relatedTarget)) { dock.classList.remove("drop-target"); marker.remove(); } });
    dock.addEventListener("drop", (e) => {
      if (!drag || !marker.isConnected || !dock.contains(marker)) return;
      e.preventDefault();
      const box = panelBox(dock);
      let ref = null;
      if (isTabbed(dock)) {
        let next = marker.nextElementSibling;
        while (next && next.dataset.panel === drag.panel.id) next = next.nextElementSibling;
        ref = next ? document.getElementById(next.dataset.panel) : null;
      } else {
        ref = marker.nextElementSibling;
        while (ref && (!ref.classList.contains("panel") || ref === drag.panel)) ref = ref.nextElementSibling;
      }
      const panel = drag.panel;
      if (ref) box.insertBefore(panel, ref); else box.appendChild(panel);
      if (isTabbed(dock)) layout.active[dock.dataset.dock] = panel.id;
      else panel.classList.remove("is-active");
      end();
      docks().filter(isTabbed).forEach(buildTabs);
      saveLayout();
      syncDocks();
      // the console / camera tiles size themselves to their new dock
      if (panel.id === "console-card" && $("#log-follow").checked) $("#console").scrollTop = $("#console").scrollHeight;
    });
  }
}

// Resize the docks by dragging the gaps between them (double-click resets).
function wireSplitters() {
  const root = document.documentElement;
  const DEFS = {
    sidebar: { prop: "--sidebar-w", min: 220, max: () => 520, axis: "x", sign: 1 },
    inspector: { prop: "--inspector-w", min: 320, max: () => Math.min(680, innerWidth * 0.5), axis: "x", sign: -1 },
    bottom: { prop: "--bottom-h", min: 110, max: () => innerHeight * 0.7, axis: "y", sign: -1 },
  };
  const sizes = store.get(SIZES_KEY, {});
  for (const [k, v] of Object.entries(sizes)) if (DEFS[k]) root.style.setProperty(DEFS[k].prop, `${v}px`);

  for (const el of document.querySelectorAll(".split[data-split]")) {
    const def = DEFS[el.dataset.split];
    const target = { sidebar: "#dock-sidebar", inspector: "#dock-inspector", bottom: "#dock-bottom" }[el.dataset.split];
    el.addEventListener("pointerdown", (e) => {
      e.preventDefault();
      el.setPointerCapture(e.pointerId);
      const start = def.axis === "x" ? e.clientX : e.clientY;
      const box = $(target).getBoundingClientRect();
      const size0 = def.axis === "x" ? box.width : box.height;
      el.classList.add("active");
      document.body.classList.add("resizing");
      const move = (ev) => {
        const d = ((def.axis === "x" ? ev.clientX : ev.clientY) - start) * def.sign;
        const v = Math.round(Math.max(def.min, Math.min(def.max(), size0 + d)));
        root.style.setProperty(def.prop, `${v}px`);
        sizes[el.dataset.split] = v;
      };
      const up = () => {
        el.removeEventListener("pointermove", move);
        el.classList.remove("active");
        document.body.classList.remove("resizing");
        store.set(SIZES_KEY, sizes);
      };
      el.addEventListener("pointermove", move);
      el.addEventListener("pointerup", up, { once: true });
      el.addEventListener("pointercancel", up, { once: true });
    });
    el.addEventListener("dblclick", () => {
      root.style.removeProperty(def.prop);
      delete sizes[el.dataset.split];
      store.set(SIZES_KEY, sizes);
    });
  }
}

// ---------------------------------------------------------------- theme & menu

const DARK_QUERY = matchMedia("(prefers-color-scheme: dark)");
function applyTheme(pref) {
  const dark = pref === "auto" ? DARK_QUERY.matches : pref !== "light";
  document.documentElement.dataset.theme = dark ? "dark" : "light";
  document.documentElement.dataset.themePref = pref;
  $("#theme-seg").querySelectorAll("button").forEach((b) => b.classList.toggle("active", b.dataset.theme === pref));
}

function wireMenu() {
  const menu = $("#menu"), btn = $("#menu-btn");
  const close = () => { menu.hidden = true; btn.setAttribute("aria-expanded", "false"); };
  btn.onclick = (e) => { e.stopPropagation(); menu.hidden = !menu.hidden; btn.setAttribute("aria-expanded", String(!menu.hidden)); };
  document.addEventListener("click", (e) => { if (!menu.hidden && !e.target.closest(".menu-wrap")) close(); });
  document.addEventListener("keydown", (e) => { if (e.key === "Escape") close(); });
  $("#theme-seg").querySelectorAll("button").forEach((b) => (b.onclick = () => {
    try { localStorage.setItem("studio-theme", b.dataset.theme); } catch (e) {}   // plain string: read by the inline script
    applyTheme(b.dataset.theme);
  }));
  DARK_QUERY.addEventListener("change", () => { if (document.documentElement.dataset.themePref === "auto") applyTheme("auto"); });
  applyTheme(document.documentElement.dataset.themePref || "dark");
}

// ---------------------------------------------------------------- wiring

function rememberSelection() {
  post("collection/select", { dataset: $("#dataset").value.trim(), task: $("#task").value.trim(), instruction: $("#instruction").value.trim(), operator: $("#operator").value.trim() });
}

function checkedArms() {
  return [...$("#calib-arms").querySelectorAll("input:checked")].map((i) => i.value);
}

function wire() {
  $("#project-select").onchange = (e) => { ui.formLoaded = null; ui.selectedPolicy = null; post("project", { project: e.target.value }); };
  $("#mode-chip").onclick = () => { const t = $("#mode-chip").dataset.target; if (t) revealPanel(t); };
  $("#start-all").onclick = async () => {
    for (const c of ui.status.components) {
      if (c.name !== ui.status.data_collector && c.configured && !c.owned && !c.detected) await post("component/start", { name: c.name });
    }
  };
  $("#stop-all").onclick = async () => { for (const c of ui.status.components) if (c.owned) await post("component/stop", { name: c.name }); };

  $("#policies-refresh").onclick = async (e) => {
    const b = e.currentTarget; b.classList.add("spinning");
    await post("policies/refresh");
    setTimeout(() => b.classList.remove("spinning"), 500);
  };
  $("#policy-filter").oninput = () => { ui.lastSig.policies = null; ui.status && renderPolicies(ui.status); };
  $("#kind-filter").querySelectorAll("button").forEach((b) => (b.onclick = () => {
    ui.kindFilter = b.dataset.kind;
    $("#kind-filter").querySelectorAll("button").forEach((x) => x.classList.toggle("active", x === b));
    ui.lastSig.policies = null; ui.status && renderPolicies(ui.status);
  }));
  $("#load-btn").onclick = loadSelected;
  $("#unload-btn").onclick = () => post("policy/unload");
  $("#run-btn").onclick = () => post("policy/run", { prompt: $("#prompt").value });
  $("#stop-btn").onclick = () => post("policy/stop");
  $("#home-btn").onclick = () => post("home");
  $("#home-top-btn").onclick = () => post("home");
  $("#halt-btn").onclick = () => post("halt");
  $("#resume-btn").onclick = () => post("resume");
  $("#calib-watch").onchange = (e) => post("calibration/watch", { on: e.target.checked });

  $("#obj-add-form").onsubmit = (e) => {
    e.preventDefault();
    const prompt = $("#obj-new").value.trim();
    if (!prompt) return;
    $("#obj-new").value = "";
    saveObjects([...objectRows(), { name: "", prompt }]);
  };
  $("#obj-list").addEventListener("click", async (e) => {
    const rescan = e.target.closest("[data-rescan]");
    if (rescan) {
      const name = rescan.closest(".obj-row").dataset.name;
      const auto = $("#obj-auto").checked;
      const ok = await ask(auto
        ? { title: `Scan ${name} again?`, body: "Its mesh is replaced.", ok: "Scan", tone: "warn", iconId: "i-scan",
            motion: "Both arms move on their own around it, one after the other (slowly, within 30 cm of where they stand now). Keep the area clear and the Stop button in reach." }
        : { title: `Scan ${name} again?`, body: "Its mesh is replaced.\n\nMove the wrist cameras to views of it from all sides, Capture view at each, then Finish.", ok: "Scan", iconId: "i-scan" });
      if (ok) post("objects/scan", { names: [name], auto });
      return;
    }
    const btn = e.target.closest("[data-remove]");
    if (!btn) return;
    btn.closest(".obj-row").remove();
    saveObjects(objectRows());
  });
  $("#obj-list").addEventListener("change", (e) => {
    const row = e.target.closest(".obj-row");
    if (row) row.dataset.name = "";   // a new prompt is a new object (its own mesh folder)
    saveObjects(objectRows());
  });
  $("#obj-scan").onclick = async () => {
    const specs = ui.status?.pose_tracking?.specs || [];
    const names = specs.filter((o) => !o.mesh).map((o) => o.prompt).join(", ");
    if (!names) { post("objects/scan"); return; }   // all meshed: only starts the tracking
    const auto = $("#obj-auto").checked;
    if (auto && !await ask({ title: `Scan ${names}?`, ok: "Start scan", tone: "warn", iconId: "i-scan", body: "Point the wrist cameras roughly at the objects first.",
      motion: "Both arms move on their own around the objects, one after the other (slowly, within 30 cm of where they stand now). Keep the area clear and the Stop button in reach." })) return;
    post("objects/scan", { auto });
  };
  $("#obj-capture").onclick = () => post("objects/capture");
  $("#obj-finish").onclick = () => post("objects/finish");
  $("#obj-stop").onclick = () => post("objects/stop");

  $("#calib-start").onclick = async () => {
    const arms = checkedArms();
    if (!arms.length) { toast("Check at least one arm.", "warn", true); return; }
    if (!await ask({ title: "Calibrate cameras?", ok: "Calibrate", tone: "warn", iconId: "i-target",
      motion: `The ${arms.join(" and ")} will move on its own around the board (slowly, within 20 cm of its start pose). Keep the area clear and the Stop button in reach.` })) return;
    post("calibration/start", { arms });
  };
  $("#calib-add").onclick = async () => {
    const arms = checkedArms();
    if (!arms.length) { toast("Check at least one arm.", "warn", true); return; }
    if (!await ask({ title: "Board moved?", ok: "Add placement", tone: "warn", iconId: "i-target", body: "Every placement is then solved together.",
      motion: `The ${arms.join(" and ")} will move on its own around the board at its new place (slowly, within 20 cm of its start pose). Keep the area clear and the Stop button in reach.` })) return;
    post("calibration/add", { arms });
  };
  $("#calib-save").onclick = async () => {
    const r = await post("calibration/save", { name: $("#calib-name").value.trim() });
    if (r.ok) toast(r.message, "success", true);
  };
  $("#calib-apply").onclick = async () => {
    const name = $("#calib-saved").value;
    if (!name || !await ask({ title: `Apply “${name}”?`, ok: "Apply", tone: "warn", iconId: "i-target",
      body: "It overwrites the camera mounts / bases in the robot configs and the project's camera poses." })) return;
    const r = await post("calibration/apply", { name });
    if (r.ok) { toast(r.message, "success", true); $("#calib-restart").hidden = !r.restart_robots; }
  };
  $("#calib-restart").onclick = async () => {
    if (!await ask({ title: "Restart the robots now?", ok: "Restart", tone: "warn", iconId: "i-refresh",
      body: "Both arms stop and come back with the new description." })) return;
    await post("component/restart", { name: "robot" });
    $("#calib-restart").hidden = true;
  };
  $("#prompt").onkeydown = (e) => { if (e.key === "Enter" && (e.ctrlKey || e.metaKey)) $("#run-btn").click(); };

  for (const id of ["#dataset", "#task", "#instruction", "#operator"]) $(id).onchange = rememberSelection;
  $("#task").addEventListener("change", () => {
    const t = $("#task").value.trim();
    const instr = ui.status?.tasks?.[t];
    if (instr && !$("#instruction").value) $("#instruction").value = instr;
  });
  $("#rec-btn").onclick = () => {
    const c = ui.status.collection;
    if (c.recording) post("collection/stop");
    else post("collection/start", { dataset: $("#dataset").value.trim(), task: $("#task").value.trim(), instruction: $("#instruction").value.trim(), operator: $("#operator").value.trim() });
  };
  $("#discard-btn").onclick = async () => {
    if (await ask({ title: "Discard the last episode?", body: "It is deleted from disk. This cannot be undone.", ok: "Discard", tone: "danger", iconId: "i-trash" })) post("collection/discard");
  };

  const syncBody = (convert) => ({ dataset: $("#dataset").value.trim(), task: $("#task").value.trim(), convert_config: $("#convert-config").value, convert });
  $("#sync-only").onclick = () => post("cluster/sync", syncBody(false));
  $("#sync-convert").onclick = () => post("cluster/sync", syncBody(true));
  $("#train-btn").onclick = () => post("cluster/train", { config: $("#train-config").value, dataset: $("#dataset").value.trim(), task: $("#task").value.trim() });
  $("#jobs-refresh").onclick = async (e) => {
    const b = e.currentTarget; b.classList.add("spinning");
    await post("cluster/refresh");
    b.classList.remove("spinning");
  };

  $("#log-clear").onclick = () => { $("#console").innerHTML = ""; };
  $("#viewer-reload").onclick = () => { const f = $("#viewer-frame"); if (f.src) f.src = f.src; };
  $("#viewer-full").onclick = () => $("#viewer").requestFullscreen?.();

  // keyboard: space = Stop (holds the arms: policy, teleop, homing, calibration), when not typing
  document.addEventListener("keydown", (e) => {
    if (["INPUT", "TEXTAREA", "SELECT"].includes(document.activeElement?.tagName)) return;
    if (e.code === "Space") { e.preventDefault(); post("halt"); }
  });
}

// Image tile size per strip (remembered in this browser)
function wireSizes() {
  document.querySelectorAll(".size-range").forEach((r) => {
    const grid = document.getElementById(r.dataset.target);
    const key = `studio-tile-${r.dataset.target}`;
    try { const v = localStorage.getItem(key); if (v) r.value = v; } catch (e) {}
    const apply = () => grid.style.setProperty("--tile", `${r.value}px`);
    r.oninput = () => { apply(); try { localStorage.setItem(key, r.value); } catch (e) {} };
    apply();
  });
}

wire();
wireMenu();
wireSizes();
wireLayout();
connect();
