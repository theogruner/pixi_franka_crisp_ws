// Franka Studio front end. All state comes from the server's `status` message; the page keeps
// only view state (selection, filters, followed log) and the form inputs.

const $ = (sel) => document.querySelector(sel);
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const PAGE_T0 = Date.now() / 1000;

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

// ---------------------------------------------------------------- helpers

const ICONS = { robot: "i-robot", camera: "i-camera", teleop: "i-gamepad", gamepad: "i-gamepad", record: "i-record", server: "i-server", brain: "i-brain", cloud: "i-points", points: "i-points" };
const MODALITY_ICONS = { state: "i-robot", rgb: "i-camera", pointcloud: "i-points", depth: "i-layers" };

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
  el.innerHTML = `<div>${esc(text)}</div>`;
  $("#toasts").appendChild(el);
  setTimeout(() => { el.classList.add("out"); setTimeout(() => el.remove(), 350); }, level === "error" ? 7000 : 4000);
  while ($("#toasts").children.length > 5) $("#toasts").firstChild.remove();
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
  el.querySelector("span:last-child").textContent = online ? "live" : "reconnecting…";
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
  renderViewer(s);
  for (const e of s.events || []) if (e.t > PAGE_T0 - 1) toast(e.text, e.level);
}

function renderTop(s) {
  if (changed("projects", [s.projects, s.project?.id])) {
    $("#project-select").innerHTML = (s.projects.length ? s.projects : ["(no projects)"])
      .map((p) => `<option ${p === s.project?.id ? "selected" : ""}>${esc(p)}</option>`).join("");
  }
  $("#env-name").textContent = s.env ? s.env.name : "no scene env";
  const chip = $("#mode-chip");
  const rec = s.collection.recording;
  const running = s.policy?.state === "running";
  const calib = s.calibration?.state === "running";
  chip.className = "chip " + (s.halted ? "halted" : rec ? "recording" : running || calib ? "running" : "");
  $("#mode-text").textContent = s.halted ? "STOPPED · resume to continue" : rec ? `recording${s.policy ? " · DAgger" : ""}` : calib ? "calibrating cameras" : running ? "policy running" : s.loading ? "loading policy…" : s.policy ? "policy ready" : "idle";
  $("#resume-btn").hidden = !s.halted;
  $("#home-top-btn").disabled = !!s.halted;
  document.body.classList.toggle("is-halted", !!s.halted);
}

function renderComponents(s) {
  const comps = s.components;
  const box = $("#components");
  if (changed("comp-list", comps.map((c) => c.name))) {
    box.innerHTML = comps.map((c) => `
      <div class="comp" data-name="${esc(c.name)}">
        <div class="comp-icon"><svg class="i"><use href="#${ICONS[c.icon] || "i-box"}"/></svg></div>
        <div class="comp-body">
          <div class="comp-name">${esc(c.label)}</div>
          <div class="comp-state"><span class="dot"></span><span class="txt"></span></div>
        </div>
        <div class="comp-actions">
          <button class="comp-log" title="Show output">log</button>
          <button class="btn btn-soft comp-toggle"></button>
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
      el.querySelector(".comp-log").onclick = () => follow(name);
    });
    renderTabs(s);
  }
  for (const c of comps) {
    const el = box.querySelector(`.comp[data-name="${CSS.escape(c.name)}"]`);
    if (!el) continue;
    const st = c.owned ? c.state : c.detected ? "running" : c.state;
    el.className = `comp is-${st}`;
    let txt = { running: "running", starting: "starting…", stopping: "stopping…", stopped: "stopped", exited: "exited", failed: `failed (${c.exit_code})` }[st] || st;
    if (c.owned && c.uptime != null) txt += ` · ${fmtDuration(c.uptime)}`;
    if (!c.owned && c.detected) txt = "running · external";
    if (!c.configured) txt = "not configured";
    el.querySelector(".txt").textContent = txt;
    const btn = el.querySelector(".comp-toggle");
    btn.textContent = c.owned ? "Stop" : "Start";
    btn.className = `btn comp-toggle ${c.owned ? "btn-soft danger-text" : "btn-soft"}`;
    btn.disabled = !c.configured || st === "stopping" || (!c.owned && c.detected);
    btn.title = c.cmd || "Set a command in config/studio.yaml";
  }
  renderTabStates(s);
}

function renderTabs(s) {
  $("#log-tabs").innerHTML = s.components.map((c) => `<button class="tab" data-name="${esc(c.name)}"><span class="dot"></span>${esc(c.label)}</button>`).join("");
  $("#log-tabs").querySelectorAll(".tab").forEach((t) => (t.onclick = () => follow(t.dataset.name)));
  if (!ui.logName && s.components.length) follow(s.components[0].name);
  else highlightTab();
}

function renderTabStates(s) {
  for (const c of s.components) {
    const t = $(`#log-tabs .tab[data-name="${CSS.escape(c.name)}"]`);
    if (t) t.classList.toggle("running", c.owned && c.state === "running"), t.classList.toggle("failed", c.state === "failed");
  }
}

function highlightTab() {
  document.querySelectorAll("#log-tabs .tab").forEach((t) => t.classList.toggle("active", t.dataset.name === ui.logName));
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
      <div class="mod" data-id="${esc(m.id)}">
        <div class="mod-icon"><svg class="i"><use href="#${MODALITY_ICONS[m.id] || (m.component ? "i-points" : "i-eye")}"/></svg></div>
        <div class="mod-body">
          <div class="mod-name"><span>${esc(m.label)}</span><span class="tags"></span></div>
          <div class="mod-desc"></div>
          <div class="mod-keys"></div>
        </div>
        <label class="toggle" title=""><input type="checkbox"><span></span></label>
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
    el.querySelector(".toggle").title = m.required ? "Always part of the observation" : recording ? "Stop the recording first" : m.enabled ? "Switch off" : "Switch on";
    const tags = [];
    if (m.required) tags.push(`<span class="tag lock">always</span>`);
    if (m.enabled && m.record.length) tags.push(`<span class="tag rec" title="${esc(m.record.join("\n"))}">recorded</span>`);
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
  const extra = s.record_topics || [];
  $("#record-line").innerHTML = extra.length ? `Recorded with each episode: ${extra.map((t) => `<code>${esc(t)}</code>`).join(" ")}` : "Recorded: the collector's topics and RGB videos.";
}

function globMatch(pattern, key) {
  const re = new RegExp("^" + pattern.replace(/[.+^${}()|[\]\\]/g, "\\$&").replace(/\*/g, ".*").replace(/\?/g, ".") + "$");
  return re.test(key);
}

function renderPointCloud(s) {
  const pcMod = (s.modalities || []).find((m) => m.component && m.enabled);
  const pc = s.pointcloud;
  const el = $("#pc-status");
  el.hidden = !pcMod;
  if (!pcMod) return;
  el.className = `pc-status ${pc?.live ? "live" : "stale"}`;
  if (!pc || !pc.live) {
    el.innerHTML = `<div class="pc-head"><span class="dot"></span><span>${pcMod.component_up ? "Waiting for the point cloud node…" : `Starting ${esc(pcMod.component)}… (needs the cameras)`}</span></div>`;
    return;
  }
  const cams = Object.entries(pc.cameras || {});
  el.innerHTML = `<div class="pc-head"><span class="dot"></span><b>${pc.hz.toFixed(1)} Hz</b><span><b>${(pc.points / 1000).toFixed(1)}k</b> points</span><span class="muted">engine ${esc(pc.engine)}</span></div>
    <div class="pc-cams">${cams.map(([n, c]) => `<span>${esc(n)}</span><span>${c.ms ? `${c.ms.toFixed(0)} ms` : "—"}</span>${c.error ? `<span class="err">${esc(c.error)}</span>` : ""}`).join("")}</div>`;
}

// ---- policies

function policyMatches(p) {
  const words = $("#policy-filter").value.toLowerCase().split(/\s+/).filter(Boolean);
  const hay = `${p.name} ${p.file} ${p.policy_type}`.toLowerCase();
  return (!ui.kindFilter || p.kind === ui.kindFilter) && words.every((w) => hay.includes(w));
}

function renderPolicies(s) {
  const list = s.policies.filter(policyMatches);
  const loadedId = s.policy?.id;
  if (!ui.selectedPolicy || !s.policies.some((p) => p.id === ui.selectedPolicy)) {
    ui.selectedPolicy = s.remembered?.policy && s.policies.some((p) => p.id === s.remembered.policy) ? s.remembered.policy : s.policies[0]?.id || null;
  }
  if (changed("policies", [list, loadedId, ui.selectedPolicy, s.loading])) {
    $("#policy-list").innerHTML = list.length ? list.map((p) => {
      const kind = p.kind === "vision_language" ? `<span class="tag vl">Vision + Language</span>` : `<span class="tag v">Vision</span>`;
      const step = p.step ? ` · step ${esc(p.step.replace(/^0+/, "") || p.step)}` : "";
      const trained = p.trained ? ` · ${esc(p.trained.replace("T", " "))}` : "";
      return `<button class="pol ${p.id === ui.selectedPolicy ? "selected" : ""} ${p.id === loadedId ? "loaded" : ""}" data-id="${p.id}" title="${esc(p.file)}">
        <span class="pol-name">${esc(p.name)}${p.id === loadedId ? " ●" : ""}${s.loading === p.id ? " …" : ""}</span>
        <span class="pol-meta">${kind}${p.policy_type ? `<span class="tag">${esc(p.policy_type)}</span>` : ""}${p.error ? `<span class="tag err" title="${esc(p.error)}">problem</span>` : ""}<span>${step}${trained}</span></span>
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
  btn.innerHTML = `<svg class="i"><use href="#i-download"/></svg>${s.loading ? "Loading…" : sel && sel.id === loadedId ? "Loaded" : loadedId ? "Switch to this policy" : "Load policy"}`;
  btn.title = sel?.error || "";
}

function loadSelected() {
  if (ui.selectedPolicy) post("policy/load", { id: ui.selectedPolicy });
}

function renderPolicyActive(s) {
  const p = s.policy;
  $("#policy-active").hidden = !p;
  if (!p) return;
  const spec = s.policies.find((x) => x.id === p.id) || {};
  $("#pa-name").textContent = p.name;
  $("#pa-meta").textContent = [spec.policy_type, spec.server_address, p.frequency ? `${p.frequency} Hz` : ""].filter(Boolean).join(" · ");
  const pill = $("#pa-state");
  pill.textContent = p.state;
  pill.className = `state-pill ${p.state}`;
  const frac = Math.min(1, p.queued / Math.max(1, p.actions_per_chunk));
  $("#queue-fill").style.width = `${frac * 100}%`;
  $("#queue-threshold").style.left = `${Math.min(100, (p.threshold / Math.max(1, p.actions_per_chunk)) * 100)}%`;
  $("#pa-stats").innerHTML = `<span>queue <b>${p.queued}/${p.actions_per_chunk}</b></span><span>ctrl <b>${p.control_hz ?? "—"}</b> Hz</span><span>chunks <b>${p.inference_hz ?? "—"}</b>/s</span><span>steps <b>${p.steps}</b>${p.run_time != null ? ` · ${fmtDuration(p.run_time)}` : ""}</span>`;
  const lang = p.kind === "vision_language";
  $("#prompt-box").hidden = !lang;
  if (lang && changed("prompt-chips", [p.id, spec.prompts])) {
    const prompt = $("#prompt");
    if (!prompt.value) prompt.value = s.remembered?.prompt && (spec.prompts || []).length === 0 ? s.remembered.prompt : (spec.prompts || [])[0] || s.remembered?.prompt || "";
    $("#prompt-chips").innerHTML = (spec.prompts || []).map((t) => `<button title="${esc(t)}">${esc(t)}</button>`).join("");
    $("#prompt-chips").querySelectorAll("button").forEach((b) => (b.onclick = () => ($("#prompt").value = b.title)));
  }
  const running = p.state === "running";
  $("#run-btn").disabled = running || p.state !== "ready" || s.collection.recording;
  $("#run-btn").title = s.collection.recording ? "Stop the recording first" : "";
  $("#stop-btn").disabled = !running;
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
  $("#ds-count").textContent = ds && task ? (n != null ? `${n} episode${n === 1 ? "" : "s"} in ${ds}/${task}` : `new task — ${ds}/${task} will be created`) : "Choose a dataset and task.";
  const badge = $("#collect-kind");
  badge.textContent = s.policy ? "DAgger" : "Demo";
  badge.className = `badge ${s.policy ? "dagger" : ""}`;
  badge.title = s.policy ? `Episodes are tagged with ${s.policy.name}` : "";

  const rec = $("#rec-btn");
  const collectorUp = s.components.some((x) => x.name === s.data_collector && (x.owned && x.state !== "stopping" || x.detected));
  const policyRunning = s.policy?.state === "running";
  rec.classList.toggle("on", c.recording);
  rec.disabled = !c.recording && (policyRunning || !collectorUp || !ds || !task);
  rec.title = policyRunning ? "Stop the policy first" : !collectorUp ? "Start the data collector" : !ds || !task ? "Choose dataset and task" : c.recording ? "Stop recording" : "Start recording";
  $("#rec-time").textContent = c.recording && c.current ? fmtDuration(c.current.duration) : "00:00";
  $("#rec-sub").textContent = c.recording && c.current ? `→ ${c.current.dataset}/${c.current.task}` : policyRunning ? "Policy running — stop it to record" : !collectorUp ? "Data collector not running" : c.message || "Ready";
  $("#discard-btn").disabled = !c.can_discard || c.recording;

  if (changed("episodes", c.history)) {
    $("#episodes").innerHTML = c.history.map((e) => `<div class="ep" title="${esc(e.path)}${e.error ? "\n" + esc(e.error) : ""}">
      <span class="ep-name">${esc(e.name)}</span><span class="muted">${fmtDuration(e.duration)}</span>
      ${e.kind === "dagger" ? `<span class="tag vl">dagger</span>` : ""}
      <span class="ep-status ${e.status}">${e.status}</span></div>`).join("");
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
  $("#features").innerHTML = vecs.map((f) => `<span class="feat ${f.ok ? "ok" : "bad"}" title="${esc(f.topics.join("\n"))}"><span class="dot"></span><b>${esc(f.key)}</b>${f.shape ? ` [${f.shape.join("×")}]` : ""}${f.age != null ? ` · ${(f.age * 1000).toFixed(0)} ms` : " · waiting"}</span>`).join("");
  const missing = feats.filter((f) => !f.ok).length;
  $("#obs-missing").textContent = feats.length ? (missing ? `${missing} of ${feats.length} features not live` : `${feats.length} features live`) : "";
  const p = s.policy;
  $("#obs-rate").textContent = p?.state === "running" && p.control_hz ? `${p.control_hz} Hz · policy` : s.env ? `monitor` : "— Hz";
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
    el.querySelector(".rate").textContent = c.ok ? `${(c.age * 1000).toFixed(0)} ms` : "stale";
  }
  const live = cams.filter((c) => c.ok).length;
  $("#rig-status").textContent = cams.length ? `${live} of ${cams.length} live` : "";
}

function pollCam(el, urlFor) {
  const img = el.querySelector("img");
  const key = el.dataset.key;
  const next = () => {
    if (!el.isConnected) return;
    ui.camTimers[urlFor(key)] = setTimeout(load, el.classList.contains("stale") ? 1000 : 110);
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
    $("#convert-config").innerHTML = (s.conversion_configs.length ? [""].concat(s.conversion_configs) : [""]).map((c) => `<option value="${esc(c)}">${c ? esc(c) : "— none (command default) —"}</option>`).join("");
  }
  if (changed("train-configs", s.train_configs)) {
    $("#train-config").innerHTML = s.train_configs.length ? s.train_configs.map((c) => `<option>${esc(c)}</option>`).join("") : `<option value="">— add configs to train_configs/ —</option>`;
  }
  const ds = $("#dataset").value.trim(), task = $("#task").value.trim();
  const ready = s.cluster?.configured && ds && task && !s.collection.recording;
  $("#sync-only").disabled = !ready;
  $("#sync-convert").disabled = !ready;
  $("#train-btn").disabled = !s.cluster?.configured || !s.train_configs.length;
  if (changed("jobs", s.jobs)) {
    const open = new Set([...document.querySelectorAll(".job[open]")].map((d) => d.dataset.id));
    $("#jobs").innerHTML = s.jobs.length ? s.jobs.map((j) => {
      const state = j.slurm_state || j.status;
      const when = new Date(j.created * 1000).toLocaleString([], { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" });
      return `<details class="job" data-id="${esc(j.id)}" ${open.has(j.id) ? "open" : ""}>
        <summary><span class="job-title" title="${esc(j.title)}">${esc(j.title)}</span><span class="muted small">${when}</span>
        <span class="job-state ${esc(state)}">${esc(state)}${j.slurm_id ? ` #${esc(j.slurm_id)}` : ""}</span></summary>
        <pre>${esc(j.lines.join("\n"))}</pre></details>`;
    }).join("") : `<div class="empty">No cluster jobs yet.</div>`;
  }
}

// ---- viewer

// ---- camera calibration

function renderCalibration(s) {
  const c = s.calibration;
  if (!c) return;
  const watch = $("#calib-watch");
  if (document.activeElement !== watch) watch.checked = c.watching;
  const vis = c.visibility || {};
  $("#calib-vis").innerHTML = Object.entries(vis).map(([cam, n]) => {
    const cls = n == null ? "" : n >= 4 ? "seen" : "none";
    const txt = n == null ? (c.watching || c.state === "running" ? "…" : "off") : `${n} markers`;
    return `<span class="cam-chip ${cls}" title="${esc(cam)}">${esc(cam.replace("zed_", ""))} · ${txt}</span>`;
  }).join("");
  const arms = Object.keys(c.arms || {});
  if (changed("calib-arms", arms)) {
    $("#calib-arms").innerHTML = arms.map((a) => `<label><input type="checkbox" value="${esc(a)}" checked>${esc(a.replace("zed_", ""))}</label>`).join("");
  }
  const runningCal = c.state === "running";
  $("#calib-start").disabled = runningCal || s.halted;
  $("#calib-start").title = s.halted ? "Resume first" : runningCal ? "Running — Stop aborts it" : "Probe and drive to the views with the checked arms, one after the other";
  const prog = $("#calib-progress");
  prog.hidden = !(c.progress || []).length;
  if (!prog.hidden && changed("calib-prog", [c.state, c.phase, c.progress])) {
    prog.innerHTML = `<div class="phase ${esc(c.state)}">${esc(c.state === "running" ? c.phase : c.state)}</div>${esc((c.progress || []).join("\n"))}`;
    prog.scrollTop = prog.scrollHeight;
  }
  const res = $("#calib-result");
  res.hidden = !c.result;
  $("#calib-save-row").hidden = !c.result;
  if (c.result && changed("calib-res", c.result)) {
    const r = c.result;
    const rows = Object.entries(r.rms).map(([cam, v]) => {
      const before = r.rms_before?.[cam];
      return `<span>${esc(cam)}</span><span class="${v < 2 ? "good" : "bad"}">${v.toFixed(2)} px${before != null ? ` <span class="muted">(was ${before.toFixed(1)})</span>` : ""}</span>`;
    }).join("");
    const robots = Object.entries(r.values?.robots || {}).map(([robot, vals]) =>
      `<span>${esc(robot)}</span><span class="muted">${esc(Object.keys(vals).some((k) => k.startsWith("cam_")) ? "camera mount" : "")}${vals.xyz ? (Object.keys(vals).length > 2 ? " + base" : "base") : ""}</span>`).join("");
    const statics = Object.keys(r.values?.static || {}).map((cam) => `<span>${esc(cam)}</span><span class="muted">pose</span>`).join("");
    res.innerHTML = `<span class="head">Reprojection error</span>${rows}<span class="head">Changes when applied</span>${robots}${statics}`;
  }
  const saved = c.saved || [];
  if (changed("calib-saved", saved.map((x) => x.name))) {
    $("#calib-saved").innerHTML = saved.length
      ? saved.map((x) => `<option value="${esc(x.name)}">${esc(x.name)}${x.created ? ` · ${esc(x.created.replace("T", " ").slice(0, 16))}` : ""}</option>`).join("")
      : `<option value="">(none yet)</option>`;
  }
  $("#calib-apply").disabled = !saved.length || runningCal;
}

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

// ---------------------------------------------------------------- wiring

function rememberSelection() {
  post("collection/select", { dataset: $("#dataset").value.trim(), task: $("#task").value.trim(), instruction: $("#instruction").value.trim() });
}

function wire() {
  $("#theme-toggle").onclick = () => {
    const next = document.documentElement.dataset.theme === "light" ? "dark" : "light";
    document.documentElement.dataset.theme = next;
    try { localStorage.setItem("studio-theme", next); } catch (e) {}
  };
  $("#project-select").onchange = (e) => { ui.formLoaded = null; ui.selectedPolicy = null; post("project", { project: e.target.value }); };
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
  $("#calib-start").onclick = () => {
    const arms = [...$("#calib-arms").querySelectorAll("input:checked")].map((i) => i.value);
    if (!arms.length) { toast("Check at least one arm.", "warn", true); return; }
    if (!confirm(`The ${arms.join(" and ")} will move on its own around the board (slowly, within 20 cm of its start pose).\n\nKeep the area clear and the Stop button in reach.`)) return;
    post("calibration/start", { arms });
  };
  $("#calib-save").onclick = async () => {
    const r = await post("calibration/save", { name: $("#calib-name").value.trim() });
    if (r.ok) toast(r.message, "success", true);
  };
  $("#calib-apply").onclick = async () => {
    const name = $("#calib-saved").value;
    if (!name || !confirm(`Apply calibration '${name}'? It overwrites the camera mounts / bases in the robot configs and the project's camera poses.`)) return;
    const r = await post("calibration/apply", { name });
    if (r.ok) { toast(r.message, "success", true); $("#calib-restart").hidden = !r.restart_robots; }
  };
  $("#calib-restart").onclick = async () => {
    if (!confirm("Restart the robots now? Both arms stop and come back with the new description.")) return;
    await post("component/restart", { name: "robot" });
    $("#calib-restart").hidden = true;
  };
  $("#prompt").onkeydown = (e) => { if (e.key === "Enter" && (e.ctrlKey || e.metaKey)) $("#run-btn").click(); };

  for (const id of ["#dataset", "#task", "#instruction"]) $(id).onchange = rememberSelection;
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
  $("#discard-btn").onclick = () => { if (confirm("Delete the last recorded demo from disk?")) post("collection/discard"); };

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
wireSizes();
connect();
