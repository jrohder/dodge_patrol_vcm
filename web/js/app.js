/* Dodge Patrol VCM dashboard */
"use strict";

const $ = (id) => document.getElementById(id);
const T = (id, v) => { const e = $(id); if (e && e.textContent !== String(v)) e.textContent = v; };

/* ---------------------------------------------------------- navigation */
document.querySelectorAll("#tabbar button").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll("#tabbar button").forEach((b) => b.classList.remove("active"));
    document.querySelectorAll(".page").forEach((p) => p.classList.remove("active"));
    btn.classList.add("active");
    $("page-" + btn.dataset.page).classList.add("active");
    if (btn.dataset.page === "config" && !cfgLoaded) loadConfig();
    if (btn.dataset.page === "fw") refreshOta();
    if (btn.dataset.page === "sys") loadSystem();
  });
});

async function api(path, opts) {
  const res = await fetch(path, opts);
  return res.json().catch(() => ({}));
}
function post(path, body) {
  return api(path, { method: "POST", headers: { "Content-Type": "application/json" },
    body: body ? JSON.stringify(body) : "{}" });
}

/* ---------------------------------------------------------- websocket */
let ws = null, wsUp = false, lastTelemetry = 0;

function wsConnect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { setConn(true); };
  ws.onclose = () => { setConn(false); setTimeout(wsConnect, 1500); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type === "telemetry") { lastTelemetry = Date.now(); onTelemetry(m); }
  };
}
function setConn(up) {
  wsUp = up;
  const el = $("conn");
  el.textContent = up ? "CONNECTED" : "DISCONNECTED";
  el.classList.toggle("on", up);
  el.classList.toggle("off", !up);
}
function wsSend(obj) { if (wsUp && ws.readyState === 1) ws.send(JSON.stringify(obj)); }
wsConnect();
Units.refreshLabels();
setInterval(() => { if (wsUp && Date.now() - lastTelemetry > 4000) setConn(false); }, 2000);

/* ---------------------------------------------------------- telemetry */
function fmtUptime(s) {
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
  return h ? `${h}h ${m}m` : m ? `${m}m ${s % 60}s` : `${s}s`;
}

function onTelemetry(m) {
  const { sys, drv, str, nano, pwr, imu } = m;

  if (sys.units !== undefined) {
    Units.setMode(sys.units);
    Units.refreshLabels();
  }

  // header
  const st = $("sys-state");
  st.textContent = sys.state;
  st.className = "badge state " +
    (["READY", "DRIVING"].includes(sys.state) ? "ok"
      : ["FAULT", "ESTOP"].includes(sys.state) ? "err" : "warn");
  $("banner-commission").classList.toggle("hidden", sys.comm);
  $("banner-fault").classList.toggle("hidden", !["FAULT", "ESTOP"].includes(sys.state));
  $("btn-estop-clear").classList.toggle("hidden", sys.state !== "ESTOP");

  // dashboard (speeds/distances converted for display units)
  T("d-speed", Units.fmtSpd(Math.abs(drv.spd)));
  T("d-src", sys.src);
  T("d-batt", pwr.v);
  T("d-thr", Math.round(drv.thr * 100));
  T("d-steer", str.acta);
  T("d-lw", Units.fmtSpd(drv.la)); T("d-rw", Units.fmtSpd(drv.ra));
  T("d-li", pwr.li); T("d-ri", pwr.ri); T("d-si", pwr.si);
  T("d-rc", nano.rcv ? "OK" : "LOST");
  T("d-nano", nano.online ? "ONLINE" : "OFFLINE");
  T("d-bias", drv.bias);
  T("d-odo", Units.fmtDist(drv.odo)); T("d-trip", Units.fmtDist(drv.trip));
  T("d-up", fmtUptime(sys.up));

  // remote
  T("r-spd", Units.fmtSpd(drv.spd)); T("r-act", str.acta);

  // calibration live values
  T("c-raw", str.raw); T("c-filt", str.filt);
  T("c-pos", str.act); T("c-ang", str.acta);
  T("c-pwm", str.pwm); T("c-cur", str.cur);
  T("c-lf", nano.lf); T("c-rf", nano.rf);
  T("c-lp", nano.lp); T("c-rp", nano.rp);
  T("c-lc", nano.lc); T("c-rc", nano.rc2);
  T("c-lrpm", drv.lrpm); T("c-rrpm", drv.rrpm);
  T("c-ls", Units.fmtSpd(drv.la)); T("c-rs", Units.fmtSpd(drv.ra));
  T("c-iraw", imu.raw_a.map((v) => v.toFixed(2)).join(" / "));
  T("c-icor", imu.a.map((v) => v.toFixed(2)).join(" / "));
  T("c-pitch", imu.pitch); T("c-roll", imu.roll); T("c-yaw", imu.yaw);
  T("c-imuh", imu.health);

  // diagnostics
  T("g-sreq", str.req); T("g-sact", str.act); T("g-serr", str.err);
  T("g-spid", `${str.p} / ${str.i} / ${str.d}`);
  T("g-sout", str.out); T("g-spwm", str.pwm);
  T("g-thr", drv.thr);
  T("g-req", Units.fmtSpd(drv.req)); T("g-lim", Units.fmtSpd(drv.lim));
  T("g-l", `${Units.fmtSpd(drv.lt)} / ${Units.fmtSpd(drv.la)}`);
  T("g-r", `${Units.fmtSpd(drv.rt)} / ${Units.fmtSpd(drv.ra)}`);
  T("g-pwm", `${drv.lpwm} / ${drv.rpwm}`);
  T("g-slip", drv.slip ? "DETECTED" : "no");
  T("g-bias", drv.bias);
  T("g-nano", nano.online ? "ONLINE" : "OFFLINE");
  T("g-rate", nano.rate); T("g-rx", nano.rx); T("g-tel", nano.tel);
  T("g-lost", nano.lost); T("g-crc", nano.crc); T("g-ferr", nano.ferr);
  T("g-seq", nano.seq); T("g-age", nano.age); T("g-hbage", nano.hbage);
  T("g-jit", nano.jit); T("g-proto", nano.proto); T("g-nfw", nano.fw);
  T("g-nup", nano.nup); T("g-boot", nano.boot); T("g-wdg", nano.wdg);
  T("g-ncrc", nano.ncrc); T("g-nferr", nano.nferr);
  T("g-nloop", `${nano.nlavg} / ${nano.nloop} us`);
  T("g-ncpu", nano.ncpu);
  T("g-nover", `${nano.nsched} / ${nano.nsamp}`);
  T("g-ntx", `${nano.ntxd} / ${nano.ncmd}`);
  T("g-nglitch", nano.nglitch); T("g-nram", nano.nram);
  T("g-acks", `${nano.acks} / ${nano.flts}`);
  T("g-ts", nano.ts + " us");
  T("g-per", `${nano.lp} / ${nano.rp} us`);
  T("g-rcus", nano.rc.join(" "));
  T("g-rcm", "0x" + (nano.rcm || 0).toString(16).padStart(2, "0"));
  T("g-adc", nano.adc.join(" "));
  T("g-nff", "0x" + nano.ff.toString(16).padStart(4, "0"));
  T("g-nout", `${nano.nstate} / 0x${(nano.out||0).toString(16)}`);
  T("g-cpu", sys.cpu);
  T("g-heap", (sys.heap / 1024).toFixed(0) + " kB");
  T("g-minheap", (sys.minheap / 1024).toFixed(0) + " kB");
  T("g-dyn", `${sys.dyn_avg}us avg, ${sys.dyn_max}us max, ${sys.dyn_miss} miss`);
  T("g-str", `${sys.str_avg}us avg, ${sys.str_max}us max, ${sys.str_miss} miss`);
  T("g-rssi", sys.rssi + " dBm");
}

/* ---------------------------------------------------------- estop */
$("btn-estop").onclick = () => { wsSend({ type: "estop" }); post("/api/estop"); };
$("btn-estop-clear").onclick = () => post("/api/estop/clear");

/* ---------------------------------------------------------- web remote */
let remoteActive = false, lights = false, siren = false;

function remoteLoop() {
  if (remoteActive && wsUp) {
    wsSend({ type: "ctrl", t: +$("r-thr").value / 100, s: +$("r-str").value / 100,
      brake: false, lights, siren });
  }
}
setInterval(remoteLoop, 100);

$("btn-take").onclick = () => {
  remoteActive = true;
  wsSend({ type: "take" });
  $("remote-status").textContent = "WEB CONTROL ACTIVE - vehicle follows the sliders";
};
$("btn-release").onclick = () => {
  remoteActive = false;
  wsSend({ type: "release" });
  resetSliders();
  $("remote-status").textContent = "Control released";
};
$("btn-rstop").onclick = () => {
  resetSliders();
  wsSend({ type: "ctrl", t: 0, s: 0, brake: true, lights, siren });
};
$("btn-lights").onclick = (e) => { lights = !lights; e.target.classList.toggle("on", lights); };
$("btn-siren").onclick = (e) => { siren = !siren; e.target.classList.toggle("on", siren); };

function resetSliders() {
  $("r-thr").value = 0; $("r-str").value = 0;
  T("r-thr-val", "0%"); T("r-str-val", "0%");
}
["r-thr", "r-str"].forEach((id) => {
  const el = $(id);
  el.addEventListener("input", () => T(id + "-val", el.value + "%"));
  const recenter = () => { el.value = 0; T(id + "-val", "0%"); remoteLoop(); };
  el.addEventListener("pointerup", recenter);
  el.addEventListener("touchend", recenter);
});

/* ---------------------------------------------------------- config */
let cfgLoaded = false, schema = null, dirtyKeys = {};

async function loadConfig() {
  schema = await api("/api/config/schema");
  cfgLoaded = true;
  const unitsParam = schema.params.find((p) => p.key === "ui.units");
  if (unitsParam) {
    Units.setMode(Math.round(unitsParam.value));
    Units.refreshLabels();
  }
  const cats = [...new Set(schema.params.map((p) => p.key.split(".")[0])
    .concat(schema.strings.map((p) => p.key.split(".")[0])))].sort();
  $("cfg-cat").innerHTML = cats.map((c) => `<option>${c}</option>`).join("");
  renderConfig();
}
$("cfg-cat").onchange = renderConfig;
$("cfg-level").onchange = renderConfig;

function levelRank(l) { return { basic: 0, advanced: 1, expert: 2 }[l] ?? 0; }

function renderConfig() {
  const cat = $("cfg-cat").value;
  const maxLevel = levelRank($("cfg-level").value);
  const host = $("cfg-list");
  host.innerHTML = "";
  const items = schema.params.filter((p) => p.key.startsWith(cat + ".") &&
    levelRank(p.level) <= maxLevel);
  const strs = schema.strings.filter((p) => p.key.startsWith(cat + ".") &&
    levelRank(p.level) <= maxLevel);

  for (const p of items) host.appendChild(cfgItem(p, false));
  for (const p of strs) host.appendChild(cfgItem(p, true));
  if (!items.length && !strs.length)
    host.innerHTML = '<p class="hint">No parameters at this level.</p>';
}

function cfgItem(p, isString) {
  const div = document.createElement("div");
  div.className = "cfg-item" + (p.danger ? " danger" : "");
  const dispUnit = Units.displayUnit(p.units);
  const dispVal = isString ? p.value
    : (p.type === "bool" || p.type === "enum") ? p.value
    : Units.roundDisp(Units.fromSi(p.value, p.units), p.units);
  const dispMin = Units.roundDisp(Units.fromSi(p.min, p.units), p.units);
  const dispMax = Units.roundDisp(Units.fromSi(p.max, p.units), p.units);
  const dispDef = Units.roundDisp(Units.fromSi(p.def, p.units), p.units);
  let ctrl;
  if (isString) {
    ctrl = `<input type="${p.key.includes("password") ? "password" : "text"}" value="${String(p.value).replace(/"/g, "&quot;")}" data-key="${p.key}" data-str="1" style="max-width:220px">`;
  } else if (p.type === "bool") {
    ctrl = `<input type="checkbox" ${p.value ? "checked" : ""} data-key="${p.key}">`;
  } else if (p.type === "enum") {
    const opts = p.options.split("|").map((o, i) =>
      `<option value="${i}" ${i === Math.round(p.value) ? "selected" : ""}>${o}</option>`).join("");
    ctrl = `<select data-key="${p.key}">${opts}</select>`;
  } else {
    const step = p.type === "int" ? 1 : "any";
    ctrl = `<input type="number" value="${dispVal}" min="${dispMin}" max="${dispMax}" step="${step}" data-key="${p.key}" data-si-unit="${p.units || ""}">`;
  }
  div.innerHTML = `<div class="head"><span class="name">${p.name}
      ${dispUnit ? `<span class="units">(${dispUnit})</span>` : ""}
      ${p.danger ? '<span class="dtag">&#9888; CAUTION</span>' : ""}
      ${p.restart ? '<span class="units">[restart]</span>' : ""}</span>${ctrl}</div>
    <div class="desc">${p.desc || ""}</div>
    ${!isString && p.type !== "bool" && p.type !== "enum"
      ? `<div class="range">min ${dispMin} &middot; default ${dispDef} &middot; max ${dispMax}</div>` : ""}`;
  div.querySelector("[data-key]").addEventListener("change", (e) => {
    const el = e.target;
    if (p.danger && !confirm(`"${p.name}" is a safety-critical setting.\n\nApply this change deliberately?`)) {
      renderConfig(); return;
    }
    let v;
    if (el.dataset.str) v = el.value;
    else if (el.type === "checkbox") v = el.checked ? 1 : 0;
    else if (p.type === "enum") v = +el.value;
    else v = Units.toSi(+el.value, p.units);
    dirtyKeys[p.key] = v;
    div.classList.add("dirty");
    if (p.key === "ui.units") {
      Units.setMode(+el.value);
      Units.refreshLabels();
    }
  });
  return div;
}

$("cfg-apply").onclick = async () => {
  if (!Object.keys(dirtyKeys).length) return alert("No changes to apply");
  const r = await post("/api/config/apply", dirtyKeys);
  if (r.ok) { dirtyKeys = {}; await loadConfig(); }
  alert(r.ok ? "Applied (RAM only - use SAVE to persist)" : `Applied ${r.applied}, failed ${r.failed}`);
};
$("cfg-save").onclick = async () => {
  if (Object.keys(dirtyKeys).length) {
    await post("/api/config/apply", dirtyKeys); dirtyKeys = {};
  }
  await post("/api/config/save");
  await loadConfig();
  alert("Configuration saved to flash");
};
$("cfg-revert").onclick = async () => {
  await post("/api/config/revert"); dirtyKeys = {}; await loadConfig();
};
$("cfg-factory").onclick = async () => {
  if (!confirm("Restore ALL configuration to factory defaults?")) return;
  if (!confirm("This cannot be undone (calibration data is kept). Continue?")) return;
  await post("/api/config/factory-reset");
  await loadConfig();
};
$("cfg-import").addEventListener("change", async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const json = JSON.parse(await file.text());
  const r = await post("/api/config/import", json);
  alert(r.ok ? `Imported ${r.applied} parameters (use SAVE to persist)` : "Import failed");
  await loadConfig();
});

/* ---------------------------------------------------------- calibration */
async function refreshCal() {
  if (!$("page-cal").classList.contains("active")) return;
  const c = await api("/api/cal/status");
  const s = c.steering;
  T("c-state", s.state);
  T("c-lims", s.valid ? `${s.left_adc.toFixed(0)} / ${s.center_effective.toFixed(0)} / ${s.right_adc.toFixed(0)}` : "not set");
  const pill = $("cal-str-valid");
  pill.textContent = s.valid ? "VALID" : "NOT CALIBRATED";
  pill.className = "pill " + (s.valid ? "ok" : "err");
  T("cal-comm", c.commissioned ? "COMMISSIONED" : "NOT COMMISSIONED");
}
setInterval(refreshCal, 1000);

$("cal-str-start").onclick = async () => {
  if (!confirm("Front wheels lifted and steering travel unobstructed?")) return;
  const r = await post("/api/cal/steering/start", {
    pwm: +$("cal-pwm").value, current: +$("cal-amp").value,
    no_motion_ms: +$("cal-hold").value });
  if (!r.ok) alert(r.error || "Could not start");
};
$("cal-str-abort").onclick = () => post("/api/cal/steering/abort");
$("cal-msave").onclick = () => post("/api/cal/steering/manual", {
  left: +$("cal-mleft").value, center: +$("cal-mcenter").value,
  right: +$("cal-mright").value });

document.querySelectorAll("[data-steer]").forEach((b) => {
  b.onclick = () => {
    $("cal-steer-slider").value = b.dataset.steer;
    post("/api/cal/steering/target", { pct: +b.dataset.steer, enabled: true });
  };
});
$("cal-steer-slider").addEventListener("change", (e) =>
  post("/api/cal/steering/target", { pct: +e.target.value, enabled: true }));
$("cal-steer-off").onclick = () =>
  post("/api/cal/steering/target", { pct: 50, enabled: false });

let trim = 0;
document.querySelectorAll("[data-trim]").forEach((b) => {
  b.onclick = async () => {
    trim += +b.dataset.trim;
    T("trim-val", trim.toFixed(1));
    await post("/api/config/apply", { "steering.center_trim": trim });
  };
});

$("mt-left").onclick = () => motorTest(true, false);
$("mt-right").onclick = () => motorTest(false, true);
$("mt-both").onclick = () => motorTest(true, true);
$("mt-stop").onclick = () => post("/api/cal/motor-test/stop");
async function motorTest(l, r) {
  if (!confirm("Rear wheels lifted off the ground?")) return;
  const res = await post("/api/cal/motor-test",
    { left: l, right: r, pwm: +$("mt-level").value, duration_ms: 5000 });
  if (!res.ok) alert(res.error || "Could not start");
}

$("cal-imu-zero").onclick = () => post("/api/cal/imu/zero");
$("cal-cur-zero").onclick = async () => {
  if (!confirm("All motors off / vehicle idle?")) return;
  await post("/api/cal/current/zero");
  alert("Offset captured into current.offset - APPLY+SAVE in Configuration to persist");
};
$("cal-commission").onclick = async () => {
  const r = await post("/api/cal/commission", { done: true });
  alert(r.ok ? "Vehicle commissioned - normal driving unlocked" : r.error);
};
$("cal-decommission").onclick = () => post("/api/cal/commission", { done: false });

/* ---------------------------------------------------------- diagnostics */
async function refreshFaults() {
  if (!$("page-diag").classList.contains("active")) return;
  const d = await api("/api/faults");
  const tb = $("flt-tbl").querySelector("tbody");
  tb.innerHTML = (d.faults || []).map((f) =>
    `<tr><td><b>${f.code}</b></td><td>${f.desc}</td><td>${f.severity}</td>
     <td>${f.active ? '<b class="err" style="color:var(--err)">ACTIVE</b>' : "-"}</td>
     <td>${f.count}</td></tr>`).join("") ||
    '<tr><td colspan="5" class="hint">No faults recorded</td></tr>';
}
setInterval(refreshFaults, 2000);
$("flt-clear").onclick = () => post("/api/faults/clear");
$("rec-resume").onclick = () => post("/api/recorder/resume");
$("trip-reset").onclick = () => post("/api/trip/reset");
let telPaused = false;
$("tel-pause").onclick = (e) => {
  telPaused = !telPaused;
  wsSend({ type: "pause", paused: telPaused });
  e.target.textContent = telPaused ? "Resume Telemetry" : "Pause Telemetry";
};

/* ---------------------------------------------------------- logs */
let logAfter = 0;
async function pollLogs() {
  if (!$("page-logs").classList.contains("active")) return;
  const d = await api("/api/logs?after=" + logAfter);
  if (!d.entries || !d.entries.length) return;
  const view = $("log-view");
  const filter = $("log-filter").value;
  for (const e of d.entries) {
    logAfter = Math.max(logAfter, e.seq);
    if (filter && e.lvl !== filter) continue;
    const t = (e.ms / 1000).toFixed(3).padStart(9);
    const line = document.createElement("span");
    line.className = e.lvl;
    line.textContent = `${t} [${e.lvl.padEnd(5)}] ${e.mod.padEnd(7)} ${e.msg}\n`;
    view.appendChild(line);
  }
  while (view.childNodes.length > 400) view.removeChild(view.firstChild);
  view.scrollTop = view.scrollHeight;
}
setInterval(pollLogs, 1000);
$("log-clear-view").onclick = () => { $("log-view").innerHTML = ""; };
$("log-filter").onchange = () => { $("log-view").innerHTML = ""; logAfter = 0; };

/* ---------------------------------------------------------- firmware */
async function refreshOta() {
  const s = await api("/api/ota/status");
  T("fw-cur", s.current_version);
  T("fw-date", s.build_date);
  T("fw-latest", s.latest_version || "--");
  T("fw-state", s.state);
  const busy = ["DOWNLOADING", "FLASHING", "CHECKING"].includes(s.state);
  $("fw-prog").classList.toggle("hidden", !["DOWNLOADING", "FLASHING"].includes(s.state));
  $("fw-bar").style.width = s.progress + "%";
  T("fw-pct", s.progress + "%");
  $("fw-install").classList.toggle("hidden", s.state !== "UPDATE_AVAILABLE");
  $("fw-err").classList.toggle("hidden", !s.error);
  if (s.error) T("fw-err", s.error);
  const notes = $("fw-notes");
  notes.classList.toggle("hidden", !s.release_notes);
  notes.textContent = s.release_notes || "";
  if (busy || $("page-fw").classList.contains("active"))
    setTimeout(refreshOta, busy ? 700 : 3000);
}
$("fw-check").onclick = async () => { await post("/api/ota/check"); refreshOta(); };
$("fw-install").onclick = async () => {
  if (!confirm("Install the update? The vehicle stops and reboots when done.")) return;
  await post("/api/ota/install");
  refreshOta();
};
let fwFile = null;
$("fw-file").addEventListener("change", (e) => {
  fwFile = e.target.files[0] || null;
  T("fw-fname", fwFile ? `${fwFile.name} (${(fwFile.size / 1024).toFixed(0)} kB)` : "");
  $("fw-upload").disabled = !fwFile;
});
$("fw-upload").onclick = async () => {
  if (!fwFile || !confirm("Flash this firmware image?")) return;
  const form = new FormData();
  form.append("firmware", fwFile);
  $("fw-upload").disabled = true;
  try {
    const res = await fetch("/api/ota/upload", { method: "POST", body: form });
    const d = await res.json().catch(() => ({}));
    alert(d.ok ? "Update verified - rebooting. Reconnect in ~20 s." : "Update failed: " + (d.error || res.status));
  } catch (err) {
    alert("Upload failed: " + err);
  }
  $("fw-upload").disabled = false;
  refreshOta();
};

/* ---------------------------------------------------------- system */
async function loadSystem() {
  const s = await api("/api/system");
  T("veh-name", s.vehicle_name || "Dodge Patrol");
  const rows = {
    "Firmware": s.fw_version, "Commit": s.git_commit, "Built": s.build_date,
    "Nano protocol": "v" + s.protocol_version, "Chip": s.chip,
    "Display units": s.units || Units.name(),
    "Flash": (s.flash_kb / 1024).toFixed(0) + " MB",
    "Sketch": s.sketch_kb + " kB", "Boot partition": s.partition,
    "Uptime": fmtUptime(s.uptime_s), "Free heap": (s.heap / 1024).toFixed(0) + " kB",
    "State": s.state, "Commissioned": s.commissioned ? "yes" : "no",
    "IP address": s.ip, "AP active": s.ap_active ? "yes" : "no",
    "Hostname": s.hostname + ".local",
  };
  $("sys-tbl").querySelector("tbody").innerHTML = Object.entries(rows)
    .map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join("");
}
$("sys-reboot").onclick = async () => {
  if (!confirm("Reboot the VCM?")) return;
  await post("/api/reboot");
};

loadSystem();
