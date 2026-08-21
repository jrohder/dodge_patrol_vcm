/* Steering diagnostics graph — canvas oscilloscope, 60 s × 200 Hz. */
"use strict";

const SDiag = (() => {
  const CAP = 12000;
  const SAMPLE = 28;
  const STATES = ["IDLE","MOVING_LEFT","MOVING_RIGHT","APPROACHING","SETTLING",
    "HOLDING","DEADBAND","LIMIT","FAULT","CALIBRATION"];
  const TRACES = [
    { id: "sp", name: "Setpoint", color: "#6cb6ff", axis: "pos", on: true },
    { id: "filt", name: "Filtered Feedback", color: "#3dd68c", axis: "pos", on: true },
    { id: "raw", name: "Raw Feedback", color: "#8b97ad", axis: "pos", on: false },
    { id: "err", name: "Error", color: "#e3b341", axis: "pos", on: false },
    { id: "pwm", name: "PWM", color: "#f778ba", axis: "pwm", on: true },
    { id: "p", name: "P", color: "#a371f7", axis: "pwm", on: false },
    { id: "i", name: "I", color: "#79c0ff", axis: "pwm", on: false },
    { id: "d", name: "D", color: "#ffa657", axis: "pwm", on: false },
    { id: "ff", name: "Feed Forward", color: "#56d4dd", axis: "pwm", on: false },
    { id: "vel", name: "Velocity", color: "#db6d28", axis: "vel", on: false },
    { id: "cur", name: "Current", color: "#f85149", axis: "cur", on: false },
  ];
  const PRESETS = {
    basic: ["sp", "filt", "pwm"],
    loop: ["sp", "filt", "err", "p", "i", "d", "ff", "pwm"],
    mech: ["filt", "vel", "pwm", "cur"],
    char: ["sp", "filt", "pwm", "vel"],
    all: TRACES.map((t) => t.id),
  };

  const t_us = new Uint32Array(CAP);
  const buf = {};
  TRACES.forEach((tr) => { buf[tr.id] = new Float32Array(CAP); });
  const dir = new Int8Array(CAP);
  const st = new Uint8Array(CAP);
  const curOk = new Uint8Array(CAP);
  let head = 0, count = 0, t0 = 0;
  let paused = false, viewEndUs = 0, viewDurUs = 60000000;
  let cursorIdx = -1, streaming = false, raf = 0;
  let events = [];

  function pushSample(s) {
    const i = head;
    t_us[i] = s.t;
    buf.sp[i] = s.sp; buf.filt[i] = s.filt; buf.raw[i] = s.raw;
    buf.err[i] = s.err; buf.pwm[i] = s.pwm; buf.p[i] = s.p;
    buf.i[i] = s.i; buf.d[i] = s.d; buf.ff[i] = s.ff;
    buf.vel[i] = s.vel; buf.cur[i] = s.curOk ? s.cur : NaN;
    dir[i] = s.dir; st[i] = s.st; curOk[i] = s.curOk ? 1 : 0;
    if (!t0) t0 = s.t;
    head = (head + 1) % CAP;
    if (count < CAP) count++;
  }

  function unpack(dv, off) {
    const flags = dv.getUint8(off + 26);
    const cma = dv.getInt16(off + 24, true);
    const curValid = (flags & 0x08) !== 0 && cma !== -32768;
    return {
      t: dv.getUint32(off, true),
      sp: dv.getInt16(off + 4, true) / 100,
      raw: dv.getUint16(off + 6, true),
      filt: dv.getInt16(off + 8, true) / 100,
      err: dv.getInt16(off + 10, true) / 100,
      pwm: dv.getInt16(off + 12, true) / 100,
      p: dv.getInt16(off + 14, true) / 100,
      i: dv.getInt16(off + 16, true) / 100,
      d: dv.getInt16(off + 18, true) / 100,
      ff: dv.getInt16(off + 20, true) / 100,
      vel: dv.getInt16(off + 22, true) / 100,
      cur: curValid ? cma / 1000 : NaN,
      curOk: curValid,
      dir: (flags & 0x03) - 1,
      st: dv.getUint8(off + 27),
    };
  }

  function ingestBinary(buf) {
    if (buf.byteLength < 12) return;
    const dv = new DataView(buf);
    if (dv.getUint8(0) !== 0x53 || dv.getUint8(1) !== 0x44) return;
    const countN = dv.getUint16(4, true);
    const payloadOff = 12;
    for (let i = 0; i < countN; i++) {
      const off = payloadOff + i * SAMPLE;
      if (off + SAMPLE > buf.byteLength) break;
      pushSample(unpack(dv, off));
    }
  }

  function idxAt(nFromOldest) {
    return (head - count + nFromOldest + CAP) % CAP;
  }

  function lastIdx() { return (head + CAP - 1) % CAP; }

  function nowUs() {
    if (!count) return 0;
    return t_us[lastIdx()];
  }

  function sampleAtTime(tu) {
    if (!count) return -1;
    let best = 0, bestD = 1e15;
    for (let n = 0; n < count; n++) {
      const i = idxAt(n);
      const d = Math.abs(t_us[i] - tu);
      if (d < bestD) { bestD = d; best = i; }
    }
    return best;
  }

  function draw() {
    const canvas = $("sdiag-canvas");
    if (!canvas) return;
    const wrap = $("sdiag-wrap");
    const dpr = window.devicePixelRatio || 1;
    const w = wrap.clientWidth, h = wrap.clientHeight;
    if (canvas.width !== (w * dpr) | 0 || canvas.height !== (h * dpr) | 0) {
      canvas.width = (w * dpr) | 0;
      canvas.height = (h * dpr) | 0;
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#07090e";
    ctx.fillRect(0, 0, w, h);
    if (!count) return;

    const tEnd = paused ? viewEndUs : nowUs();
    const tStart = tEnd - viewDurUs;
    const padL = 42, padR = 42, padT = 8, padB = 18;
    const gw = w - padL - padR, gh = h - padT - padB;

    ctx.strokeStyle = "#262f45";
    ctx.lineWidth = 1;
    ctx.font = "10px -apple-system, sans-serif";
    ctx.fillStyle = "#8592ab";
    for (let g = 0; g <= 4; g++) {
      const y = padT + gh * g / 4;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + gw, y); ctx.stroke();
      ctx.fillText(String(100 - g * 25), 4, y + 3);
      ctx.textAlign = "right";
      ctx.fillText(String(100 - g * 50), w - 4, y + 3);
      ctx.textAlign = "left";
    }
    ctx.fillText("% pos", 4, padT + 10);
    ctx.textAlign = "right";
    ctx.fillText("PWM", w - 4, padT + 10);
    ctx.textAlign = "left";

    const yPos = (v) => padT + gh * (1 - (v / 100));
    const yPwm = (v) => padT + gh * (1 - ((v + 100) / 200));
    const yVel = (v) => padT + gh * (1 - ((v + 20) / 40));
    const yCur = (v) => padT + gh * (1 - (v / 10));
    const xOf = (t) => padL + gw * (t - tStart) / (tEnd - tStart || 1);

    const axisY = { pos: yPos, pwm: yPwm, vel: yVel, cur: yCur };

    // markers
    events.forEach((e) => {
      if (e.t < tStart || e.t > tEnd) return;
      const x = xOf(e.t);
      ctx.strokeStyle = "rgba(210,153,34,.7)";
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + gh); ctx.stroke();
    });

    TRACES.forEach((tr) => {
      if (!tr.on) return;
      const arr = buf[tr.id];
      const yfn = axisY[tr.axis];
      ctx.strokeStyle = tr.color;
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      let started = false;
      const visHz = 200;
      const visN = Math.max(1, Math.ceil(viewDurUs / 1e6 * visHz));
      const step = Math.max(1, Math.floor(visN / (gw * 2)));
      for (let n = 0; n < count; n += step) {
        const i = idxAt(n);
        const t = t_us[i];
        if (t < tStart || t > tEnd) continue;
        const v = arr[i];
        if (v !== v) continue;
        const x = xOf(t), y = yfn(tr.id === "raw" ? v / 40.95 : v);
        if (!started) { ctx.moveTo(x, y); started = true; }
        else ctx.lineTo(x, y);
      }
      if (started) ctx.stroke();
    });

    if (cursorIdx >= 0) {
      const t = t_us[cursorIdx];
      const x = xOf(t);
      ctx.strokeStyle = "#dfe6f2";
      ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + gh); ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  function cursorText(i) {
    if (i < 0) return "Touch the graph for a cursor";
    const t = (t_us[i] - t0) / 1e6;
    const cur = curOk[i] ? buf.cur[i].toFixed(3) + " A" : "n/a";
    return [
      `Time ${t.toFixed(4)} s`,
      `Setpoint ${buf.sp[i].toFixed(2)} %`,
      `Raw Feedback ${buf.raw[i].toFixed(0)}`,
      `Filtered ${buf.filt[i].toFixed(2)} %`,
      `Error ${buf.err[i].toFixed(3)} %`,
      `PWM ${buf.pwm[i].toFixed(2)} %`,
      `P ${buf.p[i].toFixed(2)}  I ${buf.i[i].toFixed(2)}  D ${buf.d[i].toFixed(2)}`,
      `Feed Forward ${buf.ff[i].toFixed(2)} %`,
      `Velocity ${buf.vel[i].toFixed(2)} %/s`,
      `Current ${cur}`,
      `State ${STATES[st[i]] || st[i]}`,
    ].join("\n");
  }

  function windowStats() {
    if (!count) return;
    const tEnd = paused ? viewEndUs : nowUs();
    const tStart = tEnd - viewDurUs;
    let n = 0, e2 = 0, eSum = 0, maxE = 0, pAbs = 0, pSum = 0, maxV = 0;
    let lastD = 0, lastP = 0, dch = 0, prev = 0;
    for (let k = 0; k < count; k++) {
      const i = idxAt(k);
      if (t_us[i] < tStart || t_us[i] > tEnd) continue;
      const e = buf.err[i], p = buf.pwm[i], v = buf.vel[i];
      e2 += e * e; eSum += e; maxE = Math.max(maxE, Math.abs(e));
      pAbs = Math.max(pAbs, Math.abs(p)); pSum += Math.abs(p);
      maxV = Math.max(maxV, Math.abs(v));
      const d = dir[i];
      const ps = p > 0.8 ? 1 : p < -0.8 ? -1 : 0;
      if (d && lastD && d !== lastD) dch++;
      if (ps && lastP && ps !== lastP) prev++;
      if (d) lastD = d;
      if (ps) lastP = ps;
      n++;
    }
    if (!n) return;
    T("sd-rms", (Math.sqrt(e2 / n)).toFixed(3));
    T("sd-maxe", maxE.toFixed(3));
    T("sd-avge", (eSum / n).toFixed(3));
    T("sd-ppwm", pAbs.toFixed(1));
    T("sd-apwm", (pSum / n).toFixed(1));
    T("sd-mvel", maxV.toFixed(2));
    T("sd-dch", dch);
    T("sd-prev", prev);
  }

  function loop() {
    if ($("page-steer") && $("page-steer").classList.contains("active")) {
      draw();
      windowStats();
    }
    raf = requestAnimationFrame(loop);
  }

  function bindCanvas() {
    const canvas = $("sdiag-canvas");
    if (!canvas) return;
    const toTime = (clientX) => {
      const r = canvas.getBoundingClientRect();
      const x = (clientX - r.left) / r.width;
      const tEnd = paused ? viewEndUs : nowUs();
      return tEnd - viewDurUs + x * viewDurUs;
    };
    let panLastX = 0;
    canvas.addEventListener("pointerdown", (e) => {
      panLastX = e.clientX;
      const tu = toTime(e.clientX);
      cursorIdx = sampleAtTime(tu);
      T("sdiag-cursor", cursorText(cursorIdx));
    });
    canvas.addEventListener("pointermove", (e) => {
      if (!paused || !(e.buttons & 1) || !count) return;
      const r = canvas.getBoundingClientRect();
      const dt = -(e.clientX - panLastX) / Math.max(1, r.width) * viewDurUs;
      panLastX = e.clientX;
      const tMin = t_us[idxAt(0)];
      const tMax = nowUs();
      viewEndUs = Math.min(tMax, Math.max(tMin + viewDurUs, viewEndUs + dt));
    });
    canvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      const scale = e.deltaY > 0 ? 1.12 : 0.88;
      viewDurUs = Math.min(60000000, Math.max(500000, viewDurUs * scale));
    }, { passive: false });
    let pinch0 = 0;
    canvas.addEventListener("touchstart", (e) => {
      if (e.touches.length === 2) {
        pinch0 = Math.hypot(
          e.touches[0].clientX - e.touches[1].clientX,
          e.touches[0].clientY - e.touches[1].clientY);
      }
    }, { passive: true });
    canvas.addEventListener("touchmove", (e) => {
      if (e.touches.length === 2 && pinch0) {
        const d = Math.hypot(
          e.touches[0].clientX - e.touches[1].clientX,
          e.touches[0].clientY - e.touches[1].clientY);
        const scale = pinch0 / d;
        viewDurUs = Math.min(60000000, Math.max(500000, viewDurUs * scale));
        pinch0 = d;
        if (!paused) viewEndUs = nowUs();
      } else if (e.touches.length === 1 && paused) {
        const tu = toTime(e.touches[0].clientX);
        cursorIdx = sampleAtTime(tu);
        T("sdiag-cursor", cursorText(cursorIdx));
      }
    }, { passive: true });
  }

  function setPreset(name) {
    const ids = PRESETS[name] || PRESETS.basic;
    TRACES.forEach((tr) => { tr.on = ids.includes(tr.id); });
    renderToggles();
  }

  function renderToggles() {
    const el = $("sdiag-toggles");
    if (!el) return;
    el.innerHTML = TRACES.map((tr) =>
      `<label><input type="checkbox" data-tr="${tr.id}" ${tr.on ? "checked" : ""}>
       <span class="swatch" style="background:${tr.color}"></span>${tr.name}</label>`
    ).join("");
    el.querySelectorAll("input").forEach((inp) => {
      inp.onchange = () => {
        const tr = TRACES.find((t) => t.id === inp.dataset.tr);
        if (tr) tr.on = inp.checked;
      };
    });
  }

  async function loadSnapshot() {
    try {
      const res = await fetch("/api/steer/diag.bin");
      const ab = await res.arrayBuffer();
      ingestBinary(ab);
    } catch (e) { /* first boot, empty */ }
    try {
      const ev = await api("/api/steer/diag/events");
      events = (ev.events || []).map((e) => ({ t: e.t, type: e.type }));
    } catch (e) { events = []; }
  }

  function onWsBinary(ev) {
    if (ev.data instanceof ArrayBuffer) ingestBinary(ev.data);
    else if (ev.data instanceof Blob) {
      ev.data.arrayBuffer().then(ingestBinary);
    }
  }

  function setStreaming(on) {
    streaming = on;
    if (typeof wsSend === "function") wsSend({ type: "sdiag", on });
  }

  function pngDownload(canvas, name) {
    canvas.toBlob((blob) => {
      if (!blob) return;
      const a = document.createElement("a");
      a.href = URL.createObjectURL(blob);
      a.download = name;
      a.click();
      setTimeout(() => URL.revokeObjectURL(a.href), 2000);
    }, "image/png");
  }

  function samplesJson() {
    const rows = [];
    for (let n = 0; n < count; n++) {
      const i = idxAt(n);
      rows.push({
        timestamp_us: t_us[i],
        time_s: (t_us[i] - t0) / 1e6,
        setpoint: buf.sp[i],
        raw_feedback: buf.raw[i],
        filtered_feedback: buf.filt[i],
        error: buf.err[i],
        pwm: buf.pwm[i],
        direction: dir[i],
        p_term: buf.p[i], i_term: buf.i[i], d_term: buf.d[i],
        feedforward: buf.ff[i], velocity: buf.vel[i],
        control_state: STATES[st[i]] || st[i],
        current: curOk[i] ? buf.cur[i] : null,
      });
    }
    return rows;
  }

  function init() {
    renderToggles();
    bindCanvas();
    document.querySelectorAll("[data-preset]").forEach((b) => {
      b.onclick = () => setPreset(b.dataset.preset);
    });
    $("sdiag-pause").onclick = () => {
      paused = true;
      viewEndUs = nowUs();
      $("sdiag-pause").textContent = "PAUSED";
    };
    $("sdiag-live").onclick = () => {
      paused = false;
      viewDurUs = 60000000;
      $("sdiag-pause").textContent = "PAUSE";
    };
    $("sdiag-reset").onclick = () => {
      viewDurUs = 60000000;
      if (!paused) viewEndUs = nowUs();
      cursorIdx = -1;
      T("sdiag-cursor", "Touch the graph for a cursor");
    };
    $("sdiag-clear").onclick = async () => {
      if (!confirm("Clear the 60-second diagnostic recorder? Characterization history is kept.")) return;
      await post("/api/steer/diag/clear", {});
      head = 0; count = 0; t0 = 0; cursorIdx = -1;
    };
    $("sdiag-png").onclick = () => pngDownload($("sdiag-canvas"), "steering_diagnostics.png");
    $("sdiag-session").onclick = async () => {
      const meta = await api("/api/system");
      const cal = await api("/api/cal/status");
      const ch = await api("/api/cal/char/status");
      const blob = new Blob([JSON.stringify({
        saved: new Date().toISOString(),
        firmware: meta.fw_version,
        git: meta.git_commit,
        schema: meta.protocol_version,
        configuration_note: "See /api/config/export",
        steering_calibration: cal.steering,
        characterization: ch.current,
        traces: samplesJson(),
      }, null, 2)], { type: "application/json" });
      const a = document.createElement("a");
      a.href = URL.createObjectURL(blob);
      a.download = "steering_session.json";
      a.click();
    };
    $("sdiag-export").onclick = () => exportZip();
    if ($("diag-open-steer")) {
      $("diag-open-steer").onclick = (e) => {
        e.preventDefault();
        document.querySelector('#tabbar button[data-page="steer"]').click();
      };
    }
    raf = requestAnimationFrame(loop);
    setInterval(async () => {
      if (!$("page-steer") || !$("page-steer").classList.contains("active")) return;
      try {
        const s = await api("/api/steer/diag/status");
        T("sd-health", s.health != null ? s.health : "--");
      } catch (e) {}
      try {
        const ev = await api("/api/steer/diag/events");
        events = (ev.events || []).map((e) => ({ t: e.t, type: e.type }));
      } catch (e) {}
    }, 800);
  }

  async function exportZip() {
    const files = {};
    const fetchText = async (url) => {
      const r = await fetch(url);
      return await r.text();
    };
    try { files["metadata.json"] = await fetchText("/api/steer/export/metadata.json"); } catch (e) {}
    try { files["configuration.json"] = await fetchText("/api/config/export"); } catch (e) {}
    try { files["steering_calibration.json"] = JSON.stringify(await api("/api/cal/status"), null, 2); } catch (e) {}
    try { files["characterization.json"] = await fetchText("/api/cal/char/json"); } catch (e) {}
    try { files["diagnostic.csv"] = await fetchText("/api/steer/diag/csv"); } catch (e) {}
    const zip = makeZip(files);
    const a = document.createElement("a");
    a.href = URL.createObjectURL(zip);
    a.download = "steering_diagnostics.zip";
    a.click();
  }

  function crc32(buf) {
    let c = ~0;
    for (let i = 0; i < buf.length; i++) {
      c ^= buf[i];
      for (let k = 0; k < 8; k++) c = (c >>> 1) ^ (0xedb88320 & -(c & 1));
    }
    return ~c >>> 0;
  }

  function makeZip(files) {
    const enc = new TextEncoder();
    const locals = [], centrals = [];
    let offset = 0;
    const parts = [];
    Object.keys(files).forEach((name) => {
      const data = enc.encode(files[name] || "");
      const n = enc.encode(name);
      const crc = crc32(data);
      const loc = new Uint8Array(30 + n.length);
      const dv = new DataView(loc.buffer);
      dv.setUint32(0, 0x04034b50, true);
      dv.setUint16(4, 20, true);
      dv.setUint16(8, 0, true);
      dv.setUint32(14, crc, true);
      dv.setUint32(18, data.length, true);
      dv.setUint32(22, data.length, true);
      dv.setUint16(26, n.length, true);
      loc.set(n, 30);
      parts.push(loc, data);
      const cen = new Uint8Array(46 + n.length);
      const c = new DataView(cen.buffer);
      c.setUint32(0, 0x02014b50, true);
      c.setUint16(4, 20, true);
      c.setUint16(6, 20, true);
      c.setUint32(16, crc, true);
      c.setUint32(20, data.length, true);
      c.setUint32(24, data.length, true);
      c.setUint16(28, n.length, true);
      c.setUint32(42, offset, true);
      cen.set(n, 46);
      centrals.push(cen);
      offset += loc.length + data.length;
    });
    const centralSize = centrals.reduce((s, x) => s + x.length, 0);
    const end = new Uint8Array(22);
    const e = new DataView(end.buffer);
    e.setUint32(0, 0x06054b50, true);
    e.setUint16(8, centrals.length, true);
    e.setUint16(10, centrals.length, true);
    e.setUint32(12, centralSize, true);
    e.setUint32(16, offset, true);
    const blobParts = [...parts, ...centrals, end];
    return new Blob(blobParts, { type: "application/zip" });
  }

  function onPage(active) {
    if (active) {
      setStreaming(true);
      loadSnapshot();
    } else setStreaming(false);
  }

  return { init, onWsBinary, onPage, ingestBinary, samples: () => ({ t_us, buf, head, count, idxAt, t0 }), pngDownload, TRACES };
})();
