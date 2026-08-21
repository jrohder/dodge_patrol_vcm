/* Smart characterization wizard UI + graphs / report. */
"use strict";

const SChar = (() => {
  let last = null;

  function pct(v, u) {
    if (v === undefined || v === null || v === "") return "--";
    return Number(v).toFixed(1) + (u || "%");
  }

  function render(d) {
    last = d;
    const c = d.current || {};
    const pill = $("cal-char-valid");
    if (pill) {
      pill.textContent = c.status || "NONE";
      pill.className = "pill " + (c.status === "OK" ? "ok" : c.status === "FAILED" ? "err" : "");
    }
    T("ch-sl", pct(c.min_start_left)); T("ch-sr", pct(c.min_start_right));
    T("ch-hl", pct(c.hold_left)); T("ch-hr", pct(c.hold_right));
    T("ch-pl", pct(c.preferred_left)); T("ch-pr", pct(c.preferred_right));
    T("ch-ml", pct(c.max_test_left)); T("ch-mr", pct(c.max_test_right));
    T("ch-vl", c.max_vel_left != null ? Number(c.max_vel_left).toFixed(2) + " %/s" : "--");
    T("ch-vr", c.max_vel_right != null ? Number(c.max_vel_right).toFixed(2) + " %/s" : "--");
    T("ch-bl", pct(c.backlash_left)); T("ch-br", pct(c.backlash_right));
    T("ch-set", `${c.overshoot || "--"} % / ${c.settling_s || "--"} s  err ${c.final_error || "--"}`);
    T("ch-status", c.status === "FAILED" ? ("FAILED: " + (c.fail_reason || "")) : (c.status || "--"));
    T("char-live", d.active
      ? `${d.phase}: ${d.message || ""} (${d.progress || 0}%)`
      : (d.phase === "DONE" ? "Complete" : d.phase === "FAILED" ? ("FAILED " + (c.fail_reason || "")) : "Idle"));
    $("char-prog").classList.toggle("hidden", !d.active);
    $("char-bar").style.width = (d.progress || 0) + "%";
    T("char-pct", (d.progress || 0) + "%");

    const r = d.recommended || {}, s = d.settings || {};
    if (r.valid) {
      $("char-recs").textContent =
        `                Current     Recommended\n` +
        `P               ${fmt(s.kp)}      ${fmt(r.kp)}\n` +
        `I               ${fmt(s.ki)}      ${fmt(r.ki)}\n` +
        `D               ${fmt(s.kd)}      ${fmt(r.kd)}\n` +
        `Far P           ${fmt(s.far_p)}      ${fmt(r.far_p)}\n` +
        `Near P          ${fmt(s.near_p)}      ${fmt(r.near_p)}\n` +
        `Hold P          ${fmt(s.hold_p)}      ${fmt(r.hold_p)}\n` +
        `Start PWM L/R   ${fmt(s.start_left)}/${fmt(s.start_right)}   ${fmt(r.start_left)}/${fmt(r.start_right)}\n` +
        `Hold PWM L/R    ${fmt(s.hold_left)}/${fmt(s.hold_right)}   ${fmt(r.hold_left)}/${fmt(r.hold_right)}\n` +
        `Deadband        ${fmt(s.deadband)}      ${fmt(r.deadband)}\n` +
        `PWM slew        ${fmt(r.pwm_slew)} %/s\n` +
        `Feed forward    ${s.ff ? "ON" : "OFF"} (not auto-enabled)`;
    }

    const dev = d.deviation || {};
    const b = d.baseline || {};
    if (dev.comparable) {
      $("char-dev").textContent =
        `Minimum PWM Left   baseline ${pct(b.min_start_left)}  current ${pct(c.min_start_left)}  CHANGE ${dev.min_pwm_left}%\n` +
        `Minimum PWM Right  baseline ${pct(b.min_start_right)}  current ${pct(c.min_start_right)}  CHANGE ${dev.min_pwm_right}%\n` +
        `Max velocity Left  baseline ${b.max_vel_left}  current ${c.max_vel_left}  CHANGE ${dev.max_vel_left}%\n` +
        `Max velocity Right baseline ${b.max_vel_right}  current ${c.max_vel_right}  CHANGE ${dev.max_vel_right}%\n` +
        `Hold PWM Left      CHANGE ${dev.hold_left}%\n` +
        `Hold PWM Right     CHANGE ${dev.hold_right}%\n` +
        `(Performance Deviation — not a specific failure diagnosis)`;
    } else {
      $("char-dev").textContent = b.valid
        ? "Current result is not comparable yet."
        : "No baseline saved. Run a characterization, then Save as Baseline.";
    }
    drawGraphs(d);
  }

  function fmt(v) {
    const n = Number(v);
    return Number.isFinite(n) ? n.toFixed(2).padStart(6) : "    --";
  }

  function lineChart(canvas, series, xLabel, yLabel) {
    if (!canvas) return;
    const wrap = canvas.parentElement;
    const dpr = window.devicePixelRatio || 1;
    const w = wrap.clientWidth, h = wrap.clientHeight;
    canvas.width = (w * dpr) | 0;
    canvas.height = (h * dpr) | 0;
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#07090e";
    ctx.fillRect(0, 0, w, h);
    const pad = { l: 36, r: 10, t: 8, b: 20 };
    const gw = w - pad.l - pad.r, gh = h - pad.t - pad.b;
    let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
    series.forEach((s) => s.pts.forEach((p) => {
      xmin = Math.min(xmin, p[0]); xmax = Math.max(xmax, p[0]);
      ymin = Math.min(ymin, p[1]); ymax = Math.max(ymax, p[1]);
    }));
    if (!(xmax > xmin)) { xmin = 0; xmax = 1; }
    if (!(ymax > ymin)) { ymin = 0; ymax = 1; }
    const ypad = (ymax - ymin) * 0.1 || 1;
    ymin -= ypad; ymax += ypad;
    const xOf = (x) => pad.l + gw * (x - xmin) / (xmax - xmin);
    const yOf = (y) => pad.t + gh * (1 - (y - ymin) / (ymax - ymin));
    ctx.strokeStyle = "#262f45"; ctx.strokeRect(pad.l, pad.t, gw, gh);
    ctx.fillStyle = "#8592ab"; ctx.font = "10px -apple-system,sans-serif";
    ctx.fillText(yLabel, 4, 12);
    ctx.fillText(xLabel, w / 2 - 20, h - 4);
    series.forEach((s) => {
      ctx.strokeStyle = s.color;
      ctx.setLineDash(s.dash || []);
      ctx.beginPath();
      s.pts.forEach((p, i) => {
        const x = xOf(p[0]), y = yOf(p[1]);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = s.color;
      ctx.fillText(s.name, pad.l + 6, pad.t + 12 + series.indexOf(s) * 12);
    });
  }

  function drawGraphs(d) {
    const c = d.current || {};
    const b = d.baseline || {};
    const pwm = c.pwm || [];
    const vl = c.vel_left || [];
    const vr = c.vel_right || [];
    const s1 = [];
    if (pwm.length) {
      s1.push({ name: "Left", color: "#6cb6ff", pts: pwm.map((p, i) => [Number(p), Math.abs(Number(vl[i] || 0))]) });
      s1.push({ name: "Right", color: "#f778ba", pts: pwm.map((p, i) => [Number(p), Math.abs(Number(vr[i] || 0))]) });
    }
    if (b.valid && (b.pwm || []).length) {
      s1.push({ name: "Baseline L", color: "#6cb6ff", dash: [4, 3],
        pts: b.pwm.map((p, i) => [Number(p), Math.abs(Number((b.vel_left || [])[i] || 0))]) });
      s1.push({ name: "Baseline R", color: "#f778ba", dash: [4, 3],
        pts: b.pwm.map((p, i) => [Number(p), Math.abs(Number((b.vel_right || [])[i] || 0))]) });
    }
    lineChart($("char-g1"), s1, "PWM %", "Velocity %/s");

    const samp = (typeof SDiag !== "undefined" && SDiag.samples) ? SDiag.samples() : null;
    const pos = [], cmd = [];
    if (samp && samp.count) {
      for (let n = 0; n < samp.count; n++) {
        const i = samp.idxAt(n);
        const t = (samp.t_us[i] - samp.t0) / 1e6;
        cmd.push([t, samp.buf.sp[i]]);
        pos.push([t, samp.buf.filt[i]]);
      }
    }
    lineChart($("char-g2"), [
      { name: "Command", color: "#6cb6ff", pts: cmd },
      { name: "Actual", color: "#3dd68c", pts: pos },
    ], "Time s", "Position %");

    const errP = [];
    if (samp && samp.count) {
      for (let n = 0; n < samp.count; n++) {
        const i = samp.idxAt(n);
        errP.push([samp.buf.pwm[i], samp.buf.err[i]]);
      }
    }
    lineChart($("char-g3"), [{ name: "PWM vs error", color: "#e3b341", pts: errP }], "PWM %", "Error %");

    const rev = [];
    if (samp && samp.count) {
      for (let n = 0; n < samp.count; n++) {
        const i = samp.idxAt(n);
        const t = (samp.t_us[i] - samp.t0) / 1e6;
        rev.push([t, samp.buf.filt[i]]);
      }
    }
    lineChart($("char-g4"), [{ name: "Position (reversal)", color: "#ffa657", pts: rev }], "Time s", "Position %");
  }

  async function poll() {
    if (!$("page-cal") || !$("page-cal").classList.contains("active")) return;
    try { render(await api("/api/cal/char/status")); } catch (e) {}
  }

  async function loadHistory() {
    const d = await api("/api/cal/char/history");
    const el = $("char-hist");
    const recs = d.records || [];
    if (!recs.length) { el.textContent = "No saved records."; return; }
    el.innerHTML = recs.map((r) =>
      `<div class="card" style="margin-bottom:6px">
        <label>${r.status} · ${r.fw_version || ""} · t=${r.timestamp || 0}</label>
        <b>L ${Number(r.min_start_left).toFixed(1)}% / R ${Number(r.min_start_right).toFixed(1)}%
        · vmax ${Number(r.max_vel_left).toFixed(2)}/${Number(r.max_vel_right).toFixed(2)} %/s</b>
        ${r.fail_reason ? `<div class="hint">${r.fail_reason}</div>` : ""}
      </div>`).join("");
  }

  function downloadCanvasesPng() {
    ["char-g1", "char-g2", "char-g3", "char-g4"].forEach((id, n) => {
      const c = $(id);
      if (c && SDiag.pngDownload) SDiag.pngDownload(c, `steering_char_graph${n + 1}.png`);
    });
  }

  async function downloadReport() {
    const sys = await api("/api/system");
    const d = last || await api("/api/cal/char/status");
    const imgs = ["char-g1", "char-g2", "char-g3", "char-g4"].map((id) => {
      try { return $(id).toDataURL("image/png"); } catch (e) { return ""; }
    });
    const html = `<!DOCTYPE html><html><head><meta charset="utf-8"><title>Steering Characterization</title>
      <style>body{font:14px/1.4 -apple-system,sans-serif;max-width:800px;margin:24px auto;color:#111}
      td,th{text-align:left;padding:4px 8px;border-bottom:1px solid #ddd} img{max-width:100%}</style></head><body>
      <h1>Steering Actuator Characterization</h1>
      <p>Date ${new Date().toISOString()} · Firmware ${sys.fw_version} · Schema via VCM</p>
      <p>Status: ${(d.current && d.current.status) || ""} ${(d.current && d.current.fail_reason) || ""}</p>
      <pre>${($("char-recs") && $("char-recs").textContent) || ""}</pre>
      <pre>${($("char-dev") && $("char-dev").textContent) || ""}</pre>
      <h2>Graphs</h2>
      ${imgs.map((s, i) => s ? `<p><img alt="graph ${i + 1}" src="${s}"></p>` : "").join("")}
      <p>Raw data: download CSV/JSON from the vehicle dashboard. This report is self-contained HTML.</p>
      </body></html>`;
    const blob = new Blob([html], { type: "text/html" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = "steering_characterization_report.html";
    a.click();
  }

  function init() {
    $("char-start").onclick = async () => {
      if (!$("char-ack").checked) {
        alert("Tick the safety confirmation first.");
        return;
      }
      if (!confirm("The steering actuator will move automatically. Continue?")) return;
      const r = await post("/api/cal/char/start", {
        confirm: true, ack: "ACTUATOR WILL MOVE" });
      if (!r.ok) alert(r.error || "Could not start");
    };
    $("char-abort").onclick = () => post("/api/cal/char/abort");
    $("char-apply").onclick = async () => {
      if (!confirm("Apply recommended settings to RAM? Use SAVE in Configuration to persist.")) return;
      const r = await post("/api/cal/char/apply-recommended");
      alert(r.ok ? "Applied (RAM). SAVE to persist. Feed-forward stays OFF." : (r.error || "Failed"));
    };
    $("char-keep").onclick = () => alert("Keeping current settings.");
    $("char-baseline").onclick = async () => {
      const r = await post("/api/cal/char/save-baseline");
      alert(r.ok ? "Saved as baseline." : (r.error || "Failed"));
    };
    $("char-png").onclick = downloadCanvasesPng;
    $("char-report").onclick = downloadReport;
    $("char-hist-refresh").onclick = loadHistory;
    setInterval(poll, 700);
  }

  return { init, poll, loadHistory };
})();
