/* Shared display-unit helpers. Control math stays SI; the UI converts. */
"use strict";

const Units = (() => {
  const M_PER_IN = 0.0254;
  const M_PER_MI = 1609.344;
  const MPS_PER_MPH = 0.44704;
  const KG_PER_LB = 0.45359237;
  const MPS2_PER_FTPS2 = 0.3048;

  // 0 = IMPERIAL (default), 1 = METRIC — matches ui.units enum
  let mode = 0;

  function setMode(m) { mode = (m === 1 || m === "METRIC") ? 1 : 0; }
  function isImperial() { return mode === 0; }
  function name() { return isImperial() ? "IMPERIAL" : "METRIC"; }

  function spd(mps) {
    const v = Number(mps) || 0;
    return isImperial() ? v / MPS_PER_MPH : v;
  }
  function dist(m) {
    const v = Number(m) || 0;
    return isImperial() ? v / M_PER_MI : v;
  }
  function len(m) {
    const v = Number(m) || 0;
    return isImperial() ? v / M_PER_IN : v;
  }
  function mass(kg) {
    const v = Number(kg) || 0;
    return isImperial() ? v / KG_PER_LB : v;
  }
  function accel(mps2) {
    const v = Number(mps2) || 0;
    return isImperial() ? v / MPS2_PER_FTPS2 : v;
  }
  function temp(c) {
    const v = Number(c) || 0;
    return isImperial() ? v * 9 / 5 + 32 : v;
  }

  function spdLabel() { return isImperial() ? "mph" : "m/s"; }
  function distLabel() { return isImperial() ? "mi" : "m"; }
  function lenLabel() { return isImperial() ? "in" : "m"; }
  function massLabel() { return isImperial() ? "lb" : "kg"; }
  function accelLabel() { return isImperial() ? "ft/s²" : "m/s²"; }
  function tempLabel() { return isImperial() ? "°F" : "°C"; }

  function fmtSpd(mps, digits = 1) { return spd(mps).toFixed(digits); }
  function fmtDist(m, digits = 2) { return dist(m).toFixed(digits); }
  function fmtLen(m, digits = 2) { return len(m).toFixed(digits); }

  /** Map SI unit string from param metadata → display unit. */
  function displayUnit(si) {
    if (!isImperial()) return si || "";
    switch (si) {
      case "m": return "in";
      case "m/s": return "mph";
      case "m/s2": return "ft/s²";
      case "kg": return "lb";
      case "C": case "°C": return "°F";
      default: return si || "";
    }
  }

  /** SI → display for a config value with the given SI unit. */
  function fromSi(value, si) {
    if (!isImperial()) return value;
    switch (si) {
      case "m": return value / M_PER_IN;
      case "m/s": return value / MPS_PER_MPH;
      case "m/s2": return value / MPS2_PER_FTPS2;
      case "kg": return value / KG_PER_LB;
      default: return value;
    }
  }

  /** Display → SI for apply/save. */
  function toSi(value, si) {
    if (!isImperial()) return value;
    switch (si) {
      case "m": return value * M_PER_IN;
      case "m/s": return value * MPS_PER_MPH;
      case "m/s2": return value * MPS2_PER_FTPS2;
      case "kg": return value * KG_PER_LB;
      default: return value;
    }
  }

  function roundDisp(value, si) {
    if (si === "m" || si === "m/s" || si === "m/s2" || si === "kg") {
      return Math.round(value * 1000) / 1000;
    }
    return value;
  }

  function refreshLabels() {
    document.querySelectorAll("[data-u]").forEach((el) => {
      const kind = el.getAttribute("data-u");
      if (kind === "spd") el.textContent = spdLabel();
      else if (kind === "dist") el.textContent = distLabel();
      else if (kind === "len") el.textContent = lenLabel();
      else if (kind === "mass") el.textContent = massLabel();
      else if (kind === "accel") el.textContent = accelLabel();
      else if (kind === "temp") el.textContent = tempLabel();
    });
  }

  return {
    setMode, isImperial, name,
    spd, dist, len, mass, accel, temp,
    spdLabel, distLabel, lenLabel, massLabel, accelLabel, tempLabel,
    fmtSpd, fmtDist, fmtLen,
    displayUnit, fromSi, toSi, roundDisp, refreshLabels,
  };
})();
