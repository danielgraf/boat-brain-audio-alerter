"use strict";

// ---- helpers ----------------------------------------------------------
const $ = (id) => document.getElementById(id);
const rad = (d) => (d * Math.PI) / 180;
const fmtDeg = (v) => String(Math.round(((v % 360) + 360) % 360)).padStart(3, "0") + "°";

async function post(path, body) {
  await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
}
const cmd = (o) => post("/api/command", o);

// ---- control state ----------------------------------------------------
let mode = "heading";
let target = 0;         // shown target (heading or COG depending on mode)
let latest = null;      // last state snapshot

// ---- autopilot controls ----------------------------------------------
$("engage").onclick = () => cmd({ action: "engage" });
$("standby").onclick = () => cmd({ action: "standby" });
$("reset").onclick = () => cmd({ action: "reset" });

function setMode(m) {
  mode = m;
  $("mode-heading").classList.toggle("active", m === "heading");
  $("mode-cog").classList.toggle("active", m === "cog");
  $("target-label").textContent = m === "cog" ? "COG" : "HDG";
  cmd({ action: "mode", mode: m });
  // Seed the target from the current relevant value.
  if (latest) {
    target = m === "cog" ? latest.boat.cog : latest.boat.heading;
    cmd({ action: "set", value: target });
  }
}
$("mode-heading").onclick = () => setMode("heading");
$("mode-cog").onclick = () => setMode("cog");

document.querySelectorAll(".dial button[data-adj]").forEach((b) => {
  b.onclick = () => {
    target = (((target + Number(b.dataset.adj)) % 360) + 360) % 360;
    $("target-val").textContent = fmtDeg(target);
    cmd({ action: "adjust", value: Number(b.dataset.adj) });
  };
});

const zeta = $("zeta");
zeta.oninput = () => {
  $("zeta-out").textContent = Number(zeta.value).toFixed(2);
  cmd({ action: "tune", value: Number(zeta.value) });
};

// ---- environment controls --------------------------------------------
const stw = $("stw");
stw.oninput = () => { $("stw-o").textContent = Number(stw.value).toFixed(1) + " kn"; cmd({ action: "stw", value: Number(stw.value) }); };

const envInputs = ["windspd", "winddir", "waveamp", "waveper", "curspd", "curdir"].map($);
function pushEnv() {
  $("windspd-o").textContent = $("windspd").value + " kn";
  $("winddir-o").textContent = fmtDeg(+$("winddir").value);
  $("waveamp-o").textContent = Number($("waveamp").value).toFixed(1);
  $("waveper-o").textContent = Number($("waveper").value).toFixed(1) + " s";
  $("curspd-o").textContent = Number($("curspd").value).toFixed(1) + " kn";
  $("curdir-o").textContent = fmtDeg(+$("curdir").value);
  post("/api/environment", {
    wind_speed_kn: +$("windspd").value,
    wind_dir_deg: +$("winddir").value,
    wave_amp_dps2: +$("waveamp").value,
    wave_period_s: +$("waveper").value,
    current_speed_kn: +$("curspd").value,
    current_dir_deg: +$("curdir").value,
    leeway_coef: 0.03,
    weather_helm: 0.01,
  });
}
envInputs.forEach((el) => (el.oninput = pushEnv));

// ---- drawing: nav view -----------------------------------------------
const view = $("view").getContext("2d");
const VW = 520, VH = 520, cx = VW / 2, cy = VH / 2;
let scale = 4; // px per metre for the ground track

function arrow(ctx, ang, len, color, dashed, label) {
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(rad(ang));
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 3;
  if (dashed) ctx.setLineDash([7, 6]);
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(0, -len); ctx.stroke();
  ctx.setLineDash([]);
  ctx.beginPath(); ctx.moveTo(0, -len); ctx.lineTo(-6, -len + 12); ctx.lineTo(6, -len + 12); ctx.closePath(); ctx.fill();
  if (label) { ctx.rotate(-rad(ang)); ctx.font = "11px sans-serif"; ctx.textAlign = "center";
    const lx = Math.sin(rad(ang)) * (len + 12), ly = -Math.cos(rad(ang)) * (len + 12);
    ctx.fillText(label, lx, ly); }
  ctx.restore();
}

function drawBoat(ctx, headingDeg, rudderDeg) {
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(rad(headingDeg));
  // hull (bow up = -y)
  ctx.beginPath();
  ctx.moveTo(0, -46); ctx.quadraticCurveTo(16, -20, 15, 24);
  ctx.lineTo(-15, 24); ctx.quadraticCurveTo(-16, -20, 0, -46); ctx.closePath();
  ctx.fillStyle = "#26303e"; ctx.strokeStyle = "#5a6b82"; ctx.lineWidth = 2;
  ctx.fill(); ctx.stroke();
  // deck line
  ctx.beginPath(); ctx.moveTo(0, -40); ctx.lineTo(0, 20); ctx.strokeStyle = "#3c4a5e"; ctx.stroke();
  // rudder at stern, deflected (starboard positive -> kicks to starboard/right)
  ctx.save();
  ctx.translate(0, 24);
  ctx.rotate(rad(rudderDeg));
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(0, 16);
  ctx.strokeStyle = "#ff7a59"; ctx.lineWidth = 3.5; ctx.stroke();
  ctx.restore();
  ctx.restore();
}

function drawView(s) {
  view.clearRect(0, 0, VW, VH);
  // compass ring
  view.save();
  view.strokeStyle = "#233042"; view.lineWidth = 1;
  view.beginPath(); view.arc(cx, cy, 210, 0, 2 * Math.PI); view.stroke();
  view.fillStyle = "#5a6b82"; view.font = "12px sans-serif"; view.textAlign = "center";
  ["N", "E", "S", "W"].forEach((c, i) => {
    const a = rad(i * 90);
    view.fillText(c, cx + Math.sin(a) * 226, cy - Math.cos(a) * 226 + 4);
  });
  view.restore();

  // ground track trail (boat-centered)
  const b = s.boat;
  if (s.trail && s.trail.length > 1) {
    view.save();
    view.strokeStyle = "#56607a"; view.lineWidth = 2; view.beginPath();
    s.trail.forEach((p, i) => {
      const sx = cx + (p.x - b.x) * scale, sy = cy - (p.y - b.y) * scale;
      i ? view.lineTo(sx, sy) : view.moveTo(sx, sy);
    });
    view.stroke(); view.restore();
  }

  // direction arrows
  arrow(view, b.heading, 150, "#ffcc4a", false, "HDG");
  if (b.sog > 0.3) arrow(view, b.cog, 175, "#4aa3ff", true, "COG");
  const env = s.env;
  if (env.wind_speed_kn > 0) arrow(view, env.wind_dir_deg + 180, 120, "#7cc7ff", false, "wind");
  if (env.current_speed_kn > 0) arrow(view, env.current_dir_deg, 100, "#35d0ba", false, "cur");

  drawBoat(view, b.heading, b.rudder_deg);
}

// ---- drawing: helm (rudder / tiller / ram) ---------------------------
const helm = $("helm").getContext("2d");
const HW = 340, HH = 300;

function drawHelm(s) {
  helm.clearRect(0, 0, HW, HH);
  const b = s.boat, cal = s.cal || {};
  const px = HW / 2, py = 90;           // rudder pivot
  const tillerLen = 110;                 // tiller arm length (forward = down)
  const rudDeg = b.rudder_deg;
  const mountStbd = (s.mount_side === "starboard");

  // hull reference (stern block)
  helm.fillStyle = "#1a2230"; helm.fillRect(px - 120, py - 44, 240, 40);
  helm.fillStyle = "#5a6b82"; helm.font = "11px sans-serif"; helm.textAlign = "center";
  helm.fillText("stern", px, py - 50);

  // rudder blade (aft, up = -y in this schematic where forward is +y/down)
  helm.save();
  helm.translate(px, py);
  helm.rotate(rad(rudDeg));
  helm.strokeStyle = "#ff7a59"; helm.lineWidth = 6; helm.lineCap = "round";
  helm.beginPath(); helm.moveTo(0, 0); helm.lineTo(0, -60); helm.stroke();
  helm.restore();

  // tiller arm (forward from pivot, swings opposite the blade)
  const tillerAng = -rudDeg; // tiller forward end swings opposite to blade aft
  const tx = px + Math.sin(rad(tillerAng)) * tillerLen;
  const ty = py + Math.cos(rad(tillerAng)) * tillerLen;
  helm.strokeStyle = "#8b98a8"; helm.lineWidth = 5; helm.lineCap = "round";
  helm.beginPath(); helm.moveTo(px, py); helm.lineTo(tx, ty); helm.stroke();
  helm.fillStyle = "#c9d4e2"; helm.beginPath(); helm.arc(px, py, 5, 0, 2 * Math.PI); helm.fill();

  // ram: from a fixed mount on one side to the tiller end
  const mountX = mountStbd ? px + 130 : px - 130;
  const mountY = py + 70;
  helm.strokeStyle = "#35d0ba"; helm.lineWidth = 8; helm.lineCap = "round";
  helm.beginPath(); helm.moveTo(mountX, mountY); helm.lineTo(tx, ty); helm.stroke();
  // mount block
  helm.fillStyle = "#243244"; helm.fillRect(mountX - 10, mountY - 10, 20, 20);
  helm.fillStyle = "#5a6b82"; helm.fillText(mountStbd ? "ram (stbd)" : "ram (port)", mountX, mountY + 26);

  $("helmreadout").textContent =
    `rudder ${rudDeg.toFixed(1)}°   ram ${b.ram_percent.toFixed(0)}%   stroke ${b.ram_stroke.toFixed(2)}`;
}

// ---- instruments ------------------------------------------------------
function updateInstruments(s) {
  const t = s.tick, b = s.boat, p = s.pid;
  $("status").textContent = s.engaged ? "ENGAGED" : "STANDBY";
  $("status").className = "status " + (s.engaged ? "engaged" : "standby");
  $("i-hdg").textContent = fmtDeg(b.heading);
  $("i-cog").textContent = b.sog > 0.3 ? fmtDeg(b.cog) : "—";
  $("i-sog").textContent = b.sog.toFixed(1) + " kn";
  $("i-set").textContent = fmtDeg(t.heading_setpoint);
  $("i-err").textContent = t.error.toFixed(1) + "°";
  $("i-crab").textContent = (s.mode === "cog" ? t.crab.toFixed(1) + "°" : "—");
  $("i-rud").textContent = b.rudder_deg.toFixed(1) + "°";
  $("i-ram").textContent = b.ram_percent.toFixed(0) + "%";
  $("i-mode").textContent = s.mode === "cog" ? "COG track" : "Heading";
  $("i-reason").textContent = t.reason;
  $("i-kp").textContent = p.kp.toFixed(2);
  $("i-ki").textContent = p.ki.toFixed(3);
  $("i-kd").textContent = p.kd.toFixed(2);
  $("i-p").textContent = p.p.toFixed(1);
  $("i-i").textContent = p.i.toFixed(1);
  $("i-d").textContent = p.d.toFixed(1);
}

// ---- SSE stream -------------------------------------------------------
function connect() {
  const es = new EventSource("/api/stream");
  es.onmessage = (e) => {
    const s = JSON.parse(e.data);
    latest = s;
    // Keep the target readout in sync when idle.
    if (document.activeElement === document.body) {
      const shown = s.mode === "cog" ? s.tick.desired_cog : s.tick.desired_heading;
      $("target-val").textContent = fmtDeg(shown);
      target = shown;
    }
    drawView(s);
    drawHelm(s);
    updateInstruments(s);
  };
  es.onerror = () => { es.close(); setTimeout(connect, 1000); };
}
connect();
pushEnv();
