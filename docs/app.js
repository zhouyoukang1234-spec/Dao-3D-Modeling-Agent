import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import * as KIN from "./kin.js";

// ── scene ───────────────────────────────────────────────────────────────────
const host = document.getElementById("viewport");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
host.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0d1117);

const camera = new THREE.PerspectiveCamera(45, 1, 1, 5000);
camera.up.set(0, 0, 1); // Z up (model frame)
camera.position.set(320, -360, 300);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 130);
controls.enableDamping = true;

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.1);
key.position.set(200, -300, 400); scene.add(key);
const rim = new THREE.DirectionalLight(0x88aaff, 0.5);
rim.position.set(-250, 200, 150); scene.add(rim);

// ground grid in XY plane (Z up)
const grid = new THREE.GridHelper(600, 24, 0x30363d, 0x21262d);
grid.rotation.x = Math.PI / 2; scene.add(grid);

// ── geometry helpers ─────────────────────────────────────────────────────────
const UP_Y = new THREE.Vector3(0, 1, 0);
function tube(radius, color, opacity = 1) {
  const m = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, 1, 20),
    new THREE.MeshStandardMaterial({ color, metalness: 0.3, roughness: 0.5,
      transparent: opacity < 1, opacity }));
  m.matrixAutoUpdate = true;
  return m;
}
function setTube(mesh, p, q) {
  const a = new THREE.Vector3(...p), b = new THREE.Vector3(...q);
  const dir = new THREE.Vector3().subVectors(b, a);
  const len = dir.length();
  mesh.position.copy(a).addScaledVector(dir, 0.5);
  mesh.scale.set(1, Math.max(len, 1e-3), 1);
  mesh.quaternion.setFromUnitVectors(UP_Y, dir.clone().normalize());
}
function ball(radius, color) {
  return new THREE.Mesh(new THREE.SphereGeometry(radius, 20, 16),
    new THREE.MeshStandardMaterial({ color, metalness: 0.4, roughness: 0.35 }));
}

const COL_MAIN = 0x4ea1ff, COL_PITCH = 0xffb000, COL_ARM = 0xc9d1d9,
      COL_SERVO = 0x6e7681, COL_BALL = 0xff5d5d, COL_REC = 0x2ea043;

// servos (static)
for (const s of KIN.SERVOS) {
  const O = KIN.SERVO_O[s];
  const body = new THREE.Mesh(new THREE.BoxGeometry(40, 20, 20),
    new THREE.MeshStandardMaterial({ color: COL_SERVO, metalness: 0.5, roughness: 0.6 }));
  body.position.set(O[0], O[1], O[2]);
  scene.add(body);
}

// per-leg dynamic meshes
const legMesh = {};
for (const s of KIN.SERVOS) {
  const isMain = KIN.MAIN_SERVOS.includes(s);
  const arm = tube(4, COL_ARM);
  const rod = tube(3, isMain ? COL_MAIN : COL_PITCH);
  const jTip = ball(5, COL_BALL);
  const jB = ball(5, COL_BALL);
  scene.add(arm, rod, jTip, jB);
  legMesh[s] = { arm, rod, jTip, jB };
}

// receiver platform (moving): sleeve + spokes to balls
const sleeve = tube(28, COL_REC, 0.35); scene.add(sleeve);
const spokes = {};
for (const s of KIN.SERVOS) { spokes[s] = tube(2.2, COL_REC, 0.8); scene.add(spokes[s]); }

// ── pose state ───────────────────────────────────────────────────────────────
const pose = { tx: 0, ty: 0, tz: KIN.HOME_H, roll: 0, pitch: 0, yaw: 0 };
let prevAngles = KIN.HOME_ANGLES;

function update() {
  const sol = KIN.solve(pose, prevAngles);
  const badge = document.getElementById("reach");
  if (!sol.reachable) {
    badge.textContent = "位姿不可达 (超出工作空间)";
    badge.className = "badge bad";
    return;
  }
  prevAngles = sol.angles;
  badge.textContent = "可达 · 刚体闭环";
  badge.className = "badge ok";

  // platform center world
  const c = [pose.tx, pose.ty, pose.tz];
  const zc = [pose.tx, pose.ty, pose.tz + 40];
  setTube(sleeve, [c[0], c[1], c[2] - 35], [zc[0], zc[1], zc[2]]);

  for (const s of KIN.SERVOS) {
    const L = sol.legs[s];
    setTube(legMesh[s].arm, L.O, L.tip);
    setTube(legMesh[s].rod, L.tip, L.B);
    legMesh[s].jTip.position.set(...L.tip);
    legMesh[s].jB.position.set(...L.B);
    setTube(spokes[s], c, L.B);
  }

  // readouts
  const rows = KIN.SERVOS.map(s => {
    const L = sol.legs[s];
    return `<tr><td>${s}</td><td>${L.rod.toFixed(3)}</td><td>${L.gap.toFixed(3)}</td><td>${L.h.toFixed(1)}</td></tr>`;
  }).join("");
  document.getElementById("legtable").innerHTML = rows;
  document.getElementById("m_rod").textContent = sol.worstRod.toExponential(1) + " mm";
  document.getElementById("m_gapm").textContent = sol.gapMain.toFixed(3) + " mm";
  document.getElementById("m_gapp").textContent = sol.gapPitch.toFixed(3) + " mm";
  document.getElementById("m_oop").textContent = Math.max(sol.oopMain, sol.oopPitch).toFixed(1) + " mm";
}

// ── sliders ───────────────────────────────────────────────────────────────────
const defs = [
  ["dz", "升降 thrust (mm)", -30, 30, 0, v => pose.tz = KIN.HOME_H + v],
  ["ty", "前后 fwd (mm)",   -20, 20, 0, v => pose.ty = v],
  ["tx", "横移 side (mm)",  -20, 20, 0, v => pose.tx = v],
  ["roll", "横滚 roll (°)", -15, 15, 0, v => pose.roll = v * Math.PI / 180],
  ["pitch", "俯仰 pitch (°)", -15, 15, 0, v => pose.pitch = v * Math.PI / 180],
];
const panel = document.getElementById("sliders");
const sliderEls = {};
for (const [id, label, mn, mx, val, fn] of defs) {
  const wrap = document.createElement("div"); wrap.className = "ctl";
  wrap.innerHTML = `<label>${label}<span id="v_${id}">${val}</span></label>`;
  const inp = document.createElement("input");
  inp.type = "range"; inp.min = mn; inp.max = mx; inp.step = 0.5; inp.value = val;
  inp.oninput = () => { fn(parseFloat(inp.value)); document.getElementById("v_" + id).textContent = inp.value; update(); };
  wrap.appendChild(inp); panel.appendChild(wrap); sliderEls[id] = { inp, fn };
}
function resetPose() {
  pose.tx = pose.ty = pose.roll = pose.pitch = pose.yaw = 0; pose.tz = KIN.HOME_H;
  for (const [id,,,,val] of defs) { sliderEls[id].inp.value = val; document.getElementById("v_"+id).textContent = val; }
  prevAngles = KIN.HOME_ANGLES; update();
}
document.getElementById("reset").onclick = resetPose;

// presets
const presets = {
  home:  { tx:0, ty:0, tz:KIN.HOME_H, roll:0, pitch:0 },
  side:  { tx:20, ty:0, tz:KIN.HOME_H, roll:0, pitch:0 },
  pitch: { tx:0, ty:0, tz:KIN.HOME_H, roll:0, pitch:15*Math.PI/180 },
  combo: { tx:15, ty:10, tz:208, roll:8*Math.PI/180, pitch:-6*Math.PI/180 },
};
for (const k of Object.keys(presets)) {
  document.getElementById("p_" + k).onclick = () => {
    Object.assign(pose, presets[k]); pose.yaw = 0;
    sliderEls.dz.inp.value = pose.tz - KIN.HOME_H; document.getElementById("v_dz").textContent = (pose.tz-KIN.HOME_H);
    sliderEls.ty.inp.value = pose.ty; document.getElementById("v_ty").textContent = pose.ty;
    sliderEls.tx.inp.value = pose.tx; document.getElementById("v_tx").textContent = pose.tx;
    sliderEls.roll.inp.value = Math.round(pose.roll*180/Math.PI); document.getElementById("v_roll").textContent = Math.round(pose.roll*180/Math.PI);
    sliderEls.pitch.inp.value = Math.round(pose.pitch*180/Math.PI); document.getElementById("v_pitch").textContent = Math.round(pose.pitch*180/Math.PI);
    prevAngles = KIN.HOME_ANGLES; update();
  };
}

// auto demo
let demo = false, t0 = 0;
document.getElementById("demo").onclick = (e) => {
  demo = !demo; e.target.textContent = demo ? "停止演示" : "自动演示";
  if (demo) t0 = performance.now();
};
function demoStep(now) {
  if (!demo) return;
  const t = (now - t0) / 1000;
  pose.tx = 18 * Math.sin(t * 0.7);
  pose.ty = 14 * Math.sin(t * 0.5 + 1);
  pose.tz = KIN.HOME_H + 18 * Math.sin(t * 0.4 + 2);
  pose.roll = 12 * Math.PI / 180 * Math.sin(t * 0.6 + 0.5);
  pose.pitch = 12 * Math.PI / 180 * Math.sin(t * 0.55);
  for (const id of ["dz","ty","tx","roll","pitch"]) {
    let v; if (id==="dz") v=pose.tz-KIN.HOME_H; else if (id==="roll") v=Math.round(pose.roll*180/Math.PI);
    else if (id==="pitch") v=Math.round(pose.pitch*180/Math.PI); else v=Math[id==="tx"?"round":"round"](pose[id]);
    sliderEls[id].inp.value = (id==="dz"||id==="roll"||id==="pitch")?v:pose[id].toFixed(1);
    document.getElementById("v_"+id).textContent = (id==="dz"||id==="roll"||id==="pitch")?v:pose[id].toFixed(1);
  }
  update();
}

// ── resize + loop ─────────────────────────────────────────────────────────────
function resize() {
  const w = host.clientWidth, h = host.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h; camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);
new ResizeObserver(resize).observe(host);

function loop(now) {
  demoStep(now);
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
}
resize(); update(); requestAnimationFrame(loop);

// expose for tab re-show resize
window.__sr6_resize = resize;
