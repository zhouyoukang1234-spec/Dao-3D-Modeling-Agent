// SR6 / ORS6 kinematics — faithful JS port of closed_loop/true_kinematics.py
// Servos in-plane with their receiver balls (main X=±60, pitch X=±61) => rods EXACTLY 175.
// All math mirrors the Python module so the web view shows the SAME closed-loop model.

export const ROD = 175.0;
export const HOME_H = 193.0;
const MAIN_Z = HOME_H - 162.48; // 30.52

// Servo shaft origins (world mm); all axes ‖ +X.
export const SERVO_O = {
  LowerLeft:  [-60.0,  15.0, MAIN_Z],
  UpperLeft:  [-60.0, -15.0, MAIN_Z],
  LowerRight: [ 60.0,  15.0, MAIN_Z],
  UpperRight: [ 60.0, -15.0, MAIN_Z],
  LeftPitch:  [-61.0,  -6.1, 69.4],
  RightPitch: [ 61.0,  -6.1, 69.4],
};

// Receiver ball joints in receiver-local frame (measured from Receiver STL through-bores).
export const B_LOCAL = {
  LowerLeft:  [-59.98,  0.0,    0.0],
  UpperLeft:  [-59.98,  0.0,    0.0],
  LowerRight: [ 59.98,  0.0,    0.0],
  UpperRight: [ 59.98,  0.0,    0.0],
  LeftPitch:  [-61.0,  -14.235, 53.126],
  RightPitch: [ 61.0,  -14.235, 53.126],
};

export const ARMLEN = {
  LowerLeft: 50, UpperLeft: 50, LowerRight: 50, UpperRight: 50,
  LeftPitch: 75, RightPitch: 75,
};

export const SERVOS = Object.keys(B_LOCAL);
export const MAIN_SERVOS = ["LowerLeft", "UpperLeft", "LowerRight", "UpperRight"];
export const PITCH_SERVOS = ["LeftPitch", "RightPitch"];

// ── tiny vec3 helpers ───────────────────────────────────────────────────────
const sub = (a, b) => [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
const add = (a, b) => [a[0]+b[0], a[1]+b[1], a[2]+b[2]];
const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);

// R = Rz(yaw) Ry(roll) Rx(pitch); platform X=side, Y=fwd, Z=up. (matches euler_R)
function eulerR(roll, pitch, yaw) {
  const cx = Math.cos(pitch), sx = Math.sin(pitch);
  const cy = Math.cos(roll),  sy = Math.sin(roll);
  const cz = Math.cos(yaw),   sz = Math.sin(yaw);
  // Rx
  const Rx = [[1,0,0],[0,cx,-sx],[0,sx,cx]];
  const Ry = [[cy,0,sy],[0,1,0],[-sy,0,cy]];
  const Rz = [[cz,-sz,0],[sz,cz,0],[0,0,1]];
  const mul = (A,B) => {
    const C = [[0,0,0],[0,0,0],[0,0,0]];
    for (let i=0;i<3;i++) for (let j=0;j<3;j++) for (let k=0;k<3;k++) C[i][j]+=A[i][k]*B[k][j];
    return C;
  };
  return mul(Rz, mul(Ry, Rx));
}
function applyR(R, v) {
  return [
    R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
    R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
    R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2],
  ];
}

// pose = {tx,ty,tz,roll,pitch,yaw} (mm, rad)
export function bWorld(servo, pose) {
  const R = eulerR(pose.roll, pose.pitch, pose.yaw);
  return add(applyR(R, B_LOCAL[servo]), [pose.tx, pose.ty, pose.tz]);
}

// Per-leg frame: axis a=+X => u0=(0,1,0), w0=(0,0,1). arm swings in YZ plane.
const A_AXIS = [1, 0, 0];
const U0 = [0, 1, 0];
const W0 = [0, 0, 1];

export function armTip(servo, theta) {
  const O = SERVO_O[servo], L = ARMLEN[servo];
  return [O[0], O[1] + L*Math.cos(theta), O[2] + L*Math.sin(theta)];
}

export function outOfPlane(servo, pose) {
  return dot(A_AXIS, sub(bWorld(servo, pose), SERVO_O[servo]));
}

// Geometric IK: solve arm angle so |armTip - B| == ROD. Returns theta (rad) or null.
export function legIK(servo, pose, prev) {
  const O = SERVO_O[servo], L = ARMLEN[servo];
  const B = bWorld(servo, pose);
  const rel = sub(B, O);
  const h = dot(A_AXIS, rel);
  if (ROD*ROD - h*h <= 0) return null;
  const d2d = Math.sqrt(ROD*ROD - h*h);
  const bu = dot(U0, rel), bw = dot(W0, rel);
  const rho = Math.hypot(bu, bw);
  if (rho < 1e-9) return null;
  let cosd = (L*L + rho*rho - d2d*d2d) / (2*L*rho);
  if (Math.abs(cosd) > 1.0) return null;
  cosd = Math.max(-1, Math.min(1, cosd));
  const base = Math.atan2(bw, bu);
  const delta = Math.acos(cosd);
  let cands = [base + delta, base - delta];
  if (prev != null) {
    cands.sort((s, t) =>
      Math.abs(Math.atan2(Math.sin(s-prev), Math.cos(s-prev))) -
      Math.abs(Math.atan2(Math.sin(t-prev), Math.cos(t-prev))));
  }
  return cands[0];
}

export function ikAll(pose, prev) {
  const out = {};
  for (const s of SERVOS) {
    const th = legIK(s, pose, prev ? prev[s] : null);
    if (th == null) return null;
    out[s] = th;
  }
  return out;
}

export const HOME_POSE = { tx:0, ty:0, tz:HOME_H, roll:0, pitch:0, yaw:0 };
export const HOME_ANGLES = ikAll(HOME_POSE, null);

// Full per-leg solve for rendering + readouts.
export function solve(pose, prev) {
  const angles = ikAll(pose, prev || HOME_ANGLES);
  if (!angles) return { reachable: false };
  const legs = {};
  let worstRod = 0, gapMain = 0, gapPitch = 0, oopMain = 0, oopPitch = 0;
  for (const s of SERVOS) {
    const O = SERVO_O[s];
    const tip = armTip(s, angles[s]);
    const B = bWorld(s, pose);
    const rod = norm(sub(tip, B));
    const h = outOfPlane(s, pose);
    const gap = Math.sqrt(ROD*ROD + h*h) - ROD;
    legs[s] = { O, tip, B, theta: angles[s], rod, h, gap };
    worstRod = Math.max(worstRod, Math.abs(rod - ROD));
    if (MAIN_SERVOS.includes(s)) { gapMain = Math.max(gapMain, gap); oopMain = Math.max(oopMain, Math.abs(h)); }
    else { gapPitch = Math.max(gapPitch, gap); oopPitch = Math.max(oopPitch, Math.abs(h)); }
  }
  return { reachable: true, angles, legs, worstRod, gapMain, gapPitch,
           gapMax: Math.max(gapMain, gapPitch), oopMain, oopPitch };
}
