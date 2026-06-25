// SR6 real-part interactive mechanism.
// Loads the real STL parts and animates them with the frame-anchored kinematics
// (servos from the real frame geometry; 6 rigid links span exactly 175 mm).
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const PARTS_URL = "assets/parts/";
const LEGS = ["L_mainA","L_mainB","R_mainA","R_mainB","L_pitch","R_pitch"];

const V = (a) => new THREE.Vector3(a[0], a[1], a[2]);

// ---- kinematics ------------------------------------------------------------
let RIG = null;
const prevTh = {};            // branch continuity per leg

function eulerR(rollDeg, pitchDeg) {
  // R = Ry(roll) * Rx(pitch)  (yaw=0); roll about Y, pitch about X (matches python)
  const ro = rollDeg * Math.PI/180, pi = pitchDeg * Math.PI/180;
  const cx=Math.cos(pi), sx=Math.sin(pi), cy=Math.cos(ro), sy=Math.sin(ro);
  const Rx=new THREE.Matrix3().set(1,0,0, 0,cx,-sx, 0,sx,cx);
  const Ry=new THREE.Matrix3().set(cy,0,sy, 0,1,0, -sy,0,cy);
  return mul3(Ry, Rx);
}
function mul3(a,b){ const r=new THREE.Matrix3(); const ae=a.elements,be=b.elements,re=r.elements;
  // column-major 3x3
  for(let c=0;c<3;c++)for(let row=0;row<3;row++){let s=0;for(let k=0;k<3;k++)s+=ae[k*3+row]*be[c*3+k];re[c*3+row]=s;}
  return r;
}
function applyM3(m,v){ return v.clone().applyMatrix3(m); }

function legState(pose){
  // pose = {thrust,fwd,side,roll,pitch}
  const R = eulerR(pose.roll, pose.pitch);
  const t = new THREE.Vector3(pose.side, pose.fwd, RIG.HOME_H + pose.thrust);
  const ROD = RIG.ROD;
  const out = {R, t, legs:{}, reachable:true};
  for(const k of LEGS){
    const o = V(RIG.servo[k]); const L = RIG.armlen[k];
    const piv = applyM3(R, V(RIG.blocal[k])).add(t);
    const h = piv.x - o.x;                       // out-of-plane (axis = X)
    const dy = piv.y - o.y, dz = piv.z - o.z;
    const rho = Math.hypot(dy, dz);
    const d2dsq = ROD*ROD - h*h;
    let th=null, reachable=true;
    if(d2dsq<=0){ reachable=false; }
    else{
      const d2d=Math.sqrt(d2dsq);
      const cosd=(L*L+rho*rho-d2d*d2d)/(2*L*rho);
      if(Math.abs(cosd)>1){ reachable=false; }
      else{
        const base=Math.atan2(dz,dy), d=Math.acos(cosd);
        const cands=[base+d, base-d];
        const pv = (prevTh[k]!==undefined)?prevTh[k]:Math.PI/2;
        th = cands.reduce((a,b)=> Math.abs(Math.atan2(Math.sin(a-pv),Math.cos(a-pv))) <=
                                  Math.abs(Math.atan2(Math.sin(b-pv),Math.cos(b-pv))) ? a:b);
        prevTh[k]=th;
      }
    }
    if(!reachable){ out.reachable=false; }
    const ball = (th===null)? o.clone() : new THREE.Vector3(o.x, o.y+L*Math.cos(th), o.z+L*Math.sin(th));
    const len = ball.distanceTo(piv);
    out.legs[k]={o, piv, ball, th, reachable, len, h:Math.abs(h)};
  }
  return out;
}

// rigid transform mapping local point a->A and dir (b-a)->(B-A); returns Matrix4
function alignMatrix(la, lb, wa, wb){
  const v1=lb.clone().sub(la).normalize();
  const v2=wb.clone().sub(wa).normalize();
  const q=new THREE.Quaternion().setFromUnitVectors(v1,v2);
  const m=new THREE.Matrix4().makeRotationFromQuaternion(q);
  // translation: wa - R*la
  const Rla=la.clone().applyMatrix4(m);
  m.setPosition(wa.x-Rla.x, wa.y-Rla.y, wa.z-Rla.z);
  return m;
}

// ---- scene -----------------------------------------------------------------
const vp = document.getElementById("viewport");
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
vp.appendChild(renderer.domElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x0d1117);
const camera = new THREE.PerspectiveCamera(45, 1, 1, 5000);
camera.up.set(0,0,1); camera.position.set(330, -360, 330);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0,0,120); controls.enableDamping=true;

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const d1=new THREE.DirectionalLight(0xffffff,0.8); d1.position.set(1,-1.2,1.4); scene.add(d1);
const d2=new THREE.DirectionalLight(0xffffff,0.35); d2.position.set(-1,0.6,0.4); scene.add(d2);
const grid=new THREE.GridHelper(600,12,0x30363d,0x21262d); grid.rotation.x=Math.PI/2; scene.add(grid);

const MAT = {
  frame: new THREE.MeshStandardMaterial({color:0xc0392b, metalness:.1, roughness:.7}),
  base:  new THREE.MeshStandardMaterial({color:0x8e2b22, metalness:.1, roughness:.8}),
  arm:   new THREE.MeshStandardMaterial({color:0xe9e9ee, metalness:.2, roughness:.5}),
  mlink: new THREE.MeshStandardMaterial({color:0xd23b2f, metalness:.2, roughness:.5}),
  plink: new THREE.MeshStandardMaterial({color:0xe07a3c, metalness:.2, roughness:.5}),
  recv:  new THREE.MeshStandardMaterial({color:0xb83228, metalness:.1, roughness:.7}),
};

const loader = new STLLoader();
function loadSTL(name){
  return new Promise((res,rej)=> loader.load(PARTS_URL+name+".stl", g=>{g.computeVertexNormals();res(g);}, undefined, rej));
}

const dyn = {}; // dynamic meshes by leg + receiver
let geomCache = {};

async function build(){
  RIG = await (await fetch(PARTS_URL+"rig.json")).json();
  const names=["base","Lframe","Rframe","recv","arm","Lpitch","Rpitch","mlink","plink"];
  const gs = await Promise.all(names.map(loadSTL));
  names.forEach((n,i)=> geomCache[n]=gs[i]);

  // static frames + base
  const Lf=new THREE.Mesh(geomCache.Lframe, MAT.frame); Lf.position.x=RIG.Lframe_dx; scene.add(Lf);
  const Rf=new THREE.Mesh(geomCache.Rframe, MAT.frame); Rf.position.x=RIG.Rframe_dx; scene.add(Rf);
  const baseB=new THREE.Box3().setFromObject(new THREE.Mesh(geomCache.base));
  const bmesh=new THREE.Mesh(geomCache.base, MAT.base);
  // drop base under frames
  const lfB=new THREE.Box3().setFromObject(Lf);
  bmesh.position.set(-(baseB.max.x+baseB.min.x)/2, -(baseB.max.y+baseB.min.y)/2, lfB.min.z - baseB.max.z);
  scene.add(bmesh);

  // dynamic arms + links
  for(const k of LEGS){
    const isP = k.includes("pitch");
    const arm = new THREE.Mesh(isP? (k[0]==="L"?geomCache.Lpitch:geomCache.Rpitch) : geomCache.arm, MAT.arm);
    arm.matrixAutoUpdate=false; scene.add(arm);
    const link = new THREE.Mesh(isP?geomCache.plink:geomCache.mlink, isP?MAT.plink:MAT.mlink);
    link.matrixAutoUpdate=false; scene.add(link);
    dyn[k]={arm, link};
  }
  dyn.recv = new THREE.Mesh(geomCache.recv, MAT.recv); dyn.recv.matrixAutoUpdate=false; scene.add(dyn.recv);

  resize(); update(currentPose()); animate();
}

function currentPose(){
  return {thrust:+S.thrust.value, fwd:+S.fwd.value, side:+S.side.value, roll:+S.roll.value, pitch:+S.pitch.value};
}

function update(pose){
  const st = legState(pose);
  // receiver
  const Rm=new THREE.Matrix4().setFromMatrix3(st.R); Rm.setPosition(st.t.x,st.t.y,st.t.z);
  dyn.recv.matrix.copy(Rm);
  let worstRod=0, worstH=0;
  const rows=[];
  for(const k of LEGS){
    const ls=st.legs[k]; const isP=k.includes("pitch");
    const pivLocal = V(isP?RIG.arm_pivot_pitch:RIG.arm_pivot_main);
    const ballLocal= V(isP?RIG.arm_ball_pitch:RIG.arm_ball_main);
    dyn[k].arm.matrix.copy(alignMatrix(pivLocal, ballLocal, ls.o, ls.ball));
    const e0=V(RIG.meta[isP?"plink":"mlink"].eye0), e1=V(RIG.meta[isP?"plink":"mlink"].eye1);
    dyn[k].link.matrix.copy(alignMatrix(e0,e1, ls.ball, ls.piv));
    worstRod=Math.max(worstRod, Math.abs(ls.len-RIG.ROD)); worstH=Math.max(worstH, ls.h);
    rows.push(`<tr><td>${k}</td><td>${ls.len.toFixed(3)}</td><td>${ls.reachable?"✓":"✗"}</td><td>${ls.h.toFixed(1)}</td></tr>`);
  }
  document.getElementById("legtable").innerHTML=rows.join("");
  document.getElementById("m_rod").textContent=worstRod.toExponential(1)+" mm";
  document.getElementById("m_oop").textContent=worstH.toFixed(1)+" mm";
  const badge=document.getElementById("reach");
  badge.textContent = st.reachable? "可达 · 6 杆闭合 175" : "不可达 · 超出工作空间";
  badge.className = "badge "+(st.reachable?"ok":"bad");
}

function resize(){ const w=vp.clientWidth,h=vp.clientHeight; renderer.setSize(w,h); camera.aspect=w/h; camera.updateProjectionMatrix(); }
window.__sr6_resize=resize; addEventListener("resize",resize);
let demoT=null;
function animate(){ requestAnimationFrame(animate); controls.update(); renderer.render(scene,camera); }

// ---- sliders ---------------------------------------------------------------
const SPEC=[["thrust","推力 Z",-30,30,0],["fwd","前后 Y",-25,25,0],["side","左右 X",-25,25,0],
            ["roll","Roll°",-15,15,0],["pitch","Pitch°",-15,15,0]];
const S={};
const box=document.getElementById("sliders");
box.innerHTML=SPEC.map(([id,lab,mn,mx,v])=>`<div class="ctl"><label>${lab}<span id="lab_${id}">${v}</span></label>
  <input type="range" id="s_${id}" min="${mn}" max="${mx}" step="0.5" value="${v}"></div>`).join("");
SPEC.forEach(([id])=>{ S[id]=document.getElementById("s_"+id);
  S[id].oninput=()=>{ document.getElementById("lab_"+id).textContent=(+S[id].value).toFixed(1); update(currentPose()); }; });

function setPose(p){ for(const id in p){ S[id].value=p[id]; document.getElementById("lab_"+id).textContent=(+p[id]).toFixed(1);} update(currentPose()); }
const zero={thrust:0,fwd:0,side:0,roll:0,pitch:0};
document.getElementById("p_home").onclick=()=>setPose(zero);
document.getElementById("p_side").onclick=()=>setPose({...zero,side:20});
document.getElementById("p_pitch").onclick=()=>setPose({...zero,pitch:12});
document.getElementById("p_combo").onclick=()=>setPose({thrust:15,fwd:10,side:12,roll:8,pitch:-6});
document.getElementById("reset").onclick=()=>setPose(zero);
document.getElementById("demo").onclick=()=>{
  if(demoT){clearInterval(demoT);demoT=null;return;}
  let a=0; demoT=setInterval(()=>{ a+=0.04;
    setPose({thrust:18*Math.sin(a*0.8), fwd:16*Math.sin(a*0.6+1), side:18*Math.sin(a),
             roll:12*Math.sin(a*0.7), pitch:10*Math.sin(a*0.9+2)}); }, 40);
};

build().catch(e=>{ document.getElementById("reach").textContent="加载失败: "+e; console.error(e); });
