import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

// ---------- math helpers (port of build_h.py) ----------
const V = (x,y,z)=>new THREE.Vector3(x,y,z);
function rotvecToMat(rv){
  const a=Math.hypot(rv[0],rv[1],rv[2]);
  const m=new THREE.Matrix4();
  if(a<1e-9) return m;
  const ax=new THREE.Vector3(rv[0]/a,rv[1]/a,rv[2]/a);
  m.makeRotationAxis(ax,a); return m;
}
function frameFrom(p0,p1,ax){
  const e1=p1.clone().sub(p0).normalize();
  const e3=ax.clone().sub(e1.clone().multiplyScalar(ax.dot(e1))).normalize();
  const e2=e3.clone().cross(e1);
  return new THREE.Matrix4().makeBasis(e1,e2,e3);
}
// rigid transform mapping (H,B,axL)->(S,Bw,axW)
function alignMat(H,B,axL,S,Bw,axW){
  const Rl=frameFrom(H,B,axL), Rw=frameFrom(S,Bw,axW);
  const R=Rw.multiply(Rl.transpose());
  const t=S.clone().sub(applyR(R,H));
  const m=R.clone(); m.setPosition(t); return m;
}
function applyR(m,v){ // rotation part only
  const e=m.elements;
  return new THREE.Vector3(
    e[0]*v.x+e[4]*v.y+e[8]*v.z,
    e[1]*v.x+e[5]*v.y+e[9]*v.z,
    e[2]*v.x+e[6]*v.y+e[10]*v.z);
}
// 2-bar IK: ball in plane through servo perpendicular to axis
function solveBall(servo,pivot,arm,rod,axis,branch){
  const ax=axis.clone().normalize();
  let u=ax.clone().cross(V(0,0,1));
  if(u.length()<1e-6) u=ax.clone().cross(V(0,1,0));
  u.normalize();
  const w=ax.clone().cross(u);
  const rel=pivot.clone().sub(servo);
  const dz=rel.dot(ax);
  const ir2=rod*rod-dz*dz; if(ir2<=0) return null;
  const ir=Math.sqrt(ir2);
  const px=rel.dot(u), py=rel.dot(w), L=Math.hypot(px,py);
  if(L>arm+ir||L<Math.abs(arm-ir)) return {ball:null,reach:false};
  const a=(arm*arm-ir*ir+L*L)/(2*L), h=Math.sqrt(Math.max(0,arm*arm-a*a));
  const mx=a*px/L, my=a*py/L, ex=-py/L, ey=px/L;
  const bx=mx+branch*h*ex, by=my+branch*h*ey;
  const ball=servo.clone().add(u.clone().multiplyScalar(bx)).add(w.clone().multiplyScalar(by));
  return {ball,reach:true};
}

// ---------- globals ----------
let RIG=null, scene, camera, renderer, controls, root;
const dyn={}; // dynamic part objects
const MAT={
  housing:new THREE.MeshStandardMaterial({color:0xd6291c,metalness:.05,roughness:.6,emissive:0x2a0805,side:THREE.DoubleSide}),
  housing2:new THREE.MeshStandardMaterial({color:0xc8221a,metalness:.05,roughness:.6,emissive:0x260704,side:THREE.DoubleSide}),
  receiver:new THREE.MeshStandardMaterial({color:0xee3b2a,metalness:.05,roughness:.55,emissive:0x300906,side:THREE.DoubleSide}),
  arm:new THREE.MeshStandardMaterial({color:0xf4f5f9,metalness:.1,roughness:.5,side:THREE.DoubleSide}),
  rod:new THREE.MeshStandardMaterial({color:0xe2e6ed,metalness:.25,roughness:.4}),
  sleeve:new THREE.MeshStandardMaterial({color:0x26262c,metalness:.3,roughness:.4,side:THREE.DoubleSide}),
};
const pose={stroke:0,surge:0,sway:0,roll:0,pitch:0};
let branchCache={};

const loader=new GLTFLoader();
function loadPart(name){
  return new Promise((res,rej)=>loader.load(`./models/${name}.glb`,g=>{
    let mesh=null; g.scene.traverse(o=>{if(o.isMesh&&!mesh)mesh=o;});
    const geo=mesh.geometry; geo.deleteAttribute('normal'); geo.computeVertexNormals();
    res(geo);
  },undefined,rej));
}

async function init(){
  RIG=await fetch('./models/rig.json').then(r=>r.json());
  const vp=document.getElementById('viewport');
  scene=new THREE.Scene(); scene.background=new THREE.Color(0x0d1117);
  camera=new THREE.PerspectiveCamera(42, vp.clientWidth/vp.clientHeight, 1, 5000);
  camera.position.set(430,300,540);
  renderer=new THREE.WebGLRenderer({antialias:true});
  renderer.setSize(vp.clientWidth,vp.clientHeight); renderer.setPixelRatio(Math.min(2,devicePixelRatio));
  vp.appendChild(renderer.domElement);
  controls=new OrbitControls(camera,renderer.domElement); controls.enableDamping=true;
  controls.target.set(0,120,0);
  scene.add(new THREE.HemisphereLight(0xffffff,0x2a2a34,1.5));
  scene.add(new THREE.AmbientLight(0xffffff,0.45));
  const d=new THREE.DirectionalLight(0xffffff,2.0); d.position.set(250,450,350); scene.add(d);
  const d2=new THREE.DirectionalLight(0xbcd4ff,.8); d2.position.set(-350,180,-220); scene.add(d2);
  const d3=new THREE.DirectionalLight(0xffffff,.7); d3.position.set(0,120,-400); scene.add(d3);
  // root: housing Z-up -> world Y-up
  root=new THREE.Group(); root.rotation.x=-Math.PI/2; scene.add(root);
  // grid
  const grid=new THREE.GridHelper(800,16,0x30363d,0x21262d); grid.position.y=0; scene.add(grid);

  // geometries
  const G={};
  for(const n of ['base','frameL','frameR','cover','receiver','arm','pitcherL','pitcherR'])
    G[n]=await loadPart(n);

  // static housing (native coords)
  add(root,G.base,MAT.housing,'base');
  add(root,G.frameL,MAT.housing2,'frameL');
  add(root,G.frameR,MAT.housing2,'frameR');
  add(root,G.cover,MAT.housing,'cover');
  // dynamic
  dyn.receiver=add(root,G.receiver,MAT.receiver,'receiver');
  dyn.arms={};
  for(const k of RIG.legs){
    const geo = k[0]==='P' ? (k==='PL'?G.pitcherL:G.pitcherR) : G.arm;
    dyn.arms[k]=add(root,geo,MAT.arm,'arm_'+k);
  }
  // rods (cylinders, rebuilt each frame)
  dyn.rods={};
  for(const k of RIG.legs){
    const m=new THREE.Mesh(new THREE.CylinderGeometry(3,3,1,16),MAT.rod);
    root.add(m); dyn.rods[k]=m;
  }
  // sleeve
  dyn.sleeve=new THREE.Mesh(new THREE.CylinderGeometry(24,27,120,40),MAT.sleeve);
  root.add(dyn.sleeve);

  buildUI();
  update();
  window.addEventListener('resize',onResize);
  animate();
  document.getElementById('loading')?.remove();
}
function add(parent,geo,mat,name){
  const m=new THREE.Mesh(geo,mat); m.name=name; parent.add(m); return m;
}

// receiver world matrix from pose
function receiverMat(){
  const rvHome=rotvecToMat(RIG.rec_home_rotvec);
  const tHome=RIG.rec_home_t;
  // extra rotations: pitch about X, roll about Y (housing axes)
  const Rp=new THREE.Matrix4().makeRotationX(pose.pitch*Math.PI/180);
  const Rr=new THREE.Matrix4().makeRotationY(pose.roll*Math.PI/180);
  const R=new THREE.Matrix4().multiplyMatrices(Rp,Rr).multiply(rvHome);
  R.setPosition(tHome[0]+pose.sway, tHome[1]+pose.surge, tHome[2]+pose.stroke);
  return R;
}
function pivotWorld(Rm, local){
  const p=new THREE.Vector3(local[0],local[1],local[2]).applyMatrix4(Rm);
  return p;
}

function update(){
  const Rm=receiverMat();
  dyn.receiver.matrixAutoUpdate=false; dyn.receiver.matrix.copy(Rm);
  // sleeve along receiver local Z
  const sMat=Rm.clone();
  const off=new THREE.Matrix4().makeTranslation(0,0,78); // up the cup axis
  dyn.sleeve.matrixAutoUpdate=false; dyn.sleeve.matrix.multiplyMatrices(Rm,off)
     .multiply(new THREE.Matrix4().makeRotationX(Math.PI/2));
  const rows=[];
  let allReach=true;
  for(const k of RIG.legs){
    const sv=new THREE.Vector3(...RIG.servo[k]);
    const piv=pivotWorld(Rm, RIG.rec_local[RIG.assign[k]]);
    const arm=RIG.armlen[k];
    const axis=V(Math.sign(sv.x)||1,0,0);
    // branch: pick higher-Z ball at home, cache
    if(branchCache[k]===undefined){
      let best=null;
      for(const br of [1,-1]){ const r=solveBall(sv,piv,arm,RIG.rod,axis,br);
        if(r&&r.reach&&(best===null||r.ball.z>best.b.z)) best={br,b:r.ball}; }
      branchCache[k]=best?best.br:1;
    }
    const r=solveBall(sv,piv,arm,RIG.rod,axis,branchCache[k]);
    let rodLen=RIG.rod, reach=r&&r.reach;
    const feat = k[0]==='P' ? (k==='PL'?RIG.pitchL_feat:RIG.pitchR_feat) : RIG.arm_feat;
    if(reach){
      const ball=r.ball; rodLen=ball.distanceTo(piv);
      // arm transform
      const H=new THREE.Vector3(...feat.H), B=new THREE.Vector3(...feat.B), axL=new THREE.Vector3(...feat.axis);
      const Am=alignMat(H,B,axL, sv,ball,axis);
      const o=dyn.arms[k]; o.matrixAutoUpdate=false; o.matrix.copy(Am);
      // rod cylinder ball->pivot
      placeCyl(dyn.rods[k],ball,piv,k[0]==='P'?2.6:3.0);
      dyn.rods[k].visible=true; dyn.arms[k].visible=true;
    } else { allReach=false; dyn.rods[k].visible=false; }
    rows.push([k,rodLen,reach]);
  }
  renderReadout(rows,allReach);
}
function placeCyl(mesh,a,b,rad){
  const dir=b.clone().sub(a); const len=dir.length();
  mesh.matrixAutoUpdate=false;
  const mid=a.clone().add(b).multiplyScalar(.5);
  const up=V(0,1,0); const q=new THREE.Quaternion().setFromUnitVectors(up,dir.clone().normalize());
  const m=new THREE.Matrix4().compose(mid,q,new THREE.Vector3(rad/3,len,rad/3));
  mesh.matrix.copy(m);
}

function renderReadout(rows,allReach){
  const el=document.getElementById('readout'); if(!el) return;
  let h=`<div class="badge ${allReach?'ok':'bad'}">${allReach?'工作空间内 · 6杆闭合':'超出工作空间'}</div>`;
  h+='<table class="rod"><tr><th>连杆</th><th>长度(mm)</th><th>状态</th></tr>';
  for(const [k,len,reach] of rows){
    const ok=Math.abs(len-175)<1e-3;
    h+=`<tr><td>${k}</td><td>${len.toFixed(3)}</td><td class="${reach?'g':'r'}">${reach?'✓ 175':'—'}</td></tr>`;
  }
  h+='</table>';
  el.innerHTML=h;
}

function buildUI(){
  const defs=[['stroke','冲程 Stroke',-55,55],['surge','前后 Surge',-35,35],
    ['sway','左右 Sway',-35,35],['roll','滚转 Roll°',-18,18],['pitch','俯仰 Pitch°',-18,18]];
  const box=document.getElementById('sliders'); box.innerHTML='';
  for(const [key,label,mn,mx] of defs){
    const w=document.createElement('div'); w.className='ctl';
    w.innerHTML=`<label>${label} <span id="v_${key}">0</span></label>
      <input type="range" id="s_${key}" min="${mn}" max="${mx}" step="0.5" value="0">`;
    box.appendChild(w);
    w.querySelector('input').addEventListener('input',e=>{
      pose[key]=parseFloat(e.target.value); document.getElementById('v_'+key).textContent=e.target.value;
      update();
    });
  }
  document.getElementById('btnHome').onclick=()=>{
    for(const k in pose) pose[k]=0;
    for(const [key] of defs){document.getElementById('s_'+key).value=0;document.getElementById('v_'+key).textContent='0';}
    update();
  };
  let demo=null;
  document.getElementById('btnDemo').onclick=(e)=>{
    if(demo){clearInterval(demo);demo=null;e.target.textContent='▶ 自动演示';return;}
    e.target.textContent='⏸ 停止'; let t=0;
    demo=setInterval(()=>{ t+=0.05;
      pose.stroke=40*Math.sin(t); pose.surge=18*Math.sin(t*0.7);
      pose.pitch=12*Math.sin(t*0.9); pose.roll=10*Math.sin(t*1.3); pose.sway=18*Math.sin(t*1.1);
      for(const key of ['stroke','surge','sway','roll','pitch']){
        const s=document.getElementById('s_'+key); if(s){s.value=pose[key].toFixed(1);document.getElementById('v_'+key).textContent=pose[key].toFixed(1);}}
      update();
    },33);
  };
  document.getElementById('chkEnc').onchange=e=>{
    for(const n of ['base','frameL','frameR','cover']){
      const o=root.getObjectByName(n); if(o) o.visible=e.target.checked;
    }
  };
}
function onResize(){
  const vp=document.getElementById('viewport');
  camera.aspect=vp.clientWidth/vp.clientHeight; camera.updateProjectionMatrix();
  renderer.setSize(vp.clientWidth,vp.clientHeight);
}
function animate(){ requestAnimationFrame(animate); controls.update(); renderer.render(scene,camera); }

init().catch(err=>{console.error(err); const l=document.getElementById('loading'); if(l)l.textContent='加载失败: '+err;});
