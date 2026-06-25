# -*- coding: utf-8 -*-
"""Copy the real SR6 STLs into docs/assets/parts with clean names + write rig.json
(servo positions, arm/link local pivots, kinematics constants) for the JS viewer."""
import trimesh, numpy as np, json, os, shutil

STL="SR6 完整资料进阶版本 签收后提供解压密码/STLs/SR6测试版零件/"
SRC={"base":"SR6 底座 Beta1A.stl","Lframe":"SR6 L形框架 Beta1.stl","Rframe":"SR6 R-Frame Beta1.stl",
 "recv":"SR6 Receiver Beta1.stl","arm":"SR6 臂 Beta1.stl","Lpitch":"SR6 L-投手 Beta1.stl",
 "Rpitch":"SR6 R-投手 Beta1.stl","mlink":"SR6 轴承主连杆 Beta1.stl","plink":"SR6 轴承投手链接 Beta1.stl"}
OUT=os.path.expanduser("~/repos/Dao-3D-Modeling-Agent/docs/assets/parts")
os.makedirs(OUT,exist_ok=True)

# convert each STL to binary STL with ascii name (model loads faster, no unicode in URL)
meta={}
for key,fn in SRC.items():
    m=trimesh.load(STL+fn,force="mesh")
    m.export(os.path.join(OUT,key+".stl"))
    b=m.bounds
    meta[key]={"bounds":b.tolist(),"nfaces":int(len(m.faces))}

# link local eye endpoints (along X extremes, mid Y/Z)
for key in ("mlink","plink"):
    b=np.array(meta[key]["bounds"])
    yc=(b[0,1]+b[1,1])/2; zc=(b[0,2]+b[1,2])/2
    meta[key]["eye0"]=[float(b[0,0]),float(yc),float(zc)]
    meta[key]["eye1"]=[float(b[1,0]),float(yc),float(zc)]

rig={
 "ROD":175.0,"HOME_H":200.0,
 "servo":{
   "L_mainA":[-60,30,22],"L_mainB":[-60,-30,22],"R_mainA":[60,30,22],"R_mainB":[60,-30,22],
   "L_pitch":[-60,0,33],"R_pitch":[60,0,33]},
 "armlen":{"L_mainA":50,"L_mainB":50,"R_mainA":50,"R_mainB":50,"L_pitch":75,"R_pitch":75},
 "blocal":{
   "L_mainA":[-59.98,0,0],"L_mainB":[-59.98,0,0],"R_mainA":[59.98,0,0],"R_mainB":[59.98,0,0],
   "L_pitch":[-61,-14.235,53.126],"R_pitch":[61,-14.235,53.126]},
 "arm_pivot_main":[67.5,0,51],"arm_ball_main":[67.5,50,51],
 "arm_pivot_pitch":[-7.5,30,51],"arm_ball_pitch":[-39.74,97.72,51],
 "mesh_of_leg":{"L_mainA":"arm","L_mainB":"arm","R_mainA":"arm","R_mainB":"arm",
                "L_pitch":"Lpitch","R_pitch":"Rpitch"},
 # frame placement: translate so main shafts land at x=+/-60 ; R mirrored
 "Lframe_dx":float(-60.0-(np.array(meta['Lframe']['bounds'])[0,0]+4.0)),
 "Rframe_dx":float( 60.0-(np.array(meta['Rframe']['bounds'])[1,0]-4.0)),
 "meta":meta,
}
json.dump(rig,open(os.path.join(OUT,"rig.json"),"w"),indent=2)
print("wrote",len(SRC),"STLs + rig.json to",OUT)
for k in SRC: print("  ",k,meta[k]["nfaces"],"faces")
