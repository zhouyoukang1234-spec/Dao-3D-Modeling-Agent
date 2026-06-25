# -*- coding: utf-8 -*-
import trimesh, numpy as np, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

glb=sys.argv[1] if len(sys.argv)>1 else "sr6_real_assembly.glb"
scene=trimesh.load(glb)
geoms=list(scene.geometry.values())
# gather faces with colors
all_tris=[]; all_col=[]; all_cen=[]; all_nrm=[]
for g in geoms:
    Tlist=scene.graph.geometry_nodes.get(g.metadata.get('name',''),None)
for name,g in scene.geometry.items():
    # transform to world
    for node in scene.graph.nodes_geometry:
        tr,gn=scene.graph[node]
        if gn==name:
            m=g.copy(); m.apply_transform(tr)
            fc=m.visual.face_colors[:, :3]/255.0
            all_tris.append(m.vertices[m.faces]); all_col.append(fc)
            all_cen.append(m.triangles_center); all_nrm.append(m.face_normals)
            break
tris=np.concatenate(all_tris); col=np.concatenate(all_col); cen=np.concatenate(all_cen); nrm=np.concatenate(all_nrm)
light=np.array([0.5,-0.6,0.7]); light/=np.linalg.norm(light)
sh=np.clip(nrm@light,0,1)[:,None]*0.75+0.25
fcol=np.clip(col*sh,0,1)
allv=tris.reshape(-1,3); ctr=(allv.min(0)+allv.max(0))/2; r=(allv.max(0)-allv.min(0)).max()/2

views=[(18,-72,"3/4 front (match cover)"),(8,-90,"front"),(18,-108,"3/4 other"),(80,-90,"top")]
fig=plt.figure(figsize=(20,6))
for i,(el,az,lbl) in enumerate(views):
    ax=fig.add_subplot(1,4,i+1,projection="3d")
    azr,elr=np.radians(az),np.radians(el)
    cam=np.array([np.cos(elr)*np.cos(azr),np.cos(elr)*np.sin(azr),np.sin(elr)])
    order=np.argsort(-(cen@cam))
    pc=Poly3DCollection(tris[order],facecolors=fcol[order],edgecolor=(0,0,0,0.12),linewidths=0.03)
    ax.add_collection3d(pc)
    ax.set_xlim(ctr[0]-r,ctr[0]+r);ax.set_ylim(ctr[1]-r,ctr[1]+r);ax.set_zlim(ctr[2]-r,ctr[2]+r)
    ax.view_init(elev=el,azim=az); ax.set_title(lbl,fontsize=10)
    ax.set_xlabel("X");ax.set_ylabel("Y");ax.set_zlabel("Z")
fig.suptitle("SR6 real-part assembly",fontsize=13); fig.tight_layout()
fig.savefig("assembly_render.png",dpi=95); print("wrote assembly_render.png")
