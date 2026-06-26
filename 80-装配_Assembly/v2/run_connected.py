"""run_connected.py -- gate the SNAPPED-link assembly with the dual critics and
emit a side-by-side proof panel (real photo | connected render)."""
import os,sys,math,numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from PIL import Image
HERE="C:/Users/Administrator/repos/Dao-3D-Modeling-Agent/80-装配_Assembly/v2"; sys.path.insert(0,HERE)
import perceptual_critic as pc
import critic
from assemble_connected import build
from render import _view_dirs

REF=os.path.join(HERE,"ref_machine.jpg")
def roty(v,d):
    a=math.radians(d);R=np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]]);return v@R.T

def fast_silhouette(parts,elev,azim,res=600,pad=0.06):
    cam,right,trueup=_view_dirs(elev,azim)
    allv=np.vstack([v for v,f,c in parts])
    polys=[];depths=[]
    cu_all=allv@right; cw_all=allv@trueup
    for v,f,c in parts:
        tris=v[f]; cen=tris.mean(1)
        u=tris@right; w=tris@trueup
        scr=np.stack([u,w],axis=-1)
        for i in range(len(tris)):
            polys.append(scr[i]); depths.append(cen[i]@cam)
    fig=plt.figure(figsize=(6,6),dpi=res/6); ax=fig.add_axes([0,0,1,1])
    pc_=PolyCollection(polys,facecolors="black",edgecolors="black",linewidths=0.3,antialiased=False)
    ax.add_collection(pc_)
    umid=(cu_all.min()+cu_all.max())/2; wmid=(cw_all.min()+cw_all.max())/2
    rng=max(cu_all.max()-cu_all.min(),cw_all.max()-cw_all.min())/2*(1+pad)
    ax.set_xlim(umid-rng,umid+rng); ax.set_ylim(wmid-rng,wmid+rng)
    ax.set_aspect("equal"); ax.axis("off"); fig.patch.set_facecolor("white")
    fig.canvas.draw()
    buf=np.frombuffer(fig.canvas.buffer_rgba(),dtype=np.uint8).reshape(fig.canvas.get_width_height()[::-1]+(4,))
    plt.close(fig)
    gray=buf[:,:,:3].mean(2)
    return (gray<128).astype(np.uint8)

VIEW=dict(elev=22,azim=-65)
parts=build(verbose=False)
ctr=np.vstack([v for v,f,c in parts]).mean(0)
ph=[(roty(v-ctr,-90),f,c) for v,f,c in parts]

ref_mask=pc.segment_reference(REF); ref_d=pc.descriptors(ref_mask)
con_mask=fast_silhouette(ph,**VIEW); con_d=pc.descriptors(con_mask)
print("REF :",ref_d)
print("CONN:",con_d)
verdict=pc.report("snapped-connected (horizontal)",con_d,ref_d)

# structural critic
names=["Base","L_Frame","R_Frame","Lid","PowerBus"]+["Arm"]*4+["L_Pitcher","R_Pitcher","Receiver"]+["Link"]*6
try:
    critic.critique(parts,names=names,verbose=True)
except Exception as e:
    print("structural critic note:",e)

# proof panel
ref_im=np.asarray(Image.open(REF).convert("RGB"))
fig,ax=plt.subplots(1,3,figsize=(18,6))
ax[0].imshow(ref_im); ax[0].set_title("REAL SR6 (photo)"); ax[0].axis("off")
ax[1].imshow(con_mask,cmap="gray_r"); ax[1].set_title(f"connected silhouette [{verdict}]"); ax[1].axis("off")
ax[2].imshow(ref_mask,cmap="gray_r"); ax[2].set_title("reference silhouette"); ax[2].axis("off")
fig.suptitle("perceptual gate: snapped-link assembly vs real machine")
fig.tight_layout(); fig.savefig(os.path.join(HERE,"connected_panel.png"),dpi=110,facecolor="white")
print("-> connected_panel.png  VERDICT",verdict)
