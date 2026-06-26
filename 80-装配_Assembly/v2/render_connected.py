import os,sys,math,numpy as np
HERE="C:/Users/Administrator/repos/Dao-3D-Modeling-Agent/80-装配_Assembly/v2"; sys.path.insert(0,HERE)
from render import render_views
from assemble_connected import build
def roty(v,d):
    a=math.radians(d);R=np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]]);return v@R.T
parts=build(verbose=True)
ph=[(roty(v,-90),f,c) for v,f,c in parts]
render_views(ph,os.path.join(HERE,"connected_views.png"),title="snapped links (horizontal)",
             views=[("photo",22,-65),("photo2",18,-115),("top",80,-90)],figsize=(18,6))
print("-> connected_views.png")
