"""Headless perception (the ``view.*`` tool group).

Runs inside freecadcmd. Gives the agent *eyes* without a GUI: tessellates the
live shapes and renders shaded multi-view PNGs with matplotlib's Agg backend.
This powers the visual half of the perceive/verify loop and provides proof
images. Geometric perception (volumes, bbox, DoF) comes from ``solid.measure`` /
``param.measure`` / ``param.diagnose``; this module is the optical channel.
"""
import os

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

_VIEWS = {
    "iso": (28, -60), "front": (0, -90), "top": (89, -90),
    "right": (0, 0), "rear": (0, 90), "left": (0, 180),
}


def register(state):
    doc = state.doc

    def _collect(names, assembly=None):
        """Return list of (label, Shape) for named/visible shapes.

        When ``assembly`` is given, collect every component of that assembly at
        its *assembled* global placement (link placement composed with the source
        body placement) -- so a render reflects where parts actually sit, not the
        raw bodies stacked at the origin. ``names`` may also reference components
        directly (their lowercase add-name) for the same effect.
        """
        objs = []
        if assembly:
            asm = doc.getObject(assembly)
            if asm is not None:
                for o in doc.Objects:
                    if not o.isDerivedFrom("App::Link"):
                        continue
                    par = o.getParentGeoFeatureGroup()
                    if par is None or par.Name != asm.Name:
                        continue
                    lo = o.LinkedObject
                    if lo is None or not hasattr(lo, "Shape") or lo.Shape.isNull():
                        continue
                    shp = lo.Shape.copy()
                    shp.Placement = o.Placement.multiply(lo.Placement)
                    objs.append((o.Name, shp))
            return objs
        if names:
            for n in names:
                if n in state.shapes and doc.getObject(state.shapes[n]):
                    objs.append((n, doc.getObject(state.shapes[n]).Shape))
                elif n in state.bodies and doc.getObject(state.bodies[n]):
                    objs.append((n, doc.getObject(state.bodies[n]).Shape))
                elif n in state.components:
                    src = doc.getObject(state.components[n]["src"])
                    link = doc.getObject(state.components[n]["link"])
                    shp = src.Shape.copy()
                    shp.Placement = link.Placement.multiply(src.Placement)
                    objs.append((n, shp))
        else:
            for o in doc.Objects:
                if getattr(o, "Visibility", False) and hasattr(o, "Shape") \
                        and not o.Shape.isNull() and o.Shape.Solids:
                    objs.append((o.Label, o.Shape))
        return objs

    def _tris(shape, tol):
        verts, facets = shape.tessellate(tol)
        vp = np.array([[v.x, v.y, v.z] for v in verts], dtype=float)
        tri = np.array([vp[list(f)] for f in facets], dtype=float) if facets else np.zeros((0, 3, 3))
        return tri

    def op_render(a):
        names = a.get("names")
        view = a.get("view", "iso")
        tol = float(a.get("tolerance", 0.5))
        size = int(a.get("size", 700))
        path = a["path"]
        objs = _collect(names, a.get("assembly"))
        if not objs:
            return {"rendered": False, "reason": "no solids to render"}

        elev, azim = _VIEWS.get(view, _VIEWS["iso"])
        fig = plt.figure(figsize=(size / 100.0, size / 100.0), dpi=100)
        ax = fig.add_subplot(111, projection="3d")
        palette = ["#4f8cc9", "#c97f4f", "#69b36b", "#b36bb0", "#b0b36b"]
        allpts = []
        for idx, (label, shape) in enumerate(objs):
            tri = _tris(shape, tol)
            if len(tri) == 0:
                continue
            allpts.append(tri.reshape(-1, 3))
            coll = Poly3DCollection(tri, alpha=1.0)
            coll.set_facecolor(palette[idx % len(palette)])
            coll.set_edgecolor((0, 0, 0, 0.12))
            coll.set_linewidth(0.2)
            ax.add_collection3d(coll)
        pts = np.concatenate(allpts) if allpts else np.zeros((1, 3))
        mn, mx = pts.min(0), pts.max(0)
        ctr = (mn + mx) / 2.0
        rad = max((mx - mn).max() / 2.0, 1.0)
        ax.set_xlim(ctr[0] - rad, ctr[0] + rad)
        ax.set_ylim(ctr[1] - rad, ctr[1] + rad)
        ax.set_zlim(ctr[2] - rad, ctr[2] + rad)
        try:
            ax.set_box_aspect((1, 1, 1))
        except Exception:
            pass
        ax.view_init(elev=elev, azim=azim)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("%s  [%s]" % ("+".join(lbl for lbl, _ in objs), view), fontsize=9)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        fig.savefig(path, bbox_inches="tight")
        plt.close(fig)
        return {"rendered": True, "path": path, "view": view, "objects": [lbl for lbl, _ in objs],
                "bytes": os.path.getsize(path) if os.path.exists(path) else 0}

    def op_views(a):
        """Render a contact sheet of front/top/right/iso into one PNG."""
        names = a.get("names")
        tol = float(a.get("tolerance", 0.5))
        size = int(a.get("size", 900))
        path = a["path"]
        objs = _collect(names, a.get("assembly"))
        if not objs:
            return {"rendered": False, "reason": "no solids"}
        order = ["iso", "front", "top", "right"]
        fig = plt.figure(figsize=(size / 100.0, size / 100.0), dpi=100)
        palette = ["#4f8cc9", "#c97f4f", "#69b36b", "#b36bb0", "#b0b36b"]
        meshes = [(lbl, _tris(s, tol)) for lbl, s in objs]
        allpts = np.concatenate([t.reshape(-1, 3) for _, t in meshes if len(t)]) \
            if any(len(t) for _, t in meshes) else np.zeros((1, 3))
        mn, mx = allpts.min(0), allpts.max(0)
        ctr = (mn + mx) / 2.0
        rad = max((mx - mn).max() / 2.0, 1.0)
        for i, view in enumerate(order):
            ax = fig.add_subplot(2, 2, i + 1, projection="3d")
            for idx, (label, tri) in enumerate(meshes):
                if len(tri) == 0:
                    continue
                coll = Poly3DCollection(tri, alpha=1.0)
                coll.set_facecolor(palette[idx % len(palette)])
                coll.set_edgecolor((0, 0, 0, 0.12))
                coll.set_linewidth(0.2)
                ax.add_collection3d(coll)
            ax.set_xlim(ctr[0] - rad, ctr[0] + rad)
            ax.set_ylim(ctr[1] - rad, ctr[1] + rad)
            ax.set_zlim(ctr[2] - rad, ctr[2] + rad)
            try:
                ax.set_box_aspect((1, 1, 1))
            except Exception:
                pass
            elev, azim = _VIEWS[view]
            ax.view_init(elev=elev, azim=azim)
            ax.set_title(view, fontsize=9)
            ax.tick_params(labelsize=6)
        fig.suptitle("+".join(lbl for lbl, _ in objs), fontsize=11)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        fig.savefig(path, bbox_inches="tight")
        plt.close(fig)
        return {"rendered": True, "path": path, "views": order,
                "objects": [lbl for lbl, _ in objs],
                "bytes": os.path.getsize(path) if os.path.exists(path) else 0}

    def op_scene(a):
        """Dump the live model as tessellated triangle meshes for a 3D viewer.

        Returns one entry per solid (flat ``positions`` + triangle ``indices``,
        ready for a WebGL ``BufferGeometry``) plus an overall bounding box. This
        is the interactive optical channel behind the web workspace — the same
        live document the agent edits, streamed to the browser as real geometry.
        """
        names = a.get("names")
        tol = float(a.get("tolerance", 0.3))
        objs = _collect(names, a.get("assembly"))
        palette = ["#4f8cc9", "#c97f4f", "#69b36b", "#b36bb0", "#b0b36b", "#5fb0c9"]
        out = []
        gmn = [float("inf")] * 3
        gmx = [float("-inf")] * 3
        for idx, (label, shape) in enumerate(objs):
            verts, facets = shape.tessellate(tol)
            if not facets:
                continue
            positions = []
            for v in verts:
                positions.extend((round(v.x, 4), round(v.y, 4), round(v.z, 4)))
                for k, c in enumerate((v.x, v.y, v.z)):
                    gmn[k] = min(gmn[k], c)
                    gmx[k] = max(gmx[k], c)
            indices = []
            for f in facets:
                indices.extend(int(i) for i in f[:3])
            try:
                vol = round(float(shape.Volume), 3)
            except Exception:
                vol = None
            out.append({
                "name": label,
                "color": palette[idx % len(palette)],
                "positions": positions,
                "indices": indices,
                "volume": vol,
                "faces": len(shape.Faces),
                "triangles": len(facets),
            })
        if not out:
            return {"objects": [], "bbox": None, "empty": True}
        ctr = [(gmn[i] + gmx[i]) / 2.0 for i in range(3)]
        rad = max(max(gmx[i] - gmn[i] for i in range(3)) / 2.0, 1.0)
        return {"objects": out,
                "bbox": {"min": [round(x, 4) for x in gmn],
                         "max": [round(x, 4) for x in gmx],
                         "center": [round(x, 4) for x in ctr],
                         "radius": round(rad, 4)},
                "count": len(out)}

    return {"view.render": op_render, "view.views": op_views, "view.scene": op_scene}
