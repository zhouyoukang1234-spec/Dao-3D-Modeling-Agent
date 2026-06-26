"""v2/cylinders.py -- general cylindrical-feature extractor (holes & bosses).

A joint in this machine is a pin/bearing in a round hole. Every such feature is a
cylindrical surface. We read them straight from the mesh, nothing assumed:

  1. faces that belong to a large *planar* facet are flat -> not cylinder walls.
  2. the remaining 'curved' faces are clustered by adjacency.
  3. each cluster is fit to a cylinder:
        axis d   = eigenvector of  Sum(area_i * n_i n_i^T)  with SMALLEST eigval
                   (cylinder normals are perpendicular to the axis)
        center   = circle fit of face centroids projected onto plane _|_ d
        radius r, and axis extent [smin, smax] along d
  4. accept if normals are genuinely perpendicular to d (lambda_min small) and
     the projected radii are consistent (low round-rms).

Returns Cylinder(axis_point, axis_dir, radius, half_len, n_faces, kind) where
kind in {'hole','boss'} from whether face normals point inward (toward axis) or out.
"""
import os, glob, numpy as np, trimesh
from dataclasses import dataclass


@dataclass
class Cylinder:
    center: np.ndarray      # a point on the axis (mid of extent)
    axis: np.ndarray        # unit axis direction
    radius: float
    half_len: float         # half the axial extent
    n_faces: int
    kind: str               # 'hole' | 'boss'

    def __repr__(self):
        return (f"Cyl({self.kind} r={self.radius:5.2f} L={2*self.half_len:5.1f} "
                f"C={np.array2string(self.center,precision=1)} "
                f"ax={np.array2string(self.axis,precision=2,suppress_small=True)})")


def _planar_face_mask(mesh, min_facet_area=20.0):
    """True for faces that lie in a sizeable planar facet (flat regions)."""
    mask = np.zeros(len(mesh.faces), bool)
    for fac, area in zip(mesh.facets, mesh.facets_area):
        if area >= min_facet_area:
            mask[fac] = True
    return mask


def _fit_cylinder(mesh, faces):
    n = mesh.face_normals[faces]
    a = mesh.area_faces[faces]
    cen = mesh.triangles_center[faces]
    M = (n * a[:, None]).T @ n
    w, V = np.linalg.eigh(M)          # ascending
    axis = V[:, 0]                    # least normal-variance dir = cylinder axis
    lam = w[0] / max(w[2], 1e-12)     # perpendicularity quality (small = good)
    # circle fit of centroids in plane _|_ axis
    e1 = np.cross(axis, [1, 0, 0]);
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(axis, [0, 1, 0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    c0 = cen.mean(0)
    q = cen - c0
    u, v = q @ e1, q @ e2
    A = np.c_[2 * u, 2 * v, np.ones(len(u))]
    b = u ** 2 + v ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cu, cv = sol[0], sol[1]
    r = np.sqrt(max(sol[2] + cu ** 2 + cv ** 2, 0))
    radii = np.sqrt((u - cu) ** 2 + (v - cv) ** 2)
    round_rms = np.std(radii)
    axis_c = c0 + cu * e1 + cv * e2          # point on axis
    s = q @ axis
    smin, smax = s.min(), s.max()
    center = axis_c + axis * (0.5 * (smin + smax))
    half_len = 0.5 * (smax - smin)
    # inward vs outward: do face normals point toward axis (hole) or away (boss)?
    radial = (cen - (axis_c + np.outer(s, axis)))
    rn = np.linalg.norm(radial, axis=1, keepdims=True)
    radial = np.divide(radial, rn, out=np.zeros_like(radial), where=rn > 1e-9)
    inward = np.mean(np.sum(n * radial, axis=1)) < 0
    return Cylinder(center, axis, float(r), float(half_len), len(faces),
                    'hole' if inward else 'boss'), lam, float(round_rms)


def detect_cylinders(mesh, min_faces=8, min_r=1.5, max_r=80.0,
                     lam_max=0.06, round_tol=0.18):
    planar = _planar_face_mask(mesh)
    curved = np.where(~planar)[0]
    if len(curved) == 0:
        return []
    cset = set(curved.tolist())
    # adjacency graph restricted to curved faces
    import networkx as nx
    g = nx.Graph()
    g.add_nodes_from(curved.tolist())
    for f0, f1 in mesh.face_adjacency:
        if f0 in cset and f1 in cset:
            g.add_edge(int(f0), int(f1))
    out = []
    for comp in nx.connected_components(g):
        faces = np.array(list(comp))
        if len(faces) < min_faces:
            continue
        cyl, lam, rrms = _fit_cylinder(mesh, faces)
        if (min_r <= cyl.radius <= max_r and lam <= lam_max
                and rrms <= round_tol * cyl.radius):
            out.append(cyl)
    out.sort(key=lambda c: -c.radius)
    return out


if __name__ == "__main__":
    STL_DIR = os.path.join(os.path.dirname(__file__), "..", "ground_truth", "stl")
    for p in sorted(glob.glob(os.path.join(STL_DIR, "*.stl"))):
        m = trimesh.load(p, process=True)
        cyls = detect_cylinders(m)
        print(f"\n=== {os.path.basename(p)} ===  cylinders={len(cyls)}")
        for c in cyls:
            print("   ", c)
