"""v2/probe_features.py -- what raw features can we robustly read from each mesh?

Two general, mesh-only signals:
  (1) planar facets   -> mating faces      (trimesh.facets: coplanar face groups)
  (2) boundary loops  -> hole rims / pin bores (open-edge loops; circle-fit each)

A circular boundary loop is a joint axis: center C, normal n (=axis), radius r.
These are exactly the 'where parts hook together' features. Print a census so we
can see how clean the signal is before building the extractor on top of it.
"""
import os, glob, numpy as np, trimesh

STL_DIR = os.path.join(os.path.dirname(__file__), "..", "ground_truth", "stl")


def loops_from_open_edges(mesh):
    edges = mesh.edges[trimesh.grouping.group_rows(mesh.edges_sorted, require_count=1)]
    if len(edges) == 0:
        return []
    import networkx as nx
    g = nx.Graph()
    g.add_edges_from(edges)
    loops = []
    for comp in nx.connected_components(g):
        loops.append(np.array(list(comp)))
    return loops


def circle_fit(pts):
    """Best-fit plane normal + in-plane circle (center, radius, planarity)."""
    c0 = pts.mean(0)
    q = pts - c0
    _, _, Vt = np.linalg.svd(q, full_matrices=False)
    n = Vt[2]                                   # plane normal = smallest sing dir
    planar_rms = float(np.sqrt(np.mean((q @ n) ** 2)))
    # project to plane basis
    e1, e2 = Vt[0], Vt[1]
    u, v = q @ e1, q @ e2
    A = np.c_[2 * u, 2 * v, np.ones(len(u))]
    b = u ** 2 + v ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cu, cv = sol[0], sol[1]
    r = np.sqrt(max(sol[2] + cu ** 2 + cv ** 2, 0))
    center = c0 + cu * e1 + cv * e2
    radii = np.sqrt((u - cu) ** 2 + (v - cv) ** 2)
    round_rms = float(np.std(radii))
    return center, n, r, planar_rms, round_rms


def main():
    for p in sorted(glob.glob(os.path.join(STL_DIR, "*.stl"))):
        m = trimesh.load(p, process=True)
        name = os.path.basename(p)
        facets = m.facets
        big = sorted(m.facets_area, reverse=True)[:4] if len(facets) else []
        loops = loops_from_open_edges(m)
        print(f"\n=== {name} ===  facets={len(facets)}  open_loops={len(loops)}")
        if len(big):
            print("  top facet areas:", np.array2string(np.array(big), precision=0))
        circ = []
        for lp in loops:
            if len(lp) < 6:
                continue
            pts = m.vertices[lp]
            c, n, r, prms, rrms = circle_fit(pts)
            if r > 1.0 and rrms < 0.15 * r and prms < 0.5:   # clean round planar loop
                circ.append((r, c, n))
        circ.sort(reverse=True)
        print(f"  circular bores/rims: {len(circ)}")
        for r, c, n in circ[:8]:
            print(f"    r={r:6.2f}  C={np.array2string(c,precision=1):>22}  "
                  f"axis={np.array2string(np.abs(n),precision=2)}")


if __name__ == "__main__":
    main()
