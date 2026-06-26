"""v2/inspect_meshes.py -- ground truth, from the mesh itself, nothing assumed.

For every STL: counts, bounds, extents, volume, watertightness, center of mass,
and PCA principal axes (eigvecs + sqrt-eigvals as physical half-extents along
each principal direction). This is the raw evidence the feature extractor and
pose solver must build on -- we read the parts as they actually are.
"""
import os, glob, numpy as np, trimesh

STL_DIR = os.path.join(os.path.dirname(__file__), "..", "ground_truth", "stl")


def pca(mesh):
    v = mesh.vertices - mesh.vertices.mean(0)
    cov = v.T @ v / len(v)
    w, V = np.linalg.eigh(cov)          # ascending
    order = np.argsort(w)[::-1]         # descending: V[:,0] = longest axis
    return w[order], V[:, order]


def main():
    paths = sorted(glob.glob(os.path.join(STL_DIR, "*.stl")))
    for p in paths:
        m = trimesh.load(p, process=False)
        name = os.path.basename(p)
        ext = m.extents
        w, V = pca(m)
        half = np.sqrt(w)
        print(f"\n=== {name} ===")
        print(f"  verts={len(m.vertices):6d} faces={len(m.faces):6d} "
              f"watertight={m.is_watertight} vol={m.volume:11.1f} mm^3")
        print(f"  bbox extents (x,y,z) = {np.array2string(ext, precision=2)}")
        print(f"  centroid             = {np.array2string(m.centroid, precision=2)}")
        print(f"  PCA half-extents     = {np.array2string(half, precision=2)}  "
              f"(elong = {half[0]/max(half[2],1e-9):.1f}x)")
        for i in range(3):
            print(f"    axis{i} dir={np.array2string(V[:,i], precision=3, suppress_small=True)}")


if __name__ == "__main__":
    main()
