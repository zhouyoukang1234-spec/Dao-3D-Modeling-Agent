#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
perception.py — 三维感知本源 · AI 的 "眼" (engine-agnostic)
═══════════════════════════════════════════════════════════════════════════════
反者道之动 — 不从 "替人建模" 出发, 从 "像人一样看懂三维" 出发.
弱者道之用 — 零重依赖: 仅 numpy 必需 (软件光栅器自携), trimesh/Pillow 可选增强.

本模块是 "AI 参与三维建模" 通用架构的最底层 —— 把任意几何体 (mesh / BREP→mesh)
翻译为两类 AI 可消化的产物:

  1. 多视角渲染 (silhouette / depth / shaded)   ← 让 LLM "看见"
  2. 结构化场景描述 (bbox/主轴/对称/连通/凸性…)  ← 让 LLM "读懂"

二者合一 = 一份 LLM 可直接推理的三维状态报告.

这层与 IDE 里 "语言服务器把代码翻译成符号/诊断给 agent" 同构:
    源码 → AST/类型/诊断          ←→   几何 → 视图/度量/结构
不锁定任何 CAD 引擎; 只要能给出 (vertices, faces) 即可被感知.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

try:  # 可选增强: 鲁棒的体积/面积/连通/凸包/加载
    import trimesh  # type: ignore
    _HAS_TRIMESH = True
except Exception:  # pragma: no cover
    trimesh = None  # type: ignore
    _HAS_TRIMESH = False

try:  # 可选: PNG 落盘
    from PIL import Image  # type: ignore
    _HAS_PIL = True
except Exception:  # pragma: no cover
    Image = None  # type: ignore
    _HAS_PIL = False

PathLike = Union[str, Path]
Array = np.ndarray

__all__ = [
    "Camera", "orbit_camera", "RenderResult", "render",
    "Mesh", "load_mesh", "describe", "perceive", "iou",
    "STANDARD_VIEWS",
]

# 六标准工程视角 (azimuth, elevation) 度 — 等轴测 + 三正视
STANDARD_VIEWS: List[Tuple[str, float, float]] = [
    ("iso",   45.0,  30.0),
    ("front",  0.0,   0.0),
    ("right", 90.0,   0.0),
    ("top",    0.0,  89.9),
]


# ═══════════════════════════════════════════════════════════════════════════
# 一、相机 · 针孔模型 + 环绕取景
# ═══════════════════════════════════════════════════════════════════════════
def _normalize(v: Array) -> Array:
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


@dataclass
class Camera:
    """针孔相机. 看向 target, 位于 eye, 上方向 up; 透视投影."""
    eye: Array
    target: Array
    up: Array
    fov_deg: float = 35.0
    width: int = 256
    height: int = 256

    def basis(self) -> Tuple[Array, Array, Array]:
        """返回相机正交基 (right, up, forward); forward 指向被摄物."""
        f = _normalize(np.asarray(self.target, float) - np.asarray(self.eye, float))
        up0 = np.asarray(self.up, float)
        r = _normalize(np.cross(f, up0))
        if np.linalg.norm(r) < 1e-9:  # forward 与 up 平行的退化情形
            r = _normalize(np.cross(f, np.array([1.0, 0.0, 0.0])))
        u = np.cross(r, f)
        return r, u, f

    def project(self, pts: Array) -> Tuple[Array, Array]:
        """世界点 (N,3) → 像素坐标 (N,2) 与相机深度 (N,). 深度>0 在镜前."""
        r, u, f = self.basis()
        rel = np.asarray(pts, float) - np.asarray(self.eye, float)
        xc = rel @ r
        yc = rel @ u
        zc = rel @ f
        zc_safe = np.where(np.abs(zc) < 1e-6, 1e-6, zc)
        focal = (self.height * 0.5) / math.tan(math.radians(self.fov_deg) * 0.5)
        px = self.width * 0.5 + (xc / zc_safe) * focal
        py = self.height * 0.5 - (yc / zc_safe) * focal
        return np.stack([px, py], axis=1), zc


def orbit_camera(center: Sequence[float], radius: float, az_deg: float,
                 el_deg: float, *, width: int = 256, height: int = 256,
                 fov_deg: float = 35.0, dist_factor: float = 2.6) -> Camera:
    """环绕相机: 以 center 为靶心, 在球面 (方位 az, 仰角 el) 处取景.
    radius = 物体外接球半径; eye 距离 = radius * dist_factor."""
    c = np.asarray(center, float)
    a = math.radians(az_deg)
    e = math.radians(el_deg)
    d = max(radius, 1e-6) * dist_factor
    dir_w = np.array([math.cos(e) * math.cos(a),
                      math.cos(e) * math.sin(a),
                      math.sin(e)])
    eye = c + dir_w * d
    up = np.array([0.0, 0.0, 1.0])
    if abs(el_deg) > 88.0:  # 俯视/仰视: 换 up 防退化
        up = np.array([0.0, 1.0, 0.0])
    return Camera(eye=eye, target=c, up=up, fov_deg=fov_deg, width=width, height=height)


# ═══════════════════════════════════════════════════════════════════════════
# 二、软件光栅器 · z-buffer 三角形扫描 (纯 numpy)
# ═══════════════════════════════════════════════════════════════════════════
@dataclass
class RenderResult:
    depth: Array        # (H,W) float, +inf = 空
    silhouette: Array   # (H,W) bool
    shaded: Array       # (H,W) float 0..1 (Lambert 灰度)
    face_id: Array      # (H,W) int, -1 = 空
    camera: Camera

    def coverage(self) -> float:
        return float(self.silhouette.mean())


def _face_normals(V: Array, F: Array) -> Array:
    tris = V[F]
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln < 1e-12] = 1.0
    return n / ln


def render(V: Array, F: Array, camera: Camera, *,
           light_dir: Optional[Sequence[float]] = None,
           ambient: float = 0.25) -> RenderResult:
    """把三角网格渲染为 depth/silhouette/shaded/face_id 四缓冲.

    纯 numpy 逐面光栅化 (每面在其包围盒内向量化). 适合 ~万级面片的预览/感知;
    更大网格请先抽稀 (见 perceive 的 max_faces)."""
    V = np.asarray(V, float)
    F = np.asarray(F, int)
    W, H = camera.width, camera.height
    depth = np.full((H, W), np.inf, float)
    fid = np.full((H, W), -1, int)
    shaded = np.zeros((H, W), float)

    if len(F) == 0 or len(V) == 0:
        return RenderResult(depth, np.zeros((H, W), bool), shaded, fid, camera)

    px, zc = camera.project(V)            # (N,2), (N,)
    fn = _face_normals(V, F)              # (M,3)

    # 光照: 默认从相机右上方打光 (相机坐标系)
    r, u, f = camera.basis()
    if light_dir is None:
        ld = _normalize(-f + 0.4 * u + 0.3 * r)
    else:
        ld = _normalize(np.asarray(light_dir, float))
    lambert = np.clip(np.abs(fn @ ld), 0.0, 1.0)
    intensity = ambient + (1.0 - ambient) * lambert    # (M,) 平片着色

    p2 = px[F]                            # (M,3,2)
    z3 = zc[F]                            # (M,3)
    front = (z3 > 1e-5).all(axis=1)       # 全部在镜前的面

    xs = p2[..., 0]
    ys = p2[..., 1]
    minx = np.floor(xs.min(axis=1)).astype(int)
    maxx = np.ceil(xs.max(axis=1)).astype(int)
    miny = np.floor(ys.min(axis=1)).astype(int)
    maxy = np.ceil(ys.max(axis=1)).astype(int)
    onscreen = front & (maxx >= 0) & (minx < W) & (maxy >= 0) & (miny < H)

    # 前后排序 + 深度余量: 密网格在布尔接缝处常有近共面/重合三角, 任意序 + 严格 zi<d
    # 会逐像素抖动 (z-fighting 麻点). 改为【由近及远】绘制, 且仅当显著更近 (zi < d - eps)
    # 才覆盖 → 最前面者先占像素, 近共面后来者不再翻覆, 麻点消除.
    idx = np.nonzero(onscreen)[0]
    if len(idx):
        zrep = z3[idx].mean(axis=1)
        idx = idx[np.argsort(zrep)]              # 近(小z)在前
        zspan = float(zc.max() - zc.min())
        eps = max(zspan * 1e-4, 1e-7)
    else:
        eps = 1e-7

    for i in idx:
        x0, x1 = max(0, minx[i]), min(W - 1, maxx[i])
        y0, y1 = max(0, miny[i]), min(H - 1, maxy[i])
        if x0 > x1 or y0 > y1:
            continue
        ax, ay = p2[i, 0]
        bx, by = p2[i, 1]
        cx, cy = p2[i, 2]
        denom = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(denom) < 1e-9:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
        gxf = gx + 0.5
        gyf = gy + 0.5
        w0 = ((by - cy) * (gxf - cx) + (cx - bx) * (gyf - cy)) / denom
        w1 = ((cy - ay) * (gxf - cx) + (ax - cx) * (gyf - cy)) / denom
        w2 = 1.0 - w0 - w1
        inside = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
        if not inside.any():
            continue
        zi = w0 * z3[i, 0] + w1 * z3[i, 1] + w2 * z3[i, 2]
        sub_d = depth[y0:y1 + 1, x0:x1 + 1]
        closer = inside & (zi < sub_d - eps)
        if not closer.any():
            continue
        sub_d[closer] = zi[closer]
        depth[y0:y1 + 1, x0:x1 + 1] = sub_d
        sub_f = fid[y0:y1 + 1, x0:x1 + 1]
        sub_f[closer] = i
        fid[y0:y1 + 1, x0:x1 + 1] = sub_f
        sub_s = shaded[y0:y1 + 1, x0:x1 + 1]
        sub_s[closer] = intensity[i]
        shaded[y0:y1 + 1, x0:x1 + 1] = sub_s

    sil = fid >= 0
    return RenderResult(depth, sil, shaded, fid, camera)


def iou(a: Array, b: Array) -> float:
    """两个布尔掩码的 IoU."""
    a = a.astype(bool); b = b.astype(bool)
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return float(inter) / float(union) if union else 0.0


# ═══════════════════════════════════════════════════════════════════════════
# 三、网格容器与加载
# ═══════════════════════════════════════════════════════════════════════════
@dataclass
class Mesh:
    """最小三角网格容器. 任何 CAD 引擎产物 → (V,F) → Mesh → 可感知."""
    vertices: Array
    faces: Array
    name: str = "mesh"

    @property
    def bounds(self) -> Array:
        return np.array([self.vertices.min(axis=0), self.vertices.max(axis=0)])

    @property
    def extents(self) -> Array:
        b = self.bounds
        return b[1] - b[0]

    @property
    def centroid(self) -> Array:
        return self.vertices.mean(axis=0)

    @property
    def bounding_radius(self) -> float:
        c = (self.bounds[0] + self.bounds[1]) * 0.5
        return float(np.linalg.norm(self.vertices - c, axis=1).max())

    def to_trimesh(self):  # pragma: no cover
        if not _HAS_TRIMESH:
            raise RuntimeError("trimesh 未安装")
        return trimesh.Trimesh(vertices=self.vertices, faces=self.faces, process=False)


def load_mesh(source: Union[PathLike, "Mesh", Any], name: Optional[str] = None) -> Mesh:
    """从文件路径 / Mesh / trimesh / (V,F) 元组载入统一 Mesh."""
    if isinstance(source, Mesh):
        return source
    if isinstance(source, (tuple, list)) and len(source) == 2:
        V, F = source
        return Mesh(np.asarray(V, float), np.asarray(F, int), name or "mesh")
    if _HAS_TRIMESH and isinstance(source, trimesh.Trimesh):
        return Mesh(np.asarray(source.vertices, float),
                    np.asarray(source.faces, int), name or "mesh")
    # 路径
    p = Path(source)
    if not _HAS_TRIMESH:
        raise RuntimeError("从文件加载需要 trimesh")
    loaded = trimesh.load(str(p), process=False, force="mesh")
    return Mesh(np.asarray(loaded.vertices, float),
                np.asarray(loaded.faces, int), name or p.stem)


# ═══════════════════════════════════════════════════════════════════════════
# 四、结构化场景描述 · 让 LLM "读懂" 几何
# ═══════════════════════════════════════════════════════════════════════════
def _triangle_areas(V: Array, F: Array) -> Array:
    tris = V[F]
    return 0.5 * np.linalg.norm(
        np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)


def _symmetry_scores(V: Array, center: Array, axes: Array) -> Dict[str, float]:
    """沿三主轴定义的镜面对称分: 1.0 = 完美镜像对称.
    采样顶点, 关于过质心、法向为各主轴的平面镜像, 用最近点平均偏差归一化."""
    pts = V - center
    if len(pts) > 4000:
        idx = np.random.default_rng(0).choice(len(pts), 4000, replace=False)
        pts = pts[idx]
    diag = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0))) or 1.0
    out: Dict[str, float] = {}
    for k in range(3):
        nrm = axes[k]
        d = pts @ nrm
        mirrored = pts - 2.0 * np.outer(d, nrm)
        # 最近邻平均距离 (子采样以控开销)
        q = mirrored
        if len(pts) > 1500:
            sel = np.random.default_rng(1).choice(len(pts), 1500, replace=False)
            base = pts[sel]
        else:
            base = pts
        # 分块算最近点距离
        dists = []
        for j in range(0, len(q), 256):
            blk = q[j:j + 256]
            dd = np.linalg.norm(base[None, :, :] - blk[:, None, :], axis=2).min(axis=1)
            dists.append(dd)
        mean_err = float(np.concatenate(dists).mean()) if dists else diag
        out[f"axis{k}"] = max(0.0, 1.0 - mean_err / (0.05 * diag))
    return out


def describe(mesh: Union[Mesh, Any]) -> Dict[str, Any]:
    """输出一份结构化几何报告 (纯数据, 可 JSON 序列化)."""
    m = load_mesh(mesh) if not isinstance(mesh, Mesh) else mesh
    V, F = m.vertices, m.faces
    bounds = m.bounds
    ext = m.extents
    dims_sorted = np.sort(ext)[::-1]
    center = (bounds[0] + bounds[1]) * 0.5
    diag = float(np.linalg.norm(ext))

    rep: Dict[str, Any] = {
        "name": m.name,
        "n_vertices": int(len(V)),
        "n_faces": int(len(F)),
        "bounds_min": [round(x, 4) for x in bounds[0]],
        "bounds_max": [round(x, 4) for x in bounds[1]],
        "extents": [round(x, 4) for x in ext],
        "dims_sorted_desc": [round(x, 4) for x in dims_sorted],
        "diagonal": round(diag, 4),
        "centroid": [round(x, 4) for x in m.centroid],
        "bbox_center": [round(x, 4) for x in center],
    }

    # 面积 (自携) + trimesh 增强项
    area = float(_triangle_areas(V, F).sum())
    rep["surface_area"] = round(area, 4)
    bbox_vol = float(ext[0] * ext[1] * ext[2])
    rep["bbox_volume"] = round(bbox_vol, 4)

    volume = None; watertight = None; n_comp = None; convexity = None; genus = None
    if _HAS_TRIMESH:
        try:
            tm = m.to_trimesh()
            watertight = bool(tm.is_watertight)
            volume = float(abs(tm.volume)) if watertight else None
            try:
                n_comp = int(len(tm.split(only_watertight=False)))
            except Exception:
                n_comp = None
            if watertight and volume:
                try:
                    chv = float(abs(tm.convex_hull.volume))
                    convexity = round(volume / chv, 4) if chv > 0 else None
                except Exception:
                    convexity = None
                try:
                    genus = int(tm.euler_number)  # 报 Euler 数, 由调用方解读
                except Exception:
                    genus = None
        except Exception:
            pass
    rep["watertight"] = watertight
    rep["volume"] = round(volume, 4) if volume is not None else None
    rep["n_components"] = n_comp
    rep["convexity"] = convexity
    rep["euler_number"] = genus
    rep["bbox_fill_ratio"] = round(volume / bbox_vol, 4) if (volume and bbox_vol > 0) else None

    # PCA 主轴
    try:
        pc = V - V.mean(axis=0)
        cov = pc.T @ pc / max(1, len(pc))
        evals, evecs = np.linalg.eigh(cov)
        order = np.argsort(evals)[::-1]
        evals = evals[order]; evecs = evecs[:, order]
        axes = evecs.T  # 行为主轴
        s = float(evals.sum()) or 1.0
        rep["principal_axes"] = [[round(x, 4) for x in axes[k]] for k in range(3)]
        rep["principal_variance_ratio"] = [round(float(evals[k] / s), 4) for k in range(3)]
        rep["symmetry"] = {k: round(v, 3) for k, v in
                           _symmetry_scores(V, V.mean(axis=0), axes).items()}
    except Exception:
        rep["principal_axes"] = None
        rep["symmetry"] = None

    return rep


def summarize(rep: Dict[str, Any]) -> str:
    """把结构化报告渲染成 LLM 友好的自然语言要点 (中文)."""
    L: List[str] = []
    L.append(f"几何体「{rep.get('name')}」: {rep.get('n_vertices')} 顶点 / {rep.get('n_faces')} 面.")
    d = rep.get("dims_sorted_desc")
    if d:
        L.append(f"尺寸 (降序): {d[0]} × {d[1]} × {d[2]} (对角线 {rep.get('diagonal')}).")
    wt = rep.get("watertight")
    if wt is not None:
        s = "封闭水密" if wt else "非水密 (有开口/缝隙)"
        if rep.get("volume") is not None:
            s += f", 体积 {rep['volume']}"
        L.append(f"拓扑: {s}.")
    nc = rep.get("n_components")
    if nc:
        L.append(f"连通体: {nc} 个{' (单体)' if nc == 1 else ' (多部件/装配)'}.")
    cv = rep.get("convexity")
    if cv is not None:
        kind = "近似凸体" if cv > 0.92 else ("中等凹凸" if cv > 0.6 else "强凹/镂空")
        L.append(f"凸度 {cv} ({kind}).")
    bf = rep.get("bbox_fill_ratio")
    if bf is not None:
        L.append(f"包围盒填充率 {bf} ({'实心方正' if bf > 0.6 else '细长/中空/复杂'}).")
    sym = rep.get("symmetry")
    if sym:
        best = max(sym.items(), key=lambda kv: kv[1])
        L.append(f"对称性: 最强镜面对称沿主轴 {best[0]} (分值 {best[1]}).")
    return " ".join(L)


# ═══════════════════════════════════════════════════════════════════════════
# 五、perceive · 一步取景 (多视角渲染 + 结构描述 + 摘要)
# ═══════════════════════════════════════════════════════════════════════════
def _maybe_decimate(V: Array, F: Array, max_faces: int) -> Tuple[Array, Array, bool]:
    if len(F) <= max_faces:
        return V, F, False
    if _HAS_TRIMESH:
        try:
            tm = trimesh.Trimesh(vertices=V, faces=F, process=False)
            dec = tm.simplify_quadric_decimation(face_count=max_faces)
            # 仅当抽稀仍封闭水密时采用 (CAD 抽稀常生破面/尖刺, 宁可不抽)
            if len(dec.faces) > 0 and bool(dec.is_watertight):
                return np.asarray(dec.vertices, float), np.asarray(dec.faces, int), True
        except Exception:
            pass
    # 抽稀不可用或不可靠: 直接用原网格 (软栅器可胜任数万面, 正确优先于快).
    # 切忌随机抽面 —— 那会留空洞, 透出内壁背面 → 渲染麻点/尖刺.
    return V, F, False


def perceive(mesh: Union[Mesh, PathLike, Any], *,
             views: Optional[Sequence[Tuple[str, float, float]]] = None,
             resolution: int = 256, max_faces: int = 80000,
             out_dir: Optional[PathLike] = None,
             save_png: bool = False) -> Dict[str, Any]:
    """对一个几何体做完整感知:
        · describe()  结构化报告
        · 多标准视角 silhouette/shaded 渲染 (numpy 缓冲, 可选落 PNG)
        · summarize() 自然语言摘要
    返回 dict (renders 字段中的 numpy 缓冲在 save_png=True 时另存为 PNG 路径)."""
    m = load_mesh(mesh) if not isinstance(mesh, Mesh) else mesh
    views = list(views or STANDARD_VIEWS)
    rep = describe(m)

    Vr, Fr, decimated = _maybe_decimate(m.vertices, m.faces, max_faces)
    center = (m.bounds[0] + m.bounds[1]) * 0.5
    radius = m.bounding_radius

    renders: Dict[str, Any] = {}
    if out_dir is not None:
        out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)

    for vname, az, el in views:
        cam = orbit_camera(center, radius, az, el,
                           width=resolution, height=resolution)
        rr = render(Vr, Fr, cam)
        entry: Dict[str, Any] = {
            "az": az, "el": el,
            "coverage": round(rr.coverage(), 4),
        }
        if save_png and _HAS_PIL and out_dir is not None:
            img = (np.clip(rr.shaded, 0, 1) * 255).astype(np.uint8)
            rgb = np.stack([img] * 3, axis=2)
            rgb[~rr.silhouette] = np.array([245, 245, 248], np.uint8)
            path = out_dir / f"{m.name}_{vname}.png"
            Image.fromarray(rgb).save(path)
            entry["png"] = str(path)
        renders[vname] = entry

    return {
        "report": rep,
        "summary": summarize(rep),
        "renders": renders,
        "decimated_for_render": decimated,
        "render_faces": int(len(Fr)),
    }


if __name__ == "__main__":  # 自检: 渲染一个盒子, 打印感知报告
    import json as _json
    V = np.array([[0, 0, 0], [40, 0, 0], [40, 30, 0], [0, 30, 0],
                  [0, 0, 20], [40, 0, 20], [40, 30, 20], [0, 30, 20]], float)
    F = np.array([[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6],
                  [0, 4, 5], [0, 5, 1], [1, 5, 6], [1, 6, 2],
                  [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0]], int)
    out = perceive(Mesh(V, F, "demo_box"), resolution=96)
    print(out["summary"])
    print(_json.dumps(out["renders"], ensure_ascii=False, indent=2))
