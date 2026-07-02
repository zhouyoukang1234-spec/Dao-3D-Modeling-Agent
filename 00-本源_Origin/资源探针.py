#!/usr/bin/env python3
"""
资源探针 · 世界3D建模资源连通器  v2.0
道法自然 — 反者道之动 — 逆向所有平台底层API，打通一切，获取本源之资

支持平台 (20个, HTTP + Playwright):
  printables    Printables.com (Prusa)     GraphQL API — 无需认证
  sketchfab     Sketchfab                  REST API — 搜索免费/下载需token
  cults3d       Cults3D                    HTML解析 — 无需认证
  yeggi         Yeggi 元搜索引擎            HTML解析 — 无需认证 (40M+, JS挑战)
  nasa          NASA 3D Resources          images-api REST — 无需认证
  github        GitHub 3D建模仓库           REST API — 可选token
  3d66          3D溜溜                      HTML解析 — 无需认证
  mohou         魔猴.com 中国工程平台        HTML解析 — 无需认证
  mmf           MyMiniFactory              REST API — 可选key
  nih           NIH 3D Print Exchange      REST API — 无需认证
  thangs        Thangs.com                 REST API — 无需认证
  thingiverse   Thingiverse                REST API — 需token
  stlfinder     STLFinder 聚合引擎          HTML解析 — CloudFlare保护
  rtd           ReadTheDocs文档搜索         REST API — 无需认证
  thangs_pw     Thangs (Playwright)        SSR __NEXT_DATA__ 直提
  grabcad_pw    GrabCAD (Playwright)       网络流量拦截
  yeggi_pw      Yeggi (Playwright)         JS机器人挑战绕过 (40M+)
  stlfinder_pw  STLFinder (Playwright)     CloudFlare WAF 绕过
  mmf_pw        MyMiniFactory (Playwright) React SPA / 网络拦截
  nih_pw        NIH 3D Print (Playwright)  React SPA / 旧API已下线

用法:
  python 资源探针.py search "gear" [--platform all|printables|sketchfab|...] [--parallel]
  python 资源探针.py batch "gear" --pages 5 --platform printables,sketchfab,cults3d
  python 资源探针.py download --platform printables --id 12345 [--out ./downloads]
  python 资源探针.py probe [--platform all]        # 探测API状态
  python 资源探针.py sync-libs                     # 同步GitHub核心库元数据
  python 资源探针.py docs "fillet chamfer"         # 搜索文档
  python 资源探针.py github-code "involute gear"   # GitHub代码搜索
  python 资源探针.py report                        # 生成全平台状态报告
"""

import os
import sys
import json
import time
import argparse
import urllib.request
import urllib.parse
import urllib.error
import ssl
from pathlib import Path
from datetime import datetime
from typing import Optional

# Windows下修复SSL证书验证问题
_SSL_CTX = ssl.create_default_context()
try:
    import certifi
    _SSL_CTX = ssl.create_default_context(cafile=certifi.where())
except ImportError:
    _SSL_CTX.check_hostname = False
    _SSL_CTX.verify_mode = ssl.CERT_NONE

# 无验证SSL上下文 — 用于HTML爬虫 (部分站使用非标准证书)
_SSL_CTX_NOVERIFY = ssl.create_default_context()
_SSL_CTX_NOVERIFY.check_hostname = False
_SSL_CTX_NOVERIFY.verify_mode = ssl.CERT_NONE

# ─── 配置 ────────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent.resolve()

# ═══ 万法归一 · 路径引导 ════════════════════════════════════════════
_DAO_ROOT = next((p for p in Path(__file__).resolve().parents
                  if (p / '_paths.py').is_file()), SCRIPT_DIR.parent)
if str(_DAO_ROOT) not in sys.path:
    sys.path.insert(0, str(_DAO_ROOT))
import _paths as _dao_paths  # noqa: F401  (registers 五层 sys.path)
ROOT_DIR = _DAO_ROOT
# ═══════════════════════════════════════════════════════════════════

CACHE_DIR = _dao_paths.WORLD / ".resource_cache"
DOWNLOAD_DIR = _dao_paths.WORLD / "downloads"
CACHE_DIR.mkdir(parents=True, exist_ok=True)
DOWNLOAD_DIR.mkdir(parents=True, exist_ok=True)

# 环境变量覆盖认证
THINGIVERSE_TOKEN = os.environ.get("THINGIVERSE_TOKEN", "")
MMF_API_KEY       = os.environ.get("MMF_API_KEY", "")
SKETCHFAB_TOKEN   = os.environ.get("SKETCHFAB_TOKEN", "")
GITHUB_TOKEN      = os.environ.get("GITHUB_TOKEN", "")

UA = "ModelForge-ResourceProbe/1.0 (3D Modeling Agent; dao-agi)"

# ─── HTTP 底层 ────────────────────────────────────────────────────────────────
def _http_get(url: str, headers: dict = None, timeout: int = 10) -> dict | None:
    """通用GET请求，返回JSON或None。SSL失败自动降级noverify。"""
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    if headers:
        for k, v in headers.items():
            req.add_header(k, v)
    for ctx in (_SSL_CTX, _SSL_CTX_NOVERIFY):
        try:
            with urllib.request.urlopen(req, timeout=timeout, context=ctx) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
                return json.loads(raw)
        except ssl.SSLError:
            continue  # SSL失败 → 降级noverify重试
        except urllib.error.HTTPError as e:
            print(f"  ✗ HTTP {e.code}: {url}")
            return None
        except urllib.error.URLError as e:
            if "SSL" in str(e.reason) or "CERTIFICATE" in str(e.reason).upper():
                continue  # SSL相关URLError → 降级重试
            print(f"  ✗ 网络错误: {e.reason}")
            return None
        except json.JSONDecodeError:
            print(f"  ✗ JSON解析失败: {url}")
            return None
        except Exception as e:
            print(f"  ✗ 错误: {e}")
            return None
    return None


def _http_post_json(url: str, payload: dict, headers: dict = None, timeout: int = 10) -> dict | None:
    """通用POST JSON请求。SSL失败自动降级noverify。"""
    data = json.dumps(payload).encode("utf-8")
    for ctx in (_SSL_CTX, _SSL_CTX_NOVERIFY):
        req = urllib.request.Request(url, data=data, method="POST")
        req.add_header("User-Agent", UA)
        req.add_header("Content-Type", "application/json")
        if headers:
            for k, v in headers.items():
                req.add_header(k, v)
        try:
            with urllib.request.urlopen(req, timeout=timeout, context=ctx) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
                return json.loads(raw)
        except ssl.SSLError:
            continue
        except urllib.error.URLError as e:
            if "SSL" in str(e.reason) or "CERTIFICATE" in str(e.reason).upper():
                continue
            print(f"  ✗ 错误: {e}")
            return None
        except urllib.error.HTTPError as e:
            body = e.read().decode("utf-8", errors="replace")[:200]
            print(f"  ✗ HTTP {e.code}: {url} — {body}")
            return None
        except Exception as e:
            print(f"  ✗ 错误: {e}")
            return None
    return None


def _download_file(url: str, dest: Path, headers: dict = None, timeout: int = 30) -> bool:
    """下载二进制文件"""
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    if headers:
        for k, v in headers.items():
            req.add_header(k, v)
    try:
        with urllib.request.urlopen(req, timeout=timeout, context=_SSL_CTX) as resp:
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(resp.read())
        return True
    except Exception as e:
        print(f"  ✗ 下载失败 {url}: {e}")
        return False


# ─── 各平台客户端 ─────────────────────────────────────────────────────────────

class PrintablesClient:
    """Printables.com GraphQL API — 无需认证"""
    ENDPOINT = "https://api.printables.com/graphql/"

    SEARCH_QUERY = """
    query SearchPrints($query: String!, $limit: Int, $offset: Int) {
      searchPrints2(query: $query, limit: $limit, offset: $offset) {
        items {
          id
          name
          slug
          user { publicUsername }
          likesCount
          makesCount
          downloadCount
          image { filePath }
          summary
          license { abbreviation name }
          tags { name }
        }
        totalCount
      }
    }
    """

    FILES_QUERY = """
    query PrintFiles($id: ID!) {
      print(id: $id) {
        id
        name
        stls { id name fileSize downloadUrl }
        images { id filePath }
      }
    }
    """

    def search(self, query: str, limit: int = 20) -> list:
        payload = {
            "query": self.SEARCH_QUERY,
            "variables": {"query": query, "limit": limit, "offset": 0}
        }
        resp = _http_post_json(self.ENDPOINT, payload)
        if not resp:
            return []
        items = resp.get("data", {}).get("searchPrints2", {}).get("items", [])
        results = []
        for item in items:
            results.append({
                "platform": "printables",
                "id": item.get("id"),
                "name": item.get("name"),
                "author": item.get("user", {}).get("publicUsername", "?"),
                "url": f"https://www.printables.com/model/{item.get('id')}-{item.get('slug','')}",
                "likes": item.get("likesCount", 0),
                "downloads": item.get("downloadCount", 0),
                "license": item.get("license", {}).get("abbreviation", "?") if item.get("license") else "?",
                "tags": [t["name"] for t in item.get("tags", [])],
                "thumbnail": item.get("image", {}).get("filePath", ""),
                "summary": (item.get("summary") or "")[:100],
            })
        return results

    def get_files(self, model_id: str) -> list:
        payload = {
            "query": self.FILES_QUERY,
            "variables": {"id": model_id}
        }
        resp = _http_post_json(self.ENDPOINT, payload)
        if not resp:
            return []
        stls = resp.get("data", {}).get("print", {}).get("stls", [])
        return [{"name": s.get("name"), "size": s.get("fileSize"), "url": s.get("downloadUrl")} for s in stls]

    def download(self, model_id: str, out_dir: Path) -> list:
        files = self.get_files(model_id)
        downloaded = []
        for f in files:
            if not f.get("url"):
                continue
            fname = Path(f["name"]) if f["name"] else Path(f"printables_{model_id}.stl")
            dest = out_dir / f"printables_{model_id}" / fname
            print(f"  ↓ {fname.name} ({f.get('size', 0)//1024}KB)")
            if _download_file(f["url"], dest):
                downloaded.append(str(dest))
        return downloaded

    def probe(self) -> dict:
        payload = {"query": "{ __typename }", "variables": {}}
        resp = _http_post_json(self.ENDPOINT, payload)
        return {"platform": "printables", "status": "✅ 在线" if resp else "✗ 离线", "auth": "无需认证"}


class MyMiniFactoryClient:
    """MyMiniFactory REST API v2"""
    BASE = "https://www.myminifactory.com/api/v2"

    def _headers(self):
        h = {}
        if MMF_API_KEY:
            h["X-Api-Key"] = MMF_API_KEY
        return h

    def search(self, query: str, limit: int = 20) -> list:
        url = f"{self.BASE}/search?q={urllib.parse.quote(query)}&per_page={limit}"
        resp = _http_get(url, self._headers())
        if not resp:
            return []
        items = resp.get("items", [])
        results = []
        for item in items:
            results.append({
                "platform": "myminifactory",
                "id": str(item.get("id")),
                "name": item.get("name", ""),
                "author": item.get("designer", {}).get("username", "?"),
                "url": item.get("url", ""),
                "likes": item.get("likes", 0),
                "downloads": item.get("download_count", 0),
                "license": item.get("license", {}).get("label", "?") if isinstance(item.get("license"), dict) else str(item.get("license", "?")),
                "tags": item.get("tags", []),
                "thumbnail": item.get("images", [{}])[0].get("thumbnail", {}).get("url", "") if item.get("images") else "",
                "summary": (item.get("description") or "")[:100],
            })
        return results

    def get_files(self, object_id: str) -> list:
        url = f"{self.BASE}/objects/{object_id}"
        resp = _http_get(url, self._headers())
        if not resp:
            return []
        files = resp.get("files", {}).get("items", [])
        return [{"name": f.get("filename"), "size": f.get("filesize"), "url": f.get("download_url")} for f in files]

    def download(self, object_id: str, out_dir: Path) -> list:
        files = self.get_files(object_id)
        downloaded = []
        for f in files:
            if not f.get("url"):
                continue
            fname = f.get("name") or f"mmf_{object_id}.stl"
            dest = out_dir / f"mmf_{object_id}" / fname
            print(f"  ↓ {fname}")
            if _download_file(f["url"], dest, self._headers()):
                downloaded.append(str(dest))
        return downloaded

    def probe(self) -> dict:
        resp = _http_get(f"{self.BASE}/categories", self._headers())
        key_status = f"key={'已设置' if MMF_API_KEY else '未设置(可选)'}"
        return {"platform": "myminifactory", "status": "✅ 在线" if resp else "✗ 离线", "auth": key_status}


class NIHClient:
    """NIH 3D Print Exchange — 完全公开 (SSL使用NOVERIFY)"""
    BASE = "https://3dprint.nih.gov/api/v1"

    def _get(self, url: str) -> dict | None:
        """NIH专用GET，使用NOVERIFY SSL"""
        import urllib.request as ur
        req = ur.Request(url, headers={"User-Agent": "ModelForge/1.0", "Accept": "application/json"})
        try:
            with ur.urlopen(req, context=_SSL_CTX_NOVERIFY, timeout=15) as r:
                return json.loads(r.read())
        except Exception:
            return None

    def search(self, query: str, limit: int = 20) -> list:
        url = f"{self.BASE}/model?type=All&search={urllib.parse.quote(query)}&limit={limit}"
        resp = self._get(url)
        if not resp:
            return []
        items = resp.get("data", []) if isinstance(resp, dict) else (resp if isinstance(resp, list) else [])
        results = []
        for item in items[:limit]:
            results.append({
                "platform": "nih",
                "id": str(item.get("nid", "")),
                "name": item.get("title", ""),
                "author": item.get("username", "?"),
                "url": f"https://3dprint.nih.gov/discover/{item.get('nid', '')}",
                "likes": item.get("flag_count", 0),
                "downloads": item.get("download_count", 0),
                "license": item.get("license", "CC0"),
                "tags": item.get("tags", []),
                "thumbnail": item.get("image_url", ""),
                "summary": (item.get("description") or "")[:100],
            })
        return results

    def probe(self) -> dict:
        resp = self._get(f"{self.BASE}/model?limit=1")
        if resp:
            return {"platform": "nih", "status": "✅ 在线", "auth": "无需认证"}
        return {"platform": "nih", "status": "✗ 离线 (REST API已迁移React SPA, 用nih_pw)", "auth": "无需认证"}


class NASAClient:
    """NASA Images & 3D Resources — images-api.nasa.gov (稳定可用)"""
    IMAGES_API = "https://images-api.nasa.gov/search"
    GITHUB_MODELS = "https://api.github.com/repos/nasa/NASA-3D-Resources/git/trees/master?recursive=1"

    def _search_images_api(self, query: str, page_size: int = 20) -> list:
        params = urllib.parse.urlencode({"q": query, "page_size": page_size})
        resp = _http_get(f"{self.IMAGES_API}?{params}")
        if not resp:
            return []
        items = resp.get("collection", {}).get("items", [])
        results = []
        for item in items:
            data = item.get("data", [{}])[0]
            links = item.get("links", [{}])
            thumb = next((lk.get("href", "") for lk in links if lk.get("rel") == "preview"), "")
            nasa_id = data.get("nasa_id", "")
            results.append({
                "platform": "nasa",
                "id": nasa_id,
                "name": data.get("title", ""),
                "author": "NASA",
                "url": f"https://images.nasa.gov/details/{nasa_id}",
                "likes": 0,
                "downloads": 0,
                "license": "CC0 / Public Domain",
                "tags": data.get("keywords", []),
                "thumbnail": thumb,
                "summary": (data.get("description", "") or "")[:100],
            })
        return results

    def _search_github(self, query: str, limit: int = 20) -> list:
        """从GitHub nasa/NASA-3D-Resources仓库树中搜索模型文件"""
        client = GitHubClient()
        resp = _http_get(self.GITHUB_MODELS, client._headers())
        if not resp:
            return []
        tree = resp.get("tree", [])
        query_lower = query.lower()
        results = []
        seen_dirs = set()
        for node in tree:
            path = node.get("path", "")
            parts = path.split("/")
            if len(parts) < 2:
                continue
            folder = parts[0]
            fname = parts[-1].lower()
            ext = fname.rsplit(".", 1)[-1] if "." in fname else ""
            if ext not in ("stl", "obj", "fbx", "blend", "step", "iges", "3ds", "zip"):
                continue
            if query_lower not in path.lower() and query_lower not in folder.lower():
                if not any(query_lower in p.lower() for p in parts):
                    continue
            if folder in seen_dirs:
                continue
            seen_dirs.add(folder)
            results.append({
                "platform": "nasa",
                "id": folder,
                "name": folder.replace("_", " ").replace("-", " "),
                "author": "NASA",
                "url": f"https://github.com/nasa/NASA-3D-Resources/tree/master/{urllib.parse.quote(folder)}",
                "likes": 0,
                "downloads": 0,
                "license": "CC0 / Public Domain",
                "tags": [ext],
                "thumbnail": "",
                "summary": path,
            })
            if len(results) >= limit:
                break
        return results

    def search(self, query: str, limit: int = 20) -> list:
        results = self._search_github(query, limit)
        if not results:
            results = self._search_images_api(query, limit)
        return results[:limit]

    def probe(self) -> dict:
        params = urllib.parse.urlencode({"q": "spacecraft", "page_size": 1})
        resp = _http_get(f"{self.IMAGES_API}?{params}")
        if resp:
            total = resp.get("collection", {}).get("metadata", {}).get("total_hits", 0)
            return {"platform": "nasa", "status": f"✅ images-api在线 total={total}", "auth": "无需认证"}
        return {"platform": "nasa", "status": "✗ 离线", "auth": "无需认证"}


def _sketchfab_url(item: dict) -> str:
    """Build canonical Sketchfab URL from name+uid (viewerUrl often has 'none' slug)."""
    import re as _re
    uid = item.get("uid", "")
    name = item.get("name", "")
    viewer = item.get("viewerUrl", "")
    if viewer and "none" not in viewer:
        return viewer
    if name and uid:
        slug = _re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-")
        return f"https://sketchfab.com/3d-models/{slug}-{uid}"
    return f"https://sketchfab.com/3d-models/{uid}"


class SketchfabClient:
    """Sketchfab REST API v3"""
    BASE = "https://api.sketchfab.com/v3"

    def _headers(self):
        h = {}
        if SKETCHFAB_TOKEN:
            h["Authorization"] = f"Token {SKETCHFAB_TOKEN}"
        return h

    def search(self, query: str, limit: int = 20) -> list:
        p = {"type": "models", "q": query, "count": min(limit, 24)}
        if SKETCHFAB_TOKEN:
            p["downloadable"] = "true"
        params = urllib.parse.urlencode(p)
        url = f"{self.BASE}/search?{params}"
        resp = _http_get(url, self._headers())
        if not resp:
            return []
        results = []
        for item in resp.get("results", []):
            results.append({
                "platform": "sketchfab",
                "id": item.get("uid"),
                "name": item.get("name", ""),
                "author": item.get("user", {}).get("username", "?"),
                "url": _sketchfab_url(item),
                "likes": item.get("likeCount", 0),
                "downloads": item.get("downloadCount", 0),
                "license": item.get("license", {}).get("label", "?") if item.get("license") else "?",
                "tags": [t.get("name") for t in item.get("tags", [])],
                "thumbnail": (item.get("thumbnails", {}).get("images", [{}])[0].get("url", "")),
                "summary": (item.get("description") or "")[:100],
            })
        return results

    def get_download_url(self, uid: str) -> Optional[str]:
        if not SKETCHFAB_TOKEN:
            print("  ! Sketchfab下载需要Token (设置 SKETCHFAB_TOKEN 环境变量)")
            return None
        resp = _http_get(f"{self.BASE}/models/{uid}/download", self._headers())
        if not resp:
            return None
        return resp.get("gltf", {}).get("url") or resp.get("source", {}).get("url")

    def probe(self) -> dict:
        resp = _http_get(f"{self.BASE}/models?count=1")
        auth = f"token={'已设置' if SKETCHFAB_TOKEN else '未设置(无token搜索结果随机，需token获精准结果)'}"
        return {"platform": "sketchfab", "status": "✅ 在线" if resp else "✗ 离线", "auth": auth}


class GitHubClient:
    """GitHub 3D建模仓库/代码搜索"""
    BASE = "https://api.github.com"

    def _headers(self):
        h = {"Accept": "application/vnd.github.v3+json"}
        if GITHUB_TOKEN:
            h["Authorization"] = f"token {GITHUB_TOKEN}"
        return h

    def search_repos(self, query: str, limit: int = 20) -> list:
        params = urllib.parse.urlencode({
            "q": f"{query} topic:3d-printing OR topic:openscad OR topic:cadquery",
            "sort": "stars",
            "per_page": min(limit, 30),
        })
        url = f"{self.BASE}/search/repositories?{params}"
        resp = _http_get(url, self._headers())
        if not resp:
            return []
        results = []
        for item in resp.get("items", []):
            results.append({
                "platform": "github",
                "id": str(item.get("id")),
                "name": item.get("full_name", ""),
                "author": item.get("owner", {}).get("login", "?"),
                "url": item.get("html_url", ""),
                "likes": item.get("stargazers_count", 0),
                "downloads": item.get("forks_count", 0),
                "license": item.get("license", {}).get("spdx_id", "?") if item.get("license") else "?",
                "tags": item.get("topics", []),
                "thumbnail": item.get("owner", {}).get("avatar_url", ""),
                "summary": (item.get("description") or "")[:100],
            })
        return results

    def search(self, query: str, limit: int = 20) -> list:
        return self.search_repos(query, limit)

    def search_code(self, query: str, language: str = "python", limit: int = 10) -> list:
        params = urllib.parse.urlencode({
            "q": f"{query} language:{language}",
            "per_page": min(limit, 30),
        })
        url = f"{self.BASE}/search/code?{params}"
        resp = _http_get(url, self._headers())
        if not resp:
            return []
        results = []
        for item in resp.get("items", []):
            results.append({
                "platform": "github_code",
                "name": item.get("name", ""),
                "path": item.get("path", ""),
                "repo": item.get("repository", {}).get("full_name", ""),
                "url": item.get("html_url", ""),
                "raw_url": item.get("git_url", "").replace("api.github.com/repos", "raw.githubusercontent.com").replace("/git/blobs/", "/"),
            })
        return results

    def get_raw_file(self, owner: str, repo: str, path: str, ref: str = "master") -> Optional[str]:
        url = f"https://raw.githubusercontent.com/{owner}/{repo}/{ref}/{path}"
        req = urllib.request.Request(url, headers={"User-Agent": UA})
        if GITHUB_TOKEN:
            req.add_header("Authorization", f"token {GITHUB_TOKEN}")
        try:
            with urllib.request.urlopen(req, timeout=15) as resp:
                return resp.read().decode("utf-8", errors="replace")
        except Exception:
            return None

    def probe(self) -> dict:
        resp = _http_get(f"{self.BASE}/rate_limit", self._headers())
        if not resp:
            return {"platform": "github", "status": "✗ 离线", "auth": ""}
        core = resp.get("resources", {}).get("core", {})
        remaining = core.get("remaining", 0)
        limit = core.get("limit", 60)
        auth = f"token={'已设置' if GITHUB_TOKEN else '未设置(60次/小时)'}"
        return {"platform": "github", "status": f"✅ 剩余配额 {remaining}/{limit}", "auth": auth}


class ThangsClient:
    """Thangs.com — 逆向得到的非官方API"""
    BASE = "https://thangs.com/api/v1"

    def search(self, query: str, limit: int = 20) -> list:
        params = urllib.parse.urlencode({"search": query, "per_page": limit, "page": 1})
        url = f"{self.BASE}/models?{params}"
        resp = _http_get(url)
        if not resp:
            return []
        items = resp if isinstance(resp, list) else resp.get("models", resp.get("data", resp.get("results", [])))
        results = []
        for item in items[:limit]:
            results.append({
                "platform": "thangs",
                "id": str(item.get("id", "")),
                "name": item.get("name", item.get("filename", "")),
                "author": item.get("owner", {}).get("username", "?") if isinstance(item.get("owner"), dict) else str(item.get("owner", "?")),
                "url": f"https://thangs.com/model/{item.get('id', '')}",
                "likes": item.get("likes", 0),
                "downloads": item.get("downloads", 0),
                "license": item.get("license", "?"),
                "tags": item.get("tags", []),
                "thumbnail": item.get("thumbnail", ""),
                "summary": (item.get("description") or "")[:100],
            })
        return results

    def probe(self) -> dict:
        resp = _http_get(f"{self.BASE}/models?per_page=1")
        return {"platform": "thangs", "status": "✅ 在线" if resp else "✗ 离线/端点变更", "auth": "无需认证"}


class ReadTheDocsClient:
    """ReadTheDocs 文档搜索API"""

    PROJECTS = {
        "cadquery": "https://cadquery.readthedocs.io/_/api/v2/search/?project=cadquery&version=latest",
        "build123d": "https://build123d.readthedocs.io/_/api/v2/search/?project=build123d&version=latest",
        "open3d": "https://www.open3d.org/docs/",
        "trimesh": "https://trimesh.org/",
    }

    def search(self, query: str, projects: list = None) -> list:
        if projects is None:
            projects = ["cadquery", "build123d"]
        results = []
        for proj in projects:
            base = self.PROJECTS.get(proj)
            if not base or "_/api/v2/search" not in base:
                continue
            url = f"{base}&q={urllib.parse.quote(query)}"
            resp = _http_get(url)
            if not resp:
                continue
            for item in resp.get("results", []):
                results.append({
                    "platform": f"docs:{proj}",
                    "name": item.get("title", ""),
                    "url": item.get("domain", "") + item.get("path", ""),
                    "summary": " ".join(b.get("content", "") for b in item.get("blocks", []))[:200],
                })
        return results

    def probe(self) -> dict:
        resp = _http_get("https://cadquery.readthedocs.io/_/api/v2/search/?project=cadquery&version=latest&q=box")
        return {"platform": "readthedocs", "status": "✅ 在线" if resp else "✗ 离线", "auth": "无需认证"}


class ThingiverClient:
    """Thingiverse REST API — 需要Bearer Token"""
    BASE = "https://api.thingiverse.com"

    def _headers(self):
        if not THINGIVERSE_TOKEN:
            return {}
        return {"Authorization": f"Bearer {THINGIVERSE_TOKEN}"}

    def search(self, query: str, limit: int = 20) -> list:
        if not THINGIVERSE_TOKEN:
            print("  ! Thingiverse需要Token (设置 THINGIVERSE_TOKEN 环境变量)")
            return []
        params = urllib.parse.urlencode({"per_page": limit, "page": 1, "type": "things"})
        url = f"{self.BASE}/search/{urllib.parse.quote(query)}?{params}"
        resp = _http_get(url, self._headers())
        if not resp or not isinstance(resp, list):
            return []
        results = []
        for item in resp[:limit]:
            results.append({
                "platform": "thingiverse",
                "id": str(item.get("id")),
                "name": item.get("name", ""),
                "author": item.get("creator", {}).get("name", "?"),
                "url": item.get("public_url", ""),
                "likes": item.get("like_count", 0),
                "downloads": item.get("download_count", 0),
                "license": item.get("license", "?"),
                "tags": [t.get("name") for t in item.get("tags", [])],
                "thumbnail": item.get("thumbnail", ""),
                "summary": (item.get("description") or "")[:100],
            })
        return results

    def probe(self) -> dict:
        auth = f"token={'已设置' if THINGIVERSE_TOKEN else '未设置(必须)'}"
        if not THINGIVERSE_TOKEN:
            return {"platform": "thingiverse", "status": "⚠ 需要Token", "auth": auth}
        resp = _http_get(f"{self.BASE}/newest?per_page=1", self._headers())
        return {"platform": "thingiverse", "status": "✅ 在线" if resp else "✗ 离线", "auth": auth}


class Cults3DClient:
    """Cults3D — HTML解析 (confirmed 200, 无官方API)"""
    BASE = "https://cults3d.com"

    def _fetch_html(self, url: str) -> bytes:
        import urllib.request as ur
        req = ur.Request(url, headers={
            "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 Chrome/124.0 Safari/537.36",
            "Accept": "text/html,*/*",
            "Accept-Language": "en-US,en;q=0.9",
            "Referer": "https://cults3d.com/",
        })
        try:
            with ur.urlopen(req, context=_SSL_CTX_NOVERIFY, timeout=15) as r:
                return r.read()
        except Exception:
            return b""

    def search(self, query: str, limit: int = 20) -> list:
        import re as _re
        url = f"{self.BASE}/en/search?q={urllib.parse.quote(query)}&page=1"
        html = self._fetch_html(url)
        if not html:
            return []
        results = []
        # Parse per-article: each <article> contains one model card
        articles = _re.findall(rb'<article[^>]*>(.*?)</article>', html, _re.DOTALL)
        for art in articles:
            if len(results) >= limit:
                break
            art_str = art.decode("utf-8", "replace")
            # URL + title from main anchor: href="/en/3d-model/..." title="..."
            m_href = _re.search(r'href="/en/3d-model/([^"]+)"', art_str)
            if not m_href:
                continue
            slug = m_href.group(1)
            if not slug or "/" not in slug:
                continue
            m_title = _re.search(r'title="([^"]+)"', art_str)
            name = m_title.group(1) if m_title else slug.split("/")[-1].replace("-", " ").title()
            # Author from profile link: /en/users/username or /en/user/username
            m_author = _re.search(r'href="/en/(?:users?|creator|profile)/([^/"]+)"', art_str)
            author = m_author.group(1) if m_author else "?"
            # Thumbnail from data-srcset (first HTTPS url)
            m_thumb = _re.search(r'data-srcset="(https://[^"]+)"', art_str)
            if not m_thumb:
                m_thumb = _re.search(r'src="(https://(?:fbi|files|videos)\.cults3d\.com/[^"]+)"', art_str)
            thumb = m_thumb.group(1).split(" ")[0] if m_thumb else ""
            results.append({
                "platform": "cults3d",
                "id": slug,
                "name": name,
                "author": author,
                "url": f"{self.BASE}/en/3d-model/{slug}",
                "likes": 0,
                "downloads": 0,
                "license": "?",
                "tags": [],
                "thumbnail": thumb,
                "summary": "",
            })
        return results

    def probe(self) -> dict:
        html = self._fetch_html(f"{self.BASE}/en/search?q=gear&page=1")
        if html and b"/3d-model/" in html:
            return {"platform": "cults3d", "status": "✅ HTML解析在线", "auth": "无需认证"}
        return {"platform": "cults3d", "status": "✗ 离线或结构变更", "auth": "无需认证"}


class ThreeD66Client:
    """3D溜溜 — HTML renderData解析 (中国平台, search/index.html)"""
    SEARCH_URL = "https://www.3d66.com/search/index.html"
    BASE = "https://www.3d66.com"

    def _fetch_html(self, url: str) -> bytes:
        import urllib.request as ur
        req = ur.Request(url, headers={
            "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 Chrome/124.0 Safari/537.36",
            "Accept": "text/html,*/*",
            "Accept-Language": "zh-CN,zh;q=0.9,en;q=0.8",
            "Referer": "https://www.3d66.com/",
        })
        try:
            with ur.urlopen(req, context=_SSL_CTX_NOVERIFY, timeout=15) as r:
                return r.read()
        except Exception:
            return b""

    def search(self, query: str, limit: int = 20) -> list:
        import re as _re
        url = f"{self.SEARCH_URL}?keyword={urllib.parse.quote(query)}"
        html = self._fetch_html(url)
        if not html:
            return []
        results = []
        # Try renderData textarea
        m = _re.search(rb'id=["\']renderData["\'][^>]*>(.*?)</textarea>', html, _re.DOTALL)
        if m:
            raw = m.group(1).decode("utf-8", "replace")
            try:
                data = json.loads(raw)
                items = data.get("list", data.get("data", data.get("models", [])))
                if isinstance(items, list):
                    for item in items[:limit]:
                        mid = str(item.get("id", item.get("model_id", "")))
                        name = item.get("name", item.get("title", item.get("model_name", mid)))
                        results.append({
                            "platform": "3d66",
                            "id": mid,
                            "name": name,
                            "author": item.get("author", item.get("username", "?")),
                            "url": f"{self.BASE}/3dxz/{mid}.html" if mid else self.BASE,
                            "likes": item.get("like_count", item.get("like", 0)),
                            "downloads": item.get("download_count", item.get("down", 0)),
                            "license": "商业",
                            "tags": item.get("tags", []),
                            "thumbnail": item.get("thumb", item.get("image", item.get("img_url", ""))),
                            "summary": (item.get("desc", item.get("description", "")) or "")[:100],
                        })
                    return results
            except (json.JSONDecodeError, Exception):
                pass
        # Fallback: extract from HTML links
        model_links = _re.findall(rb'href="(/3dxz/([0-9]+)\.html)"', html)
        model_names  = _re.findall(rb'title="([^"]{2,80})"', html)
        thumbs = _re.findall(rb'data-src="(https?://[^"]+(?:jpg|png|webp))"', html)
        for i, (link, mid) in enumerate(model_links[:limit]):
            name = model_names[i].decode("utf-8", "replace") if i < len(model_names) else f"Model {mid.decode()}"
            thumb = thumbs[i].decode("utf-8", "replace") if i < len(thumbs) else ""
            results.append({
                "platform": "3d66",
                "id": mid.decode("utf-8"),
                "name": name,
                "author": "?",
                "url": f"{self.BASE}{link.decode('utf-8')}",
                "likes": 0,
                "downloads": 0,
                "license": "商业",
                "tags": [],
                "thumbnail": thumb,
                "summary": "",
            })
        return results

    def probe(self) -> dict:
        html = self._fetch_html(f"{self.SEARCH_URL}?keyword=gear")
        if html and (b"renderData" in html or b"3dxz" in html):
            return {"platform": "3d66", "status": "✅ HTML解析在线", "auth": "无需认证"}
        return {"platform": "3d66", "status": "✗ 离线或结构变更", "auth": "无需认证"}


class STLFinderClient:
    """STLFinder聚合搜索引擎 — HTML解析 (聚合Thingiverse/MMF/Printables等)"""
    BASE = "https://www.stlfinder.com"

    def _fetch(self, url: str) -> bytes:
        import urllib.request as ur
        req = ur.Request(url, headers={
            "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) Chrome/124.0 Safari/537.36",
            "Accept": "text/html,*/*",
            "Accept-Language": "en-US,en;q=0.9",
            "Referer": "https://www.stlfinder.com/",
        })
        try:
            with ur.urlopen(req, context=_SSL_CTX_NOVERIFY, timeout=15) as r:
                return r.read()
        except Exception:
            return b""

    def search(self, query: str, limit: int = 20) -> list:
        import re as _re
        url = f"{self.BASE}/models/?q={urllib.parse.quote(query)}&page=1"
        html = self._fetch(url)
        if not html:
            return []
        results = []
        # Extract model cards: href="/model/slug/"
        slugs = _re.findall(rb'href="(/model/[a-z0-9_\-]+/)"', html)
        names = _re.findall(rb'class="[^"]*model-title[^"]*"[^>]*>([^<]+)<', html)
        sources = _re.findall(rb'class="[^"]*model-source[^"]*"[^>]*>([^<]+)<', html)
        thumbs = _re.findall(rb'<img[^>]+src="(https://[^"]+(?:jpg|png|webp|jpeg))"[^>]*class="[^"]*model', html)
        seen = []
        for s in slugs:
            dec = s.decode()
            if dec not in seen:
                seen.append(dec)
        for i, slug in enumerate(seen[:limit]):
            name = names[i].decode("utf-8", "replace").strip() if i < len(names) else slug
            source = sources[i].decode("utf-8", "replace").strip() if i < len(sources) else "?"
            thumb = thumbs[i].decode("utf-8", "replace") if i < len(thumbs) else ""
            results.append({
                "platform": "stlfinder",
                "id": slug.strip("/").split("/")[-1],
                "name": name,
                "author": source,
                "url": f"{self.BASE}{slug}",
                "likes": 0,
                "downloads": 0,
                "license": "?",
                "tags": [],
                "thumbnail": thumb,
                "summary": f"来源: {source}",
            })
        return results

    def probe(self) -> dict:
        html = self._fetch(f"{self.BASE}/models/?q=gear")
        if html and b"/model/" in html:
            return {"platform": "stlfinder", "status": "✅ HTML解析在线", "auth": "无需认证"}
        return {"platform": "stlfinder", "status": "✗ 403/离线 (CloudFlare保护)", "auth": "无需认证"}


class YeggiClient:
    """Yeggi — 40M+模型元搜索引擎 (HTML解析, 无官方API)
    聚合来源: Thingiverse / Printables / MyMiniFactory / Cults3D 等40+站"""
    BASE = "https://www.yeggi.com"

    def _fetch(self, url: str) -> bytes:
        import urllib.request as ur
        req = ur.Request(url, headers={
            "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 Chrome/124.0 Safari/537.36",
            "Accept": "text/html,*/*",
            "Accept-Language": "en-US,en;q=0.9",
            "Referer": "https://www.yeggi.com/",
        })
        try:
            with ur.urlopen(req, context=_SSL_CTX_NOVERIFY, timeout=15) as r:
                return r.read()
        except Exception:
            return b""

    def search(self, query: str, limit: int = 20) -> list:
        import re as _re
        url = f"{self.BASE}/q/{urllib.parse.quote(query)}/"
        html = self._fetch(url)
        if not html:
            return []
        results = []
        # JSON-LD结构化数据 (最可靠)
        json_ld = _re.findall(rb'<script type="application/ld\+json">(.*?)</script>', html, _re.DOTALL)
        for block in json_ld:
            try:
                data = json.loads(block)
                items = data if isinstance(data, list) else data.get("itemListElement", [])
                for item in items:
                    if len(results) >= limit:
                        break
                    if isinstance(item, dict):
                        name = item.get("name", item.get("item", {}).get("name", ""))
                        ext_url = item.get("url", item.get("item", {}).get("url", ""))
                        if name and ext_url:
                            results.append({
                                "platform": "yeggi",
                                "id": str(len(results)),
                                "name": name,
                                "author": "?",
                                "url": ext_url,
                                "likes": 0, "downloads": 0, "license": "?",
                                "tags": [], "thumbnail": "",
                                "summary": "via Yeggi 元搜索",
                            })
            except Exception:
                continue
        if results:
            return results[:limit]
        # Fallback: href到已知3D平台的链接
        ext_links = _re.findall(
            rb'href="(https?://(?:www\.thingiverse\.com|www\.printables\.com|cults3d\.com|www\.myminifactory\.com|grabcad\.com|sketchfab\.com)[^"]+)"[^>]*>\s*([^<]{3,80})',
            html)
        sources = _re.findall(rb'class="[^"]*source[^"]*"[^>]*>([^<]{2,40})<', html)
        thumbs  = _re.findall(rb'(?:data-src|src)="(https?://[^"]+\.(?:jpg|jpeg|png|webp))"', html)
        for i, (ext_url, title) in enumerate(ext_links[:limit]):
            ext_url_s = ext_url.decode("utf-8", "replace")
            title_s   = title.decode("utf-8", "replace").strip()
            source    = sources[i].decode("utf-8", "replace").strip() if i < len(sources) else "?"
            thumb     = thumbs[i].decode("utf-8", "replace")          if i < len(thumbs)  else ""
            results.append({
                "platform": "yeggi",
                "id": str(i),
                "name": title_s,
                "author": source,
                "url": ext_url_s,
                "likes": 0, "downloads": 0, "license": "?",
                "tags": [], "thumbnail": thumb,
                "summary": f"via Yeggi · 来源: {source}",
            })
        return results[:limit]

    def probe(self) -> dict:
        html = self._fetch(f"{self.BASE}/q/gear/")
        if html and b"turnstile" in html.lower():
            return {"platform": "yeggi", "status": "⚠ CloudFlare Turnstile拦截 (需浏览器)", "auth": "无需认证"}
        if html and len(html) > 8000:
            return {"platform": "yeggi", "status": "✅ HTML在线 (元搜索40M+)", "auth": "无需认证"}
        return {"platform": "yeggi", "status": "✗ 离线或结构变更", "auth": "无需认证"}


class MohouClient:
    """魔猴.com — 中国工程3D模型平台 (HTML解析)"""
    BASE = "https://www.mohou.com"

    def _fetch(self, url: str) -> bytes:
        import urllib.request as ur
        req = ur.Request(url, headers={
            "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 Chrome/124.0 Safari/537.36",
            "Accept": "text/html,*/*",
            "Accept-Language": "zh-CN,zh;q=0.9,en;q=0.8",
            "Referer": "https://www.mohou.com/",
        })
        try:
            with ur.urlopen(req, context=_SSL_CTX_NOVERIFY, timeout=15) as r:
                return r.read()
        except Exception:
            return b""

    def search(self, query: str, limit: int = 20) -> list:
        import re as _re
        url = f"{self.BASE}/models/list?keyword={urllib.parse.quote(query)}&page=1"
        html = self._fetch(url)
        if not html:
            return []
        results = []
        # 尝试window.__NUXT__/window.__DATA__ JSON嵌入
        for pat in [
            rb'window\.__NUXT__\s*=\s*(\{.*?\});\s*</script>',
            rb'window\.__DATA__\s*=\s*(\{.*?\});',
            rb'<script[^>]*>\s*var\s+pageData\s*=\s*(\{.*?\});',
        ]:
            m = _re.search(pat, html, _re.DOTALL)
            if m:
                try:
                    data = json.loads(m.group(1))
                    # 递归寻找模型列表
                    def _find_list(d, depth=0):
                        if depth > 5: return None
                        if isinstance(d, list) and d and isinstance(d[0], dict) and "id" in d[0]:
                            return d
                        if isinstance(d, dict):
                            for k, v in d.items():
                                r = _find_list(v, depth+1)
                                if r: return r
                        return None
                    items = _find_list(data)
                    if items:
                        for item in items[:limit]:
                            mid = str(item.get("id", item.get("model_id", "")))
                            name = item.get("name", item.get("title", mid))
                            user = item.get("user", {})
                            author = user.get("name", "?") if isinstance(user, dict) else str(user) if user else "?"
                            results.append({
                                "platform": "mohou",
                                "id": mid,
                                "name": name,
                                "author": author,
                                "url": f"{self.BASE}/models/detail/{mid}" if mid else self.BASE,
                                "likes": item.get("like_count", 0),
                                "downloads": item.get("download_count", 0),
                                "license": "商业",
                                "tags": item.get("tags", []),
                                "thumbnail": item.get("cover", item.get("image", "")),
                                "summary": (item.get("desc", "") or "")[:100],
                            })
                        return results
                except Exception:
                    pass
        # Fallback: 链接提取
        model_links = _re.findall(rb'href="(/models/detail/([0-9]+))"', html)
        titles = _re.findall(rb'title="([^"]{2,80})"', html)
        thumbs = _re.findall(rb'(?:data-src|src)="(https?://[^"]+\.(?:jpg|png|webp))"', html)
        for i, (link, mid) in enumerate(model_links[:limit]):
            name  = titles[i].decode("utf-8", "replace").strip() if i < len(titles) else f"Model {mid.decode()}"
            thumb = thumbs[i].decode("utf-8", "replace") if i < len(thumbs) else ""
            results.append({
                "platform": "mohou",
                "id": mid.decode("utf-8"),
                "name": name,
                "author": "?",
                "url": f"{self.BASE}{link.decode('utf-8')}",
                "likes": 0, "downloads": 0, "license": "商业",
                "tags": [], "thumbnail": thumb, "summary": "",
            })
        return results

    def probe(self) -> dict:
        html = self._fetch(f"{self.BASE}/models/list?keyword=gear")
        if html and len(html) > 3000 and b"mohou" in html.lower():
            return {"platform": "mohou", "status": "✅ HTML在线 (搜索需AJAX/token)", "auth": "无需认证"}
        return {"platform": "mohou", "status": "✗ 离线或结构变更", "auth": "无需认证"}


# ─── 动态载入Playwright平台 ───────────────────────────────────────────────────
try:
    from _playwright_scrapers import PLAYWRIGHT_PLATFORMS as _PW_PLATFORMS
    _HAS_PW = True
except ImportError:
    _PW_PLATFORMS = {}
    _HAS_PW = False


# ─── 平台注册表 ───────────────────────────────────────────────────────────────
PLATFORMS = {
    "printables":  PrintablesClient(),
    "mmf":         MyMiniFactoryClient(),
    "nih":         NIHClient(),
    "nasa":        NASAClient(),
    "sketchfab":   SketchfabClient(),
    "github":      GitHubClient(),
    "thangs":      ThangsClient(),
    "rtd":         ReadTheDocsClient(),
    "thingiverse": ThingiverClient(),
    "cults3d":     Cults3DClient(),
    "3d66":        ThreeD66Client(),
    "stlfinder":   STLFinderClient(),
    "yeggi":       YeggiClient(),
    "mohou":       MohouClient(),
    **_PW_PLATFORMS,
}

ALL_SEARCH_PLATFORMS = [
    "printables", "sketchfab", "cults3d", "yeggi",
    "nasa", "github", "3d66", "mohou",
    "mmf", "nih", "thangs", "thingiverse",
    "stlfinder",
] + list(_PW_PLATFORMS.keys())


# ─── 命令实现 ─────────────────────────────────────────────────────────────────

def _print_results(results: list, verbose: bool = False):
    if not results:
        print("  (无结果)")
        return
    for i, r in enumerate(results, 1):
        name = r.get("name", "?")[:60]
        plat = r.get("platform", "?")
        auth = r.get("author", "?")
        likes = r.get("likes", 0)
        dl = r.get("downloads", 0)
        url = r.get("url", "")
        print(f"  {i:2}. [{plat}] {name}")
        print(f"      作者:{auth}  ★{likes}  ↓{dl}  {url[:80]}")
        if verbose and r.get("summary"):
            print(f"      {r['summary'][:120]}")


def _do_search_one(plat_name: str, client, query: str, limit: int) -> tuple:
    """单平台搜索 (供并行调用)"""
    try:
        if plat_name == "rtd":
            return plat_name, client.search(query)
        elif plat_name == "github":
            return plat_name, client.search_repos(query, limit)
        else:
            return plat_name, client.search(query, limit)
    except Exception as e:
        return plat_name, []


def cmd_search(args):
    """搜索3D模型"""
    query = " ".join(args.query)
    platforms = args.platform.split(",") if args.platform != "all" else ALL_SEARCH_PLATFORMS
    limit = args.limit
    parallel = getattr(args, "parallel", False)

    print(f"\n🔍 搜索: '{query}'  平台: {', '.join(platforms)}{' [并行]' if parallel else ''}\n")

    all_results = []

    if parallel:
        import concurrent.futures
        workers = min(8, len(platforms))
        with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
            futures = {
                ex.submit(_do_search_one, p, PLATFORMS[p], query, limit): p
                for p in platforms if p in PLATFORMS
            }
            results_map = {}
            for future in concurrent.futures.as_completed(futures):
                plat_name, results = future.result()
                results_map[plat_name] = results
        for plat_name in platforms:
            results = results_map.get(plat_name, [])
            print(f"  ▶ {plat_name} ({len(results)}个结果):")
            _print_results(results, verbose=args.verbose)
            all_results.extend(results)
            print()
    else:
        for plat_name in platforms:
            client = PLATFORMS.get(plat_name)
            if not client:
                print(f"  ! 未知平台: {plat_name}")
                continue
            _, results = _do_search_one(plat_name, client, query, limit)
            print(f"  ▶ {plat_name} ({len(results)}个结果):")
            _print_results(results, verbose=args.verbose)
            all_results.extend(results)
            print()

    if args.save:
        out_path = Path(args.save)
        out_path.write_text(json.dumps(all_results, ensure_ascii=False, indent=2))
        print(f"💾 已保存 {len(all_results)} 条结果 → {out_path}")

    return all_results


def cmd_download(args):
    """下载模型文件"""
    plat_name = args.platform
    model_id = args.id
    out_dir = Path(args.out) if args.out else DOWNLOAD_DIR

    client = PLATFORMS.get(plat_name)
    if not client:
        print(f"✗ 未知平台: {plat_name}")
        return

    print(f"\n⬇ 下载: [{plat_name}] id={model_id} → {out_dir}")

    if hasattr(client, "download"):
        files = client.download(model_id, out_dir)
        if files:
            print(f"✅ 下载完成: {len(files)} 个文件")
            for f in files:
                print(f"   {f}")
        else:
            print("✗ 下载失败或无文件")
    else:
        print(f"✗ 平台 {plat_name} 暂不支持下载")


def cmd_probe(args):
    """探测各平台API状态"""
    platforms = args.platform.split(",") if args.platform != "all" else list(PLATFORMS.keys())
    print(f"\n🔬 探针诊断 — {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    results = []
    for name in platforms:
        client = PLATFORMS.get(name)
        if not client:
            continue
        if hasattr(client, "probe"):
            r = client.probe()
            print(f"  {r['status']:20s} [{r['platform']}]  {r.get('auth','')}")
            results.append(r)
    print()
    online = sum(1 for r in results if "✅" in r.get("status", ""))
    print(f"  在线: {online}/{len(results)}")


def cmd_sync_libs(args):
    """同步GitHub核心3D建模库元数据"""
    libs = [
        ("BelfrySCAD", "BOSL2"),
        ("openscad", "MCAD"),
        ("nophead", "NopSCADlib"),
        ("CadQuery", "cadquery"),
        ("gumyr", "build123d"),
        ("mikedh", "trimesh"),
        ("SolidCode", "SolidPython"),
        ("isl-org", "Open3D"),
        ("JustinSDK", "dotSCAD"),
        ("Open-Cascade-SAS", "OCCT"),
    ]

    client = PLATFORMS["github"]
    print(f"\n📦 同步 {len(libs)} 个核心库元数据...\n")
    meta = []
    for owner, repo in libs:
        url = f"https://api.github.com/repos/{owner}/{repo}"
        resp = _http_get(url, client._headers())
        if resp:
            entry = {
                "name": resp.get("full_name"),
                "description": resp.get("description", ""),
                "stars": resp.get("stargazers_count", 0),
                "forks": resp.get("forks_count", 0),
                "language": resp.get("language", ""),
                "license": resp.get("license", {}).get("spdx_id", "?") if resp.get("license") else "?",
                "url": resp.get("html_url"),
                "clone_url": resp.get("clone_url"),
                "updated": resp.get("updated_at", ""),
            }
            meta.append(entry)
            print(f"  ✅ {entry['name']:40s} ★{entry['stars']:6d}  {entry['description'][:50]}")
        else:
            print(f"  ✗  {owner}/{repo}")
        time.sleep(0.5)

    out = CACHE_DIR / "libs_meta.json"
    out.write_text(json.dumps(meta, ensure_ascii=False, indent=2))
    print(f"\n💾 元数据已缓存 → {out}")
    return meta


def cmd_docs(args):
    """搜索文档"""
    query = " ".join(args.query)
    projects = args.projects.split(",") if args.projects else ["cadquery", "build123d"]
    print(f"\n📖 文档搜索: '{query}'  项目: {', '.join(projects)}\n")
    client = PLATFORMS["rtd"]
    results = client.search(query, projects)
    for r in results:
        print(f"  [{r['platform']}] {r['name']}")
        print(f"    {r['url']}")
        if r.get("summary"):
            print(f"    {r['summary'][:150]}")
        print()
    if not results:
        print("  (无结果 — 部分文档站不支持搜索API)")


def cmd_github_code(args):
    """在GitHub代码中搜索3D建模示例"""
    query = " ".join(args.query)
    lang = args.lang or "python"
    print(f"\n💻 GitHub代码搜索: '{query}' (language={lang})\n")
    client = PLATFORMS["github"]
    results = client.search_code(query, lang, 10)
    for r in results:
        print(f"  [{r['repo']}] {r['name']}")
        print(f"    {r['url']}")
    if not results:
        print("  (无结果)")


def cmd_batch(args):
    """批量分页拉取 (多页结果汇总)"""
    query = " ".join(args.query)
    platforms = args.platform.split(",") if args.platform != "all" else ["printables", "sketchfab", "cults3d"]
    pages = args.pages
    limit_pp = args.limit
    print(f"\n\U0001f4e6 批量拉取: '{query}'  平台: {', '.join(platforms)}  {pages}页x{limit_pp}/页\n")
    all_results = []
    for plat_name in platforms:
        client = PLATFORMS.get(plat_name)
        if not client:
            print(f"  ! 未知平台: {plat_name}")
            continue
        plat_results = []
        for pg in range(1, pages + 1):
            if plat_name == "printables":
                payload = {
                    "query": client.SEARCH_QUERY,
                    "variables": {"query": query, "limit": limit_pp, "offset": (pg - 1) * limit_pp}
                }
                resp = _http_post_json(client.ENDPOINT, payload)
                items = resp.get("data", {}).get("searchPrints2", {}).get("items", []) if resp else []
                batch = []
                for item in items:
                    batch.append({
                        "platform": "printables",
                        "id": item.get("id"),
                        "name": item.get("name"),
                        "author": item.get("user", {}).get("publicUsername", "?"),
                        "url": f"https://www.printables.com/model/{item.get('id')}-{item.get('slug','')}",
                        "likes": item.get("likesCount", 0),
                        "downloads": item.get("downloadCount", 0),
                        "license": item.get("license", {}).get("abbreviation", "?") if item.get("license") else "?",
                        "tags": [t["name"] for t in item.get("tags", [])],
                        "thumbnail": item.get("image", {}).get("filePath", ""),
                        "summary": (item.get("summary") or "")[:100],
                    })
            else:
                batch = client.search(query, limit_pp) if pg == 1 else []
            plat_results.extend(batch)
            print(f"  \u25b6 {plat_name} 第{pg}页: {len(batch)}结果")
            if len(batch) < limit_pp:
                break
            time.sleep(0.3)
        all_results.extend(plat_results)
        print(f"  小计 {plat_name}: {len(plat_results)}条\n")
    print(f"\U0001f4ca 汇总: {len(all_results)} 条结果 (来自 {len(platforms)} 个平台 \u00d7 {pages} 页)")
    if args.save:
        out_path = Path(args.save)
        out_path.write_text(json.dumps(all_results, ensure_ascii=False, indent=2))
        print(f"\U0001f4be 已保存 \u2192 {out_path}")
    return all_results


def cmd_report(args):
    """生成资源状态报告"""
    print("\n" + "=" * 60)
    print("  世界3D建模资源报告 · 道法自然")
    print(f"  生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)

    print("\n■ 平台状态")
    for name in PLATFORMS:
        client = PLATFORMS[name]
        if hasattr(client, "probe"):
            r = client.probe()
            print(f"  {r['status']:25s} [{r['platform']}]  {r.get('auth','')}")

    print("\n■ 缓存文件")
    for f in CACHE_DIR.glob("*.json"):
        size = f.stat().st_size
        mtime = datetime.fromtimestamp(f.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
        print(f"  {f.name:30s} {size//1024:4d}KB  {mtime}")

    print("\n■ 已下载模型")
    count = sum(1 for _ in DOWNLOAD_DIR.rglob("*.stl"))
    count += sum(1 for _ in DOWNLOAD_DIR.rglob("*.obj"))
    count += sum(1 for _ in DOWNLOAD_DIR.rglob("*.step"))
    print(f"  共 {count} 个模型文件  路径: {DOWNLOAD_DIR}")

    print("\n■ 认证状态")
    tokens = {
        "THINGIVERSE_TOKEN": THINGIVERSE_TOKEN,
        "MMF_API_KEY": MMF_API_KEY,
        "SKETCHFAB_TOKEN": SKETCHFAB_TOKEN,
        "GITHUB_TOKEN": GITHUB_TOKEN,
    }
    for k, v in tokens.items():
        status = "✅ 已设置" if v else "○ 未设置"
        print(f"  {status}  {k}")

    print("\n■ 资源索引")
    idx = _dao_paths.WORLD / "网络资源库" / "世界资源大全.md"
    if idx.exists():
        lines = len(idx.read_text().splitlines())
        print(f"  世界资源大全.md  {lines}行  {idx.stat().st_size//1024}KB")

    print("\n" + "=" * 60)
    print("  道法自然 · 万法归宗 · 反者道之动")
    print("=" * 60 + "\n")


# ─── CLI入口 ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        prog="资源探针",
        description="世界3D建模资源连通器 · 道法自然",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    # search
    p_search = sub.add_parser("search", help="搜索3D模型")
    p_search.add_argument("query", nargs="+", help="搜索关键词")
    p_search.add_argument("--platform", "-p", default="all",
                          help="平台: all|printables|sketchfab|cults3d|yeggi|nasa|github|3d66|mohou|mmf|等 (逗号分隔)")
    p_search.add_argument("--limit", "-n", type=int, default=10, help="每平台结果数 (默认10)")
    p_search.add_argument("--verbose", "-v", action="store_true", help="显示摘要")
    p_search.add_argument("--save", "-s", help="保存结果到JSON文件")
    p_search.add_argument("--parallel", action="store_true", help="并行搜索所有平台 (速度快8x)")

    # batch
    p_batch = sub.add_parser("batch", help="批量分页拉取")
    p_batch.add_argument("query", nargs="+", help="搜索关键词")
    p_batch.add_argument("--platform", "-p", default="printables,sketchfab,cults3d", help="平台 (逗号分隔)")
    p_batch.add_argument("--pages", type=int, default=3, help="每平台拉取页数 (默认3)")
    p_batch.add_argument("--limit", "-n", type=int, default=20, help="每页结果数 (默认20)")
    p_batch.add_argument("--save", "-s", help="保存结果到JSON文件")

    # download
    p_dl = sub.add_parser("download", help="下载模型文件")
    p_dl.add_argument("--platform", "-p", required=True, help="平台名")
    p_dl.add_argument("--id", required=True, help="模型ID")
    p_dl.add_argument("--out", "-o", help="输出目录 (默认: downloads/)")

    # probe
    p_probe = sub.add_parser("probe", help="探测API状态")
    p_probe.add_argument("--platform", "-p", default="all", help="平台 (逗号分隔，默认all)")

    # sync-libs
    sub.add_parser("sync-libs", help="同步GitHub核心库元数据")

    # docs
    p_docs = sub.add_parser("docs", help="搜索文档")
    p_docs.add_argument("query", nargs="+", help="关键词")
    p_docs.add_argument("--projects", help="项目: cadquery,build123d (逗号分隔)")

    # github-code
    p_gc = sub.add_parser("github-code", help="GitHub代码搜索")
    p_gc.add_argument("query", nargs="+", help="关键词")
    p_gc.add_argument("--lang", help="语言 (默认: python)")

    # report
    sub.add_parser("report", help="生成资源报告")

    args = parser.parse_args()

    dispatch = {
        "search": cmd_search,
        "batch": cmd_batch,
        "download": cmd_download,
        "probe": cmd_probe,
        "sync-libs": cmd_sync_libs,
        "docs": cmd_docs,
        "github-code": cmd_github_code,
        "report": cmd_report,
    }
    fn = dispatch.get(args.cmd)
    if fn:
        fn(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
