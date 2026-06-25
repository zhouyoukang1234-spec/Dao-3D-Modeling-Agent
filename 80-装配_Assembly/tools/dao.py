# -*- coding: utf-8 -*-
"""Minimal client for the DAO Bridge (remote operation of the user's PC).

The cloudflared tunnel URL rotates; per the knowledge note the *current* URL is
re-injected there.  If health fails, re-read the note for a fresh URL.
"""
import urllib.request, json, ssl, os, sys

for k in ('HTTP_PROXY', 'HTTPS_PROXY', 'http_proxy', 'https_proxy'):
    os.environ.pop(k, None)
os.environ['NO_PROXY'] = '*'
_ctx = ssl.create_default_context()
_ctx.check_hostname = False
_ctx.verify_mode = ssl.CERT_NONE
urllib.request.install_opener(urllib.request.build_opener(
    urllib.request.ProxyHandler({}), urllib.request.HTTPSHandler(context=_ctx)))

URL = os.environ.get("DAO_URL", "https://raise-wiring-browser-writers.trycloudflare.com")
TOKEN = os.environ.get("DAO_TOKEN", "ba799706f9878b2f44e384e7aa21e69805f9308e11556fca")


def api(method, path, body=None, timeout=60):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(
        f"{URL}{path}", data=data,
        headers={"Authorization": f"Bearer {TOKEN}", "Content-Type": "application/json"},
        method=method)
    return json.loads(urllib.request.urlopen(req, timeout=timeout).read())


def exec_cmd(cmd, timeout=120):
    return api("POST", "/api/exec", {"cmd": cmd}, timeout=timeout)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        print(json.dumps(exec_cmd(" ".join(sys.argv[1:])), ensure_ascii=False, indent=2))
    else:
        print(json.dumps(api("GET", "/api/health"), ensure_ascii=False, indent=2))
