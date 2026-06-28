"""Minimal MCP (Model Context Protocol) server over stdio.

Exposes the whole CAD tool registry to any MCP client (Claude Desktop, Cursor,
the DAO bridge) as discoverable, callable tools — no third-party MCP SDK needed,
just JSON-RPC 2.0 framed one object per line on stdin/stdout.

Run:  python -m cad_agent.mcp_server          # live FreeCAD kernel
      DAO_MOCK=1 python -m cad_agent.mcp_server  # FreeCAD-free mock
"""
from __future__ import annotations

import json
import os
import sys
from typing import Any, Dict, Optional

PROTOCOL_VERSION = "2024-11-05"


def _build_registry():
    if os.environ.get("DAO_MOCK") == "1":
        from .backends.mock_backend import build_mock_registry
        return build_mock_registry()
    from . import build_freecad_registry
    return build_freecad_registry()


def _tool_to_mcp(t: Dict[str, Any]) -> Dict[str, Any]:
    schema = t.get("schema") or {}
    if "type" not in schema:
        schema = {"type": "object", "properties": schema.get("properties", {}),
                  "additionalProperties": True}
    return {"name": t["name"],
            "description": t.get("summary") or t["name"],
            "inputSchema": schema}


class MCPServer:
    def __init__(self, registry) -> None:
        self.registry = registry
        self.initialized = False

    def handle(self, req: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        method = req.get("method")
        rid = req.get("id")
        params = req.get("params") or {}
        if method == "initialize":
            return self._ok(rid, {
                "protocolVersion": PROTOCOL_VERSION,
                "capabilities": {"tools": {"listChanged": False}},
                "serverInfo": {"name": "dao-freecad-agent", "version": "1.0.0"},
            })
        if method == "notifications/initialized":
            self.initialized = True
            return None
        if method == "tools/list":
            tools = [_tool_to_mcp(t) for t in self.registry.manifest()]
            return self._ok(rid, {"tools": tools})
        if method == "tools/call":
            name = params.get("name")
            args = params.get("arguments") or {}
            result = self.registry.call(name, args)
            payload = json.dumps(result.to_dict(), ensure_ascii=False, indent=2)
            return self._ok(rid, {
                "content": [{"type": "text", "text": payload}],
                "isError": not result.ok,
            })
        if method == "ping":
            return self._ok(rid, {})
        return self._err(rid, -32601, "method not found: %s" % method)

    @staticmethod
    def _ok(rid: Any, result: Dict[str, Any]) -> Dict[str, Any]:
        return {"jsonrpc": "2.0", "id": rid, "result": result}

    @staticmethod
    def _err(rid: Any, code: int, message: str) -> Dict[str, Any]:
        return {"jsonrpc": "2.0", "id": rid, "error": {"code": code, "message": message}}


def main() -> None:
    registry = _build_registry()
    server = MCPServer(registry)
    out = sys.stdout
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except json.JSONDecodeError:
            continue
        resp = server.handle(req)
        if resp is not None:
            out.write(json.dumps(resp, ensure_ascii=False) + "\n")
            out.flush()


if __name__ == "__main__":
    main()
