"""MCP server protocol tests using the FreeCAD-free mock registry."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent.backends.mock_backend import build_mock_registry  # noqa: E402
from cad_agent.mcp_server import MCPServer  # noqa: E402


def _srv():
    return MCPServer(build_mock_registry())


def test_initialize():
    r = _srv().handle({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {}})
    assert r["result"]["serverInfo"]["name"] == "dao-freecad-agent"
    assert "protocolVersion" in r["result"]


def test_tools_list():
    r = _srv().handle({"jsonrpc": "2.0", "id": 2, "method": "tools/list"})
    names = [t["name"] for t in r["result"]["tools"]]
    assert "solid.box" in names
    for t in r["result"]["tools"]:
        assert t["inputSchema"]["type"] == "object"


def test_tools_call_success():
    s = _srv()
    r = s.handle({"jsonrpc": "2.0", "id": 3, "method": "tools/call",
                  "params": {"name": "solid.box",
                             "arguments": {"name": "b", "length": 2, "width": 2, "height": 2}}})
    assert r["result"]["isError"] is False
    assert '"volume": 8' in r["result"]["content"][0]["text"]


def test_tools_call_error():
    s = _srv()
    r = s.handle({"jsonrpc": "2.0", "id": 4, "method": "tools/call",
                  "params": {"name": "solid.measure", "arguments": {"name": "ghost"}}})
    assert r["result"]["isError"] is True


def test_unknown_method():
    r = _srv().handle({"jsonrpc": "2.0", "id": 5, "method": "bogus"})
    assert r["error"]["code"] == -32601


def test_initialized_notification_has_no_response():
    assert _srv().handle({"jsonrpc": "2.0", "method": "notifications/initialized"}) is None
