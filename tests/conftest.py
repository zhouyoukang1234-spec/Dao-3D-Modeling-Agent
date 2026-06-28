import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _freecad_available() -> bool:
    if os.environ.get("DAO_MOCK") == "1":
        return False
    try:
        from cad_agent.backends.freecad_backend import find_freecadcmd
        return find_freecadcmd() is not None
    except Exception:
        return False


requires_freecad = pytest.mark.skipif(
    not _freecad_available(),
    reason="freecadcmd not available (set FREECADCMD or install FreeCAD)",
)


@pytest.fixture(scope="session")
def freecad_session():
    from cad_agent import new_session
    s = new_session("pytest")
    yield s
    try:
        s.registry.kernel.shutdown()
    except Exception:
        pass
