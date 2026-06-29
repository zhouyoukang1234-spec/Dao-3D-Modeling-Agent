"""Spreadsheet (``ss.*``) state/arg guard smoke.

Practice exposed that ``ss.set``/``ss.bind``/``ss.table`` read ``state._sheet``
directly, so calling them before ``ss.create`` leaked a bare
``AttributeError: 'KernelState' object has no attribute '_sheet'`` instead of a
guided message. ``ss.set`` also leaked a bare ``KeyError`` for an unknown alias,
and ``ss.create`` leaked a ``TypeError`` for a non-string name. They now refuse
with a guided ``ValueError`` while the valid drive-by-spreadsheet flow keeps
working.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cad_agent import new_session  # noqa: E402


def _bad(r, token):
    err = r.error or ""
    assert not r.ok, "expected failure, got %r" % (r.data,)
    for raw in ("AttributeError", "TypeError", "KeyError"):
        assert raw not in err, "leaked raw %s: %r" % (raw, err)
    assert token in err, "error %r lacks %r" % (err, token)


def main():
    s = new_session("ss_guards")
    print("FreeCAD", s.registry.kernel.freecad_version)

    # ---- ss.* before ss.create must point at the missing ss.create, not leak
    #      a bare AttributeError on the absent state._sheet ------------------ #
    _bad(s.act("ss.set", {"alias": "t", "value": 9}), "call ss.create first")
    _bad(s.act("ss.bind", {"param": "X.length", "alias": "t"}), "call ss.create first")
    _bad(s.act("ss.table", {}), "call ss.create first")
    # a non-string sheet name used to leak 'TypeError: argument 2 must be str'.
    _bad(s.act("ss.create", {"name": 123, "cells": {"t": 5}}), "must be a string")
    print("ss.* pre-create / bad-name calls all refused cleanly")

    # ---- valid spreadsheet flow still works ------------------------------- #
    assert s.act("ss.create", {"cells": {"thickness": 5, "hole": 3}}).ok
    # unknown alias now names the valid ones instead of leaking a KeyError.
    _bad(s.act("ss.set", {"alias": "nope", "value": 1}), "no such alias")
    assert s.act("ss.set", {"alias": "thickness", "value": 9}).ok
    tbl = s.act("ss.table", {}).data["table"]
    assert "thickness" in tbl and "hole" in tbl, tbl
    print("valid ss.create/set/table still work: %s" % tbl)

    print("SS GUARDS SMOKE OK", s.summary())
    s.registry.kernel.shutdown()


if __name__ in ("__main__", "smoke_ss_guards"):
    main()
