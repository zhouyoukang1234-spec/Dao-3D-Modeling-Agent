"""Headless render of any .FCStd file using the agent's own ``view.*`` perception.

Gives a GPU-independent way to *see* a model: tessellate every visible solid and
render a 4-up contact sheet (iso/front/top/right) plus a single iso view with
matplotlib's Agg backend. Run inside freecadcmd::

    DAO_FCSTD=model.FCStd DAO_CONTACT=contact.png DAO_ISO=iso.png \
        freecadcmd examples/render_fcstd.py

(``freecadcmd`` treats trailing CLI args as documents to open, so paths are read
from env vars rather than ``sys.argv``.) This reuses
:mod:`cad_agent.backends.freecad_perceive` unchanged, so the images match exactly
what the live agent perceives through the ``view.views`` / ``view.render`` tools.
"""
import os
import sys

import FreeCAD as App

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cad_agent.backends import freecad_perceive  # noqa: E402


class _State(object):
    """Minimal duck-typed state for the perceive ops (doc + empty name maps)."""

    def __init__(self, doc):
        self.doc = doc
        self.shapes = {}
        self.bodies = {}
        self.components = {}


def main():
    src = os.environ.get("DAO_FCSTD", "demo.FCStd")
    contact = os.environ.get("DAO_CONTACT", "contact_sheet.png")
    iso = os.environ.get("DAO_ISO", "iso.png")

    doc = App.openDocument(src)
    doc.recompute()
    ops = freecad_perceive.register(_State(doc))

    r_views = ops["view.views"]({"path": contact, "size": 1100, "tolerance": 0.3})
    r_iso = ops["view.render"]({"path": iso, "view": "iso", "size": 1000, "tolerance": 0.3})
    print("view.views ->", r_views)
    print("view.render ->", r_iso)
    return 0 if r_views.get("rendered") and r_iso.get("rendered") else 1


main()
