# -*- coding: utf-8 -*-
"""Self-tests for the general L1/2 datum layer: the honesty guards must fire."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from uam.datum import Authority, kabsch, require_physical_dof, solve_placement


def test_two_points_refused():
    try:
        solve_placement({"a": np.zeros(3), "b": np.array([1., 0, 0])},
                        {"a": np.zeros(3), "b": np.array([1., 0, 0])}, Authority.CAD)
    except ValueError:
        return True
    raise AssertionError("2-point pose should be refused")


def test_collinear_refused():
    col = {k: np.array([float(i), 0, 0]) for i, k in enumerate("abc")}
    try:
        solve_placement(col, col, Authority.CAD)
    except ValueError:
        return True
    raise AssertionError("collinear pose should be refused")


def test_fake_dof_refused():
    try:
        require_physical_dof("ball_horizontal_sweep", exists_in_mechanism=False)
    except ValueError:
        return True
    raise AssertionError("non-physical DOF should be refused")


def test_proper_rotation():
    R, _ = kabsch(np.eye(3), np.array([[0, 0, 1.], [1, 0, 0], [0, 1, 0]]))
    assert abs(np.linalg.det(R) - 1.0) < 1e-9, "Kabsch must yield a proper rotation"
    return True


def test_clean_fit_roundtrips():
    perc = {"p0": np.array([1., 0, 0]), "p1": np.array([0, 1., 0]),
            "p2": np.array([0, 0, 1.]), "p3": np.array([1., 1, 0])}
    R = kabsch(np.eye(3), np.array([[0, 0, 1.], [1, 0, 0], [0, 1, 0]]))[0]
    t = np.array([10., -3., 7.])
    auth = {k: R @ v + t for k, v in perc.items()}
    fit = solve_placement(perc, auth, Authority.CONTROL_LAW)
    assert fit.rms < 1e-9, f"clean data must fit exactly, rms={fit.rms}"
    return True


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
        print(f"OK  {fn.__name__}")
    print(f"\n{len(tests)} datum-layer guards verified.")
