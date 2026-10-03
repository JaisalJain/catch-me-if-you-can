import json
import os

import numpy as np

from catch_throw.library import Library

CFG = os.path.join(os.path.dirname(__file__), "..", "config")


def _lib(tmp_path, flags):
    d = dict(library="t", entries=[dict(id=i, params={}, hash="", moveit_start_valid=f) for i, f in enumerate(flags)])
    p = tmp_path / "library_t.json"
    p.write_text(json.dumps(d))
    return Library("t", path=str(p), paths=("x", "y", "z"))


def test_draw_skips_moveit_invalid_entries(tmp_path):
    lib = _lib(tmp_path, [True, False, True, False])
    rng = np.random.default_rng(0)
    got = {lib.draw(rng) for _ in range(200)}
    assert got == {0, 2}


def test_draw_unchanged_without_flags(tmp_path):
    # without flagged entries, draw() is exactly rng.integers(n): seeded benchmark sequences stay the same
    lib = _lib(tmp_path, [True] * 7)
    a, b = np.random.default_rng(5), np.random.default_rng(5)
    assert [lib.draw(a) for _ in range(50)] == [int(b.integers(7)) for _ in range(50)]


def test_installed_libraries_flag_format():
    for name in ("pinch", "cup"):
        d = json.load(open(os.path.join(CFG, f"library_{name}.json")))
        for e in d["entries"]:
            assert isinstance(e.get("moveit_start_valid", True), bool)
