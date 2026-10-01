"""The committed mjcf/scene.xml must equal a fresh generation from config/*.yaml (venv test, needs mujoco)."""
import os
import sys
import tempfile

import pytest

pytest.importorskip("mujoco")
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG, "scripts"))


def test_scene_xml_matches_generator():
    import build_mjcf
    with tempfile.TemporaryDirectory() as tmp:
        build_mjcf.generate(out_dir=tmp, verbose=False)
        fresh = open(os.path.join(tmp, "scene.xml")).read()
    committed = open(os.path.join(PKG, "mjcf", "scene.xml")).read()
    assert fresh == committed, "mjcf/scene.xml is stale: run scripts/build_mjcf.py"
