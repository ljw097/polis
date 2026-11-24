"""RTL co-simulation (slow: compiles with Verilator). Skipped if Verilator is missing."""
import os
import shutil
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

pytestmark = pytest.mark.skipif(shutil.which("verilator") is None, reason="verilator not installed")


def test_rtl_matches_model_on_random_program():
    import rtl_build
    res = rtl_build.fuzz_case(seed=7, n=256, gridb=2, ticks=20)
    assert res["spikes"] > 0
    assert res["mismatches"] == [] and res["misses"] == 0
