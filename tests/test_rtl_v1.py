"""RTL v1 co-simulation (slow: Verilator). Learning, rewards and firmware
migration included; spikes and every table must match hw1 bit for bit."""
import os
import shutil
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
pytestmark = pytest.mark.skipif(shutil.which("verilator") is None, reason="verilator not installed")


def test_v1_learning_and_migration_bit_exact():
    import rtl1_build
    chip, script = rtl1_build.scenario(5, n=64, gridb=2, ticks=24)
    res = rtl1_build.run(chip, script, 2)
    assert res["spikes"] > 0
    assert res["tick_mism"] == [] and res["table_mism"] == [] and res["misses"] == 0


def test_gateway_bridge_end_to_end():
    import nep_soc_test
    assert nep_soc_test.main(n_adds=2)
