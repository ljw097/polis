import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from polis.program import Program, ripple_adder  # noqa: E402
from polis.protocol import Network  # noqa: E402


def run_adds(mode, cases, moves=3.0):
    p = Program()
    ripple_adder(p, 4)
    m = p.load()
    ports = list(p.inputs.values()) + list(p.outputs.values())
    net = Network(len(p.thr), 8, mode=mode, rng=np.random.default_rng(1), pinned=ports)
    ok = 0
    for x, y in cases:
        ev = {}
        for i in range(4):
            ev[f"a{i}"] = range(24) if x >> i & 1 else []
            ev[f"b{i}"] = range(24) if y >> i & 1 else []
        out = m.run(ev, 24, deliver=net.deliver, on_tick=lambda t: net.migrate(moves))
        ok += sum(1 << i for i in range(5) if 23 in out[f"s{i}"]) == x + y
    return ok, net


def test_polis_protocol_never_loses_a_packet():
    cases = [(x, y) for x in range(16) for y in range(16)]
    ok, net = run_adds("polis", cases)
    assert ok == len(cases)
    assert net.stats["lost"] == 0 and net.stats["moves"] > 1000


def test_naive_migration_breaks_the_program():
    cases = [(x, 15 - x) for x in range(16)]
    ok, net = run_adds("naive", cases)
    assert net.stats["lost"] > 0 and ok < len(cases)
