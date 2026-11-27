import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from polis.program import Program, gate_and, gate_xor, latch  # noqa: E402


def fired(out, name):
    return len(out[name]) > 0


def test_half_adder_truth_table():
    p = Program()
    a, b = p.input("a"), p.input("b")
    p.output("sum", gate_xor(p, a, b))
    p.output("carry", gate_and(p, a, b))
    m = p.load()
    for x in (0, 1):
        for y in (0, 1):
            ev = {"a": [0] if x else [], "b": [0] if y else []}
            out = m.run(ev, ticks=6)
            assert fired(out, "sum") == bool(x ^ y), (x, y, out)
            assert fired(out, "carry") == bool(x & y), (x, y, out)


def test_latch_remembers_until_reset():
    p = Program()
    s, r = p.input("set"), p.input("reset")
    p.output("q", latch(p, s, r))
    m = p.load()
    out = m.run({"set": [2], "reset": [10]}, ticks=20)["q"]
    assert out[0] == 3                      # turns on right after set
    assert all(t in out for t in range(3, 11))  # holds while nobody resets
    assert not any(t > 12 for t in out)     # silent after reset
