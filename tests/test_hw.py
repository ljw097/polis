import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from polis import hw  # noqa: E402
from polis.program import Program, latch, ripple_adder  # noqa: E402


def test_chip_matches_field_bit_for_bit_on_adder():
    p = Program()
    ripple_adder(p, 8)
    m = p.load()
    chip = hw.from_program(p, n_tiles=16)
    rnd = random.Random(0)
    for _ in range(40):
        x, y = rnd.randrange(256), rnd.randrange(256)
        ev = {}
        for i in range(8):
            ev[f"a{i}"] = range(40) if x >> i & 1 else []
            ev[f"b{i}"] = range(40) if y >> i & 1 else []
        assert hw.run(chip, p, ev, 40) == m.run(ev, 40)


def test_chip_latch():
    p = Program()
    s, r = p.input("set"), p.input("reset")
    p.output("q", latch(p, s, r))
    chip = hw.from_program(p, n_tiles=4)
    ev = {"set": [2], "reset": [10]}
    assert hw.run(chip, p, ev, 20) == p.load().run(ev, 20)
