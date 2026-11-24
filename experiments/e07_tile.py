"""e07: the tile design on real workloads.

1. compiled 8-bit adder on the hardware model: cycles per tick, SRAM.
2. the e01 field after learning (frozen), mapped onto the chip in fixed
   point with leak and refractory. Checks how closely int8/int16 hardware
   reproduces the float field, and measures cycles per tick for random vs
   federal placement.

Run: python experiments/e07_tile.py [--seed 0]
"""
import argparse
import copy
import os
import sys
from collections import Counter

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from polis import Field, Substrate, hw  # noqa: E402
from polis.program import Program, ripple_adder  # noqa: E402


def net_cycles(pkts, grid):
    load, longest = Counter(), 0
    for a, b in pkts:
        x, y, bx, by = a % grid, a // grid, b % grid, b // grid
        h = 0
        while x != bx:
            nx = x + (1 if bx > x else -1)
            load[(x, y, nx, y)] += 1
            x, h = nx, h + 1
        while y != by:
            ny = y + (1 if by > y else -1)
            load[(x, y, x, ny)] += 1
            y, h = ny, h + 1
        longest = max(longest, h)
    return max(max(load.values(), default=0), longest)


def summarize(chip, grid, label):
    comp = [c for c, _ in chip.cycles]
    net = [net_cycles(p, grid) for _, p in chip.cycles]
    tot, worst = chip.sram_bits()
    kib = {k: v / 8 / 1024 for k, v in tot.items()}
    print(f"{label:<28} compute {np.mean(comp):6.1f}  network {np.mean(net):6.1f}  "
          f"tick {np.mean(np.add(comp, net)):6.1f} cycles   "
          f"SRAM total {sum(kib.values()):7.1f} KiB (synapse {kib['synapse']:.1f}, "
          f"index {kib['index']:.1f}, node {kib['node']:.1f}, fan-out {kib['fanout']:.1f}), "
          f"worst tile {worst / 8 / 1024:.1f} KiB")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--train", type=int, default=3000)
    ap.add_argument("--test", type=int, default=300)
    ap.add_argument("--balance", type=float, default=0.0,
                    help="work-balance weight in migration (0 = traffic only)")
    a = ap.parse_args()

    # ---- 1. compiled adder ------------------------------------------------
    p = Program()
    ripple_adder(p, 8)
    chip = hw.from_program(p, n_tiles=16)
    rng = np.random.default_rng(a.seed)
    for x, y in rng.integers(0, 256, (50, 2)):
        ev = {}
        for i in range(8):
            ev[f"a{i}"] = range(40) if x >> i & 1 else []
            ev[f"b{i}"] = range(40) if y >> i & 1 else []
        hw.run(chip, p, ev, 40)
    print(f"8-bit adder: {len(p.thr)} nodes, {len(p.links)} links, 4x4 tiles")
    summarize(chip, 4, "  random placement")

    # ---- 2. learned field, frozen ----------------------------------------
    N = 1024
    field = Field(N, init_degree=8, rng=rng, w_init=(0.2, 0.5))
    sub = Substrate(N, rng=np.random.default_rng(a.seed + 1), balance=a.balance)
    static_tile = sub.tile.copy()
    pat = 0
    for t in range(a.train):
        if t % 30 == 0:
            pat = int(rng.integers(4))
        ext = np.zeros(N)
        if t % 30 < 20:
            ext[pat * 32:(pat + 1) * 32] = 1.2
        src, dst = field.step(ext)
        if len(src):
            sub.route(src, dst)
        if (t + 1) % 25 == 0:
            sub.migrate()
    c = field.c
    c.lr = c.decay = c.grow_prob = 0.0
    c.prune_below = 0.0
    links = [(s, d, w) for s in range(N) for d, w in field.out[s].items()]
    print(f"\nlearned field: {N} nodes, {len(links)} links, 8x8 tiles, frozen after "
          f"{a.train} ticks")

    test_pats = rng.integers(0, 4, a.test // 30 + 1)
    for label, place in (("  random placement", static_tile), ("  federal placement", sub.tile)):
        f = copy.deepcopy(field)
        f.v[:] = 0
        f.fired[:] = False
        f.refr[:] = 0
        chip = hw.Chip(N, f.thr, links, f.sign, place, 64, leak=c.leak,
                       refractory=c.refractory)
        agree, n_f, n_c = [], 0, 0
        for t in range(a.test):
            pat = test_pats[t // 30]
            on = t % 30 < 20
            ext = np.zeros(N)
            nodes = list(range(pat * 32, (pat + 1) * 32)) if on else []
            ext[nodes] = 1.2
            f.step(ext)
            fired_c = chip.step(ext_current={g: 1.2 for g in nodes})
            ff = set(np.flatnonzero(f.fired).tolist())
            n_f, n_c = n_f + len(ff), n_c + len(fired_c)
            if ff or fired_c:
                agree.append(len(ff & fired_c) / len(ff | fired_c))
        summarize(chip, 8, label)
        print(f"{'':<28} spikes float {n_f}, fixed-point {n_c}, "
              f"per-tick agreement (Jaccard) {np.mean(agree):.3f}")


if __name__ == "__main__":
    main()
