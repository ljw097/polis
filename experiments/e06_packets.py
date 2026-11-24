"""e06: on-chip packet format, routing and the real length of a tick.

Two packet schemes on the same traffic:
  A per-link : one packet per (spike, destination node); sender holds weights
               bits = type 4 + dst tile 12 + dst node 24 + weight 8 = 48
  B per-tile : one packet per (spike, destination tile); the receiving tile
               fans out using its own table of (src -> local dst, weight)
               bits = type 4 + dst tile 12 + src node 24 = 40

Routing: dimension-ordered XY on the tile mesh, one 48-bit flit per link
per cycle. Estimated cycles per tick = max(busiest link load, longest path)
— the usual lower bound for a congested mesh; ignores router pipeline depth.

Placements compared: static random vs after federal migration (as in e01).

Run: python experiments/e06_packets.py [--ticks 3000] [--seed 0]
"""
import argparse
import copy
import os
import sys
from collections import Counter

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from polis import Field, Substrate  # noqa: E402

N, N_PAT, PER_PAT = 1024, 4, 32
BITS = {"A": 48, "B": 40}


def tick_cost(sub, src, dst):
    """Packets, bit-hops and estimated cycles for one tick, both schemes."""
    out = {}
    ts, td = sub.tile[src], sub.tile[dst]
    pairs = {"A": list(zip(ts.tolist(), td.tolist())),
             "B": list({(s, int(t)) for s, t in zip(src.tolist(), td.tolist())})}
    pairs["B"] = [(int(sub.tile[s]), t) for s, t in pairs["B"]]
    g = sub.grid
    for k, pp in pairs.items():
        load, longest, hops_total = Counter(), 0, 0
        for a, b in pp:
            x, y, bx, by = a % g, a // g, b % g, b // g
            h = 0
            while x != bx:                     # X first
                nx = x + (1 if bx > x else -1)
                load[(x, y, nx, y)] += 1
                x, h = nx, h + 1
            while y != by:                     # then Y
                ny = y + (1 if by > y else -1)
                load[(x, y, x, ny)] += 1
                y, h = ny, h + 1
            longest, hops_total = max(longest, h), hops_total + h
        out[k] = (len(pp), hops_total * BITS[k], max(max(load.values(), default=0), longest))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ticks", type=int, default=3000)
    ap.add_argument("--measure", type=int, default=600, help="last ticks measured")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed)
    field = Field(N, init_degree=8, rng=rng, w_init=(0.2, 0.5))
    static = Substrate(N, rng=np.random.default_rng(a.seed + 1))
    federal = copy.deepcopy(static)
    acc = {(p, k): np.zeros(3) for p in ("static", "federal") for k in "AB"}
    spikes = 0
    pat = 0
    for t in range(a.ticks):
        if t % 30 == 0:
            pat = int(rng.integers(N_PAT))
        ext = np.zeros(N)
        if t % 30 < 20:
            ext[pat * PER_PAT:(pat + 1) * PER_PAT] = 1.2
        src, dst = field.step(ext)
        federal.route(src, dst)
        if (t + 1) % 25 == 0:
            federal.migrate()
        if t >= a.ticks - a.measure and len(src):
            spikes += len(set(src.tolist()))
            for name, sub in (("static", static), ("federal", federal)):
                for k, v in tick_cost(sub, src, dst).items():
                    acc[(name, k)] += v

    m = a.measure
    print(f"last {m} ticks, {spikes / m:.1f} spikes/tick, 8x8 tiles\n")
    print(f"{'placement':>9} {'scheme':>8} {'packets/tick':>12} {'kbit-hops/tick':>14} "
          f"{'est cycles/tick':>15}")
    for (name, k), v in acc.items():
        lab = "A link" if k == "A" else "B tile"
        print(f"{name:>9} {lab:>8} {v[0] / m:>12.1f} {v[1] / m / 1000:>14.2f} {v[2] / m:>15.1f}")


if __name__ == "__main__":
    main()
