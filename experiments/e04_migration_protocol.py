"""e04: can nodes keep migrating while a compiled program runs, without
changing a single output bit?

Program: an 8-bit ripple-carry adder in compiled mode (deterministic).
Every tick, `--moves` random nodes jump to a random tile (ports pinned).
Modes:
  none  : no migration (reference)
  naive : home directory updated, caches stale, no forwarding
  polis : home directory + address caches + forwarding + tick barrier

Run: python experiments/e04_migration_protocol.py [--adds 200] [--moves 5]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from polis.program import Program, ripple_adder  # noqa: E402
from polis.protocol import Network  # noqa: E402

BITS, TICKS = 8, 40


def add(machine, x, y, net):
    ev = {}
    for i in range(BITS):
        ev[f"a{i}"] = range(TICKS) if x >> i & 1 else []
        ev[f"b{i}"] = range(TICKS) if y >> i & 1 else []
    deliver = net.deliver if net else None
    on_tick = (lambda t: net.migrate(net_moves)) if net else None
    out = machine.run(ev, TICKS, deliver=deliver, on_tick=on_tick)
    return sum(1 << i for i in range(BITS + 1) if TICKS - 1 in out[f"s{i}"])


def main():
    global net_moves
    ap = argparse.ArgumentParser()
    ap.add_argument("--adds", type=int, default=200)
    ap.add_argument("--moves", type=float, default=5, help="mean moves per tick")
    ap.add_argument("--tiles", type=int, default=16)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--ttl", type=int, default=8, help="forwarding pointer lifetime (ticks)")
    a = ap.parse_args()
    net_moves = a.moves

    p = Program()
    ripple_adder(p, BITS)
    m = p.load()
    ports = list(p.inputs.values()) + list(p.outputs.values())
    n = len(p.thr)
    rng = np.random.default_rng(a.seed)
    cases = rng.integers(0, 2 ** BITS, (a.adds, 2))
    print(f"program: {n} nodes, {len(p.links)} links; {a.adds} additions, "
          f"{a.moves} moves/tick on {a.tiles} tiles\n")
    print(f"{'mode':>6} {'correct':>8} {'moves':>7} {'packets':>8} {'lost':>6} "
          f"{'fwd':>6} {'fallbk':>6} {'homeQ':>6} {'ctrl msgs/pkt':>13}")
    for mode in ["none", "naive", "polis"]:
        net = None if mode == "none" else Network(
            n, a.tiles, mode=mode, rng=np.random.default_rng(a.seed + 1), pinned=ports,
            fwd_ttl=a.ttl)
        ok = sum(add(m, int(x), int(y), net) == x + y for x, y in cases)
        st = net.stats if net else {}
        pk = st.get("packets", 0)
        ctrl = (2 * st.get("home_queries", 0) + st.get("corrections", 0)
                + st.get("dir_updates", 0) + 2 * st.get("fallbacks", 0))
        print(f"{mode:>6} {ok:>4}/{a.adds:<3} {st.get('moves', 0):>7} {pk:>8} "
              f"{st.get('lost', 0):>6} {st.get('forwards', 0):>6} {st.get('fallbacks', 0):>6} "
              f"{st.get('home_queries', 0):>6} {ctrl / pk if pk else 0:>13.3f}")


if __name__ == "__main__":
    main()
