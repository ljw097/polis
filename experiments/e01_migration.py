"""e01: does federal migration make traffic local, and do demes emerge?

One virtual field is driven by 4 input patterns. Its packet stream is fed
to two identical substrates that start from the same random placement:
  * static  : nodes never move
  * federal : intra-region self-government + inter-region treaties
Same traffic, so any difference is caused by migration alone.

Run: python experiments/e01_migration.py [--ticks 3000] [--seed 0]
"""
import argparse
import copy
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from polis import Field, Substrate  # noqa: E402

N, N_PAT, PER_PAT = 1024, 4, 32


def purity(sub, pref, active):
    """Share of active nodes in each region that belong to its dominant pattern."""
    vals = []
    for r in range(sub.n_regions):
        m = active & (sub.region_of_tile[sub.tile] == r)
        if m.sum():
            vals.append(np.bincount(pref[m], minlength=N_PAT).max() / m.sum())
    return float(np.mean(vals))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ticks", type=int, default=3000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--every", type=int, default=25)
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed)
    field = Field(N, init_degree=8, rng=rng, w_init=(0.2, 0.5))
    static = Substrate(N, rng=np.random.default_rng(a.seed + 1))
    federal = copy.deepcopy(static)
    subs = {"static": static, "federal": federal}

    fire_by_pat = np.zeros((N, N_PAT))
    window = {k: [] for k in subs}
    rows = []
    pat = 0
    for t in range(a.ticks):
        phase = t % 30
        if phase == 0:
            pat = int(rng.integers(N_PAT))
        ext = np.zeros(N)
        if phase < 20:
            ext[pat * PER_PAT:(pat + 1) * PER_PAT] = 1.2
        src, dst = field.step(ext)
        fire_by_pat[field.fired, pat] += 1
        for k, s in subs.items():
            if len(src):
                window[k].append(s.route(src, dst))
        if (t + 1) % a.every == 0:
            federal.migrate()
        if (t + 1) % (a.ticks // 10) == 0:
            row = {"tick": t + 1, "links": field.n_links()}
            for k in subs:
                h = np.concatenate(window[k]) if window[k] else np.full(1, np.nan)
                row[k] = (h.mean(), (h == 0).mean())
                window[k] = []
            rows.append(row)

    print(f"{'tick':>6} {'links':>6} | {'static hops':>11} {'local%':>6} | "
          f"{'federal hops':>12} {'local%':>6}")
    for r in rows:
        print(f"{r['tick']:>6} {r['links']:>6} | {r['static'][0]:>11.2f} "
              f"{r['static'][1]*100:>5.1f}% | {r['federal'][0]:>12.2f} "
              f"{r['federal'][1]*100:>5.1f}%")

    active = fire_by_pat.sum(1) > 0
    pref = fire_by_pat.argmax(1)
    chance = np.bincount(pref[active], minlength=N_PAT).max() / active.sum()
    print(f"\nactive nodes: {active.sum()}/{N}")
    print(f"deme purity  static: {purity(static, pref, active):.2f}  "
          f"federal: {purity(federal, pref, active):.2f}  (chance ~{chance:.2f})")
    print(f"migrations   intra: {federal.stats['intra']}  "
          f"inter proposed: {federal.stats['proposed']}  approved: {federal.stats['inter']}")


if __name__ == "__main__":
    main()
