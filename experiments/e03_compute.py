"""e03: can a Polis field compute?

The field is taught only by a broadcast reward (Art. 8): +1 if the answer
was right, -1 if wrong. No backprop, no wiring by hand.

Tasks (two input bits a, b; a bias group is always on):
  copy : answer = a              (linearly separable, warm-up)
  xor  : answer = a XOR b        (needs a hidden computation)

Answer = the output group (out0 / out1) that fires more during the trial.

Run: python experiments/e03_compute.py --task xor [--trials 3000] [--seed 0]
     add --no-teach to run the same field without reward (control)
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from polis import Constitution, Field  # noqa: E402

G = 16  # nodes per input group
O = 8   # nodes per output group


def build(args):
    c = Constitution(
        homeostasis=True, target_rate=args.rate, thr_range=(0.3, 3.0), homeo_eta=0.01,
        decay=0.0, budget=4.0, max_out=64,
        inhib_frac=args.inhib, inhib_gain=2.0, inhib_lr=args.inhib_lr,
        lr=args.hebb, lr_reward=0.0 if args.no_teach else args.lr_reward,
    )
    return Field(args.n, init_degree=args.degree, const=c,
                 rng=np.random.default_rng(args.seed), w_init=(0.1, 0.3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", choices=["copy", "xor"], default="xor")
    ap.add_argument("--trials", type=int, default=3000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n", type=int, default=512)
    ap.add_argument("--degree", type=int, default=16)
    ap.add_argument("--ticks", type=int, default=25)
    ap.add_argument("--rate", type=float, default=0.03)
    ap.add_argument("--inhib", type=float, default=0.2)
    ap.add_argument("--inhib-lr", type=float, default=0.005)
    ap.add_argument("--hebb", type=float, default=0.02)
    ap.add_argument("--lr-reward", type=float, default=0.02)
    ap.add_argument("--noise", type=float, default=0.3,
                    help="spontaneous activity (exploration), std of random input")
    ap.add_argument("--no-teach", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    f = build(args)
    n = args.n
    A, B, BIAS = np.arange(0, G), np.arange(G, 2 * G), np.arange(2 * G, 3 * G)
    OUT = [np.arange(n - 2 * O, n - O), np.arange(n - O, n)]
    rng = np.random.default_rng(args.seed + 100)

    hist = []
    per_case = {}
    for trial in range(args.trials):
        a, b = int(rng.integers(2)), int(rng.integers(2))
        target = a if args.task == "copy" else a ^ b
        ext = np.zeros(n)
        ext[BIAS] = 1.2
        if a:
            ext[A] = 1.2
        if b:
            ext[B] = 1.2
        f.reset_state()
        votes = np.zeros(2)
        for t in range(args.ticks):
            f.step(ext + rng.normal(0, args.noise, n) * (np.arange(n) >= 3 * G))
            if t >= 5:
                votes += [f.fired[OUT[0]].sum(), f.fired[OUT[1]].sum()]
        answer = int(np.argmax(votes)) if votes[0] != votes[1] else -1
        ok = answer == target
        f.reward(1.0 if ok else -1.0, key=(a, b))
        hist.append(ok)
        if trial >= args.trials - 400:
            per_case.setdefault((a, b), []).append(ok)
        if not args.quiet and (trial + 1) % (args.trials // 10) == 0:
            print(f"trial {trial+1:>5}  acc(last 200) {np.mean(hist[-200:]):.2f}  "
                  f"links {f.n_links()}")

    final = np.mean(hist[-400:])
    cases = "  ".join(f"{k[0]}{k[1]}:{np.mean(v):.2f}" for k, v in sorted(per_case.items()))
    print(f"{args.task} seed={args.seed} teach={not args.no_teach}  "
          f"final acc(last 400) {final:.3f}   per case {cases}")
    return final


if __name__ == "__main__":
    main()
