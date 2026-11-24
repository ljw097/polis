"""Virtual field: the upper layer of Polis.

Any node may connect to any other node. Connections are stored as
addresses (dst id -> weight), never as wires, so growing or cutting a
connection is just editing a table. Every node obeys the same
Constitution: purely local rules, no global error signal.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class Constitution:
    """The local rules every node follows (the Nomos 'ISA')."""

    threshold: float = 1.0      # Art. 1: fire when potential reaches this
    leak: float = 0.7           # Art. 1: potential fades each tick
    refractory: int = 2         # Art. 1: rest after firing
    lr: float = 0.08            # Art. 2: links that cause firing get stronger
    decay: float = 0.003        # Art. 3: links that don't, fade (forgetting)
    prune_below: float = 0.02   # Art. 3: faded links are removed
    grow_prob: float = 0.05     # Art. 4: a firing node may link to a co-firing node
    budget: float = 2.5         # Art. 5: total outgoing influence is capped
    max_out: int = 24           # Art. 5: and so is the number of links


class Field:
    def __init__(self, n, init_degree, const=None, rng=None, w_init=(0.05, 0.25)):
        self.n = n
        self.c = const or Constitution()
        self.rng = rng or np.random.default_rng(0)
        lo, hi = w_init
        self.out = [dict() for _ in range(n)]
        for s in range(n):
            for d in self.rng.choice(n, init_degree, replace=False):
                if d != s:
                    self.out[s][int(d)] = float(self.rng.uniform(lo, hi))
        self.v = np.zeros(n)
        self.refr = np.zeros(n, dtype=int)
        self.fired = np.zeros(n, dtype=bool)

    def step(self, ext):
        """Advance one tick. Returns the packets sent: (src, dst) arrays."""
        c = self.c
        I = ext.astype(float).copy()
        srcs, dsts = [], []
        pre = np.flatnonzero(self.fired)
        for s in pre:
            for d, w in self.out[s].items():
                I[d] += w
                srcs.append(s)
                dsts.append(d)

        self.v = self.v * c.leak + I
        self.v[self.refr > 0] = 0.0
        self.refr = np.maximum(self.refr - 1, 0)
        new = self.v >= c.threshold
        self.v[new] = 0.0
        self.refr[new] = c.refractory

        post = np.flatnonzero(new)
        for s in pre:
            links = self.out[s]
            for d in list(links):
                if new[d]:  # Art. 2
                    links[d] += c.lr * (1.0 - links[d])
                else:       # Art. 3
                    links[d] -= c.decay
                    if links[d] < c.prune_below:
                        del links[d]
            # Art. 4: free connection — anyone may link to anyone who co-fired
            if len(post) and len(links) < c.max_out and self.rng.random() < c.grow_prob:
                d = int(self.rng.choice(post))
                if d != s and d not in links:
                    links[d] = 0.1
            # Art. 5: influence budget
            tot = sum(links.values())
            if tot > c.budget:
                k = c.budget / tot
                for d in links:
                    links[d] *= k

        self.fired = new
        return np.array(srcs, dtype=int), np.array(dsts, dtype=int)

    def n_links(self):
        return sum(len(o) for o in self.out)
