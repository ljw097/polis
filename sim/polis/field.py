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
    homeostasis: bool = False   # Art. 6 (experimental, off by default): each node steers its own threshold
    target_rate: float = 0.02   # Art. 6: ...toward this firing rate
    homeo_eta: float = 0.05     # Art. 6: adaptation speed
    rate_tau: float = 100.0     # Art. 6: memory of own activity (ticks)
    thr_range: tuple = (0.15, 3.0)
    inhib_frac: float = 0.0     # Art. 7: share of nodes whose signals inhibit
    inhib_gain: float = 1.0     # Art. 7: strength of inhibitory signals
    inhib_lr: float = 0.0       # Art. 7b: inhibition learns its targets: links onto
                                #   nodes busier than target grow, onto quiet ones fade
    lr_reward: float = 0.0      # Art. 8: education — a broadcast reward scales each
                                #   link's recent contribution (eligibility)


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
        self.thr = np.full(n, self.c.threshold)
        # Art. 7: some citizens speak against — their spikes lower potential
        self.sign = np.ones(n)
        if self.c.inhib_frac > 0:
            self.sign[self.rng.random(n) < self.c.inhib_frac] = -self.c.inhib_gain
        self.rate = np.full(n, self.c.target_rate)
        self.elig = [dict() for _ in range(n)]
        self.r_avg = {}

    def step(self, ext, deliver=None):
        """Advance one tick. Returns the packets sent: (src, dst) arrays.
        `deliver(src, dst) -> bool` lets a network model decide whether each
        packet actually reached its node; undelivered packets are lost."""
        c = self.c
        I = ext.astype(float).copy()
        srcs, dsts = [], []
        pre = np.flatnonzero(self.fired)
        for s in pre:
            sg = self.sign[s]
            for d, w in self.out[s].items():
                if deliver is None or deliver(s, d):
                    I[d] += sg * w
                srcs.append(s)
                dsts.append(d)

        self.v = self.v * c.leak + I
        self.v[self.refr > 0] = 0.0
        self.refr = np.maximum(self.refr - 1, 0)
        new = self.v >= self.thr
        self.v[new] = 0.0
        self.refr[new] = c.refractory

        post = np.flatnonzero(new)
        a = 1.0 / c.rate_tau
        self.rate = self.rate * (1 - a) + a * new
        for s in pre:
            links = self.out[s]
            inhib = self.sign[s] < 0
            el = self.elig[s]
            for d in list(links):
                if inhib and c.inhib_lr:  # Art. 7b
                    links[d] = min(1.0, links[d] + c.inhib_lr *
                                   (self.rate[d] - c.target_rate) / c.target_rate)
                else:
                    if c.lr_reward:  # Art. 8 credit: did I push d above its usual rate?
                        el[d] = el.get(d, 0.0) + (float(new[d]) - self.rate[d])
                    if new[d]:    # Art. 2
                        links[d] += c.lr * (1.0 - links[d])
                    else:         # Art. 3
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

        # Art. 6: homeostasis — quiet nodes grow sensitive, loud ones dull
        if c.homeostasis:
            self.thr = np.clip(self.thr + c.homeo_eta * (self.rate - c.target_rate),
                               *c.thr_range)

        self.fired = new
        return np.array(srcs, dtype=int), np.array(dsts, dtype=int)

    def reward(self, r, key=None):
        """Art. 8: broadcast a reward. Every link is nudged by
        (reward - expected reward) x its recent credit, then credit is cleared.
        `key` lets the teacher keep a separate expectation per question;
        a single shared expectation gets stuck when the field is always wrong."""
        c = self.c
        if not isinstance(self.r_avg, dict):
            self.r_avg = {}
        exp = self.r_avg.get(key, 0.0)
        delta = r - exp
        self.r_avg[key] = exp + 0.05 * delta
        for s in range(self.n):
            el = self.elig[s]
            if not el:
                continue
            links = self.out[s]
            for d, e in el.items():
                if d in links:
                    links[d] = float(np.clip(links[d] + c.lr_reward * delta * e, 0.0, 1.0))
                    if links[d] < c.prune_below:
                        del links[d]
            el.clear()
        return delta

    def reset_state(self):
        self.v[:] = 0.0
        self.fired[:] = False
        self.refr[:] = 0

    def n_links(self):
        return sum(len(o) for o in self.out)
