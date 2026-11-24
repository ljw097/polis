"""Compiled mode: write the connections directly instead of teaching them.

A Program is a graph: nodes (threshold, excitatory/inhibitory), links
(src -> dst, weight) and named ports at the edge. `load()` turns it into a
Machine running on a Field with every learning article switched off, so
execution is deterministic and repeatable.

    p = Program()
    a, b = p.input("a"), p.input("b")
    ...
    m = p.load()
    m.run({"a": [0], "b": [0]}, ticks=6)   # input name -> ticks it fires
"""
import numpy as np

from .field import Constitution, Field

COMPILED = Constitution(
    threshold=1.0, leak=0.7, refractory=0,
    lr=0.0, decay=0.0, prune_below=0.0, grow_prob=0.0,
    budget=1e9, max_out=10**6, homeostasis=False,
)


class Program:
    def __init__(self):
        self.thr, self.sign, self.links = [], [], []
        self.inputs, self.outputs = {}, {}

    def node(self, thr=1.0, inhibitory=False):
        self.thr.append(thr)
        self.sign.append(-1.0 if inhibitory else 1.0)
        return len(self.thr) - 1

    def input(self, name):
        self.inputs[name] = self.node()
        return self.inputs[name]

    def output(self, name, node):
        self.outputs[name] = node

    def link(self, src, dst, w):
        self.links.append((src, dst, float(w)))

    def load(self):
        n = len(self.thr)
        f = Field(n, init_degree=0, const=COMPILED, rng=np.random.default_rng(0))
        f.thr = np.array(self.thr, float)
        f.sign = np.array(self.sign, float)
        for s, d, w in self.links:
            f.out[s][d] = w
        return Machine(f, self.inputs, self.outputs)


class Machine:
    def __init__(self, field, inputs, outputs):
        self.f, self.inputs, self.outputs = field, inputs, outputs

    def run(self, events, ticks):
        """events: {input name: [ticks it fires]}. Returns {output: [ticks fired]}."""
        f = self.f
        f.reset_state()
        out = {k: [] for k in self.outputs}
        for t in range(ticks):
            ext = np.zeros(f.n)
            for name, when in events.items():
                if t in when:
                    ext[self.inputs[name]] = f.thr[self.inputs[name]]
            f.step(ext)
            for k, node in self.outputs.items():
                if f.fired[node]:
                    out[k].append(t)
        return out


# ---- a tiny standard library ------------------------------------------

def gate_or(p, a, b):
    g = p.node(thr=1.0)
    p.link(a, g, 1.0)
    p.link(b, g, 1.0)
    return g


def gate_and(p, a, b, inhibitory=False):
    g = p.node(thr=1.0, inhibitory=inhibitory)
    p.link(a, g, 0.5)
    p.link(b, g, 0.5)
    return g


def gate_xor(p, a, b):
    """OR, vetoed by an inhibitory AND one tick later."""
    o = gate_or(p, a, b)
    n = gate_and(p, a, b, inhibitory=True)
    x = p.node(thr=1.0)
    p.link(o, x, 1.0)
    p.link(n, x, 2.0)
    return x


def latch(p, set_, reset):
    """1-bit memory: a node that keeps itself firing until vetoed."""
    m = p.node(thr=1.0)
    p.link(set_, m, 1.0)
    p.link(m, m, 1.0)
    r = p.node(thr=1.0, inhibitory=True)
    p.link(reset, r, 1.0)
    p.link(r, m, 5.0)
    return m
