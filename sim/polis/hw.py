"""Hardware-level model of a Polis chip (inference / compiled mode).

Unlike `Field` (floats, Python dicts), this model uses the data layout and
arithmetic of the tile design in docs/tile.md:

  * fixed point: weights int8 (scale 1/16, sign of the source folded in),
    potentials and thresholds int16 with saturation, leak as an 8-bit
    multiplier (v * leak_q >> 8)
  * per tile SRAM tables, sizes counted in bits:
      node table   : potential 16 | threshold 16 | refractory 4 | flags 4
      src index    : hash table (load factor 0.8), entry 48 bit:
                     key = src node id 24 | ptr 16 + len 8          (segment)
                                          | dst 12 + weight 8 + flags 4 (inline)
                     a source with a single synapse here is stored inline,
                     so it needs no synapse-store entry at all
      synapse store: local dst 12 | weight 8 | plastic 1 | credit 3   (24 bit)
      fan-out table: per local node, list of destination tiles 12 bit each
  * per tick cycle model of each tile:
      receive : per packet, 1 hash lookup (+ probe misses) + 1 cycle per synapse
      update  : touched nodes only, `lanes` nodes per cycle (event driven)
      send    : 1 cycle per outgoing packet (one per destination tile)
    tile cycles = receive + update + send; tick = slowest tile (+ network).

The model executes the program itself, so its outputs can be checked
against `Field` bit for bit.
"""
from collections import defaultdict

import numpy as np

W_SCALE = 16          # weight / potential fixed-point scale
I16 = (-32768, 32767)

BITS = dict(node=16 + 16 + 4 + 4, index=24 + 16 + 8, synapse=24, fanout=12)


def q(x):
    return int(np.clip(round(x * W_SCALE), *I16))


class Tile:
    def __init__(self, tid):
        self.tid = tid
        self.nodes = []                 # global ids, local index = position
        self.index = {}                 # src id -> (ptr, len)   (hash table model)
        self.syn_dst, self.syn_w = [], []
        self.fanout = {}                # local idx -> [dst tiles]

    def finalize(self, n_buckets_factor=1.25):
        self.syn_dst = np.array(self.syn_dst, dtype=np.int32)
        self.syn_w = np.array(self.syn_w, dtype=np.int32)
        n = len(self.nodes)
        self.v = np.zeros(n, dtype=np.int32)
        self.refr = np.zeros(n, dtype=np.int32)
        # hash table with linear probing: count probes per key once
        nb = max(1, int(np.ceil(n_buckets_factor * len(self.index))))
        occupied, self.probes = set(), {}
        for src in self.index:
            h, p = hash(src * 2654435761) % nb, 0
            while h in occupied:
                h, p = (h + 1) % nb, p + 1
            occupied.add(h)
            self.probes[src] = p
        self.n_buckets = nb

    def sram_bits(self):
        fo = sum(len(v) for v in self.fanout.values())
        inline = sum(1 for _, ln in self.index.values() if ln == 1)
        return dict(node=len(self.nodes) * BITS["node"],
                    index=self.n_buckets * BITS["index"],
                    synapse=(len(self.syn_dst) - inline) * BITS["synapse"],
                    fanout=fo * BITS["fanout"])


class Chip:
    def __init__(self, n, thr, links, sign, placement, n_tiles, leak=0.0,
                 refractory=0, lanes=8):
        """links: list of (src, dst, w); sign[src] folds into the weight;
        placement[node] = tile."""
        self.n, self.lanes, self.refractory = n, lanes, refractory
        self.leak_q = int(round(leak * 256))
        self.place = np.asarray(placement)
        self.tiles = [Tile(t) for t in range(n_tiles)]
        self.local = np.zeros(n, dtype=np.int64)
        for g in range(n):
            t = self.tiles[self.place[g]]
            self.local[g] = len(t.nodes)
            t.nodes.append(g)
        for t in self.tiles:
            t.thr = np.array([q(thr[g]) for g in t.nodes], dtype=np.int32)
        by_dst_tile = defaultdict(lambda: defaultdict(list))
        for s, d, w in links:
            by_dst_tile[self.place[d]][s].append((self.local[d], q(sign[s] * w)))
        for tid, srcs in by_dst_tile.items():
            t = self.tiles[tid]
            for s, lst in srcs.items():
                t.index[s] = (len(t.syn_dst), len(lst))
                for ld, wq in lst:
                    t.syn_dst.append(ld)
                    t.syn_w.append(wq)
                src_t = self.tiles[self.place[s]]
                src_t.fanout.setdefault(int(self.local[s]), []).append(tid)
        for t in self.tiles:
            t.finalize()
        self.fired_prev = []            # global ids fired last tick
        self.cycles = []                # per tick: (compute, network, packets)

    def reset(self):
        for t in self.tiles:
            t.v[:] = 0
            t.refr[:] = 0
        self.fired_prev = []

    def step(self, ext_nodes=(), ext_current=None):
        """ext_nodes: global ids receiving external input = their threshold.
        ext_current: {global id: current (float)} for arbitrary input sizes."""
        recv = np.zeros(len(self.tiles), dtype=np.int64)
        send = np.zeros(len(self.tiles), dtype=np.int64)
        I = [np.zeros(len(t.nodes), dtype=np.int32) for t in self.tiles]
        touched = [set() for _ in self.tiles]
        pkts = []
        for g in self.fired_prev:                     # send phase of last tick
            st = self.tiles[self.place[g]]
            for dt in st.fanout.get(int(self.local[g]), []):
                send[st.tid] += 1
                pkts.append((st.tid, dt))
                t = self.tiles[dt]
                ptr, ln = t.index[g]
                recv[dt] += 1 + t.probes[g] + ln
                seg = slice(ptr, ptr + ln)
                np.add.at(I[dt], t.syn_dst[seg], t.syn_w[seg])
                touched[dt].update(t.syn_dst[seg].tolist())
        for g in ext_nodes:
            t = self.tiles[self.place[g]]
            I[t.tid][self.local[g]] += t.thr[self.local[g]]
            touched[t.tid].add(int(self.local[g]))
        for g, cur in (ext_current or {}).items():
            t = self.tiles[self.place[g]]
            I[t.tid][self.local[g]] += q(cur)
            touched[t.tid].add(int(self.local[g]))
        fired = []
        upd = np.zeros(len(self.tiles), dtype=np.int64)
        for t in self.tiles:
            if self.leak_q or self.refractory:          # timers running: update all
                idx = np.arange(len(t.nodes))
            else:                                      # event driven: touched only
                idx = np.fromiter(touched[t.tid], dtype=np.int64)
                t.v[:] = 0                             # leak 0: untouched stay at 0
            upd[t.tid] = -(-len(idx) // self.lanes)
            if not len(idx):
                continue
            v = (t.v[idx] * self.leak_q >> 8) + I[t.tid][idx]
            v = np.clip(v, *I16)
            v[t.refr[idx] > 0] = 0
            t.refr[idx] = np.maximum(t.refr[idx] - 1, 0)
            new = v >= t.thr[idx]
            v[new] = 0
            t.refr[idx[new]] = self.refractory
            t.v[idx] = v
            fired.extend(t.nodes[i] for i in idx[new])
        self.fired_prev = fired
        self.cycles.append((int((recv + upd + send).max()), pkts))
        return set(fired)

    def sram_bits(self):
        tot = defaultdict(int)
        per_tile = []
        for t in self.tiles:
            b = t.sram_bits()
            per_tile.append(sum(b.values()))
            for k, v in b.items():
                tot[k] += v
        return dict(tot), max(per_tile)


def from_program(prog, n_tiles, placement=None, rng=None, **kw):
    n = len(prog.thr)
    if placement is None:
        placement = (rng or np.random.default_rng(0)).integers(0, n_tiles, n)
    return Chip(n, prog.thr, prog.links, prog.sign, placement, n_tiles, **kw)


def run(chip, prog, events, ticks):
    """Same interface as Machine.run: {input: [ticks]} -> {output: [ticks fired]}."""
    chip.reset()
    out = {k: [] for k in prog.outputs}
    for t in range(ticks):
        ext = [prog.inputs[name] for name, when in events.items() if t in when]
        fired = chip.step(ext)
        for k, node in prog.outputs.items():
            if node in fired:
                out[k].append(t)
    return out
