"""Reference model for RTL v1 (rtl/v1). Every table, field and integer
operation here is the one the RTL implements, in the same order, so the two
can be compared bit for bit: spikes every tick AND final table contents.

Fixed point
  synapse word (24 bit): w12 signed [23:12] (unit 1/256) | dst [11:4] | plastic [3] | credit [2:0]
      contribution to a potential = w12 >>> 4  (unit 1/16)
  node word   (36 bit):  thr int16 [35:20] | refr u4 [19:16] | v int16 [15:0]   (unit 1/16)
  rate        (8 bit):   firing-rate estimate, 256 = always firing
  index entry (50 bit):  valid [49] | key24 [48:25] | inline [24] | payload [23:0]
      payload = synapse word (inline) or {ptr16, len8} (segment)
  hash: xor-fold of the 24-bit source id, linear probing

Per tick
  COMM   : send fired list of last tick (one packet per destination tile);
           receive: probe index, accumulate every synapse word of the source
           (inline or segment), mark dst touched, remember plastic ones (visit list)
  UPDATE : full mode (leak, refractory or homeostasis on): every node in
           order; else touched nodes in touch order. Art. 1 and Art. 6.
  LEARN  : (if enabled) visit list in order. Art. 2, 3, 7b, credit for Art. 8.
Commands
  REWARD(delta): sweep index slots then synapse store; Art. 8.
  Firmware (management core) edits tables between ticks through the config
  port: node migration, new links, budget sweeps. See `Firmware`.
"""
from dataclasses import dataclass, field

import numpy as np

W12_MIN, W12_MAX = -2048, 2047
I16 = (-32768, 32767)


@dataclass
class Params:
    leak: int = 0          # u8: v <- (v * leak) >>> 8
    refr: int = 0          # u4
    homeo: int = 0
    target: int = 8        # u8 rate target (256 = always)
    rs: int = 4            # rate EMA shift
    hs: int = 4            # threshold adaptation shift
    tmin: int = 4          # thresholds clamp (1/16)
    tmax: int = 48
    learn: int = 0
    lp: int = 3            # Art. 2: w += (2047 - w) >> lp
    dec: int = 1           # Art. 3: w -= dec   (1/256 per pre spike without post spike)
    prune: int = 4         # Art. 3: w <= prune -> dead synapse
    il: int = 8            # Art. 7b: inhibitory step
    rsh: int = 3           # Art. 8: w += (delta * credit) >> rsh

    def full(self):
        return bool(self.leak or self.refr or self.homeo)


def fold(src, b):
    h, mask = 0, (1 << b) - 1
    for i in range(0, 24, b):
        h ^= (src >> i) & mask
    return h


def sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def syn_pack(w12, dst, plastic, credit):
    return ((w12 & 0xFFF) << 12) | ((dst & 0xFF) << 4) | (plastic << 3) | (credit & 7)


def syn_unpack(word):
    return sx(word >> 12, 12), (word >> 4) & 0xFF, (word >> 3) & 1, word & 7


def node_pack(thr, refr, v):
    return ((thr & 0xFFFF) << 20) | ((refr & 0xF) << 16) | (v & 0xFFFF)


def node_unpack(word):
    return sx(word >> 20, 16), (word >> 16) & 0xF, sx(word, 16)


def clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Tile:
    def __init__(self, logn, lognb, logs, logf):
        self.N, self.NB, self.S, self.F = 1 << logn, 1 << lognb, 1 << logs, 1 << logf
        self.lognb = lognb
        self.node = [node_pack(32767, 0, 0)] * self.N      # free slot: never fires
        self.rate = [0] * self.N
        self.gid = [0] * self.N
        self.fo_ptr = [0] * self.N                          # (ptr << 8) | len
        self.fo = [0] * self.F
        self.idx = [0] * self.NB
        self.syn = [0] * self.S
        self.acc = [0] * self.N
        self.touched = [False] * self.N
        self.tl, self.fl, self.vis = [], [], []
        self.firedv = [False] * self.N
        self.free_nodes = []
        self.syn_top = 0                                    # bump allocators
        self.fo_top = 0

    # -- index --------------------------------------------------------------
    def probe(self, src):
        """Slot holding src, or the first empty slot of its probe chain."""
        h = fold(src, self.lognb)
        for _ in range(self.NB):
            e = self.idx[h]
            if not e >> 49 or (e >> 25) & 0xFFFFFF == src:
                return h
            h = (h + 1) % self.NB
        raise RuntimeError("index full")

    def accumulate(self, word):
        w12, d, plastic, _ = syn_unpack(word)
        self.acc[d] += w12 >> 4
        if not self.touched[d]:
            self.touched[d] = True
            self.tl.append(d)
        return plastic


class Chip1:
    def __init__(self, n_tiles, logn, lognb, logs, logf, params=None):
        self.T = n_tiles
        self.p = params or Params()
        self.tiles = [Tile(logn, lognb, logs, logf) for _ in range(n_tiles)]
        self.where = {}                                     # gid -> (tile, local)
        self.ticks = 0
        self.draining = []                                  # (tile, local, tick vacated)

    # ---- construction (also used by firmware) -----------------------------
    @classmethod
    def build(cls, n, thr, links, placement, n_tiles, params=None, plastic=None,
              spare_nodes=4, slack=2.0):
        """links: (src, dst, w float, may be negative). plastic: set of link indices."""
        import math
        place = np.asarray(placement)
        per_tile = np.bincount(place, minlength=n_tiles)
        incoming = [dict() for _ in range(n_tiles)]
        fan = [dict() for _ in range(n_tiles)]
        for k, (s, d, w) in enumerate(links):
            incoming[place[d]].setdefault(s, []).append((d, w, int(plastic is not None and k in plastic)))
            fan[place[s]].setdefault(s, set()).add(int(place[d]))
        lg = lambda x: max(1, math.ceil(math.log2(max(x, 2))))
        logn = lg(per_tile.max() + spare_nodes + 1)
        assert logn <= 8, "v1 synapse word holds an 8-bit local index"
        lognb = lg(max(len(i) for i in incoming) * slack / 0.8 + 1)
        logs = lg(max(sum(len(v) for v in i.values()) for i in incoming) * slack + 1)
        logf = lg(max(sum(len(v) for v in f.values()) for f in fan) * slack + 1)
        chip = cls(n_tiles, logn, lognb, logs, logf, params)
        for g in range(n):
            t = chip.tiles[place[g]]
            li = len([1 for x in chip.where.values() if x[0] == place[g]])
            chip.where[g] = (int(place[g]), li)
            t.node[li] = node_pack(q16(thr[g]), 0, 0)
            t.gid[li] = g
        for ti, t in enumerate(chip.tiles):
            used = {li for (tt, li) in chip.where.values() if tt == ti}
            t.free_nodes = [li for li in range(t.N) if li not in used]
        for ti, inc in enumerate(incoming):
            t = chip.tiles[ti]
            for s, lst in inc.items():
                words = [syn_pack(q12(w), chip.where[d][1], pl, 0) for d, w, pl in lst]
                chip._install(ti, s, words)
        for ti, f in enumerate(fan):
            for s, tiles in f.items():
                chip._set_fanout(s, sorted(tiles))
        return chip

    def _install(self, ti, s, words):
        t = self.tiles[ti]
        h = t.probe(s)
        if len(words) == 1:
            t.idx[h] = (1 << 49) | (s << 25) | (1 << 24) | words[0]
        else:
            ptr = t.syn_top
            assert ptr + len(words) <= t.S, "synapse store full"
            for j, wd in enumerate(words):
                t.syn[ptr + j] = wd
            t.syn_top += len(words)
            t.idx[h] = (1 << 49) | (s << 25) | (ptr << 8) | len(words)

    def _set_fanout(self, s, dst_tiles):
        ti, li = self.where[s]
        t = self.tiles[ti]
        ptr = t.fo_top
        assert ptr + len(dst_tiles) <= t.F, "fan-out table full"
        for j, d in enumerate(dst_tiles):
            t.fo[ptr + j] = d
        t.fo_top += len(dst_tiles)
        t.fo_ptr[li] = (ptr << 8) | len(dst_tiles)

    # ---- one tick -----------------------------------------------------------
    def tick(self, ext=()):
        """ext: gids receiving their threshold as input (injected before the tick).
        Returns the set of gids fired in this tick."""
        p = self.p
        for g in ext:                                       # host injection
            ti, li = self.where[g]
            t = self.tiles[ti]
            thr, _, _ = node_unpack(t.node[li])
            t.acc[li] += thr
            if not t.touched[li]:
                t.touched[li] = True
                t.tl.append(li)
        # COMM: packets of last tick's fired lists; order within a tile does
        # not matter for spikes, and the visit list order only orders
        # independent per-synapse updates, so tiles can be processed in turn.
        packets = [[] for _ in range(self.T)]
        for ti, t in enumerate(self.tiles):
            for li in t.fl:
                ptr, ln = t.fo_ptr[li] >> 8, t.fo_ptr[li] & 0xFF
                for j in range(ln):
                    packets[t.fo[ptr + j]].append(t.gid[li])
        for ti, t in enumerate(self.tiles):
            t.vis = []
            for src in sorted(packets[ti]):
                h = t.probe(src)
                e = t.idx[h]
                if not e >> 49:
                    continue                                 # unknown source (miss)
                if (e >> 24) & 1:
                    if t.accumulate(e & 0xFFFFFF):
                        t.vis.append((1, h))
                else:
                    ptr, ln = (e >> 8) & 0xFFFF, e & 0xFF
                    for j in range(ln):
                        if t.accumulate(t.syn[ptr + j]):
                            t.vis.append((0, ptr + j))
        fired = set()
        self.ticks += 1
        self.last_fired = fired
        for t in self.tiles:
            t.firedv = [False] * t.N
            t.fl = []
            order = range(t.N) if p.full() else list(t.tl)
            for li in order:
                thr, refr, v = node_unpack(t.node[li])
                v = clip(((v * p.leak) >> 8) + t.acc[li], *I16)
                if refr > 0:
                    v, refr = 0, refr - 1
                fire = v >= thr
                if fire:
                    v, refr = 0, p.refr
                    t.fl.append(li)
                    t.firedv[li] = True
                    fired.add(t.gid[li])
                if p.homeo:
                    r = t.rate[li]
                    r = min(255, r - (r >> p.rs) + ((256 >> p.rs) if fire else 0))
                    t.rate[li] = r
                    thr = clip(thr + ((r - p.target) >> p.hs), p.tmin, p.tmax)
                t.node[li] = node_pack(thr, refr, v)
                t.acc[li] = 0
                t.touched[li] = False
            t.tl = []
            if p.learn:
                for kind, addr in t.vis:
                    word = (t.idx[addr] & 0xFFFFFF) if kind else t.syn[addr]
                    word = self._learn(t, word)
                    if kind:
                        t.idx[addr] = (t.idx[addr] & ~0xFFFFFF) | word
                    else:
                        t.syn[addr] = word
        return fired

    def _learn(self, t, word):
        p = self.p
        w, d, pl, cr = syn_unpack(word)
        if not pl:
            return word
        if w < 0:                                            # Art. 7b
            if t.rate[d] > p.target:
                w = max(w - p.il, W12_MIN)
            elif t.rate[d] < p.target:
                w = min(w + p.il, 0)
        elif t.firedv[d]:                                    # Art. 2 + credit
            w = w + ((W12_MAX - w) >> p.lp)
            cr = min(cr + 1, 7)
        else:                                                # Art. 3
            w = w - p.dec
            if w <= p.prune:
                w, pl, cr = 0, 0, 0
        return syn_pack(w, d, pl, cr)

    def reward(self, delta):
        """Art. 8 sweep. delta: signed 8-bit (reward minus expectation, set by the teacher)."""
        p = self.p
        for t in self.tiles:
            for h in range(t.NB):
                e = t.idx[h]
                if e >> 49 and (e >> 24) & 1:
                    t.idx[h] = (e & ~0xFFFFFF) | self._rew(e & 0xFFFFFF, delta)
            for a in range(t.S):
                t.syn[a] = self._rew(t.syn[a], delta)

    def _rew(self, word, delta):
        w, d, pl, cr = syn_unpack(word)
        if not pl or not cr:
            return word
        dw = (delta * cr) >> self.p.rsh
        w = clip(w + dw, 0, W12_MAX) if w >= 0 else clip(w + dw, W12_MIN, 0)
        return syn_pack(w, d, pl, 0)


def q16(x):
    return clip(int(round(x * 16)), *I16)


def q12(x):
    return clip(int(round(x * 256)), W12_MIN, W12_MAX)


class Firmware:
    """Management-core procedures. Each operation is logged as a config-port
    op so the identical sequence can be replayed on the RTL:
        ("cp",  ti, sel, addr, tj, sel2, addr2, set_dst)   copy a word (runtime value)
        ("wr",  ti, sel, addr, value)                       write a constant
    sel: 0 node, 1 gid, 2 fo_ptr, 3 fo, 4 idx, 5 syn, 6 rate.
    'cp' between index (4) and synapse (5) moves the 24-bit synapse word;
    set_dst rewrites the word's dst field (local index changes on migration).
    """

    def __init__(self, chip):
        self.c = chip
        self.ops = []

    def _get(self, ti, sel, a):
        t = self.c.tiles[ti]
        return {0: t.node, 1: t.gid, 2: t.fo_ptr, 3: t.fo, 4: t.idx, 5: t.syn, 6: t.rate}[sel][a]

    def _put(self, ti, sel, a, v):
        t = self.c.tiles[ti]
        {0: t.node, 1: t.gid, 2: t.fo_ptr, 3: t.fo, 4: t.idx, 5: t.syn, 6: t.rate}[sel][a] = v

    def wr(self, ti, sel, a, v):
        self.ops.append(("wr", ti, sel, a, v))
        self._put(ti, sel, a, v)

    def cp(self, ti, sel, a, tj, sel2, a2, set_dst=None):
        self.ops.append(("cp", ti, sel, a, tj, sel2, a2, -1 if set_dst is None else set_dst))
        v = self._get(ti, sel, a)
        if sel in (4, 5) or sel2 in (4, 5):
            word = v & 0xFFFFFF
            if set_dst is not None:
                word = (word & ~0xFF0) | ((set_dst & 0xFF) << 4)
            if sel2 == 4:
                old = self._get(tj, 4, a2)
                v = (old & ~0xFFFFFF) | word
            else:
                v = word
        self._put(tj, sel2, a2, v)

    def migrate(self, g, tb):
        """Move node g to tile tb (between ticks). Synapses keep their runtime
        weights and learning state; senders' fan-out lists are corrected now
        (eager), so no packet is ever addressed to a stale tile."""
        c = self.c
        self.retire()
        ta, la = c.where[g]
        if ta == tb:
            return
        A, B = c.tiles[ta], c.tiles[tb]
        lb = B.free_nodes.pop(0)
        # 1. node state, rate, id
        self.cp(ta, 0, la, tb, 0, lb)
        self.cp(ta, 6, la, tb, 6, lb)
        self.cp(ta, 1, la, tb, 1, lb)
        # 2. its own fan-out list moves along
        fp = A.fo_ptr[la]
        ptr, ln = fp >> 8, fp & 0xFF
        nptr = B.fo_top
        assert nptr + ln <= B.F
        for j in range(ln):
            self.cp(ta, 3, ptr + j, tb, 3, nptr + j)
        B.fo_top += ln
        self.wr(tb, 2, lb, (nptr << 8) | ln)
        # 3. vacate the old slot. It keeps its id and fan-out for one more
        #    tick: if g fired in the last tick, that spike is sent from here.
        self.wr(ta, 0, la, node_pack(32767, 0, 0))
        self.wr(ta, 6, la, 0)
        c.draining.append((ta, la, c.ticks))
        c.where[g] = (tb, lb)
        # 4. incoming synapses: every word in A whose dst is la
        for h in range(A.NB):
            e = A.idx[h]
            if not e >> 49:
                continue
            src = (e >> 25) & 0xFFFFFF
            if (e >> 24) & 1:
                locs = [(4, h)] if ((e >> 4) & 0xFF) == la and self._live(e & 0xFFFFFF) else []
            else:
                ptr, ln = (e >> 8) & 0xFFFF, e & 0xFF
                locs = [(5, ptr + j) for j in range(ln)
                        if ((A.syn[ptr + j] >> 4) & 0xFF) == la and self._live(A.syn[ptr + j])]
            for sel, a in locs:
                self._move_synapse(ta, sel, a, tb, src, lb)

    def retire(self):
        """Free slots vacated at least one tick ago."""
        c = self.c
        keep = []
        for ta, la, t0 in c.draining:
            if c.ticks > t0:
                self.wr(ta, 2, la, 0)
                c.tiles[ta].free_nodes.append(la)
            else:
                keep.append((ta, la, t0))
        c.draining = keep

    @staticmethod
    def _live(word):
        w, _, pl, _ = syn_unpack(word)
        return pl or w != 0

    # ---- Art. 4: new links ---------------------------------------------------
    def add_link(self, s, d, w12, plastic=1):
        """Create synapse s -> d (between ticks). Allocation lives here, in
        firmware, not in the data path."""
        tb, lb = self.c.where[d]
        self._place_word(tb, s, None, syn_pack(w12, lb, plastic, 0))

    def grow(self, rng, k, w12=48):
        """Art. 4: up to k new links between nodes that fired together in
        the last tick (pairs chosen at random, existing links skipped)."""
        fired = sorted(getattr(self.c, "last_fired", ()))
        made = 0
        for _ in range(4 * k):
            if made == k or len(fired) < 2:
                break
            s, d = (int(x) for x in rng.choice(fired, 2, replace=False))
            if self._has_link(s, d):
                continue
            self.add_link(s, d, w12)
            made += 1
        return made

    def _has_link(self, s, d):
        tb, lb = self.c.where[d]
        B = self.c.tiles[tb]
        e = B.idx[B.probe(s)]
        if not e >> 49:
            return False
        if (e >> 24) & 1:
            words = [e & 0xFFFFFF]
        else:
            ptr, ln = (e >> 8) & 0xFFFF, e & 0xFF
            words = B.syn[ptr:ptr + ln]
        return any(((w >> 4) & 0xFF) == lb and self._live(w) for w in words)

    # ---- Art. 5: influence budget (incoming, synaptic scaling) ---------------
    def budget(self, cap):
        """Every node whose positive plastic incoming weight sum exceeds cap
        (1/256 units) has those weights reduced by 1/8."""
        for ti, t in enumerate(self.c.tiles):
            locs = []
            for h in range(t.NB):
                e = t.idx[h]
                if not e >> 49:
                    continue
                if (e >> 24) & 1:
                    locs.append((4, h, e & 0xFFFFFF))
                else:
                    ptr, ln = (e >> 8) & 0xFFFF, e & 0xFF
                    locs += [(5, ptr + j, t.syn[ptr + j]) for j in range(ln)]
            tot = [0] * t.N
            for _, _, wd in locs:
                w, d, pl, _ = syn_unpack(wd)
                if pl and w > 0:
                    tot[d] += w
            for sel, a, wd in locs:
                w, d, pl, cr = syn_unpack(wd)
                if pl and w > 0 and tot[d] > cap:
                    nw = syn_pack(w - (w >> 3), d, pl, cr)
                    if sel == 5:
                        self.wr(ti, 5, a, nw)
                    else:
                        self.wr(ti, 4, a, (t.idx[a] & ~0xFFFFFF) | nw)

    # ---- shared: put one synapse word for source s into tile tb --------------
    def _place_word(self, tb, src, from_loc, new_word):
        """from_loc = (tile, sel, addr, set_dst) to copy a runtime word, or None
        to write new_word."""
        c = self.c
        B = c.tiles[tb]
        h = B.probe(src)
        e = B.idx[h]

        def put(sel2, a2):
            if from_loc is None:
                if sel2 == 4:
                    self.wr(tb, 4, a2, (B.idx[a2] & ~0xFFFFFF) | new_word)
                else:
                    self.wr(tb, 5, a2, new_word)
            else:
                ta, sel, a, sd = from_loc
                self.cp(ta, sel, a, tb, sel2, a2, set_dst=sd)

        if not e >> 49:
            self.wr(tb, 4, h, (1 << 49) | (src << 25) | (1 << 24))
            put(4, h)
        else:
            if (e >> 24) & 1:
                old = [(4, h)]
            else:
                p0, l0 = (e >> 8) & 0xFFFF, e & 0xFF
                old = [(5, p0 + j) for j in range(l0)]
            nptr = B.syn_top
            assert nptr + len(old) + 1 <= B.S, "synapse store full"
            for j, (s2, a2) in enumerate(old):
                self.cp(tb, s2, a2, tb, 5, nptr + j)
            put(5, nptr + len(old))
            B.syn_top += len(old) + 1
            self.wr(tb, 4, h, (1 << 49) | (src << 25) | (nptr << 8) | (len(old) + 1))
        self._ensure_fanout(src, tb)

    def _ensure_fanout(self, src, tb):
        c = self.c
        st, sl = c.where[src]
        S = c.tiles[st]
        fp = S.fo_ptr[sl]
        ptr, ln = fp >> 8, fp & 0xFF
        dsts = [S.fo[ptr + j] for j in range(ln)]
        if tb not in dsts:
            nptr = S.fo_top
            assert nptr + ln + 1 <= S.F, "fan-out table full"
            for j in range(ln):
                self.cp(st, 3, ptr + j, st, 3, nptr + j)
            self.wr(st, 3, nptr + ln, tb)
            S.fo_top += ln + 1
            self.wr(st, 2, sl, (nptr << 8) | (ln + 1))

    def _move_synapse(self, ta, sel, a, tb, src, lb):
        self._place_word(tb, src, (ta, sel, a, lb), None)
        if sel == 5:
            self.wr(ta, 5, a, 0)
        else:
            self._tomb_inline(ta, a)

    def _tomb_inline(self, ta, h):
        e = self.c.tiles[ta].idx[h]
        self.wr(ta, 4, h, (e & ~0xFFFFFF) | (1 << 24))       # inline, word 0
