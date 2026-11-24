"""Physical substrate: the lower layer of Polis.

A grid of tiles (citizens). Each tile hosts up to `capacity` virtual
nodes. Tiles are grouped into regions (demes). A spike between two
virtual nodes becomes a packet that travels Manhattan hops between their
tiles. Nodes migrate so that chatty nodes end up close together.

Governance:
  * intra-region moves: decided by the region itself, no limit
  * inter-region moves: proposed to the region representatives, who
    approve at most `treaty_budget` swaps per region pair per round,
    best gains first
  * the representative is re-elected every round: the least-loaded tile
"""
from collections import defaultdict

import numpy as np


class Substrate:
    def __init__(self, n, grid=8, capacity=16, region_tiles=4, rng=None,
                 treaty_budget=8, traffic_halflife=4):
        assert n <= grid * grid * capacity
        self.n, self.grid, self.cap = n, grid, capacity
        self.rng = rng or np.random.default_rng(0)
        self.n_tiles = grid * grid
        self.tx = np.arange(self.n_tiles) % grid
        self.ty = np.arange(self.n_tiles) // grid
        rpr = grid // region_tiles
        self.region_of_tile = (self.ty // region_tiles) * rpr + (self.tx // region_tiles)
        self.n_regions = rpr * rpr
        slots = np.repeat(np.arange(self.n_tiles), capacity)[:n]
        self.tile = self.rng.permutation(slots)
        self.traffic = [defaultdict(float) for _ in range(n)]
        self.load = np.zeros(self.n_tiles)
        self.treaty_budget = treaty_budget
        self.traffic_decay = 0.5 ** (1 / traffic_halflife)
        self.reps = np.zeros(self.n_regions, dtype=int)
        self.stats = defaultdict(int)

    # ---- routing -------------------------------------------------------
    def route(self, src, dst):
        """Send packets; return hop counts per packet."""
        ts, td = self.tile[src], self.tile[dst]
        hops = np.abs(self.tx[ts] - self.tx[td]) + np.abs(self.ty[ts] - self.ty[td])
        for s, d in zip(src.tolist(), dst.tolist()):
            self.traffic[s][d] += 1
            self.traffic[d][s] += 1
        np.add.at(self.load, ts, 1)
        return hops

    def region(self, node):
        return self.region_of_tile[self.tile[node]]

    # ---- migration -----------------------------------------------------
    def _cost(self, u, t):
        tr = self.traffic[u]
        if not tr:
            return 0.0
        nb = np.fromiter(tr.keys(), int)
        w = np.fromiter(tr.values(), float)
        tn = self.tile[nb]
        return float(w @ (np.abs(self.tx[t] - self.tx[tn]) + np.abs(self.ty[t] - self.ty[tn])))

    def _wanted_tile(self, u):
        """Weighted median of partners' positions (minimises Manhattan cost)."""
        tr = self.traffic[u]
        nb = np.fromiter(tr.keys(), int)
        w = np.fromiter(tr.values(), float)
        tn = self.tile[nb]

        def wmed(vals):
            o = np.argsort(vals)
            cw = np.cumsum(w[o])
            return vals[o][np.searchsorted(cw, cw[-1] / 2)]
        return int(wmed(self.ty[tn]) * self.grid + wmed(self.tx[tn]))

    def _best_swap(self, u):
        a = self.tile[u]
        b = self._wanted_tile(u)
        if b == a:
            return None
        cu = self._cost(u, a) - self._cost(u, b)
        best, gain = None, 0.0
        for x in np.flatnonzero(self.tile == b):
            g = cu + self._cost(x, b) - self._cost(x, a)
            if g > gain:
                best, gain = int(x), g
        return (gain, u, best, a, b) if best is not None else None

    def migrate(self, sample=256):
        # elections: least-loaded tile in each region becomes representative
        for r in range(self.n_regions):
            tiles = np.flatnonzero(self.region_of_tile == r)
            self.reps[r] = tiles[np.argmin(self.load[tiles])]

        active = [u for u in range(self.n) if self.traffic[u]]
        if active:
            pick = self.rng.choice(active, min(sample, len(active)), replace=False)
        else:
            pick = []
        treaties = defaultdict(list)
        for u in pick:
            p = self._best_swap(int(u))
            if p is None:
                continue
            _, u, x, a, b = p
            ra, rb = self.region_of_tile[a], self.region_of_tile[b]
            if ra == rb:
                self._swap(u, x)            # local self-government
                self.stats["intra"] += 1
            else:
                treaties[(min(ra, rb), max(ra, rb))].append(p)
                self.stats["proposed"] += 1
        # parliament: reps of each region pair approve the best swaps
        for props in treaties.values():
            props.sort(key=lambda p: -p[0])
            for g, u, x, a, b in props[: self.treaty_budget]:
                if self.tile[u] == a and self.tile[x] == b:  # still valid
                    self._swap(u, x)
                    self.stats["inter"] += 1

        for i, tr in enumerate(self.traffic):
            nt = defaultdict(float)
            for k, v in tr.items():
                v *= self.traffic_decay
                if v > 0.05:
                    nt[k] = v
            self.traffic[i] = nt
        self.load *= self.traffic_decay

    def _swap(self, u, x):
        self.tile[u], self.tile[x] = self.tile[x], self.tile[u]
