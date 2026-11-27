"""Migration protocol: moving nodes without losing a single packet.

Problem. A sender knows only the destination's virtual ID (its 'resident
number'). The node may have moved since the sender last looked.

Protocol (mode="polis"):
  * Home directory  — every ID has a home tile, home(id) = id mod tiles,
                      that always knows where the node lives now.
  * Address cache   — each tile caches ID -> tile for the IDs it sends to
                      (like a TLB). Miss: ask the home (2 messages).
  * Forwarding      — when a node leaves a tile, the tile keeps a
                      forwarding pointer for `fwd_ttl` ticks. A packet that
                      lands at the old tile is forwarded, and the sender's
                      cache is corrected (1 message).
  * Fallback        — packet lands where the node is not and no pointer is
                      left: the tile asks the home and forwards.
  * Tick barrier    — moves happen only between ticks, and a tick ends only
                      when all its packets (forwarded ones too) have arrived.
                      Forwarding costs time, never correctness.

Baseline (mode="naive"): the move updates the home but nobody tells the
caches, and no forwarding pointers are left. Stale packets are lost.
"""
from collections import defaultdict

import numpy as np


class Network:
    def __init__(self, n_nodes, n_tiles, mode="polis", fwd_ttl=8, rng=None, pinned=()):
        self.n_tiles, self.mode, self.fwd_ttl = n_tiles, mode, fwd_ttl
        self.rng = rng or np.random.default_rng(0)
        self.loc = self.rng.integers(0, n_tiles, n_nodes)      # ground truth
        self.directory = self.loc.copy()                       # held at the homes
        self.cache = [dict() for _ in range(n_tiles)]
        self.fwd = [dict() for _ in range(n_tiles)]            # id -> (tile, expiry)
        self.pinned = set(pinned)
        self.movable = np.array([i for i in range(n_nodes) if i not in self.pinned])
        self.tick = 0
        self.stats = defaultdict(int)

    def home(self, i):
        return i % self.n_tiles

    # ---- data path -----------------------------------------------------
    def deliver(self, src, dst):
        st = self.loc[src]
        self.stats["packets"] += 1
        guess = self.cache[st].get(dst)
        if guess is None:
            self.stats["home_queries"] += 1                    # miss: ask home
            guess = int(self.directory[dst])
            self.cache[st][dst] = guess
        at, steps = guess, 0
        while self.loc[dst] != at:
            if self.mode == "naive":
                self.stats["lost"] += 1
                return False
            entry = self.fwd[at].get(dst)
            if entry and entry[1] >= self.tick:
                at = entry[0]
                self.stats["forwards"] += 1
            else:
                at = int(self.directory[dst])                 # fallback
                self.stats["fallbacks"] += 1
            steps += 1
            assert steps < 64, "forwarding loop"
        if at != guess:
            self.cache[st][dst] = at                           # correct sender
            self.stats["corrections"] += 1
        return True

    # ---- control path (between ticks only) ------------------------------
    def migrate(self, rate):
        """Move on average `rate` random (unpinned) nodes; called between ticks."""
        k = min(int(self.rng.poisson(rate)), len(self.movable))
        for u in self.rng.choice(self.movable, k, replace=False):
            old, new = int(self.loc[u]), int(self.rng.integers(self.n_tiles))
            if new == old:
                continue
            self.loc[u] = new
            self.directory[u] = new                            # tell the home
            self.stats["moves"] += 1
            self.stats["dir_updates"] += 1
            if self.mode == "polis":
                self.fwd[old][u] = (new, self.tick + self.fwd_ttl)
        self.tick += 1
        if self.tick % 32 == 0:                                # expire pointers
            for f in self.fwd:
                for i in [i for i, (_, e) in f.items() if e < self.tick]:
                    del f[i]
