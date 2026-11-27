"""e08: Polis at scale — memory formats, placement, cycles, area, energy.

Fully vectorised (numpy), so it reaches hundreds of thousands of nodes.
Workload is synthetic but brain-like:
  * N nodes in modules of 256; each node has K outgoing links, a fraction
    `local` inside its own module, the rest to uniformly random nodes
  * activity: each tick a random 10% of modules are "active" (rate 0.1),
    the rest fire at 0.005
Placements (tile = 256 nodes, region = 4x4 tiles):
  random   : nodes scattered at random (no migration)
  packed   : each module on one tile  (migration optimising traffic only)
  striped  : each module spread over the 16 tiles of one region
             (migration optimising traffic + work balance)
Memory formats per tile (see docs/tile.md):
  base     : 48-bit hash entries (24-bit src key), load 0.8, inline singletons,
             24-bit synapses for longer segments
  quotient : hash position stands in for log2(buckets) bits of the key
  region   : sources from the same region use a 12-bit region-local key
  both     : quotient + region
Area/energy use stated assumptions (see printed header and docs/scale.md).

Run: python experiments/e08_scale.py [--n 65536] [--k 64] [--local 0.9]
"""
import argparse
import math

import numpy as np

C = 256            # nodes per tile
MOD = 256          # nodes per module
RT = 4             # region = RT x RT tiles
LANES = 8
PROBE = 3.0        # expected SRAM accesses per successful lookup, linear probing, load 0.8
LOAD = 0.8

# ---- assumptions for area / energy ----------------------------------------
SRAM_MB_PER_MM2 = {"N5-class (assumed macro 25 Mb/mm2)": 25.0,
                   "N2 (38.1 Mb/mm2 macro, TSMC)": 38.1}
LOGIC_MM2_PER_TILE = (0.01, 0.05)     # router + datapath + control, assumed range
E_SRAM_PJ_PER_BIT_45NM = 20 / 64      # 32KB SRAM, 64-bit read = 20 pJ (Horowitz, 45nm)
E_ADD8_PJ_45NM = 0.03
E_HOP_PJ_PER_BIT = 0.1                # assumed on-chip link energy per bit per hop
NODE_BITS, SYN_BITS, ENTRY_BITS, FANOUT_BITS, PKT_BITS = 40, 24, 48, 12, 40


def build(n, k, local, rng):
    src = np.repeat(np.arange(n, dtype=np.int64), k)
    mod = src // MOD
    is_local = rng.random(src.size) < local
    dst = np.where(is_local, mod * MOD + rng.integers(0, MOD, src.size),
                   rng.integers(0, n, src.size))
    return src, dst


def placements(n, rng):
    n_tiles = n // C
    grid = int(math.isqrt(n_tiles))
    assert grid * grid == n_tiles and grid % RT == 0, "n must make a square grid of 4x4 regions"
    nodes = np.arange(n)
    # tile numbering: region-major so that 16 consecutive tiles form one region
    def region_major_to_xy(t):
        r, i = t // (RT * RT), t % (RT * RT)
        rpr = grid // RT
        return (r % rpr) * RT + i % RT, (r // rpr) * RT + i // RT
    tx, ty = region_major_to_xy(np.arange(n_tiles))
    tile_id = ty * grid + tx                       # mesh index of region-major tile t
    out = {}
    out["random"] = tile_id[rng.permutation(n) // C]
    out["packed"] = tile_id[nodes // C]            # module m -> tile m
    m, j = nodes // MOD, nodes % MOD               # module m -> region (m // 16), striped
    reg, slot = m // (RT * RT), m % (RT * RT)
    t_rm = reg * RT * RT + (j * (RT * RT) // MOD + slot) % (RT * RT)
    out["striped"] = tile_id[t_rm]
    return out, grid


def region_of(tile, grid):
    return (tile // grid // RT) * (grid // RT) + (tile % grid) // RT


def memory(src, dst, tile, n, grid):
    n_tiles = n // C
    dt = tile[dst]
    key = src * n_tiles + dt
    pairs, seglen = np.unique(key, return_counts=True)
    p_src, p_tile = pairs // n_tiles, pairs % n_tiles
    entries = np.bincount(p_tile, minlength=n_tiles)
    buckets = np.ceil(entries / LOAD)
    same_region = region_of(tile[p_src], grid) == region_of(p_tile, grid)
    loc_entries = np.bincount(p_tile, weights=same_region, minlength=n_tiles)
    rem_entries = entries - loc_entries
    syn_long = np.bincount(p_tile, weights=np.where(seglen > 1, seglen, 0), minlength=n_tiles)
    fanout = np.bincount(tile[p_src], minlength=n_tiles) * FANOUT_BITS
    node = np.full(n_tiles, C * NODE_BITS)

    def qbits(b):
        return np.floor(np.log2(np.maximum(b, 1)))

    res = {}
    for fmt in ("base", "quotient", "region", "both"):
        if fmt in ("base", "quotient"):
            saved = qbits(buckets) if fmt == "quotient" else 0
            idx = buckets * (ENTRY_BITS - saved)
        else:  # two tables: region-local keys 12 bits shorter
            bl, br = np.ceil(loc_entries / LOAD), np.ceil(rem_entries / LOAD)
            sl = qbits(bl) if fmt == "both" else 0
            sr = qbits(br) if fmt == "both" else 0
            idx = bl * (ENTRY_BITS - 12 - sl) + br * (ENTRY_BITS - sr)
        per_tile = idx + syn_long * SYN_BITS + fanout + node
        res[fmt] = per_tile
    return res, len(pairs), seglen.mean()


def run_ticks(src, dst, tile, n, grid, rng, ticks):
    n_tiles = n // C
    order = np.lexsort((tile[dst], src))
    s_sorted, d_sorted = src[order], dst[order]
    pair_key = s_sorted * n_tiles + tile[d_sorted]
    first = np.r_[True, pair_key[1:] != pair_key[:-1]]
    pair_src, pair_tile = s_sorted[first], tile[d_sorted][first]
    stats = []
    n_mod = n // MOD
    for _ in range(ticks):
        active = rng.random(n_mod) < 0.1
        rate = np.where(active[np.arange(n) // MOD], 0.1, 0.005)
        fired = rng.random(n) < rate
        pk = fired[pair_src]
        p_s, p_t = pair_src[pk], pair_tile[pk]
        e = fired[s_sorted]
        syn_t = tile[d_sorted[e]]
        touched = np.unique(d_sorted[e])
        recv = PROBE * np.bincount(p_t, minlength=n_tiles) + np.bincount(syn_t, minlength=n_tiles)
        upd = np.ceil(np.bincount(tile[touched], minlength=n_tiles) / LANES)
        send = np.bincount(tile[p_s], minlength=n_tiles)
        comp = recv + upd + send
        # XY link loads via difference arrays
        a, b = tile[p_s], p_t
        ax, ay, bx, by = a % grid, a // grid, b % grid, b // grid
        hops = np.abs(ax - bx) + np.abs(ay - by)
        h = np.zeros((2, grid, grid + 1))              # [dir, row, x]
        right = bx > ax
        np.add.at(h[0], (ay[right], ax[right]), 1)
        np.add.at(h[0], (ay[right], bx[right]), -1)
        left = bx < ax
        np.add.at(h[1], (ay[left], bx[left]), 1)
        np.add.at(h[1], (ay[left], ax[left]), -1)
        v = np.zeros((2, grid, grid + 1))              # [dir, col, y]
        down = by > ay
        np.add.at(v[0], (bx[down], ay[down]), 1)
        np.add.at(v[0], (bx[down], by[down]), -1)
        up = by < ay
        np.add.at(v[1], (bx[up], by[up]), 1)
        np.add.at(v[1], (bx[up], ay[up]), -1)
        link = max(np.cumsum(h, axis=2).max(), np.cumsum(v, axis=2).max())
        net = max(link, hops.max() if hops.size else 0)
        energy_pj = (E_SRAM_PJ_PER_BIT_45NM * (PROBE * len(p_t) * ENTRY_BITS
                                               + len(syn_t) * SYN_BITS
                                               + 2 * len(touched) * NODE_BITS)
                     + E_ADD8_PJ_45NM * len(syn_t)
                     + E_HOP_PJ_PER_BIT * hops.sum() * PKT_BITS)
        stats.append((fired.sum(), len(syn_t), len(p_t), comp.max(), comp.mean(), net,
                      hops.sum() * PKT_BITS, energy_pj))
    return np.array(stats).mean(axis=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=65536)
    ap.add_argument("--k", type=int, default=64)
    ap.add_argument("--local", type=float, default=0.9)
    ap.add_argument("--ticks", type=int, default=30)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    src, dst = build(a.n, a.k, a.local, rng)
    places, grid = placements(a.n, rng)
    print(f"N={a.n} nodes, K={a.k} links/node ({len(src):,} synapses), local={a.local}, "
          f"{grid}x{grid} tiles of {C}\n")
    print(f"{'placement':>9} | {'bits/synapse  base  quot  region  both':>38} | "
          f"{'worst tile KiB (both)':>21} | {'seg len':>7}")
    mem = {}
    for name, tile in places.items():
        res, n_pairs, seg = memory(src, dst, tile, a.n, grid)
        mem[name] = res
        bps = {f: v.sum() / len(src) for f, v in res.items()}
        print(f"{name:>9} | {'':>12}{bps['base']:6.1f}{bps['quotient']:6.1f}{bps['region']:8.1f}"
              f"{bps['both']:6.1f} | {res['both'].max() / 8 / 1024:21.1f} | {seg:7.2f}")
    print(f"\n{'placement':>9} | {'spikes':>6} {'syn ops':>8} {'packets':>8} | "
          f"{'compute max':>11} {'mean':>6} {'network':>7} {'tick':>6} | {'kbit-hop':>8} | "
          f"{'energy/tick (45nm SRAM)':>23}")
    perf = {}
    for name, tile in places.items():
        s = run_ticks(src, dst, tile, a.n, grid, np.random.default_rng(a.seed + 7), a.ticks)
        perf[name] = s
        print(f"{name:>9} | {s[0]:6.0f} {s[1]:8.0f} {s[2]:8.0f} | {s[3]:11.0f} {s[4]:6.1f} "
              f"{s[5]:7.0f} {s[3] + s[5]:6.0f} | {s[6] / 1000:8.1f} | {s[7] / 1e6:19.2f} uJ")

    best = min(perf, key=lambda k: perf[k][3] + perf[k][5])
    tot_bits = mem[best]["both"].sum()
    n_tiles = a.n // C
    print(f"\narea estimate (placement '{best}', format 'both'): "
          f"SRAM {tot_bits / 8 / 2**20:.1f} MiB")
    for proc, dens in SRAM_MB_PER_MM2.items():
        sram_mm2 = tot_bits / 1e6 / dens
        lo, hi = (n_tiles * x for x in LOGIC_MM2_PER_TILE)
        print(f"  {proc:<36} SRAM {sram_mm2:7.1f} mm2 + logic {lo:.0f}-{hi:.0f} mm2 "
              f"= {sram_mm2 + lo:.0f}-{sram_mm2 + hi:.0f} mm2")
    s = perf[best]
    print(f"energy per synaptic op (45nm SRAM, incl. network): {s[7] / max(s[1], 1):.1f} pJ")


if __name__ == "__main__":
    main()
