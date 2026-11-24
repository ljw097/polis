"""Build RTL memory images from a Polis program, simulate the RTL, and
compare it tick by tick with the Python hardware model (sim/polis/hw.py).

    python tools/rtl_build.py adder      # 8-bit adder, 4x4 mesh
    python tools/rtl_build.py fuzz       # random compiled programs

For every tick the set of fired nodes from the RTL must equal the set from
`hw.Chip.step`. Any difference is reported and the script exits non-zero.
"""
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "sim"))
from polis import hw  # noqa: E402
from polis.program import Program, ripple_adder  # noqa: E402

SIM = os.environ.get("POLIS_SIM", "verilator")    # or "iverilog"
RTL = [os.path.join(ROOT, "rtl", f) for f in ("polis_router.v", "polis_tile.v", "polis_mesh.v")]


def fold(src, b):
    h, mask = 0, (1 << b) - 1
    for i in range(0, 24, b):
        h ^= (src >> i) & mask
    return h


def clog2(x):
    return max(1, math.ceil(math.log2(max(x, 2))))


def images(chip):
    """Per-tile memory contents in RTL layout, plus the size parameters."""
    tiles = []
    for t in chip.tiles:
        thr = [int(v) & 0xFFFF for v in t.thr]
        gid = list(t.nodes)
        fo_ptr, fo = [], []
        for li in range(len(t.nodes)):
            dsts = t.fanout.get(li, [])
            fo_ptr.append((len(fo), len(dsts)))
            fo.extend(dsts)
        idx, syn = [], []
        for src, (ptr, ln) in t.index.items():
            entries = [((int(t.syn_dst[ptr + j]) & 0xFFF) << 12) | ((int(t.syn_w[ptr + j]) & 0xFF) << 4)
                       for j in range(ln)]
            if ln == 1:
                idx.append((src, 1, entries[0]))
            else:
                idx.append((src, 0, (len(syn) << 8) | ln))
                syn.extend(entries)
        tiles.append(dict(thr=thr, gid=gid, fo_ptr=fo_ptr, fo=fo, idx=idx, syn=syn))
    logn = clog2(max(len(t["thr"]) for t in tiles) + 1)
    lognb = clog2(max(math.ceil(len(t["idx"]) / 0.8) for t in tiles) + 1)
    logs = clog2(max(len(t["syn"]) for t in tiles) + 1)
    logf = clog2(max(len(t["fo"]) for t in tiles) + 1)
    for t in tiles:                                   # hash tables in RTL layout
        nb = 1 << lognb
        table = [0] * nb
        for src, inline, payload in t["idx"]:
            h = fold(src, lognb)
            while table[h]:
                h = (h + 1) % nb
            table[h] = (1 << 49) | (src << 25) | (inline << 24) | payload
        t["idx_table"] = table
    return tiles, dict(LOGN=logn, LOGNB=lognb, LOGS=logs, LOGF=logf)


def write_hex(path, values, width_bits, depth):
    digits = (width_bits + 3) // 4
    vals = list(values) + [0] * (depth - len(values))
    with open(path, "w") as f:
        f.write("\n".join(f"{v:0{digits}x}" for v in vals) + "\n")


def build_tb(workdir, chip, tiles, p, gridb, stim, ticks):
    """stim[t] = list of (tile, local) external inputs at tick t."""
    G = 1 << gridb
    lines = [
        "`timescale 1ns/1ps",
        "module tb;",
        "  reg clk = 0, rst = 1, go = 0, ext_valid = 0;",
        f"  reg [{2 * gridb - 1}:0] ext_tile = 0;",
        f"  reg [{p['LOGN'] - 1}:0] ext_local = 0;",
        "  wire done; wire [31:0] cycles, misses;",
        "  integer fo, k;",
        "  always #1 clk = ~clk;",
        f"  polis_mesh #(.GRIDB({gridb}), .LOGN({p['LOGN']}), .LOGNB({p['LOGNB']}),"
        f" .LOGS({p['LOGS']}), .LOGF({p['LOGF']})) dut (",
        "    .clk(clk), .rst(rst), .go(go), .done(done), .cycles(cycles), .misses(misses),",
        "    .cfg_we(1'b0), .cfg_tile(0), .cfg_sel(3'd0), .cfg_addr(16'd0), .cfg_data(50'd0),",
        "    .ext_valid(ext_valid), .ext_tile(ext_tile), .ext_local(ext_local));",
        "  task inject(input integer t, input integer l); begin",
        "    @(negedge clk); ext_valid = 1; ext_tile = t; ext_local = l;",
        "    @(negedge clk); ext_valid = 0; end endtask",
        "  task tick; begin",
        "    @(negedge clk); go = 1; @(negedge clk); go = 0;",
        "    while (!done) @(posedge clk);",
        "    @(negedge clk); end endtask",
        "  initial begin",
    ]
    names = ["thr_mem", "gid_mem", "fo_ptr", "fo_mem", "idx_mem", "syn_mem"]
    for ti in range(len(tiles)):
        y, x = divmod(ti, G)
        h = f"dut.g_y[{y}].g_x[{x}].u_tile"
        for nm in names:
            lines.append(f'    $readmemh("{workdir}/t{ti}_{nm}.hex", {h}.{nm});')
    lines += [f'    fo = $fopen("{workdir}/rtl_out.txt", "w");',
              "    repeat (4) @(negedge clk); rst = 0; repeat (2) @(negedge clk);"]
    for t in range(ticks):
        for (ti, li) in stim[t]:
            lines.append(f"    inject({ti}, {li});")
        lines.append("    tick;")
        lines.append(f'    $fdisplay(fo, "C {t} %0d", cycles);')
        for ti in range(len(tiles)):
            y, x = divmod(ti, G)
            h = f"dut.g_y[{y}].g_x[{x}].u_tile"
            lines.append(f"    for (k = 0; k < {h}.fl_cnt; k = k + 1)"
                         f' $fdisplay(fo, "F {t} %0d", {h}.gid_mem[{h}.fl_mem[k]]);')
    lines += ['    $fdisplay(fo, "M %0d", misses);', "    $fclose(fo); $finish;", "  end",
              "endmodule"]
    path = os.path.join(workdir, "tb.v")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return path


def cosim(chip, gridb, stim, ticks, keep=None):
    """Run the RTL and the Python model on the same stimulus; compare."""
    tiles, p = images(chip)
    work = keep or tempfile.mkdtemp(prefix="polis_rtl_")
    os.makedirs(work, exist_ok=True)
    widths = dict(thr_mem=16, gid_mem=24, fo_ptr=p["LOGF"] + 8, fo_mem=12, idx_mem=50, syn_mem=24)
    N, NB, S, F = (1 << p[k] for k in ("LOGN", "LOGNB", "LOGS", "LOGF"))
    for ti, t in enumerate(tiles):
        write_hex(f"{work}/t{ti}_thr_mem.hex", t["thr"], 16, N)
        write_hex(f"{work}/t{ti}_gid_mem.hex", t["gid"], 24, N)
        write_hex(f"{work}/t{ti}_fo_ptr.hex", [(a << 8) | b for a, b in t["fo_ptr"]], widths["fo_ptr"], N)
        write_hex(f"{work}/t{ti}_fo_mem.hex", t["fo"], 12, F)
        write_hex(f"{work}/t{ti}_idx_mem.hex", t["idx_table"], 50, NB)
        write_hex(f"{work}/t{ti}_syn_mem.hex", t["syn"], 24, S)
    tb = build_tb(work, chip, tiles, p, gridb, stim, ticks)
    if SIM == "iverilog":
        exe = os.path.join(work, "sim.vvp")
        subprocess.run(["iverilog", "-g2005", "-o", exe, tb] + RTL, check=True)
        subprocess.run(["vvp", "-n", exe], check=True, stdout=subprocess.DEVNULL)
    else:
        obj = os.path.join(work, "obj")
        subprocess.run(["verilator", "--binary", "--timing", "-O2", "-Wno-fatal", "-Wno-lint",
                        "-Wno-style", "--top-module", "tb", "--Mdir", obj, "-j", "2", tb] + RTL,
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run([os.path.join(obj, "Vtb")], check=True, stdout=subprocess.DEVNULL)
    rtl_fired = [set() for _ in range(ticks)]
    rtl_cycles, misses = [], None
    with open(os.path.join(work, "rtl_out.txt")) as f:
        for line in f:
            parts = line.split()
            if parts[0] == "F":
                rtl_fired[int(parts[1])].add(int(parts[2]))
            elif parts[0] == "C":
                rtl_cycles.append(int(parts[2]))
            elif parts[0] == "M":
                misses = int(parts[1])
    chip.reset()
    mismatches = []
    for t in range(ticks):
        ext = [chip.tiles[ti].nodes[li] for ti, li in stim[t]]
        ref = chip.step(ext)
        if ref != rtl_fired[t]:
            mismatches.append((t, sorted(ref - rtl_fired[t]), sorted(rtl_fired[t] - ref)))
    if not keep:
        shutil.rmtree(work, ignore_errors=True)
    return dict(mismatches=mismatches, cycles=rtl_cycles, misses=misses, params=p,
                spikes=sum(len(s) for s in rtl_fired))


# ---------------------------------------------------------------- scenarios

def adder_case(seed=0, adds=8):
    p = Program()
    ripple_adder(p, 8)
    gridb = 2
    chip = hw.from_program(p, n_tiles=16, rng=np.random.default_rng(seed))
    rng = np.random.default_rng(seed + 1)
    stim, expected = [], []
    for _ in range(adds):
        x, y = (int(v) for v in rng.integers(0, 256, 2))
        expected.append(x + y)
        for t in range(40):
            ev = []
            for i in range(8):
                for name, val in ((f"a{i}", x), (f"b{i}", y)):
                    if val >> i & 1:
                        g = p.inputs[name]
                        ev.append((int(chip.place[g]), int(chip.local[g])))
            stim.append(ev)
    res = cosim(chip, gridb, stim, len(stim))
    # also decode the sums from the RTL-equivalent outputs (model == RTL if no mismatch)
    return res, p, expected


def fuzz_case(seed, n=256, gridb=2, fanout=6, p_ext=0.04, ticks=40):
    """Random compiled program: random thresholds, signed weights in 1/16 units."""
    rng = np.random.default_rng(seed)
    thr = rng.integers(8, 40, n) / 16
    sign = np.where(rng.random(n) < 0.25, -1.0, 1.0)
    links = []
    for s in range(n):
        for d in rng.choice(n, fanout, replace=False):
            links.append((s, int(d), float(rng.integers(1, 32)) / 16))
    T = (1 << gridb) ** 2
    place = rng.integers(0, T, n)
    chip = hw.Chip(n, thr, links, sign, place, T)
    stim = []
    for _ in range(ticks):
        ev = [(int(place[g]), int(chip.local[g])) for g in np.flatnonzero(rng.random(n) < p_ext)]
        stim.append(ev)
    return cosim(chip, gridb, stim, ticks)


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    ok = True
    if what in ("adder", "all"):
        res, p, expected = adder_case()
        print(f"adder 8-bit, 4x4 mesh, params {res['params']}: {len(res['cycles'])} ticks, "
              f"{res['spikes']} spikes, mismatching ticks {len(res['mismatches'])}, "
              f"misses {res['misses']}, cycles/tick mean {np.mean(res['cycles']):.1f} "
              f"max {max(res['cycles'])}")
        ok &= not res["mismatches"] and res["misses"] == 0
    if what in ("fuzz", "all"):
        for seed in range(int(sys.argv[2]) if len(sys.argv) > 2 else 5):
            for gridb in (2, 3):
                res = fuzz_case(seed, n=128 * (1 << gridb), gridb=gridb)
                print(f"fuzz seed {seed} {1 << gridb}x{1 << gridb} mesh, {128 * (1 << gridb)} nodes: "
                      f"{res['spikes']} spikes, mismatching ticks {len(res['mismatches'])}, "
                      f"misses {res['misses']}, cycles/tick mean {np.mean(res['cycles']):.1f}")
                if res["mismatches"]:
                    print("   first mismatch:", res["mismatches"][0])
                ok &= not res["mismatches"] and res["misses"] == 0
    print("RESULT:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
