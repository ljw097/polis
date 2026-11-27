"""Co-simulate RTL v1 against sim/polis/hw1.py.

A script is a list of steps:
    ("tick", [gid, ...])     external inputs, then one tick
    ("reward", delta)        Art. 8 sweep
    ("migrate", gid, tile)   firmware moves a node (between ticks)
The same script runs on the Python model and, as a generated testbench, on
the RTL. Compared: the fired set of every tick, and at the end every table
of every tile (node, rate, index, synapse store, fan-out).

    python tools/rtl1_build.py            # all scenarios
"""
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "sim"))
from polis import hw1  # noqa: E402

RTL = [os.path.join(ROOT, "rtl", "v1", f) for f in ("polis_router.v", "polis_tile1.v", "polis_mesh1.v")]
SEL_NAMES = {0: "node_mem", 1: "gid_mem", 2: "fop_mem", 3: "fo_mem", 4: "idx_mem", 5: "syn_mem"}


def hexfile(path, values, bits, depth):
    d = (bits + 3) // 4
    with open(path, "w") as f:
        f.write("\n".join(f"{v & ((1 << bits) - 1):0{d}x}" for v in list(values) + [0] * (depth - len(values))) + "\n")


def widths(t, logf):
    return {0: 36, 1: 24, 2: logf + 8, 3: 12, 4: 50, 5: 24}


def run(chip, script, gridb, work=None):
    """Runs the model (mutating `chip`) and the RTL; returns comparison."""
    keep = work is not None
    work = work or tempfile.mkdtemp(prefix="polis_v1_")
    os.makedirs(work, exist_ok=True)
    t0 = chip.tiles[0]
    logn, lognb = t0.N.bit_length() - 1, t0.NB.bit_length() - 1
    logs, logf = t0.S.bit_length() - 1, t0.F.bit_length() - 1
    logv = logs + 1
    G = 1 << gridb
    p = chip.p
    # initial images
    for ti, t in enumerate(chip.tiles):
        for sel, arr in ((0, t.node), (1, t.gid), (2, t.fo_ptr), (3, t.fo), (4, t.idx), (5, t.syn)):
            hexfile(f"{work}/t{ti}_{sel}.hex", arr, widths(t, logf)[sel], len(arr))
    # model run + testbench body
    model_fired, body = [], []
    tick_no = 0
    for step in script:
        if step[0] == "tick":
            for g in step[1]:
                ti, li = chip.where[g]
                body.append(f"    inject({ti}, {li});")
            model_fired.append(chip.tick(step[1]))
            body.append("    tick;")
            body.append(f'    $fdisplay(fo, "C {tick_no} %0d", cycles);')
            for ti in range(chip.T):
                h = f"dut.g_y[{ti // G}].g_x[{ti % G}].u_tile"
                body.append(f"    for (k = 0; k < {h}.fl_cnt; k = k + 1)"
                            f' $fdisplay(fo, "F {tick_no} %0d", {h}.gid_mem[{h}.fl_mem[k]]);')
            tick_no += 1
        elif step[0] == "reward":
            chip.reward(step[1])
            body.append(f"    reward({step[1]});")
        elif step[0] in ("migrate", "grow", "budget"):
            fw = hw1.Firmware(chip)
            if step[0] == "migrate":
                fw.migrate(step[1], step[2])
            elif step[0] == "grow":
                fw.grow(np.random.default_rng(step[2]), step[1])
            else:
                fw.budget(step[1])
            for op in fw.ops:
                if op[0] == "wr":
                    _, ti, sel, a, v = op
                    body.append(f"    cfgw({ti}, {sel}, {a}, 50'h{v:013x});")
                else:
                    _, ti, sel, a, tj, sel2, a2, sd = op
                    body.append(f"    cfgcp({ti}, {sel}, {a}, {tj}, {sel2}, {a2}, {sd});")
    # final dumps
    for ti in range(chip.T):
        h = f"dut.g_y[{ti // G}].g_x[{ti % G}].u_tile"
        for sel in range(6):
            body.append(f'    $writememh("{work}/out_t{ti}_{sel}.hex", {h}.{SEL_NAMES[sel]});')
        body.append(f'    $writememh("{work}/out_t{ti}_6.hex", {h}.rate);')
    tb = f"""`timescale 1ns/1ps
module tb;
  reg clk = 0, rst = 1, go = 0, rew_go = 0, ext_valid = 0, cfg_we = 0, cfg_re = 0;
  reg signed [7:0] rew_delta = 0;
  reg [{2 * gridb - 1}:0] ext_tile = 0, cfg_tile = 0;
  reg [{logn - 1}:0] ext_local = 0;
  reg [2:0] cfg_sel = 0; reg [15:0] cfg_addr = 0; reg [49:0] cfg_wdata = 0, rd, old, word;
  wire done; wire [31:0] cycles, misses; wire [49:0] cfg_rdata;
  integer fo, k;
  always #1 clk = ~clk;
  polis_mesh1 #(.GRIDB({gridb}), .LOGN({logn}), .LOGNB({lognb}), .LOGS({logs}), .LOGF({logf}), .LOGV({logv})) dut (
    .clk(clk), .rst(rst), .go(go), .rew_go(rew_go), .rew_delta(rew_delta), .done(done), .cycles(cycles), .misses(misses),
    .p_leak(8'd{p.leak}), .p_refr(4'd{p.refr}), .p_homeo(1'b{p.homeo}), .p_target(8'd{p.target}),
    .p_rs(4'd{p.rs}), .p_hs(4'd{p.hs}), .p_tmin(16'sd{p.tmin}), .p_tmax(16'sd{p.tmax}), .p_learn(1'b{p.learn}),
    .p_lp(4'd{p.lp}), .p_dec(12'd{p.dec}), .p_prune(12'd{p.prune}), .p_il(12'd{p.il}), .p_rsh(4'd{p.rsh}),
    .cfg_we(cfg_we), .cfg_re(cfg_re), .cfg_tile(cfg_tile), .cfg_sel(cfg_sel), .cfg_addr(cfg_addr),
    .cfg_wdata(cfg_wdata), .cfg_rdata(cfg_rdata),
    .ext_valid(ext_valid), .ext_tile(ext_tile), .ext_local(ext_local));
  task inject(input integer t, input integer l); begin
    @(negedge clk); ext_valid = 1; ext_tile = t; ext_local = l;
    @(negedge clk); ext_valid = 0; @(negedge clk); end endtask
  task tick; begin
    @(negedge clk); go = 1; @(negedge clk); go = 0;
    while (!done) @(posedge clk);
    @(negedge clk); end endtask
  task reward(input integer dl); begin
    @(negedge clk); rew_delta = dl; rew_go = 1; @(negedge clk); rew_go = 0;
    while (!done) @(posedge clk);
    @(negedge clk); end endtask
  task cfgw(input integer t, input integer s, input integer a, input [49:0] d); begin
    @(negedge clk); cfg_we = 1; cfg_tile = t; cfg_sel = s; cfg_addr = a; cfg_wdata = d;
    @(negedge clk); cfg_we = 0; end endtask
  task cfgr(input integer t, input integer s, input integer a); begin
    @(negedge clk); cfg_re = 1; cfg_tile = t; cfg_sel = s; cfg_addr = a;
    @(negedge clk); cfg_re = 0; @(negedge clk); @(negedge clk); rd = cfg_rdata; end endtask
  // firmware copy: synapse words move between index (4) and store (5)
  task cfgcp(input integer t, input integer s, input integer a, input integer t2,
             input integer s2, input integer a2, input integer setdst); begin
    cfgr(t, s, a);
    if (s == 4 || s == 5 || s2 == 4 || s2 == 5) begin
      word = {{26'd0, rd[23:0]}};
      if (setdst >= 0) word[11:4] = setdst;
      if (s2 == 4) begin cfgr(t2, 4, a2); word = {{old[49:24], word[23:0]}}; word[49:24] = rd[49:24]; end
      cfgw(t2, s2, a2, word);
    end else cfgw(t2, s2, a2, rd);
  end endtask
  initial begin
"""
    for ti in range(chip.T):
        h = f"dut.g_y[{ti // G}].g_x[{ti % G}].u_tile"
        for sel in range(6):
            tb += f'    $readmemh("{work}/t{ti}_{sel}.hex", {h}.{SEL_NAMES[sel]});\n'
    tb += f'    fo = $fopen("{work}/rtl_out.txt", "w");\n'
    tb += "    repeat (4) @(negedge clk); rst = 0; repeat (2) @(negedge clk);\n"
    tb += "\n".join(body) + '\n    $fdisplay(fo, "M %0d", misses);\n    $fclose(fo); $finish;\n  end\nendmodule\n'
    with open(f"{work}/tb.v", "w") as f:
        f.write(tb)
    obj = f"{work}/obj"
    r = subprocess.run(["verilator", "--binary", "--timing", "-O2", "-Wno-fatal", "-Wno-lint", "-Wno-style",
                        "--top-module", "tb", "--Mdir", obj, "-j", "2", f"{work}/tb.v"] + RTL,
                       capture_output=True, text=True)
    if r.returncode:
        print(r.stderr[-3000:])
        raise SystemExit("verilator failed")
    subprocess.run([f"{obj}/Vtb"], check=True, stdout=subprocess.DEVNULL)
    rtl_fired = [set() for _ in model_fired]
    cycles, misses = [], None
    for line in open(f"{work}/rtl_out.txt"):
        a = line.split()
        if a[0] == "F":
            rtl_fired[int(a[1])].add(int(a[2]))
        elif a[0] == "C":
            cycles.append(int(a[2]))
        else:
            misses = int(a[1])
    tick_mism = [t for t in range(len(model_fired)) if model_fired[t] != rtl_fired[t]]
    table_mism = []
    for ti, t in enumerate(chip.tiles):
        for sel, arr in ((0, t.node), (2, t.fo_ptr), (3, t.fo), (4, t.idx), (5, t.syn), (6, t.rate)):
            got = [int(x, 16) for x in open(f"{work}/out_t{ti}_{sel}.hex").read().split()
                   if not x.startswith("//")]
            mask = (1 << (8 if sel == 6 else widths(t, logf)[sel])) - 1
            for a, (m, r_) in enumerate(zip(arr, got)):
                if (m & mask) != r_:
                    table_mism.append((ti, sel, a, m & mask, r_))
    if not keep:
        shutil.rmtree(work, ignore_errors=True)
    return dict(tick_mism=tick_mism, table_mism=table_mism, cycles=cycles, misses=misses,
                spikes=sum(len(s) for s in model_fired), fired=model_fired)


def random_net(seed, n, gridb, params, fanout=6, p_plastic=0.7, p_inhib=0.2):
    rng = np.random.default_rng(seed)
    T = (1 << gridb) ** 2
    thr = rng.integers(10, 30, n) / 16
    inhib = rng.random(n) < p_inhib
    links, plastic = [], set()
    for s in range(n):
        for d in rng.choice(n, fanout, replace=False):
            if d == s:
                continue
            w = rng.integers(40, 200) / 256 * (-2 if inhib[s] else 1)
            if rng.random() < p_plastic:
                plastic.add(len(links))
            links.append((s, int(d), w))
    place = rng.integers(0, T, n)
    return hw1.Chip1.build(n, thr, links, place, T, params=params, plastic=plastic), rng


def scenario(seed, n=96, gridb=2, ticks=60, migrate=True, learn=True):
    params = hw1.Params(leak=180, refr=2, homeo=1, target=20, rs=4, hs=5, tmin=6, tmax=60,
                        learn=int(learn), lp=3, dec=2, prune=6, il=4, rsh=2)
    chip, rng = random_net(seed, n, gridb, params)
    T = chip.T
    script = []
    for t in range(ticks):
        ext = [int(g) for g in np.flatnonzero(rng.random(n) < 0.08)]
        script.append(("tick", ext))
        if learn and t % 10 == 9:
            script.append(("reward", int(rng.integers(-60, 60))))
        if learn and t % 10 == 4:
            script.append(("grow", 3, seed * 1000 + t))
        if learn and t % 20 == 13:
            script.append(("budget", 600))
        if migrate and t % 15 == 7:
            for g in rng.choice(n, 3, replace=False):
                script.append(("migrate", int(g), int(rng.integers(T))))
    return chip, script


def main():
    ok = True
    for seed in range(int(sys.argv[1]) if len(sys.argv) > 1 else 3):
        for gridb in (2, 3):
            n = 96 if gridb == 2 else 320
            chip, script = scenario(seed, n=n, gridb=gridb)
            # transparency: the same script without migration on a fresh chip
            chip_ref, script_ref = scenario(seed, n=n, gridb=gridb)
            script_nomig = [s for s in script_ref if s[0] != "migrate"]
            ref_fired = []
            for s in script_nomig:
                if s[0] == "tick":
                    ref_fired.append(chip_ref.tick(s[1]))
                elif s[0] == "reward":
                    chip_ref.reward(s[1])
                elif s[0] == "grow":
                    hw1.Firmware(chip_ref).grow(np.random.default_rng(s[2]), s[1])
                else:
                    hw1.Firmware(chip_ref).budget(s[1])
            res = run(chip, script, gridb)
            transparent = ref_fired == res["fired"]
            n_mig = sum(1 for s in script if s[0] == "migrate")
            print(f"seed {seed} {1 << gridb}x{1 << gridb}, {n} nodes, {n_mig} migrations, "
                  f"{sum(1 for s in script if s[0] == 'reward')} rewards, "
                  f"{sum(1 for s in script if s[0] == 'grow')} growth, "
                  f"{sum(1 for s in script if s[0] == 'budget')} budget sweeps: "
                  f"{res['spikes']} spikes | tick mismatches {len(res['tick_mism'])} | "
                  f"table word mismatches {len(res['table_mism'])} | misses {res['misses']} | "
                  f"migration transparent {transparent} | cycles/tick {np.mean(res['cycles']):.0f}")
            if res["table_mism"]:
                print("   first table mismatch (tile, sel, addr, model, rtl):", res["table_mism"][:3])
            ok &= not res["tick_mism"] and not res["table_mism"] and res["misses"] == 0 and transparent
    print("RESULT:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
