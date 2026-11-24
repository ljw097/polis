"""End-to-end gateway test on RTL: NEP bytes in, NEP bytes out.

The request bytes are produced by sim/polis/gateway.py (exactly what a
socket client sends over UDP); the SoC RTL (rtl/v1/polis_soc1.v: NEP bridge +
mesh) executes the compiled 8-bit adder; the reply bytes are decoded by
gateway.py again. Also sends a malformed request and expects ERROR.

    python tools/nep_soc_test.py
"""
import os
import subprocess
import sys
import tempfile

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "experiments"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from e05_network import request  # noqa: E402
from polis import hw1  # noqa: E402
from polis.gateway import decode_result  # noqa: E402
from polis.program import Program, ripple_adder  # noqa: E402
from rtl1_build import SEL_NAMES, hexfile, widths  # noqa: E402

V1 = [os.path.join(ROOT, "rtl", "v1", f) for f in
      ("polis_router.v", "polis_tile1.v", "polis_mesh1.v", "polis_nep.v", "polis_soc1.v")]


def main(n_adds=6, seed=0, debug=False):
    p = Program()
    ripple_adder(p, 8)
    n, gridb = len(p.thr), 2
    T = (1 << gridb) ** 2
    links = [(s, d, w * p.sign[s]) for s, d, w in p.links]
    place = np.random.default_rng(seed).integers(0, T, n)
    chip = hw1.Chip1.build(n, p.thr, links, place, T, params=hw1.Params())
    t0 = chip.tiles[0]
    logn, lognb = t0.N.bit_length() - 1, t0.NB.bit_length() - 1
    logs, logf = t0.S.bit_length() - 1, t0.F.bit_length() - 1
    ports = list(p.inputs.values()) + list(p.outputs.values())
    nin, nout = len(p.inputs), len(p.outputs)
    work = tempfile.mkdtemp(prefix="polis_nep_")
    for ti, t in enumerate(chip.tiles):
        for sel, arr in ((0, t.node), (1, t.gid), (2, t.fo_ptr), (3, t.fo), (4, t.idx), (5, t.syn)):
            hexfile(f"{work}/t{ti}_{sel}.hex", arr, widths(t, logf)[sel], len(arr))
    rng = np.random.default_rng(seed + 1)
    cases = [tuple(int(v) for v in rng.integers(0, 256, 2)) for _ in range(n_adds)]
    msgs = [request(x, y) for x, y in cases] + [b"NM\x01\x01\x00\x05\x00\x01\x00\x63\x00\x00\x00\x02",  # port 99
                                                  b"hello"]
    G = 1 << gridb
    body = []
    for i, (g) in enumerate(ports):
        ti, li = chip.where[g]
        body.append(f"    @(negedge clk); pm_we = 1; pm_addr = {i}; pm_data = {(ti << logn) | li}; "
                    f"@(negedge clk); pm_we = 0;")
    for m in msgs:
        for j, b in enumerate(m):
            body.append(f"    send({b}, {int(j == len(m) - 1)});")
        body.append("    recv;")
    load = ""
    for ti in range(T):
        h = f"dut.mesh.g_y[{ti // G}].g_x[{ti % G}].u_tile"
        for sel in range(6):
            load += f'    $readmemh("{work}/t{ti}_{sel}.hex", {h}.{SEL_NAMES[sel]});\n'
    tb = f"""`timescale 1ns/1ps
module tb;
  reg clk = 0, rst = 1, pm_we = 0, rx_valid = 0, rx_last = 0;
  reg [7:0] pm_addr = 0, rx_data = 0; reg [{2 * gridb + logn - 1}:0] pm_data = 0;
  wire rx_ready, tx_valid, tx_last; wire [7:0] tx_data; wire [31:0] cycles, misses; wire [49:0] crd;
  integer fo, done_msg, guard = 0;
  always #1 clk = ~clk;
  polis_soc1 #(.GRIDB({gridb}), .LOGN({logn}), .LOGNB({lognb}), .LOGS({logs}), .LOGF({logf}), .LOGV({logs + 1}),
               .NIN({nin}), .NOUT({nout})) dut (
    .clk(clk), .rst(rst), .p_leak(8'd0), .p_refr(4'd0), .p_homeo(1'b0), .p_target(8'd0), .p_rs(4'd4), .p_hs(4'd4),
    .p_tmin(16'sd0), .p_tmax(16'sd0), .p_learn(1'b0), .p_lp(4'd0), .p_dec(12'd0), .p_prune(12'd0), .p_il(12'd0),
    .p_rsh(4'd0), .cfg_we(1'b0), .cfg_re(1'b0), .cfg_tile(0), .cfg_sel(3'd0), .cfg_addr(16'd0), .cfg_wdata(50'd0),
    .cfg_rdata(crd), .pm_we(pm_we), .pm_addr(pm_addr), .pm_data(pm_data),
    .rx_valid(rx_valid), .rx_data(rx_data), .rx_last(rx_last), .rx_ready(rx_ready),
    .tx_valid(tx_valid), .tx_data(tx_data), .tx_last(tx_last), .tx_ready(1'b1),
    .cycles(cycles), .misses(misses));
  task send(input [7:0] b, input l); begin
    @(negedge clk); rx_valid = 1; rx_data = b; rx_last = l;
    @(posedge clk); while (!rx_ready) @(posedge clk);
    @(negedge clk); rx_valid = 0; rx_last = 0; end endtask
  task recv; begin
    done_msg = 0; guard = 0;
    while (!done_msg) begin
      @(posedge clk); guard = guard + 1;
      if (guard > 200000) begin $display("TIMEOUT st=%0d t=%0d ei=%0d nev=%0d ticks=%0d", dut.nep.st, dut.nep.t, dut.nep.ei, dut.nep.nev, dut.nep.ticks); $finish; end
      if (tx_valid) begin $fwrite(fo, "%02x", tx_data); if (tx_last) begin $fwrite(fo, "\\n"); done_msg = 1; end end
    end end endtask
  initial begin
{load}    fo = $fopen("{work}/tx.txt", "w");
    repeat (4) @(negedge clk); rst = 0; repeat (2) @(negedge clk);
{chr(10).join(body)}
    $fclose(fo); $finish;
  end
endmodule
"""
    open(f"{work}/tb.v", "w").write(tb)
    r = subprocess.run(["verilator", "--binary", "--timing", "-O2", "-Wno-fatal", "-Wno-lint", "-Wno-style",
                        "--top-module", "tb", "--Mdir", f"{work}/obj", "-j", "2", f"{work}/tb.v"] + V1,
                       capture_output=True, text=True)
    if r.returncode:
        print(r.stderr[-3000:])
        raise SystemExit(1)
    subprocess.run([f"{work}/obj/Vtb"], check=True, stdout=subprocess.DEVNULL)
    replies = [bytes.fromhex(l.strip()) for l in open(f"{work}/tx.txt") if l.strip()]
    ok = len(replies) == len(msgs)
    for (x, y), rep in zip(cases, replies):
        outs = decode_result(rep)
        s = sum(1 << k for k, (_, _, last) in enumerate(outs) if last == 39)
        print(f"RTL gateway: {x} + {y} = {s}   ({len(rep)}-byte reply)")
        ok &= s == x + y
    for rep in replies[len(cases):]:
        try:
            decode_result(rep)
            print("expected an ERROR reply, got", rep)
            ok = False
        except RuntimeError as e:
            print(f"malformed request -> ERROR '{e}'")
    print("RESULT:", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
