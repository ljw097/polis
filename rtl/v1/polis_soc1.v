// Polis SoC v1: mesh + NEP gateway bridge. The host / firmware reaches the
// tables through the config port; requests arrive as NEP byte streams.
`default_nettype none
module polis_soc1 #(
    parameter GRIDB = 2, LOGN = 4, LOGNB = 4, LOGS = 4, LOGF = 4, LOGV = 6,
    parameter NIN = 16, NOUT = 9
) (
    input  wire clk, input wire rst,
    input  wire [7:0]  p_leak, input wire [3:0] p_refr, input wire p_homeo, input wire [7:0] p_target,
    input  wire [3:0]  p_rs, input wire [3:0] p_hs, input wire signed [15:0] p_tmin, input wire signed [15:0] p_tmax,
    input  wire p_learn, input wire [3:0] p_lp, input wire [11:0] p_dec, input wire [11:0] p_prune,
    input  wire [11:0] p_il, input wire [3:0] p_rsh,
    input  wire cfg_we, input wire cfg_re, input wire [2*GRIDB-1:0] cfg_tile, input wire [2:0] cfg_sel,
    input  wire [15:0] cfg_addr, input wire [49:0] cfg_wdata, output wire [49:0] cfg_rdata,
    input  wire pm_we, input wire [7:0] pm_addr, input wire [2*GRIDB+LOGN-1:0] pm_data,
    input  wire rx_valid, input wire [7:0] rx_data, input wire rx_last, output wire rx_ready,
    output wire tx_valid, output wire [7:0] tx_data, output wire tx_last, input wire tx_ready,
    output wire [31:0] cycles, output wire [31:0] misses
);
    wire go, done, ext_valid, obs_fired;
    wire [2*GRIDB-1:0] ext_tile, obs_tile;
    wire [LOGN-1:0] ext_local, obs_local;
    polis_mesh1 #(.GRIDB(GRIDB), .LOGN(LOGN), .LOGNB(LOGNB), .LOGS(LOGS), .LOGF(LOGF), .LOGV(LOGV)) mesh (
        .clk(clk), .rst(rst), .go(go), .rew_go(1'b0), .rew_delta(8'sd0), .done(done),
        .cycles(cycles), .misses(misses),
        .p_leak(p_leak), .p_refr(p_refr), .p_homeo(p_homeo), .p_target(p_target), .p_rs(p_rs), .p_hs(p_hs),
        .p_tmin(p_tmin), .p_tmax(p_tmax), .p_learn(p_learn), .p_lp(p_lp), .p_dec(p_dec), .p_prune(p_prune),
        .p_il(p_il), .p_rsh(p_rsh),
        .obs_tile(obs_tile), .obs_local(obs_local), .obs_fired(obs_fired),
        .cfg_re(cfg_re), .cfg_rdata(cfg_rdata), .cfg_we(cfg_we), .cfg_tile(cfg_tile), .cfg_sel(cfg_sel),
        .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .ext_valid(ext_valid), .ext_tile(ext_tile), .ext_local(ext_local));
    polis_nep #(.TB(2*GRIDB), .LOGN(LOGN), .NIN(NIN), .NOUT(NOUT)) nep (
        .clk(clk), .rst(rst), .pm_we(pm_we), .pm_addr(pm_addr), .pm_data(pm_data),
        .rx_valid(rx_valid), .rx_data(rx_data), .rx_last(rx_last), .rx_ready(rx_ready),
        .tx_valid(tx_valid), .tx_data(tx_data), .tx_last(tx_last), .tx_ready(tx_ready),
        .go(go), .done(done), .ext_valid(ext_valid), .ext_tile(ext_tile), .ext_local(ext_local),
        .obs_tile(obs_tile), .obs_local(obs_local), .obs_fired(obs_fired));
endmodule
`default_nettype wire
