// Polis tile, RTL v1. Reference model: sim/polis/hw1.py (bit exact).
//
// Large tables are synchronous SRAMs (1R1W, read data one cycle after the
// address): node, gid, fo_ptr, fo, idx, syn. Small per-node arrays are
// register files: acc, rate, touched, firedv, touch list, fired list,
// visit list.
//
// Word formats (see hw1.py):
//   synapse  24b : w12 [23:12] signed 1/256 | dst [11:4] | plastic [3] | credit [2:0]
//   node     36b : thr [35:20] | refr [19:16] | v [15:0]   (1/16)
//   index    50b : valid [49] | key [48:25] | inline [24] | payload [23:0]
// Phases: COMM (send || receive), UPDATE (Art. 1, 6), LEARN (Art. 2, 3, 7b,
// credit), REWARD sweep (Art. 8). Config port: read/write every table
// between ticks (loader, firmware: migration, new links).
`default_nettype none

module polis_tile1 #(
    parameter LOGN  = 4,
    parameter LOGNB = 4,
    parameter LOGS  = 4,
    parameter LOGF  = 4,
    parameter LOGV  = 6
) (
    input  wire        clk,
    input  wire        rst,
    // phase control
    input  wire        tick_start,
    input  wire        upd_start,
    input  wire        rew_start,
    input  wire signed [7:0] rew_delta,
    output reg         send_done,
    output reg         dp_done,       // update (+ learn) or reward finished
    output reg  [31:0] sent_cnt,
    output reg  [31:0] cons_cnt,
    output reg  [31:0] miss_cnt,
    // constitution parameters
    input  wire [7:0]  p_leak,
    input  wire [3:0]  p_refr,
    input  wire        p_homeo,
    input  wire [7:0]  p_target,
    input  wire [3:0]  p_rs,
    input  wire [3:0]  p_hs,
    input  wire signed [15:0] p_tmin,
    input  wire signed [15:0] p_tmax,
    input  wire        p_learn,
    input  wire [3:0]  p_lp,
    input  wire [11:0] p_dec,
    input  wire [11:0] p_prune,
    input  wire [11:0] p_il,
    input  wire [3:0]  p_rsh,
    // config port: sel 0 node, 1 gid, 2 fo_ptr, 3 fo, 4 idx, 5 syn, 6 rate
    input  wire        cfg_we,
    input  wire        cfg_re,
    input  wire [2:0]  cfg_sel,
    input  wire [15:0] cfg_addr,
    input  wire [49:0] cfg_wdata,
    output reg  [49:0] cfg_rdata,
    // host input between ticks
    input  wire            ext_valid,
    input  wire [LOGN-1:0] ext_local,
    // router local port
    output reg         out_valid,
    output reg  [39:0] out_data,
    input  wire        out_ready,
    input  wire        in_valid,
    input  wire [39:0] in_data,
    output wire        in_ready,
    output wire [(1<<LOGN)-1:0] firedv_o
);
    localparam N = 1 << LOGN, NB = 1 << LOGNB, S = 1 << LOGS, F = 1 << LOGF, V = 1 << LOGV;

    // ------------------------------------------------------------ storage
    reg [35:0]     node_mem [0:N-1];
    reg [23:0]     gid_mem  [0:N-1];
    reg [LOGF+7:0] fop_mem  [0:N-1];
    reg [11:0]     fo_mem   [0:F-1];
    reg [49:0]     idx_mem  [0:NB-1];
    reg [23:0]     syn_mem  [0:S-1];

    reg signed [23:0] acc   [0:N-1];
    reg [7:0]         rate  [0:N-1];
    reg [N-1:0]       touched, firedv;
    assign firedv_o = firedv;
    reg [LOGN-1:0]    tl_mem [0:N-1];
    reg [LOGN:0]      tl_cnt;
    reg [LOGN-1:0]    fl_mem [0:N-1];
    reg [LOGN:0]      fl_cnt;
    reg [16:0]        vis_mem [0:V-1];      // {kind (1 = index slot), addr16}
    reg [LOGV:0]      vis_cnt;

`ifndef SYNTHESIS
    integer z;
    initial for (z = 0; z < N; z = z + 1) begin acc[z] = 0; rate[z] = 0; end
`endif

    function [LOGNB-1:0] fold;
        input [23:0] s;
        integer i;
        begin
            fold = {LOGNB{1'b0}};
            for (i = 0; i < 24; i = i + LOGNB) fold = fold ^ (s >> i);
        end
    endfunction

    // ------------------------------------------------ SRAM ports (1R1W)
    // read addresses are driven combinationally by the FSMs; data is
    // registered (available in the following cycle)
    reg [LOGN-1:0]  node_ra, gid_ra, fop_ra;
    reg [LOGF-1:0]  fo_ra;
    reg [LOGNB-1:0] idx_ra;
    reg [LOGS-1:0]  syn_ra;
    reg [35:0] node_rd; reg [23:0] gid_rd; reg [LOGF+7:0] fop_rd; reg [11:0] fo_rd;
    reg [49:0] idx_rd;  reg [23:0] syn_rd;

    reg node_we, idx_we, syn_we;
    reg [LOGN-1:0]  node_wa; reg [35:0] node_wd;
    reg [LOGNB-1:0] idx_wa;  reg [49:0] idx_wd;
    reg [LOGS-1:0]  syn_wa;  reg [23:0] syn_wd;

    always @(posedge clk) begin
        node_rd <= node_mem[node_ra];
        gid_rd  <= gid_mem[gid_ra];
        fop_rd  <= fop_mem[fop_ra];
        fo_rd   <= fo_mem[fo_ra];
        idx_rd  <= idx_mem[idx_ra];
        syn_rd  <= syn_mem[syn_ra];
        if (node_we) node_mem[node_wa] <= node_wd;
        if (idx_we)  idx_mem[idx_wa]   <= idx_wd;
        if (syn_we)  syn_mem[syn_wa]   <= syn_wd;
        if (cfg_we && cfg_sel == 3'd1) gid_mem[cfg_addr[LOGN-1:0]] <= cfg_wdata[23:0];
        if (cfg_we && cfg_sel == 3'd2) fop_mem[cfg_addr[LOGN-1:0]] <= cfg_wdata[LOGF+7:0];
        if (cfg_we && cfg_sel == 3'd3) fo_mem[cfg_addr[LOGF-1:0]]  <= cfg_wdata[11:0];
    end

    // ------------------------------------------------------------- SEND
    localparam S_IDLE = 3'd0, S_LOAD = 3'd1, S_HDR = 3'd2, S_EMIT = 3'd3, S_OUT = 3'd4;
    reg [2:0]      s_state;
    reg [LOGN:0]   s_i;
    reg [LOGF-1:0] s_ptr;
    reg [7:0]      s_left;
    reg [23:0]     s_gid;
    wire [LOGN-1:0] s_node = fl_mem[s_i[LOGN-1:0]];

    always @(posedge clk) begin
        if (rst) begin
            s_state <= S_IDLE; send_done <= 1'b1; out_valid <= 1'b0; sent_cnt <= 0;
        end else begin
            if (out_valid && out_ready) out_valid <= 1'b0;
            case (s_state)
                S_IDLE: if (tick_start) begin
                    s_i <= 0; send_done <= 1'b0; sent_cnt <= 0; s_state <= S_LOAD;
                end
                S_LOAD: begin                       // issue fo_ptr[node], gid[node]
                    if (s_i == fl_cnt) begin send_done <= 1'b1; s_state <= S_IDLE; end
                    else s_state <= S_HDR;
                end
                S_HDR: begin                        // read data arrives
                    s_ptr <= fop_rd[LOGF+7:8]; s_left <= fop_rd[7:0]; s_gid <= gid_rd;
                    if (fop_rd[7:0] == 8'd0) begin s_i <= s_i + 1'b1; s_state <= S_LOAD; end
                    else s_state <= S_EMIT;
                end
                S_EMIT: s_state <= S_OUT;           // issue fo[ptr]
                S_OUT: if (!out_valid || out_ready) begin
                    out_valid <= 1'b1;
                    out_data  <= {4'd1, fo_rd, s_gid};
                    sent_cnt  <= sent_cnt + 1;
                    s_ptr     <= s_ptr + 1'b1;
                    s_left    <= s_left - 1'b1;
                    if (s_left == 8'd1) begin s_i <= s_i + 1'b1; s_state <= S_LOAD; end
                    else s_state <= S_EMIT;
                end
                default: s_state <= S_IDLE;
            endcase
        end
    end

    // ----------------------------------------------------- DATA PLANE FSM
    localparam D_IDLE = 4'd0, D_EXT = 4'd1, D_PI = 4'd2, D_PC = 4'd3, D_SEG = 4'd4,
               D_UPD = 4'd5, D_LRN = 4'd6, D_LRN2 = 4'd7, D_REW = 4'd8, D_REW2 = 4'd9,
               D_CFG = 4'd10;
    reg [3:0]       d_state;
    reg [23:0]      r_src;
    reg [LOGNB-1:0] r_h;
    reg [15:0]      r_ptr;
    reg [7:0]       r_iss, r_proc, r_len;
    reg             r_pv;                   // a segment read is in flight
    reg [LOGS-1:0]  r_paddr;                // address of the word in flight
    reg [LOGN:0]    u_i;                    // update: index issued
    reg             u_pv;
    reg [LOGN-1:0]  u_node;                 // node whose word is in flight
    reg [LOGN-1:0]  x_node;
    reg [LOGV:0]    l_i;
    reg [16:0]      l_ent;
    reg [16:0]      w_a;                    // reward sweep address (idx then syn)
    reg [16:0]      w_cur;
    reg signed [7:0] w_delta;
    reg [2:0]       c_sel;

    wire full_mode = (p_leak != 8'd0) || (p_refr != 4'd0) || p_homeo;
    assign in_ready = (d_state == D_IDLE) && !ext_valid && !upd_start && !rew_start
                      && !cfg_we && !cfg_re;

    // ---- learning rule on one synapse word (Art. 2, 3, 7b + credit)
    function [23:0] learn_word;
        input [23:0] word;
        input        post_fired;
        input [7:0]  post_rate;
        reg signed [12:0] w;
        reg [7:0] d; reg pl; reg [2:0] cr;
        begin
            w = {word[23], word[23:12]}; d = word[11:4]; pl = word[3]; cr = word[2:0];
            if (pl) begin
                if (w < 0) begin
                    if (post_rate > p_target)      w = (w - $signed({1'b0, p_il}) < -13'sd2048) ? -13'sd2048 : w - $signed({1'b0, p_il});
                    else if (post_rate < p_target) w = (w + $signed({1'b0, p_il}) > 0) ? 13'sd0 : w + $signed({1'b0, p_il});
                end else if (post_fired) begin
                    w = w + ((13'sd2047 - w) >>> p_lp);
                    cr = (cr == 3'd7) ? 3'd7 : cr + 3'd1;
                end else begin
                    w = w - $signed({1'b0, p_dec});
                    if (w <= $signed({1'b0, p_prune})) begin w = 0; pl = 1'b0; cr = 3'd0; end
                end
            end
            learn_word = {w[11:0], d, pl, cr};
        end
    endfunction

    // ---- reward on one synapse word (Art. 8)
    function [23:0] rew_word;
        input [23:0] word;
        input signed [7:0] delta;
        reg signed [13:0] w, dw;
        begin
            w = {{2{word[23]}}, word[23:12]};
            if (!word[3] || word[2:0] == 3'd0) rew_word = word;
            else begin
                dw = (delta * $signed({1'b0, word[2:0]})) >>> p_rsh;
                if (w >= 0) begin
                    w = w + dw;
                    if (w < 0) w = 0; else if (w > 14'sd2047) w = 14'sd2047;
                end else begin
                    w = w + dw;
                    if (w > 0) w = 0; else if (w < -14'sd2048) w = -14'sd2048;
                end
                rew_word = {w[11:0], word[11:4], word[3], 3'd0};
            end
        end
    endfunction

    // ---- update of one node (Art. 1, 6); combinational on node_rd
    wire signed [15:0] n_thr  = node_rd[35:20];
    wire [3:0]         n_refr = node_rd[19:16];
    wire signed [15:0] n_v    = node_rd[15:0];
    wire signed [23:0] n_acc  = acc[u_node];
    wire signed [31:0] n_leak = (n_v * $signed({1'b0, p_leak})) >>> 8;
    wire signed [31:0] n_sum  = n_leak + n_acc;
    wire signed [15:0] n_v1   = (n_sum > 32'sd32767) ? 16'sd32767 :
                                (n_sum < -32'sd32768) ? -16'sd32768 : n_sum[15:0];
    wire signed [15:0] n_v2   = (n_refr != 0) ? 16'sd0 : n_v1;
    wire [3:0]         n_r2   = (n_refr != 0) ? n_refr - 4'd1 : 4'd0;
    wire               n_fire = (n_v2 >= n_thr);
    wire [7:0]         n_rate0 = rate[u_node];
    wire [8:0]         n_rate1 = n_rate0 - (n_rate0 >> p_rs) + (n_fire ? (9'd256 >> p_rs) : 9'd0);
    wire [7:0]         n_rate  = (n_rate1 > 9'd255) ? 8'd255 : n_rate1[7:0];
    wire signed [8:0]  n_rdiff = $signed({1'b0, n_rate}) - $signed({1'b0, p_target});
    wire signed [16:0] n_thr1  = n_thr + (n_rdiff >>> p_hs);
    wire signed [15:0] n_thr2  = (n_thr1 < p_tmin) ? p_tmin : (n_thr1 > p_tmax) ? p_tmax : n_thr1[15:0];
    wire [35:0] n_word = {p_homeo ? n_thr2 : n_thr,
                          n_fire ? p_refr : n_r2,
                          n_fire ? 16'sd0 : n_v2};

    // ---- read-address / write-port steering
    wire [LOGN-1:0] upd_target = full_mode ? u_i[LOGN-1:0] : tl_mem[u_i[LOGN-1:0]];
    always @* begin
        node_ra = upd_target; gid_ra = s_node; fop_ra = s_node; fo_ra = s_ptr;
        idx_ra = r_h; syn_ra = r_ptr[LOGS-1:0];
        node_we = 1'b0; node_wa = 0; node_wd = 0;
        idx_we = 1'b0;  idx_wa = 0;  idx_wd = 0;
        syn_we = 1'b0;  syn_wa = 0;  syn_wd = 0;
        case (d_state)
            D_IDLE: begin
                if (ext_valid) node_ra = ext_local;
                if (cfg_re) begin
                    node_ra = cfg_addr[LOGN-1:0]; gid_ra = cfg_addr[LOGN-1:0];
                    fop_ra = cfg_addr[LOGN-1:0];  fo_ra = cfg_addr[LOGF-1:0];
                    idx_ra = cfg_addr[LOGNB-1:0]; syn_ra = cfg_addr[LOGS-1:0];
                end
                if (cfg_we && cfg_sel == 3'd0) begin node_we = 1'b1; node_wa = cfg_addr[LOGN-1:0]; node_wd = cfg_wdata[35:0]; end
                if (cfg_we && cfg_sel == 3'd4) begin idx_we = 1'b1; idx_wa = cfg_addr[LOGNB-1:0]; idx_wd = cfg_wdata; end
                if (cfg_we && cfg_sel == 3'd5) begin syn_we = 1'b1; syn_wa = cfg_addr[LOGS-1:0]; syn_wd = cfg_wdata[23:0]; end
            end
            D_UPD: if (u_pv) begin node_we = 1'b1; node_wa = u_node; node_wd = n_word; end
            D_LRN: begin
                if (vis_mem[l_i[LOGV-1:0]][16]) idx_ra = vis_mem[l_i[LOGV-1:0]][LOGNB-1:0];
                else                            syn_ra = vis_mem[l_i[LOGV-1:0]][LOGS-1:0];
            end
            D_LRN2: begin
                if (l_ent[16]) begin
                    idx_we = 1'b1; idx_wa = l_ent[LOGNB-1:0];
                    idx_wd = {idx_rd[49:24], learn_word(idx_rd[23:0], firedv[idx_rd[11:4]], rate[idx_rd[11:4]])};
                end else begin
                    syn_we = 1'b1; syn_wa = l_ent[LOGS-1:0];
                    syn_wd = learn_word(syn_rd, firedv[syn_rd[11:4]], rate[syn_rd[11:4]]);
                end
            end
            D_REW: begin
                if (w_a < NB) idx_ra = w_a[LOGNB-1:0];
                else          syn_ra = w_a - NB;
            end
            D_REW2: begin
                if (w_cur < NB) begin
                    if (idx_rd[49] && idx_rd[24]) begin
                        idx_we = 1'b1; idx_wa = w_cur[LOGNB-1:0];
                        idx_wd = {idx_rd[49:24], rew_word(idx_rd[23:0], w_delta)};
                    end
                end else begin
                    syn_we = 1'b1; syn_wa = w_cur - NB; syn_wd = rew_word(syn_rd, w_delta);
                end
            end
            default: ;
        endcase
    end

    // ---- accumulate one synapse word (COMM)
    task accumulate;
        input [23:0] word;
        input        kind;
        input [15:0] addr;
        reg [LOGN-1:0] d;
        begin
            d = word[4 +: LOGN];
            acc[d] <= acc[d] + {{16{word[23]}}, word[23:16]};
            if (!touched[d]) begin
                touched[d] <= 1'b1;
                tl_mem[tl_cnt[LOGN-1:0]] <= d;
                tl_cnt <= tl_cnt + 1'b1;
            end
            if (word[3]) begin
                vis_mem[vis_cnt[LOGV-1:0]] <= {kind, addr};
                vis_cnt <= vis_cnt + 1'b1;
            end
        end
    endtask

    always @(posedge clk) begin
        if (rst) begin
            d_state <= D_IDLE; touched <= 0; firedv <= 0; tl_cnt <= 0; fl_cnt <= 0;
            vis_cnt <= 0; cons_cnt <= 0; miss_cnt <= 0; dp_done <= 1'b1;
        end else begin
            if (tick_start) begin cons_cnt <= 0; vis_cnt <= 0; end
            case (d_state)
                D_IDLE: begin
                    if (upd_start) begin
                        u_i <= 0; u_pv <= 1'b0; fl_cnt <= 0; firedv <= 0;
                        dp_done <= 1'b0; d_state <= D_UPD;
                    end else if (rew_start) begin
                        w_a <= 0; w_delta <= rew_delta; dp_done <= 1'b0; d_state <= D_REW;
                    end else if (cfg_re) begin
                        c_sel <= cfg_sel; d_state <= D_CFG;
                    end else if (cfg_we && cfg_sel == 3'd6) begin
                        rate[cfg_addr[LOGN-1:0]] <= cfg_wdata[7:0];
                    end else if (ext_valid) begin
                        x_node <= ext_local; d_state <= D_EXT;
                    end else if (in_valid) begin
                        r_src <= in_data[23:0]; r_h <= fold(in_data[23:0]); d_state <= D_PI;
                    end
                end
                D_CFG: begin
                    case (c_sel)
                        3'd0: cfg_rdata <= {14'd0, node_rd};
                        3'd1: cfg_rdata <= {26'd0, gid_rd};
                        3'd2: cfg_rdata <= fop_rd;
                        3'd3: cfg_rdata <= {38'd0, fo_rd};
                        3'd4: cfg_rdata <= idx_rd;
                        3'd5: cfg_rdata <= {26'd0, syn_rd};
                        default: cfg_rdata <= {42'd0, rate[cfg_addr[LOGN-1:0]]};
                    endcase
                    d_state <= D_IDLE;
                end
                D_EXT: begin                        // node word of x_node is in node_rd
                    acc[x_node] <= acc[x_node] + {{8{node_rd[35]}}, node_rd[35:20]};
                    if (!touched[x_node]) begin
                        touched[x_node] <= 1'b1;
                        tl_mem[tl_cnt[LOGN-1:0]] <= x_node;
                        tl_cnt <= tl_cnt + 1'b1;
                    end
                    d_state <= D_IDLE;
                end
                D_PI: d_state <= D_PC;              // idx[r_h] read issued
                D_PC: begin
                    if (!idx_rd[49]) begin
                        miss_cnt <= miss_cnt + 1; cons_cnt <= cons_cnt + 1; d_state <= D_IDLE;
                    end else if (idx_rd[48:25] == r_src) begin
                        if (idx_rd[24]) begin
                            accumulate(idx_rd[23:0], 1'b1, {{(16-LOGNB){1'b0}}, r_h});
                            cons_cnt <= cons_cnt + 1; d_state <= D_IDLE;
                        end else begin
                            r_ptr <= idx_rd[23:8]; r_len <= idx_rd[7:0];
                            r_iss <= 0; r_proc <= 0; r_pv <= 1'b0; d_state <= D_SEG;
                        end
                    end else begin
                        r_h <= r_h + 1'b1; d_state <= D_PI;
                    end
                end
                D_SEG: begin                        // pipelined: issue next, use previous
                    if (r_pv) begin
                        accumulate(syn_rd, 1'b0, {{(16-LOGS){1'b0}}, r_paddr});
                        r_proc <= r_proc + 1'b1;
                    end
                    if (r_iss != r_len) begin
                        r_paddr <= r_ptr[LOGS-1:0]; r_ptr <= r_ptr + 1'b1;
                        r_iss <= r_iss + 1'b1; r_pv <= 1'b1;
                    end else r_pv <= 1'b0;
                    if (r_pv && r_proc + 1'b1 == r_len) begin
                        cons_cnt <= cons_cnt + 1; d_state <= D_IDLE;
                    end
                end
                D_UPD: begin                        // pipelined over nodes
                    if (u_pv) begin
                        if (n_fire) begin
                            fl_mem[fl_cnt[LOGN-1:0]] <= u_node;
                            fl_cnt <= fl_cnt + 1'b1;
                            firedv[u_node] <= 1'b1;
                        end
                        if (p_homeo) rate[u_node] <= n_rate;
                        acc[u_node] <= 0;
                        touched[u_node] <= 1'b0;
                    end
                    if (u_i != (full_mode ? N : tl_cnt)) begin
                        u_node <= upd_target; u_i <= u_i + 1'b1; u_pv <= 1'b1;
                    end else begin
                        u_pv <= 1'b0;
                        if (!u_pv || u_i == 0) begin
                            tl_cnt <= 0;
                            if (p_learn && vis_cnt != 0) begin l_i <= 0; d_state <= D_LRN; end
                            else begin dp_done <= 1'b1; d_state <= D_IDLE; end
                        end
                    end
                end
                D_LRN: begin                        // issue read of visit l_i
                    l_ent <= vis_mem[l_i[LOGV-1:0]];
                    d_state <= D_LRN2;
                end
                D_LRN2: begin                       // write back (in steering block)
                    if (l_i + 1'b1 == vis_cnt) begin dp_done <= 1'b1; d_state <= D_IDLE; end
                    else begin l_i <= l_i + 1'b1; d_state <= D_LRN; end
                end
                D_REW: begin w_cur <= w_a; d_state <= D_REW2; end
                D_REW2: begin
                    if (w_a + 1 == NB + S) begin dp_done <= 1'b1; d_state <= D_IDLE; end
                    else begin w_a <= w_a + 1'b1; d_state <= D_REW; end
                end
                default: d_state <= D_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
