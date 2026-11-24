// Polis tile (citizen), compiled mode: leak 0, refractory 0, no learning.
// See docs/tile.md and sim/polis/hw.py (the reference model).
//
// Tables (SRAM macros in silicon, register arrays here):
//   thr_mem  [N]  : signed 16-bit threshold (fixed point, 1/16)
//   gid_mem  [N]  : 24-bit virtual node id (resident number)
//   fo_ptr   [N]  : {ptr, len8} into fo_mem   (fan-out: destination tiles)
//   fo_mem   [F]  : 12-bit destination tile
//   idx_mem  [NB] : {valid, key24, inline, payload24}, linear probing,
//                   hash = xor-fold of the 24-bit source id
//                   payload = {ptr16, len8} or inline synapse {dst12, w8, flags4}
//   syn_mem  [S]  : synapse {dst12, w8 (signed), flags4}
// Phases per tick (driven by polis_mesh):
//   tick_start -> SEND fired list of last tick || RECV packets, accumulate
//   upd_start  -> UPDATE touched nodes: fire if clip16(acc) >= thr
`default_nettype none

module polis_tile #(
    parameter LOGN  = 4,
    parameter LOGNB = 4,
    parameter LOGS  = 4,
    parameter LOGF  = 4
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        tick_start,
    input  wire        upd_start,
    output reg         send_done,
    output reg         upd_done,
    output reg  [31:0] sent_cnt,
    output reg  [31:0] cons_cnt,
    output reg  [31:0] miss_cnt,
    // configuration write port (loader writes the tables before running)
    // sel: 0 thr, 1 gid, 2 fo_ptr, 3 fo_mem, 4 idx_mem, 5 syn_mem
    input  wire            cfg_we,
    input  wire [2:0]      cfg_sel,
    input  wire [15:0]     cfg_addr,
    input  wire [49:0]     cfg_data,
    // external input from the host, only between ticks
    input  wire            ext_valid,
    input  wire [LOGN-1:0] ext_local,
    // router local port
    output reg         out_valid,
    output reg  [39:0] out_data,
    input  wire        out_ready,
    input  wire        in_valid,
    input  wire [39:0] in_data,
    output wire        in_ready
);
    localparam N = 1 << LOGN, NB = 1 << LOGNB, S = 1 << LOGS, F = 1 << LOGF;

    reg signed [15:0] thr_mem [0:N-1];
    reg [23:0]        gid_mem [0:N-1];
    reg [LOGF+7:0]    fo_ptr  [0:N-1];
    reg [11:0]        fo_mem  [0:F-1];
    reg [49:0]        idx_mem [0:NB-1];
    reg [23:0]        syn_mem [0:S-1];

    reg signed [23:0] acc     [0:N-1];
    reg [N-1:0]       touched;
    reg [LOGN-1:0]    tl_mem  [0:N-1];
    reg [LOGN:0]      tl_cnt;
    reg [LOGN-1:0]    fl_mem  [0:N-1];
    reg [LOGN:0]      fl_cnt;

    always @(posedge clk) begin
        if (cfg_we) begin
            case (cfg_sel)
                3'd0: thr_mem[cfg_addr[LOGN-1:0]]  <= cfg_data[15:0];
                3'd1: gid_mem[cfg_addr[LOGN-1:0]]  <= cfg_data[23:0];
                3'd2: fo_ptr[cfg_addr[LOGN-1:0]]   <= cfg_data[LOGF+7:0];
                3'd3: fo_mem[cfg_addr[LOGF-1:0]]   <= cfg_data[11:0];
                3'd4: idx_mem[cfg_addr[LOGNB-1:0]] <= cfg_data[49:0];
                3'd5: syn_mem[cfg_addr[LOGS-1:0]]  <= cfg_data[23:0];
                default: ;
            endcase
        end
    end

`ifndef SYNTHESIS
    integer z;
    initial for (z = 0; z < N; z = z + 1) acc[z] = 24'sd0;
`endif

    function [LOGNB-1:0] fold;
        input [23:0] s;
        integer i;
        begin
            fold = {LOGNB{1'b0}};
            for (i = 0; i < 24; i = i + LOGNB) fold = fold ^ (s >> i);
        end
    endfunction

    // ------------------------------------------------------------ SEND
    localparam S_IDLE = 2'd0, S_LOAD = 2'd1, S_EMIT = 2'd2;
    reg [1:0]      s_state;
    reg [LOGN:0]   s_i;
    reg [LOGF-1:0] s_ptr;
    reg [7:0]      s_left;
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
                S_LOAD: if (s_i == fl_cnt) begin
                    send_done <= 1'b1; s_state <= S_IDLE;
                end else begin
                    {s_ptr, s_left} <= fo_ptr[s_node];
                    if (fo_ptr[s_node][7:0] == 8'd0) s_i <= s_i + 1'b1;
                    else s_state <= S_EMIT;
                end
                S_EMIT: if (!out_valid || out_ready) begin
                    out_valid <= 1'b1;
                    out_data  <= {4'd1, fo_mem[s_ptr], gid_mem[s_node]};
                    sent_cnt  <= sent_cnt + 1;
                    s_ptr     <= s_ptr + 1'b1;
                    s_left    <= s_left - 1'b1;
                    if (s_left == 8'd1) begin s_i <= s_i + 1'b1; s_state <= S_LOAD; end
                end
                default: s_state <= S_IDLE;
            endcase
        end
    end

    // ------------------------------------------- RECV + UPDATE (data plane)
    localparam R_IDLE = 3'd0, R_PROBE = 3'd1, R_SEG = 3'd2, R_UPD = 3'd3;
    reg [2:0]       r_state;
    reg [23:0]      r_src;
    reg [LOGNB-1:0] r_h;
    reg [15:0]      r_ptr;
    reg [7:0]       r_len;
    reg [LOGN:0]    u_i;

    assign in_ready = (r_state == R_IDLE) && !ext_valid;

    wire [49:0] e      = idx_mem[r_h];
    wire [23:0] seg_e  = syn_mem[r_ptr[LOGS-1:0]];

    // accumulate one synapse entry {dst12, w8, flags4}
    task accumulate;
        input [23:0] syn;
        reg [LOGN-1:0] d;
        begin
            d = syn[12 +: LOGN];
            acc[d] <= acc[d] + {{16{syn[11]}}, syn[11:4]};
            if (!touched[d]) begin
                touched[d] <= 1'b1;
                tl_mem[tl_cnt[LOGN-1:0]] <= d;
                tl_cnt <= tl_cnt + 1'b1;
            end
        end
    endtask

    wire [LOGN-1:0]   u_node = tl_mem[u_i[LOGN-1:0]];
    wire signed [23:0] u_acc = acc[u_node];
    wire signed [15:0] u_v   = (u_acc > 24'sd32767)  ? 16'sd32767 :
                               (u_acc < -24'sd32768) ? -16'sd32768 : u_acc[15:0];

    always @(posedge clk) begin
        if (rst) begin
            r_state <= R_IDLE; touched <= 0; tl_cnt <= 0; fl_cnt <= 0;
            cons_cnt <= 0; miss_cnt <= 0; upd_done <= 1'b1;
        end else begin
            if (tick_start) cons_cnt <= 0;
            case (r_state)
                R_IDLE: begin
                    if (upd_start) begin
                        u_i <= 0; fl_cnt <= 0; upd_done <= 1'b0; r_state <= R_UPD;
                    end else if (ext_valid) begin
                        // host input: current equal to the node's threshold
                        acc[ext_local] <= acc[ext_local] + {{8{thr_mem[ext_local][15]}}, thr_mem[ext_local]};
                        if (!touched[ext_local]) begin
                            touched[ext_local] <= 1'b1;
                            tl_mem[tl_cnt[LOGN-1:0]] <= ext_local;
                            tl_cnt <= tl_cnt + 1'b1;
                        end
                    end else if (in_valid) begin
                        r_src <= in_data[23:0]; r_h <= fold(in_data[23:0]); r_state <= R_PROBE;
                    end
                end
                R_PROBE: begin
                    if (!e[49]) begin                       // empty slot: unknown source
                        miss_cnt <= miss_cnt + 1; cons_cnt <= cons_cnt + 1; r_state <= R_IDLE;
                    end else if (e[48:25] == r_src) begin
                        if (e[24]) begin                    // inline single synapse
                            accumulate(e[23:0]);
                            cons_cnt <= cons_cnt + 1; r_state <= R_IDLE;
                        end else begin
                            r_ptr <= e[23:8]; r_len <= e[7:0]; r_state <= R_SEG;
                        end
                    end else begin
                        r_h <= r_h + 1'b1;                  // linear probing
                    end
                end
                R_SEG: begin
                    accumulate(seg_e);
                    r_ptr <= r_ptr + 1'b1; r_len <= r_len - 1'b1;
                    if (r_len == 8'd1) begin cons_cnt <= cons_cnt + 1; r_state <= R_IDLE; end
                end
                R_UPD: begin
                    if (u_i == tl_cnt) begin
                        tl_cnt <= 0; upd_done <= 1'b1; r_state <= R_IDLE;
                    end else begin
                        if (u_v >= thr_mem[u_node]) begin
                            fl_mem[fl_cnt[LOGN-1:0]] <= u_node;
                            fl_cnt <= fl_cnt + 1'b1;
                        end
                        acc[u_node] <= 24'sd0;
                        touched[u_node] <= 1'b0;
                        u_i <= u_i + 1'b1;
                    end
                end
                default: r_state <= R_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
