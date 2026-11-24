// Polis chip top: G x G tiles + routers, global tick barrier.
// The barrier here is a central counter; in silicon it is the region
// representatives aggregating "my region is done" (docs/migration-protocol.md).
`default_nettype none

module polis_mesh1 #(
    parameter GRIDB = 2,
    parameter LOGN  = 4,
    parameter LOGNB = 4,
    parameter LOGS  = 4,
    parameter LOGF  = 4,
    parameter LOGV  = 6
) (
    input  wire              clk,
    input  wire              rst,
    input  wire              go,          // start one tick
    input  wire              rew_go,      // start a reward sweep (Art. 8)
    input  wire signed [7:0] rew_delta,
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
    input  wire [2*GRIDB-1:0] obs_tile,
    input  wire [LOGN-1:0]   obs_local,
    output wire              obs_fired,
    input  wire              cfg_re,
    output wire [49:0]       cfg_rdata,
    output reg               done,        // tick finished (one-cycle pulse)
    output reg  [31:0]       cycles,      // cycles of the last tick
    output wire [31:0]       misses,
    input  wire              cfg_we,
    input  wire [2*GRIDB-1:0] cfg_tile,
    input  wire [2:0]        cfg_sel,
    input  wire [15:0]       cfg_addr,
    input  wire [49:0]       cfg_wdata,
    input  wire              ext_valid,
    input  wire [2*GRIDB-1:0] ext_tile,
    input  wire [LOGN-1:0]   ext_local
);
    localparam G = 1 << GRIDB, T = G * G;

    reg tick_start, upd_start, rew_start;
    wire [T-1:0] send_done, upd_done;
    wire [50*T-1:0] rdata;
    wire [(T<<LOGN)-1:0] fv;
    assign obs_fired = fv[{obs_tile, obs_local}];   // tile * N + local, full width
    reg  [2*GRIDB-1:0] rsel;
    always @(posedge clk) if (cfg_re) rsel <= cfg_tile;
    assign cfg_rdata = rdata[rsel*50 +: 50];
    wire [32*T-1:0] sent, cons, miss;

    // router port wiring: index r*5 + p, p: 0 L, 1 N, 2 E, 3 S, 4 W
    wire [5*T-1:0]  r_in_valid, r_in_ready, r_out_valid, r_out_ready;
    wire [200*T-1:0] r_in_data, r_out_data;

    genvar gy, gx;
    generate
        for (gy = 0; gy < G; gy = gy + 1) begin : g_y
            for (gx = 0; gx < G; gx = gx + 1) begin : g_x
                localparam integer R = gy * G + gx;
                wire t_out_valid, t_in_ready;
                wire [39:0] t_out_data;

                polis_tile1 #(.LOGN(LOGN), .LOGNB(LOGNB), .LOGS(LOGS), .LOGF(LOGF), .LOGV(LOGV)) u_tile (
                    .clk(clk), .rst(rst),
                    .tick_start(tick_start), .upd_start(upd_start),
                    .rew_start(rew_start), .rew_delta(rew_delta),
                    .send_done(send_done[R]), .dp_done(upd_done[R]),
                    .p_leak(p_leak), .p_refr(p_refr), .p_homeo(p_homeo), .p_target(p_target),
                    .p_rs(p_rs), .p_hs(p_hs), .p_tmin(p_tmin), .p_tmax(p_tmax), .p_learn(p_learn),
                    .p_lp(p_lp), .p_dec(p_dec), .p_prune(p_prune), .p_il(p_il), .p_rsh(p_rsh),
                    .cfg_re(cfg_re && cfg_tile == R), .cfg_rdata(rdata[R*50 +: 50]),
                    .firedv_o(fv[R*(1<<LOGN) +: (1<<LOGN)]),
                    .sent_cnt(sent[R*32 +: 32]), .cons_cnt(cons[R*32 +: 32]),
                    .miss_cnt(miss[R*32 +: 32]),
                    .cfg_we(cfg_we && cfg_tile == R), .cfg_sel(cfg_sel),
                    .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
                    .ext_valid(ext_valid && ext_tile == R), .ext_local(ext_local),
                    .out_valid(t_out_valid), .out_data(t_out_data),
                    .out_ready(r_in_ready[R*5 + 0]),
                    .in_valid(r_out_valid[R*5 + 0]), .in_data(r_out_data[R*200 + 0 +: 40]),
                    .in_ready(t_in_ready));

                // local port
                assign r_in_valid[R*5 + 0]          = t_out_valid;
                assign r_in_data[R*200 + 0 +: 40]   = t_out_data;
                assign r_out_ready[R*5 + 0]         = t_in_ready;
                // north (from the tile above, its south output)
                if (gy > 0) begin : g_n
                    assign r_in_valid[R*5 + 1]        = r_out_valid[(R-G)*5 + 3];
                    assign r_in_data[R*200 + 40 +: 40] = r_out_data[(R-G)*200 + 120 +: 40];
                    assign r_out_ready[R*5 + 1]       = r_in_ready[(R-G)*5 + 3];
                end else begin : g_n0
                    assign r_in_valid[R*5 + 1] = 1'b0;
                    assign r_in_data[R*200 + 40 +: 40] = 40'd0;
                    assign r_out_ready[R*5 + 1] = 1'b1;
                end
                // east
                if (gx < G - 1) begin : g_e
                    assign r_in_valid[R*5 + 2]        = r_out_valid[(R+1)*5 + 4];
                    assign r_in_data[R*200 + 80 +: 40] = r_out_data[(R+1)*200 + 160 +: 40];
                    assign r_out_ready[R*5 + 2]       = r_in_ready[(R+1)*5 + 4];
                end else begin : g_e0
                    assign r_in_valid[R*5 + 2] = 1'b0;
                    assign r_in_data[R*200 + 80 +: 40] = 40'd0;
                    assign r_out_ready[R*5 + 2] = 1'b1;
                end
                // south
                if (gy < G - 1) begin : g_s
                    assign r_in_valid[R*5 + 3]         = r_out_valid[(R+G)*5 + 1];
                    assign r_in_data[R*200 + 120 +: 40] = r_out_data[(R+G)*200 + 40 +: 40];
                    assign r_out_ready[R*5 + 3]        = r_in_ready[(R+G)*5 + 1];
                end else begin : g_s0
                    assign r_in_valid[R*5 + 3] = 1'b0;
                    assign r_in_data[R*200 + 120 +: 40] = 40'd0;
                    assign r_out_ready[R*5 + 3] = 1'b1;
                end
                // west
                if (gx > 0) begin : g_w
                    assign r_in_valid[R*5 + 4]          = r_out_valid[(R-1)*5 + 2];
                    assign r_in_data[R*200 + 160 +: 40] = r_out_data[(R-1)*200 + 80 +: 40];
                    assign r_out_ready[R*5 + 4]         = r_in_ready[(R-1)*5 + 2];
                end else begin : g_w0
                    assign r_in_valid[R*5 + 4] = 1'b0;
                    assign r_in_data[R*200 + 160 +: 40] = 40'd0;
                    assign r_out_ready[R*5 + 4] = 1'b1;
                end

                polis_router #(.GRIDB(GRIDB), .MX(gx), .MY(gy)) u_router (
                    .clk(clk), .rst(rst),
                    .in_valid(r_in_valid[R*5 +: 5]), .in_data(r_in_data[R*200 +: 200]),
                    .in_ready(r_in_ready[R*5 +: 5]),
                    .out_valid(r_out_valid[R*5 +: 5]), .out_data(r_out_data[R*200 +: 200]),
                    .out_ready(r_out_ready[R*5 +: 5]),
                    .empty());
            end
        end
    endgenerate

    // ---------------------------------------------------------- barrier
    reg [31:0] sum_sent, sum_cons, sum_miss;
    integer i;
    always @* begin
        sum_sent = 0; sum_cons = 0; sum_miss = 0;
        for (i = 0; i < T; i = i + 1) begin
            sum_sent = sum_sent + sent[i*32 +: 32];
            sum_cons = sum_cons + cons[i*32 +: 32];
            sum_miss = sum_miss + miss[i*32 +: 32];
        end
    end
    assign misses = sum_miss;

    localparam B_IDLE = 3'd0, B_ARM = 3'd1, B_COMM = 3'd2, B_UPD = 3'd3, B_REW = 3'd4;
    reg [2:0] b_state;
    always @(posedge clk) begin
        if (rst) begin
            b_state <= B_IDLE; tick_start <= 1'b0; upd_start <= 1'b0; rew_start <= 1'b0; done <= 1'b0;
            cycles <= 0;
        end else begin
            tick_start <= 1'b0; upd_start <= 1'b0; rew_start <= 1'b0; done <= 1'b0;
            case (b_state)
                B_IDLE: if (go) begin tick_start <= 1'b1; cycles <= 1; b_state <= B_ARM; end
                        else if (rew_go) begin rew_start <= 1'b1; cycles <= 1; b_state <= B_REW; end
                B_REW: begin
                    cycles <= cycles + 1;
                    if (&upd_done && !rew_start) begin done <= 1'b1; b_state <= B_IDLE; end
                end
                B_ARM:  begin cycles <= cycles + 1; b_state <= B_COMM; end   // flags settle
                B_COMM: begin
                    cycles <= cycles + 1;
                    if (&send_done && sum_sent == sum_cons) begin
                        upd_start <= 1'b1; b_state <= B_UPD;
                    end
                end
                B_UPD: begin
                    cycles <= cycles + 1;
                    if (&upd_done && !upd_start) begin done <= 1'b1; b_state <= B_IDLE; end
                end
            endcase
        end
    end
endmodule

`default_nettype wire
