// Polis 5-port mesh router: XY dimension-order routing, single-flit 40-bit
// packets {type[39:36], dst_tile[35:24], src[23:0]}, 2-entry FIFO per input,
// round-robin arbitration per output. Ports: 0 Local, 1 North, 2 East,
// 3 South, 4 West. y grows southwards. Deadlock-free (XY).
`default_nettype none

module polis_fifo2 #(parameter W = 40) (
    input  wire         clk,
    input  wire         rst,
    input  wire         in_valid,
    input  wire [W-1:0] in_data,
    output wire         in_ready,
    output wire         out_valid,
    output wire [W-1:0] out_data,
    input  wire         pop
);
    reg [W-1:0] m0, m1;
    reg [1:0]   cnt;
    assign in_ready  = (cnt != 2'd2);
    assign out_valid = (cnt != 2'd0);
    assign out_data  = m0;
    wire push = in_valid & in_ready;
    wire popv = pop & out_valid;
    always @(posedge clk) begin
        if (rst) begin
            cnt <= 2'd0;
        end else begin
            case ({push, popv})
                2'b10: begin
                    if (cnt == 2'd0) m0 <= in_data; else m1 <= in_data;
                    cnt <= cnt + 2'd1;
                end
                2'b01: begin
                    m0  <= m1;
                    cnt <= cnt - 2'd1;
                end
                2'b11: begin
                    if (cnt == 2'd1) m0 <= in_data;
                    else begin m0 <= m1; m1 <= in_data; end
                end
                default: ;
            endcase
        end
    end
endmodule

module polis_router #(
    parameter GRIDB = 2,
    parameter MX = 0,
    parameter MY = 0
) (
    input  wire          clk,
    input  wire          rst,
    input  wire [4:0]    in_valid,
    input  wire [199:0]  in_data,
    output wire [4:0]    in_ready,
    output reg  [4:0]    out_valid,
    output reg  [199:0]  out_data,
    input  wire [4:0]    out_ready,
    output wire          empty
);
    wire [4:0]   h_valid;
    wire [199:0] h_data;
    reg  [4:0]   pop;
    genvar i;
    generate
        for (i = 0; i < 5; i = i + 1) begin : g_in
            polis_fifo2 #(.W(40)) f (
                .clk(clk), .rst(rst),
                .in_valid(in_valid[i]), .in_data(in_data[i*40 +: 40]),
                .in_ready(in_ready[i]),
                .out_valid(h_valid[i]), .out_data(h_data[i*40 +: 40]),
                .pop(pop[i]));
        end
    endgenerate
    assign empty = ~|h_valid;

    // route computation for each input head
    function [2:0] route;
        input [39:0] p;
        reg [GRIDB-1:0] dx, dy;
        begin
            dx = p[24 +: GRIDB];
            dy = p[24 + GRIDB +: GRIDB];
            if (dx > MX)      route = 3'd2;
            else if (dx < MX) route = 3'd4;
            else if (dy > MY) route = 3'd3;
            else if (dy < MY) route = 3'd1;
            else              route = 3'd0;
        end
    endfunction

    reg [2:0] rr [0:4];          // round-robin pointer per output
    reg [2:0] grant [0:4];       // granted input per output
    reg [4:0] gvalid;
    integer o, k, cand;
    always @* begin
        pop = 5'b0;
        for (o = 0; o < 5; o = o + 1) begin
            gvalid[o] = 1'b0;
            grant[o]  = 3'd0;
            for (k = 0; k < 5; k = k + 1) begin
                cand = (rr[o] + k) % 5;
                if (!gvalid[o] && h_valid[cand] && route(h_data[cand*40 +: 40]) == o) begin
                    gvalid[o] = 1'b1;
                    grant[o]  = cand[2:0];
                end
            end
            out_valid[o] = gvalid[o];
            out_data[o*40 +: 40] = h_data[grant[o]*40 +: 40];
            if (gvalid[o] && out_ready[o]) pop[grant[o]] = 1'b1;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            for (o = 0; o < 5; o = o + 1) rr[o] <= 3'd0;
        end else begin
            for (o = 0; o < 5; o = o + 1)
                if (gvalid[o] && out_ready[o])
                    rr[o] <= (grant[o] == 3'd4) ? 3'd0 : grant[o] + 3'd1;
        end
    end
endmodule

`default_nettype wire
