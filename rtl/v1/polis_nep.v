// NEP v1 bridge (gateway tile): turns a NEP RUN message, delivered as the
// payload bytes of a UDP datagram (or a TCP frame), into host inputs and
// ticks of the mesh, and streams back the RESULT message.
// The Ethernet MAC and the IP/UDP/TCP stack sit in front of this block
// (standard IP or the gateway's general-purpose core); see docs/network.md.
//
// RUN    : "NM" 01 01 | ticks u16 | n u16 | n x (port u16, start u16, end u16)
// RESULT : "NM" 01 02 | n u16 | per output port (port u16, spikes u16, last u16)
// ERROR  : "NM" 01 04 | len u16 | "bad request"
// Port numbers: inputs 0..NIN-1, outputs NIN..NIN+NOUT-1; port_map gives
// each port its (tile, local node). PORTS (type 3) is answered by the core.
`default_nettype none

module polis_nep #(
    parameter TB   = 4,          // tile index bits
    parameter LOGN = 4,
    parameter NIN  = 16,
    parameter NOUT = 9,
    parameter LOGE = 6           // event buffer entries
) (
    input  wire          clk,
    input  wire          rst,
    // port map configuration
    input  wire          pm_we,
    input  wire [7:0]    pm_addr,
    input  wire [TB+LOGN-1:0] pm_data,
    // request byte stream
    input  wire          rx_valid,
    input  wire [7:0]    rx_data,
    input  wire          rx_last,
    output wire          rx_ready,
    // reply byte stream
    output reg           tx_valid,
    output reg  [7:0]    tx_data,
    output reg           tx_last,
    input  wire          tx_ready,
    // mesh control
    output reg           go,
    input  wire          done,
    output reg           ext_valid,
    output reg  [TB-1:0] ext_tile,
    output reg  [LOGN-1:0] ext_local,
    output wire [TB-1:0] obs_tile,
    output wire [LOGN-1:0] obs_local,
    input  wire          obs_fired
);
    localparam E = 1 << LOGE;
    reg [TB+LOGN-1:0] pmap [0:NIN+NOUT-1];
    always @(posedge clk) if (pm_we) pmap[pm_addr] <= pm_data;

    reg [15:0] ev_port [0:E-1];
    reg [15:0] ev_s    [0:E-1];
    reg [15:0] ev_e    [0:E-1];
    reg [15:0] spikes  [0:NOUT-1];
    reg [15:0] last    [0:NOUT-1];

    localparam P_HDR = 4'd0, P_TICKS = 4'd1, P_N = 4'd2, P_EV = 4'd3, P_DRAIN = 4'd4,
               X_TICK = 4'd5, X_INJ = 4'd6, X_INJ2 = 4'd7, X_GO = 4'd8, X_WAIT = 4'd9,
               X_OBS = 4'd10, T_RES = 4'd11, T_ERR = 4'd12;
    reg [3:0]  st;
    reg [4:0]  bcnt;            // byte counter within a field group
    reg [15:0] ticks, nev, ei, t;
    reg [47:0] sh;              // shift register for one event (6 bytes)
    reg        bad;
    reg [7:0]  oi;              // output port index while observing / sending
    reg [3:0]  ob;              // byte within an output record
    reg [15:0] tlen;

    assign rx_ready = (st <= P_DRAIN);
    wire rx = rx_valid && rx_ready;

    assign obs_tile  = pmap[NIN + oi][TB+LOGN-1:LOGN];
    assign obs_local = pmap[NIN + oi][LOGN-1:0];

    // "bad request"
    function [7:0] errch;
        input [3:0] i;
        case (i)
            0: errch = "b"; 1: errch = "a"; 2: errch = "d"; 3: errch = " ";
            4: errch = "r"; 5: errch = "e"; 6: errch = "q"; 7: errch = "u";
            8: errch = "e"; 9: errch = "s"; default: errch = "t";
        endcase
    endfunction

    integer k;
    wire in_window = (t >= ev_s[ei]) && (t < ev_e[ei]);

    always @(posedge clk) begin
        if (rst) begin
            st <= P_HDR; bcnt <= 0; bad <= 0; go <= 0; ext_valid <= 0; tx_valid <= 0; t <= 0; ei <= 0;
        end else begin
            go <= 1'b0; ext_valid <= 1'b0;
            if (tx_valid && tx_ready) tx_valid <= 1'b0;
            case (st)
                P_HDR: if (rx) begin
                    case (bcnt)
                        0: bad <= (rx_data != "N");
                        1: if (rx_data != "M") bad <= 1'b1;
                        2: if (rx_data != 8'd1) bad <= 1'b1;
                        3: if (rx_data != 8'd1) bad <= 1'b1;
                    endcase
                    bcnt <= bcnt + 1'b1;
                    if (bcnt == 3) begin
                        bcnt <= 0;
                        if (rx_last) st <= T_ERR;
                        else if (bad || rx_data != 8'd1) begin bad <= 1'b1; st <= P_DRAIN; end
                        else st <= P_TICKS;
                    end else if (rx_last) begin bcnt <= 0; st <= T_ERR; end
                end
                P_TICKS: if (rx) begin
                    ticks <= {ticks[7:0], rx_data};
                    bcnt <= bcnt + 1'b1;
                    if (bcnt == 1) begin bcnt <= 0; st <= rx_last ? T_ERR : P_N; end
                    else if (rx_last) begin bcnt <= 0; st <= T_ERR; end
                end
                P_N: if (rx) begin
                    nev <= {nev[7:0], rx_data};
                    bcnt <= bcnt + 1'b1;
                    if (bcnt == 1) begin
                        bcnt <= 0; ei <= 0;
                        if ({nev[7:0], rx_data} == 16'd0) st <= (bad || !rx_last) ? (rx_last ? T_ERR : P_DRAIN) : X_TICK;
                        else if ({nev[7:0], rx_data} > E) begin bad <= 1'b1; st <= rx_last ? T_ERR : P_DRAIN; end
                        else st <= rx_last ? T_ERR : P_EV;
                    end else if (rx_last) begin bcnt <= 0; st <= T_ERR; end
                end
                P_EV: if (rx) begin
                    sh <= {sh[39:0], rx_data};
                    bcnt <= bcnt + 1'b1;
                    if (bcnt == 5) begin
                        ev_port[ei[LOGE-1:0]] <= sh[39:24];
                        ev_s[ei[LOGE-1:0]]    <= sh[23:8];
                        ev_e[ei[LOGE-1:0]]    <= {sh[7:0], rx_data};
                        if (sh[39:24] >= NIN) bad <= 1'b1;
                        bcnt <= 0; ei <= ei + 1'b1;
                        if (ei + 1'b1 == nev) st <= (bad || sh[39:24] >= NIN || !rx_last) ? (rx_last ? T_ERR : P_DRAIN) : X_TICK;
                        else if (rx_last) st <= T_ERR;
                    end else if (rx_last) begin bcnt <= 0; st <= T_ERR; end
                end
                P_DRAIN: if (rx && rx_last) begin bcnt <= 0; st <= T_ERR; end  // swallow the rest
                // ---------------------------------------------------- execute
                X_TICK: begin
                    if (t == 16'd0 && ei != 16'hFFFF) begin
                        for (k = 0; k < NOUT; k = k + 1) begin spikes[k] <= 0; last[k] <= 16'hFFFF; end
                        ei <= 16'hFFFF;                    // marker: counters cleared
                    end else if (t == ticks) begin
                        oi <= 0; ob <= 0; bcnt <= 0; st <= T_RES;
                    end else begin
                        ei <= 0; st <= X_INJ;
                    end
                end
                X_INJ: begin                                 // inject events active at t
                    if (ei == nev) st <= X_GO;
                    else begin
                        if (in_window) begin
                            ext_valid <= 1'b1;
                            ext_tile  <= pmap[ev_port[ei]][TB+LOGN-1:LOGN];
                            ext_local <= pmap[ev_port[ei]][LOGN-1:0];
                            st <= X_INJ2;
                        end
                        ei <= ei + 1'b1;
                    end
                end
                X_INJ2: st <= X_INJ;                         // tile takes 2 cycles per input
                X_GO:   begin go <= 1'b1; st <= X_WAIT; end
                X_WAIT: if (done) begin oi <= 0; st <= X_OBS; end
                X_OBS: begin                                 // sample each output port
                    if (obs_fired) begin spikes[oi] <= spikes[oi] + 1'b1; last[oi] <= t; end
                    if (oi + 1 == NOUT) begin t <= t + 1'b1; ei <= 0; st <= X_TICK; end
                    else oi <= oi + 1'b1;
                end
                // ------------------------------------------------------ reply
                T_RES: if (!tx_valid || tx_ready) begin
                    tx_valid <= 1'b1; tx_last <= 1'b0;
                    if (bcnt < 6) begin
                        case (bcnt)
                            0: tx_data <= "N"; 1: tx_data <= "M"; 2: tx_data <= 8'd1; 3: tx_data <= 8'd2;
                            4: tx_data <= 8'd0; default: tx_data <= NOUT;
                        endcase
                        bcnt <= bcnt + 1'b1;
                    end else begin
                        case (ob)
                            0: tx_data <= (NIN + oi) >> 8;  1: tx_data <= NIN + oi;
                            2: tx_data <= spikes[oi][15:8]; 3: tx_data <= spikes[oi][7:0];
                            4: tx_data <= last[oi][15:8];   default: tx_data <= last[oi][7:0];
                        endcase
                        if (ob == 5) begin
                            ob <= 0;
                            if (oi + 1 == NOUT) begin tx_last <= 1'b1; t <= 0; bad <= 0; bcnt <= 0; st <= P_HDR; end
                            else oi <= oi + 1'b1;
                        end else ob <= ob + 1'b1;
                    end
                end
                T_ERR: if (!tx_valid || tx_ready) begin
                    tx_valid <= 1'b1; tx_last <= 1'b0;
                    case (bcnt)
                        0: tx_data <= "N"; 1: tx_data <= "M"; 2: tx_data <= 8'd1; 3: tx_data <= 8'd4;
                        4: tx_data <= 8'd0; 5: tx_data <= 8'd11;
                        default: tx_data <= errch(bcnt - 6);
                    endcase
                    if (bcnt == 16) begin tx_last <= 1'b1; bcnt <= 0; bad <= 0; t <= 0; st <= P_HDR; end
                    else bcnt <= bcnt + 1'b1;
                end
                default: st <= P_HDR;
            endcase
        end
    end
endmodule

`default_nettype wire
