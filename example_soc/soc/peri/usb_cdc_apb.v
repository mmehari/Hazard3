// APB wrapper for usb_cdc core: exposes a small control/status and simple CPU-accessible
// TX/RX data registers. This is intentionally minimal — intended to allow CPU-driven
// transfers and status reads for the example SoC.

`default_nettype none

module usb_cdc_apb (
    input  wire        clk,
    input  wire        rst_n,

    // APB slave interface
    input  wire        apbs_psel,
    input  wire        apbs_penable,
    input  wire        apbs_pwrite,
    input  wire [15:0] apbs_paddr,
    input  wire [31:0] apbs_pwdata,
    output reg  [31:0] apbs_prdata,
    output reg         apbs_pready,
    output reg         apbs_pslverr,

    // usb physical pins (optional for SoC; may be tied off)
    input  wire        dp_rx_i,
    input  wire        dn_rx_i,
    output wire        dp_pu_o,
    output wire        tx_en_o,
    output wire        dp_tx_o,
    output wire        dn_tx_o,

    // optional status outputs
    output wire [10:0] frame_o,
    output wire        configured_o
);

// Simple APB read/write behaviour
wire [1:0] addr_word = apbs_paddr[3:2];

// Simple register map (word addressed by apbs_paddr[3:2]):
// 0x0 STATUS     [0] out_valid, [1] in_ready
// 0x1 TX_DATA    [7:0] write a byte -> will drive in_data_i + in_valid until accepted
// 0x2 RX_DATA    [7:0] read last received byte (from out_data_o)

localparam ADDR_FSTAT = 0;
localparam ADDR_TX = 1;
localparam ADDR_RX = 2;

reg [7:0] in_data_r;
reg in_valid_r; // set when CPU writes TX_DATA, cleared when usb core accepts

reg [7:0] out_data_r;
reg out_valid_r;

// Instantiate core. Tie app_clk to clk for simplicity.
wire [7:0]  in_data_i;
wire        in_valid_i;
wire        in_ready_o;
wire [7:0]  out_data_o;
wire        out_valid_o;
wire        out_ready_i;

usb_cdc #(
    .VENDORID (16'h1209),
    .PRODUCTID(16'h0001),
    .IN_BULK_MAXPACKETSIZE(64),
    .OUT_BULK_MAXPACKETSIZE(64)
) usb_core (
    .clk_i      (clk),
    .rstn_i     (rst_n),
    .app_clk_i  (clk),

    .out_data_o (out_data_o),
    .out_valid_o(out_valid_o),
    .out_ready_i(out_ready_i),

    .in_data_i  (in_data_i),
    .in_valid_i (in_valid_i),
    .in_ready_o (in_ready_o),

    .frame_o    (frame_o),
    .configured_o(configured_o),

    .dp_pu_o    (dp_pu_o),
    .tx_en_o    (tx_en_o),
    .dp_tx_o    (dp_tx_o),
    .dn_tx_o    (dn_tx_o),
    .dp_rx_i    (dp_rx_i),
    .dn_rx_i    (dn_rx_i)
);

// Connect simple single-channel app FIFO handshake
assign in_data_i = in_data_r;
assign in_valid_i = in_valid_r;
assign out_ready_i = 1'b1; // always ready to accept core's OUT endpoint consumption

// Capture incoming OUT data when asserted
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_data_r <= 8'h00;
        out_valid_r <= 1'b0;
    end else begin
        if (out_valid_o) begin
            out_data_r <= out_data_o;
            out_valid_r <= 1'b1;
        end else begin
            // keep last value until CPU reads it
            if (apbs_psel && apbs_penable && ~apbs_pwrite && (addr_word == ADDR_RX)) begin
                out_valid_r <= 1'b0;
            end
        end
    end
end

// TX handshake: clear tx_req when accepted by core
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        in_valid_r <= 1'b0;
        in_data_r <= 8'h00;
    end else begin
        if (in_valid_r && in_ready_o)
            in_valid_r <= 1'b0;

        // CPU write
        if (apbs_psel && apbs_penable && apbs_pwrite && (addr_word == ADDR_TX)) begin
            in_data_r <= apbs_pwdata[7:0];
            in_valid_r <= 1'b1;
        end
    end
end

always @(*) begin
    apbs_prdata = 32'h0;
    apbs_pready = 1'b0;
    apbs_pslverr = 1'b0;

    if (apbs_psel && ~apbs_penable) begin
        // APB phase 1: just indicate not ready yet (we'll assert pready in next cycle)
        apbs_pready = 1'b0;
    end else if (apbs_psel && apbs_penable) begin
        apbs_pready = 1'b1;
        if (apbs_pwrite) begin
            // writes handled in sequential logic; return zero
            apbs_prdata = 32'h0;
        end else begin
            // reads
            case (addr_word)
                ADDR_FSTAT: apbs_prdata = {30'h0, out_valid_r, in_ready_o};
                ADDR_TX:    apbs_prdata = {24'h0, in_data_r};
                ADDR_RX:    apbs_prdata = {24'h0, out_data_r};
                default:    apbs_prdata = 32'h0;
            endcase
        end
    end
end

endmodule
