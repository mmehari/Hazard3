// Simple APB GPIO peripheral
`default_nettype none

module gpio_apb #(
	parameter NGPIO = 28,
	parameter W_ADDR = 16,
	parameter W_DATA = 32
) (
	input  wire                  clk,
	input  wire                  rst_n,

	input  wire [W_ADDR-1:0]     apbs_paddr,
	input  wire                  apbs_psel,
	input  wire                  apbs_penable,
	input  wire                  apbs_pwrite,
	input  wire [W_DATA-1:0]     apbs_pwdata,
	output reg  [W_DATA-1:0]     apbs_prdata,
	output wire                  apbs_pready,
	output wire                  apbs_pslverr,

	inout  wire [NGPIO-1:0]      gpio_io
);

// Simple register map:
// 0x00 - DATA (read current input, write sets OUT register)
// 0x04 - DIR  (1 = output, 0 = input)

localparam ADDR_DATA = 3'h0;
localparam ADDR_DIR  = 3'h4;

reg [NGPIO-1:0] dir_reg; // 1 -> drive
reg [NGPIO-1:0] out_reg;

wire [NGPIO-1:0] gpio_in;

// Read the external pin values
assign gpio_in = gpio_io;

// Drive outputs when dir bit is set
genvar i;
generate
	for (i = 0; i < NGPIO; i = i + 1) begin : drv
		assign gpio_io[i] = dir_reg[i] ? out_reg[i] : 1'bz;
	end
endgenerate

// APB simple ready/err
assign apbs_pready = 1'b1;
assign apbs_pslverr = 1'b0;

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		apbs_prdata <= {W_DATA{1'b0}};
		dir_reg <= {NGPIO{1'b0}};
		out_reg <= {NGPIO{1'b0}};
	end else begin
		if (apbs_psel && apbs_penable && !apbs_pwrite) begin
			case (apbs_paddr[2:0])
				ADDR_DATA: apbs_prdata <= { { (W_DATA-NGPIO){1'b0} }, gpio_in };
				ADDR_DIR:  apbs_prdata <= { { (W_DATA-NGPIO){1'b0} }, dir_reg };
				default:   apbs_prdata <= {W_DATA{1'b0}};
			endcase
		end
		if (apbs_psel && apbs_penable && apbs_pwrite) begin
			case (apbs_paddr[2:0])
				ADDR_DATA: out_reg <= apbs_pwdata[NGPIO-1:0];
				ADDR_DIR:  dir_reg <= apbs_pwdata[NGPIO-1:0];
				default: ;
			endcase
		end
	end
end

endmodule
