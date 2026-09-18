// Simple APB I2C master (open-drain, clock-stretch aware)
`default_nettype none

module i2c_apb #(
	parameter W_ADDR = 16,
	parameter W_DATA = 32
) (
	input  wire              clk,
	input  wire              rst_n,

	input  wire [W_ADDR-1:0] apbs_paddr,
	input  wire              apbs_psel,
	input  wire              apbs_penable,
	input  wire              apbs_pwrite,
	input  wire [W_DATA-1:0] apbs_pwdata,
	output reg  [W_DATA-1:0] apbs_prdata,
	output wire              apbs_pready,
	output wire              apbs_pslverr,

	// i2c_en claims SDA/SCL from GPIO. oe=1 drives the line low.
	output wire              i2c_en,
	output reg               scl_oe,
	output reg               sda_oe,
	input  wire              scl_i,
	input  wire              sda_i
);

// Register map:
// 0x00 CSR
//   [0]     EN    RW  Claim SDA/SCL from GPIO
//   [1]     STA   WO  START (or repeated START) before the byte
//   [2]     STO   WO  STOP after the byte (or immediately if no RD/WR)
//   [3]     RD    WO  Read one byte
//   [4]     WR    WO  Write DATA
//   [5]     NACK  WO  Send NACK after a read (otherwise ACK)
//   [8]     BUSY  RO  Transfer in progress
//   [9]     RXACK RO  1 if the slave NACKed the last write
// 0x04 DIV  [15:0] SCL half-period in clk cycles (100 kHz @ 48 MHz => 240)
// 0x08 DATA [7:0]  Write: TX byte; read: last RX byte

localparam ADDR_CSR  = 4'h0;
localparam ADDR_DIV  = 4'h4;
localparam ADDR_DATA = 4'h8;

localparam S_IDLE        = 4'd0;
localparam S_START_SETUP = 4'd1;
localparam S_START_SCLH  = 4'd2;
localparam S_START_SDA   = 4'd3;
localparam S_START_SCLL  = 4'd4;
localparam S_DATA_SCLL   = 4'd5;
localparam S_DATA_SCLH   = 4'd6;
localparam S_ACK_SCLL    = 4'd7;
localparam S_ACK_SCLH    = 4'd8;
localparam S_POST_ACK    = 4'd9;
localparam S_STOP_SCLL   = 4'd10;
localparam S_STOP_SCLH   = 4'd11;
localparam S_STOP_SDA    = 4'd12;

assign apbs_pready  = 1'b1;
assign apbs_pslverr = 1'b0;

wire wen = apbs_psel && apbs_penable && apbs_pwrite;
wire ren = apbs_psel && apbs_penable && !apbs_pwrite;
wire [3:0] addr = apbs_paddr[3:0];

reg        csr_en;
reg        busy;
reg        rxack;
reg        bus_active;
reg        cmd_sta;
reg        cmd_sto;
reg        cmd_rd;
reg        cmd_wr;
reg        cmd_nack;
reg        cmd_issue;
reg [15:0] div;
reg [7:0]  tx_byte;
reg [7:0]  rx_byte;

assign i2c_en = csr_en;

reg [3:0]  state;
reg [3:0]  bit_idx;
reg [7:0]  shifter;
reg [15:0] tcnt;

wire scl_high_wait =
	(state == S_START_SCLH) ||
	(state == S_DATA_SCLH)  ||
	(state == S_ACK_SCLH)   ||
	(state == S_STOP_SCLH);
wire stretch = scl_high_wait && !scl_i;
wire [15:0] div_m1 = (div == 16'd0) ? 16'd0 : (div - 16'd1);
wire tick = busy && !stretch && (tcnt == div_m1);

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		csr_en     <= 1'b0;
		cmd_sta    <= 1'b0;
		cmd_sto    <= 1'b0;
		cmd_rd     <= 1'b0;
		cmd_wr     <= 1'b0;
		cmd_nack   <= 1'b0;
		cmd_issue  <= 1'b0;
		div        <= 16'd240;
		tx_byte    <= 8'h00;
		apbs_prdata <= {W_DATA{1'b0}};
	end else begin
		cmd_issue <= 1'b0;

		if (ren) begin
			case (addr)
				ADDR_CSR:  apbs_prdata <= {{(W_DATA-10){1'b0}}, rxack, busy, 7'b0, csr_en};
				ADDR_DIV:  apbs_prdata <= {{(W_DATA-16){1'b0}}, div};
				ADDR_DATA: apbs_prdata <= {{(W_DATA-8){1'b0}}, rx_byte};
				default:   apbs_prdata <= {W_DATA{1'b0}};
			endcase
		end

		if (wen) begin
			case (addr)
				ADDR_CSR: begin
					csr_en <= apbs_pwdata[0];
					if (apbs_pwdata[0] && !busy &&
					    (apbs_pwdata[1] || apbs_pwdata[2] ||
					     apbs_pwdata[3] || apbs_pwdata[4])) begin
						cmd_sta   <= apbs_pwdata[1];
						cmd_sto   <= apbs_pwdata[2];
						cmd_wr    <= apbs_pwdata[4];
						cmd_rd    <= apbs_pwdata[3] && !apbs_pwdata[4];
						cmd_nack  <= apbs_pwdata[5];
						cmd_issue <= 1'b1;
					end
				end
				ADDR_DIV:  div     <= apbs_pwdata[15:0];
				ADDR_DATA: tx_byte <= apbs_pwdata[7:0];
				default: ;
			endcase
		end
	end
end

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		busy       <= 1'b0;
		rxack      <= 1'b0;
		bus_active <= 1'b0;
		state      <= S_IDLE;
		bit_idx    <= 4'd0;
		shifter    <= 8'h00;
		rx_byte    <= 8'h00;
		tcnt       <= 16'd0;
		scl_oe     <= 1'b0;
		sda_oe     <= 1'b0;
	end else if (!csr_en && !cmd_issue) begin
		busy       <= 1'b0;
		bus_active <= 1'b0;
		state      <= S_IDLE;
		tcnt       <= 16'd0;
		scl_oe     <= 1'b0;
		sda_oe     <= 1'b0;
	end else if (cmd_issue) begin
		busy    <= 1'b1;
		tcnt    <= 16'd0;
		bit_idx <= 4'd7;
		shifter <= tx_byte;
		if (cmd_sta)
			state <= S_START_SETUP;
		else if (cmd_wr || cmd_rd)
			state <= S_DATA_SCLL;
		else
			state <= S_STOP_SCLL;
	end else if (busy) begin
		if (!stretch) begin
			if (tick)
				tcnt <= 16'd0;
			else
				tcnt <= tcnt + 16'd1;
		end else begin
			tcnt <= 16'd0;
		end

		case (state)
			S_START_SETUP: begin
				sda_oe <= 1'b0;
				scl_oe <= bus_active;
				if (tick)
					state <= S_START_SCLH;
			end
			S_START_SCLH: begin
				sda_oe <= 1'b0;
				scl_oe <= 1'b0;
				if (tick)
					state <= S_START_SDA;
			end
			S_START_SDA: begin
				sda_oe <= 1'b1;
				scl_oe <= 1'b0;
				if (tick)
					state <= S_START_SCLL;
			end
			S_START_SCLL: begin
				sda_oe <= 1'b1;
				scl_oe <= 1'b1;
				if (tick) begin
					bus_active <= 1'b1;
					if (cmd_wr || cmd_rd) begin
						state   <= S_DATA_SCLL;
						bit_idx <= 4'd7;
						shifter <= tx_byte;
					end else if (cmd_sto) begin
						state <= S_STOP_SCLL;
					end else begin
						state <= S_IDLE;
						busy  <= 1'b0;
					end
				end
			end
			S_DATA_SCLL: begin
				scl_oe <= 1'b1;
				if (cmd_wr)
					sda_oe <= ~shifter[7];
				else
					sda_oe <= 1'b0;
				if (tick)
					state <= S_DATA_SCLH;
			end
			S_DATA_SCLH: begin
				scl_oe <= 1'b0;
				if (tick) begin
					scl_oe <= 1'b1;
					if (cmd_rd)
						shifter <= {shifter[6:0], sda_i};
					else
						shifter <= {shifter[6:0], 1'b0};
					if (bit_idx == 4'd0) begin
						state <= S_ACK_SCLL;
					end else begin
						bit_idx <= bit_idx - 4'd1;
						state   <= S_DATA_SCLL;
					end
				end
			end
			S_ACK_SCLL: begin
				scl_oe <= 1'b1;
				if (cmd_rd)
					sda_oe <= ~cmd_nack;
				else
					sda_oe <= 1'b0;
				if (tick)
					state <= S_ACK_SCLH;
			end
			S_ACK_SCLH: begin
				scl_oe <= 1'b0;
				if (tick) begin
					scl_oe <= 1'b1;
					if (cmd_wr)
						rxack <= sda_i;
					else begin
						rxack   <= 1'b0;
						rx_byte <= shifter;
					end
					state <= S_POST_ACK;
				end
			end
			S_POST_ACK: begin
				scl_oe <= 1'b1;
				if (cmd_sto)
					sda_oe <= 1'b1;
				if (tick) begin
					if (cmd_sto) begin
						state <= S_STOP_SCLH;
					end else begin
						state      <= S_IDLE;
						busy       <= 1'b0;
						bus_active <= 1'b1;
					end
				end
			end
			S_STOP_SCLL: begin
				scl_oe <= 1'b1;
				sda_oe <= 1'b1;
				if (tick)
					state <= S_STOP_SCLH;
			end
			S_STOP_SCLH: begin
				scl_oe <= 1'b0;
				sda_oe <= 1'b1;
				if (tick)
					state <= S_STOP_SDA;
			end
			S_STOP_SDA: begin
				scl_oe <= 1'b0;
				sda_oe <= 1'b0;
				if (tick) begin
					state      <= S_IDLE;
					busy       <= 1'b0;
					bus_active <= 1'b0;
				end
			end
			default: begin
				state <= S_IDLE;
				busy  <= 1'b0;
			end
		endcase
	end
end

endmodule
