////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	ahbscope.v
// {{{
// Project:	AHBScope, an AHB3-Lite hosted scope
//
// Purpose:	This is a generic/library routine for providing a bus accessed
//	'scope' or (perhaps more appropriately) a bus accessed logic analyzer.
//	The general operation is such that this 'scope' can record and report
//	on any 32 bit value transiting through the FPGA.  Once started and
//	reset, the scope records a copy of the input data every time the clock
//	ticks with the circuit enabled.  That is, it records these values up
//	until the trigger.  Once the trigger goes high, the scope will record
//	for br_holdoff more counts before stopping.  Values may then be read
//	from the buffer, oldest to most recent.  After reading, the scope may
//	then be reset for another run.
//
//	This particular version is designed to use an AHB3-Lite slave
//	interface, instead of a wishbone interface.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2015-2024, Gisselquist Technology, LLC
// {{{
// This program is free software (firmware): you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
// }}}
// License:	GPL, v3, as defined and found on www.gnu.org,
// {{{
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
`default_nettype	none
// }}}
module ahbscope #(
		// {{{
		parameter [4:0]			LGMEM = 5'd10,
		parameter			BUSW = 32,
		parameter [0:0]			SYNCHRONOUS=1,
		parameter		 	HOLDOFFBITS = 20,
		parameter [(HOLDOFFBITS-1):0]	DEFAULT_HOLDOFF = ((1<<(LGMEM-1))-4)
		// }}}
	) (
		// {{{
		// The input signals that we wish to record
		input	wire			i_data_clk, i_ce, i_trigger,
		input	wire	[(BUSW-1):0]	i_data,
		// The AHB3-Lite slave interface
		// {{{
		input	wire			HCLK,
		input	wire			HRESETn,
		input	wire			HSEL,
		input	wire	[31:0]		HADDR,
		input	wire	[1:0]		HTRANS,
		input	wire			HWRITE,
		input	wire	[2:0]		HSIZE,
		input	wire	[2:0]		HBURST,
		input	wire	[3:0]		HPROT,
		input	wire	[31:0]		HWDATA,
		input	wire			HREADY,
		output	wire	[31:0]		HRDATA,
		output	wire			HREADYOUT,
		output	wire			HRESP,
		// }}}
		// And, finally, for a final flair --- offer to interrupt the
		// CPU after our trigger has gone off.  This line is equivalent
		// to the scope being stopped.  It is not maskable here.
		output	wire			o_interrupt
		// }}}
	);

	// Signal declarations
	// {{{
	wire			bus_clock;
	wire			i_reset;
	wire			read_from_data;
	wire			write_stb;
	wire			write_to_control;
	wire			addr_phase_stb;
	wire			addr_phase_addr;
	wire			addr_phase_err;
	wire	[31:0]		i_bus_data;
	reg			br_active;
	reg			br_write;
	reg			br_addr;
	reg			br_error;
	reg	[(LGMEM-1):0]	raddr;
	reg	[(BUSW-1):0]	mem[0:((1<<LGMEM)-1)];
	wire		bw_reset_request, bw_manual_trigger,
			bw_disable_trigger, bw_reset_complete;
	reg	[2:0]	br_config;
	reg	[(HOLDOFFBITS-1):0]	br_holdoff;
	wire			dw_reset, dw_manual_trigger, dw_disable_trigger;
	reg			dr_triggered, dr_primed;
	wire			dw_trigger;
	(* ASYNC_REG="TRUE" *) reg	[(HOLDOFFBITS-1):0]	counter;
	reg			dr_stopped;
	reg	[(LGMEM-1):0]	waddr;
	localparam	STOPDELAY = 1;	// Calibrated value--don't change this
	wire	[(BUSW-1):0]		wr_piped_data;
	wire			bw_stopped, bw_triggered, bw_primed;
	wire	[19:0]		full_holdoff;
	reg	[31:0]		o_bus_data;
	wire	[4:0]		bw_lgmem;
	reg			br_level_interrupt;
	// }}}

	assign	bus_clock = HCLK;
	assign	i_reset = !HRESETn;
	assign	i_bus_data = HWDATA;

	////////////////////////////////////////////////////////////////////////
	//
	// Decode and handle the bus signaling in a (somewhat) portable manner
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	assign	addr_phase_stb  = HSEL && HREADY && HTRANS[1];
	assign	addr_phase_addr = HADDR[2];
	assign	addr_phase_err  = (HSIZE != 3'b010);

	initial	br_active = 1'b0;
	initial	br_write  = 1'b0;
	initial	br_addr   = 1'b0;
	initial	br_error  = 1'b0;
	always @(posedge bus_clock)
	if (i_reset)
	begin
		br_active <= 1'b0;
		br_write  <= 1'b0;
		br_addr   <= 1'b0;
		br_error  <= 1'b0;
	end else begin
		br_active <= addr_phase_stb;
		br_write  <= HWRITE;
		br_addr   <= addr_phase_addr;
		br_error  <= addr_phase_stb && addr_phase_err;
	end

	assign	read_from_data   = br_active && !br_write && br_addr && !br_error;
	assign	write_stb        = br_active &&  br_write && !br_error;
	assign	write_to_control = write_stb && !br_addr;

	assign	HREADYOUT = 1'b1;
	assign	HRESP = br_error;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Our status/config register
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	initial	br_config = 3'b0;
	initial	br_holdoff = DEFAULT_HOLDOFF;
	always @(posedge bus_clock)
	begin
		if (write_to_control)
		begin
			br_config[1:0] <= {
				i_bus_data[27],
				i_bus_data[26] };
			if (!i_bus_data[31] && br_config[2])
				br_holdoff <= i_bus_data[(HOLDOFFBITS-1):0];
		end

		//
		// Reset logic
		if (bw_reset_complete)
			br_config[2] <= 1'b1;
		else if (!br_config[2])
			br_config[2] <= 1'b0;
		else if (write_to_control && !i_bus_data[31])
			br_config[2] <= 1'b0;

		if (i_reset)
			br_config[2] <= 1'b0;
	end
	assign	bw_reset_request   = (!br_config[2]);
	assign	bw_manual_trigger  = (br_config[1]);
	assign	bw_disable_trigger = (br_config[0]);

	generate
	if (SYNCHRONOUS > 0)
	begin : GEN_SYNCHRONOUS
		assign	dw_reset = bw_reset_request;
		assign	dw_manual_trigger = bw_manual_trigger;
		assign	dw_disable_trigger = bw_disable_trigger;
		assign	bw_reset_complete = bw_reset_request;
	end else begin : GEN_ASYNC
		reg		r_reset_complete;
		(* ASYNC_REG = "TRUE" *) reg	[2:0]	q_iflags, r_iflags;

		initial	{ q_iflags, r_iflags } = 6'h0;
		initial	r_reset_complete = 1'b0;
		always @(posedge i_data_clk or posedge i_reset)
		if (i_reset)
		begin
			{ q_iflags, r_iflags } <= 6'h0;
			r_reset_complete <= 1'b0;
		end else begin
			q_iflags <= { bw_reset_request, bw_manual_trigger,
				bw_disable_trigger };
			r_iflags <= q_iflags;
			r_reset_complete <= (dw_reset);
		end

		assign	dw_reset = r_iflags[2];
		assign	dw_manual_trigger = r_iflags[1];
		assign	dw_disable_trigger = r_iflags[0];

		(* ASYNC_REG = "TRUE" *) reg	q_reset_complete,
						qq_reset_complete;
		initial	q_reset_complete = 1'b0;
		initial	qq_reset_complete = 1'b0;
		always @(posedge bus_clock or posedge i_reset)
		if (i_reset)
		begin
			q_reset_complete  <= 1'b0;
			qq_reset_complete <= 1'b0;
		end else begin
			q_reset_complete  <= r_reset_complete;
			qq_reset_complete <= q_reset_complete;
		end

		assign bw_reset_complete = qq_reset_complete;
	end endgenerate
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Set up the trigger
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	assign	dw_trigger = (dr_primed)&&(
				((i_trigger)&&(!dw_disable_trigger))
				||(dw_manual_trigger));

	initial	dr_triggered = 1'b0;
	always @(posedge i_data_clk)
	if (dw_reset)
		dr_triggered <= 1'b0;
	else if ((i_ce)&&(dw_trigger))
		dr_triggered <= 1'b1;

	initial	counter = 0;
	always @(posedge i_data_clk)
	if (dw_reset)
		counter <= 0;
	else if ((i_ce)&&(dr_triggered)&&(!dr_stopped))
		counter <= counter + 1'b1;

	initial	dr_stopped = 1'b0;
	always @(posedge i_data_clk)
	if ((!dr_triggered)||(dw_reset))
		dr_stopped <= 1'b0;
	else if (!dr_stopped)
	begin
		if (HOLDOFFBITS > 1)
			dr_stopped <= (counter >= br_holdoff);
		else if (HOLDOFFBITS <= 1)
			dr_stopped <= ((i_ce)&&(dw_trigger));
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Write to memory
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	initial	waddr = {(LGMEM){1'b0}};
	initial	dr_primed = 1'b0;
	always @(posedge i_data_clk)
	if (dw_reset)
	begin
		waddr <= 0;
		dr_primed <= 1'b0;
	end else if (i_ce && !dr_stopped)
	begin
		waddr <= waddr + {{(LGMEM-1){1'b0}},1'b1};
		if (!dr_primed)
			dr_primed <= (&waddr);
	end

	generate
	if (STOPDELAY == 0)
	begin : NO_STOPDLY
		assign	wr_piped_data = i_data;
	end else if (STOPDELAY == 1)
	begin : GEN_ONE_STOPDLY
		reg	[(BUSW-1):0]	data_pipe;
		always @(posedge i_data_clk)
		if (i_ce)
			data_pipe <= i_data;

		assign	wr_piped_data = data_pipe;
	end else begin : GEN_STOPDELAY
		reg	[(STOPDELAY*BUSW-1):0]	data_pipe;

		always @(posedge i_data_clk)
		if (i_ce)
			data_pipe <= { data_pipe[((STOPDELAY-1)*BUSW-1):0], i_data };
		assign	wr_piped_data = { data_pipe[(STOPDELAY*BUSW-1):((STOPDELAY-1)*BUSW)] };
	end endgenerate

	always @(posedge i_data_clk)
	if ((i_ce)&&(!dr_stopped))
		mem[waddr] <= wr_piped_data;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Move the status signals back to the bus clock
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	generate if (SYNCHRONOUS)
	begin : SYNCHRONOUS_RETURN
		assign	bw_stopped   = dr_stopped;
		assign	bw_triggered = dr_triggered;
		assign	bw_primed    = dr_primed;
	end else begin : ASYNC_STATUS
		(* ASYNC_REG = "TRUE" *) reg	[2:0]	q_oflags;
		reg	[2:0]	r_oflags;
		initial	q_oflags = 3'h0;
		initial	r_oflags = 3'h0;
		always @(posedge bus_clock or posedge i_reset)
		if (i_reset || bw_reset_request)
		begin
			q_oflags <= 3'h0;
			r_oflags <= 3'h0;
		end else begin
			q_oflags <= { dr_stopped, dr_triggered, dr_primed };
			r_oflags <= q_oflags;
		end

		assign	bw_stopped   = r_oflags[2];
		assign	bw_triggered = r_oflags[1];
		assign	bw_primed    = r_oflags[0];
	end endgenerate
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Read from the memory, using the bus clock
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	initial	raddr = 0;
	always @(posedge bus_clock)
	begin
		if ((bw_reset_request)||(write_to_control))
			raddr <= 0;
		else if ((read_from_data)&&(bw_stopped))
			raddr <= raddr + 1'b1;
	end

	assign full_holdoff[(HOLDOFFBITS-1):0] = br_holdoff;
	generate if (HOLDOFFBITS < 20)
	begin : GEN_FULL_HOLDOFF
		assign full_holdoff[19:(HOLDOFFBITS)] = 0;
	end endgenerate

	assign		bw_lgmem = LGMEM;

	initial	o_bus_data = 0;
	always @(posedge bus_clock)
	begin
		if (i_reset)
			o_bus_data <= 0;
		else if (br_active && !br_write && !br_error)
		begin
			if (!br_addr)
				o_bus_data <= { bw_reset_request,
						bw_stopped,
						bw_triggered,
						bw_primed,
						bw_manual_trigger,
						bw_disable_trigger,
						(raddr == {(LGMEM){1'b0}}),
						bw_lgmem,
						full_holdoff  };
			else if (!bw_stopped)
				o_bus_data <= i_data;
			else
				o_bus_data <= mem[raddr + waddr];
		end
	end

	assign	HRDATA = o_bus_data;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Interrupt generation
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	initial	br_level_interrupt = 1'b0;
	always @(posedge bus_clock)
	if ((bw_reset_complete)||(bw_reset_request)||(i_reset))
		br_level_interrupt <= 1'b0;
	else
		br_level_interrupt <= (bw_stopped)&&(!bw_disable_trigger);

	assign	o_interrupt = br_level_interrupt;
	// }}}

	// Make verilator happy
	// {{{
	// verilator lint_off UNUSED
	wire	unused;
	assign unused = &{ 1'b0, HTRANS[0], i_bus_data[30:28],
			i_bus_data[25:20], HADDR[31:3], HADDR[1:0],
			HBURST, HPROT };
	// verilator lint_on UNUSED
	// }}}
endmodule