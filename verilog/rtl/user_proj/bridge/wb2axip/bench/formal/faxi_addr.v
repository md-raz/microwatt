////////////////////////////////////////////////////////////////////////////////
//
// Filename:	bench/formal/faxi_addr.v
// {{{
// Project:	WB2AXIPSP: bus bridges and other odds and ends
//
// Purpose:	The AXI (full) standard has some rather complicated addressing
//		modes, where the address can either be FIXED, INCRementing, or
//	even where it can WRAP around some boundary.  When in either INCR or
//	WRAP modes, the next address must always be aligned.  In WRAP mode,
//	the next address calculation needs to wrap around a given value, and
//	that value is dependent upon the burst size (i.e. bytes per beat) and
//	length (total numbers of beats).  Since this calculation can be
//	non-trivial, and since it needs to be done multiple times, the logic
//	below captures it for every time it might be needed.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2019-2025, Gisselquist Technology, LLC
// {{{
// This file is part of the WB2AXIP project.
//
// The WB2AXIP project contains free software and gateware, licensed under the
// Apache License, Version 2.0 (the "License").  You may not use this project,
// or this file, except in compliance with the License.  You may obtain a copy
// of the License at
// }}}
//	http://www.apache.org/licenses/LICENSE-2.0
// {{{
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
// License for the specific language governing permissions and limitations
// under the License.
//
////////////////////////////////////////////////////////////////////////////////
//
`default_nettype none
// }}}
module	faxi_addr #(
		// {{{
		parameter	AW=32
		// }}}
	) (
		// {{{
		input	wire	[AW-1:0]	i_last_addr,
		input	wire	[2:0]		i_size, // 1b, 2b, 4b, 8b, etc
		input	wire	[1:0]		i_burst, // fixed, incr, wrap, reserved
		input	wire	[7:0]		i_len,
		output	reg	[7:0]		o_incr,
		output	reg	[AW-1:0]	o_next_addr
		// }}}
	);

	(* keep *) reg	[AW-1:0]	wrap_mask, increment;

	// increment
	// {{{
	always @(*)
	begin
		increment = 0;
		if (i_burst != 0)
		begin
			// verilator lint_off WIDTH
			case(i_size)
			0: increment =  1;
			1: increment =  2;
			2: increment =  4;
			3: increment =  8;
			4: increment = 16;
			5: increment = 32;
			6: increment = 64;
			7: increment = 128;
			default: increment = 0;
			endcase
			// verilator lint_on WIDTH
		end
	end
	// }}}

	// wrap_mask
	// {{{
	always @(*)
	begin
		wrap_mask = 0;
		if (i_burst == 2'b10)
		begin
			if (i_len == 1)
				wrap_mask = (1<<(i_size+1));
			else if (i_len == 3)
				wrap_mask = (1<<(i_size+2));
			else if (i_len == 7)
				wrap_mask = (1<<(i_size+3));
			else if (i_len == 15)
				wrap_mask = (1<<(i_size+4));
			wrap_mask = wrap_mask - 1;
		end
	end
	// }}}

	// o_next_addr
	// {{{
	always @(*)
	begin
		o_next_addr = i_last_addr + increment;
		if (i_burst != 2'b00)
		begin
			// Align any subsequent address
			// verilator lint_off SELRANGE
			if(i_size == 1)
				o_next_addr[0] = 0;
			else if ((i_size == 2)&&(AW>=2))
				o_next_addr[1:0] = 0;
			else if ((i_size == 3)&&(AW>=3))
				o_next_addr[2:0] = 0;
			else if ((i_size == 4)&&(AW>=4))
				o_next_addr[3:0] = 0;
			else if ((i_size == 5)&&(AW>=5))
				o_next_addr[4:0] = 0;
			else if ((i_size == 6)&&(AW>=6))
				o_next_addr[((AW>6)?5:AW-1):0] = 0;
			else if ((i_size == 7)&&(AW>=7))
				o_next_addr[((AW>7)?6:AW-1):0] = 0;
			// verilator lint_on  SELRANGE
		end
		if (i_burst == 2'b10)
		begin
			// WRAP!
			o_next_addr[AW-1:0] = (i_last_addr & ~wrap_mask)
					| (o_next_addr & wrap_mask);
		end
	end
	// }}}

	// o_incr
	// {{{
	always @(*)
	begin
		o_incr = 0;
		o_incr[((AW>7)?7:AW-1):0] = increment[((AW>7)?7:AW-1):0];
	end
	// }}}

endmodule
