// SPDX-FileCopyrightText: 2025 Umbralogic Technologies LLC d/b/a ChipFoundry and its Licensors, All Rights Reserved
// ========================================================================================
//
// This software is proprietary and protected by copyright and other intellectual property
// rights. Any reproduction, modification, translation, compilation, or representation
// beyond expressly permitted use is strictly prohibited.
//
// Access and use of this software are granted solely for integration into semiconductor
// chip designs created by you as part of ChipFoundry shuttles or ChipFoundry managed
// production programs. It is exclusively for Umbralogic Technologies LLC d/b/a ChipFoundry production purposes, and you may
// not modify or convey the software for any other purpose.
//
// DISCLAIMER: UMBRALOGIC TECHNOLOGIES LLC D/B/A CHIPFOUNDRY AND ITS LICENSORS PROVIDE THIS MATERIAL "AS IS," WITHOUT
// WARRANTIES OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
// Umbralogic Technologies LLC d/b/a ChipFoundry reserves the right to make changes without notice. Neither Umbralogic Technologies LLC d/b/a ChipFoundry nor its
// licensors assume any liability arising from the application or use of any product or
// circuit described herein. Umbralogic Technologies LLC d/b/a ChipFoundry products are not authorized for use as components
// in life-support devices.
//
// This license is subject to the terms of any separate agreement you have with Umbralogic Technologies LLC d/b/a ChipFoundry
// concerning the use of this software, which shall control in case of conflict.

// ram_wb_controller.v
// This module implements a Wishbone B4 compliant slave interface
// for controlling a generic 32-bit wide SRAM (Static RAM).
// It handles address decoding, byte lane enabling, data transfer,
// and generating the Wishbone acknowledge signal.

module ram_controller_wb #(parameter WIDTH = 12) (
    // Wishbone Bus Interface (Slave)
    input wb_clk_i,             // Wishbone clock input
    input wb_rst_i,             // Wishbone reset input (active high)
    input wbs_stb_i,            // Wishbone strobe input (indicates valid cycle)
    input wbs_cyc_i,            // Wishbone cycle input (indicates active bus cycle)
    input wbs_we_i,             // Wishbone write enable input (1 for write, 0 for read)
    input [3:0] wbs_sel_i,      // Wishbone byte lane select input (controls 4 byte lanes)
    input [31:0] wbs_dat_i,     // Wishbone data input (data from master for writes)
    input [31:0] wbs_adr_i,     // Wishbone address input (32-bit address)
    output reg wbs_ack_o,       // Wishbone acknowledge output (asserted for transaction completion)
    output [31:0] wbs_dat_o,    // Wishbone data output (data to master for reads)

    // SRAM Device Interface
    input  [31:0] DO,           // Data output from SRAM (data read from SRAM)
    output [31:0] DI,           // Data input to SRAM (data written to SRAM)
    output [31:0] BEN,          // Byte Enable for SRAM (32-bit byte enable for 32-bit data)
    output [WIDTH-3:0] AD,      // Address output to SRAM (WIDTH-3 bits for word addressing)
    output EN,                  // Chip Enable for SRAM (active high)
    output R_WB,                // Read/Write Bar for SRAM (1 for read, 0 for write)
    output CLKin                // Clock input for SRAM
);

    // SRAM Interface Control Signal Assignments
    // These signals are driven combinatorially based on the Wishbone inputs.

    // AD: SRAM Address Lines
    // The Wishbone bus is typically byte-addressable (32-bit words), meaning the lower 2 bits
    // of wbs_adr_i (wbs_adr_i[1:0]) select a byte within a 32-bit word.
    // The SRAM is assumed to be word-addressable.
    // Therefore, we use wbs_adr_i[WIDTH-1:2] to generate the SRAM address.
    // Example: If WIDTH=12, then AD is 10 bits wide (wbs_adr_i[11:2]).
    assign AD = wbs_adr_i[WIDTH-1:2];

    // DI: Data Input to SRAM
    // During a Wishbone write operation, the data from the Wishbone bus (wbs_dat_i)
    // is passed directly to the SRAM's data input.
    // During a read, the value of DI does not matter for the SRAM, but it should be driven.
    assign DI = wbs_dat_i;

    // BEN: Byte Enable for SRAM
    // This 32-bit signal controls which bytes within the 32-bit SRAM word are enabled
    // for read or write operations. It is derived from the 4-bit Wishbone byte select (wbs_sel_i).
    // Each bit of wbs_sel_i corresponds to an 8-bit byte lane:
    // wbs_sel_i[0] -> BEN[7:0]   (Byte 0)
    // wbs_sel_i[1] -> BEN[15:8]  (Byte 1)
    // wbs_sel_i[2] -> BEN[23:16] (Byte 2)
    // wbs_sel_i[3] -> BEN[31:24] (Byte 3)
    assign BEN = {
        {8{wbs_sel_i[3]}}, // Replicate wbs_sel_i[3] 8 times for BEN[31:24]
        {8{wbs_sel_i[2]}}, // Replicate wbs_sel_i[2] 8 times for BEN[23:16]
        {8{wbs_sel_i[1]}}, // Replicate wbs_sel_i[1] 8 times for BEN[15:8]
        {8{wbs_sel_i[0]}}  // Replicate wbs_sel_i[0] 8 times for BEN[7:0]
    };

    // EN: SRAM Chip Enable
    // The SRAM is enabled when a Wishbone bus cycle is active (wbs_cyc_i) and
    // the strobe is asserted (wbs_stb_i), indicating a valid transaction.
    assign EN = wbs_cyc_i && wbs_stb_i;

    // R_WB: SRAM Read/Write Bar
    // This signal determines whether the SRAM performs a read or a write operation.
    // It is active high for reads and active low for writes.
    // `!wbs_we_i` means: if `wbs_we_i` is 0 (read), R_WB is 1; if `wbs_we_i` is 1 (write), R_WB is 0.
    assign R_WB = !wbs_we_i;

    // CLKin: SRAM Clock Input
    // The SRAM clock is directly connected to the Wishbone clock.
    assign CLKin = wb_clk_i;

    // wbs_dat_o: Wishbone Data Output
    // This output drives the data read from the SRAM back to the Wishbone bus master.
    // It is active only during a read operation (EN is asserted and it's not a write).
    // If no active read, it defaults to 32'b0. In a real hardware implementation,
    // this output would typically be tri-stated when not actively driving.
    assign wbs_dat_o = EN && !wbs_we_i ? DO : 32'b0;

    // wbs_ack_o: Wishbone Acknowledge Output (Registered)
    // This signal indicates that the Wishbone slave (this controller) has completed
    // the current transaction. It is asserted for one clock cycle when a valid
    // Wishbone strobe and cycle are active.
    always @(posedge wb_clk_i or posedge wb_rst_i) begin
        if (wb_rst_i)
            // On reset, de-assert acknowledge.
            wbs_ack_o <= 1'b0;
        else
            // Assert ACK when a new valid Wishbone transaction starts.
            // De-assert it in the next cycle if the strobe/cycle are no longer active,
            // ensuring a single-cycle acknowledge pulse.
            if (wbs_stb_i && wbs_cyc_i)
                wbs_ack_o <= 1'b1;
             else
                wbs_ack_o <= 1'b0;

            // if (EN & ~wbs_ack_o)
    end

endmodule
