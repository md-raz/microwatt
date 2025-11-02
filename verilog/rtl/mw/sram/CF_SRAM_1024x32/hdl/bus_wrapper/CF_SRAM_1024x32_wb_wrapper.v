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

// CF_SRAM_1024x32_wb_wrapper.v
// This module instantiates a Wishbone RAM controller and a 1024x32 SRAM macro,
// connecting them to form a complete Wishbone slave memory block.

// This wrapper provides a Wishbone B4 compliant slave interface.
// It uses the ram_wb_controller to translate Wishbone transactions
// into signals suitable for a generic 1024x32 SRAM macro.

`ifdef USE_POWER_PINS
    `define USE_PG_PIN
`endif

// CF_SRAM_1024x32_wb_wrapper.v
// This module instantiates a Wishbone RAM controller and a 1024x32 SRAM macro,
// connecting them to form a complete Wishbone slave memory block.

// This wrapper provides a Wishbone B4 compliant slave interface.
// It uses the ram_wb_controller to translate Wishbone transactions
// into signals suitable for a generic 1024x32 SRAM macro.

module CF_SRAM_1024x32_wb_wrapper #(parameter WIDTH = 12) (
`ifdef USE_POWER_PINS
    inout VPWR,
    inout VGND,
`endif
    // Wishbone Bus Interface (Slave)
    input wb_clk_i,             // Wishbone clock input
    input wb_rst_i,             // Wishbone reset input (active high)
    input wbs_stb_i,            // Wishbone strobe input (indicates valid cycle)
    input wbs_cyc_i,            // Wishbone cycle input (indicates active bus cycle)
    input wbs_we_i,             // Wishbone write enable input (1 for write, 0 for read)
    input [3:0] wbs_sel_i,      // Wishbone byte lane select input (controls 4 byte lanes)
    input [31:0] wbs_dat_i,     // Wishbone data input (data from master for writes)
    input [31:0] wbs_adr_i,     // Wishbone address input (32-bit Wishbone address)
    output wbs_ack_o,           // Wishbone acknowledge output (asserted for transaction completion)
    output [31:0] wbs_dat_o     // Wishbone data output (data to master for reads)
);

    // Internal wires to connect the ram_wb_controller outputs to the CF_SRAM_1024x32 inputs,
    // and the CF_SRAM_1024x32 output to the ram_wb_controller input.
    // These signals form the SRAM-specific interface.
    wire [31:0] sram_do;    // Data output from SRAM to controller
    wire [31:0] sram_di;    // Data input from controller to SRAM
    wire [31:0] sram_ben;   // Byte enable from controller to SRAM (32-bit, derived from wbs_sel_i)
    wire [WIDTH-3:0] sram_ad;  // Address from controller to SRAM (word-aligned address), now using WIDTH
    wire sram_en;          // Chip enable from controller to SRAM (active high)
    wire sram_r_wb;        // Read/Write bar from controller to SRAM (1=Read, 0=Write)
    wire sram_clk_in;      // Clock signal for SRAM, directly from Wishbone clock

    // New wires for the CF_SRAM_1024x32 module's additional pins
    wire sram_scan_out_cc;  // Scan chain output

    // Instantiate the ram_wb_controller module
    // This module translates the Wishbone protocol into generic SRAM control signals.
    ram_controller_wb #(
        .WIDTH (WIDTH) // Pass the new WIDTH parameter to the controller's WIDTH parameter
    ) i_ram_wb_controller (
        // Wishbone Bus Connections
        .wb_clk_i    (wb_clk_i),
        .wb_rst_i    (wb_rst_i),
        .wbs_stb_i   (wbs_stb_i),
        .wbs_cyc_i   (wbs_cyc_i),
        .wbs_we_i    (wbs_we_i),
        .wbs_sel_i   (wbs_sel_i),
        .wbs_dat_i   (wbs_dat_i),
        .wbs_adr_i   (wbs_adr_i),
        .wbs_ack_o   (wbs_ack_o), // Controller generates Wishbone ACK
        .wbs_dat_o   (wbs_dat_o), // Controller provides Wishbone data out

        // SRAM Interface Connections (to be connected to the SRAM macro)
        .DO          (sram_do),       // Connect SRAM's Data Output to controller's DO input
        .DI          (sram_di),       // Connect controller's Data Input to SRAM's DI input
        .BEN         (sram_ben),      // Connect controller's Byte Enable to SRAM's BEN input
        .AD          (sram_ad),       // Connect controller's Address to SRAM's AD input
        .EN          (sram_en),       // Connect controller's Chip Enable to SRAM's EN input
        .R_WB        (sram_r_wb),     // Connect controller's Read/Write Bar to SRAM's R_WB input
        .CLKin       (sram_clk_in)    // Connect controller's Clock to SRAM's CLKin input
    );

    // Instantiate the CF_SRAM_1024x32 macro
    // This is where your actual SRAM IP or memory block would be placed.
    // The dummy module definition has been removed as per your request.
    CF_SRAM_1024x32 i_sram (
        .DO         (sram_do),
        .AD         (sram_ad),
        .BEN        (sram_ben),
        .CLKin      (sram_clk_in),
        .DI         (sram_di),
        .EN         (sram_en),
        .R_WB       (sram_r_wb),
        // Connect new pins based on the provided list
        .ScanOutCC  (sram_scan_out_cc), // Output from SRAM
        .ScanInCC   (1'b0),             // Tie to 0 for unused scan input (example)
        .ScanInDL   (1'b0),             // Tie to 0 for unused scan input (example)
        .ScanInDR   (1'b0),             // Tie to 0 for unused scan input (example)
        .SM         (1'b0),             // Tie to 0 (example)
        .TM         (1'b0),             // Tie to 0 (example)
        .WLBI       (1'b0),             // Tie to 0 (example)
        .WLOFF      (1'b0),             // Tie to 0 (example)
    `ifdef USE_PG_PIN
        .vgnd       (1'b0),             // Tie to ground
        .vnb        (1'b0),             // Tie to ground (body bias)
        .vpb        (1'b1),             // Tie to VDD (body bias)
        .vpwra      (1'b1),             // Tie to VDD
    `endif
        .vpwrac     (1'b1),             // Tie to VDD
    `ifdef USE_PG_PIN
        .vpwrm      (1'b1),             // Tie to VDD
        .vpwrp      (1'b1),             // Tie to VDD
    `endif
        .vpwrpc     (1'b1)              // Tie to VDD
    );

endmodule

