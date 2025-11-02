// SPDX-License-Identifier: Apache-2.0
`default_nettype none

// wb_rom.v
// Parameterized Wishbone ROM slave module
// - Single-cycle read access
// - Read-only (write attempts are ignored)
// - Loads initial content from hex file

module wb_rom #(
    parameter ADDR_SIZE = 10,                    // Address width in words (default 1024 words = 4KB)
    parameter INIT_HEX_FILE = "rom.hex"          // Hex file to load
) (
    input  wire                  i_clk,
    input  wire                  i_rst,
    
    // Wishbone slave interface
    input  wire                  i_cyc,          // Cycle active
    input  wire                  i_stb,          // Strobe
    input  wire                  i_we,           // Write enable (ignored for ROM)
    input  wire [3:0]            i_sel,          // Byte select (ignored, always reads full word)
    input  wire [ADDR_SIZE-1:0]  i_adr,          // Word address
    input  wire [31:0]           i_dat,          // Write data (ignored)
    output reg  [31:0]           o_dat,          // Read data
    output reg                   o_ack           // Acknowledge
);

    // ROM storage array
    reg [31:0] rom_data [0:(1<<ADDR_SIZE)-1];
    
    // Initialize ROM from hex file
    initial begin
        // Initialize to zero first
        integer i;
        for (i = 0; i < (1<<ADDR_SIZE); i = i + 1) begin
            rom_data[i] = 32'h00000000;
        end
        
        // Load hex file if it exists
        if (INIT_HEX_FILE != "") begin
            $readmemh(INIT_HEX_FILE, rom_data);
            $display("wb_rom: Loaded %s into ROM (%0d words)", INIT_HEX_FILE, 1<<ADDR_SIZE);
        end
    end
    
    // Combinational read
    always @(*) begin
        if (i_cyc && i_stb && !i_we) begin
            o_dat = rom_data[i_adr];
        end else begin
            o_dat = 32'h00000000;
        end
    end
    
    // Registered acknowledge (single-cycle response)
    always @(posedge i_clk) begin
        if (i_rst) begin
            o_ack <= 1'b0;
        end else begin
            // Acknowledge any valid cycle+strobe (read or ignored write)
            o_ack <= i_cyc && i_stb;
        end
    end

endmodule

`default_nettype wire

