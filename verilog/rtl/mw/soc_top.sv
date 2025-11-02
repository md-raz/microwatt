// SPDX-License-Identifier: Apache-2.0
`default_nettype none

module soc_top (
`ifdef USE_POWER_PINS
    inout vccd1,   // 1.8V digital
    inout vssd1,   // ground
`endif
    input  wire wb_clk_i,
    input  wire wb_rst_i,

    // user IOs
    input  wire [10:0] io_in,
    output wire [10:0] io_out,
    output wire [10:0] io_oeb,

    // UART pads
    input  wire uart_rx,
    output wire uart_tx
);

    // ---------------------------
    // Wishbone address map
    // ---------------------------
    localparam int NS = 2;
    localparam [32*NS-1:0] BASE = {
        32'h0000_2000,  // [1] RAM
        32'h0000_0000   // [0] ROM
    };
    localparam [32*NS-1:0] MASK = {
        32'hFFFF_E000,  // [1] 32 KiB window
        32'hFFFF_F000   // [0] 4 KiB window
    };

    // optional: index aliases
    localparam int S_ROM = 0;
    localparam int S_RAM = 1;

    // Slice out per-slave base/mask (single source of truth)
    localparam logic [31:0] ROM_BASE = BASE[32*S_ROM +: 32];
    localparam logic [31:0] RAM_BASE = BASE[32*S_RAM +: 32];

    // ---------------------------
    // Single clock/reset
    // ---------------------------
    wire clk = wb_clk_i;
    wire rst = wb_rst_i;

    // ---------------------------
    // Master <-> Fabric wires
    // ---------------------------
    wire                 wbm_cyc, wbm_stb, wbm_we;
    wire [3:0]           wbm_sel;
    wire [31:0]          wbm_adr, wbm_dat_w, wbm_dat_r;
    wire                 wbm_ack;
    wire                 wbm_err;   // fabric exposes err, Microwatt ignores it

    // Fabric vectors to/from slaves
    wire [NS-1:0]        wbs_cyc, wbs_stb, wbs_we;
    wire [4*NS-1:0]      wbs_sel;
    wire [32*NS-1:0]     wbs_adr, wbs_dat_w, wbs_dat_r;
    wire [NS-1:0]        wbs_ack, wbs_err;

    // ---------------------------
    // Fabric
    // ---------------------------
    wb_fabric #(
        .NS(NS),
        .BASE(BASE),
        .MASK(MASK)
    ) u_wb (
        // master
        .m_cyc  (wbm_cyc),
        .m_stb  (wbm_stb),
        .m_we   (wbm_we),
        .m_sel  (wbm_sel),
        .m_adr  (wbm_adr),
        .m_dat_w(wbm_dat_w),
        .m_dat_r(wbm_dat_r),
        .m_ack  (wbm_ack),
        .m_err  (wbm_err),

        // slaves
        .s_cyc  (wbs_cyc),
        .s_stb  (wbs_stb),
        .s_we   (wbs_we),
        .s_sel  (wbs_sel),
        .s_adr  (wbs_adr),
        .s_dat_w(wbs_dat_w),
        .s_dat_r(wbs_dat_r),
        .s_ack  (wbs_ack),
        .s_err  (wbs_err)
    );

    // ------------------------------------------------------------------------
    // Microwatt core (through wrapper that exposes a clean 32-bit WB master)
    // ------------------------------------------------------------------------
    wire [31:0] mw_gpio_out;
    wire [31:0] mw_gpio_dir;
    wire [31:0] mw_gpio_in;

    // Map our 11 user inputs to GPIO[10:0]
    assign mw_gpio_in[10:0]  = io_in;
    assign mw_gpio_in[31:11] = 21'h0;

    microwatt_top u_cpu (
        .clk      (clk),
        .rst      (rst),

        // WB master -> fabric
        .wbm_cyc  (wbm_cyc),
        .wbm_stb  (wbm_stb),
        .wbm_we   (wbm_we),
        .wbm_sel  (wbm_sel),
        .wbm_adr  (wbm_adr),
        .wbm_dat_w(wbm_dat_w),
        .wbm_dat_r(wbm_dat_r),
        .wbm_ack  (wbm_ack),
        .wbm_stall(1'b0),       // fabric is classic B4 single-beat; no stall

        // UART0 pads
        .uart_rx  (uart_rx),
        .uart_tx  (uart_tx),

        // GPIO
        .gpio_in  (mw_gpio_in),
        .gpio_out (mw_gpio_out),
        .gpio_dir (mw_gpio_dir)
    );

    // Map lower 11 GPIOs to user IOs
    assign io_out       = mw_gpio_out[10:0];
    // gpio_dir: 1 = output; io_oeb: 1 = Hi-Z -> invert
    assign io_oeb       = ~mw_gpio_dir[10:0];

    // =========================================================================
    // SLAVE 0: Boot ROM (read-only)
    // =========================================================================
    wire [31:0] rom_addr     = (wbs_adr[32*S_ROM +: 32] - ROM_BASE);
    wire [9:0]  rom_word_adr = rom_addr[11:2];

    wb_rom #(
        .ADDR_SIZE(10),                 // 4 KiB = 1024 words
        .INIT_HEX_FILE("rom.hex")
    ) u_boot_rom (
        .i_clk (clk),
        .i_rst (rst),
        .i_cyc (wbs_cyc[S_ROM]),
        .i_stb (wbs_stb[S_ROM]),
        .i_we  (1'b0),
        .i_sel (wbs_sel[4*S_ROM +: 4]),
        .i_adr (rom_word_adr),
        .i_dat (wbs_dat_w[32*S_ROM +: 32]),
        .o_dat (wbs_dat_r[32*S_ROM +: 32]),
        .o_ack (wbs_ack[S_ROM])
    );

    // ROM never errors
    assign wbs_err[S_ROM] = 1'b0;

    // =========================================================================
    // SLAVE 1: System SRAM (CF_SRAM_4096x32) via WB
    // =========================================================================
    // Using ChipFoundry 4096x32 SRAM with Wishbone wrapper
    // WIDTH=14: 4096 words × 4 bytes = 16KB, needs 14-bit byte address (bits [13:0])
    CF_SRAM_4096x32_wb_wrapper #(
        .WIDTH(14)
    ) u_sys_sram (
    `ifdef USE_POWER_PINS
        .VPWR      (vccd1),
        .VGND      (vssd1),
    `endif
        .wb_clk_i  (clk),
        .wb_rst_i  (rst),
        .wbs_cyc_i (wbs_cyc[S_RAM]),
        .wbs_stb_i (wbs_stb[S_RAM]),
        .wbs_we_i  (wbs_we[S_RAM]),
        .wbs_sel_i (wbs_sel[4*S_RAM +: 4]),
        .wbs_adr_i (wbs_adr[32*S_RAM +: 32]),
        .wbs_dat_i (wbs_dat_w[32*S_RAM +: 32]),
        .wbs_dat_o (wbs_dat_r[32*S_RAM +: 32]),
        .wbs_ack_o (wbs_ack[S_RAM])
    );

    // SRAM controller doesnt raise bus errors in normal use
    assign wbs_err[S_RAM] = 1'b0;

endmodule
`default_nettype wire
