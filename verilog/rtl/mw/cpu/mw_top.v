// SPDX-License-Identifier: Apache-2.0
`default_nettype none
module microwatt_top (
    input  wire        clk,
    input  wire        rst,

    // 32-bit external Wishbone MASTER (Microwatt -> your fabric)
    output wire        wbm_cyc,
    output wire        wbm_stb,
    output wire        wbm_we,
    output wire [3:0]  wbm_sel,
    output wire [31:0] wbm_adr,   // byte address
    output wire [31:0] wbm_dat_w,
    input  wire [31:0] wbm_dat_r,
    input  wire        wbm_ack,
    input  wire        wbm_stall, // tie 0 if classic WB

    // UART0
    input  wire        uart_rx,
    output wire        uart_tx,

    // GPIO
    input  wire [31:0] gpio_in,
    output wire [31:0] gpio_out,
    output wire [31:0] gpio_dir
);

    // Microwatt ext-IO ports are word-addressed (30 bits). Convert to byte addr.
    wire [29:0] ext_adr_w;

    // Unused master ports (outputs from Microwatt) we don’t care to route
    wire [28:0] wb_dram_adr;
    wire [63:0] wb_dram_dat_w;
    wire [7:0]  wb_dram_sel;
    wire        wb_dram_cyc, wb_dram_stb, wb_dram_we;

    wire [31:0] dma_dat_w;
    wire        dma_ack, dma_stall;

    // External feature flags (unused but harmless)
    wire wb_ext_is_dram_csr, wb_ext_is_dram_init, wb_ext_is_eth, wb_ext_is_sdcard;

    // Run indicators/reset request (optional)
    wire run_out, run_outs, sw_soc_reset;

    // Map byte/word addressing
    assign wbm_adr   = {ext_adr_w, 2'b00};   // word->byte
    assign wbm_stall = 1'b0;                 // classic WB fabric? then tie to 0

    // Instantiate generated netlist
    soc u_soc (
        .rst         (rst),
        .system_clk  (clk),

        .run_out     (run_out),
        .run_outs    (run_outs),

        // -------- DRAM WB (UNUSED) : tie return channel to idle -----------
        .\wb_dram_in[adr]  (wb_dram_adr),
        .\wb_dram_in[dat]  (wb_dram_dat_w),
        .\wb_dram_in[sel]  (wb_dram_sel),
        .\wb_dram_in[cyc]  (wb_dram_cyc),
        .\wb_dram_in[stb]  (wb_dram_stb),
        .\wb_dram_in[we]   (wb_dram_we),
        .\wb_dram_out[dat] (64'h0),
        .\wb_dram_out[ack] (1'b0),
        .\wb_dram_out[stall] (1'b0),

        // -------- External IO Wishbone (USE THIS as your master) ----------
        .\wb_ext_io_in[adr] (ext_adr_w),
        .\wb_ext_io_in[dat] (wbm_dat_w),
        .\wb_ext_io_in[sel] (wbm_sel),
        .\wb_ext_io_in[cyc] (wbm_cyc),
        .\wb_ext_io_in[stb] (wbm_stb),
        .\wb_ext_io_in[we]  (wbm_we),

        .\wb_ext_io_out[dat] (wbm_dat_r),
        .\wb_ext_io_out[ack] (wbm_ack),
        .\wb_ext_io_out[stall] (wbm_stall),

        .wb_ext_is_dram_csr (wb_ext_is_dram_csr),
        .wb_ext_is_dram_init (wb_ext_is_dram_init),
        .wb_ext_is_eth      (wb_ext_is_eth),
        .wb_ext_is_sdcard   (wb_ext_is_sdcard),

        // -------- External DMA WB (UNUSED) : tie inputs low ---------------
        .\wishbone_dma_out[adr] (30'h0),
        .\wishbone_dma_out[dat] (32'h0),
        .\wishbone_dma_out[sel] (4'h0),
        .\wishbone_dma_out[cyc] (1'b0),
        .\wishbone_dma_out[stb] (1'b0),
        .\wishbone_dma_out[we]  (1'b0),
        .\wishbone_dma_in[dat]  (dma_dat_w),
        .\wishbone_dma_in[ack]  (dma_ack),
        .\wishbone_dma_in[stall ](dma_stall),

        // -------- External IRQs (unused) ----------------------------------
        .ext_irq_eth   (1'b0),
        .ext_irq_sdcard(1'b0),

        // -------- UARTs ----------------------------------------------------
        .uart0_txd (uart_tx),
        .uart0_rxd (uart_rx),
        .uart1_txd (/* unused */),
        .uart1_rxd (1'b1),   // idle high

        // -------- SPI flash (unused) --------------------------------------
        .spi_flash_sck    (/* unused */),
        .spi_flash_cs_n   (/* unused */),
        .spi_flash_sdat_o (/* unused */),
        .spi_flash_sdat_oe (/* unused */),
        .spi_flash_sdat_i (1'b0),

        // -------- GPIO -----------------------------------------------------
        .gpio_out (gpio_out),
        .gpio_dir (gpio_dir),
        .gpio_in  (gpio_in),

        // -------- SW reset request (optional observe) ----------------------
        .sw_soc_reset (sw_soc_reset)
    );

endmodule
`default_nettype wire
