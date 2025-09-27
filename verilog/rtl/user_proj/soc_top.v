// SPDX-License-Identifier: Apache-2.0
// Top-level SoC for Microwatt + peripherals + AXI-Lite accelerator

`default_nettype none

// Feature toggles (leave on; edit when you refine the build)
`define INCLUDE_PIC
`define INCLUDE_SPI
`define INCLUDE_TIMER

module soc_top (
`ifdef USE_POWER_PINS
    inout vccd1,   // 1.8V digital
    inout vssd1,   // ground
`endif

    input  wire        wb_clk_i,
    input  wire        wb_rst_i,

    // Simple user IO bank (11 pins available at wrapper)
    input  wire [10:0] io_in,
    output wire [10:0] io_out,
    output wire [10:0] io_oeb,

    // UART pins to pads
    input  wire        uart_rx,
    output wire        uart_tx
);

    // =========================================================================
    // CLOCK / RESET (single domain)
    // =========================================================================
    wire clk   = wb_clk_i;
    wire rst   = wb_rst_i;

    // =========================================================================
    // WISHBONE MASTER (from Microwatt)
    // =========================================================================
    wire        wbm_cyc;
    wire        wbm_stb;
    wire        wbm_we;
    wire [3:0]  wbm_sel;
    wire [31:0] wbm_adr;     // byte address (Microwatt uses byte addressing)
    wire [31:0] wbm_dat_w;
    wire [31:0] wbm_dat_r;
    wire        wbm_ack;
    wire        wbm_err;

    // =========================================================================
    // SLAVE SELECTS / ADDRESS MAP
    //   0x0000_0000 - 0x0000_0FFF : boot ROM (4 KiB)
    //   0x0000_2000 - 0x0000_9FFF : SYS SRAM 32 KiB (CF_SRAM_4096x32)
    //   0x1000_0000 - 0x1000_0FFF : UART (wbuart32)
    //   0x1000_1000 - 0x1000_1FFF : GPIO-out
    //   0x1000_2000 - 0x1000_2FFF : SPI (optional)
    //   0x1000_3000 - 0x1000_3FFF : TIMER (optional)
    //   0x1000_4000 - 0x1000_4FFF : PIC
    //   0x3000_0000 - 0x3000_0FFF : AXI-Lite (accelerator control)
    // =========================================================================
    localparam ROM_BASE   = 32'h0000_0000, ROM_MASK   = 32'hFFFF_F000;
    localparam RAM_BASE   = 32'h0000_2000, RAM_MASK   = 32'hFFFF_E000; // 32 KiB @ 0x2000
    localparam UART_BASE  = 32'h1000_0000, UART_MASK  = 32'hFFFF_F000;
    localparam GPIO_BASE  = 32'h1000_1000, GPIO_MASK  = 32'hFFFF_F000;
    localparam SPI_BASE   = 32'h1000_2000, SPI_MASK   = 32'hFFFF_F000;
    localparam TMR_BASE   = 32'h1000_3000, TMR_MASK   = 32'hFFFF_F000;
    localparam PIC_BASE   = 32'h1000_4000, PIC_MASK   = 32'hFFFF_F000;
    localparam AXIL_BASE  = 32'h3000_0000, AXIL_MASK  = 32'hFFFF_F000;

    wire sel_rom  = ((wbm_adr & ROM_MASK)  == ROM_BASE);
    wire sel_ram  = ((wbm_adr & RAM_MASK)  == RAM_BASE);
    wire sel_uart = ((wbm_adr & UART_MASK) == UART_BASE);
    wire sel_gpio = ((wbm_adr & GPIO_MASK) == GPIO_BASE);
    wire sel_spi  = ((wbm_adr & SPI_MASK)  == SPI_BASE);
    wire sel_tmr  = ((wbm_adr & TMR_MASK)  == TMR_BASE);
    wire sel_pic  = ((wbm_adr & PIC_MASK)  == PIC_BASE);
    wire sel_axil = ((wbm_adr & AXIL_MASK) == AXIL_BASE);

    // =========================================================================
    // BOOT ROM (simple WB read-only ROM; hex file optional)
    // =========================================================================
    wire [31:0] rom_dat_r;
    wire        rom_ack;

    wb_rom #(.AW(10), .INIT_HEX("")) u_boot_rom ( // 4 KiB
        .i_clk (clk), .i_rst (rst),
        .i_cyc (wbm_cyc & sel_rom),
        .i_stb (wbm_stb & sel_rom),
        .i_we  (1'b0),
        .i_sel (wbm_sel),
        .i_adr ( (wbm_adr - ROM_BASE) [11:2] ), // word index
        .i_dat (wbm_dat_w),
        .o_dat (rom_dat_r),
        .o_ack (rom_ack)
    );

    // =========================================================================
    // SYSTEM SRAM (CF_SRAM_4096x32) + WB controller
    // =========================================================================
    wire [31:0] ram_dat_r;
    wire        ram_ack;

    wb_cf_sram32 #(
        .BASE_ADDR(RAM_BASE)
    ) u_sys_sram (
        .i_clk (clk),
        .i_rst (rst),

        // Wishbone side
        .i_wb_cyc (wbm_cyc & sel_ram),
        .i_wb_stb (wbm_stb & sel_ram),
        .i_wb_we  (wbm_we),
        .i_wb_sel (wbm_sel),
        .i_wb_adr (wbm_adr),
        .i_wb_dat (wbm_dat_w),
        .o_wb_dat (ram_dat_r),
        .o_wb_ack (ram_ack),

        // SRAM macro pins
`ifdef USE_POWER_PINS
        .vgnd   (vssd1), .vnb(1'b0), .vpb(1'b1), .vpwra(vccd1), .vpwrm(vccd1), .vpwrp(vccd1),
`endif
        .vpwrac (1'b1),
        .vpwrpc (1'b1)
    );

    // =========================================================================
    // GPIO-out (WB) -> drives 11 user IO bits (DATA & OEB regs)
    // =========================================================================
    wire [31:0] gpio_dat_r;
    wire        gpio_ack;
    wire [10:0] gpio_data;
    wire [10:0] gpio_oeb_reg;

    wb_gpio_out #(.DW(11)) u_gpio (
        .i_clk (clk), .i_rst (rst),
        .i_cyc (wbm_cyc & sel_gpio),
        .i_stb (wbm_stb & sel_gpio),
        .i_we  (wbm_we),
        .i_sel (wbm_sel),
        .i_adr ( (wbm_adr - GPIO_BASE) [3:2] ),
        .i_dat (wbm_dat_w),
        .o_dat (gpio_dat_r),
        .o_ack (gpio_ack),
        .o_data(gpio_data),
        .o_oeb (gpio_oeb_reg)
    );

    assign io_out = gpio_data;
    assign io_oeb = gpio_oeb_reg;

    // =========================================================================
    // UART (wbuart32) — console + boot channel
    // =========================================================================
    wire [31:0] uart_dat_r;
    wire        uart_ack;
    wire        uart_stall;

    wbuart32 u_uart0 (
        .i_clk      (clk),
        .i_reset    (rst),
        .i_wb_cyc   (wbm_cyc & sel_uart),
        .i_wb_stb   (wbm_stb & sel_uart),
        .i_wb_we    (wbm_we),
        .i_wb_addr  ( (wbm_adr - UART_BASE) [3:2] ),
        .i_wb_data  (wbm_dat_w),
        .i_wb_sel   (wbm_sel),
        .o_wb_ack   (uart_ack),
        .o_wb_stall (uart_stall),
        .o_wb_data  (uart_dat_r),
        .i_uart_rx  (uart_rx),
        .o_uart_tx  (uart_tx)
        // tie-offs/defaults for CTS/RTS etc can be added when needed
    );

    // =========================================================================
    // SPI (optional)
    // =========================================================================
`ifdef INCLUDE_SPI
    wire [31:0] spi_dat_r;
    wire        spi_ack;
    wire        spi_irq;

    wb_spi_simple u_spi0 (
        .i_clk (clk), .i_rst (rst),
        .i_cyc (wbm_cyc & sel_spi), .i_stb (wbm_stb & sel_spi),
        .i_we  (wbm_we), .i_sel (wbm_sel),
        .i_adr ( (wbm_adr - SPI_BASE) [3:2] ),
        .i_dat (wbm_dat_w),
        .o_dat (spi_dat_r),
        .o_ack (spi_ack),
        .irq_o (spi_irq),
        // pads later via padmux if you expose pins
        .mosi (), .miso (1'b0), .sck (), .csn ()
    );
`else
    wire [31:0] spi_dat_r = 32'h0; wire spi_ack = 1'b0; wire spi_irq = 1'b0;
`endif

    // =========================================================================
    // TIMER (optional)
    // =========================================================================
`ifdef INCLUDE_TIMER
    wire [31:0] tmr_dat_r;
    wire        tmr_ack;
    wire        tmr_irq;

    wb_timer_simple u_timer0 (
        .i_clk (clk), .i_rst (rst),
        .i_cyc (wbm_cyc & sel_tmr), .i_stb (wbm_stb & sel_tmr),
        .i_we  (wbm_we), .i_sel (wbm_sel),
        .i_adr ( (wbm_adr - TMR_BASE) [3:2] ),
        .i_dat (wbm_dat_w),
        .o_dat (tmr_dat_r),
        .o_ack (tmr_ack),
        .irq_o (tmr_irq)
    );
`else
    wire [31:0] tmr_dat_r = 32'h0; wire tmr_ack = 1'b0; wire tmr_irq = 1'b0;
`endif

    // =========================================================================
    // PIC (programmable interrupt controller)
    // =========================================================================
`ifdef INCLUDE_PIC
    wire [31:0] pic_dat_r;
    wire        pic_ack;
    wire        irq_pic;

    // Map a few sources now; add accelerator_irq later
    wire [7:0] irq_sources = {5'b0, spi_irq, tmr_irq, uart_stall /* or uart irq if available */};

    simple_pic_wrapper u_pic (
        .i_clk (clk), .i_rst (rst),
        .i_cyc (wbm_cyc & sel_pic), .i_stb (wbm_stb & sel_pic),
        .i_we  (wbm_we), .i_sel (wbm_sel),
        .i_adr ( (wbm_adr - PIC_BASE) [4:2] ),
        .i_dat (wbm_dat_w),
        .o_dat (pic_dat_r),
        .o_ack (pic_ack),
        .irq_o (irq_pic),
        .irqs_i(irq_sources)
    );
`else
    wire [31:0] pic_dat_r = 32'h0; wire pic_ack = 1'b0; wire irq_pic = 1'b0;
`endif

    // =========================================================================
    // WB -> AXI-Lite bridge (accelerator control plane @ 0x3000_0000)
    // =========================================================================
    // AXI-Lite master wires out of the bridge
    wire [31:0] m_axil_awaddr, m_axil_wdata, m_axil_araddr;
    wire [3:0]  m_axil_wstrb;
    wire        m_axil_awvalid, m_axil_awready;
    wire        m_axil_wvalid,  m_axil_wready;
    wire [1:0]  m_axil_bresp;   wire m_axil_bvalid, m_axil_bready;
    wire        m_axil_arvalid, m_axil_arready;
    wire [31:0] m_axil_rdata;   wire [1:0] m_axil_rresp; wire m_axil_rvalid, m_axil_rready;

    // WB slave appearance of the AXI-Lite window at 0x3000_0000
    wire [31:0] axil_dat_r;
    wire        axil_ack;

    wb2axil_bridge u_wb2axil (
        .i_clk   (clk),
        .i_rst   (rst),
        // WB side
        .i_wb_cyc (wbm_cyc & sel_axil),
        .i_wb_stb (wbm_stb & sel_axil),
        .i_wb_we  (wbm_we),
        .i_wb_sel (wbm_sel),
        .i_wb_adr (wbm_adr),
        .i_wb_dat (wbm_dat_w),
        .o_wb_dat (axil_dat_r),
        .o_wb_ack (axil_ack),
        .o_wb_err (), // unused

        // AXI-Lite master side
        .m_axil_awaddr (m_axil_awaddr),
        .m_axil_awvalid(m_axil_awvalid),
        .m_axil_awready(m_axil_awready),
        .m_axil_wdata  (m_axil_wdata),
        .m_axil_wstrb  (m_axil_wstrb),
        .m_axil_wvalid (m_axil_wvalid),
        .m_axil_wready (m_axil_wready),
        .m_axil_bresp  (m_axil_bresp),
        .m_axil_bvalid (m_axil_bvalid),
        .m_axil_bready (m_axil_bready),
        .m_axil_araddr (m_axil_araddr),
        .m_axil_arvalid(m_axil_arvalid),
        .m_axil_arready(m_axil_arready),
        .m_axil_rdata  (m_axil_rdata),
        .m_axil_rresp  (m_axil_rresp),
        .m_axil_rvalid (m_axil_rvalid),
        .m_axil_rready (m_axil_rready)
    );

    // =========================================================================
    // ACCELERATOR TOP (AXI-Lite control; irq)
    // (accel_top may internally instantiate its SRAM scratchpads W/X/Y)
    // =========================================================================
    wire accel_irq;

    accel_top u_accel (
        .clk    (clk),
        .rst    (rst),
        // AXI-Lite slave
        .s_axil_awaddr (m_axil_awaddr),
        .s_axil_awvalid(m_axil_awvalid),
        .s_axil_awready(m_axil_awready),
        .s_axil_wdata  (m_axil_wdata),
        .s_axil_wstrb  (m_axil_wstrb),
        .s_axil_wvalid (m_axil_wvalid),
        .s_axil_wready (m_axil_wready),
        .s_axil_bresp  (m_axil_bresp),
        .s_axil_bvalid (m_axil_bvalid),
        .s_axil_bready (m_axil_bready),
        .s_axil_araddr (m_axil_araddr),
        .s_axil_arvalid(m_axil_arvalid),
        .s_axil_arready(m_axil_arready),
        .s_axil_rdata  (m_axil_rdata),
        .s_axil_rresp  (m_axil_rresp),
        .s_axil_rvalid (m_axil_rvalid),
        .s_axil_rready (m_axil_rready),

        .irq_o  (accel_irq)
        // add AXI/AXIS data ports later if/when you enable them
    );

    // =========================================================================
    // WISHBONE MERGE
    // =========================================================================
    reg  [31:0] wb_dat_r_mux;
    reg         wb_ack_mux;
    reg         wb_err_mux;

    always @(*) begin
        wb_dat_r_mux = 32'h0;
        wb_ack_mux   = 1'b0;
        wb_err_mux   = 1'b0;

        if (sel_rom)  begin wb_dat_r_mux = rom_dat_r;  wb_ack_mux = rom_ack;  end
        if (sel_ram)  begin wb_dat_r_mux = ram_dat_r;  wb_ack_mux = ram_ack;  end
        if (sel_uart) begin wb_dat_r_mux = uart_dat_r; wb_ack_mux = uart_ack; end
        if (sel_gpio) begin wb_dat_r_mux = gpio_dat_r; wb_ack_mux = gpio_ack; end
        if (sel_spi)  begin wb_dat_r_mux = spi_dat_r;  wb_ack_mux = spi_ack;  end
        if (sel_tmr)  begin wb_dat_r_mux = tmr_dat_r;  wb_ack_mux = tmr_ack;  end
        if (sel_pic)  begin wb_dat_r_mux = pic_dat_r;  wb_ack_mux = pic_ack;  end
        if (sel_axil) begin wb_dat_r_mux = axil_dat_r; wb_ack_mux = axil_ack; end

        // Optionally set wb_err_mux on unmapped addresses
        // if (wbm_cyc & wbm_stb & ~(sel_rom|sel_ram|sel_uart|sel_gpio|sel_spi|sel_tmr|sel_pic|sel_axil))
        //     wb_err_mux = 1'b1;
    end

    assign wbm_dat_r = wb_dat_r_mux;
    assign wbm_ack   = wb_ack_mux;
    assign wbm_err   = wb_err_mux;

    // =========================================================================
    // MICROWATT CORE (VHDL entity: core) — Wishbone master + IRQ
    // =========================================================================
    wire cpu_irq = irq_pic | accel_irq; // add others as desired

    core u_cpu (
        .clk        (clk),
        .rst        (rst),

        // Microwatt typically has byte-addressed WB; pass full address bus
        .wbm_cyc_o  (wbm_cyc),
        .wbm_stb_o  (wbm_stb),
        .wbm_we_o   (wbm_we),
        .wbm_sel_o  (wbm_sel),
        .wbm_adr_o  (wbm_adr),
        .wbm_dat_o  (wbm_dat_w),
        .wbm_dat_i  (wbm_dat_r),
        .wbm_ack_i  (wbm_ack),
        .wbm_err_i  (wbm_err),

        .ext_irq    (cpu_irq)

        // add/route any other Microwatt ports you enable (icache/dcache opts, etc.)
    );

endmodule

// ============================================================================
// Wishbone ROM (read-only), word-addressed (depth = 2^AW)
// ============================================================================
module wb_rom #(parameter AW=10, parameter INIT_HEX="")(
    input  wire         i_clk,
    input  wire         i_rst,
    input  wire         i_cyc,
    input  wire         i_stb,
    input  wire         i_we,
    input  wire [3:0]   i_sel,
    input  wire [AW-1:0]i_adr,
    input  wire [31:0]  i_dat,
    output reg  [31:0]  o_dat,
    output reg          o_ack
);
    reg [31:0] mem [0:(1<<AW)-1];
    initial if (INIT_HEX != "") $readmemh(INIT_HEX, mem);

    always @(posedge i_clk) begin
        o_ack <= 1'b0;
        if (i_cyc && i_stb && !i_we) begin
            o_dat <= mem[i_adr];
            o_ack <= 1'b1;
        end
    end
endmodule

// ============================================================================
// Wishbone controller for CF_SRAM_4096x32 (single-port, synchronous)
//   - Maps 32 KiB window at BASE_ADDR
//   - 32-bit data, byte enables via BEN
//   - 1-cycle latency acknowledge
// ============================================================================
module wb_cf_sram32 #(
    parameter [31:0] BASE_ADDR = 32'h0000_2000
)(
    input  wire        i_clk,
    input  wire        i_rst,

    // Wishbone
    input  wire        i_wb_cyc,
    input  wire        i_wb_stb,
    input  wire        i_wb_we,
    input  wire [3:0]  i_wb_sel,
    input  wire [31:0] i_wb_adr,
    input  wire [31:0] i_wb_dat,
    output reg  [31:0] o_wb_dat,
    output reg         o_wb_ack,

`ifdef USE_POWER_PINS
    input  wire        vgnd,
    input  wire        vnb,
    input  wire        vpb,
    input  wire        vpwra,
    input  wire        vpwrm,
    input  wire        vpwrp,
`endif
    input  wire        vpwrac,
    input  wire        vpwrpc
);

    // Local address within the 32 KiB window (word index)
    wire [11:0] word_addr = (i_wb_adr - BASE_ADDR) [13:2];

    // SRAM pins
    wire [31:0] sram_do;
    wire        sram_scan_cc;

    // Byte enables to BEN
    wire [31:0] ben = { {8{i_wb_sel[3]}},
                        {8{i_wb_sel[2]}},
                        {8{i_wb_sel[1]}},
                        {8{i_wb_sel[0]}} };

    // Registered ack for single-cycle latency
    reg pending;
    always @(posedge i_clk) begin
        if (i_rst) begin
            o_wb_ack <= 1'b0;
            pending  <= 1'b0;
        end else begin
            o_wb_ack <= 1'b0;
            if (i_wb_cyc && i_wb_stb && !pending) begin
                // Issue access this cycle; ack next
                pending  <= 1'b1;
            end else if (pending) begin
                o_wb_ack <= 1'b1;
                pending  <= 1'b0;
            end
        end
    end

    // Capture read data
    always @(posedge i_clk) begin
        if (i_wb_cyc && i_wb_stb && !i_wb_we) begin
            // Data available next cycle; latch on ack
            o_wb_dat <= sram_do;
        end
    end

    // Drive macro
    CF_SRAM_4096x32 u_cf_sram (
        .DO       (sram_do),
        .ScanOutCC(sram_scan_cc),
        .DI       (i_wb_dat),
        .BEN      (ben),
        .AD       (word_addr),        // 12-bit word address
        .EN       (i_wb_cyc & i_wb_stb),
        .R_WB     (~i_wb_we),         // 1 = read, 0 = write
        .CLKin    (i_clk),
        .WLBI     (1'b0),
        .WLOFF    (1'b0),
        .TM       (1'b0),
        .SM       (1'b0),
        .ScanInCC (1'b0),
        .ScanInDL (1'b0),
        .ScanInDR (1'b0),
        .vpwrac   (vpwrac),
        .vpwrpc   (vpwrpc)
`ifdef USE_POWER_PINS
       ,.vgnd     (vgnd),
        .vnb      (vnb),
        .vpb      (vpb),
        .vpwra    (vpwra),
        .vpwrm    (vpwrm),
        .vpwrp    (vpwrp)
`endif
    );

endmodule

// ============================================================================
// Minimal GPIO-out Wishbone slave: two 32-bit regs @ 0x0 (DATA), 0x4 (OEB)
// ============================================================================
module wb_gpio_out #(parameter integer DW=11)(
    input  wire        i_clk,
    input  wire        i_rst,
    input  wire        i_cyc,
    input  wire        i_stb,
    input  wire        i_we,
    input  wire [3:0]  i_sel,
    input  wire [1:0]  i_adr,       // 0=data, 1=oeb
    input  wire [31:0] i_dat,
    output reg  [31:0] o_dat,
    output reg         o_ack,
    output reg  [DW-1:0] o_data,
    output reg  [DW-1:0] o_oeb
);
    wire access = i_cyc & i_stb;

    always @(posedge i_clk) begin
        if (i_rst) begin
            o_data <= {DW{1'b0}};
            o_oeb  <= {DW{1'b0}};   // 0 = drive, 1 = Hi-Z
            o_ack  <= 1'b0;
            o_dat  <= 32'h0;
        end else begin
            o_ack <= 1'b0;
            if (access) begin
                if (i_we) begin
                    case (i_adr)
                        2'd0: begin
                            if (i_sel[0]) o_data <= i_dat[DW-1:0];
                        end
                        2'd1: begin
                            if (i_sel[0]) o_oeb  <= i_dat[DW-1:0];
                        end
                        default: ;
                    endcase
                end
                case (i_adr)
                    2'd0: o_dat <= {{(32-DW){1'b0}}, o_data};
                    2'd1: o_dat <= {{(32-DW){1'b0}}, o_oeb};
                    default: o_dat <= 32'h0;
                endcase
                o_ack <= 1'b1;
            end
        end
    end
endmodule

// ============================================================================
// PIC wrapper placeholder (map to your imported OpenCores simple_pic ports)
// For now this compiles and behaves as a small WB slave with a combined IRQ.
// Replace with the exact RTL when you import it.
// ============================================================================
module simple_pic_wrapper (
    input  wire        i_clk,
    input  wire        i_rst,
    input  wire        i_cyc,
    input  wire        i_stb,
    input  wire        i_we,
    input  wire [3:0]  i_sel,
    input  wire [2:0]  i_adr,
    input  wire [31:0] i_dat,
    output wire [31:0] o_dat,
    output wire        o_ack,
    output wire        irq_o,
    input  wire [7:0]  irqs_i
);
    // Trivial W1C status + mask (2 regs) to emulate a minimal PIC behavior
    reg [7:0] mask, status;
    assign o_dat = (i_adr == 3'd0) ? {24'h0, mask} :
                   (i_adr == 3'd1) ? {24'h0, status} : 32'h0;
    assign o_ack = i_cyc & i_stb;

    always @(posedge i_clk) begin
        if (i_rst) begin
            mask   <= 8'h00;
            status <= 8'h00;
        end else begin
            // latch level IRQS
            status <= status | (irqs_i & mask);
            if (i_cyc & i_stb & i_we) begin
                case (i_adr)
                    3'd0: if (i_sel[0]) mask   <= i_dat[7:0];
                    3'd1: if (i_sel[0]) status <= status & ~i_dat[7:0]; // W1C
                    default: ;
                endcase
            end
        end
    end
    assign irq_o = |status;
endmodule

// ============================================================================
// WB->AXI-Lite bridge shim (name/ports aligned with common open cores)
// Replace with your chosen bridge IP (e.g., ZipCPU wb2axip axil variant).
// ============================================================================
module wb2axil_bridge (
    input  wire        i_clk,
    input  wire        i_rst,
    // WB
    input  wire        i_wb_cyc,
    input  wire        i_wb_stb,
    input  wire        i_wb_we,
    input  wire [3:0]  i_wb_sel,
    input  wire [31:0] i_wb_adr,
    input  wire [31:0] i_wb_dat,
    output wire [31:0] o_wb_dat,
    output wire        o_wb_ack,
    output wire        o_wb_err,
    // AXI-Lite
    output wire [31:0] m_axil_awaddr,
    output wire        m_axil_awvalid,
    input  wire        m_axil_awready,
    output wire [31:0] m_axil_wdata,
    output wire [3:0]  m_axil_wstrb,
    output wire        m_axil_wvalid,
    input  wire        m_axil_wready,
    input  wire [1:0]  m_axil_bresp,
    input  wire        m_axil_bvalid,
    output wire        m_axil_bready,
    output wire [31:0] m_axil_araddr,
    output wire        m_axil_arvalid,
    input  wire        m_axil_arready,
    input  wire [31:0] m_axil_rdata,
    input  wire [1:0]  m_axil_rresp,
    input  wire        m_axil_rvalid,
    output wire        m_axil_rready
);
    // Minimal pass-through model: single outstanding; WB acks when AXI completes.
    // Replace with a proven core for tapeout.
    assign m_axil_awaddr  = i_wb_adr;
    assign m_axil_awvalid = i_wb_cyc & i_wb_stb & i_wb_we;
    assign m_axil_wdata   = i_wb_dat;
    assign m_axil_wstrb   = i_wb_sel;
    assign m_axil_wvalid  = i_wb_cyc & i_wb_stb & i_wb_we;
    assign m_axil_bready  = 1'b1;

    assign m_axil_araddr  = i_wb_adr;
    assign m_axil_arvalid = i_wb_cyc & i_wb_stb & ~i_wb_we;
    assign m_axil_rready  = 1'b1;

    assign o_wb_dat = m_axil_rdata;
    assign o_wb_ack = (i_wb_we ? (m_axil_awready & m_axil_wready & m_axil_bvalid)
                               :  m_axil_rvalid);
    assign o_wb_err = (i_wb_we ? (m_axil_bvalid & (|m_axil_bresp))
                               :  (m_axil_rvalid & (|m_axil_rresp)));
endmodule

`default_nettype wire
