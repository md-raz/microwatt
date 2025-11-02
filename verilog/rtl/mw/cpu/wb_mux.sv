// SPDX-License-Identifier: Apache-2.0
// wb_fabric_simple.sv
// Single-master -> N-slave Wishbone decoder + return mux.
// Combinational decode/mux; no arbitration (exactly one master).
// Address match rule per slave i: (ADR & MASK[i]) == BASE[i]

`default_nettype none

module wb_fabric #(
    parameter int NS = 2,                        // number of slaves
    // Flattened arrays: BASE[0] at bits [31:0], BASE[1] at [63:32], ...
    parameter logic [32*NS-1:0] BASE = { 32'h0000_2000, 32'h0000_0000 }, // ex: RAM, ROM
    parameter logic [32*NS-1:0] MASK = { 32'hFFFF_E000, 32'hFFFF_F000 }, // ex: 32KiB, 4KiB
    // Behavior when an access doesn't hit any slave
    parameter bit DEFAULT_ERR_ON_MISS = 1'b1,
    // Enable one-hot check in simulation
    parameter bit CHECK_ONE_HOT = 1'b1
) (
    // ---------------- Master (Wishbone B4 interface) ----------------
    input  wire                  m_cyc,     // Bus cycle active
    input  wire                  m_stb,     // Transfer request
    input  wire                  m_we,      // Write enable
    input  wire [3:0]            m_sel,     // Byte select
    input  wire [31:0]           m_adr,     // Address
    input  wire [31:0]           m_dat_w,   // Write data
    output logic [31:0]          m_dat_r,   // Read data
    output logic                 m_ack,     // Acknowledge
    output logic                 m_err,     // Error

    // ---------------- Slaves (vectors) ----------------
    // To slaves
    output wire [NS-1:0]         s_cyc,     // Slave cycle signals
    output wire [NS-1:0]         s_stb,     // Slave strobe signals
    output wire [NS-1:0]         s_we,      // Slave write enables
    output wire [4*NS-1:0]       s_sel,     // Slave byte selects
    output wire [32*NS-1:0]      s_adr,     // Slave addresses
    output wire [32*NS-1:0]      s_dat_w,   // Slave write data
    // From slaves
    input  wire [32*NS-1:0]      s_dat_r,   // Slave read data
    input  wire [NS-1:0]         s_ack,     // Slave acknowledges
    input  wire [NS-1:0]         s_err      // Slave errors
);

    // Parameter validation
    initial begin
        if (NS <= 0) $fatal("wb_fabric: NS must be positive");
    end

    // Decode hits
    wire [NS-1:0] hit;

    genvar i;
    generate
        for (i = 0; i < NS; i++) begin : GEN_DEC
            // Slice base and mask for this slave
            wire [31:0] base_i = BASE[32*i +: 32];
            wire [31:0] mask_i = MASK[32*i +: 32];

            // Address match: (m_adr & mask) == base
            assign hit[i] = ((m_adr & mask_i) == base_i);

            // Drive slave signals when hit
            assign s_cyc[i]   = m_cyc & hit[i];           // Cycle active for selected slave
            assign s_stb[i]   = m_stb & hit[i];           // Strobe active for selected slave
            assign s_we[i]    = m_we;                     // Write enable (same for all)
            assign s_sel[4*i +: 4]     = m_sel;           // Byte select
            assign s_adr[32*i +: 32]   = m_adr;           // Address
            assign s_dat_w[32*i +: 32] = m_dat_w;         // Write data
        end
    endgenerate

    // Any hit?
    wire any_hit = |hit;

    // Return mux (combinational)
    always_comb begin
    m_dat_r = '0; m_ack = 1'b0; m_err = 1'b0;
    if (m_cyc && m_stb) begin
        for (int k = 0; k < NS; k++) begin
        if (hit[k]) begin
            m_dat_r = s_dat_r[32*k +: 32];
            m_ack   = s_ack[k];
            m_err   = s_err[k];
            break; // <- lowest index wins
        end
        end
        if (DEFAULT_ERR_ON_MISS && !any_hit) begin
        m_err = 1'b1; m_ack = 1'b1;
        end
    end
    end


    // Simulation-only one-hot check for address overlaps
    `ifndef SYNTHESIS
    if (CHECK_ONE_HOT) begin : ONE_HOT_CHECK
        always_comb begin
            if (m_cyc && m_stb) begin
                int ones = 0;
                for (int k = 0; k < NS; k++) begin
                    if (hit[k]) ones++;
                end
                if (ones > 1)
                    $warning("wb_fabric: address overlaps detected (more than one slave hit)");
            end
        end
    end
    `endif

endmodule

`default_nettype wire