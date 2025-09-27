`default_nettype none
module wb_crossbar #(
    parameter NS = 8,                        // number of slaves
    parameter [32*NS-1:0] BASE = {NS{32'h0}},
    parameter [32*NS-1:0] MASK = {NS{32'h0}} // address match: (adr & MASK) == BASE
)(
    input  wire        clk,
    input  wire        rst,
    // Master WB
    input  wire        m_cyc,
    input  wire        m_stb,
    input  wire        m_we,
    input  wire [3:0]  m_sel,
    input  wire [31:0] m_adr,
    input  wire [31:0] m_dat_w,
    output reg  [31:0] m_dat_r,
    output reg         m_ack,
    output reg         m_err,
    // Slave WB (vectors, index 0..NS-1)
    output wire [NS-1:0] s_cyc,
    output wire [NS-1:0] s_stb,
    output wire [NS-1:0] s_we,
    output wire [4*NS-1:0] s_sel,
    output wire [32*NS-1:0] s_adr,
    output wire [32*NS-1:0] s_dat_w,
    input  wire [32*NS-1:0] s_dat_r,
    input  wire [NS-1:0]    s_ack,
    input  wire [NS-1:0]    s_err
);
    // decode
    wire [NS-1:0] hit;
    genvar i;
    for (i=0;i<NS;i=i+1) begin: DEC
        wire [31:0] base_i = BASE[32*i +: 32];
        wire [31:0] mask_i = MASK[32*i +: 32];
        assign hit[i] = ((m_adr & mask_i) == base_i);
    end
    wire any_hit = |hit;

    // fanout to slaves
    for (i=0;i<NS;i=i+1) begin: FAN
        assign s_cyc[i] = m_cyc & hit[i];
        assign s_stb[i] = m_stb & hit[i];
        assign s_we[i]  = m_we;
        assign s_sel[4*i +: 4] = m_sel;
        assign s_adr[32*i +: 32] = m_adr;
        assign s_dat_w[32*i +: 32] = m_dat_w;
    end

    // mux back
    always @(*) begin
        m_dat_r = 32'h0;
        m_ack   = 1'b0;
        m_err   = 1'b0;
        for (int k=0;k<NS;k++) begin
            if (hit[k]) begin
                m_dat_r = s_dat_r[32*k +: 32];
                m_ack   = s_ack[k];
                m_err   = s_err[k];
            end
        end
        // error on unmapped
        if (m_cyc & m_stb & ~any_hit) begin
            m_err = 1'b1;
        end
    end
endmodule
`default_nettype wire
