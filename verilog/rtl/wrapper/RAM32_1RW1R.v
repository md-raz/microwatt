module RAM32_1RW1R #(
    parameter BITS=5
) (
`ifdef USE_POWER_PINS
    inout VPWR,
    inout VGND,
`endif
    input CLK,

    input EN0,
    input [BITS-1:0] A0,
    input [7:0] WE0,
    input [63:0] Di0,
    output reg [63:0] Do0,

    input EN1,
    input [BITS-1:0] A1,
    output reg [63:0] Do1
);

    // Dual-port RAM for Microwatt cache
    // Port 0: Read/Write, Port 1: Read-only
    reg [63:0] mem [0:(1<<BITS)-1];

    // Port 0: Read/Write with byte enables
    always @(posedge CLK) begin
        if (EN0) begin
            if (WE0[0]) mem[A0][ 7: 0] <= Di0[ 7: 0];
            if (WE0[1]) mem[A0][15: 8] <= Di0[15: 8];
            if (WE0[2]) mem[A0][23:16] <= Di0[23:16];
            if (WE0[3]) mem[A0][31:24] <= Di0[31:24];
            if (WE0[4]) mem[A0][39:32] <= Di0[39:32];
            if (WE0[5]) mem[A0][47:40] <= Di0[47:40];
            if (WE0[6]) mem[A0][55:48] <= Di0[55:48];
            if (WE0[7]) mem[A0][63:56] <= Di0[63:56];
            Do0 <= mem[A0];
        end
    end

    // Port 1: Read-only
    always @(posedge CLK) begin
        if (EN1) begin
            Do1 <= mem[A1];
        end
    end

endmodule
