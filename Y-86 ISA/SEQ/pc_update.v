`timescale 1ns/1ps

module pc_update(
    input clk,
    input cnd,
    input [63:0] PC,
    input [3:0] icode,
    input [63:0] valC,
    input [63:0] valM,
    input [63:0] valP,
    output reg [63:0] PC_updated
);

always @(*)
begin
    case (icode)
        4'b0000, 4'b0001, 4'b0010, 4'b0011, 4'b0100, 4'b0101, 4'b0110:
            PC_updated = valP;

        4'b0111:
            PC_updated = (cnd) ? valC : valP;

        4'b1000:
            PC_updated = valC;

        4'b1001:
            PC_updated = valM;

        4'b1010, 4'b1011:
            PC_updated = valP;
    endcase
end

endmodule