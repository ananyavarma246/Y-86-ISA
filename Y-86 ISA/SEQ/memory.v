`timescale 1ns/1ps

module memory(clk, icode, valA, valB, valE, valP, valM, Mem_output);

    reg [63:0] Mem_Mem_output [0:1023];
    integer i;

    input clk;
    input [3:0] icode;
    input signed [63:0] valA, valB;
    input [63:0] valE, valP;
    output reg [63:0] valM, Mem_output;

    initial begin
        for (i = 0; i < 1024; i = i + 1) begin
            Mem_Mem_output[i] = 64'd2;
        end
    end

    always @(*) begin

        case (icode)
            4'b0100: Mem_Mem_output[valE] = valA;   // rmmovq
            4'b0101: valM = Mem_Mem_output[valE];   // mrmovq
            4'b1000: Mem_Mem_output[valE] = valP;   // call
            4'b1001: valM = Mem_Mem_output[valA];   // ret
            4'b1010: Mem_Mem_output[valE] = valA;   // pushq
            4'b1011: valM = Mem_Mem_output[valE];   // popq
        endcase

        Mem_output = Mem_Mem_output[valE];
    end

endmodule