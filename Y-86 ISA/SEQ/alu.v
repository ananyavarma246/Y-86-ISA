module FullAdderBit(A, B, Cin, Sum, Cout);
    input A, B, Cin;
    output Sum, Cout;

    wire temp, P, Q, R;

    xor G0(temp, A, B);
    xor G1(Sum, temp, Cin);
    and G2(P, A, B);
    and G3(Q, B, Cin);
    and G4(R, A, Cin);
    or G5(Cout, P, Q, R);
endmodule

module Add(A, B, Sum, Cout,carry_overflow);
    input [63:0] A;
    input [63:0] B;
    output [63:0] Sum;
    output Cout,carry_overflow;

    wire [64:0] Carry;

    assign Carry[0] = 1'b0;

    genvar i;
    generate
        for (i = 0; i < 64; i = i + 1) begin
            FullAdderBit adderBit(
                .A(A[i]),
                .B(B[i]),
                .Cin(Carry[i]),
                .Sum(Sum[i]),
                .Cout(Carry[i + 1])
            );
        end
    endgenerate
    
    assign Cout=Carry[64];
    xor g6(carry_overflow,Cout,Carry[63]);


endmodule

module Sub(A, B, Sum, Cout,carry_overflow);
    input [63:0] A;
    input [63:0] B;
    
    output [63:0] Sum;
    output Cout,carry_overflow;

    wire [64:0] Carry;
    wire [63:0] B_com;

    assign Carry[0] = 1'b1;

    genvar j;
    generate
        for (j = 0; j < 64; j = j+ 1) begin
            xor g1(B_com[j],B[j],Carry[0]);
        end
    endgenerate


    genvar i;
    generate
        for (i = 0; i < 64; i = i + 1) begin
            FullAdderBit adderBit(
                .A(A[i]),
                .B(B_com[i]),
                .Cin(Carry[i]),
                .Sum(Sum[i]),
                .Cout(Carry[i + 1])
            );
        end
    endgenerate
    
    assign Cout=Carry[64];
    xor g7(carry_overflow,Cout,Carry[63]);


endmodule

module And(
    input [63:0] A,
    input [63:0] B,
    output [63:0] Y
);

    generate
        genvar i;
        for (i = 0; i < 64; i = i + 1) begin
            and a1(Y[i],A[i],B[i]);
        end
    endgenerate

endmodule

module Xor(
    input [63:0] A,
    input [63:0] B,
    output [63:0] Y
);

    generate
        genvar i;
        for (i = 0; i < 64; i = i + 1) begin
            xor x1(Y[i],A[i],B[i]);
        end
    endgenerate

endmodule

module ALU(
   input [63:0] A, B,
   input [1:0] S,
   output reg [63:0] result,
   output Cout_add, Cout_sub,carry_overflow_add,carry_overflow_sub
);

   wire [63:0] SA, SSub, AndOut, XorOut;
   

   Add adder(A, B,SA, Cout_add,carry_overflow_add);
   Sub subtractor(A, B,SSub, Cout_sub,carry_overflow_sub);
   And and1(A, B, AndOut);
   Xor xor1(A, B, XorOut);

   always @(*) begin
       case (S)
           2'b00: result = SA;  
           2'b01: result = SSub; 
           2'b10: result = AndOut; 
           2'b11: result = XorOut; 
           default: result = 64'bx; 
       endcase
   end

endmodule