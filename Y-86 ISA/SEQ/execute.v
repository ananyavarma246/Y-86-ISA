
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

module execute(clk,icode,ifun,valA,valB,valC,valE,Z_F,O_F,S_F,cnd);

input [3:0]icode;
input [3:0]ifun;
input clk;

input [63:0]valA;
input [63:0]valB;
input [63:0]valC;

wire [63:0]add_op_out;
wire [63:0]sub_op_out;
wire [63:0]xor_op_out;
wire [63:0]and_op_out;

wire Overflow_add;
wire Overflow_sub;
wire C_Add;
wire C_Sub;

Add A(valA,valB,add_op_out,C_Add,Overflow_add);
Sub B(valA,valB,sub_op_out,C_Sub,Overflow_sub);

Xor C(valA,valB,xor_op_out);
And D(valA,valB,and_op_out);

output reg [63:0]valE;
output reg Z_F;
output reg O_F;
output reg S_F;
output reg cnd=0;

wire xor_1;
wire xor_2;
wire out;

xor E(xor_1,valA[63],valB[63]);
xor F(xor_2,valA[63],valE[63]);

xor G(out,O_F,S_F);


always @(*)
begin
  if(clk==1)
  begin

  if(icode==4'b0110) //OPq
  begin
    O_F=0;
    S_F=0;
    Z_F=0;
    case(ifun)
  4'b0000: // Add
    begin
      valE = add_op_out;
      O_F = (xor_1 == 0 && xor_2 == 1);
      S_F = (valE[63] == 1);
      Z_F = (valE[63:0] == 0);
    end

  4'b0001: // Sub
    begin
      valE = sub_op_out;
      O_F = (xor_1 == 1 && xor_2 == 1);
      S_F = (valE[63] == 1);
      Z_F = (valE[63:0] == 0);
    end

  4'b0010: // And
    begin
      valE = and_op_out;
      S_F = (valE[63] == 1);
      Z_F = (valE[63:0] == 0);
    end

  4'b0011: // Xor
    begin
      valE = xor_op_out;
      S_F = (valE[63] == 1);
      Z_F = (valE[63:0] == 0);
    end
endcase

  end


  else if(icode==4'b0010) //cmoveX
  begin
      case(ifun)
  4'b0000: // rrmovq
    begin
      cnd = 1;
    end

  4'b0001: // cmovle
    begin
      if (out == 1 || Z_F == 1) cnd = 1;
    end

  4'b0010: // cmovl
    begin
      if (out == 1) cnd = 1;
    end

  4'b0011: // cmove
    begin
      if (Z_F == 1) cnd = 1;
    end

  4'b0100: // cmovne
    begin
      if (Z_F == 0) cnd = 1;
    end

  4'b0101: // cmovge
    begin
      if (out == 0) cnd = 1;
    end

  4'b0110: // cmovg
    begin
      if (out == 0 || Z_F == 0) cnd = 1;
    end

  default: // Default case (if ifun doesn't match any known values)
    valE = valA;
endcase

  end


  else if(icode==4'b0111) //jxx
  begin
      if(ifun==4'b0000) cnd=1; //jump
      else if(ifun==4'b0001) //jle
      begin
          if(out==1 || Z_F==1) cnd=1;
      end
      else if(ifun==4'b0010) //jl
      begin
        if(out==1) cnd=1;
      end
      else if(ifun==4'b0011) //je
      begin
        if(Z_F==1) cnd=1;
      end 
      else if(ifun==4'b0100) //jne
      begin
        if(Z_F==0) cnd=1;
      end
      else if(ifun==4'b0101) //jge
      begin
          if(out==0) cnd=1;
      end
      else if(ifun==4'b0110) //jg
      begin
          if(out==0 || Z_F==0) cnd=1;
      end
  end
  else if(icode == 4'd8) 
  begin
  valE = valB +(-64'd8); // call instruction
  end

  else if (icode == 4'd9) 
  begin
    valE = valB + 64'd8; // ret instruction
  end

  else if (icode == 4'd10) 
  begin
    valE = valB +(-64'd8); // pushq instruction
  end

  else if (icode == 4'd11) 
  begin
    valE = valB + 64'd8; // popq instruction
  end

  else if (icode == 4'd3) 
  begin
    valE = 64'd0 + valC; // irmovq instruction
  end

  else if (icode == 4'd4) 
  begin
    valE = valB + valC; // rmmovq instruction
  end
            
  else if (icode == 4'd5) 
  begin
    valE = valB + valC; // mrmovq instruction
  end

end
end 
endmodule