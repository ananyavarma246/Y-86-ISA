
module fetch(clk,icode,ifun,rA,rB,valC,valP,hlt,inst_valid,mem_error,PC);

input clk;
input [63:0] PC;


output reg hlt;
output reg inst_valid;
output reg mem_error;


output reg [63:0] valP;
output reg [63:0] valC;

output reg [3:0] rA;
output reg [3:0] rB;

output reg [3:0] icode;
output reg [3:0] ifun;

reg [7:0] inst_mem [0:1023];
reg [0:79] inst;

initial begin
    inst_mem[0] = 8'b00110000;  //icode:ifun
    inst_mem[1] = 8'b11110011;  // rA:rB
    inst_mem[2] = 8'b00000000;  //dest
    inst_mem[3] = 8'b00000000;
    inst_mem[4] = 8'b00000000;
    inst_mem[5] = 8'b00000000;
    inst_mem[6] = 8'b00000000;
    inst_mem[7] = 8'b00000000;
    inst_mem[8] = 8'b00000001;
    inst_mem[9] = 8'b00000000;

    inst_mem[10] = 8'b00110000;  //icode:ifun
    inst_mem[11] = 8'b11110010;  // rA:rB
    inst_mem[12] = 8'b00000000;  //dest
    inst_mem[13] = 8'b00000000;
    inst_mem[14] = 8'b00000000;
    inst_mem[15] = 8'b00000000;
    inst_mem[16] = 8'b00000000;
    inst_mem[17] = 8'b00000000;
    inst_mem[18] = 8'b00000010;
    inst_mem[19] = 8'b00000000;

    inst_mem[20] = 8'b01100000;  //opq:addq
    inst_mem[21] = 8'b00100011;  // rcx:rdx

    // inst_mem[20] = 8'b01100001;  //opq:subq
    // inst_mem[21] = 8'b00100011;  // rcx:rdx

    // inst_mem[20] = 8'b01100010;  //opq:andq
    // inst_mem[21] = 8'b00100011;  // rcx:rdx

    // inst_mem[20] = 8'b01100011;  //opq:xorq
    // inst_mem[21] = 8'b00100011;  // rcx:rdx

    // inst_mem[22] = 8'b00010000;  //nop
    // inst_mem[23] = 8'b00010000;  //nop
    // inst_mem[24] = 8'b00000000; // halt


end

  task halt_operation;
begin
  hlt = 1;
  valP = PC + 64'd1;
end
  endtask

task nop_operation;
begin
  valP = PC + 64'd1;
end
endtask

task cmovxx_operation;
begin
  rB = inst[12:15];
  rA = inst[8:11];
  valP = PC + 64'd2;
end
endtask

task irmov_operation;
begin
  valC = inst[16:79];
  valP = PC + 64'd10;
  rB = inst[12:15];
end
endtask



task rmmov_operation;
 begin  
      valC = inst[16:79];
      valP = PC + 64'd10;
      rA = inst[8:11];
      rB = inst[12:15];
    end
endtask

task mrmov_operation;
 begin  
      valC = inst[16:79];
      valP = PC + 64'd10;
      rA = inst[8:11];
      rB = inst[12:15];
    end
endtask

task OPq_operation;
 begin  
      valP = PC + 64'd2;
      rA = inst[8:11];
      rB = inst[12:15];
    end
endtask

task jXX_operation;
 begin   
      valC = inst[8:71];
      valP = PC + 64'd9;
    end
endtask

task call_operation;
 begin  
      valC = inst[8:71];
      valP = PC + 64'd9;
    end
endtask

task ret_operation;
 begin 
      valP = PC + 64'd1;
    end
endtask

task push_operation;
 begin  
      valP = PC + 64'd2;
      rA = inst[8:13];
      rB=inst[12:15];
    end
endtask

task pop_operation;
 begin  
      rA = inst[8:13];
      rB=inst[12:15];
      valP = PC + 64'd2;
    end
endtask

always @ (posedge clk)
begin
    
    mem_error = 0;           // finding if the given instruction is within the  
    if(PC>1023)              // instruction memory or not
    begin
      mem_error = 1;
    end

    inst_valid=1;

    inst={
        inst_mem[PC],
        inst_mem[PC+1],
        inst_mem[PC+2],
        inst_mem[PC+3],
        inst_mem[PC+4],
        inst_mem[PC+5],
        inst_mem[PC+6],
        inst_mem[PC+7],
        inst_mem[PC+8],
        inst_mem[PC+9]
    };

    icode = inst[0:3];
    ifun = inst[4:7];

    case (icode)
        4'b0000: halt_operation;
        4'b0001: nop_operation;
        4'b0010: cmovxx_operation;
        4'b0011: irmov_operation;
        4'b0100: rmmov_operation;
        4'b0101: mrmov_operation;
        4'b0110: OPq_operation;
        4'b0111: jXX_operation;
        4'b1000: call_operation;
        4'b1001: ret_operation;
        4'b1010: push_operation;
        4'b1011: pop_operation;
        default: inst_valid = 0;
    endcase

end

endmodule

module decode(
    input clk,
    input cnd,

    input [3:0] icode,

    input [3:0] rA,
    input [3:0] rB,

    output reg [63:0] valA,
    output reg [63:0] valB,

    input [63:0] valC,
    input [63:0] valE,
    input [63:0] valM,

    output reg [63:0] rax,
    output reg [63:0] rcx,
    output reg [63:0] rdx,
    output reg [63:0] rbx,
    output reg [63:0] rsp,
    output reg [63:0] rbp,
    output reg [63:0] rsi,
    output reg [63:0] rdi,
    output reg [63:0] r8,
    output reg [63:0] r9,
    output reg [63:0] r10,
    output reg [63:0] r11,
    output reg [63:0] r12,
    output reg [63:0] r13,
    output reg [63:0] r14

);

reg [63:0] reg_mem [14:0];

initial begin
  
    reg_mem[0] = rax;
    reg_mem[1] = rcx;
    reg_mem[2] = rdx;
    reg_mem[3] = rbx;
    reg_mem[4] = rsp;
    reg_mem[5] = rbp;
    reg_mem[6] = rsi;
    reg_mem[7] = rdi;
    reg_mem[8] = r8;
    reg_mem[9] = r9;
    reg_mem[10] = r10;
    reg_mem[11] = r11;
    reg_mem[12] = r12;
    reg_mem[13] = r13;
    reg_mem[14] = r14;
end

always @(*) begin
    case (icode)
        4'b0010: begin  // cmove operation
            valA = reg_mem[rA];
        end

        4'b0100: begin  // rmmove operation
            valA = reg_mem[rA];
            valB = reg_mem[rB];
        end

        4'b0101: begin  // mrmove operation
            valB = reg_mem[rB];
        end

        4'b0110: begin  // OPq
            valA = reg_mem[rA];
            valB = reg_mem[rB];
        end

        4'b1000: begin  // call operation
            valB = reg_mem[4];
        end

        4'b1001: begin  // return operation
            valA = reg_mem[4];
            valB = reg_mem[4];
        end

        4'b1010: begin  // push operation
            valA = reg_mem[rA];
            valB = reg_mem[4];
        end

        4'b1011: begin //pop operation
            valA = reg_mem[4];
            valB = reg_mem[4];
        end


    endcase


    rax = reg_mem[0];
    rcx = reg_mem[1];
    rdx = reg_mem[2];
    rbx = reg_mem[3];
    rsp = reg_mem[4];
    rbp = reg_mem[5];
    rsi = reg_mem[6];
    rdi = reg_mem[7];
    r8 = reg_mem[8];
    r9 = reg_mem[9];
    r10 = reg_mem[10];
    r11 = reg_mem[11];
    r12 = reg_mem[12];
    r13 = reg_mem[13];
    r14 = reg_mem[14];
end

always @(negedge clk) begin
    case (icode)
        4'b0010: begin // cmove operation
            if (cnd == 1'b1) reg_mem[rB] = valE;
        end

        4'b0011: begin        // irmovq $0xx rB
            reg_mem[rB] = valE;       // Here the rB register will store the final result
        end

        4'b0101: begin   // mrmovq D(rB) rA
            reg_mem[rA] = valM;     // Here the rA register will store the final result 
        end

        4'b0110: begin  // OPq rA rB
            reg_mem[rB] = valE;     // Here the rB register will store the final result
        end

        4'b1000: begin  // call Dest
            reg_mem[4] = valE;       // Here reg_mem[4] is the %esp(stack pointer) .Update stack pointer
        end

        4'b1001: begin  // ret
            reg_mem[4] = valE;      // reg_mem[4] is the %esp and we update the stack pointer
        end

        4'b1010: begin  // pushq 
            reg_mem[4] = valE;     // In push, we first decrement the address and then push the data. 
        end

        4'b1011: begin  // popq
            reg_mem[4] = valE;   // In this, first we pop out the data from the stack and then increment the address.
            reg_mem[rA] = valM;   // So, the popped-out data is again restored in register rA
        end

        default: begin
            
        end
    endcase

    // Update register values
    rax = reg_mem[0];
    rcx = reg_mem[1];
    rdx = reg_mem[2];
    rbx = reg_mem[3];
    rsp = reg_mem[4];
    rbp = reg_mem[5];
    rsi = reg_mem[6];
    rdi = reg_mem[7];
    r8 = reg_mem[8];
    r9 = reg_mem[9];
    r10 = reg_mem[10];
    r11 = reg_mem[11];
    r12 = reg_mem[12];
    r13 = reg_mem[13];
    r14 = reg_mem[14];
end

endmodule


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

module seq_processor;

reg clk;
reg [63:0]PC;
wire [63:0]PC_updated;
reg [3:0]status; //aop-----hlt-------memory_error----instruction_error

wire [63:0]valA;
wire [63:0]valB;
wire [63:0]valC;
wire [63:0]valP;
wire [63:0]valE;
wire [63:0]valM;
wire [3:0] icode;
wire [3:0] ifun;
wire [3:0] rA;
wire [3:0]rB;
wire cnd;
wire [63:0]test;
wire O_F;
wire S_F;
wire Z_F;
wire hlt;
wire inst_valid;
wire mem_error;
wire [63:0]Mem_output;

wire [63:0] rax;
wire [63:0] rcx;
wire [63:0] rdx;
wire [63:0] rbx;
wire [63:0] rsp;
wire [63:0] rbp;
wire [63:0] rsi;
wire [63:0] rdi;
wire [63:0] r8;
wire [63:0] r9;
wire [63:0] r10;
wire [63:0] r11;
wire [63:0] r12;
wire [63:0] r13;
wire [63:0] r14;

//fetch stage
  fetch A1(
    .clk(clk),
    .PC(PC),
    .icode(icode),
    .ifun(ifun),
    .rA(rA),
    .rB(rB),
    .valC(valC),
    .valP(valP),
    .hlt(hlt)
  );
 
//execute stage
execute uut1(.clk(clk),.icode(icode),.valA(valA),.valC(valC),.valE(valE),.valB(valB),.ifun(ifun),.O_F(O_F),.S_F(S_F),.Z_F(Z_F),.cnd(cnd));

//decode stage
  decode C1(
    .clk(clk),
     .valC(valC),
    .icode(icode),
    .rA(rA),
    .rB(rB),
    .valA(valA),
    .valB(valB),
    .valE(valE),
    .valM(valM),
    .rax(rax),
    .rcx(rcx),
    .rdx(rdx),
    .rbx(rbx),
    .rsp(rsp),
    .rbp(rbp),
    .rsi(rsi),
    .rdi(rdi),
    .r8(r8),
    .r9(r9),
    .r10(r10),
    .r11(r11),
    .r12(r12),
    .r13(r13),
    .r14(r14)
  );

//memory stage
memory D1(.icode(icode),.valM(valM),.valP(valP),.valE(valE),.valA(valA),.valB(valB),.Mem_output(Mem_output));

//pc_update
pc_update E1(.clk(clk),.icode(icode),.valP(valP),.cnd(cnd),.valC(valC),.valM(valM),.PC_updated(PC_updated));

initial begin
  status[0]=1;
  status[1]=0;
  status[2]=0;
  status[3]=1;

  
  clk = 0;

    #10 clk=~clk;PC=64'd0;
    #10 clk=~clk;
    #10 clk=~clk;PC=valP;
    #10 clk=~clk;
    #10 clk=~clk;PC=valP;
    #10 clk=~clk;
    #10 clk=~clk;PC=valP;
    #10 clk=~clk;
    #10 clk=~clk;PC=valP;
    #10 clk=~clk;
    #10 clk=~clk;PC=valP;
    #10 clk=~clk;
    #10 clk=~clk;PC=valP;
    #10 clk=~clk;
    #10 clk=~clk;PC=valP;
    #10 clk=~clk;
    #10 clk=~clk;PC=valP;
    

end

always@(*) begin
  status[1]=hlt;
  status[2]=mem_error;
  status[3]=inst_valid;
    if(status[1] == 1 || status[2]==1 || status[3]==0 || status[0]==0) begin
      $finish;
    end
    else
    begin
    $monitor("clk=%d cnd=%d hlt=%d PC=%d icode=%b ifun=%b rA=%b rB=%b,valC=%d,valA=%d,valB=%d valE=%b,valM=%d\n",clk,cnd,hlt,PC,icode,ifun,rA,rB,valC,valA,valB,valE,valM);
    end
  end
endmodule
