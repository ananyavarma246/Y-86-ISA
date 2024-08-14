
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
    inst_mem[1] = 8'b11110001;  // rA:rB
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

    inst_mem[20] = 8'b01100001;  //opq:addq
    inst_mem[21] = 8'b00010010;  // rcx:rdx

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
