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
