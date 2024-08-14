// `include "alu.v"

module fetch(F_predPC, M_icode, M_Cnd, M_valA, W_icode, W_valM, f_stat, f_icode, f_ifun, f_rA, f_rB, f_valC, f_valP, f_predPC);

    // inputs
    input [63:0] F_predPC,M_valA, W_valM;
    input [3:0] M_icode, W_icode;
    input M_Cnd;


    // outputs
    output reg [3:0] f_stat, f_icode, f_ifun, f_rA, f_rB;

    output reg signed [63:0] f_valC,f_valP, f_predPC;

    // Setting up instruction memory
    reg [7:0] inst_mem[0:1023];


    // Select PC
    reg [63:0] PC;

    initial begin
        // // addition works
    inst_mem[0] = 8'b00110000;  // irmovq
    inst_mem[1] = 8'b11110011;  // rA = 15, rB = 3
    inst_mem[2] = 8'b00000000;
    inst_mem[3] = 8'b00000000;
    inst_mem[4] = 8'b00000000;
    inst_mem[5] = 8'b00000000;
    inst_mem[6] = 8'b00000000;
    inst_mem[7] = 8'b00000000;
    inst_mem[8] = 8'b00000001;
    inst_mem[9] = 8'b00000000;  // valC
    inst_mem[10] = 8'b00110000; // irmovq
    inst_mem[11] = 8'b11110010; // rA = 15, rB = 2
    inst_mem[12] = 8'b00000000;
    inst_mem[13] = 8'b00000000;
    inst_mem[14] = 8'b00000000;
    inst_mem[15] = 8'b00000000;
    inst_mem[16] = 8'b00000000;
    inst_mem[17] = 8'b00000000;
    inst_mem[18] = 8'b00000010;
    inst_mem[19] = 8'b00000000;  // valC 
    inst_mem[20] = 8'b01100011;  // OPq add
    inst_mem[21] = 8'b00100011;  // rA = 2, rB = 3 
    inst_mem[22] = 8'b00000000;  // halt


    end



    always @(*) 
    begin        
        if (M_icode == 7 && M_Cnd == 0) 
            PC <= M_valA;
        else if (W_icode == 9) 
            PC <= W_valM;
        else 
            PC <= F_predPC;
    end

    // Select f_icode, f_ifun, imem_error
    reg imem_error;

    always @(*) 
    begin
        if (PC >= 0 && PC < 4096) begin 
            f_icode    <= inst_mem[PC][7:4];
            f_ifun     <= inst_mem[PC][3:0];
            imem_error <= 0;
        end
        else begin
            f_icode    <= 1;
            f_ifun     <= 0;
            imem_error <= 1;
        end
    end

    // instr_valid flag
    reg instr_valid;
    
    always @(*) 
    begin
        if (f_icode >= 0 && f_icode <= 11) 
            instr_valid <= 1;
        else 
            instr_valid <= 0;
    end

    // Setting rA, rB, valC, valP
    always @(*) begin
        if (f_icode == 4'h2) begin // cmov
            f_rA   = inst_mem[PC + 1][7:4];
            f_rB   = inst_mem[PC + 1][3:0];
            f_valC = 0;
            f_valP = PC + 2;
        end
        else if (f_icode == 4'h3) begin // irmovq
            f_rA   = inst_mem[PC + 1][7:4];
            f_rB   = inst_mem[PC + 1][3:0];
            f_valC = {inst_mem[PC+2],inst_mem[PC+3],inst_mem[PC+4],inst_mem[PC+5],inst_mem[PC+6],inst_mem[PC+7],inst_mem[PC+8],inst_mem[PC+9]};
            f_valP = PC + 10;
        end
        else if (f_icode == 4'h4) begin // rmmovq
            f_rA   = inst_mem[PC + 1][7:4];
            f_rB   = inst_mem[PC + 1][3:0];
            f_valC = {inst_mem[PC+2],inst_mem[PC+3],inst_mem[PC+4],inst_mem[PC+5],inst_mem[PC+6],inst_mem[PC+7],inst_mem[PC+8],inst_mem[PC+9]};
            f_valP = PC + 10;
        end
        else if (f_icode == 4'h5) begin // mrmovq
            f_rA   = inst_mem[PC + 1][7:4];
            f_rB   = inst_mem[PC + 1][3:0];
            f_valC = {inst_mem[PC+2],inst_mem[PC+3],inst_mem[PC+4],inst_mem[PC+5],inst_mem[PC+6],inst_mem[PC+7],inst_mem[PC+8],inst_mem[PC+9]};
            f_valP = PC + 10;
        end
        else if (f_icode == 4'h6) begin // OPq
            f_rA   = inst_mem[PC + 1][7:4];
            f_rB   = inst_mem[PC + 1][3:0];
            f_valC = 0;
            f_valP = PC + 2;
        end
        else if (f_icode == 4'h7) begin // jxx
            f_valC = {inst_mem[PC+1],inst_mem[PC+2],inst_mem[PC+3],inst_mem[PC+4],inst_mem[PC+5],inst_mem[PC+6],inst_mem[PC+7],inst_mem[PC+8]};
            f_valP = PC + 9;
        end
        else if (f_icode == 4'h8) begin // call
            f_valC = {inst_mem[PC+1],inst_mem[PC+2],inst_mem[PC+3],inst_mem[PC+4],inst_mem[PC+5],inst_mem[PC+6],inst_mem[PC+7],inst_mem[PC+8]};
            f_valP = PC + 9;
        end
        else if (f_icode == 4'h9) begin // ret
            f_valP = PC + 1; 
        end
        else if (f_icode == 4'hA) begin // pushq
            f_rA   = inst_mem[PC + 1][7:4];
            f_rB   = inst_mem[PC + 1][3:0];
            f_valC = 0;
            f_valP = PC + 2;
        end
        else if (f_icode == 4'hB) begin // popq
            f_rA   = inst_mem[PC + 1][7:4];
            f_rB   = inst_mem[PC + 1][3:0];
            f_valC = 0;
            f_valP = PC + 2;
        end
        else begin
            f_rA   = 15;
            f_rB   = 15;
            f_valP = PC + 1;
        end
    end

    // Predicting the PC
    always @(*) begin 
        if (f_icode == 4'h7)
            f_predPC = f_valC;
        else if (f_icode == 4'h8)
            f_predPC = f_valC; 
        else
            f_predPC = f_valP;
    end

    // Generating Stat Codes
    always @(*) begin
        if (imem_error)
            f_stat = 4'b0010;
        else if (!instr_valid)
            f_stat = 4'b0001;
        else if (f_icode == 4'h0)
            f_stat = 4'b0100;
        else
            f_stat = 4'b1000;
    end

endmodule

module decode (
    input clk,

    // Inputs from D register
    input [3:0] D_stat,D_icode, D_ifun,D_rA, D_rB,
    input signed [63:0] D_valC,
    input [63:0] D_valP,

    // Inputs forwarded from execute stage
    input [3:0] e_dstE,
    input signed [63:0] e_valE,

    // Inputs forwarded from M register and memory stage
    input [3:0] M_dstE, M_dstM,
    input signed [63:0] M_valE, m_valM,

    // Inputs forwarded from W register
    input [3:0] W_dstM, W_dstE,
    input signed [63:0] W_valM, W_valE,

    // Outputs
    output reg[3:0] d_stat,d_icode, d_ifun,
    output reg signed[63:0] d_valA, d_valB, d_valC,
    output reg[3:0] d_dstE, d_dstM,d_srcA, d_srcB,

    output reg signed [63:0] reg_file0,reg_file1,reg_file2,reg_file3,reg_file4,reg_file5,reg_file6,reg_file7,
    reg_file8,reg_file9,reg_file10,reg_file11,reg_file12,reg_file13,reg_file14
);

   reg [63:0] reg_array[0:15];

    always @(*) 
    begin
        reg_file0 = reg_array[0];
        reg_file1 = reg_array[1];
        reg_file2 = reg_array[2];
        reg_file3 = reg_array[3];
        reg_file4 = reg_array[4];
        reg_file5 = reg_array[5];
        reg_file6 = reg_array[6];
        reg_file7 = reg_array[7];
        reg_file8 = reg_array[8];
        reg_file9 = reg_array[9];
        reg_file10 = reg_array[10];
        reg_file11 = reg_array[11];
        reg_file12 = reg_array[12];
        reg_file13 = reg_array[13];
        reg_file14 = reg_array[14];      
    end

    initial 
    begin
        reg_array[0] = 64'd0;        
        reg_array[1] = 64'd0;         
        reg_array[2] = 64'd0;        
        reg_array[3] = 64'd0;          
        reg_array[4] = 64'd0;        
        reg_array[5] = 64'd0;        
        reg_array[6] = 64'd0;       
        reg_array[7] = 64'd0;      
        reg_array[8] = 64'd0;     
        reg_array[9] = 64'd0;    
        reg_array[10] = 64'd0;    
        reg_array[11] = 64'd0;   
        reg_array[12] = 64'd0;        
        reg_array[13] = 64'd0;     
        reg_array[14] = 64'd0;     
        reg_array[15] = 64'd0;  
       
    end
   
    // No change wires
    always @(*) 
    begin        
        d_stat  <= D_stat;
        d_icode <= D_icode;
        d_ifun  <= D_ifun;
        d_valC  <= D_valC;
    end

    // Updating register file at positive edge of clock
    always @(posedge clk) 
    begin        
        reg_array[W_dstM] <= W_valM;
        reg_array[W_dstE] <= W_valE;
    end

    // d_dstE
    always @(*) begin
    if (d_icode == 4'h2 || d_icode == 4'h3 || d_icode == 4'h6)
        d_dstE <= D_rB;
    else if (d_icode == 4'h8 || d_icode == 4'h9 || d_icode == 4'hA || d_icode == 4'hB)
        d_dstE <= 4;
    else
        d_dstE <= 15;
end


    // d_dstM
    // d_dstM
always @(*) begin
    if (d_icode == 4'h5 || d_icode == 4'hB)
        d_dstM = D_rA;
    else
        d_dstM = 15;
end

// d_srcA
always @(*) begin
    if (d_icode == 4'h2 || d_icode == 4'h4 || d_icode == 4'h6)
        d_srcA = D_rA;
    else if (d_icode == 4'h9 || d_icode == 4'hA || d_icode == 4'hB)
        d_srcA = 4;
    else
        d_srcA = 15;
end

// d_srcB
always @(*) begin
    if (d_icode == 4'h4 || d_icode == 4'h5 || d_icode == 4'h6)
        d_srcB = D_rB;
    else if (d_icode == 4'h8 || d_icode == 4'h9 || d_icode == 4'hA || d_icode == 4'hB)
        d_srcB = 4;
    else
        d_srcB = 15;
end


    // Data forwarding of A 
    always @(*) begin
        if (d_icode == 7 || d_icode == 8) 
            begin
                d_valA <= D_valP;
            end
        else if (d_srcA == e_dstE) 
            begin
                d_valA <= e_valE;
            end
        else if (d_srcA == M_dstM) 
            begin
                d_valA <= m_valM;
            end
        else if (d_srcA == M_dstE) 
            begin
                d_valA <= M_valE;
            end
        else if (d_srcA == W_dstM) 
            begin
                d_valA <= W_valM;
            end
        else if (d_srcA == W_dstE) 
            begin
                d_valA <= W_valE;
            end
        else 
            begin    
                d_valA <= reg_array[d_srcA];
            end
    end

    // Data forwarding of B
    always @(*) begin

        if (d_srcB == e_dstE) 
            begin
                d_valB <= e_valE;
            end
        else if (d_srcB == M_dstM) 
            begin
                d_valB <= m_valM;
            end
        else if (d_srcB == M_dstE) 
            begin
                d_valB <= M_valE;
            end
        else if (d_srcB == W_dstM) 
            begin
                d_valB <= W_valM;
            end
        else if (d_srcB == W_dstE) 
            begin
                d_valB <= W_valE;
            end
        else 
            begin    
                d_valB <= reg_array[d_srcB];
            end
    end

endmodule

module add_1bit(input a, input b, input c, output sum, output carry);

    wire w1, w2, w3;

    xor(w1, a, b);
    and(w2, a, b);
    xor(sum, w1, c);
    and(w3, w1, c);
    or(carry, w2, w3);

endmodule

module add_64bit(a, b, sum, overflow);

    input signed [63:0] a;
    input signed [63:0] b;
    output signed [63:0] sum;
    output overflow;

    wire [64:0] carry;

    assign carry[0] = 1'b0;

    genvar i;

    generate
        for(i = 0; i < 64; i = i+1)
            begin
              add_1bit f1(a[i], b[i], carry[i], sum[i], carry[i+1]);
            end
    endgenerate

    wire overflow1, overflow2, a_bar, b_bar, sum_bar;
    not(sum_bar, sum[63]);
    not(b_bar, b[63]);
    not(a_bar, a[63]);
    and(overflow1, a[63], b[63], sum_bar);
    and(overflow2, a_bar, b_bar, sum[63]);
    or(overflow, overflow1, overflow2);

endmodule

module sub_64bit(a, b, out, overflow);

    input signed [63:0] a;
    input signed [63:0] b;
    output signed [63:0] out;
    output overflow;

    wire g1, g2;
    wire [63:0] buffer1;
    wire [63:0] buffer2;

    genvar i;
    generate
        for(i = 0; i < 64; i = i+1)
            begin
              not(buffer1[i], b[i]);
            end
    endgenerate

    add_64bit f1(buffer1, 64'b1, buffer2, g1);
    add_64bit f2(a, buffer2, out, g2);

    wire overflow1, overflow2, a_bar, b_bar, out_bar;
    not(out_bar, out[63]);
    not(b_bar, b[63]);
    not(a_bar, a[63]);
    and(overflow1, a[63], b_bar, out_bar);
    and(overflow2, a_bar, b[63], out[63]);
    or(overflow, overflow1, overflow2);

endmodule

module and_64bit(a, b, out);

    input signed [63:0] a;
    input signed [63:0] b;
    output signed [63:0] out;

    genvar i;

    generate
        for(i = 0; i < 64; i = i+1)
            begin
              and(out[i], a[i], b[i]);
            end
    endgenerate

endmodule
module xor_64bit(a, b, out);

    input signed [63:0] a;
    input signed [63:0] b;
    output signed [63:0] out;

    genvar i;

    generate
        for(i = 0; i < 64; i = i+1)
            begin
              xor(out[i], a[i], b[i]);
            end
    endgenerate

endmodule

module alu(control, a, b, out, overflow);

    input [1:0] control;
    input signed [63:0] a;
    input signed [63:0] b;
    output signed [63:0] out;
    output overflow;
    
    reg overflow;
    reg signed [63:0] out;
  
    wire signed [63:0] out0;
    wire signed [63:0] out1;
    wire signed [63:0] out2;
    wire signed [63:0] out3;
    wire overflow0, overflow1;

    add_64bit f0(a, b, out0, overflow0);
    sub_64bit f1(a, b, out1, overflow1);
    and_64bit f2(a, b, out2);
    xor_64bit f3(a, b, out3);

    always @(*) begin
        case (control)
            2'b00: begin
              out = out0;
              overflow = overflow0;
            end 

            2'b01: begin
              out = out1;
              overflow = overflow1;
            end

            2'b10: begin
              out = out2;
              overflow = 1'b0;
            end

            2'b11: begin
              out = out3;
              overflow = 1'b0;
            end
        endcase
    end

endmodule

module execute (
    input clk,
    
    // Inputs
    input [3:0] E_stat,E_icode, E_ifun,
    input signed [63:0] E_valC, E_valA, E_valB,
    input [3:0] E_dstE, E_dstM,W_stat, m_stat,

    // Outputs
    output reg[3:0] e_stat,e_icode,
    output reg signed [63:0] e_valE,e_valA,
    output reg[3:0] e_dstE,e_dstM,
    output reg e_Cnd,ZF, SF, OF
);

    // No change wires
    always @(*) 
    begin    
        e_stat  <= E_stat;
        e_icode <= E_icode;
        e_valA  <= E_valA;
        e_dstM  <= E_dstM;
    end

    wire signed [63:0] ans;
    reg signed [63:0] aluA, aluB;
    wire overflow, set_cc;
    
    // ask
    assign set_cc = ((e_icode == 6) && (m_stat == 4'b1000) && (W_stat == 4'b1000)) ? 1 : 0;
    reg [1:0] control;
    
    // ask
    alu alu1(.control(control), .a(aluA), .b(aluB), .overflow(overflow), .out(ans));

    always @(*) 
    begin
        if (e_icode == 4'b0010)  // cmovXX rA, rB
        begin    
            e_Cnd = 0;        
            if (E_ifun == 4'b0000)  // cmove(unconditional) 
                e_Cnd = 1;
            else if (E_ifun == 4'b0001 && ((SF^OF)|ZF))  // cmovle
                e_Cnd = 1;
            else if (E_ifun == 4'b0010 && (SF^OF))  // cmovl 
                e_Cnd = 1;
            else if (E_ifun == 4'b0011 && ZF)  // cmove 
                e_Cnd = 1;
            else if (E_ifun == 4'b0100 && !ZF)  // cmovne 
                e_Cnd = 1;
            else if (E_ifun == 4'b0101 && !(SF^OF))  // cmovge 
                e_Cnd = 1;
            else if (E_ifun == 4'b0110 && !(SF^OF) && !ZF)  // cmovg 
                e_Cnd = 1;

            aluA = E_valA;
            aluB = 64'd0;
            control = 2'b00;  // valE = valA + 0
        end
        else if (e_icode == 4'b0011)  // irmovq V, rB 
        begin 
            aluA = E_valC;              
            aluB = 64'd0;
            control = 2'b00;  // valE = 0 + valC
        end
        else if (e_icode == 4'b0100)  // rmmovq rA, D(rB)
        begin                 
            aluA = E_valC;          
            aluB = E_valB;
            control = 2'b00;  // valE = valB + valC
        end     
        else if (e_icode == 4'b0101)  // mrmovq D(rB), rA
        begin 
            aluA = E_valC;
            aluB = E_valB;
            control = 2'b00;  // valE = valB + valC
        end
        else if (e_icode == 4'b0110)  // Opq rA, rB
        begin
            aluA = E_valB;
            aluB = E_valA;
            control = E_ifun[1:0];
        end
        else if (e_icode == 4'b0111)  // jXX Dest
        begin
            e_Cnd = 0;
            if (E_ifun == 4'b0000)  // jmp 
                e_Cnd = 1;  // unconditional jump
            else if (E_ifun == 4'b0001 && ((SF^OF)|ZF))  // jle 
                e_Cnd = 1;
            else if (E_ifun == 4'b0010 && (SF^OF))  // jl 
                e_Cnd = 1;
            else if (E_ifun == 4'b0011)  // je
                e_Cnd = ZF ? 1 : 0;
            else if (E_ifun == 4'b0100 && !ZF)  // jne
                e_Cnd = 1;
            else if (E_ifun == 4'b0101 && !(SF^OF))  // jge 
                e_Cnd = 1;
            else if (E_ifun == 4'b0110 && !(SF^OF) && !ZF)  // jg
                e_Cnd = 1;
        end
        else if (e_icode == 4'b1000)  // call Dest 
        begin 
            aluA = -64'd8;
            aluB = E_valB;
            control = 2'b00;  // valE = valB - 8
        end
        else if (e_icode == 4'b1001)  // ret 
        begin 
            aluA = 64'd8;
            aluB = E_valB;
            control = 2'b00;  // valE = valB + 8
        end
        else if (e_icode == 4'b1010)  // pushq rA 
        begin 
            aluA = -64'd8;
            aluB = E_valB;
            control = 2'b00;  // valE = valB - 8
        end
        else if (e_icode == 4'b1011)  // popq rA
        begin 
            aluA = 64'd8;
            aluB = E_valB;
            control = 2'b00;  // valE = valB + 8
        end
        else
            control = 2'b00;  // default control value

        e_valE = ans;
    end

    always @(posedge clk ) 
    begin
        if (set_cc == 1)
        begin
            ZF = (e_valE == 64'b0); // zero flag
            SF = (e_valE[63] == 1); // signed flag
            OF = (aluA[63] == aluB[63]) && (e_valE[63] != aluA[63]); // overflow flag
        end
    end

    // e_dstE
    always @(*) 
    begin
        if (e_icode == 2 && e_Cnd == 0) 
        begin
            e_dstE <= 15;    
        end
        else
        begin
            e_dstE <= E_dstE;
        end
    end
endmodule

module memory(
    input clk,
    input [3:0] M_stat,M_icode,
    input signed [63:0] M_valE, M_valA,
    input [3:0] M_dstE, M_dstM,
    output reg[3:0] m_stat,m_icode,
    output reg signed[63:0] m_valE, m_valM,
    output reg[3:0] m_dstE, m_dstM
);

   reg [63:0] Data_Mem[0:4095];
    integer i;

    initial begin
        for (i = 0; i < 4096; i = i+1) 
        begin
            Data_Mem[i] <= 0;
        end


    end

    // No change wires
    always @(*) 
    begin    
        m_icode <= M_icode;
        m_valE  <= M_valE;
        m_dstE  <= M_dstE;
        m_dstM  <= M_dstM;
    end

    // Mem_write block
    reg Mem_write;

    always @(*) 
    begin
        if (m_icode == 4'h4 || m_icode == 4'h8 || m_icode == 4'hA)
            Mem_write <= 1;
        else
            Mem_write <= 0; 
    end

    // Mem_read block
    reg Mem_read;

    always @(*) 
    begin
        if (m_icode == 4'h5 || m_icode == 4'h9 || m_icode == 4'hB)
            Mem_read <= 1;
        else
            Mem_read <= 0; 
    end

    // Selecting Address
    reg [63:0] m_addr;

    always @(*) 
    begin
        if (m_icode == 4'h4 || m_icode == 4'h5 || m_icode == 4'h8 || m_icode == 4'hA)
            m_addr <= m_valE;
        else if (m_icode == 4'h9 || m_icode == 4'hB)
            m_addr <= M_valA;
        else
            m_addr <= 4095; 
    end

    // Checking memory error
    reg dmem_error;     // Memory error flag

    always @(*) 
    begin
        if (m_addr < 4096 && m_addr >= 0) 
            dmem_error <= 0;
        else 
            dmem_error <= 1;
    end

    wire [63:0] m_data_in;

    assign m_data_in = M_valA;

    // Writing back to memory
    always @(posedge clk) 
    begin
        if (dmem_error == 0 && Mem_write == 1) 
            Data_Mem[m_addr] <= m_data_in;
    end

    // Reading from memory
    always @(*) 
    begin
        if (dmem_error == 0 && Mem_read == 1) 
            m_valM <= Data_Mem[m_addr];
        else 
            m_valM <= 0;
    end

    // m_stat
    always @(*) 
    begin        
        if (dmem_error == 1) 
            m_stat <= 4'b0010;
        else 
            m_stat <= M_stat;
    end
endmodule

module PIPE_con(
    input [3:0] D_icode, E_icode, M_icode,
    input [3:0] d_srcA, d_srcB,
    input [3:0] E_dstM,
    input e_Cnd,
    input [3:0] m_stat, W_stat,

    output reg W_stall, D_stall, F_stall,
    output reg M_bubble, E_bubble, D_bubble
);

    wire Ret = (D_icode == 9 || E_icode == 9 || M_icode == 9);
    wire LU_Haz = ((E_icode == 5 || E_icode == 11) && (E_dstM == d_srcA || E_dstM == d_srcB));
    wire Miss_Pred = (E_icode == 7 && e_Cnd == 0);

    // F_stall
    always @(*) 
        F_stall = (Ret || LU_Haz || (Ret && Miss_Pred)) ? 1 : 0;

    // D_stall
    always @(*) 
        D_stall = (LU_Haz || (Ret && LU_Haz)) ? 1 : 0;

    // D_bubble
    always @(*) 
        D_bubble = (D_stall == 0) ? (Ret || Miss_Pred) : 0;

    // E_bubble
    always @(*) 
        E_bubble = (LU_Haz && Ret) || (Ret && Miss_Pred) || LU_Haz || Miss_Pred;

    // M_bubble
    always @(*) 
        M_bubble = (m_stat != 4'b1000) || (W_stat != 4'b1000);

    // W_stall
    always @(*) 
        W_stall = (W_stat != 4'b1000);

endmodule

module PIPE();

    reg clk;

    // F pipeline register
    reg [63:0] F_predPC = 0;

    // D pipeline register
    reg [3:0] D_stat = 4'b1000;
    reg [3:0] D_icode = 1;
    reg [3:0] D_ifun ,D_rA ,D_rB = 0;
    reg signed [63:0] D_valC = 0;
    reg [63:0] D_valP = 0;


    // M pipeline register
    reg [3:0] M_stat = 4'b1000;
    reg [3:0] M_icode = 1;
    reg M_Cnd = 0;
    reg signed [63:0] M_valE = 0;
    reg signed [63:0] M_valA = 0;
    reg [3:0] M_dstE = 0;
    reg [3:0] M_dstM = 0;

    // E pipeline register
    reg [3:0] E_stat = 4'b1000;
    reg [3:0] E_icode = 1;
    reg [3:0] E_ifun = 0;

    reg signed [63:0] E_valA, E_valB ,E_valC = 0;
    reg [3:0] E_dstE ,E_dstM,E_srcA ,E_srcB = 0;
    wire ZF,OF,SF;

        // Registers
    wire signed [63:0] reg_file0,reg_file1,reg_file2,reg_file3,reg_file4,reg_file5,reg_file6,reg_file7,
    reg_file8,reg_file9,reg_file10,reg_file11,reg_file12,reg_file13,reg_file14;

        // Memory 
    wire [3:0] m_stat,m_icode;
    wire signed [63:0] m_valE, m_valM;
    wire [3:0] m_dstE, m_dstM;

    wire [3:0] Stat;

    // Execute
    wire [3:0] e_stat,e_icode;
    wire e_Cnd;
    wire signed [63:0] e_valE, e_valA;
    wire [3:0] e_dstE, e_dstM;
    
    // W pipeline register
    reg [3:0] W_stat = 4'b1000;
    reg [3:0] W_icode = 1;
    reg signed [63:0] W_valE,W_valM = 0;  
    reg [3:0] W_dstE ,W_dstM = 0;

    // Fetch 
    wire [3:0] f_stat,f_icode, f_ifun,f_rA, f_rB;
    wire signed [63:0] f_valC;
    wire [63:0] f_valP, f_predPC;

    // Decode 
    wire [3:0] d_stat,d_icode, d_ifun;
    wire signed [63:0] d_valC, d_valA, d_valB;
    wire [3:0] d_dstE, d_dstM,d_srcA, d_srcB;



    // Passing inputs to FETCH stage
    fetch f(
        // Inputs from F register
        .F_predPC(F_predPC),

        // Inputs forwarded from M register
        .M_icode(M_icode),.M_Cnd(M_Cnd),.M_valA(M_valA),

        // Inputs forwarded from W register
        .W_icode(W_icode),.W_valM(W_valM),

        // Outputs
        .f_stat(f_stat),.f_icode(f_icode),.f_ifun(f_ifun),.f_rA(f_rA),.f_rB(f_rB),.f_valC(f_valC),.f_valP(f_valP),
        .f_predPC(f_predPC)
            );

    
    execute e(
        .clk(clk),

        // Inputs
        .E_stat(E_stat),.E_icode(E_icode),.E_ifun(E_ifun),.E_valC(E_valC),.E_valA(E_valA),
        .E_valB(E_valB),.E_dstE(E_dstE),.E_dstM(E_dstM),.W_stat(W_stat),.m_stat(m_stat),

        // Outputs
        .e_stat(e_stat),.e_icode(e_icode),.e_Cnd(e_Cnd),.e_valE(e_valE),.e_valA(e_valA),
        .e_dstE(e_dstE),.e_dstM(e_dstM),
        .ZF(ZF),
        .OF(OF),
        .SF(SF)
    );


    decode d(
        .clk(clk),
        
        // Inputs from D register
        .D_stat(D_stat),.D_icode(D_icode),.D_ifun(D_ifun),.D_rA(D_rA),.D_rB(D_rB),.D_valC(D_valC),.D_valP(D_valP),

        // Inputs forwarded from execute stage
        .e_dstE(e_dstE),.e_valE(e_valE),

        // inputs forwarded from M register and memory stage
        .M_dstE(M_dstE),.M_valE(M_valE),.M_dstM(M_dstM),.m_valM(m_valM),

        // Inputs forwarded from W register
        .W_dstM(W_dstM),.W_valM(W_valM),.W_dstE(W_dstE),.W_valE(W_valE),

        // Outputs
        .d_stat(d_stat),.d_icode(d_icode),.d_ifun(d_ifun),.d_valC(d_valC),.d_valA(d_valA),.d_valB(d_valB),
        .d_dstE(d_dstE),.d_dstM(d_dstM),.d_srcA(d_srcA),.d_srcB(d_srcB),
        
        
        .reg_file0(reg_file0),.reg_file1(reg_file1),.reg_file2(reg_file2),.reg_file3(reg_file3),
        .reg_file4(reg_file4),.reg_file5(reg_file5),.reg_file6(reg_file6),.reg_file7(reg_file7),
        .reg_file8(reg_file8),.reg_file9(reg_file9),.reg_file10(reg_file10),.reg_file11(reg_file11),
        .reg_file12(reg_file12),.reg_file13(reg_file13),.reg_file14(reg_file14)
    );

    memory m(
        .clk(clk),


        // Outputs
        .m_stat(m_stat),
        .m_icode(m_icode),
        .m_valE(m_valE),
        .m_valM(m_valM),
        .m_dstE(m_dstE),
        .m_dstM(m_dstM),

        // Inputs
        .M_stat(M_stat),
        .M_icode(M_icode),
        .M_valE(M_valE),
        .M_valA(M_valA),
        .M_dstE(M_dstE),
        .M_dstM(M_dstM)

    );

    // Stat Code
    assign Stat = W_stat;

    // Control Logic
    wire W_stall, D_stall, F_stall;
    wire M_bubble, E_bubble, D_bubble;

        // F register 
    always @(posedge clk) 
    begin
        if (F_stall != 1) 
            begin
                F_predPC <= f_predPC;
            end
    end

    PIPE_con pipe_con(
        // Inputs
        .D_icode(D_icode),
        .d_srcA(d_srcA),
        .d_srcB(d_srcB),
        .E_icode(E_icode),
        .E_dstM(E_dstM),
        .e_Cnd(e_Cnd),
        .M_icode(M_icode),
        .m_stat(m_stat),
        .W_stat(W_stat),

        // Outputs
        .W_stall(W_stall),
        .M_bubble(M_bubble),
        .E_bubble(E_bubble),
        .D_bubble(D_bubble),
        .D_stall(D_stall),
        .F_stall(F_stall)
    );




    // D register 
    always @(posedge clk) 
    begin
        if (D_stall == 0) 
            begin
                if (D_bubble == 0) 
                begin
                    D_stat  <= f_stat;
                    D_icode <= f_icode;
                    D_ifun  <= f_ifun;

                    D_valC  <= f_valC;
                    D_valP  <= f_valP;

                    D_rA    <= f_rA;
                    D_rB    <= f_rB;


                end
                else 
                begin
                    D_stat  <= 4'b1000;

                    D_valC  <= 0;
                    D_valP  <= 0;

                    D_icode <= 1;
                    D_ifun  <= 0;

                    D_rA    <= 0;
                    D_rB    <= 0;

                end
            end       
    end

    // E register
    always @(posedge clk) 
    begin
        if (E_bubble == 1) 
            begin
                E_stat  <= 4'b1000;
                E_icode <= 1;
                E_ifun  <= 0;


                E_dstE  <= 0;
                E_dstM  <= 0;


                E_valA  <= 0;
                E_valB  <= 0;
                E_valC  <= 0;


                E_srcA  <= 0;
                E_srcB  <= 0;
            end
        else 
            begin
                E_stat  <= d_stat;
                E_icode <= d_icode;
                E_ifun  <= d_ifun;


                E_srcA  <= d_srcA;
                E_srcB  <= d_srcB;


                E_valA  <= d_valA;
                E_valB  <= d_valB;
                E_valC  <= d_valC;

                E_dstE  <= d_dstE;
                E_dstM  <= d_dstM;

            end
    end

    // W register
    always @(posedge clk) 
    begin
        if (W_stall != 1) 
            begin
                W_stat  <= m_stat;
                W_icode <= m_icode;

                W_dstE  <= m_dstE;
                W_dstM  <= m_dstM;

                W_valE  <= m_valE;
                W_valM  <= m_valM;


            end
    end
    // M register
    always @(posedge clk) 
    begin
        if (M_bubble == 1) 
            begin
                M_stat  <= 4'b1000;
                M_icode <= 1;
                M_Cnd   <= 0;

                M_dstE  <= 0;
                M_dstM  <= 0;

                M_valE  <= 0;
                M_valA  <= 0;


            end
        else 
            begin
                M_stat  <= e_stat;
                M_icode <= e_icode;
                M_Cnd   <= e_Cnd;

                M_dstE  <= e_dstE;
                M_dstM  <= e_dstM;

                M_valE  <= e_valE;
                M_valA  <= e_valA;


            end
    end




    initial begin
        
        $dumpfile("dump.vcd");
        $dumpvars;
        
    end

    initial begin
        clk <= 0;
        forever begin
            #10 clk <= ~clk;
        end
    end

    initial begin
        $monitor("time=%0d, clk=%0d, f_pc=%0d, f_icode=%0d, f_ifun=%0d, f_rA=%0d, f_rB=%0d, f_valP=%0d, f_valC=%0d, D_icode=%0d, E_icode=%0d, M_icode=%0d, W_icode=%0d Stat=%d reg1=%d reg2=%d reg3=%d reg4=%d reg5=%d reg6=%d reg7=%d reg8=%d reg9=%d reg10=%d reg11=%d reg12=%d reg13=%d reg14=%d reg15=%d ZF=%d SF=%d OF=%d W_valM=%d M_valA=%d, e_valE=%d\n", 
            $time, clk, f_predPC, f_icode, f_ifun, f_rA, f_rB, f_valP, f_valC, D_icode, E_icode, M_icode, W_icode, Stat, reg_file0, reg_file1, reg_file2, reg_file3, reg_file4, reg_file5, reg_file6, reg_file7, reg_file8, reg_file9, reg_file10, reg_file11, reg_file12, reg_file13, reg_file14, ZF, SF, OF, W_valM, M_valA, e_valE);
    end

    always @(*)
    begin 
        if(Stat == 4'b0100)
            begin
                $finish;
            end
    end


endmodule