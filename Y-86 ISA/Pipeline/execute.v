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
    wire signed [63:0] ans;
    reg signed [63:0] aluA, aluB;
    wire overflow, set_cc;
    reg [1:0] control;

    // No change wires
    always @(*) 
    begin    
        e_stat  <= E_stat;
        e_icode <= E_icode;
        e_valA  <= E_valA;
        e_dstM  <= E_dstM;
    end


    
    // ask
    assign set_cc = ((e_icode == 6) && (m_stat == 4'b1000) && (W_stat == 4'b1000)) ? 1 : 0;

    
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
