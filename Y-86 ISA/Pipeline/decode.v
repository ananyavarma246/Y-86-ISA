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
        always @(posedge clk) 
    begin        
        reg_array[W_dstM] <= W_valM;
        reg_array[W_dstE] <= W_valE;
    end
   
    // No change wires
    always @(*) 
    begin        
        d_valC  <= D_valC;
        d_stat  <= D_stat;
        d_icode <= D_icode;
        d_ifun  <= D_ifun;
    end

    // Updating register file at positive edge of clock


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
    case (1)
        d_icode == 7 || d_icode == 8: 
            d_valA <= D_valP;
        d_srcA == e_dstE:
            d_valA <= e_valE;
        d_srcA == M_dstM:
            d_valA <= m_valM;
        d_srcA == M_dstE:
            d_valA <= M_valE;
        d_srcA == W_dstM:
            d_valA <= W_valM;
        d_srcA == W_dstE:
            d_valA <= W_valE;
        default:
            d_valA <= reg_array[d_srcA];
    endcase
end

    // Data forwarding of B
always @(*) begin
    case (d_srcB)
        e_dstE: d_valB <= e_valE;
        M_dstM: d_valB <= m_valM;
        M_dstE: d_valB <= M_valE;
        W_dstM: d_valB <= W_valM;
        W_dstE: d_valB <= W_valE;
        default: d_valB <= reg_array[d_srcB];
    endcase
end


endmodule