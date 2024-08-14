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

        // $dumpvars(0, Data_Mem[0], Data_Mem[1], Data_Mem[2], Data_Mem[3], Data_Mem[4], Data_Mem[5], Data_Mem[6], Data_Mem[7], Data_Mem[8]);
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