module fetch(F_predPC, M_icode, M_Cnd, M_valA, W_icode, W_valM, f_stat, f_icode, f_ifun, f_rA, f_rB, f_valC, f_valP, f_predPC);

    // inputs
    input M_Cnd;
    input [3:0] M_icode, W_icode;

    input [63:0] F_predPC,M_valA, W_valM;

    reg [7:0] inst_mem[0:1023];
    // outputs
    output reg [3:0] f_stat, f_icode, f_ifun, f_rA, f_rB;

    output reg signed [63:0] f_valC,f_valP, f_predPC;

    // Setting up instruction memory
    reg [63:0] PC;
    reg instr_valid;
    reg imem_error;

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

    //halt works
    // inst_mem[0] = 8'b00000000;

    // nop works
    // inst_mem[0] = 8'b00010000;

    // rrmovq works
    //  inst_mem[20] = 8'b00100000;
    //  inst_mem[21] = 8'b00100011;
    //  inst_mem[22] = 8'b00000000; // halt

    // rmmovq works
    //   inst_mem[20]=8'b01000000; 
    //   inst_mem[21]=8'b00100011; 
    //   inst_mem[22]=8'b00000000; 
    //   inst_mem[23]=8'b00000000; 
    //   inst_mem[24]=8'b00000000; 
    //   inst_mem[25]=8'b00000000; 
    //   inst_mem[26]=8'b00000000; 
    //   inst_mem[27]=8'b00000000; 
    //   inst_mem[28]=8'b00000000; 
    //   inst_mem[29]=8'b00000001; 
      // inst_mem[30]=8'b00000000; //halt

    // mrmovq works
    //   inst_mem[30]=8'b01010000; 
    //   inst_mem[31]=8'b00110011; 
    //   inst_mem[32]=8'b00000000; 
    //   inst_mem[33]=8'b00000000; 
    //   inst_mem[34]=8'b00000000; 
    //   inst_mem[35]=8'b00000000; 
    //   inst_mem[36]=8'b00000000; 
    //   inst_mem[37]=8'b00000000; 
    //   inst_mem[38]=8'b00000000; 
    //   inst_mem[39]=8'b00000001; 
    //   inst_mem[40]=8'b00000000; //halt

    // jmp
//   inst_mem[20]=8'b01110000; 
//   inst_mem[21]=8'b00000000; 
//   inst_mem[22]=8'b00000000; 
//   inst_mem[23]=8'b00000000; 
//   inst_mem[24]=8'b00000000; 
//   inst_mem[25]=8'b00000000; 
//   inst_mem[26]=8'b00000000; 
//   inst_mem[27]=8'b00000000; 
//   inst_mem[28]=8'b00010000; 

    // jxx works   -- je
  // inst_mem[22]=8'b01110011; 
  // inst_mem[23]=8'b00000000; 
  // inst_mem[24]=8'b00000000; 
  // inst_mem[25]=8'b00000000; 
  // inst_mem[26]=8'b00000000; 
  // inst_mem[27]=8'b00000000; 
  // inst_mem[28]=8'b00000000; 
  // inst_mem[29]=8'b00000000; 
  // inst_mem[30]=8'b00010000; 
  // inst_mem[31]=8'b00000000; // halt
  
  
    //call works
    // inst_mem[0] = 8'b00110000;  // irmovq
    // inst_mem[1] = 8'b11110101;  // rA = 15, rB = 3
    // inst_mem[2] = 8'b00000000;
    // inst_mem[3] = 8'b00000000;
    // inst_mem[4] = 8'b00000000;
    // inst_mem[5] = 8'b00000000;
    // inst_mem[6] = 8'b00000000;
    // inst_mem[7] = 8'b00000000;
    // inst_mem[8] = 8'b00000000;
    // inst_mem[9] = 8'b00001111;  // valC
    //   inst_mem[10]=8'b10000000; 
    //   inst_mem[11]=8'b00000000; 
    //   inst_mem[12]=8'b00000000; 
    //   inst_mem[13]=8'b00000000; 
    //   inst_mem[14]=8'b00000000; 
    //   inst_mem[15]=8'b00000000; 
    //   inst_mem[16]=8'b00000000; 
    //   inst_mem[17]=8'b00000000; 
    //   inst_mem[18]=8'b00000100; 
    //   inst_mem[19]=8'b00000000; // halt


    // ret works
    // inst_mem[0] = 8'b00110000;  // irmovq
    // inst_mem[1] = 8'b11110011;  // rA = 15, rB = 3
    // inst_mem[2] = 8'b00000000;
    // inst_mem[3] = 8'b00000000;
    // inst_mem[4] = 8'b00000000;
    // inst_mem[5] = 8'b00000000;
    // inst_mem[6] = 8'b00000000;
    // inst_mem[7] = 8'b00000000;
    // inst_mem[8] = 8'b00000000;
    // inst_mem[9] = 8'b00000001;  // valC
    // inst_mem[10] = 8'b00110000;  // irmovq
    // inst_mem[11] = 8'b11110101;  // rA = 15, rB = 5  --- rsp
    // inst_mem[12] = 8'b00000000;
    // inst_mem[13] = 8'b00000000;
    // inst_mem[14] = 8'b00000000;
    // inst_mem[15] = 8'b00000000;
    // inst_mem[16] = 8'b00000000;
    // inst_mem[17] = 8'b00000000;
    // inst_mem[18] = 8'b00000000;
    // inst_mem[19] = 8'b00001111;  // valC
    //   inst_mem[20]=8'b01000000;   //rmmovq
    //   inst_mem[21]=8'b00110011; 
    //   inst_mem[22]=8'b00000000; 
    //   inst_mem[23]=8'b00000000; 
    //   inst_mem[24]=8'b00000000; 
    //   inst_mem[25]=8'b00000000; 
    //   inst_mem[26]=8'b00000000; 
    //   inst_mem[27]=8'b00000000; 
    //   inst_mem[28]=8'b00000000; 
    //   inst_mem[29]=8'b00001110; 
    //   inst_mem[30]=8'b10010000;   // ret 
    //   inst_mem[31]=8'b00000000; //halt
  

    //pushq works
    // inst_mem[0] = 8'b00110000;  // irmovq
    // inst_mem[1] = 8'b11110011;  // rA = 15, rB = 3
    // inst_mem[2] = 8'b00000000;
    // inst_mem[3] = 8'b00000000;
    // inst_mem[4] = 8'b00000000;
    // inst_mem[5] = 8'b00000000;
    // inst_mem[6] = 8'b00000000;
    // inst_mem[7] = 8'b00000000;
    // inst_mem[8] = 8'b00000000;
    // inst_mem[9] = 8'b00001111;  // valC
    // inst_mem[10] = 8'b00110000;  // irmovq
    // inst_mem[11] = 8'b11110101;  // rA = 15, rB = 5
    // inst_mem[12] = 8'b00000000;
    // inst_mem[13] = 8'b00000000;
    // inst_mem[14] = 8'b00000000;
    // inst_mem[15] = 8'b00000000;
    // inst_mem[16] = 8'b00000000;
    // inst_mem[17] = 8'b00000000;
    // inst_mem[18] = 8'b00000000;
    // inst_mem[19] = 8'b00001111;  // valC
    // inst_mem[20]=8'b10100000; 
    // inst_mem[21]=8'b00111111; // rA = 3
    // inst_mem[22]=8'b00000000; //halt

    // popq works
    // inst_mem[0] = 8'b00110000;  // irmovq
    // inst_mem[1] = 8'b11110011;  // rA = 15, rB = 3
    // inst_mem[2] = 8'b00000000;
    // inst_mem[3] = 8'b00000000;
    // inst_mem[4] = 8'b00000000;
    // inst_mem[5] = 8'b00000000;
    // inst_mem[6] = 8'b00000000;
    // inst_mem[7] = 8'b00000000;
    // inst_mem[8] = 8'b00000000;
    // inst_mem[9] = 8'b00000001;  // valC
    // inst_mem[10] = 8'b00110000;  // irmovq
    // inst_mem[11] = 8'b11110101;  // rA = 15, rB = 5  --- rsp
    // inst_mem[12] = 8'b00000000;
    // inst_mem[13] = 8'b00000000;
    // inst_mem[14] = 8'b00000000;
    // inst_mem[15] = 8'b00000000;
    // inst_mem[16] = 8'b00000000;
    // inst_mem[17] = 8'b00000000;
    // inst_mem[18] = 8'b00000000;
    // inst_mem[19] = 8'b00001111;  // valC
    // inst_mem[20]=8'b01000000;   //rmmovq
    //   inst_mem[21]=8'b00110011; 
    //   inst_mem[22]=8'b00000000; 
    //   inst_mem[23]=8'b00000000; 
    //   inst_mem[24]=8'b00000000; 
    //   inst_mem[25]=8'b00000000; 
    //   inst_mem[26]=8'b00000000; 
    //   inst_mem[27]=8'b00000000; 
    //   inst_mem[28]=8'b00000000; 
    //   inst_mem[29]=8'b00001110; 
    // inst_mem[30]=8'b10110000; // popq
    // inst_mem[31]=8'b00101111;
    // inst_mem[32]=8'b00000000; //halt

    // cmovxx works  -- cmove
    // inst_mem[0] = 8'b00110000;  // irmovq
    // inst_mem[1] = 8'b11110011;  // rA = 15, rB = 3
    // inst_mem[2] = 8'b00000000;
    // inst_mem[3] = 8'b00000000;
    // inst_mem[4] = 8'b00000000;
    // inst_mem[5] = 8'b00000000;
    // inst_mem[6] = 8'b00000000;
    // inst_mem[7] = 8'b00000000;
    // inst_mem[8] = 8'b00000000;
    // inst_mem[9] = 8'b00000000;  // valC
    // inst_mem[10] = 8'b00110000; // irmovq
    // inst_mem[11] = 8'b11110010; // rA = 15, rB = 2
    // inst_mem[12] = 8'b00000000;
    // inst_mem[13] = 8'b00000000;
    // inst_mem[14] = 8'b00000000;
    // inst_mem[15] = 8'b00000000;
    // inst_mem[16] = 8'b00000000;
    // inst_mem[17] = 8'b00000000;
    // inst_mem[18] = 8'b00000000;
    // inst_mem[19] = 8'b00000000;  // valC 
    // inst_mem[20] = 8'b01100000;  // OPq add
    // inst_mem[21] = 8'b00100011;  // rA = 2, rB = 3 
    //  inst_mem[22] = 8'b00100011;  // cmove
    //  inst_mem[23] = 8'b00100011;  
    //  inst_mem[24] = 8'b00000000; // halt

    // call + return
    // inst_mem[0] = 8'b00110000;  // irmovq
    // inst_mem[1] = 8'b11110101;  
    // inst_mem[2] = 8'b00000000;
    // inst_mem[3] = 8'b00000000;
    // inst_mem[4] = 8'b00000000;
    // inst_mem[5] = 8'b00000000;
    // inst_mem[6] = 8'b00000000;
    // inst_mem[7] = 8'b00000000;
    // inst_mem[8] = 8'b00000000;
    // inst_mem[9] = 8'b00001111; 
    // inst_mem[10]=8'b00000000; //halt
    // inst_mem[11]=8'b10000000; // call
    // inst_mem[12]=8'b00000000; 
    // inst_mem[13]=8'b00000000; 
    // inst_mem[14]=8'b00000000; 
    // inst_mem[15]=8'b00000000; 
    // inst_mem[16]=8'b00000000; 
    // inst_mem[17]=8'b00000000; 
    // inst_mem[18]=8'b00000000; 
    // inst_mem[19]=8'b00011110; 
    // inst_mem[30]=8'b10010000; // ret 
    // inst_mem[20]=8'b00000000; //halt
    end



    // Select PC


    always @(*) 
    begin        
        
        if (W_icode == 9) 
            PC <= W_valM;
        else if (M_icode == 7 && M_Cnd == 0) 
            PC <= M_valA;
        else 
            PC <= F_predPC;
    end

    // Select f_icode, f_ifun, imem_error


    always @(*) 
    begin
        if (PC >= 0 && PC < 4096) begin 
            imem_error <= 0;
            f_ifun     <= inst_mem[PC][3:0];
            f_icode    <= inst_mem[PC][7:4];
        end
        else begin
            f_ifun     <= 0;
            imem_error <= 1;
            f_icode    <= 1;

        end
    end

    // instr_valid flag
    
    
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

            f_valC = 0;

            f_valP = PC + 2;

            f_rB   = inst_mem[PC + 1][3:0];

            f_rA   = inst_mem[PC + 1][7:4];

        end
        else if (f_icode == 4'h3) begin // irmovq


            
            f_valP = PC + 10;

            f_valC = {inst_mem[PC+2],inst_mem[PC+3],inst_mem[PC+4],inst_mem[PC+5],inst_mem[PC+6],inst_mem[PC+7],inst_mem[PC+8],inst_mem[PC+9]};

            f_rB   = inst_mem[PC + 1][3:0];

            f_rA   = inst_mem[PC + 1][7:4];

        end
        else if (f_icode == 4'h4) begin // rmmovq


            f_valP = PC + 10;

            f_valC = {inst_mem[PC+2],inst_mem[PC+3],inst_mem[PC+4],inst_mem[PC+5],inst_mem[PC+6],inst_mem[PC+7],inst_mem[PC+8],inst_mem[PC+9]};

            f_rB   = inst_mem[PC + 1][3:0];
            
            f_rA   = inst_mem[PC + 1][7:4];

        end
        else if (f_icode == 4'h5) begin // mrmovq

            f_valC = {inst_mem[PC+2],inst_mem[PC+3],inst_mem[PC+4],inst_mem[PC+5],inst_mem[PC+6],inst_mem[PC+7],inst_mem[PC+8],inst_mem[PC+9]}; 

            f_rB   = inst_mem[PC + 1][3:0];
            
            f_rA   = inst_mem[PC + 1][7:4];
            
            f_valP = PC + 10;
        end
        else if (f_icode == 4'h6) begin // OPq
            f_valC = 0;
            f_valP = PC + 2;
            f_rB   = inst_mem[PC + 1][3:0];
            
            f_rA   = inst_mem[PC + 1][7:4];

        end
        else if (f_icode == 4'h7) begin // jxx
            f_valP = PC + 9;
            f_valC = {inst_mem[PC+1],inst_mem[PC+2],inst_mem[PC+3],inst_mem[PC+4],inst_mem[PC+5],inst_mem[PC+6],inst_mem[PC+7],inst_mem[PC+8]};

        end
        else if (f_icode == 4'h8) begin // call
            f_valP = PC + 9;
            f_valC = {inst_mem[PC+1],inst_mem[PC+2],inst_mem[PC+3],inst_mem[PC+4],inst_mem[PC+5],inst_mem[PC+6],inst_mem[PC+7],inst_mem[PC+8]};

        end
        else if (f_icode == 4'h9) begin // ret
            f_valP = PC + 1; 
        end
        else if (f_icode == 4'hA) begin // pushq
            f_valC = 0;
            f_rA   = inst_mem[PC + 1][7:4];

            f_valP = PC + 2;

            f_rB   = inst_mem[PC + 1][3:0];


        end
        else if (f_icode == 4'hB) begin // popq
            f_valP = PC + 2;
            f_rB   = inst_mem[PC + 1][3:0];
            f_valC = 0;
            f_rA   = inst_mem[PC + 1][7:4];

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
