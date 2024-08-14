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
