module Control_Unit_CPU(
  input         clk, rst_b,
  input  [5:0]  op,         // opcode from IR(Instruction Register)
  input         ra,         // RA - Register Address from IR
  input         start,      // start from user
  input         n, z, carry, v, // flags
  input         inp_ack,  
  input         out_ack,  
  input         ack_alu,    // finish from ALU
  output        finish,     
  output [28:0] c           // control signals for proccesor
);
  
  // Implementare OneHot

  wire [45:0] qout;
 
  // 1. HLT
  ffd_OneHot #(.reset_val(1'b1)) S0 (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    (qout[0] & ~start) | // S0 and start = 0
    qout[3] & (~op[5] & ~op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0]) // S3 and opcode = 000000

  ), .q(qout[0]));
  
  ffd_OneHot S1   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[0] & start
  ), .q(qout[1]));
  
  ffd_OneHot S2   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[1]  | // from S0
    qout[5]  | // from S5
    qout[6]  |
    qout[12] |
    qout[14] |
    qout[16] |
    qout[17] |
    qout[23] |
    qout[25] |
    qout[28] |
    qout[31] |
    qout[34] |
    (qout[36] & out_ack) |
    (qout[3] & (
      (~op[5] & ~op[4] & op[3] & ~op[2] & op[1] & op[0] & ~(z)) |             // 001011 BEQ
      (~op[5] & ~op[4] & op[3] & op[2] & ~op[1] & ~op[0] & ~(~z)) |           // 001100 BNE
      (~op[5] & ~op[4] & op[3] & op[2] & ~op[1] & op[0] & ~(~z & ~(n ^ v))) | // 001101 BGT
      (~op[5] & ~op[4] & op[3] & op[2] & op[1] & ~op[0] & ~(n ^ v)) |         // 001110 BLT
      (~op[5] & ~op[4] & op[3] & op[2] & op[1] & op[0] & ~(~(n ^ v))) |       // 001111 BGE
      (~op[5] & op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0] & ~(z | (n ^ v)))   // 010000 BLE
    )) |
    qout[37] |
    qout[42] |
    qout[45]
  ), .q(qout[2]));

  ffd_OneHot S3   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[2]
  ), .q(qout[3]));

  // 2. LDR Reg, #Immediate
  ffd_OneHot S4   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & ~op[4] & ~op[3] & ~op[2] & ~op[1] & op[0]) // 000001
  ), .q(qout[4]));

  ffd_OneHot S5   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[4] & ~ra
  ), .q(qout[5]));

  ffd_OneHot S6   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[4] & ra
  ), .q(qout[6]));

  // 3. LDA Reg, Offset
  ffd_OneHot S7   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & ~ra &
    (~op[5] & ~op[4] & ~op[3] & ~op[2] & op[1] & ~op[0]) // 000010
  ), .q(qout[7]));

  ffd_OneHot S8   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & ra &
    (~op[5] & ~op[4] & ~op[3] & ~op[2] & op[1] & ~op[0]) // 000010
  ), .q(qout[8]));

  ffd_OneHot S9   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[7] |
    qout[8]
  ), .q(qout[9]));

  ffd_OneHot S10  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[9] |
    (qout[10] & ~ack_alu)
  ), .q(qout[10]));

  ffd_OneHot S11  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[10] & ack_alu
  ), .q(qout[11]));

  ffd_OneHot S12  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[11]
  ), .q(qout[12]));

  // 6. LDA #Immediate
  ffd_OneHot S13  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & ~op[4] & ~op[3] & op[2] & ~op[1] & op[0]) // 000101
  ), .q(qout[13]));

  ffd_OneHot S14  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[13]
  ), .q(qout[14]));

  // 4. STR Reg, #Immediate
  ffd_OneHot S15  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & ~op[4] & ~op[3] & ~op[2] & op[1] & op[0]) // 000011
  ), .q(qout[15]));

  ffd_OneHot S16  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[15] & ~ra
  ), .q(qout[16]));

  ffd_OneHot S17  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[15] & ra
  ), .q(qout[17 ]));

  // 5. STA Reg, Offset
  ffd_OneHot S18  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & ~ra &
    (~op[5] & ~op[4] & ~op[3] & op[2] & ~op[1] & ~op[0]) // 000100
  ), .q(qout[18 ]));

  ffd_OneHot S19  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & ra &
    (~op[5] & ~op[4] & ~op[3] & op[2] & ~op[1] & ~op[0]) // 000100
  ), .q(qout[19]));

  ffd_OneHot S20  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[18] |
    qout[19]
  ), .q(qout[20]));

  ffd_OneHot S21  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[20] |
    (qout[21] & ~ack_alu)
  ), .q(qout[21]));

  ffd_OneHot S22  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[21] & ack_alu
  ), .q(qout[22]));

  ffd_OneHot S23  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[22]
  ), .q(qout[23]));

  // 7. STA #Immediate
  ffd_OneHot S24  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & ~op[4] & ~op[3] & op[2] & op[1] & ~op[0]) // 000110
  ), .q(qout[24]));

  ffd_OneHot S25  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[24]
  ), .q(qout[25]));

  // 8. PSH {Acc, Reg, PC}
  ffd_OneHot S26  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & ~op[4] & ~op[3] & op[2] & op[1] & op[0]) // 000111
  ), .q(qout[26]));

  ffd_OneHot S27  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[26]
  ), .q(qout[27]));

  ffd_OneHot S28  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[27]
  ), .q(qout[28]));
  
  // 9. POP {Acc, Reg, PC}
  ffd_OneHot S29  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & ~op[4] & op[3] & ~op[2] & ~op[1] & ~op[0]) // 001000
  ), .q(qout[29]));

  ffd_OneHot S30  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[29]
  ), .q(qout[30]));

  ffd_OneHot S31  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[30]
  ), .q(qout[31]));

  // 10. IN Reg
  ffd_OneHot S32  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & ~op[4] & op[3] & ~op[2] & ~op[1] & op[0]) // 001001
  ), .q(qout[32]));

  ffd_OneHot S33  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[32] |
    (qout[33] & ~inp_ack)
  ), .q(qout[33]));

  ffd_OneHot S34  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[33] & inp_ack
  ), .q(qout[34]));

  // 11. OUT Reg
  ffd_OneHot S35  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & ~op[4] & op[3] & ~op[2] & op[1] & ~op[0]) // 001010
  ), .q(qout[35]));

  ffd_OneHot S36  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[35] |
    (qout[36] & ~out_ack)
  ), .q(qout[36]));
  
  // 12. - 18. BEQ, BNE, BGT, BLT, BGE, BLE, BRA
  ffd_OneHot S37  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & (
      (~op[5] & ~op[4] & op[3] & ~op[2] & op[1] & op[0] & (z)) | // 001011 BEQ
      (~op[5] & ~op[4] & op[3] & op[2] & ~op[1] & ~op[0] & (~z)) | // 001100 BNE
      (~op[5] & ~op[4] & op[3] & op[2] & ~op[1] & op[0] & (~z & ~(n ^ v))) | // 001101 BGT
      (~op[5] & ~op[4] & op[3] & op[2] & op[1] & ~op[0] & (n ^ v)) | // 001110 BLT
      (~op[5] & ~op[4] & op[3] & op[2] & op[1] & op[0] & (~(n ^ v))) | // 001111 BGE
      (~op[5] & op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0] & (z | (n ^ v))) | // 010000 BLE
      (~op[5] & op[4] & ~op[3] & ~op[2] & ~op[1] & op[0]) // 010001 BRA
    )
  ), .q(qout[37]));

  // 19. JMP
  ffd_OneHot S38  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & ~op[3] & ~op[2] & op[1] & ~op[0]) // 010010
  ), .q(qout[38]));

  ffd_OneHot S39  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[38]
  ), .q(qout[39]));

  ffd_OneHot S40  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[39]
  ), .q(qout[40]));
  
  ffd_OneHot S41  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[40]
  ), .q(qout[41]));
  
  ffd_OneHot S42  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[41]
  ), .q(qout[42]));
  
  // 20. RET
  ffd_OneHot S43  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & ~op[3] & ~op[2] & op[1] & op[0]) // 010011
  ), .q(qout[43]));
  
  ffd_OneHot S44  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[43]
  ), .q(qout[44]));
  
  ffd_OneHot S45  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[44]
  ), .q(qout[45]));
  
  assign finish = qout[0];
  assign c[0] = qout[1];
  assign c[1] = qout[2];
  assign c[2] = qout[3];
  assign c[3] = qout[4] | qout[13];
  assign c[4] = qout[5];
  assign c[5] = qout[6];
  assign c[6] = qout[7] | qout[18];
  assign c[7] = qout[8] | qout[19];
  assign c[8] = qout[9] | qout[20];
  assign c[9] = qout[11];
  assign c[10] = qout[12] | qout[14];
  assign c[11] = qout[15] | qout[24];
  assign c[12] = qout[16];
  assign c[13] = qout[17];
  assign c[14] = qout[22];
  assign c[15] = qout[23] | qout[25];
  assign c[16] = qout[26] | qout[39];
  assign c[17] = qout[27] | qout[40];
  assign c[18] = qout[28];
  assign c[19] = qout[29] | qout[43];
  assign c[20] = qout[30] | qout[44];
  assign c[21] = qout[31];
  assign c[22] = qout[32];
  assign c[23] = qout[34];
  assign c[24] = qout[35];
  assign c[25] = qout[37] | qout[42];
  assign c[26] = qout[38];
  assign c[27] = qout[41];
  assign c[28] = qout[45];
  // signals to be continued

endmodule