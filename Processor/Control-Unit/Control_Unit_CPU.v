module Control_Unit_CPU(
  input clk, rst_b,           // clock and reset
  input [5:0] op,             // opcode from IR(Instruction Register)
  input ra,                   // RA - Register Address from IR
  input start,                // start from user
  input n, z, carry, v,       // flags
  input inp_ack,              // inp acknowledge from Input Unit
  input out_ack,              // out acknowledge from Output Unit
  input ack_alu,              // finish from ALU
  output finish,              // finish CPU
  output [122:0] c            // control signals for proccesor
);
  
  // Implementation OneHot

  wire [173:0] qout;
 
  // 1. HLT
  ffd_OneHot #(.reset_val(1'b1)) S0 (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    (qout[0] & ~start) | // S0 and start = 0
    qout[3] & (~op[5] & ~op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0]) // S3 and opcode = 000000
  ), .q(qout[0]));
  
  ffd_OneHot S1   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[0] & start
  ), .q(qout[1]));
  
  ffd_OneHot S2   (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[1]  | 
    qout[5]  | 
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
    qout[45] |
    qout[70] |
    qout[74] |
    qout[76] |
    qout[77] |
    qout[87] & (~(n ^ v)) |
    qout[114] & (~(n ^ v))
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
      (~op[5] & ~op[4] & op[3] & ~op[2] & op[1] & op[0] & (z)) |                // 001011 BEQ
      (~op[5] & ~op[4] & op[3] & op[2] & ~op[1] & ~op[0] & (~z)) |              // 001100 BNE
      (~op[5] & ~op[4] & op[3] & op[2] & ~op[1] & op[0] & (~z & ~(n ^ v))) |    // 001101 BGT
      (~op[5] & ~op[4] & op[3] & op[2] & op[1] & ~op[0] & (n ^ v)) |            // 001110 BLT
      (~op[5] & ~op[4] & op[3] & op[2] & op[1] & op[0] & (~(n ^ v))) |          // 001111 BGE
      (~op[5] & op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0] & (z | (n ^ v))) |    // 010000 BLE
      (~op[5] & op[4] & ~op[3] & ~op[2] & ~op[1] & op[0])                       // 010001 BRA
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

  // 21. ADD Reg/Imm, Acc = Acc + Reg/Imm
  ffd_OneHot S46  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & ~op[3] & op[2] & ~op[1] & ~op[0]) // 010100
  ), .q(qout[46]));
  
  // 22. SUB Reg/Imm, Acc = Acc - Reg/Imm
  ffd_OneHot S47  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & ~op[3] & op[2] & ~op[1] & op[0]) // 010101
  ), .q(qout[47]));
  
  // 23. MUL Reg/Imm, Acc = Acc * Reg/Imm
  ffd_OneHot S48  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & ~op[3] & op[2] & op[1] & ~op[0]) // 010110
  ), .q(qout[48]));
  
  // 24. DIV Reg/Imm, Acc = Acc / Reg/Imm
  ffd_OneHot S49  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & ~op[3] & op[2] & op[1] & op[0]) // 010111
  ), .q(qout[49]));
  
  // 25. MOD Reg/Imm, Acc = Acc % Reg/Imm
  ffd_OneHot S50  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & op[3] & ~op[2] & ~op[1] & ~op[0]) // 011000
  ), .q(qout[50]));
  
  // 26. AND Reg/Imm, Acc = Acc & Reg/Imm
  ffd_OneHot S51  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & op[3] & ~op[2] & ~op[1] & op[0]) // 011001
  ), .q(qout[51]));
  
  // 27. OR Reg/Imm, Acc = Acc | Reg/Imm
  ffd_OneHot S52  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & op[3] & ~op[2] & op[1] & ~op[0]) // 011010
  ), .q(qout[52]));
  
  // 28. XOR Reg/Imm, Acc = Acc ^ Reg/Imm
  ffd_OneHot S53  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & op[3] & ~op[2] & op[1] & op[0]) // 011011
  ), .q(qout[53]));
  
  // 29. NOT Reg/Imm, Acc = ~Reg/Imm
  ffd_OneHot S54  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (~op[5] & op[4] & op[3] & op[2] & ~op[1] & ~op[0]) // 011100
  ), .q(qout[54]));
  
  ffd_OneHot S78  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (
      (~op[5] & op[4] & op[3] & op[2] & ~op[1] & op[0]) |       // 011101
      (~op[5] & op[4] & op[3] & op[2] & op[1] & ~op[0]) |       // 011110
      (~op[5] & op[4] & op[3] & op[2] & op[1] & op[0])  |       // 011111
      (op[5] & ~op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0]) |    // 100000
      (op[5] & ~op[4] & ~op[3] & ~op[2] & ~op[1] & op[0]) |     // 100001
      (op[5] & ~op[4] & ~op[3] & ~op[2] & op[1] & ~op[0]) |     // 100010
      (op[5] & ~op[4] & ~op[3] & ~op[2] & op[1] & op[0]) |      // 100011
      (op[5] & ~op[4] & ~op[3] & op[2] & ~op[1] & ~op[0]) |     // 100100
      (op[5] & ~op[4] & ~op[3] & op[2] & ~op[1] & op[0])        // 100101
    ) 
  ), .q(qout[78]));

  // 30. ADD #address, Acc = Acc + Mem[address] 
  ffd_OneHot S55  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[78] & 
    (~op[5] & op[4] & op[3] & op[2] & ~op[1] & op[0]) // 011101
  ), .q(qout[55]));
  
  // 31. SUB #address, Acc = Acc - Mem[address] 
  ffd_OneHot S56  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[78] & 
    (~op[5] & op[4] & op[3] & op[2] & op[1] & ~op[0]) // 011110
  ), .q(qout[56]));
  
  // 32. MUL #address, Acc = Acc * Mem[address]
  ffd_OneHot S57  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[78] & 
    (~op[5] & op[4] & op[3] & op[2] & op[1] & op[0]) // 011111
  ), .q(qout[57]));
  
  // 33. DIV #address, Acc = Acc / Mem[address]
  ffd_OneHot S58  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[78] & 
    (op[5] & ~op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0]) // 100000
  ), .q(qout[58]));
  
  // 34. MOD #address, Acc = Acc % Mem[address]
  ffd_OneHot S59  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[78] & 
    (op[5] & ~op[4] & ~op[3] & ~op[2] & ~op[1] & op[0]) // 100001
  ), .q(qout[59]));
  
  // 35. AND #address, Acc = Acc & Mem[address]
  ffd_OneHot S60  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[78] & 
    (op[5] & ~op[4] & ~op[3] & ~op[2] & op[1] & ~op[0]) // 100010
  ), .q(qout[60]));
  
  // 36. OR #address, Acc = Acc | Mem[address]
  ffd_OneHot S61  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[78] & 
    (op[5] & ~op[4] & ~op[3] & ~op[2] & op[1] & op[0]) // 100011
  ), .q(qout[61]));
  
  // 37. XOR #address, Acc = Acc ^ Mem[address]
  ffd_OneHot S62  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[78] & 
    (op[5] & ~op[4] & ~op[3] & op[2] & ~op[1] & ~op[0]) // 100100
  ), .q(qout[62]));
  
  // 38. NOT #address, Acc = ~Mem[address]
  ffd_OneHot S63  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[78] & 
    (op[5] & ~op[4] & ~op[3] & op[2] & ~op[1] & op[0]) // 100101
  ), .q(qout[63]));
  
  // 39. LSR #address, Acc = Acc / 2^k(k = imm[3:0])
  ffd_OneHot S64  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (op[5] & ~op[4] & ~op[3] & op[2] & op[1] & ~op[0]) // 100110
  ), .q(qout[64]));
  
  // 40. LSL #address, Acc = Acc * 2^k(k = imm[3:0])
  ffd_OneHot S65  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (op[5] & ~op[4] & ~op[3] & op[2] & op[1] & op[0]) // 100111
  ), .q(qout[65]));
  
  // 41. RSR #address, Rotate Acc 1 bit Right
  ffd_OneHot S66  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (op[5] & ~op[4] & op[3] & ~op[2] & ~op[1] & ~op[0]) // 101000
  ), .q(qout[66]));
  
  // 42. RSR #address, Rotate Acc 1 bit Left
  ffd_OneHot S67  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (op[5] & ~op[4] & op[3] & ~op[2] & ~op[1] & op[0]) // 101001
  ), .q(qout[67]));
  
  ffd_OneHot S68  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[46] |
    qout[47] |
    qout[48] |
    qout[49] |
    qout[50] |
    qout[51] |
    qout[52] |
    qout[53] |
    qout[55] |
    qout[56] |
    qout[57] |
    qout[58] |
    qout[59] |
    qout[60] |
    qout[61] |
    qout[62] |
    qout[64] |
    qout[65] |
    qout[66] |
    qout[67] 
  ), .q(qout[68]));
  
  ffd_OneHot S69  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[68] |
    (qout[69] & ~ack_alu) |
    qout[54] |
    qout[63] 
  ), .q(qout[69]));
  
  ffd_OneHot S70  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[69] & ack_alu
  ), .q(qout[70]));
  
  // 43. CMP Reg, Reg,        Reg - Reg and sets the flags
  ffd_OneHot S71  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (op[5] & ~op[4] & op[3] & ~op[2] & op[1] & ~op[0]) // 101010
  ), .q(qout[71]));
  
  ffd_OneHot S72  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[71] |
    qout[75]
  ), .q(qout[72]));
  
  ffd_OneHot S73  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[72] |
    (qout[73] & ~ack_alu)
  ), .q(qout[73]));
  
  ffd_OneHot S74  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[73] & ack_alu
  ), .q(qout[74]));
  
  // 44.  TST Reg, Reg,        Reg & Reg and sets the flags
  ffd_OneHot S75  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (op[5] & ~op[4] & op[3] & ~op[2] & op[1] & op[0]) // 101011
  ), .q(qout[75]));
  
  // 45. MOV Reg, Reg       Reg[OP1] = Reg[OP2]
  ffd_OneHot S76  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (op[5] & ~op[4] & op[3] & op[2] & ~op[1] & op[0]) // 101101
  ), .q(qout[76]));
  
  // 46. MOV Reg, #imm       Reg[OP1] = imm[5:0]
  ffd_OneHot S77  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (op[5] & ~op[4] & op[3] & op[2] & ~op[1] & ~op[0]) // 101100
  ), .q(qout[77]));

  // ADDM, SUBM, ELMULM
  ffd_OneHot S79  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (
      (op[5] & ~op[4] & op[3] & op[2] & op[1] & ~op[0]) | // 101110
      (op[5] & ~op[4] & op[3] & op[2] & op[1] & op[0])  | // 101111
      (op[5] & op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0]) // 110000
    ) 
  ), .q(qout[79]));  

  ffd_OneHot S80  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[79]
  ), .q(qout[80]));

  ffd_OneHot S81  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[80]
  ), .q(qout[81]));
  
  ffd_OneHot S82  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[81] |
    (qout[82] & ~ack_alu) 
  ), .q(qout[82]));
  
  ffd_OneHot S83  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[82] & ack_alu
  ), .q(qout[83]));
  
  ffd_OneHot S84  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[83] | 
    qout[109]
  ), .q(qout[84]));
  
  ffd_OneHot S85  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[84]
  ), .q(qout[85]));
  
  ffd_OneHot S86  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[85] |
    (qout[86] & ~ack_alu)
  ), .q(qout[86]));
  
  ffd_OneHot S87  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[86] & ack_alu
  ), .q(qout[87]));
  
  ffd_OneHot S88  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[87] & ~(~(n ^ v))
  ), .q(qout[88]));
  
  ffd_OneHot S89  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[88]
  ), .q(qout[89]));
  
  ffd_OneHot S90  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[89]
  ), .q(qout[90]));
  
  ffd_OneHot S91  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[90]
  ), .q(qout[91]));
  
  ffd_OneHot S92  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[91]
  ), .q(qout[92]));
  
  ffd_OneHot S93  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[92]
  ), .q(qout[93]));
  
  ffd_OneHot S94  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[93] | 
    (qout[94] & ~ack_alu)
  ), .q(qout[94]));
  
  ffd_OneHot S95  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[94] & ack_alu
  ), .q(qout[95]));
  
  ffd_OneHot S96  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[95]
  ), .q(qout[96]));
  
  ffd_OneHot S97  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[96] | 
    (qout[97] & ~out_ack)
  ), .q(qout[97]));

  ffd_OneHot S98  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[97] & out_ack
  ), .q(qout[98]));

  ffd_OneHot S99  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[98]
  ), .q(qout[99]));

  ffd_OneHot S100  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[99] |
    (qout[100] & ~ack_alu)
  ), .q(qout[100]));

  ffd_OneHot S101  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[100] & ack_alu
  ), .q(qout[101]));

  ffd_OneHot S102  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[101]
  ), .q(qout[102]));

  ffd_OneHot S103  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[102]
  ), .q(qout[103]));

  ffd_OneHot S104  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[103] | 
    (qout[104] & ~ack_alu)
  ), .q(qout[104]));

  ffd_OneHot S105  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[104] & ack_alu
  ), .q(qout[105]));

  ffd_OneHot S106  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[105]
  ), .q(qout[106]));

  ffd_OneHot S107  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[106]
  ), .q(qout[107]));

  ffd_OneHot S108  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[107] |
    (qout[108] & ~ack_alu)
  ), .q(qout[108]));

  ffd_OneHot S109  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[108] & ack_alu
  ), .q(qout[109]));
  
  // MULM
  ffd_OneHot S110  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[3] & 
    (op[5] & op[4] & ~op[3] & ~op[2] & ~op[1] & op[0]) // 110001
     
  ), .q(qout[110]));
  
  ffd_OneHot S111  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[110] |
    qout[169]
  ), .q(qout[111]));
  
  ffd_OneHot S112  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[111]
  ), .q(qout[112]));
  
  ffd_OneHot S113  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[112] |
    (qout[113] & ~ack_alu)
  ), .q(qout[113]));
  
  ffd_OneHot S114  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[113] & ack_alu
  ), .q(qout[114]));
  
  ffd_OneHot S115  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[114] & ~(~(n ^ v)) |
    qout[165]
  ), .q(qout[115]));
  
  ffd_OneHot S116  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[115]
  ), .q(qout[116]));
  
  ffd_OneHot S117  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[116] |
    (qout[117] & ~ack_alu)
  ), .q(qout[117]));
  
  ffd_OneHot S118  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[117] & ack_alu
  ), .q(qout[118]));
  
  ffd_OneHot S119  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[118] & ~(~(n ^ v))
  ), .q(qout[119]));
  
  ffd_OneHot S120  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[119] |
    qout[159]
  ), .q(qout[120]));
  
  ffd_OneHot S121  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[120]
  ), .q(qout[121]));
  
  ffd_OneHot S122  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[121] |
    (qout[122] & ~ack_alu)
  ), .q(qout[122]));
  
  ffd_OneHot S123  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[122] & ack_alu
  ), .q(qout[123]));
  
  ffd_OneHot S124  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[123] & ~(~(n ^ v))
  ), .q(qout[124]));
  
  ffd_OneHot S125  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[124]
  ), .q(qout[125]));
  
  ffd_OneHot S126  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[125] |
    (qout[126] & ~ack_alu)
  ), .q(qout[126]));
  
  ffd_OneHot S127  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[126] & ack_alu
  ), .q(qout[127]));
  
  ffd_OneHot S128  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[127]
  ), .q(qout[128]));
  
  ffd_OneHot S129  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[128]
  ), .q(qout[129]));
  
  ffd_OneHot S130  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[129] |
    (qout[130] & ~ack_alu)
  ), .q(qout[130]));
  
  ffd_OneHot S131  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[130] & ack_alu
  ), .q(qout[131]));
  
  ffd_OneHot S132  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[131]
  ), .q(qout[132]));
  
  ffd_OneHot S133  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[132]
  ), .q(qout[133]));
  
  ffd_OneHot S134  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[133] |
    (qout[134] & ~ack_alu)
  ), .q(qout[134]));
  
  ffd_OneHot S135  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[134] & ack_alu
  ), .q(qout[135]));

  ffd_OneHot S170  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[135]
  ), .q(qout[170]));

  ffd_OneHot S171  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[170]
  ), .q(qout[171]));
  
  ffd_OneHot S136  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[171]
  ), .q(qout[136]));
  
  ffd_OneHot S137  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[136]
  ), .q(qout[137]));
  
  ffd_OneHot S138  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[137] |
    (qout[138] & ~ack_alu)
  ), .q(qout[138]));
  
  ffd_OneHot S139  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[138] & ack_alu
  ), .q(qout[139]));
  
  ffd_OneHot S140  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[139]
  ), .q(qout[140]));
  
  ffd_OneHot S141  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[140]
  ), .q(qout[141]));
  
  ffd_OneHot S142  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[141] |
    (qout[142] & ~ack_alu)
  ), .q(qout[142]));
  
  ffd_OneHot S143  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[142] & ack_alu
  ), .q(qout[143]));
  
  ffd_OneHot S144  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[143]
  ), .q(qout[144]));
  
  ffd_OneHot S145  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[144]
  ), .q(qout[145]));
  
  ffd_OneHot S146  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[145] |
    (qout[146] & ~ack_alu)
  ), .q(qout[146]));
  
  ffd_OneHot S147  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[146] & ack_alu
  ), .q(qout[147]));

  ffd_OneHot S172  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[147]
  ), .q(qout[172]));

  ffd_OneHot S173  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[172]
  ), .q(qout[173]));
  
  ffd_OneHot S148  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[173]
  ), .q(qout[148]));
  
  ffd_OneHot S149  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[148]
  ), .q(qout[149]));
  
  ffd_OneHot S150  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[149] |
    (qout[150] & ~ack_alu)
  ), .q(qout[150]));
  
  ffd_OneHot S151  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[150] & ack_alu
  ), .q(qout[151]));
  
  ffd_OneHot S152  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[151]
  ), .q(qout[152]));
  
  ffd_OneHot S153  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[152]
  ), .q(qout[153]));
  
  ffd_OneHot S154  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[153] |
    (qout[154] & ~ack_alu)
  ), .q(qout[154]));
  
  ffd_OneHot S155  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[154] & ack_alu
  ), .q(qout[155]));
  
  ffd_OneHot S156  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[155]
  ), .q(qout[156]));
  
  ffd_OneHot S157  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[156]
  ), .q(qout[157]));
  
  ffd_OneHot S158  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[157] |
    (qout[158] & ~ack_alu)
  ), .q(qout[158]));
  
  ffd_OneHot S159  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[158] & ack_alu
  ), .q(qout[159]));
  
  ffd_OneHot S160  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[123] & (~(n ^ v))
  ), .q(qout[160]));
  
  ffd_OneHot S161  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[160] |
    (qout[161] & ~out_ack)
  ), .q(qout[161]));
  
  ffd_OneHot S162  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[161] & out_ack
  ), .q(qout[162]));
  
  ffd_OneHot S163  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[162]
  ), .q(qout[163]));
  
  ffd_OneHot S164  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[163] |
    (qout[164] & ~ack_alu)
  ), .q(qout[164]));
  
  ffd_OneHot S165  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[164] & ack_alu
  ), .q(qout[165]));
  
  ffd_OneHot S166  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[118] & (~(n ^ v))
  ), .q(qout[166]));
  
  ffd_OneHot S167  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[166]
  ), .q(qout[167]));
  
  ffd_OneHot S168  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[167] |
    (qout[168] & ~ack_alu)
  ), .q(qout[168]));
  
  ffd_OneHot S169  (.clk(clk), .rst_b(rst_b), .en(1'b1), .d(
    qout[168] & ack_alu
  ), .q(qout[169]));



  assign finish  = qout[0];
  assign c[0]    = qout[1];
  assign c[1]    = qout[2];
  assign c[2]    = qout[3];
  assign c[3]    = qout[4] | qout[13];
  assign c[4]    = qout[5];
  assign c[5]    = qout[6];
  assign c[6]    = qout[7] | qout[18];
  assign c[7]    = qout[8] | qout[19];
  assign c[8]    = qout[9] | qout[20];
  assign c[9]    = qout[11];
  assign c[10]   = qout[12] | qout[14];
  assign c[11]   = qout[15] | qout[24];
  assign c[12]   = qout[16];
  assign c[13]   = qout[17];
  assign c[14]   = qout[22];
  assign c[15]   = qout[23] | qout[25];
  assign c[16]   = qout[26] | qout[39];
  assign c[17]   = qout[27] | qout[40];
  assign c[18]   = qout[28];
  assign c[19]   = qout[29] | qout[43];
  assign c[20]   = qout[30] | qout[44];
  assign c[21]   = qout[31];
  assign c[22]   = qout[32];
  assign c[23]   = qout[34];
  assign c[24]   = qout[35];
  assign c[25]   = qout[37] | qout[42];
  assign c[26]   = qout[46];
  assign c[27]   = qout[41];
  assign c[28]   = qout[45];
  assign c[29]   = qout[47];
  assign c[30]   = qout[48];
  assign c[31]   = qout[49];
  assign c[32]   = qout[50];
  assign c[33]   = qout[51];
  assign c[34]   = qout[52];
  assign c[35]   = qout[53];
  assign c[36]   = qout[54];
  assign c[37]   = qout[55];
  assign c[38]   = qout[56];
  assign c[39]   = qout[57];
  assign c[40]   = qout[58];
  assign c[41]   = qout[59];
  assign c[42]   = qout[60];
  assign c[43]   = qout[61];
  assign c[44]   = qout[62];
  assign c[45]   = qout[63];
  assign c[46]   = qout[64];
  assign c[47]   = qout[65];
  assign c[48]   = qout[66];
  assign c[49]   = qout[67];
  assign c[50]   = qout[68];
  assign c[51]   = qout[70] | qout[95];
  assign c[52]   = qout[71];
  assign c[53]   = qout[72];
  assign c[54]   = qout[75];
  assign c[55]   = qout[76];
  assign c[56]   = qout[77];
  assign c[57]   = qout[78];
  assign c[58]   = qout[79];
  assign c[59]   = qout[80];
  assign c[60]   = qout[81];
  assign c[61]   = qout[83];
  assign c[62]   = qout[84];
  assign c[63]   = qout[85];
  assign c[64]   = qout[88];
  assign c[65]   = qout[89];
  assign c[66]   = qout[90];
  assign c[67]   = qout[91];
  assign c[68]   = qout[92];
  assign c[69]   = qout[93];
  assign c[70]   = qout[98];
  assign c[71]   = qout[99] | qout[103] | qout[107];
  assign c[72]   = qout[101];
  assign c[73]   = qout[102];
  assign c[74]   = qout[105];
  assign c[75]   = qout[106];
  assign c[76]   = qout[96] | qout[160];
  assign c[77]   = qout[109];
  assign c[78]   = qout[110];
  assign c[79]   = qout[111];
  assign c[80]   = qout[112];
  assign c[81]   = qout[115];
  assign c[82]   = qout[116];
  assign c[83]   = qout[119];
  assign c[84]   = qout[120];
  assign c[85]   = qout[121];
  assign c[86]   = qout[124];
  assign c[87]   = qout[125];
  assign c[88]   = qout[127];
  assign c[89]   = qout[128];
  assign c[90]   = qout[129];
  assign c[91]   = qout[131];
  assign c[92]   = qout[132];
  assign c[93]   = qout[133];
  assign c[94]   = qout[135];
  assign c[95]   = qout[136];
  assign c[96]   = qout[137];
  assign c[97]   = qout[139];
  assign c[98]   = qout[140];
  assign c[99]   = qout[141];
  assign c[100]  = qout[143];
  assign c[101]  = qout[144];
  assign c[102]  = qout[145];
  assign c[103]  = qout[147];
  assign c[104]  = qout[148];
  assign c[105]  = qout[149];
  assign c[106]  = qout[151];
  assign c[107]  = qout[152];
  assign c[108]  = qout[153];
  assign c[109]  = qout[155];
  assign c[110]  = qout[156];
  assign c[111]  = qout[157];
  assign c[112]  = qout[159];
  assign c[113]  = qout[162];
  assign c[114]  = qout[163];
  assign c[115]  = qout[165];
  assign c[116]  = qout[166];
  assign c[117]  = qout[167];
  assign c[118]  = qout[169];
  assign c[119]  = qout[170];
  assign c[120]  = qout[171];
  assign c[121]  = qout[172];
  assign c[122]  = qout[173];

  // Decode current state in zecimal (0-173)
  reg [7:0] state_num;
  integer i;
  always @(*) begin
    state_num = 8'd255; // default: no active state
    for (i = 0; i < 174; i = i + 1) begin
      if (qout[i]) state_num = i[7:0];
    end
  end

  // Decode first active signal in zecimal (0-122)
  reg [7:0] signal_num;
  integer j;
  always @(*) begin
    signal_num = 8'd255; // default: no active signal
    for (j = 0; j < 123; j = j + 1) begin
      if (c[j]) signal_num = j[7:0];
    end
  end

endmodule