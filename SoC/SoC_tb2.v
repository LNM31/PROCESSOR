`timescale 1ns/1ps

module SoC_tb2;

  // ==========================================
  // Inputs
  // ==========================================
  reg clk, rst_b, start;

  // ==========================================
  // Outputs
  // ==========================================
  wire finish;

  // ==========================================
  // Instanțiere SoC
  // ==========================================
  SoC uut (
    .start(start),
    .clk(clk),
    .rst_b(rst_b),
    .finish(finish)
  );

  // ==========================================
  // Clock
  // ==========================================
  localparam integer CLK_PERIOD = 100; // ns
  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // ==========================================
  // Task-uri utilitare
  // ==========================================
  
  // Reset pentru N cicluri
  task automatic do_reset(input integer cycles);
    begin
      rst_b = 1'b0;
      start = 1'b0;
      repeat (cycles) @(posedge clk);
      rst_b = 1'b1;
      @(posedge clk); #0;
    end
  endtask

  // Avansează un ciclu + delta delay pentru stabilizare NBA
  task automatic step; 
    begin 
      @(posedge clk); #0; 
    end 
  endtask

  // Avansează N cicluri
  task automatic stepn(input integer n);
    integer i;
    begin
      for (i = 0; i < n; i = i + 1) step();
    end
  endtask

  // Pornește CPU-ul (puls pe start)
  task automatic cpu_start;
    begin
      start = 1'b1;
      step();
      start = 1'b0;
    end
  endtask

  // Așteaptă până finish sau timeout
  // finish = 1 în S0 (idle), 0 cât rulează, 1 când revine în S0 (terminat)
  task automatic wait_finish(input integer max_cycles);
    integer cnt;
    begin
      cnt = 0;
      // Așteaptă ca finish să devină 0 (CPU a pornit)
      while (finish == 1 && cnt < max_cycles) begin
        step();
        cnt = cnt + 1;
      end
      // Așteaptă ca finish să devină 1 (CPU s-a oprit, revine în S0)
      while (finish == 0 && cnt < max_cycles) begin
        step();
        cnt = cnt + 1;
      end
      if (finish)
        $display("CPU has finished after %0d clock cycles", cnt);
      else
        $display("TIMEOUT after %0d clock cycles!", cnt);
    end
  endtask

  // ==========================================
  // Funcții pentru decodare stare (One-Hot)
  // ==========================================
  
  // Funcție care returnează numărul stării active (0-78) din one-hot
  function [6:0] get_state_num;
    input [78:0] qout;
    integer i;
    begin
      get_state_num = 7'd127;  // Default: invalid
      for (i = 0; i < 79; i = i + 1) begin
        if (qout[i]) get_state_num = i[6:0];
      end
    end
  endfunction

  // Funcție care returnează numele stării
  function [79:0] decode_state;  // 10 caractere * 8 = 80 biți
    input [78:0] qout;
    begin
      case (get_state_num(qout))
        7'd0:  decode_state = "S0_IDLE   ";
        7'd1:  decode_state = "S1_START  ";
        7'd2:  decode_state = "S2_FETCH1 ";
        7'd3:  decode_state = "S3_DECODE ";
        7'd4:  decode_state = "S4_LDR1   ";
        7'd5:  decode_state = "S5_LDR_X  ";
        7'd6:  decode_state = "S6_LDR_Y  ";
        7'd7:  decode_state = "S7_LDA_X  ";
        7'd8:  decode_state = "S8_LDA_Y  ";
        7'd9:  decode_state = "S9_LDA2   ";
        7'd10: decode_state = "S10_ALU   ";
        7'd11: decode_state = "S11_LDA3  ";
        7'd12: decode_state = "S12_LDA4  ";
        7'd13: decode_state = "S13_LDAI1 ";
        7'd14: decode_state = "S14_LDAI2 ";
        7'd15: decode_state = "S15_STR1  ";
        7'd16: decode_state = "S16_STR_X ";
        7'd17: decode_state = "S17_STR_Y ";
        7'd18: decode_state = "S18_STA_X ";
        7'd19: decode_state = "S19_STA_Y ";
        7'd20: decode_state = "S20_STA2  ";
        7'd21: decode_state = "S21_ALU2  ";
        7'd22: decode_state = "S22_STA3  ";
        7'd23: decode_state = "S23_STA4  ";
        7'd24: decode_state = "S24_STAI1 ";
        7'd25: decode_state = "S25_STAI2 ";
        7'd26: decode_state = "S26_PSH1  ";
        7'd27: decode_state = "S27_PSH2  ";
        7'd28: decode_state = "S28_PSH3  ";
        7'd29: decode_state = "S29_POP1  ";
        7'd30: decode_state = "S30_POP2  ";
        7'd31: decode_state = "S31_POP3  ";
        7'd32: decode_state = "S32_INP1  ";
        7'd33: decode_state = "S33_INP2  ";
        7'd34: decode_state = "S34_INP3  ";
        7'd35: decode_state = "S35_OUT1  ";
        7'd36: decode_state = "S36_OUT2  ";
        7'd37: decode_state = "S37_BRANCH";
        7'd38: decode_state = "S38_JMP1  ";
        7'd39: decode_state = "S39_JMP2  ";
        7'd40: decode_state = "S40_JMP3  ";
        7'd41: decode_state = "S41_JMP4  ";
        7'd42: decode_state = "S42_JMP5  ";
        7'd43: decode_state = "S43_RET1  ";
        7'd44: decode_state = "S44_RET2  ";
        7'd45: decode_state = "S45_RET3  ";
        7'd46: decode_state = "S46_ADD_R ";
        7'd47: decode_state = "S47_SUB_R ";
        7'd48: decode_state = "S48_MUL_R ";
        7'd49: decode_state = "S49_DIV_R ";
        7'd50: decode_state = "S50_MOD_R ";
        7'd51: decode_state = "S51_AND_R ";
        7'd52: decode_state = "S52_OR_R  ";
        7'd53: decode_state = "S53_XOR_R ";
        7'd54: decode_state = "S54_NOT_R ";
        7'd55: decode_state = "S55_ADD_M ";
        7'd56: decode_state = "S56_SUB_M ";
        7'd57: decode_state = "S57_MUL_M ";
        7'd58: decode_state = "S58_DIV_M ";
        7'd59: decode_state = "S59_MOD_M ";
        7'd60: decode_state = "S60_AND_M ";
        7'd61: decode_state = "S61_OR_M  ";
        7'd62: decode_state = "S62_XOR_M ";
        7'd63: decode_state = "S63_NOT_M ";
        7'd64: decode_state = "S64_LSR   ";
        7'd65: decode_state = "S65_LSL   ";
        7'd66: decode_state = "S66_RSR   ";
        7'd67: decode_state = "S67_RSL   ";
        7'd68: decode_state = "S68_ALU0  ";
        7'd69: decode_state = "S69_ALU1  ";
        7'd70: decode_state = "S70_ALU2  ";
        7'd71: decode_state = "S71_CMP   ";
        7'd72: decode_state = "S72_CMP0  ";
        7'd73: decode_state = "S73_CMP1  ";
        7'd74: decode_state = "S74_CMP2  ";
        7'd75: decode_state = "S75_TST   ";
        7'd76: decode_state = "S76_MOV_RR";
        7'd77: decode_state = "S77_MOV_RI";
        7'd78: decode_state = "S78_MEM   ";
        default: decode_state = "UNKNOWN   ";
      endcase
    end
  endfunction
  
  // Funcție care returnează lista semnalelor de control active
  function [159:0] decode_ctrl;  // 20 caractere
    input [57:0] ctrl;
    begin
      decode_ctrl = "                    ";  // 20 spații
      if (ctrl[0])  decode_ctrl = "c0                  ";
      if (ctrl[1])  decode_ctrl = "c1                  ";
      if (ctrl[2])  decode_ctrl = "c2                  ";
      if (ctrl[3])  decode_ctrl = "c3                  ";
      if (ctrl[4])  decode_ctrl = "c4                  ";
      if (ctrl[5])  decode_ctrl = "c5                  ";
      if (ctrl[6])  decode_ctrl = "c6                  ";
      if (ctrl[7])  decode_ctrl = "c7                  ";
      if (ctrl[8])  decode_ctrl = "c8                  ";
      if (ctrl[9])  decode_ctrl = "c9                  ";
      if (ctrl[10]) decode_ctrl = "c10                 ";
      if (ctrl[11]) decode_ctrl = "c11                 ";
      if (ctrl[12]) decode_ctrl = "c12                 ";
      if (ctrl[13]) decode_ctrl = "c13                 ";
      if (ctrl[14]) decode_ctrl = "c14                 ";
      if (ctrl[15]) decode_ctrl = "c15                 ";
      if (ctrl[16]) decode_ctrl = "c16                 ";
      if (ctrl[17]) decode_ctrl = "c17                 ";
      if (ctrl[18]) decode_ctrl = "c18                 ";
      if (ctrl[19]) decode_ctrl = "c19                 ";
      if (ctrl[20]) decode_ctrl = "c20                 ";
      if (ctrl[21]) decode_ctrl = "c21                 ";
      if (ctrl[22]) decode_ctrl = "c22                 ";
      if (ctrl[23]) decode_ctrl = "c23                 ";
      if (ctrl[24]) decode_ctrl = "c24                 ";
      if (ctrl[25]) decode_ctrl = "c25                 ";
      if (ctrl[26]) decode_ctrl = "c26                 ";
      if (ctrl[27]) decode_ctrl = "c27                 ";
      if (ctrl[28]) decode_ctrl = "c28                 ";
      if (ctrl[29]) decode_ctrl = "c29                 ";
      if (ctrl[30]) decode_ctrl = "c30                 ";
      if (ctrl[31]) decode_ctrl = "c31                 ";
      if (ctrl[32]) decode_ctrl = "c32                 ";
      if (ctrl[33]) decode_ctrl = "c33                 ";
      if (ctrl[34]) decode_ctrl = "c34                 ";
      if (ctrl[35]) decode_ctrl = "c35                 ";
      if (ctrl[36]) decode_ctrl = "c36                 ";
      if (ctrl[37]) decode_ctrl = "c37                 ";
      if (ctrl[38]) decode_ctrl = "c38                 ";
      if (ctrl[39]) decode_ctrl = "c39                 ";
      if (ctrl[40]) decode_ctrl = "c40                 ";
      if (ctrl[41]) decode_ctrl = "c41                 ";
      if (ctrl[42]) decode_ctrl = "c42                 ";
      if (ctrl[43]) decode_ctrl = "c43                 ";
      if (ctrl[44]) decode_ctrl = "c44                 ";
      if (ctrl[45]) decode_ctrl = "c45                 ";
      if (ctrl[46]) decode_ctrl = "c46                 ";
      if (ctrl[47]) decode_ctrl = "c47                 ";
      if (ctrl[48]) decode_ctrl = "c48                 ";
      if (ctrl[49]) decode_ctrl = "c49                 ";
      if (ctrl[50]) decode_ctrl = "c50                 ";
      if (ctrl[51]) decode_ctrl = "c51                 ";
      if (ctrl[52]) decode_ctrl = "c52                 ";
      if (ctrl[53]) decode_ctrl = "c53                 ";
      if (ctrl[54]) decode_ctrl = "c54                 ";
      if (ctrl[55]) decode_ctrl = "c55                 ";
      if (ctrl[56]) decode_ctrl = "c56                 ";
      if (ctrl[57]) decode_ctrl = "c57                 ";
    end
  endfunction

  // ==========================================
  // Wire-uri pentru monitorizare ușoară
  // ==========================================
  wire [15:0] PC  = uut.cpu.pc_out;
  wire [15:0] IR  = uut.cpu.ir_out;
  wire [15:0] AC  = uut.cpu.ac_out;
  wire [15:0] AR  = uut.cpu.ar_out;
  wire [15:0] X   = uut.cpu.x_out;
  wire [15:0] Y   = uut.cpu.y_out;
  wire [15:0] SP  = uut.cpu.sp_out;
  wire [3:0]  FLAGS = uut.cpu.flags_out;
  wire [15:0] SEU = uut.cpu.seu_out;
  wire [15:0] ALU_OUT = uut.cpu.outbus_alu;
  
  // Control Unit
  wire [78:0] STATE = uut.cpu.cu.qout;  // Stările FF (One-Hot) - 79 stări (S0-S78)
  wire [57:0] CTRL  = uut.cpu.c;        // Semnale de control - 58 semnale
  
  // Memory
  wire [15:0] MEM_ADDR = uut.address;
  wire [15:0] MEM_DIN  = uut.mem_in_cpu_out;
  wire [15:0] MEM_DOUT = uut.mem_out_cpu_in;
  wire        MEM_RD   = uut.read;
  wire        MEM_WR   = uut.write;

  // ==========================================
  // VCD Dump
  // ==========================================
  initial begin
    $dumpfile("soc_tb2.vcd");
    $dumpvars(0, SoC_tb2);
  end

  // ==========================================
  // Monitor - Afișează la fiecare ciclu
  // ==========================================
  integer cycle_count;
  initial cycle_count = 0;
  
  // always @(posedge clk) begin
  //   #0; // Delta delay - așteaptă NBA să se stabilizeze
  //   if (rst_b) begin
  //     cycle_count = cycle_count + 1;
  //     $display("═══════════════════════════════════════════════════════════════════════════════");
  //     $display("  Ciclu: %4d | Stare: S%0d (%s) | finish=%b", cycle_count, get_state_num(STATE), decode_state(STATE), finish);
  //     $display("───────────────────────────────────────────────────────────────────────────────");
  //     $display("  [REGISTRE]  PC=%04h  IR=%04h  AC=%04h  AR=%04h", PC, IR, AC, AR);
  //     $display("              X=%04h   Y=%04h   SP=%04h  FLAGS=%04b", X, Y, SP, FLAGS);
  //     $display("  [ALU]       SEU=%04h  ALU_OUT=%04h", SEU, ALU_OUT);
  //     $display("  [CONTROL]   STATE_NUM=%2d  CTRL_ACTIVE=%s", get_state_num(STATE), decode_ctrl(CTRL));
  //     $display("              CTRL[57:0]=%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d",
  //              CTRL[57], CTRL[56],
  //              CTRL[55], CTRL[54], CTRL[53], CTRL[52],
  //              CTRL[51], CTRL[50], CTRL[49], CTRL[48],
  //              CTRL[47], CTRL[46], CTRL[45], CTRL[44],
  //              CTRL[43], CTRL[42], CTRL[41], CTRL[40],
  //              CTRL[39], CTRL[38], CTRL[37], CTRL[36],
  //              CTRL[35], CTRL[34], CTRL[33], CTRL[32]);
  //     $display("                          _%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d (c57..c0)",
  //              CTRL[31], CTRL[30], CTRL[29], CTRL[28],
  //              CTRL[27], CTRL[26], CTRL[25], CTRL[24],
  //              CTRL[23], CTRL[22], CTRL[21], CTRL[20],
  //              CTRL[19], CTRL[18], CTRL[17], CTRL[16],
  //              CTRL[15], CTRL[14], CTRL[13], CTRL[12],
  //              CTRL[11], CTRL[10], CTRL[9], CTRL[8],
  //              CTRL[7], CTRL[6], CTRL[5], CTRL[4],
  //              CTRL[3], CTRL[2], CTRL[1], CTRL[0]);
  //     $display("  [MEMORY]    ADDR=%04h  DIN=%04h  DOUT=%04h  RD=%b  WR=%b", 
  //              MEM_ADDR, MEM_DIN, MEM_DOUT, MEM_RD, MEM_WR);
  //   end
  // end

  // ==========================================
  // Contoare pentru verificări
  // ==========================================
  integer test_passed, test_failed;
  initial begin
    test_passed = 0;
    test_failed = 0;
  end

  // Task pentru verificare
  task automatic check(
    input [159:0] test_name,  // 20 caractere
    input [15:0] actual,
    input [15:0] expected
  );
    begin
      if (actual === expected) begin
        $display("[PASS] %s: got %04h (expected %04h)", test_name, actual, expected);
        test_passed = test_passed + 1;
      end else begin
        $display("[FAIL] %s: got %04h (expected %04h)", test_name, actual, expected);
        test_failed = test_failed + 1;
      end
    end
  endtask

  task automatic check_mem(
    input [8:0] addr,
    input [15:0] expected
  );
    begin
      if (uut.memory.mem[addr] === expected) begin
        $display("[PASS] Mem[%03h] = %04h (expected %04h)", addr, uut.memory.mem[addr], expected);
        test_passed = test_passed + 1;
      end else begin
        $display("[FAIL] Mem[%03h] = %04h (expected %04h)", addr, uut.memory.mem[addr], expected);
        test_failed = test_failed + 1;
      end
    end
  endtask

  // ==========================================
  // Test principal
  // ==========================================
  initial begin
    // $display("\n");
    // $display("╔═══════════════════════════════════════════════════════════════════════════════╗");
    // $display("║                         SoC TESTBENCH - START                                 ║");
    // $display("╚═══════════════════════════════════════════════════════════════════════════════╝");
    // $display("\n");

    // =========================================================================
    // TEST PROGRAM - Arithmetic, Logic, MOV, CMP (Opcodes 010100+)
    // =========================================================================
    //
    // INSTRUCTIUNI TESTATE:
    // [x] MOV Reg, #Imm (101101)
    // [x] MOV Reg, Reg (101100)
    // [x] ADD, SUB (Reg & Imm) (010100, 010101)
    // [x] MUL, DIV, MOD (Reg) (010110, 010111, 011000)
    // [x] AND, OR, XOR, NOT (Reg) (011001, 011010, 011011, 011100)
    // [x] LSR, LSL, RSR, RSL (100110, 100111, 101000, 101001)
    // [x] CMP, TST (101010, 101011)
    // [x] BEQ, BNE (cu flags de la CMP/TST)
    // [x] HLT - Halt

    // ========================================
    // RESET
    // ========================================
    // $display("\n[TEST] Aplicare RESET...\n");
    do_reset(3);

    // Verifică starea inițială
    // $display("\n[TEST] Verificare stare dupa RESET:\n");
    // check("PC dupa reset   ", PC, 16'h0000);
    // check("IR dupa reset   ", IR, 16'h0000);
    // check("AC dupa reset   ", AC, 16'h0000);
    // check("X dupa reset    ", X,  16'h0000);
    // check("Y dupa reset    ", Y,  16'h0000);

    // ========================================
    // START CPU
    // ========================================
    $display("\nCPU has started...\n");
    cpu_start();

    // ========================================
    // Execuție - așteaptă HLT
    // ========================================
    // $display("\n[TEST] Executie program - astept HLT...\n");
    wait_finish(5000);  // max 1000 cicluri pentru toate instructiunile


    // ========================================
    // Verificări finale
    // ========================================
    // $display("\n");
    // $display("╔═══════════════════════════════════════════════════════════════════════════════╗");
    // $display("║                         VERIFICARI FINALE                                     ║");
    // $display("╚═══════════════════════════════════════════════════════════════════════════════╝");
    // $display("\n");

    // // --- Test 1: MOV Reg, #Imm ---
    // $display("\n[TEST] Verificare MOV Reg, #Imm:\n");
    // check_mem(9'h050, 16'h000A);  // MOV AC, #10 -> STA -> Mem[50] = 10
    // check_mem(9'h051, 16'h0014);  // MOV X, #20 -> STR -> Mem[51] = 20
    // check_mem(9'h052, 16'h001E);  // MOV Y, #30 -> STR -> Mem[52] = 30

    // // --- Test 2: MOV Reg, Reg ---
    // $display("\n[TEST] Verificare MOV Reg, Reg:\n");
    // check_mem(9'h053, 16'h000F);  // MOV X, AC -> X = 15
    // check_mem(9'h054, 16'h000F);  // MOV Y, X -> Y = 15

    // // --- Test 3: ADD/SUB (Reg) ---
    // $display("\n[TEST] Verificare ADD/SUB (Reg):\n");
    // check_mem(9'h055, 16'h000F);  // ADD AC, X -> 10 + 5 = 15
    // check_mem(9'h056, 16'h000A);  // SUB AC, X -> 15 - 5 = 10

    // // --- Test 4: ADD/SUB (Imm) ---
    // $display("\n[TEST] Verificare ADD/SUB (Imm):\n");
    // check_mem(9'h057, 16'h0011);  // ADD AC, #7 -> 10 + 7 = 17
    // check_mem(9'h058, 16'h000E);  // SUB AC, #3 -> 17 - 3 = 14

    // // --- Test 5: MUL/DIV/MOD ---
    // $display("\n[TEST] Verificare MUL/DIV/MOD:\n");
    // check_mem(9'h059, 16'h000F);  // MUL AC, X -> 5 * 3 = 15
    // check_mem(9'h05A, 16'h0006);  // DIV AC, X -> 20 / 3 = 6
    // check_mem(9'h05B, 16'h0000);  // MOD AC, X -> 6 % 3 = 0

    // // --- Test 6: AND/OR/XOR ---
    // $display("\n[TEST] Verificare AND/OR/XOR:\n");
    // check_mem(9'h05C, 16'h0007);  // AND AC, X -> 0x0F & 0x07 = 0x07
    // check_mem(9'h05D, 16'h000F);  // OR AC, X -> 0x0F | 0x07 = 0x0F
    // check_mem(9'h05E, 16'h000F);  // XOR AC, X -> 0x08 ^ 0x07 = 0x0F

    // // --- Test 7: NOT ---
    // $display("\n[TEST] Verificare NOT:\n");
    // check_mem(9'h05F, 16'hFFF8);  // NOT X -> ~0x07 = 0xFFF8

    // // --- Test 8: LSR/LSL ---
    // $display("\n[TEST] Verificare LSR/LSL:\n");
    // check_mem(9'h060, 16'h0008);  // LSR #1 -> 16 >> 1 = 8
    // check_mem(9'h061, 16'h0010);  // LSL #2 -> 4 << 2 = 16

    // // --- Test 9: RSR/RSL ---
    // $display("\n[TEST] Verificare RSR/RSL:\n");
    // check_mem(9'h062, 16'h7FF8);  // RSR -> 0xA0 rotate right = 0x50
    // check_mem(9'h063, 16'h0014);  // RSL -> 0x05 rotate left = 0x0A

    // // --- Test 10: CMP + BEQ ---
    // $display("\n[TEST] Verificare CMP + BEQ:\n");
    // check_mem(9'h064, 16'h0001);  // CMP AC==X, BEQ taken -> marker = 1

    // // --- Test 11: TST + BNE ---
    // $display("\n[TEST] Verificare TST + BNE:\n");
    // check_mem(9'h065, 16'h0002);  // TST AC&X!=0, BNE taken -> marker = 2

    // // --- Test 12: PC after HLT ---
    // $display("\n[TEST] Verificare PC dupa HLT:\n");
    // check("PC dupa HLT     ", PC, 16'h0047);  // HLT la 0x46, PC incrementat la 0x47

    // // ========================================
    // // Sumar
    // // ========================================
    // $display("\n");
    // $display("╔═══════════════════════════════════════════════════════════════════════════════╗");
    // $display("║                              SUMAR TESTE                                      ║");
    // $display("╠═══════════════════════════════════════════════════════════════════════════════╣");
    // $display("║  PASSED: %3d                                                                  ║", test_passed);
    // $display("║  FAILED: %3d                                                                  ║", test_failed);
    // $display("╚═══════════════════════════════════════════════════════════════════════════════╝");
    // $display("\n");

    // if (test_failed == 0)
    //   $display("✓ TOATE TESTELE AU TRECUT!\n");
    // else
    //   $display("✗ EXISTA TESTE ESUATE!\n");

    #500;
    $finish;
  end

endmodule
