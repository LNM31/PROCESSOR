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
        $display("[WAIT_FINISH] CPU terminat dupa %0d cicluri", cnt);
      else
        $display("[WAIT_FINISH] TIMEOUT dupa %0d cicluri!", cnt);
    end
  endtask

  // ==========================================
  // Funcții pentru decodare stare (One-Hot)
  // ==========================================
  
  // Funcție care returnează numărul stării active (0-45) din one-hot
  function [5:0] get_state_num;
    input [45:0] qout;
    integer i;
    begin
      get_state_num = 6'd63;  // Default: invalid
      for (i = 0; i < 46; i = i + 1) begin
        if (qout[i]) get_state_num = i[5:0];
      end
    end
  endfunction

  // Funcție care returnează numele stării
  function [79:0] decode_state;  // 10 caractere * 8 = 80 biți
    input [45:0] qout;
    begin
      case (get_state_num(qout))
        6'd0:  decode_state = "S0_IDLE   ";
        6'd1:  decode_state = "S1_START  ";
        6'd2:  decode_state = "S2_FETCH1 ";
        6'd3:  decode_state = "S3_DECODE ";
        6'd4:  decode_state = "S4_LDR1   ";
        6'd5:  decode_state = "S5_LDR_X  ";
        6'd6:  decode_state = "S6_LDR_Y  ";
        6'd7:  decode_state = "S7_LDA_X  ";
        6'd8:  decode_state = "S8_LDA_Y  ";
        6'd9:  decode_state = "S9_LDA2   ";
        6'd10: decode_state = "S10_ALU   ";
        6'd11: decode_state = "S11_LDA3  ";
        6'd12: decode_state = "S12_LDA4  ";
        6'd13: decode_state = "S13_LDAI1 ";
        6'd14: decode_state = "S14_LDAI2 ";
        6'd15: decode_state = "S15_STR1  ";
        6'd16: decode_state = "S16_STR_X ";
        6'd17: decode_state = "S17_STR_Y ";
        6'd18: decode_state = "S18_STA_X ";
        6'd19: decode_state = "S19_STA_Y ";
        6'd20: decode_state = "S20_STA2  ";
        6'd21: decode_state = "S21_ALU2  ";
        6'd22: decode_state = "S22_STA3  ";
        6'd23: decode_state = "S23_STA4  ";
        6'd24: decode_state = "S24_STAI1 ";
        6'd25: decode_state = "S25_STAI2 ";
        6'd26: decode_state = "S26_PSH1  ";
        6'd27: decode_state = "S27_PSH2  ";
        6'd28: decode_state = "S28_PSH3  ";
        6'd29: decode_state = "S29_POP1  ";
        6'd30: decode_state = "S30_POP2  ";
        6'd31: decode_state = "S31_POP3  ";
        6'd32: decode_state = "S32_INP1  ";
        6'd33: decode_state = "S33_INP2  ";
        6'd34: decode_state = "S34_INP3  ";
        6'd35: decode_state = "S35_OUT1  ";
        6'd36: decode_state = "S36_OUT2  ";
        6'd37: decode_state = "S37_BRANCH";
        6'd38: decode_state = "S38_JMP1  ";
        6'd39: decode_state = "S39_JMP2  ";
        6'd40: decode_state = "S40_JMP3  ";
        6'd41: decode_state = "S41_JMP4  ";
        6'd42: decode_state = "S42_JMP5  ";
        6'd43: decode_state = "S43_RET1  ";
        6'd44: decode_state = "S44_RET2  ";
        6'd45: decode_state = "S45_RET3  ";
        default: decode_state = "UNKNOWN   ";
      endcase
    end
  endfunction
  
  // Funcție care returnează lista semnalelor de control active
  function [159:0] decode_ctrl;  // 20 caractere
    input [28:0] ctrl;
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
  wire [45:0] STATE = uut.cpu.cu.qout;  // Stările FF (One-Hot) - 46 stări (S0-S45)
  wire [28:0] CTRL  = uut.cpu.c;        // Semnale de control - 29 semnale
  
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
  
  always @(posedge clk) begin
    #0; // Delta delay - așteaptă NBA să se stabilizeze
    if (rst_b) begin
      cycle_count = cycle_count + 1;
      $display("═══════════════════════════════════════════════════════════════════════════════");
      $display("  Ciclu: %4d | Stare: S%0d (%s) | finish=%b", cycle_count, get_state_num(STATE), decode_state(STATE), finish);
      $display("───────────────────────────────────────────────────────────────────────────────");
      $display("  [REGISTRE]  PC=%04h  IR=%04h  AC=%04h  AR=%04h", PC, IR, AC, AR);
      $display("              X=%04h   Y=%04h   SP=%04h  FLAGS=%04b", X, Y, SP, FLAGS);
      $display("  [ALU]       SEU=%04h  ALU_OUT=%04h", SEU, ALU_OUT);
      $display("  [CONTROL]   STATE_NUM=%2d  CTRL_ACTIVE=%s", get_state_num(STATE), decode_ctrl(CTRL));
      $display("              CTRL[28:0]=%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d%d (c28..c0)",
               CTRL[28], CTRL[27], CTRL[26], CTRL[25],
               CTRL[24], CTRL[23], CTRL[22], CTRL[21],
               CTRL[20], CTRL[19], CTRL[18], CTRL[17],
               CTRL[16], CTRL[15], CTRL[14], CTRL[13],
               CTRL[12], CTRL[11], CTRL[10], CTRL[9],
               CTRL[8], CTRL[7], CTRL[6], CTRL[5],
               CTRL[4], CTRL[3], CTRL[2], CTRL[1], CTRL[0]);
      $display("  [MEMORY]    ADDR=%04h  DIN=%04h  DOUT=%04h  RD=%b  WR=%b", 
               MEM_ADDR, MEM_DIN, MEM_DOUT, MEM_RD, MEM_WR);
    end
  end

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
    $display("\n");
    $display("╔═══════════════════════════════════════════════════════════════════════════════╗");
    $display("║                         SoC TESTBENCH - START                                 ║");
    $display("╚═══════════════════════════════════════════════════════════════════════════════╝");
    $display("\n");

    // =========================================================================
    // TEST PROGRAM - LDR, LDA, STR, STA, PUSH, POP, JMP, RET, BRA, BNE, BEQ
    // =========================================================================
    // SP initial = 512 (0x200), stack creste in jos
    // =========================================================================
    //
    // INSTRUCTIUNI TESTATE:
    // [x] LDR X/Y       - Load register direct
    // [x] LDA_IMM       - Load accumulator direct
    // [x] STR X/Y       - Store register direct
    // [x] STA_IMM       - Store accumulator direct
    // [x] PUSH AC/X/Y   - Push pe stack
    // [x] POP AC/X/Y    - Pop de pe stack
    // [x] JMP           - Jump to subroutine (PUSH PC+1, then branch)
    // [x] RET           - Return from subroutine (POP PC)
    // [x] BRA           - Branch always
    // [x] BNE           - Branch if not equal (Z=0)
    // [x] BEQ           - Branch if equal (Z=1) - test NOT taken
    // [x] HLT           - Halt

    // ========================================
    // RESET
    // ========================================
    $display("\n[TEST] Aplicare RESET...\n");
    do_reset(3);

    // Verifică starea inițială
    $display("\n[TEST] Verificare stare dupa RESET:\n");
    check("PC dupa reset   ", PC, 16'h0000);
    check("IR dupa reset   ", IR, 16'h0000);
    check("AC dupa reset   ", AC, 16'h0000);
    check("X dupa reset    ", X,  16'h0000);
    check("Y dupa reset    ", Y,  16'h0000);

    // ========================================
    // START CPU
    // ========================================
    $display("\n[TEST] Pornire CPU (start=1)...\n");
    cpu_start();

    // ========================================
    // Execuție - așteaptă HLT
    // ========================================
    $display("\n[TEST] Executie program - astept HLT...\n");
    wait_finish(300);  // max 300 cicluri


    // ========================================
    // Verificări finale
    // ========================================
    $display("\n");
    $display("╔═══════════════════════════════════════════════════════════════════════════════╗");
    $display("║                         VERIFICARI FINALE                                     ║");
    $display("╚═══════════════════════════════════════════════════════════════════════════════╝");
    $display("\n");

    // --- Test 1: PUSH values on stack ---
    // Nota: Mem[1FF] este suprascris de JMP (salveaza PC+1=0x10 pentru RET)
    $display("\n[TEST] Verificare STACK (valorile PUSH-uite + JMP return addr):\n");
    check_mem(9'h1FF, 16'h0010);  // JMP suprascrie cu PC+1 = 0x0010 (return address)
    check_mem(9'h1FE, 16'h000A);  // PUSH X -> Mem[1FE] = X = 0x000A (10)
    check_mem(9'h1FD, 16'h0014);  // PUSH Y -> Mem[1FD] = Y = 0x0014 (20)

    // --- Test 2: POP results stored in memory ---
    $display("\n[TEST] Verificare POP (rezultate stocate in memorie):\n");
    check_mem(9'h040, 16'h000A);  // STR X, #40 -> Mem[40] = X = 0x000A (dupa POP X)
    check_mem(9'h041, 16'h0014);  // STR Y, #41 -> Mem[41] = Y = 0x0014 (dupa POP Y)
    check_mem(9'h042, 16'h001E);  // STA #42 -> Mem[42] = AC = 0x001E (dupa POP AC)

    // --- Test 3: JMP/RET - subroutine results ---
    $display("\n[TEST] Verificare JMP/RET (rezultate subroutine):\n");
    check_mem(9'h043, 16'h0055);  // STR X, #43 -> X = 0x0055 (setat in subroutine)
    check_mem(9'h044, 16'h00AA);  // STR Y, #44 -> Y = 0x00AA (setat in subroutine)

    // --- Test 4: BRA - branch always ---
    $display("\n[TEST] Verificare BRA (branch always):\n");
    check_mem(9'h045, 16'h001E);  // STA #45 -> Mem[45] = AC (BRA a sarit peste 0x13, 0x14)

    // --- Test 5: BNE - branch not equal (Z=0, should be taken) ---
    $display("\n[TEST] Verificare BNE (branch if Z=0 - TAKEN):\n");
    check_mem(9'h046, 16'h001E);  // STA #46 -> Mem[46] = AC (BNE a sarit peste 0x17, 0x18)

    // --- Test 6: BEQ - branch equal (Z=1, should NOT be taken since Z=0) ---
    $display("\n[TEST] Verificare BEQ (branch if Z=1 - NOT TAKEN):\n");
    check_mem(9'h047, 16'h001E);  // STA #47 -> Mem[47] = AC (BEQ NU a fost luat)

    // --- Test 7: Verify X and Y don't have "bad" values (branches worked) ---
    $display("\n[TEST] Verificare ca branch-urile au sarit instructiunile corecte:\n");
    check("X != 0xDEAD     ", (X != 16'hDEAD) ? 16'h0001 : 16'h0000, 16'h0001);
    check("Y != 0xBEEF     ", (Y != 16'hBEEF) ? 16'h0001 : 16'h0000, 16'h0001);

    // --- Test 8: Date originale neschimbate ---
    $display("\n[TEST] Verificare date originale (neschimbate):\n");
    check_mem(9'h030, 16'h000A);  // valoare initiala X = 10
    check_mem(9'h031, 16'h0014);  // valoare initiala Y = 20
    check_mem(9'h032, 16'h001E);  // valoare initiala AC = 30

    // --- Test 9: PC after HLT ---
    $display("\n[TEST] Verificare PC dupa HLT:\n");
    check("PC dupa HLT     ", PC, 16'h001D);  // HLT la 0x1C, PC incrementat la 0x1D

    // ========================================
    // Sumar
    // ========================================
    $display("\n");
    $display("╔═══════════════════════════════════════════════════════════════════════════════╗");
    $display("║                              SUMAR TESTE                                      ║");
    $display("╠═══════════════════════════════════════════════════════════════════════════════╣");
    $display("║  PASSED: %3d                                                                  ║", test_passed);
    $display("║  FAILED: %3d                                                                  ║", test_failed);
    $display("╚═══════════════════════════════════════════════════════════════════════════════╝");
    $display("\n");

    if (test_failed == 0)
      $display("✓ TOATE TESTELE AU TRECUT!\n");
    else
      $display("✗ EXISTA TESTE ESUATE!\n");

    #500;
    $finish;
  end

endmodule
