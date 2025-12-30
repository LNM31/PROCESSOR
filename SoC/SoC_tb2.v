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
  task automatic wait_finish(input integer max_cycles);
    integer cnt;
    begin
      cnt = 0;
      while (cnt < max_cycles) begin
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
  
  // Funcție care returnează numărul stării active (0-25) din one-hot
  function [4:0] get_state_num;
    input [25:0] qout;
    integer i;
    begin
      get_state_num = 5'd31;  // Default: invalid
      for (i = 0; i < 26; i = i + 1) begin
        if (qout[i]) get_state_num = i[4:0];
      end
    end
  endfunction

  // Funcție care returnează numele stării
  function [79:0] decode_state;  // 10 caractere * 8 = 80 biți
    input [25:0] qout;
    begin
      case (get_state_num(qout))
        5'd0:  decode_state = "S0_IDLE   ";
        5'd1:  decode_state = "S1_START  ";
        5'd2:  decode_state = "S2_FETCH1 ";
        5'd3:  decode_state = "S3_DECODE ";
        5'd4:  decode_state = "S4_LDR1   ";
        5'd5:  decode_state = "S5_LDR_X  ";
        5'd6:  decode_state = "S6_LDR_Y  ";
        5'd7:  decode_state = "S7_LDA_X  ";
        5'd8:  decode_state = "S8_LDA_Y  ";
        5'd9:  decode_state = "S9_LDA2   ";
        5'd10: decode_state = "S10_ALU   ";
        5'd11: decode_state = "S11_LDA3  ";
        5'd12: decode_state = "S12_LDA4  ";
        5'd13: decode_state = "S13_LDAI1 ";
        5'd14: decode_state = "S14_LDAI2 ";
        5'd15: decode_state = "S15_STR1  ";
        5'd16: decode_state = "S16_STR_X ";
        5'd17: decode_state = "S17_STR_Y ";
        5'd18: decode_state = "S18_STA_X ";
        5'd19: decode_state = "S19_STA_Y ";
        5'd20: decode_state = "S20_STA2  ";
        5'd21: decode_state = "S21_ALU2  ";
        5'd22: decode_state = "S22_STA3  ";
        5'd23: decode_state = "S23_STA4  ";
        5'd24: decode_state = "S24_STAI1 ";
        5'd25: decode_state = "S25_STAI2 ";
        default: decode_state = "UNKNOWN   ";
      endcase
    end
  endfunction
  
  // Funcție care returnează lista semnalelor de control active
  function [159:0] decode_ctrl;  // 20 caractere
    input [15:0] ctrl;
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
  wire [25:0] STATE = uut.cpu.cu.qout;  // Stările FF (One-Hot)
  wire [15:0] CTRL  = uut.cpu.c;                   // Semnale de control
  
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
      $display("              CTRL[15:0]=%d%d%d%d_%d%d%d%d_%d%d%d%d_%d%d%d%d (c15..c0)", 
               CTRL[15], CTRL[14], CTRL[13], CTRL[12],
               CTRL[11], CTRL[10], CTRL[9], CTRL[8],
               CTRL[7], CTRL[6], CTRL[5], CTRL[4],
               CTRL[3], CTRL[2], CTRL[1], CTRL[0]);
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

    // Program în memorie (din program.hex):
    // Addr | Hex  | Instrucțiune
    // -----|------|------------------------------------------
    // 0000 | 0408 | LDR X, #08     -> X = Mem[08] = 1234
    // 0001 | 0A09 | LDA ACC, Y+09  -> ACC = Mem[Y+09] = Mem[09] = 1235 (Y=0)
    // 0002 | 140A | LDA ACC, #0A   -> ACC = Mem[0A] = 1236
    // 0003 | 0C0B | STR X, #0B     -> Mem[0B] = X = 1234
    // 0004 | 120C | STA Y+0C       -> Mem[Y+0C] = Mem[0C] = ACC = 1236 (Y=0)
    // 0005 | 180D | STA #0D        -> Mem[0D] = ACC = 1236
    // 0006 | 040D | LDR X, #0D     -> X = Mem[0D] = 1236 (valoarea scrisa anterior)
    // 0007 | 0000 | HLT            -> Stop
    // 0008 | 1234 | Data
    // 0009 | 1235 | Data
    // 000A | 1236 | Data
    // 000B | 1237 | Data (va fi suprascris cu 1234)
    // 000C | 1238 | Data (va fi suprascris cu 1236)
    // 000D | 1239 | Data (va fi suprascris cu 1236)

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
    wait_finish(100);  // max 200 cicluri

    // ========================================
    // Verificări finale
    // ========================================
    $display("\n");
    $display("╔═══════════════════════════════════════════════════════════════════════════════╗");
    $display("║                         VERIFICARI FINALE                                     ║");
    $display("╚═══════════════════════════════════════════════════════════════════════════════╝");
    $display("\n");

    // Verifică registre
    $display("[TEST] Verificare registre:\n");
    check("X final         ", X,  16'h1236);  // LDR X, #0D -> X = Mem[0D] = 1236 (ultima instr.)
    check("AC final        ", AC, 16'h1236);  // LDA ACC, #0A -> ACC = Mem[0A] = 1236
    check("Y final         ", Y,  16'h0000);  // Y nu a fost modificat

    // Verifică memoria (după STR și STA)
    $display("\n[TEST] Verificare memorie:\n");
    check_mem(9'h00B, 16'h1234);  // STR X, #0B -> Mem[0B] = X = 1234
    check_mem(9'h00C, 16'h1236);  // STA Y+0C -> Mem[0C] = ACC = 1236
    check_mem(9'h00D, 16'h1236);  // STA #0D -> Mem[0D] = ACC = 1236

    // Date originale (nu trebuie modificate)
    check_mem(9'h008, 16'h1234);  // Date originale
    check_mem(9'h009, 16'h1235);  // Date originale
    check_mem(9'h00A, 16'h1236);  // Date originale

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
