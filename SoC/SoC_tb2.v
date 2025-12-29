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
  function [79:0] decode_state;  // 10 caractere * 8 = 80 biți
    input [25:0] qout;
    begin
      casez (qout)
        26'b?????????????????????????1: decode_state = "S0_IDLE   ";
        26'b????????????????????????10: decode_state = "S1_START  ";
        26'b???????????????????????100: decode_state = "S2_FETCH1 ";
        26'b??????????????????????1000: decode_state = "S3_DECODE ";
        26'b?????????????????????10000: decode_state = "S4_LDR1   ";
        26'b????????????????????100000: decode_state = "S5_LDR_X  ";
        26'b???????????????????1000000: decode_state = "S6_LDR_Y  ";
        26'b??????????????????10000000: decode_state = "S7_LDA_X  ";
        26'b?????????????????100000000: decode_state = "S8_LDA_Y  ";
        26'b????????????????1000000000: decode_state = "S9_LDA2   ";
        26'b???????????????10000000000: decode_state = "S10_ALU   ";
        26'b??????????????100000000000: decode_state = "S11_LDA3  ";
        26'b?????????????1000000000000: decode_state = "S12_LDA4  ";
        26'b????????????10000000000000: decode_state = "S13_LDAI1 ";
        26'b???????????100000000000000: decode_state = "S14_LDAI2 ";
        26'b??????????1000000000000000: decode_state = "S15_STR1  ";
        26'b?????????10000000000000000: decode_state = "S16_STR_X ";
        26'b????????100000000000000000: decode_state = "S17_STR_Y ";
        26'b???????1000000000000000000: decode_state = "S18_STA_X ";
        26'b??????10000000000000000000: decode_state = "S19_STA_Y ";
        26'b?????100000000000000000000: decode_state = "S20_STA2  ";
        26'b????1000000000000000000000: decode_state = "S21_ALU2  ";
        26'b???10000000000000000000000: decode_state = "S22_STA3  ";
        26'b??100000000000000000000000: decode_state = "S23_STA4  ";
        26'b?1000000000000000000000000: decode_state = "S24_STAI1 ";
        26'b10000000000000000000000000: decode_state = "S25_STAI2 ";
        default:                         decode_state = "UNKNOWN   ";
      endcase
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
      $display("  Ciclu: %4d | Stare: %s | finish=%b", cycle_count, decode_state(STATE), finish);
      $display("───────────────────────────────────────────────────────────────────────────────");
      $display("  [REGISTRE]  PC=%04h  IR=%04h  AC=%04h  AR=%04h", PC, IR, AC, AR);
      $display("              X=%04h   Y=%04h   SP=%04h  FLAGS=%04b", X, Y, SP, FLAGS);
      $display("  [ALU]       SEU=%04h  ALU_OUT=%04h", SEU, ALU_OUT);
      $display("  [CONTROL]   STATE=%026b", STATE);
      $display("              CTRL=%016b", CTRL);
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
    // 0000 | 0407 | LDR X, #07     -> X = Mem[07] = 1234
    // 0001 | 0A08 | LDA ACC, Y+08  -> ACC = Mem[Y+08] = Mem[08] = 1235 (Y=0)
    // 0002 | 1409 | LDA ACC, #09   -> ACC = Mem[09] = 1236
    // 0003 | 0C0A | STR X, #0A     -> Mem[0A] = X = 1234
    // 0004 | 120B | STA Y+0B       -> Mem[Y+0B] = Mem[0B] = ACC = 1236 (Y=0)
    // 0005 | 180C | STA #0C        -> Mem[0C] = ACC = 1236
    // 0006 | 0000 | HLT            -> Stop
    // 0007 | 1234 | Data
    // 0008 | 1235 | Data
    // 0009 | 1236 | Data
    // 000A | 1237 | Data (va fi suprascris cu 1234)
    // 000B | 1238 | Data (va fi suprascris cu 1236)
    // 000C | 1239 | Data (va fi suprascris cu 1236)

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
    check("X final         ", X,  16'h1234);  // LDR X, #07 -> X = Mem[07] = 1234
    check("AC final        ", AC, 16'h1236);  // LDA ACC, #09 -> ACC = Mem[09] = 1236
    check("Y final         ", Y,  16'h0000);  // Y nu a fost modificat

    // Verifică memoria (după STR și STA)
    $display("\n[TEST] Verificare memorie:\n");
    check_mem(9'h00A, 16'h1234);  // STR X, #0A -> Mem[0A] = X = 1234
    check_mem(9'h00B, 16'h1236);  // STA Y+0B -> Mem[0B] = ACC = 1236
    check_mem(9'h00C, 16'h1236);  // STA #0C -> Mem[0C] = ACC = 1236

    // Date originale (nu trebuie modificate)
    check_mem(9'h007, 16'h1234);  // Date originale
    check_mem(9'h008, 16'h1235);  // Date originale
    check_mem(9'h009, 16'h1236);  // Date originale

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
