`timescale 1ns/1ns

// Testbench pentru Input Unit și Output Unit
// Simulează comportamentul CU-ului: trimite req, așteaptă ack

`include "input_unit.v"
`include "output_unit.v"

module io_tb;
    reg clk;
    reg rst_b;
    
    // Semnale Input Unit
    reg inp_req;
    wire [15:0] inp_data;
    wire inp_ack;
    
    // Semnale Output Unit
    reg out_req;
    reg [15:0] out_data;
    wire out_ack;
    
    // Instanțiere Input Unit (fără fișier = stdin)
    input_unit #(
        .DW(16)
    ) inp (
        .clk(clk),
        .rst_b(rst_b),
        .inp_req(inp_req),
        .inp_data(inp_data),
        .inp_ack(inp_ack)
    );
    
    // Instanțiere Output Unit
    output_unit #(
        .DW(16)
    ) outp (
        .clk(clk),
        .rst_b(rst_b),
        .out_req(out_req),
        .out_data(out_data),
        .out_ack(out_ack)
    );
    
    // Clock generation
    initial clk = 0;
    always #50 clk = ~clk;  // 100ns perioadă
    
    // Task pentru a simula un INP request (ca CU-ul)
    task do_input;
        output [15:0] value;
        begin
            $display("\n[TB] === INP Request ===");
            @(posedge clk);
            inp_req <= 1'b1;
            
            // Așteaptă ack
            wait(inp_ack == 1'b1);
            @(posedge clk);
            value = inp_data;
            $display("[TB] Primit: %0d (0x%h)", inp_data, inp_data);
            
            // Dezactivează req
            inp_req <= 1'b0;
            
            // Așteaptă ack să cadă
            wait(inp_ack == 1'b0);
            @(posedge clk);
            $display("[TB] INP complet\n");
        end
    endtask
    
    // Task pentru a simula un OUT request (ca CU-ul)
    task do_output;
        input [15:0] value;
        begin
            $display("\n[TB] === OUT Request: %0d ===", value);
            @(posedge clk);
            out_data <= value;
            out_req <= 1'b1;
            
            // Așteaptă ack
            wait(out_ack == 1'b1);
            @(posedge clk);
            
            // Dezactivează req
            out_req <= 1'b0;
            
            // Așteaptă ack să cadă
            wait(out_ack == 1'b0);
            @(posedge clk);
            $display("[TB] OUT complet\n");
        end
    endtask
    
    // Variabilă pentru valoarea citită
    reg [15:0] read_value;
    
    // Test scenario
    initial begin
        $dumpfile("io_dump.vcd");
        $dumpvars(0, io_tb);
        
        // Inițializare
        rst_b = 0;
        inp_req = 0;
        out_req = 0;
        out_data = 0;
        
        // Reset
        #200;
        rst_b = 1;
        #100;
        
        $display("========================================");
        $display("    Test I/O Units - Mod Interactiv    ");
        $display("========================================\n");
        
        // Test 1: Citește o valoare de la utilizator
        $display("Test 1: Citire de la utilizator");
        do_input(read_value);
        $display("[TB] Valoare stocată: %0d", read_value);
        
        // Test 2: Afișează valoarea citită
        $display("Test 2: Afișare valoare citită");
        do_output(read_value);
        
        // Test 3: Afișează o valoare fixă
        $display("Test 3: Afișare valoare fixă (12345)");
        do_output(16'd12345);
        
        // Test 4: Citește altă valoare
        $display("Test 4: A doua citire");
        do_input(read_value);
        
        // Test 5: Afișează noua valoare
        $display("Test 5: Afișare a doua valoare");
        do_output(read_value);
        
        #500;
        $display("\n========================================");
        $display("         Test I/O Complet!              ");
        $display("========================================");
        $finish;
    end

endmodule
