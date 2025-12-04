`timescale 1ns/1ns

// Input Unit - citește valori de la stdin (decimal, 16 biți)
// Protocol: CU trimite inp_req=1, Input Unit citește și răspunde cu inp_ack=1

module input_unit #(
    parameter DW = 16
)(
    input               clk,
    input               rst_b,
    input               inp_req,    // request de la CU
    output reg [DW-1:0] inp_data,   // valoarea citită
    output reg          inp_ack     // acknowledge către CU
);

    // State machine
    localparam IDLE = 2'b00;
    localparam READ = 2'b01;
    localparam DONE = 2'b10;
    
    reg [1:0] state;
    reg [DW-1:0] temp_data;
    integer scan_result;

    // FSM pentru citire
    always @(posedge clk or negedge rst_b) begin
        if (!rst_b) begin
            state <= IDLE;
            inp_data <= {DW{1'b0}};
            inp_ack <= 1'b0;
        end
        else begin
            case (state)
                IDLE: begin
                    inp_ack <= 1'b0;
                    if (inp_req)
                        state <= READ;
                end
                
                READ: begin
                    // Citește de la stdin (blocking)
                    $display("[INP] Introdu o valoare (decimal): ");
                    scan_result = $fscanf(32'h8000_0000, "%d", temp_data);
                    inp_data <= temp_data;
                    $display("[INP] Citit: %0d (0x%h)", temp_data, temp_data);
                    state <= DONE;
                end
                
                DONE: begin
                    inp_ack <= 1'b1;
                    if (!inp_req) begin
                        inp_ack <= 1'b0;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule