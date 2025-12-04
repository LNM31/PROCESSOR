`timescale 1ns/1ns

// Output Unit - afișează valori în terminal (decimal, 16 biți)
// Protocol: CU trimite out_req=1 + out_data, Output Unit afișează și răspunde cu out_ack=1

module output_unit #(
    parameter DW = 16
)(
    input               clk,
    input               rst_b,
    input               out_req,    // request de la CU
    input      [DW-1:0] out_data,   // valoarea de afișat
    output reg          out_ack     // acknowledge către CU
);

    // State machine
    localparam IDLE = 2'b00;
    localparam WRITE = 2'b01;
    localparam DONE = 2'b10;
    
    reg [1:0] state;

    // FSM pentru afișare
    always @(posedge clk or negedge rst_b) begin
        if (!rst_b) begin
            state <= IDLE;
            out_ack <= 1'b0;
        end
        else begin
            case (state)
                IDLE: begin
                    out_ack <= 1'b0;
                    if (out_req) begin
                        state <= WRITE;
                    end
                end
                
                WRITE: begin
                    // Afișează în terminal
                    $display("[OUTPUT_UNIT] OUT> %0d", out_data);
                    state <= DONE;
                end
                
                DONE: begin
                    out_ack <= 1'b1;
                    if (!out_req) begin
                        // CU a văzut ack-ul și a dezactivat req
                        out_ack <= 1'b0;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule
