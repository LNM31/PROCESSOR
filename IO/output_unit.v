`timescale 1ns/1ns

// Output Unit - displays values in terminal (decimal, 16 bits)
// Protocol: CU sends out_req=1 + out_data, Output Unit displays and responds with out_ack=1

module output_unit #(
    parameter DW = 16
)(
    input               clk,
    input               rst_b,
    input               out_req,    // request from CU
    input      [DW-1:0] out_data,   // value to display
    output reg          out_ack     // acknowledge to CU
);

    // State machine
    localparam IDLE = 2'b00;
    localparam WRITE = 2'b01;
    localparam DONE = 2'b10;
    
    reg [1:0] state;
    reg [DW-1:0] data_out;

    // FSM for displaying
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
                        data_out <= out_data;
                    end
                end
                
                WRITE: begin
                    // Display in terminal
                    $display("OUTPUT_UNIT: %0d", data_out);
                    state <= DONE;
                end

                DONE: begin
                    out_ack <= 1'b1;
                    if (!out_req) begin
                        // CU has seen the ack and deactivated req
                        //out_ack <= 1'b0;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule
