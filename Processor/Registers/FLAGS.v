module FLAGS(
    input clk, rst_b,
    input en,
    input [3:0] in,
    output[3:0] out 
);
    // Wire intern pentru bypass
    wire [3:0] qout;
 
    ffd  f0(.clk(clk), .rst_b(rst_b), .en(en), .d(in[0] ), .q(qout[0] ));
    ffd  f1(.clk(clk), .rst_b(rst_b), .en(en), .d(in[1] ), .q(qout[1] ));
    ffd  f2(.clk(clk), .rst_b(rst_b), .en(en), .d(in[2] ), .q(qout[2] ));
    ffd  f3(.clk(clk), .rst_b(rst_b), .en(en), .d(in[3] ), .q(qout[3] ));

    // Output cu BYPASS - valoarea noua e disponibila imediat!
    assign out = en ? in : qout;
    
endmodule