module AC(
    input clk, rst_b,
    input en,
    input [15:0] in,
    output[15:0] out 
);
    // Wire intern pentru bypass
    wire [15:0] qout;
 
    ffd  f0(.clk(clk), .rst_b(rst_b), .en(en), .d(in[0] ), .q(qout[0] ));
    ffd  f1(.clk(clk), .rst_b(rst_b), .en(en), .d(in[1] ), .q(qout[1] ));
    ffd  f2(.clk(clk), .rst_b(rst_b), .en(en), .d(in[2] ), .q(qout[2] ));
    ffd  f3(.clk(clk), .rst_b(rst_b), .en(en), .d(in[3] ), .q(qout[3] ));
    ffd  f4(.clk(clk), .rst_b(rst_b), .en(en), .d(in[4] ), .q(qout[4] ));
    ffd  f5(.clk(clk), .rst_b(rst_b), .en(en), .d(in[5] ), .q(qout[5] ));
    ffd  f6(.clk(clk), .rst_b(rst_b), .en(en), .d(in[6] ), .q(qout[6] ));
    ffd  f7(.clk(clk), .rst_b(rst_b), .en(en), .d(in[7] ), .q(qout[7] ));
    ffd  f8(.clk(clk), .rst_b(rst_b), .en(en), .d(in[8] ), .q(qout[8] ));
    ffd  f9(.clk(clk), .rst_b(rst_b), .en(en), .d(in[9] ), .q(qout[9] ));
    ffd f10(.clk(clk), .rst_b(rst_b), .en(en), .d(in[10]), .q(qout[10]));
    ffd f11(.clk(clk), .rst_b(rst_b), .en(en), .d(in[11]), .q(qout[11]));
    ffd f12(.clk(clk), .rst_b(rst_b), .en(en), .d(in[12]), .q(qout[12]));
    ffd f13(.clk(clk), .rst_b(rst_b), .en(en), .d(in[13]), .q(qout[13]));
    ffd f14(.clk(clk), .rst_b(rst_b), .en(en), .d(in[14]), .q(qout[14]));
    ffd f15(.clk(clk), .rst_b(rst_b), .en(en), .d(in[15]), .q(qout[15]));

    // Output cu BYPASS - valoarea noua e disponibila imediat!
    assign out = en ? in : qout;
    
endmodule