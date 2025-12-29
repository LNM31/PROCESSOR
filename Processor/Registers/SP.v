module SP(
    input clk, rst_b,
    input ld, inc, dec,
    input [15:0] in,
    output [15:0] out
);
    // Wire intern pentru valorile curente din FFD-uri
    wire [15:0] qout;

    // Logica de selectie cu prioritate: ld > inc > dec
    // sel = 00 -> hold (qout)
    // sel = 01 -> load (in)
    // sel = 10 -> increment (qout + 1)
    // sel = 11 -> decrement (qout - 1)
    wire [1:0] sel;
    assign sel[0] = ld | (dec & ~inc & ~ld);
    assign sel[1] = (inc & ~ld) | (dec & ~inc & ~ld);

    // Increment: qout + 1 folosind RCA
    wire [16:0] incremented;
    wire co_inc, ovf_inc;
    RCA rca_inc(
        .x({1'b0, qout}),
        .y(17'b0),
        .ci(1'b1),
        .z(incremented),
        .co(co_inc),
        .overflow(ovf_inc)
    );

    // Decrement: qout - 1 folosind RCA (qout + 0xFFFF + 0 = qout - 1 in complement fata de 2)
    wire [16:0] decremented;
    wire co_dec, ovf_dec;
    RCA rca_dec(
        .x({1'b0, qout}),
        .y({1'b0, 16'hFFFF}),
        .ci(1'b0),
        .z(decremented),
        .co(co_dec),
        .overflow(ovf_dec)
    );

    // Mux pentru selectia datelor de intrare in registru
    wire [15:0] mux_out;
    mux_2s #(.w(16)) mux_sel(
        .d0(qout),                  // sel=00: hold
        .d1(in),                    // sel=01: load
        .d2(incremented[15:0]),     // sel=10: increment
        .d3(decremented[15:0]),     // sel=11: decrement
        .sel(sel),
        .o(mux_out)
    );

    // Registru din 16 flip-flop-uri
    // Enable mereu activ - mux-ul decide ce valoare se scrie
    ffd  f0(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[0] ), .q(qout[0] ));
    ffd  f1(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[1] ), .q(qout[1] ));
    ffd  f2(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[2] ), .q(qout[2] ));
    ffd  f3(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[3] ), .q(qout[3] ));
    ffd  f4(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[4] ), .q(qout[4] ));
    ffd  f5(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[5] ), .q(qout[5] ));
    ffd  f6(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[6] ), .q(qout[6] ));
    ffd  f7(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[7] ), .q(qout[7] ));
    ffd  f8(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[8] ), .q(qout[8] ));
    ffd  f9(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[9] ), .q(qout[9] ));
    ffd f10(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[10]), .q(qout[10]));
    ffd f11(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[11]), .q(qout[11]));
    ffd f12(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[12]), .q(qout[12]));
    ffd f13(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[13]), .q(qout[13]));
    ffd f14(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[14]), .q(qout[14]));
    ffd f15(.clk(clk), .rst_b(rst_b), .en(1'b1), .d(mux_out[15]), .q(qout[15]));

    // Output cu BYPASS - valoarea noua e disponibila imediat!
    wire writing = ld | inc | dec;
    assign out = writing ? mux_out : qout;
endmodule
