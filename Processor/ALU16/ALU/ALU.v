// `include"../Combinational/RCA/RCA.v"
// `include"../Control_Unit/CU_Extended2.v"
// `include"../Registers/A.v"
// `include"../Registers/Q.v"
// `include"../Registers/M.v"
// `include"../Combinational/muxes/mux_2s.v"
// `include"../Combinational/muxes/mux_3s.v"
// `include"../Registers/ffd.v"
// `include"../Registers/counter.v"

module ALU (
    input clk, rst_b, start,
    input  [3:0]  s,
    input  [15:0] inbus,
    output [15:0] outbus,
    output negative, zero, carry, overflow,
    output finish
);
    wire [15:0] q_out, m_out;
    wire [16:0] a_out;
    wire [3:0] cnt_out;
    wire q_1out;
    wire [18:0] c;
    wire [16:0] z;
    wire overfl;
    wire eq;

    A regA(
        .clk(clk),
        .rst_b(rst_b),
        .q15(q_out[15]),
        .sel( {c[0] | c[2] | c[3] | c[15] | c[16] | c[17] | c[18] | c[9], c[0] | c[2] | c[3] | c[15] | c[16] | c[17] | c[18] | c[6]} ),
        .in(
            {17{c[0]} } & 17'd0 | 
            {17{c[2]} } & q_out | 
            {17{c[3]} } & z |
            {17{c[15]}} & ({1'b0, q_out} & {1'b0, m_out}) |
            {17{c[16]}} & ({1'b0, q_out} | {1'b0, m_out}) |
            {17{c[17]}} & ({1'b0, q_out} ^ {1'b0, m_out}) |
            {17{c[18]}} & ({1'b0, ~q_out})
        ),
        .out(a_out)
    );

    Q regQ(
        .clk(clk),
        .rst_b(rst_b),
        .a0(a_out[0]),
        .sel( {c[11] | c[13] | c[14] ,c[1] | c[9] | c[10] | c[12] | c[14] , c[1] | c[6] | c[10] | c[13]} ),
        .in(
            {16{c[1]}} & inbus | 
            {q_out[15:1], (~a_out[16])} & {16{c[10]}} 
        ),
        .out(q_out),
        .q_1(q_1out)
    );

    M regM(
        .clk(clk),
        .rst_b(rst_b),
        .en(c[0]),
        .in(inbus),
        .out(m_out)
    );

    RCA adder(
        .x({1'b0, m_out} ^ {17{c[4]}}),
        .y(a_out),
        .ci(c[4]),
        .z(z),
        .co(carry),
        .overflow(overfl)
    );

    comp4 comparator(
        .a(cnt_out),
        .b(m_out[3:0]),
        .lt(),
        .eq(eq),
        .gt()
    );

    counter COUNT(
        .clk(clk),
        .rst_b(rst_b),
        .c_up(c[7]),
        .out(cnt_out)
    );

    Control_Unit CU(
        .clk(clk),
        .rst_b(rst_b),
        .s(s),
        .start(start),
        .q0(q_out[0]),
        .q_1(q_1out),
        .a_16(a_out[16]),
        .cmp_cnt_m4(eq),
        .cnt(cnt_out),
        .c(c),
        .finish(finish)
    );
    
    assign outbus={1'b0, {16{c[5]}}} & a_out | {{16{c[8]}}} & q_out;

    assign negative = a_out[15];
    assign zero = ~(a_out[15] | a_out[14] | a_out[13] | a_out[12] | a_out[11] | a_out[10] | a_out[9] | a_out[8] | a_out[7] | a_out[6] | a_out[5] | a_out[4] | a_out[3] | a_out[2] | a_out[1] | a_out[0]);
    assign overflow = overfl & ~s[3] & ~s[2] & ~s[1] & c[3]; 

endmodule