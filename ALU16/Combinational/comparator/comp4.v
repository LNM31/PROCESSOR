module comparator_1bit
(
    input A,
    input B,
    input A_greater_in,
    input A_equal_in,
    input A_less_in,
    output A_greater_out,
    output A_equal_out,
    output A_less_out
);

assign A_greater_out = (A_greater_in) | (A_equal_in &  A & ~B);
assign A_less_out    = (A_less_in)    | (A_equal_in & ~A &  B);
assign A_equal_out   = (A_equal_in & ~(A ^ B));

endmodule

module comp4
(
    input [3:0] a,
    input [3:0] b,
    output lt,
    output eq,
    output gt
);

wire [3:0] greater;
wire [3:0] equal;
wire [3:0] less;

comparator_1bit comp3(a[3], b[3], 1'b0, 1'b1, 1'b0, greater[3], equal[3], less[3]);
comparator_1bit comp2(a[2], b[2], greater[3], equal[3], less[3], greater[2], equal[2], less[2]);
comparator_1bit comp1(a[1], b[1], greater[2], equal[2], less[2], greater[1], equal[1], less[1]);
comparator_1bit comp0(a[0], b[0], greater[1], equal[1], less[1], gt, eq, lt);

endmodule
