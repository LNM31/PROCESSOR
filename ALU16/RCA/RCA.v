module fac(
    input x, y, ci,
    output z, co
);
    assign z = x ^ y ^ ci;
    assign co = (x & y) | (x & ci) | (y & ci);
endmodule

module RCA(
    input [16:0] x, y,
    input ci,
    output [16:0] z,
    output co,
    output overflow
);
    wire [15:0]w_co;
    genvar i;
    generate
        for(i = 0; i < 17; i = i + 1)
        begin
            if(i == 0)
            fac f(
                .x(x[0]),
                .y(y[0]),
                .ci(ci),
                .z(z[0]),
                .co(w_co[0])
            );
            else if(i == 16)
            fac f(
                .x(x[16]),
                .y(y[16]),
                .ci(w_co[15]),
                .z(z[16]),
                .co(co)
            );
            else
            fac f(
                .x(x[i]),
                .y(y[i]),
                .ci(w_co[i-1]),
                .z(z[i]),
                .co(w_co[i])
            );
        end
    endgenerate

    assign overflow = w_co[15] ^ w_co[14];
endmodule
