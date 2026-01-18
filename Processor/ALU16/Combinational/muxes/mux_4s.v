module mux_4s #(
    parameter w=4
)(
  input  [w-1:0] d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15,
  input  [3:0]   sel,
  output [w-1:0] o
);
    assign o = (sel == 4'b0000) ? d0:
               (sel == 4'b0001) ? d1:
               (sel == 4'b0010) ? d2:
               (sel == 4'b0011) ? d3:
               (sel == 4'b0100) ? d4:
               (sel == 4'b0101) ? d5:
               (sel == 4'b0110) ? d6:
               (sel == 4'b0111) ? d7:
               (sel == 4'b1000) ? d8:
               (sel == 4'b1001) ? d9:
               (sel == 4'b1010) ? d10:
               (sel == 4'b1011) ? d11:
               (sel == 4'b1100) ? d12:
               (sel == 4'b1101) ? d13:
               (sel == 4'b1110) ? d14:
               (sel == 4'b1111) ? d15: {w{1'bz}};

endmodule