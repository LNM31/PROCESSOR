module mux_3s #(
    parameter w=4
)(
  input  [w-1:0] d0, d1, d2, d3, d4, d5, d6, d7,
  input  [2:0]   sel,
  output [w-1:0] o
);
    assign o = (sel == 3'b000) ? d0:
               (sel == 3'b001) ? d1:
               (sel == 3'b010) ? d2:
               (sel == 3'b011) ? d3:
               (sel == 3'b100) ? d4:
               (sel == 3'b101) ? d5:
               (sel == 3'b110) ? d6:
               (sel == 3'b111) ? d7: {w{1'bz}};

endmodule