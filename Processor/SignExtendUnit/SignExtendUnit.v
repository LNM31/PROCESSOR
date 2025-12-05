module sign_extend_unit (
    input  [8:0]  imm,      // valoare imediata (max 9 biti)
    input  [1:0]  sel,      // selector: 00=9b, 01=8b, 10=5b, 11=zero_ext
    output reg [15:0] out   // output extins pe 16 biti
);

    always @(*) begin
        case (sel)
            2'b00: // Sign extend 9 biti
                out = {{7{imm[8]}}, imm[8:0]};
            
            2'b01: // Sign extend 8 biti
                out = {{8{imm[7]}}, imm[7:0]};
            
            2'b10: // Sign extend 5 biti
                out = {{11{imm[4]}}, imm[4:0]};
            
            2'b11: // Zero extend 9 biti
                out = {7'b0, imm[8:0]};
            
            default:
                out = 16'b0;
        endcase
    end

endmodule