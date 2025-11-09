`include "./comp4.v"
module comp4_tb;

reg [3:0] a, b;
wire eq, gt, lt;

comp4 comp(
	.a(a),
	.b(b),
	.eq(eq),
	.gt(gt),
	.lt(lt)
);

initial begin
	$dumpfile("dump.vcd");
	$dumpvars;
end

integer i, j;
initial begin
  for(i = 0; i < 16; i = i + 1) 
  begin
		a = i;
		for(j = 0; j < 16; j = j + 1) 
		begin
			b = j;
			#10;
			$display("a-%b b-%b %b%b%b", a, b, lt, eq, gt);
		end
		#10;
  end
end


endmodule