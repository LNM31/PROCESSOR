module tb_A;
    reg clk,rst_b;
    reg q7;
    reg [1:0]sel;
    reg [8:0]in;
    wire[8:0]out; 

    A a(.clk(clk),.rst_b(rst_b),.q7(q7),.sel(sel),.in(in),.out(out));

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars;
        clk=0;
        q7=0;
        rst_b=1;
        in=9'b100001111;
        sel=0;
    end

    initial begin
        #10; rst_b=1;
        #10; rst_b=0;
        #10; rst_b=1;
        #10; sel=3;
        #150; sel=1;
        #250; sel=2;
        #250; sel=0;
    end

    integer i;
    initial begin
        for(i=0;i<100;i=i+1)
        begin
            #50; clk=~clk;
        end
        #50;
    end
endmodule

module tb;

    reg [1:0]d0,d1,d2,d3;
    reg [1:0]sel;
    wire [1:0]o;

    mux_2s #(.w(2)) m(
        .d0(d0),
        .d1(d1),
        .d2(d2),
        .d3(d3),
        .sel(sel),
        .o(o)
    );
    integer i;
    initial begin
        d0=0;d1=1;d2=2;d3=3;
        sel=0;
        #50;
        sel=1;
        #50;
        sel=2;
        #50;
        sel=3;
        
    end

    initial begin
        $display("Time\td0\td1\td2\td3\tsel\to");
        $monitor("%0t\t%b\t%b\t%b\t%b\t%b\t%b",$time,d0,d1,d2,d3,sel,o);
    end
endmodule

module tb_encoder;

    reg [7:0]in;
    wire [2:0]out;

    encoder #(.w(3)) e(
        .in(in),
        .out(out)
    );  

    integer i;
    initial begin
        for (i=0;i<8;i=i+1)
        begin
            in=1<<i;
            #50;
        end
    end

    initial begin
        $display("Time\tin\tout");
        $monitor("%0t\t%b\t%b",$time,in,out);
    end
endmodule

module tb_M;
    reg clk,rst_b;
    reg en;
    reg [7:0]in;
    wire[7:0]out; 

    M a(.clk(clk),.rst_b(rst_b),.en(1'b1),.in(in),.out(out));

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars;
        clk=0;
        en=1;
        rst_b=1;
        in=8'b10001101;
    end

    initial begin
        #10; rst_b=1;
        #10; rst_b=0;
        #10; rst_b=1;
    end

    initial begin
        #320; en=0;
        #20; in=8'b1000001;
    end

    integer i;
    initial begin
        for(i=0;i<100;i=i+1)
        begin
            #50; clk=~clk;
        end
        #50;
    end
endmodule

module tb_Q;
    reg clk,rst_b;
    reg a0;
    reg [1:0]sel;
    reg [7:0]in;
    wire[7:0]out;
    wire q_1;

    Q Qq(.clk(clk),.rst_b(rst_b),.a0(a0),.sel(sel),.in(in),.out(out),.q_1(q_1));

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars;
        clk=0;
        a0=1;
        rst_b=1;
        in=8'b10001101;
        sel=0;
    end

    initial begin
        #10; rst_b=1;
        #10; rst_b=0;
        #10; rst_b=1;
        #10; sel=3;
        #150; sel=1;
        #250; sel=2;
        #250; sel=0;
    end

    initial begin
        #320; a0=0;
    end

    integer i;
    initial begin
        for(i=0;i<100;i=i+1)
        begin
            #50; clk=~clk;
        end
        #50;
    end
endmodule