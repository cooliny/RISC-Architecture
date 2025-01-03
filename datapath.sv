module datapath(writenum, write, readnum, clk, loada, loadb, loadc, loads, shift, ALUop, asel, bsel, vsel, ZNV_out, datapath_out, mdata, sximm8, sximm5, PC);

   input  [15:0] mdata, sximm8, sximm5;
   input  [2:0]	 writenum, readnum;
   input  [1:0]	 shift, ALUop, vsel;
   input	 write, clk, loada, loadb, loadc, loads, asel, bsel;
   input  [8:0]	 PC;
   output [15:0] datapath_out;
   output [2:0]	 ZNV_out;

   wire [15:0]	 data_in, data_out, in, Ain, Bin, out, sout, loadaout, C;
   wire [2:0]	 ZNV;	

   assign datapath_out = C;
   
   // Main datapath modules
   regfile REGFILE(data_in, writenum, write, readnum, clk, data_out);
   shifter U1(in, shift, sout);
   ALU U2(Ain, Bin, ALUop, out, ZNV);

   // Load enable datapath modules
   register_load_enable regA(data_out, loada, clk, loadaout);
   register_load_enable regB(data_out, loadb, clk, in);
   register_load_enable regC(out, loadc, clk, C);
   register_load_enable #(3) regZ(ZNV, loads, clk, ZNV_out); 

   // MUX datapath modules
   MUX4 V(mdata, sximm8, {7'b0, PC}, C, vsel, data_in);
   MUX2 A(16'b0, loadaout, asel, Ain);
   MUX2 B(sximm5, sout, bsel, Bin);

endmodule


module MUX4(a, b, c, d, select, out);
   input [15:0] a; // mdata
   input [15:0]	b; // sximm8
   input [15:0]	c; // {8'b9, PC}
   input [15:0]	d; // C 
   input [1:0]	select;
   output reg [15:0] out;

   always_comb begin
      case(select)
	2'b00: out = a;
	2'b01: out = b;
	2'b10: out = c;
	2'b11: out = d;
        default: out = 16'bxxxx_xxxx_xxxx_xxxx;
      endcase
   end
endmodule // MUX4


module MUX2(a, b, select, out);

   parameter n = 16;
   input[n-1:0] a; // The '1' input
   input[n-1:0] b; // The '0' input
   input       select;
   output reg[n-1:0]  out;
   
   always_comb begin
      case(select)
        1'b1: out = a;
        1'b0: out = b;
	default: out = {(n){1'bx}};
      endcase
   end
endmodule
