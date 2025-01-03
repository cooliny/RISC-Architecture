`define r0 3'b000
`define r1 3'b001
`define r2 3'b010
`define r3 3'b011
`define r4 3'b100
`define r5 3'b101
`define r6 3'b110
`define r7 3'b111

`define r0_decode 8'b00000001
`define r1_decode 8'b00000010
`define r2_decode 8'b00000100
`define r3_decode 8'b00001000
`define r4_decode 8'b00010000
`define r5_decode 8'b00100000
`define r6_decode 8'b01000000
`define r7_decode 8'b10000000
  
module regfile(data_in, writenum, write, readnum, clk, data_out);

   input[15:0]   data_in;
   input [2:0]   writenum, readnum;
   input         write, clk;
   output [15:0] data_out;

   reg [15:0]	 data_out;
   reg [7:0]	 read_decoder;
   reg [7:0]	 write_decoder ;
   reg [7:0]	 load;
   wire [15:0]	 R0, R1, R2, R3, R4, R5, R6, R7;

   // Each register will fill in the output with the input on rising edge of clock if load is set to '1', otherwise don't care
   // R0-R7 NOT MODULE INSTANCE NAMES, maybe regs?
   register_load_enable register0(data_in,load[0],clk,R0);
   register_load_enable register1(data_in,load[1],clk,R1);
   register_load_enable register2(data_in,load[2],clk,R2);
   register_load_enable register3(data_in,load[3],clk,R3);
   register_load_enable register4(data_in,load[4],clk,R4);
   register_load_enable register5(data_in,load[5],clk,R5);
   register_load_enable register6(data_in,load[6],clk,R6);
   register_load_enable register7(data_in,load[7],clk,R7);
   
   always_comb begin
      
     // 3:8 binary to one-hot decoder for writenum
      case(writenum)
	`r0: write_decoder = `r0_decode;
	`r1: write_decoder = `r1_decode;
	`r2: write_decoder = `r2_decode;
	`r3: write_decoder = `r3_decode;
	`r4: write_decoder = `r4_decode;
	`r5: write_decoder = `r5_decode;
	`r6: write_decoder = `r6_decode;
	`r7: write_decoder = `r7_decode;
	default: write_decoder = 8'bxxxxxxxx;
      endcase 

      // Load will be a one-hot code for which register to write to
      load = write_decoder & {write, write, write, write, write, write, write, write};

      
      
      // 3:8 binary to one-hot decoder for readnum
      case(readnum)
	`r0: read_decoder = `r0_decode;
	`r1: read_decoder = `r1_decode;
	`r2: read_decoder = `r2_decode;
	`r3: read_decoder = `r3_decode;
	`r4: read_decoder = `r4_decode;
	`r5: read_decoder = `r5_decode;
	`r6: read_decoder = `r6_decode;
	`r7: read_decoder = `r7_decode;
	default: read_decoder = 8'bxxxxxxxx;
      endcase

       // Depending on what the one-hot code is, will copy the value of the specified register to data_out
      case(read_decoder)
	`r0_decode: data_out = R0;
       	`r1_decode: data_out = R1;
       	`r2_decode: data_out = R2;
       	`r3_decode: data_out = R3;
       	`r4_decode: data_out = R4;
       	`r5_decode: data_out = R5;
       	`r6_decode: data_out = R6;
       	`r7_decode: data_out = R7;
	default: data_out = 16'bxxxxxxxxxxxxxxxx;
      endcase
      
   end // always_comb
   
endmodule

module register_load_enable(in, load, clk, out);

   parameter n = 16;
   input[n-1:0] in;
   input       load, clk;
   output reg[n-1:0] out;

   reg [n-1:0]	 D;
   reg [n-1:0]	 Q;
    

   always_comb begin

      case(load)
	1'b1: D = in;
	1'b0: D = Q;
	default: D = {(n){1'bx}};
      endcase


   end

   always_ff @(posedge clk) begin

      Q = D;
      out = Q;

   end
   
endmodule

