`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

module lab7_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5);
   input [3:0] KEY; // KEY0 is clk, KEY1 is reset, KEY2 is s (not sure if s is needed)
   input [9:0] SW;
   output [9:0]	LEDR;
   output [6:0]	HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

   wire		write, enable,  N, V, Z, ram_write, led_load, switch_enable;
   wire [1:0]	mem_cmd;
   wire [8:0]	mem_addr;
   wire [15:0]	read_data, write_data, dout, din;

   // Stage 2
   RAM MEM(~KEY[0], mem_addr[7:0], mem_addr[7:0], ram_write, write_data, dout);
   cpu CPU(~KEY[0], ~KEY[1], read_data, mem_cmd, mem_addr, write_data, N, V, Z);
   
   assign enable = (mem_cmd == `MREAD) && (mem_addr[8] == 1'b0);
   assign read_data = enable ? dout : {16{1'bz}};
   assign ram_write = (mem_cmd == `MWRITE) && (mem_addr[8] == 1'b0);

   // Stage 3
   register_load_enable #(8) LEDreg(write_data[7:0], led_load, ~KEY[0], LEDR[7:0]);

   assign switch_enable = (mem_cmd == `MREAD) && (mem_addr == 9'h140);
   assign led_load = (mem_cmd == `MWRITE) && (mem_addr == 9'h100);
   assign read_data = switch_enable ? {8'h00, SW[7:0]} : {16{1'bz}};
   
endmodule

module RAM(clk, read_address, write_address, write, din, dout);
   parameter data_width = 16;
   parameter addr_width = 8;
   parameter filename = "data.txt";

   input     clk;
   input [addr_width-1:0] read_address, write_address;
   input		  write;
   input [data_width-1:0] din; 
   output [data_width-1:0] dout; 
   reg [data_width-1:0]	   dout;

   reg [data_width-1:0]	   mem [2**addr_width-1:0];
   
   initial $readmemb(filename, mem); 

   always @(posedge clk) begin
      if(write)
	mem[write_address] <= din;
      dout <= mem[read_address];
   end
   
endmodule // RAM

