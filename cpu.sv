module cpu(clk, reset, read_data, mem_cmd, mem_addr, write_data, N, V, Z);

   input clk, reset;
   input [15:0]	read_data;
   output [1:0] mem_cmd;
   output [8:0] mem_addr;
   output [15:0] write_data;
   output	 N, V, Z; 

   wire [15:0]	 instruction, sximm5, sximm8, mdata, datapath_out;
   wire [8:0]	 next_pc, PC, mem_addr, data_address_out;
   wire [2:0]	 nsel, readnum, writenum, opcode, ZNV, ZNV_out;
   wire [1:0]	 ALUop, shift, op, vsel;
   wire [4:0]	 imm5;
   wire		 Z, N, V, write, loada, loadb, loadc, loads, asel, bsel, load_ir, load_pc, load_addr, reset_pc, addr_sel;
    
   register_load_enable instructionregister(read_data, load_ir, clk, instruction);
   instruction_decoder instructiondecoder(instruction, nsel, ALUop, instruction[4:0], sximm5, instruction[7:0], sximm8, shift, readnum, writenum, opcode, op); 
   state_machine_controller FSM(clk, reset, opcode, op, nsel, asel, bsel, vsel, loada, loadb, loadc, loads, write, load_ir, load_pc, load_addr, reset_pc, addr_sel, mem_cmd);
   datapath DP(writenum, write, readnum, clk, loada, loadb, loadc, loads, shift, ALUop, asel, bsel, vsel, ZNV_out, datapath_out, read_data, sximm8, sximm5, 9'd0); 

   adder pc_adder(reset_pc, PC, next_pc);
   register_load_enable #(9) pc_reg(next_pc, load_pc, clk, PC);

   MUX2 #(9) address(PC, data_address_out, addr_sel, mem_addr);
   register_load_enable #(9) dataaddress(datapath_out[8:0], load_addr, clk, data_address_out);
   
   assign write_data = datapath_out;
   assign Z = ZNV_out[2];
   assign N = ZNV_out[1];
   assign V = ZNV_out[0];
   
endmodule // cpu       

module instruction_decoder(instruction, nsel, ALUop, imm5, sximm5, imm8, sximm8, shift, readnum, writenum, opcode, op);

   input [15:0] instruction;
   input [2:0]	nsel;
   input [4:0] imm5;
   input [7:0] imm8;
   output [1:0]	ALUop, shift, op;
   output reg [15:0] sximm5, sximm8;
   output [2:0]	 readnum, writenum, opcode;

   reg [2:0]	 readnum, writenum;
   wire [2:0]	 Rn, Rd, Rm;

   assign ALUop = instruction[12:11];
   assign shift = instruction[4:3];
   assign opcode = instruction[15:13];
   assign op = instruction[12:11];
   assign Rn = instruction[10:8];
   assign Rd = instruction[7:5];
   assign Rm = instruction[2:0];
   
   always_comb begin

      // Sign extender 5/8 bits to 16 bits
      if(imm5[4] == 1'b1)
	sximm5 = {11'b1111_1111_111, instruction[4:0]};
      else
	sximm5 =  {11'b0000_0000_000, instruction[4:0]};

      if(imm8[7] == 1'b1)
      	sximm8 = {8'b1111_1111, instruction[7:0]};
      else
	sximm8 = {8'b0000_0000, instruction[7:0]};

      // 3-bit one-hot MUX to select Rn, Rd, Rm and push to regfile
      case(nsel)
	3'b100: begin readnum = Rn; writenum = Rn; end
	3'b010: begin readnum = Rd; writenum = Rd; end
        3'b001: begin readnum = Rm; writenum = Rm; end
	default: begin readnum = 3'bxxx; writenum = 3'bxxx; end
      endcase // case (nsel)

   end

endmodule


`define reset 5'b00000
`define IF1 5'b00001
`define IF2 5'b00010
`define updatepc 5'b00011
`define decode 5'b00100
`define getA 5'b00101
`define getB 5'b00110
`define writeReg 5'b00111
`define writeImm 5'b01000
`define sub 5'b01001
`define add2 5'b01010
`define add0 5'b01011
`define ldr1 5'b01100
`define ldr2 5'b01101
`define ldr3 5'b01110
`define loaddataaddress 5'b01111
`define str1 5'b10000
`define str2 5'b10001
`define str3 5'b10010
`define halt 5'b10011

`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10


module state_machine_controller(clk, reset, opcode, op, nsel, asel, bsel, vsel, loada, loadb, loadc, loads, write, load_ir, load_pc, load_addr, reset_pc, addr_sel, mem_cmd);

   input clk, reset;
   input [2:0] opcode;
   input [1:0] op;
   output reg [2:0]	nsel;
   output reg [1:0]	vsel, mem_cmd;
   output reg 	asel, bsel, loada, loadb, loadc, loads, write, load_ir, load_pc, load_addr, reset_pc, addr_sel;

   reg [4:0]	present_state;

   always_comb begin

      // Case statement to update outputs of state machine
      case(present_state)
       `reset:
	 begin asel = 1'bx; bsel = 1'bx; vsel = 2'bxx; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b0; reset_pc = 1'b1; load_pc = 1'b1; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'bx; mem_cmd = `MNONE; end
       `IF1:
	 begin asel = 1'bx; bsel = 1'bx; vsel = 2'bxx; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b1; mem_cmd = `MREAD; end
       `IF2:
	 begin asel = 1'bx; bsel = 1'bx; vsel = 2'bxx; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b1; load_addr = 1'b0; addr_sel = 1'b1; mem_cmd = `MREAD; end
       `updatepc:
	 begin asel = 1'bx; bsel = 1'bx; vsel = 2'bxx; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b1; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end
       `decode:
         begin asel = 1'bx; bsel = 1'bx; vsel = 2'bxx; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end 
       `getA:
         begin asel = 1'b0; bsel = 1'b0; vsel = 2'b00; nsel = 3'b100; loada = 1'b1; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end 
       `getB:
         begin asel = 1'b0; bsel = 1'b0; vsel = 2'b00; nsel = 3'b001; loada = 1'b0; loadb = 1'b1; loadc = 1'b0; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end 
       `writeReg:
         begin asel = 1'b0; bsel = 1'b0; vsel = 2'b11; nsel = 3'b010; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b1; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end 
       `writeImm:
         begin asel = 1'b0; bsel = 1'b0; vsel = 2'b01; nsel = 3'b100; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b1; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end 
       `sub:
         begin asel = 1'b0; bsel = 1'b0; vsel = 2'b00; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b1; loads = 1'b1; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end 
       `add2:
         begin asel = 1'b0; bsel = 1'b0; vsel = 2'b00; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b1; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end 
       `add0:
         begin asel = 1'b1; bsel = 1'b0; vsel = 2'b00; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b1; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end
       `loaddataaddress:
	 begin asel = 1'b0; bsel = 1'b0; vsel = 2'b00; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b1; addr_sel = 1'b0; mem_cmd = `MNONE; end
       `ldr1:
	 begin asel = 1'b0; bsel = 1'b1; vsel = 2'b00; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b1; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end
       `ldr2:
	 begin asel = 1'b0; bsel = 1'b0; vsel = 2'b00; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MREAD; end
       `ldr3:
	 begin asel = 1'b0; bsel = 1'b0; vsel = 2'b00; nsel = 3'b010; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b1; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MREAD; end
       `str1:
	 begin asel = 1'b0; bsel = 1'b1; vsel = 2'b00; nsel = 3'b010; loada = 1'b0; loadb = 1'b1; loadc = 1'b1; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end
       `str2:
	 begin asel = 1'b1; bsel = 1'b0; vsel = 2'b00; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b1; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MNONE; end
       `str3:
	 begin asel = 1'b1; bsel = 1'b0; vsel = 2'b00; nsel = 3'bxxx; loada = 1'b0; loadb = 1'b0; loadc = 1'b0; loads = 1'b0; write = 1'b0; reset_pc = 1'b0; load_pc = 1'b0; load_ir = 1'b0; load_addr = 1'b0; addr_sel = 1'b0; mem_cmd = `MWRITE; end
       `halt:
	 begin asel = 1'bx; bsel = 1'bx; vsel = 2'bxx; nsel = 3'bxxx; loada = 1'bx; loadb = 1'bx; loadc = 1'bx; loads = 1'bx; write = 1'bx; reset_pc = 1'b0; load_pc = 1'bx; load_ir = 1'bx; load_addr = 1'bx; addr_sel = 1'bx; mem_cmd = `MNONE; end
       default:
         begin asel = 1'bx; bsel = 1'bx; vsel = 2'bxx; nsel = 3'bxxx; loada = 1'bx; loadb = 1'bx; loadc = 1'bx; loads = 1'bx; write = 1'bx; reset_pc = 1'b0; load_pc = 1'bx; load_ir = 1'bx; load_addr = 1'bx; addr_sel = 1'bx; mem_cmd = `MNONE; end 
      endcase

   end // always_comb

   always @(posedge clk) begin

     if(reset)
	present_state = `reset;

     else begin
	
	// Case statement to udpate moving of present state at each posedge clk
	case(present_state)
	  `reset: present_state = `IF1;
	  `IF1: present_state = `IF2;
	  `IF2: present_state = `updatepc;
	  `updatepc: present_state = `decode;
	  `decode: 
	    begin
	       // MOV IMM
	       if(opcode == 3'b110 && op == 2'b10)
		 present_state = `writeImm;
	       // MOV REG
	       else if(opcode == 3'b110 && op == 2'b00)
		 present_state = `getB;
	       // ADD
	       else if(opcode == 3'b101 && op == 2'b00)
		 present_state = `getA;
	       // CMP
	       else if(opcode == 3'b101 && op == 2'b01)
		 present_state = `getA;
	       // AND
	       else if(opcode == 3'b101 && op == 2'b10)
		 present_state = `getA;
	       // MVN
	       else if(opcode == 3'b101 && op == 2'b11)
		 present_state = `getB;
	       // LDR or STR
	       else if(opcode == 3'b011 || opcode == 3'b100)
		 present_state = `getA;
	       // HALT
	       else if(opcode == 3'b111)
		 present_state = `halt;
	       else
		 present_state = 5'bxxxxx; 
	    end 
	  `getA: 
	    begin
	       if(opcode == 3'b101)
		 present_state = `getB;
	       else if(opcode == 3'b011)
		 present_state = `ldr1;
	       else if(opcode == 3'b100)
		 present_state = `str1;
	       else
		 present_state = 5'bxxxxx;
	    end
	  `getB:
	    begin
	       if((opcode == 3'b110 && op == 2'b00) || (opcode == 3'b101 && op == 2'b11))
		 present_state = `add0;
	       else if(opcode == 3'b101 && (op == 2'b00 || op == 2'b10))
		 present_state = `add2;
	       else if(opcode == 3'b101 && op == 2'b01)
		 present_state = `sub;
	       else
		 present_state = 4'bxxx;
	    end // case: `getB
	  `add0: present_state = `writeReg;
	  `add2: present_state = `writeReg;
	  `sub: present_state = `IF1;
	  `writeImm: present_state = `IF1;
	  `writeReg: present_state = `IF1;
	  `ldr1: present_state = `loaddataaddress;
	  `str1: present_state = `loaddataaddress;
	  `loaddataaddress:
	    begin
	       if(opcode == 3'b011)
		 present_state = `ldr2;
	       else if(opcode == 3'b100)
		 present_state = `str2;
	       else
		 present_state = 5'bxxxxx;
	    end
	  `ldr2: present_state = `ldr3;
	  `ldr3: present_state = `IF1;
          `str2: present_state = `str3;
	  `str3: present_state = `IF1;
	  `halt: present_state = `halt;
	  default: present_state = 5'bxxxxx;
	endcase // case (present_state)
	
      end
      	
   end

endmodule // state_machine_controller

module adder(reset_pc, PC, next_pc);
   input reset_pc;
   input [8:0] PC;
   output reg [8:0] next_pc;

   always_comb begin

      case(reset_pc)
	1'b1: next_pc = 9'd0;
	1'b0: next_pc = PC + 9'd1;
	default: next_pc = 9'bx_xxxx_xxxx;
      endcase // case (reset_pc)
  
   end
    
endmodule


