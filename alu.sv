module ALU(Ain, Bin, ALUop, out, ZNV);
   input[15:0] Ain, Bin;
   input [1:0] ALUop;
   output reg [15:0] out;
   output reg [2:0] ZNV;

   always @(*) begin

      case(ALUop)
	2'b00: out = Ain + Bin;
	2'b01: out = Ain - Bin;
	2'b10: out = Ain & Bin;
	2'b11: out = ~Bin;
	default: out = 16'bxxxx_xxxx_xxxx_xxxx;
      endcase 

      // Check for the zero flag
      case(out)
	16'd0: ZNV[2] = 1'b1;
	default: ZNV[2] = 1'b0;
      endcase // case (out)

      // Check for the negative and overflow flag
      case(out[15])
	1'b1: begin
	         ZNV[1] = 1'b1;
		 // MSB of out is negative, so if postiive - negative, results in overflow
		 if(Ain[15] == 1'b0 && Bin[15] == 1'b1)
		    ZNV[0] = 1'b1;
		 else
		    ZNV[0] = 1'b0;
	      end
	1'b0: begin
	         ZNV[1] = 1'b0;
		 // MSB of out is positive, so if negative - positive, results in overflow
		 if(Ain[15] == 1'b1 && Bin[15] == 1'b0)
		    ZNV[0] = 1'b1;
		 else
		    ZNV[0] = 1'b0;
	      end
	default: ZNV[1] = 1'bx;
      endcase // case (out[15])
      
         end
   
endmodule
