//////////////////////////////////////////////////////////////////////////////////
//// Written by David_Harris@hmc.edu
// Top level system including MIPS and memories
//////////////////////////////////////////////////////////////////////////////////

module top  (   input  logic        clk, reset, sw_input,           
                output logic[31:0]  writedata, dataadr,            
	            output logic        memwrite,
	            output logic[31:0]  pcout, instrout,
	            output logic [3:0]  AN,
                output logic [6:0]  C,
                output logic        DP );  

   logic [31:0] readdata, pc, instr;
   logic [31:0] writedata, dataadr;

   // instantiate processor and memories  
   mips mips (clk_pulse, reset_pulse, pc, instr, memwrite, dataadr, writedata, readdata); 
   imem imem (pc[7:2], instr);  
   dmem dmem (clk_pulse, memwrite, dataadr, writedata, readdata); 
   
   //7-segment display
   display_controller display(clk_pulse, reset_pulse, 4'hf, //all enabled
                                    writedata[3:0], writedata[7:4], //RF[rt]
                                    dataadr[7:4], dataadr[3:0], AN, C, DP); //ALU result
   pulse_controller pulse1(clk, sw_input, reset, clk_pulse);
   pulse_controller pulse2(clk, sw_input, reset, reset_pulse);
   
   assign pcout = pc;
   assign instrout = instr;

endmodule

module mips (input  logic        clk, reset,
             output logic[31:0]  pc,
             input  logic[31:0]  instr,
             output logic        memwrite,
             output logic[31:0]  aluout, writedata,
             input  logic[31:0]  readdata);

  logic [1:0]  memtoreg;
  logic        pcsrc, zero, alusrc, regdst, regwrite, jump;
  logic [2:0]  alucontrol;

  controller c (instr[31:26], instr[5:0], zero, memtoreg, memwrite, pcsrc,
                        alusrc, regdst, regwrite, jump, alucontrol);

  datapath dp (clk, reset, memtoreg, pcsrc, alusrc, regdst, regwrite, jump,
                          alucontrol, zero, pc, instr, aluout, writedata, readdata);

endmodule

module controller(input  logic[5:0] op, funct,
                  input  logic      zero,
                  output logic[1:0] memtoreg, 
                  output logic      memwrite, pcsrc, alusrc,
                  output logic      regdst, regwrite,
                  output logic      jump,
                  output logic[2:0] alucontrol);

   logic [1:0] aluop;
   logic       branch;

   maindec md (op, memtoreg, memwrite, branch, alusrc, regdst, regwrite, 
		 jump, aluop);

   aludec  ad (funct, aluop, alucontrol);

   assign pcsrc = branch & zero;

endmodule

module maindec (  input  logic[5:0] op, 
	              output logic[1:0] memtoreg, 
	              output logic      memwrite, branch,
	              output logic      alusrc, regdst, regwrite, jump,
	              output logic[1:0] aluop );
   logic [9:0] controls;

   assign {regwrite, regdst, alusrc, branch, memwrite,
                memtoreg,  aluop, jump} = controls;

  always_comb
    case(op)
      6'b000000: controls <= 10'b1100000100; // R-type
      6'b100011: controls <= 10'b1010001000; // LW
      6'b101011: controls <= 10'b0010100000; // SW
      6'b000100: controls <= 10'b0001000010; // BEQ
      6'b001000: controls <= 10'b1010000000; // ADDI
      6'b000010: controls <= 10'b0000000001; // J
      6'b011101: controls <= 10'b1000010000; //LUI
      default:   controls <= 10'bxxxxxxxxxxx; // illegal op
    endcase
endmodule

module aludec (input    logic[5:0] funct,
               input    logic[1:0] aluop,
               output   logic[2:0] alucontrol);
  always_comb
    case(aluop)
      2'b00: alucontrol  = 3'b010;  // add  (for lw/sw/addi)
      2'b01: alucontrol  = 3'b110;  // sub   (for beq)
      default: case(funct)          // R-TYPE instructions
          6'b100000: alucontrol  = 3'b010; // ADD
          6'b100010: alucontrol  = 3'b110; // SUB
          6'b100100: alucontrol  = 3'b000; // AND
          6'b100101: alucontrol  = 3'b001; // OR
          6'b101010: alucontrol  = 3'b111; // SLT
          default:   alucontrol  = 3'bxxx; // ???
        endcase
    endcase
endmodule



module datapath (input  logic       clk, reset, 
                 input  logic[1:0]  memtoreg,
                 input  logic       pcsrc, alusrc, regdst,
                 input  logic       regwrite, jump, 
		         input  logic[2:0]  alucontrol, 
                 output logic       zero, 
		         output logic[31:0] pc, 
	             input  logic[31:0] instr,
                 output logic[31:0] aluout, writedata, 
	             input  logic[31:0] readdata);

  logic [4:0]  writereg;
  logic [31:0] pcnext, pcnextbr, pcplus4, pcbranch, luimm;
  logic [31:0] signimm, signimmsh, srca, srcb, result;
 
  // next PC logic
  flopr #(32) pcreg(clk, reset, pcnext, pc);
  adder       pcadd1(pc, 32'b100, pcplus4);
  sl2         immsh(signimm, signimmsh);
  adder       pcadd2(pcplus4, signimmsh, pcbranch);
  mux2 #(32)  pcbrmux(pcplus4, pcbranch, pcsrc,
                      pcnextbr);
  mux2 #(32)  pcmux(pcnextbr, {pcplus4[31:28], 
                    instr[25:0], 2'b00}, jump, pcnext);

// register file logic
   regfile     rf (clk, regwrite, instr[25:21], instr[20:16], writereg,
                   result, srca, writedata);

   mux2 #(5)    wrmux (instr[20:16], instr[15:11], regdst, writereg);
   sll16        shifter (instr[15:0], luimm);
   mux3 #(32)  resmux (aluout, readdata, luimm, memtoreg, result);
   signext         se (instr[15:0], signimm);

  // ALU logic
   mux2 #(32)  srcbmux (writedata, signimm, alusrc, srcb);
   alu         alu (srca, srcb, alucontrol, aluout, zero);

endmodule


module regfile (input    logic clk, we3, 
                input    logic[4:0]  ra1, ra2, wa3, 
                input    logic[31:0] wd3, 
                output   logic[31:0] rd1, rd2);

  logic [31:0] rf [31:0];

  // three ported register file: read two ports combinationally
  // write third port on rising edge of clock. Register0 hardwired to 0.

  always_ff @ (posedge clk, posedge we3)
     if (we3) 
         rf [wa3] <= wd3;	

  assign rd1 = (ra1 != 0) ? rf [ra1] : 0;
  assign rd2 = (ra2 != 0) ? rf[ ra2] : 0;

endmodule


module sll16 (input  logic[15:0] imm, 
              output logic[31:0]   y);
    assign y = imm << 16;
endmodule


module mux3	#(parameter WIDTH = 8)
             (input  logic[WIDTH-1:0] d0, d1, d2, 
             input  logic[1:0]  s,
             output logic[WIDTH-1:0] y);
	assign y = s[1] ? d2 : (s[0]? d1 : d0);
endmodule



module alu(input  logic [31:0] a, b, 
           input  logic [2:0]  alucont, 
           output logic [31:0] result,
           output logic zero);
    
    always_comb begin     
        case(alucont)
            3'b000: result <= (a & b); //and bitwise
            3'b001: result <= (a | b); //or bitwise
            3'b010: result <= (a + b); //add
            3'b011: result <= 32'hxxxxxxxx; //not used
            3'b100: result <= (a & (~b));
            3'b101: result <= (a | (~b));
            3'b110: result <= (a - b); //sub
            3'b111: result <= (a < b) ? 32'h00000001 : 32'h00000000;
//                    if (a < b) begin
//                        result <= 32'h00000001;
//                    end
//                    else begin
//                        result <= 32'h00000000;
//                    end
            default:
                result <= 32'hxxxxxxxx; //invalid alu control signal
        endcase
    end        

    assign zero = (result == 32'h00000000);  
    
endmodule




//////////////////////////////////////////////////////////////////////////////////
// External instruction memory used by MIPS single-cycle
// processor. It models instruction memory as a stored-program 
// ROM, with address as input, and instruction as output
//////////////////////////////////////////////////////////////////////////////////


module imem ( input logic [5:0] addr, output logic [31:0] instr);

// imem is modeled as a lookup table, a stored-program byte-addressable ROM
	always_comb
	   case ({addr,2'b00})		   	// word-aligned fetch
//		address		instruction
//		-------		-----------
		8'h00: instr = 32'h20020005;  	// disassemble, by hand 
		8'h04: instr = 32'h2003000c;  	// or with a program,
		8'h08: instr = 32'h2067fff7;  	// to find out what
		8'h0c: instr = 32'h00e22025;  	// this program does!
		8'h10: instr = 32'h00642824;
		8'h14: instr = 32'h00a42820;
		8'h18: instr = 32'h10a7000a;
		8'h1c: instr = 32'h0064202a;
		8'h20: instr = 32'h10800001;
		8'h24: instr = 32'h20050000;
		8'h28: instr = 32'h00e2202a;
		8'h2c: instr = 32'h00853820;
		8'h30: instr = 32'h00e23822;
		8'h34: instr = 32'hac670044;
		8'h38: instr = 32'h8c020050;
		8'h3c: instr = 32'h08000011;
		8'h40: instr = 32'h20020001;
		8'h44: instr = 32'hac020054;
		8'h48: instr = 32'h74070003; //lui
		8'h4c: instr = 32'h08000013;	// j 48, so it will loop here
	     default:  instr = {32{1'bx}};	// unknown address
	   endcase
endmodule