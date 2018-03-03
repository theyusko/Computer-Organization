module top_basys(input  logic       sys_clk, clk, reset, reset1, sw_input,
                 output logic       memwrite, regwrite,
                 output logic[3:0]  AN,
                 output logic[6:0]  C,
                 output logic       DP);
    
    logic[31:0]  writedata, dataadr;
    logic[31:0]  pcout, instrout;
    
    top top(clk_pulse, reset_pulse, writedata, dataadr, memwrite, regwrite, pcout, instrout);
    
    //7-segment display
    display_controller display(clk_pulse, reset_pulse, 4'b1111, //all enabled
                               dataadr[7:4], dataadr[3:0],
                                writedata[7:4], writedata[3:0],      //RF[rt]
                                    AN, C, DP); 
                                     
    pulse_controller pulse1(sys_clk, clk, reset1, clk_pulse);
    pulse_controller pulse2(sys_clk, reset, reset1, reset_pulse);
    
endmodule


//////////////////////////////////////////////////////////////////////////////////
//// Written by David_Harris@hmc.edu
// Top level system including MIPS and memories
//////////////////////////////////////////////////////////////////////////////////

module top  (   input  logic        clk, reset,// sw_input,           
                output logic[31:0]  writedata, dataadr,            
	            output logic        memwrite, regwrite,
	            output logic[31:0]  pcout, instrout);

   logic [31:0] readdata, pc, instr;
   logic [31:0] result, dataadr;
   logic [31:0] writedata;

    mips mips (clk, reset, pc, instr, memwrite, regwrite, dataadr, writedata, readdata); 
    imem imem (pc[7:2], instr);  
    dmem dmem (clk, memwrite, dataadr, writedata, readdata); 
 
   assign pcout = pc;
   assign instrout = instr;

endmodule


`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// pipelined MIPS processor, with controller and datapath  //
//////////////////////////////////////////////////////////////////////////////////
module mips (input  logic        clk, reset,
             output logic[31:0]  PCF,
             input  logic[31:0]  instrF,
             output logic        memWriteM, regWriteW,
             output logic[31:0]  ALUOutM, writeDataM,
             input  logic[31:0]  readDataM);

    logic[5:0] opD, functD;
    logic[1:0] memToRegE, memToRegM, memToRegW;
    logic      regWriteE, regWriteM, regWriteW;
    logic      memWriteM; 
    logic      ALUSrcE, regDstE, branchE, jumpD, flushE;
    logic[2:0] ALUControlE;
    logic[31:0] writeDataM;

    controller c (clk, reset, opD, functD, flushE, 
                    regWriteE, regWriteM, regWriteW, 
                    memToRegE, memToRegM, memToRegW, memWriteM, 
                    ALUControlE, ALUSrcE, regDstE, branchD, branchE, jumpD);  
                    
    datapath dp (   clk, reset, 
                    regWriteE, regWriteM, regWriteW, 
                    memToRegE, memToRegM, memToRegW, memWriteM,
                    ALUControlE, ALUSrcE, regDstE, branchD, branchE, jumpD,
                    instrF, readDataM, flushE, PCF, ALUOutM, writeDataM,
                    opD, functD, resultW);
                                          
endmodule



module controller(input clk, reset,
                  input  logic[5:0] opD, functD,
                  input  logic      flushE,
                  output logic      regWriteE, regWriteM, regWriteW,
                  output logic[1:0] memToRegE, memToRegM, memToRegW,
                  output logic      memWriteM, //try deleting memWriteE
                  output logic[2:0] ALUControlE,
                  output logic      ALUSrcE, regDstE, branchD, branchE, jumpD);

    logic [1:0]  ALUOpD, memToRegD; 
    logic        memWriteD, memWriteE, branchD, ALUSrcD, regDstD, regWriteD;
    logic [2:0]  ALUControlD;

    //Inside of control unit
    maindec md (opD, memToRegD, memWriteD, branchD, ALUSrcD, 
                    regDstD, regWriteD, jumpD, ALUOpD);
    aludec  ad (functD, ALUOpD, ALUControlD);
   
      
   //Next stage registers:   
    floprc #(10)  regE(clk, reset, flushE, 
                        {regWriteD, memToRegD, memWriteD, ALUControlD, ALUSrcD, regDstD, branchD},
                        {regWriteE, memToRegE, memWriteE, ALUControlE, ALUSrcE, regDstE, branchE});
                                                
    flopr #(4)  regM(clk, reset, {regWriteE, memToRegE, memWriteE}, 
                                 {regWriteM, memToRegM, memWriteM});
                                 
    flopr #(3) regW(clk, reset, {regWriteM, memToRegM}, {regWriteW, memToRegW});

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
      default:   controls <= 10'bxxxxxxxxxx; // illegal op
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



module floprc  #(parameter WIDTH = 8)
 		(input logic clk, reset, clear,
 		 input logic [WIDTH-1:0] d,
		 output logic [WIDTH-1:0] q);
 
	always_ff @(posedge clk, posedge reset)
 		if (reset) q <= 0;
 		else if (clear) q <= 0;
 		else q <= d;
endmodule


// parameterized register
module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset, 
	           input  logic[WIDTH-1:0]  d, 
               output logic[WIDTH-1:0]  q);

  always_ff@(posedge clk, posedge reset)
    if (reset) q <= 0; 
    else       q <= d;
endmodule





module hazard_unit( input  logic[4:0]   rtD,rsD, rsE,rtE,  
                    input  logic        RegWriteE, RegWriteM, RegWriteW,
			        input  logic[1:0]   MemToRegE, MemToRegM,
                    input  logic[4:0]   WriteRegE, WriteRegM, WriteRegW,
                    input  logic        BranchD, jump, PCSrcE,
                    output logic        StallF, StallD, FlushE,  
                    output logic[1:0]   ForwardAE, ForwardBE);
                
                
//    assign StallF = 1'b0;
//    assign StallD = 1'b0;                
//    assign FlushE = 1'b0;                
//    assign ForwardAE = 2'b00;                
//    assign ForwardBE = 2'b00;                
                    
     logic lwstall,branchstall;
	
	//Solving data hazards with forwarding:
	always_comb
	begin
		//Forwarding logic for SrcA(ForwardAE):
		if (( rsE != 0 ) & ( rsE == WriteRegM) & RegWriteM) 
			ForwardAE = 2'b10;
    		else if ((rsE != 0) & (rsE == WriteRegW) & RegWriteW) 
          		ForwardAE = 2'b01;
    		else
			ForwardAE = 2'b00;

	   //Forwarding logic for SrcB(ForwardBE):
		if (( rtE != 0 ) & ( rtE == WriteRegM) & RegWriteM) 
			ForwardBE = 2'b10;
    		else if ((rtE != 0) & (rtE == WriteRegW) & RegWriteW) 
          		ForwardBE = 2'b01;
    		else
			ForwardBE = 2'b00;
    end
    
    //Solving data and control hazards with stalls:
	//Data hazard stall:
	assign lwstall = ((rsD == rtE) | (rtD == rtE)) & (MemToRegE != 2'b00);
                                                        //MemToRegE is now 2 bits
								                        //but only takes values 00,
								                        //01 and 10  

	//Control hazard (Branch) stall:
	assign branchstall = ( (BranchD & RegWriteE & (WriteRegE == rsD | WriteRegE == rtD)) 
	                       | (BranchD & MemToRegM & (WriteRegM == rsD | WriteRegM == rtD)) );

	//Stall and flush in general (data & control hazards):
	assign StallF = StallD;
	assign StallD = lwstall;
	
    assign FlushD = jump | PCSrcE;
    assign FlushE = lwstall | PCSrcE;

endmodule




module flopren #(parameter WIDTH = 8)
		 (input  logic clk, reset, en,
		  input  logic [WIDTH-1:0] d,
		  output logic [WIDTH-1:0] q);

	always_ff @(posedge clk, posedge reset)
		if (reset) q <= 0;
 		else if (en) q <= d;

endmodule



module adder (input  logic[31:0] a, b,
              output logic[31:0] y);
     
     assign y = a + b;
endmodule


// paramaterized 2-to-1 MUX
module mux2 #(parameter WIDTH = 8)
             (input  logic[WIDTH-1:0] d0, d1,  
              input  logic s, 
              output logic[WIDTH-1:0] y);
  
   assign y = s ? d1 : d0; 
endmodule



module flopenrc #(parameter WIDTH = 8)
		 (input logic clk, reset, en, clear,
		 input logic [WIDTH-1:0] d,
		 output logic [WIDTH-1:0] q);

	always_ff @(posedge clk, posedge reset)
		if (reset) q <= 0;
 		else if (clear) q <= 0;
 		else if (en) q <= d;

endmodule



module signext (input  logic[15:0] a,
                output logic[31:0] y);
              
  assign y = {{16{a[15]}}, a};    // sign-extends 16-bit a
endmodule



module sl2 (input  logic[31:0] a,
            output logic[31:0] y);
     assign y = {a[29:0], 2'b00}; // shifts left by 2
endmodule




module sll16 (input  logic[15:0] imm, 
              output logic[31:0]   y);
    assign y = imm << 16;
endmodule



module regfile (input    logic clk, we3, 
                input    logic[4:0]  ra1, ra2, wa3, 
                input    logic[31:0] wd3, 
                output   logic[31:0] rd1, rd2);

  logic [31:0] rf [31:0];

  // three ported register file: read two ports combinationally
  // write third port on rising edge of clock. Register0 hardwired to 0.

  always_ff @ (posedge clk, posedge we3) //~clk
     if (we3) 
         rf [wa3] <= wd3;	

  assign rd1 = (ra1 != 0) ? rf [ra1] : 0;
  assign rd2 = (ra2 != 0) ? rf[ ra2] : 0;

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
		8'h4c: instr = 32'h08000013;	// j 4c, so it will loop here
		8'h50: instr = 32'h20020005; //shouldn't happen
	     default:  instr = {32{1'bx}};	// unknown address
	   endcase
endmodule




`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// External data memory used by MIPS single-cycle processor
//////////////////////////////////////////////////////////////////////////////////

module dmem (input  logic        clk, we,
             input  logic[31:0]  a, wd,
             output logic[31:0]  rd);

   logic  [31:0] RAM[63:0];
  
   assign rd = RAM[a[31:2]];    // word-aligned  read (for lw)

   always_ff @(posedge clk)
     if (we)
       RAM[a[31:2]] <= wd;      // word-aligned write (for sw)

endmodule


# Clock signal 
set_property PACKAGE_PIN W5 [get_ports sys_clk]  	 	 	 	  
 	set_property IOSTANDARD LVCMOS33 [get_ports sys_clk] 
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports sys_clk] 

## Switches 
set_property PACKAGE_PIN V17 [get_ports {reset1}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {reset1}] 
set_property PACKAGE_PIN V16 [get_ports {sw_input}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {sw_input}]
  
# LEDs 
set_property PACKAGE_PIN V13 [get_ports {memwrite}]  	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {memwrite}] 
set_property PACKAGE_PIN V14 [get_ports {regwrite}]  	 	 	 	 
          set_property IOSTANDARD LVCMOS33 [get_ports {regwrite}] 			
 	 
#7 segment display 
set_property PACKAGE_PIN W7 [get_ports {C[6]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {C[6]}] 
set_property PACKAGE_PIN W6 [get_ports {C[5]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {C[5]}] 
set_property PACKAGE_PIN U8 [get_ports {C[4]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {C[4]}] 
set_property PACKAGE_PIN V8 [get_ports {C[3]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {C[3]}] 
set_property PACKAGE_PIN U5 [get_ports {C[2]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {C[2]}] 
set_property PACKAGE_PIN V5 [get_ports {C[1]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {C[1]}] 
set_property PACKAGE_PIN U7 [get_ports {C[0]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {C[0]}] 
set_property PACKAGE_PIN V7 [get_ports DP]  	 	 	 	  
 	set_property IOSTANDARD LVCMOS33 [get_ports DP] 
set_property PACKAGE_PIN U2 [get_ports {AN[0]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {AN[0]}] 
set_property PACKAGE_PIN U4 [get_ports {AN[1]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {AN[1]}] 
set_property PACKAGE_PIN V4 [get_ports {AN[2]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {AN[2]}] 
set_property PACKAGE_PIN W4 [get_ports {AN[3]}] 	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports {AN[3]}] 

#Buttons 
set_property PACKAGE_PIN W19 [get_ports clk]  	 	 	 	 	 
 	set_property IOSTANDARD LVCMOS33 [get_ports clk] 
set_property PACKAGE_PIN T17 [get_ports reset]  	 	 	 	 	 
          set_property IOSTANDARD LVCMOS33 [get_ports reset]  	




module top_tb();
    logic       clk, reset, memwrite;            
    logic[31:0] writedata, dataadr, result, pc, instr;

	// Instantiate the Unit Under Test (UUT)
	top uut (clk, reset, writedata, dataadr, result, memwrite, pc, instr);

	initial begin
		clk <= 0; reset <= 1;
        # 22; 
        reset <= 0;
	end
	
	always begin
        clk <= 1; #5;
        clk <= 0; #5;
	end
	
endmodule


















