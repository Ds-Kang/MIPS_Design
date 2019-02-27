`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg;
  wire        pcsrc, zero, conts, memread, memread2;
  wire        alusrc, regdst, regwrite, regwrite2, jump, jal, jr; 
  wire [2:0]  alucontrol;
  wire [31:0] instr2,instr3;

  // Instantiate Controller
  controller c(
      .clk        (clk),
		.reset      (reset),
      .op         (instr2[31:26]), 
      .funct      (instr3[5:0]), 
      .zero       (zero),
      .conts      (conts),
      .signext    (signext),
      .shiftl16   (shiftl16),
      .memtoreg4   (memtoreg),
      .memwrite3   (memwrite),
      .pcsrc      (pcsrc),
      .alusrc2     (alusrc),
      .regdst2     (regdst),
      .regwrite4   (regwrite),
      .regwrite3   (regwrite2),
      .jump2       (jump),
      .jal2        (jal), 
      .jr         (jr),
      .alucontrol (alucontrol),
      .memread3   (memread),
		.memread2   (memread2));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .regwrite2  (regwrite2),
    .jump       (jump),
    .jal        (jal),
    .jr         (jr),
    .alucontrol (alucontrol),
    .zero      (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout2    (memaddr), 
    .writedata3 (memwritedata),
    .memread    (memread),
	 .memread2   (memread2),
    .conts      (conts),
	 .instr2     (instr2),
	 .instr3     (instr3),
    .readdata   (memreaddata));

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,clk,
                  input        reset, conts,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg4, memwrite3, memread3, memread2,
                  output       pcsrc, alusrc2,
                  output       regdst2, regwrite4, regwrite3,
                  output       jump2,
                  output       jal2, jr,
                  output [2:0] alucontrol);

  wire [1:0] aluop,aluop2,aluop_c;
  wire       branch,branch2;
  wire       bne,bne2, bnez;
  wire       memtoreg,memtoreg2;
  wire       memwrite, memtoreg3, memwrite2;
  wire       memread,slti;
  wire       jump,jal;
  wire       alusrc;
  wire       regwrite,regwrite2;
  wire       regdst;
  wire       signext_c,shiftl16_c,memtoreg_c,memwrite_c,branch_c,bne_c,alusrc_c;
  wire       regdst_c,regwrite_c,jump_c,jal_c,memread_c,slti_c;
  
  maindec md(
    .op       (op),
    .signext  (signext_c),
    .shiftl16 (shiftl16_c),
    .memtoreg (memtoreg_c),
    .memwrite (memwrite_c),
    .branch   (branch_c),
    .bne      (bne_c),
    .alusrc   (alusrc_c),
    .regdst   (regdst_c),
    .regwrite (regwrite_c),
    .jump     (jump_c),
    .jal      (jal_c),
	 .slti     (slti_c),
    .memread  (memread_c),
    .aluop    (aluop_c));


  aludec ad( 
    .funct      (funct),
    .aluop      (aluop2),
	 .slti       (slti2),
    .jr         (jr),
    .alucontrol (alucontrol));

  assign bnez = zero ^ bne2;
  assign pcsrc = branch2 & bnez;

// Dongsig Kang Start 
    flopr #(1) exregdstff(
    .clk   (clk),
    .reset (reset),
    .d     (regdst),
    .q     (regdst2));
                
  flopr #(1) exalusrcff(
    .clk   (clk),
    .reset (reset),
    .d     (alusrc),
    .q     (alusrc2));         

  flopr #(2) exaluopff(
    .clk   (clk),
    .reset (reset),
    .d     (aluop),
    .q     (aluop2));     
  
  flopr #(1) exbranchff(
    .clk   (clk),
    .reset (reset),
    .d     (branch),
    .q     (branch2));    

  flopr #(1) exbneff(
    .clk   (clk),
    .reset (reset),
    .d     (bne),
    .q     (bne2));    
	 
  flopr #(1) exmemwriteff(
    .clk   (clk),
    .reset (reset),
    .d     (memwrite),
    .q     (memwrite2));    
   
  flopr #(1) mmemwriteff(
    .clk   (clk),
    .reset (reset),
    .d     (memwrite2),
    .q     (memwrite3));   
    
  flopr #(1) exmemreadff(
    .clk   (clk),
    .reset (reset),
    .d     (memread),
    .q     (memread2));    
   
  flopr #(1) mmemreadff(
    .clk   (clk),
    .reset (reset),
    .d     (memread2),
    .q     (memread3));   
        
  flopr #(1) exregwriteff(
    .clk   (clk),
    .reset (reset),
    .d     (regwrite),
    .q     (regwrite2));    
   
  flopr #(1) mregwriteff(
    .clk   (clk),
    .reset (reset),
    .d     (regwrite2),
    .q     (regwrite3));   
               
  flopr #(1) wbregwriteff(
    .clk   (clk),
    .reset (reset),
    .d     (regwrite3),
    .q     (regwrite4));   

  flopr #(1) exmemtoregff(
    .clk   (clk),
    .reset (reset),
    .d     (memtoreg),
    .q     (memtoreg2));    
   
  flopr #(1) mmemtoregff(
    .clk   (clk),
    .reset (reset),
    .d     (memtoreg2),
    .q     (memtoreg3));   
               
  flopr #(1) wbmemtoregff(
    .clk   (clk),
    .reset (reset),
    .d     (memtoreg3),
    .q     (memtoreg4));

  flopr #(1) exjumpff(
    .clk   (clk),
	 .reset (reset),
	 .d     (jump),
	 .q     (jump2));

  flopr #(1) exjalff(
    .clk   (clk),
	 .reset (reset),
	 .d     (jal),
	 .q     (jal2));

  flopr #(1) exsltiff(
    .clk   (clk),
	 .reset (reset),
	 .d     (slti),
	 .q     (slti2));
	 	 	 
	 
  mux2 #(15) cont(
    .d0    (15'b0),
    .d1    ({signext_c, shiftl16_c, regwrite_c, regdst_c, alusrc_c, branch_c, bne_c, memwrite_c, memtoreg_c, jal_c, jump_c, aluop_c,memread_c,slti_c}),
    .s     (conts),
    .y     ({signext, shiftl16, regwrite, regdst, alusrc, branch, bne, memwrite, memtoreg, jal, jump, aluop,memread,slti}));    
    
//Dongsig Kang end    
    endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump, jal, bne, memread, slti,
               output [1:0] aluop);

  reg [14:0] controls;
  
  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, bne, memwrite,
          memtoreg, jal, jump, aluop,memread,slti} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 15'b001100000001100; // Rtype
      6'b100011: controls <= #`mydelay 15'b101010001000010; // LW
      6'b101011: controls <= #`mydelay 15'b100010010000000; // SW
      6'b000100: controls <= #`mydelay 15'b100001000000100; // BEQ
      6'b000101: controls <= #`mydelay 15'b100001100000100; // BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 15'b101010000000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 15'b001010000001000; // ORI
      6'b001111: controls <= #`mydelay 15'b011010000000000; // LUI
      6'b000010: controls <= #`mydelay 15'b000000000010000; // J
      6'b000011: controls <= #`mydelay 15'b001000000110000; // JAL
		6'b001010: controls <= #`mydelay 15'b101010000000101; // SLTI
      default:   controls <= #`mydelay 15'bxxxxxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
				  input      slti,
              output reg jr,
              output reg [2:0] alucontrol);

  always @(*) begin
  jr <= 1'b0;
    case(aluop)
      3'b00: alucontrol <= #`mydelay 3'b010;  // add
      3'b01:
		begin
			if (slti) alucontrol <= #`mydelay 3'b111; //slti
			else alucontrol <= #`mydelay 3'b110;  // sub
		end
      3'b10: alucontrol <= #`mydelay 3'b001;  // or
      default: case(funct)          // RTYPE
          6'b100000,
          6'b100001: alucontrol <= #`mydelay 3'b010; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 3'b110; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 3'b000; // AND
          6'b100101: alucontrol <= #`mydelay 3'b001; // OR
          6'b101010,
          6'b101011: alucontrol <= #`mydelay 3'b111; // SLT, SLTU
          6'b001000: 
          begin
            alucontrol <= #`mydelay 3'b001;
            jr <= #`mydelay 1'b1;
          end
          default:   alucontrol <= #`mydelay 3'bxxx; // ???
        endcase
    endcase
    end
endmodule
                                                                                                           

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pcsrc, memread, memread2,
                input         alusrc, regdst,
                input         regwrite, regwrite2, jump, jal, jr,	
                input  [2:0]  alucontrol,
                output        zero, conts,
                output [31:0] pc, instr2, instr3,
                input  [31:0] instr,
                output [31:0] aluout2, writedata3,
                input  [31:0] readdata);

  wire [4:0]  writereg,writereg2,writereg3,writereg4;
  wire [4:0]  writeregaddr;
  wire [31:0] writeregdata; 
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
  wire [31:0] signimm, signimmsh, shiftedimm, shiftedimm2;
  wire [31:0] srca,srcam, srcb;
  wire [31:0] result;
  wire [31:0] instr4,instr5,pcplus42,pcplus43,pcplus44,pcplus45,srca2,writedata2,writedata,writedatam,signimm2;
  wire [31:0] pcbranch2,aluout,aluout3,readdata2,fwout1,fwout2,pcnextjr,instrfl;
  wire        idwrite,pcwrite,flush,jal2,jal3;
  wire [1:0]  fwA,fwB;
  wire        idfwA,idfwB;

  // next PC logic
  flopenr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
	 .en    (pcwrite),
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (signimm2),
    .y (signimmsh));
             
  adder pcadd2(
    .a (pcplus43),
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcmux(
    .d0   (pcnextbr),
    .d1   ({pcplus42[31:28], instr3[25:0], 2'b00}),
    .s    (jump),
    .y    (pcnextjr));
	 
  mux2 #(32) jrmux(
    .d0  (pcnextjr),
	 .d1  (aluout),
	 .s   (jr),
	 .y   (pcnext));

  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    .ra1     (instr2[25:21]),
    .ra2     (instr2[20:16]),
    .wa      (writeregaddr),
    .wd      (writeregdata),
    .rd1     (srca),
    .rd2     (writedata));

  mux2 #(5) wrmux(
    .d0  (instr3[20:16]),
    .d1  (instr3[15:11]),
    .s   (regdst),
    .y   (writereg));
	

  mux2 #(5) wrjalmux( 
    .d0   (writereg3),
    .d1   (5'b11111),
    .s   (jal3),
    .y   (writeregaddr));
   
  mux2 #(32) resmux(	
    .d0 (aluout3),
    .d1 (readdata2),
    .s  (memtoreg),
    .y  (result));
   
  mux2 #(32) resjalmux(
    .d0 (result),
    .d1 (pcplus45),
    .s  (jal3),
    .y  (writeregdata));
	 
  flopr #(1) jalff(
    .clk (clk),
	 .reset (reset),
	 .d  (jal),
	 .q  (jal2));	 
	 
  flopr #(1) jalff2(
    .clk (clk),
	 .reset (reset),
	 .d  (jal2),
	 .q  (jal3));	 
	 
 
  sign_zero_ext sze(
    .a       (instr2[15:0]),
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (shiftedimm[31:0]));

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (fwout2),
    .d1 (shiftedimm2),
    .s  (alusrc),
    .y  (srcb));
    


  alu alu(
    .a       (fwout1),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout),
    .zero    (zero));
    
// Dongsig Kang Start

  flush flus(
    .jump (jump),
    .jal (jal),
    .jr  (jr),
    .pcsrc (pcsrc),
    .flush (flush));
	 
  mux2 #(32)flmux(
    .d0 (instr),
	 .d1 (32'b0),
	 .s  (flush),
	 .y  (instrfl));	 
    
  flopenr #(32) instff(
    .clk   (clk),
    .reset (reset),
    .en    (idwrite),
    .d     (instrfl),
    .q     (instr2));

  flopenr #(32) pcplus4ff(
    .clk   (clk),
    .reset (reset),
    .en    (idwrite),
    .d     (pcplus4),
    .q     (pcplus42));

  flopr #(32) pcplus4ff2(
    .clk   (clk),
    .reset (reset),
    .d     (pcplus42),
    .q     (pcplus43));
	 
  flopr #(32) pcplus4ff3(
    .clk   (clk),
    .reset (reset),
    .d     (pcplus43),
    .q     (pcplus44));
	 
  flopr #(32) pcplus4ff4(
    .clk   (clk),
    .reset (reset),
    .d     (pcplus44),
    .q     (pcplus45));

  flopr #(32) rd1ff(
    .clk   (clk),
    .reset (reset),
    .d     (srcam),
    .q     (srca2));
    
  flopr #(32) rd2ff(
    .clk   (clk),
    .reset (reset),
    .d     (writedatam),
    .q     (writedata2));
    
  flopr #(32) rd2ff2(
    .clk   (clk),
    .reset (reset),
    .d     (fwout2),
    .q     (writedata3));    
    

  flopr #(32) signextff(
    .clk   (clk),
    .reset (reset),
    .d     (signimm),
    .q     (signimm2));
    
  flopr #(32) wrmuxff(
    .clk   (clk),
    .reset (reset),
    .d     (instr2),
    .q     (instr3));
    
  flopr #(32) wrmux4ff(
    .clk   (clk),
    .reset (reset),
    .d     (instr3),
    .q     (instr4));
        
  flopr #(32) wrmux5ff(
    .clk   (clk),
    .reset (reset),
    .d     (instr4),
    .q     (instr5));
       
  flopr #(32) pcadd2ff(
    .clk   (clk),
    .reset (reset),
    .d     (pcbranch),
    .q     (pcbranch2));
    
  flopr #(32) aluresff(
    .clk   (clk),
    .reset (reset),
    .d     (aluout),
    .q     (aluout2));
    
    
  flopr #(32) datardff(
    .clk   (clk),
    .reset (reset),
    .d     (readdata),
    .q     (readdata2));
    
  flopr #(32) alures2ff(
    .clk   (clk),
    .reset (reset),
    .d     (aluout2),
    .q     (aluout3));
	 
  flopr # (5) writeregff(
    .clk   (clk),
	 .reset (reset),
	 .d     (writereg),
	 .q     (writereg2));
	 
  flopr # (5) writereg2ff(
    .clk   (clk),
	 .reset (reset),
	 .d     (writereg2),
	 .q     (writereg3));
	 
  
  flopr # (5) writereg3ff(
    .clk   (clk),
	 .reset (reset),
	 .d     (writereg3),
	 .q     (writereg4));	 
	 
  flopr # (32) sl16ff(
    .clk   (clk),
	 .reset (reset),
	 .d     (shiftedimm),
	 .q     (shiftedimm2));
	 
                
  fwunit fwu(
    .exRs        (instr3[25:21]),
    .exRt        (instr3[20:16]),
    .memRd       (writereg2),
    .wbRd        (writereg3),
    .memregwrite (regwrite2),
    .wbregwrite  (regwrite),
    .fwA         (fwA),
    .fwB         (fwB));
      
  fwmux fwmux1(
    .a     (srca2),
    .b     (aluout2),
    .c     (result),
    .fw    (fwA),
    .y     (fwout1));             

  fwmux fwmux2(
    .a     (writedata2),
    .b     (aluout2),
    .c     (result),
    .fw    (fwB),
    .y     (fwout2));

  hdunit hdu(
    .exRt        (instr3[20:16]),
    .idRs        (instr2[25:21]),
    .idRt        (instr2[20:16]),
    .exmemread   (memread2),
    .conts       (conts),
    .pcwrite     (pcwrite),
    .idwrite     (idwrite));

  idfwunit idfwu(
    .idRs       (instr2[25:21]),
	 .idRt       (instr2[20:16]),
	 .wbRd       (writereg3),
    .wbregwrite (regwrite),
	 .idfwA      (idfwA),
	 .idfwB      (idfwB));
  
  mux2 #(32) idfwmux1(
    .d0    (srca),
	 .d1    (result), 
	 .s     (idfwA),
	 .y     (srcam));
	 
  mux2 #(32) idfwmux2(
    .d0    (writedata),
	 .d1    (result), 
	 .s     (idfwB),
	 .y     (writedatam));

//Dongsig Kang end  
endmodule