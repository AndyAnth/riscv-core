//4-stages pipeline risc-v core

`timescale 1ns / 1ps

// implemented opcodes:

`define LUI     7'b01101_11      // lui   rd,imm[31:12]
`define AUIPC   7'b00101_11      // auipc rd,imm[31:12]
`define JAL     7'b11011_11      // jal   rd,imm[xxxxx]
`define JALR    7'b11001_11      // jalr  rd,rs1,imm[11:0]
`define BCC     7'b11000_11      // bcc   rs1,rs2,imm[12:1]
`define LCC     7'b00000_11      // lxx   rd,rs1,imm[11:0]
`define SCC     7'b01000_11      // sxx   rs1,rs2,imm[11:0]
`define MCC     7'b00100_11      // xxxi  rd,rs1,imm[11:0]
`define RCC     7'b01100_11      // xxx   rd,rs1,rs2
`define CCC     7'b11100_11      // exx, csrxx, mret

// proprietary extension (custom-0)
`define CUS     7'b00010_11      // cus   rd,rs1,rs2,fc3,fct5

// not implemented opcodes:
//`define FCC     7'b00011_11      // fencex

//取指-----译码、派遣-----执行(访存)-----写回
//取指-----译码、派遣、执行、访存------写回

// configuration file
`include "../rtl/config.vh"

module darkriscv
//#(
//    parameter [31:0] RESET_PC = 0,
//    parameter [31:0] RESET_SP = 4096
//)
(
    input             CLK,   // clock
    input             RES,   // reset
    input             HLT,   // halt

`ifdef __THREADS__
    output [`__THREADS__-1:0] TPTR,  // thread pointer
`endif

`ifdef __INTERRUPT__
    input             INT,   // interrupt request
`endif

    input      [31:0] IDATA, // instruction data bus
    output     [31:0] IADDR, // instruction addr bus

    input      [31:0] DATAI, // data bus (input)
    output     [31:0] DATAO, // data bus (output)
    output     [31:0] DADDR, // addr bus

`ifdef __FLEXBUZZ__
    output     [ 2:0] DLEN, // data length
    output            RW,   // data read/write
`else
    output     [ 3:0] BE,   // byte enable
    output            WR,    // write enable
    output            RD,    // read enable
`endif

    output            IDLE,   // idle output

    output [3:0]  DEBUG       // old-school osciloscope based debug! :)
);

    // dummy 32-bit words w/ all-0s and all-1s:

    wire [31:0] ALL0  = 0;
    wire [31:0] ALL1  = -1;

    reg XRES = 1;

`ifdef __THREADS__
    reg [`__THREADS__-1:0] XMODE = 0;     // thread ptr, used to determine which thread is under operation

    assign TPTR = XMODE;
`endif

    // decode: IDATA is break apart as described in the RV32I specification

`ifdef __3STAGE__

    reg [31:0] XIDATA;

    reg XLUI, XAUIPC, XJAL, XJALR, XBCC, XLCC, XSCC, XMCC, XRCC, XCUS, XCCC; //, XFCC, XCCC;

    reg [31:0] XSIMM;
    reg [31:0] XUIMM;

    reg [31:0]  XDATAI;

    always@(posedge CLK)
    begin
        //把IDATA的值传递给XIDATA的目的是使always块下面的指令也能同步在CLK的时钟下
        XIDATA <= XRES ? 0 : HLT ? XIDATA : IDATA;	//if halt, all signals remain old values
        XDATAI <= XRES ? 0 : HLT ? XDATAI : DATAI;  //对DATAI做同步


        XLUI   <= XRES ? 0 : HLT ? XLUI   : IDATA[6:0]==`LUI;	//IDATA is input instruction
        XAUIPC <= XRES ? 0 : HLT ? XAUIPC : IDATA[6:0]==`AUIPC;
        XJAL   <= XRES ? 0 : HLT ? XJAL   : IDATA[6:0]==`JAL;
        XJALR  <= XRES ? 0 : HLT ? XJALR  : IDATA[6:0]==`JALR;

        XBCC   <= XRES ? 0 : HLT ? XBCC   : IDATA[6:0]==`BCC;
        XLCC   <= XRES ? 0 : HLT ? XLCC   : IDATA[6:0]==`LCC;
        XSCC   <= XRES ? 0 : HLT ? XSCC   : IDATA[6:0]==`SCC;
        XMCC   <= XRES ? 0 : HLT ? XMCC   : IDATA[6:0]==`MCC;

        XRCC   <= XRES ? 0 : HLT ? XRCC   : IDATA[6:0]==`RCC;
        XCUS   <= XRES ? 0 : HLT ? XRCC   : IDATA[6:0]==`CUS;
        //XFCC   <= XRES ? 0 : HLT ? XFCC   : IDATA[6:0]==`FCC;
        XCCC   <= XRES ? 0 : HLT ? XCCC   : IDATA[6:0]==`CCC;

        // signal extended immediate, according to the instruction type:

        XSIMM  <= XRES ? 0 : HLT ? XSIMM :
                 IDATA[6:0]==`SCC ? { IDATA[31] ? ALL1[31:12]:ALL0[31:12], IDATA[31:25],IDATA[11:7] } : // s-type
                 IDATA[6:0]==`BCC ? { IDATA[31] ? ALL1[31:13]:ALL0[31:13], IDATA[31],IDATA[7],IDATA[30:25],IDATA[11:8],ALL0[0] } : // b-type
                 IDATA[6:0]==`JAL ? { IDATA[31] ? ALL1[31:21]:ALL0[31:21], IDATA[31], IDATA[19:12], IDATA[20], IDATA[30:21], ALL0[0] } : // j-type
                 IDATA[6:0]==`LUI||
                 IDATA[6:0]==`AUIPC ? { IDATA[31:12], ALL0[11:0] } : // u-type
                                      { IDATA[31] ? ALL1[31:12]:ALL0[31:12], IDATA[31:20] }; // i-type
        // non-signal extended immediate, according to the instruction type:

        XUIMM  <= XRES ? 0: HLT ? XUIMM :
                 IDATA[6:0]==`SCC ? { ALL0[31:12], IDATA[31:25],IDATA[11:7] } : // s-type
                 IDATA[6:0]==`BCC ? { ALL0[31:13], IDATA[31],IDATA[7],IDATA[30:25],IDATA[11:8],ALL0[0] } : // b-type
                 IDATA[6:0]==`JAL ? { ALL0[31:21], IDATA[31], IDATA[19:12], IDATA[20], IDATA[30:21], ALL0[0] } : // j-type
                 IDATA[6:0]==`LUI||
                 IDATA[6:0]==`AUIPC ? { IDATA[31:12], ALL0[11:0] } : // u-type
                                      { ALL0[31:12], IDATA[31:20] }; // i-type
    end

    reg [1:0] FLUSH = -1;  // flush instruction pipeline

`else

    wire [31:0] XIDATA;

    wire XLUI, XAUIPC, XJAL, XJALR, XBCC, XLCC, XSCC, XMCC, XRCC, XCUS, XCCC; //, XFCC, XCCC;

    wire [31:0] XSIMM;
    wire [31:0] XUIMM;

    assign XIDATA = XRES ? 0 : IDATA;

    assign XLUI   = XRES ? 0 : IDATA[6:0]==`LUI;
    assign XAUIPC = XRES ? 0 : IDATA[6:0]==`AUIPC;
    assign XJAL   = XRES ? 0 : IDATA[6:0]==`JAL;
    assign XJALR  = XRES ? 0 : IDATA[6:0]==`JALR;

    assign XBCC   = XRES ? 0 : IDATA[6:0]==`BCC;
    assign XLCC   = XRES ? 0 : IDATA[6:0]==`LCC;
    assign XSCC   = XRES ? 0 : IDATA[6:0]==`SCC;
    assign XMCC   = XRES ? 0 : IDATA[6:0]==`MCC;

    assign XRCC   = XRES ? 0 : IDATA[6:0]==`RCC;
    assign XCUS   = XRES ? 0 : IDATA[6:0]==`CUS;
    //assign XFCC   <= XRES ? 0 : IDATA[6:0]==`FCC;
    assign XCCC   = XRES ? 0 : IDATA[6:0]==`CCC;


    // signal extended immediate, according to the instruction type:

    assign XSIMM  = XRES ? 0 : 
                     IDATA[6:0]==`SCC ? { IDATA[31] ? ALL1[31:12]:ALL0[31:12], IDATA[31:25],IDATA[11:7] } : // s-type
                     IDATA[6:0]==`BCC ? { IDATA[31] ? ALL1[31:13]:ALL0[31:13], IDATA[31],IDATA[7],IDATA[30:25],IDATA[11:8],ALL0[0] } : // b-type
                     IDATA[6:0]==`JAL ? { IDATA[31] ? ALL1[31:21]:ALL0[31:21], IDATA[31], IDATA[19:12], IDATA[20], IDATA[30:21], ALL0[0] } : // j-type
                     IDATA[6:0]==`LUI||
                     IDATA[6:0]==`AUIPC ? { IDATA[31:12], ALL0[11:0] } : // u-type
                                          { IDATA[31] ? ALL1[31:12]:ALL0[31:12], IDATA[31:20] }; // i-type
        // non-signal extended immediate, according to the instruction type:

    assign XUIMM  = XRES ? 0: 
                     IDATA[6:0]==`SCC ? { ALL0[31:12], IDATA[31:25],IDATA[11:7] } : // s-type
                     IDATA[6:0]==`BCC ? { ALL0[31:13], IDATA[31],IDATA[7],IDATA[30:25],IDATA[11:8],ALL0[0] } : // b-type
                     IDATA[6:0]==`JAL ? { ALL0[31:21], IDATA[31], IDATA[19:12], IDATA[20], IDATA[30:21], ALL0[0] } : // j-type
                     IDATA[6:0]==`LUI||
                     IDATA[6:0]==`AUIPC ? { IDATA[31:12], ALL0[11:0] } : // u-type
                                          { ALL0[31:12], IDATA[31:20] }; // i-type

    reg FLUSH = -1;  // flush instruction pipeline

`endif

//处理HLT信号
reg    HLT_P, HLT_PP;
always@(posedge CLK) HLT_P <= HLT;
always@(posedge CLK) HLT_PP <= HLT_P;

`ifdef __THREADS__
    `ifdef __RV32E__

        reg [`__THREADS__-1:0] RESMODE = -1;
		//XMODE is critical in distinguishing different thread registers (for being responsible to determine different threads)
        wire [`__THREADS__+3:0] DPTR   = XRES ? { RESMODE, 4'd0 } : { XMODE, XIDATA[10: 7] }; // set SP_RESET when RES==1
        wire [`__THREADS__+3:0] S1PTR  = { XMODE, XIDATA[18:15] };
        wire [`__THREADS__+3:0] S2PTR  = { XMODE, XIDATA[23:20] };
    `else
        reg [`__THREADS__-1:0] RESMODE = -1;

        wire [`__THREADS__+4:0] DPTR   = XRES ? { RESMODE, 5'd0 } : { XMODE, XIDATA[11: 7] }; // set SP_RESET when RES==1
        wire [`__THREADS__+4:0] S1PTR  = { XMODE, XIDATA[19:15] };
        wire [`__THREADS__+4:0] S2PTR  = { XMODE, XIDATA[24:20] };
    `endif
`else
    `ifdef __RV32E__
        wire [3:0] DPTR   = XRES ? 0 : XIDATA[10: 7]; // set SP_RESET when RES==1
        wire [3:0] S1PTR  = XIDATA[18:15];
        wire [3:0] S2PTR  = XIDATA[23:20];
    `else   //仍然同步于第二流水级(Decode)的CLK时钟
        wire [4:0] DPTR   = XRES ? 0 : XIDATA[11: 7]; //destination register
        wire [4:0] S1PTR  = XIDATA[19:15];
        wire [4:0] S2PTR  = XIDATA[24:20];
    `endif
`endif

    //同步stage-3/4的DPTR
    `ifdef __RV32E__
        reg [3:0] DPTR_ST3, DPTR_ST4;
    `else
        reg [4:0] DPTR_ST3, DPTR_ST4;
    `endif

    always@(posedge CLK)   DPTR_ST3 <= DPTR;
    always@(posedge CLK)   DPTR_ST4 <= DPTR_ST3;

    //冲突检测
    //wire  [31:0]  S1_CONF = (S1PTR && DPTR) ? ALL1 : ALL0;
    //wire  [31:0]  S2_CONF = (S2PTR && DPTR) ? ALL1 : ALL0;
    wire    S1_CONF = (S1PTR == DPTR_ST3);// ||  (S1PTR == DPTR_ST4);
    wire    S2_CONF = (S2PTR == DPTR_ST3);// ||  (S2PTR == DPTR_ST4);

    //都同步于第二流水级(Decode)的CLK时钟下
    wire [6:0] OPCODE = FLUSH ? 0 : XIDATA[6:0];
    wire [2:0] FCT3   = XIDATA[14:12];
    wire [6:0] FCT7   = XIDATA[31:25];

    wire [31:0] SIMM  = XSIMM;
    wire [31:0] UIMM  = XUIMM;

    // main opcode decoder:
    //译码得到的XLUI指令类型信息传递给当前指令指令译码指示信号(同步于第二流水级时钟)
    wire    LUI = FLUSH ? 0 : XLUI;   // OPCODE==7'b0110111;
    wire  AUIPC = FLUSH ? 0 : XAUIPC; // OPCODE==7'b0010111;
    wire    JAL = FLUSH ? 0 : XJAL;   // OPCODE==7'b1101111;
    wire   JALR = FLUSH ? 0 : XJALR;  // OPCODE==7'b1100111;

    wire    BCC = FLUSH ? 0 : XBCC; // OPCODE==7'b1100011; //FCT3
    wire    LCC = FLUSH ? 0 : XLCC; // OPCODE==7'b0000011; //FCT3
    wire    SCC = FLUSH ? 0 : XSCC; // OPCODE==7'b0100011; //FCT3
    wire    MCC = FLUSH ? 0 : XMCC; // OPCODE==7'b0010011; //FCT3

    wire    RCC = FLUSH ? 0 : XRCC; // OPCODE==7'b0110011; //FCT3
    wire    CUS = FLUSH ? 0 : XCUS; // OPCODE==7'b0110011; //FCT3
    //wire    FCC = FLUSH ? 0 : XFCC; // OPCODE==7'b0001111; //FCT3
    wire    CCC = FLUSH ? 0 : XCCC; // OPCODE==7'b1110011; //FCT3

   //同步第三流水级译码信号
    reg   LUI_P;
    reg AUIPC_P;
    reg   JAL_P;
    reg  JALR_P;
    reg   BCC_P;
    reg   LCC_P;
    reg   SCC_P;
    reg   MCC_P;
    reg   RCC_P;
    reg   CUS_P;
    reg   CCC_P;
 
    always@(posedge CLK) begin
        if(FLUSH) begin
            LUI_P   <=  0 ;
            AUIPC_P <=  0 ;
            JAL_P   <=  0 ;
            JALR_P  <=  0 ;

            BCC_P   <=  0 ;
            LCC_P   <=  0 ;
            SCC_P   <=  0 ;
            MCC_P   <=  0 ;
  
            RCC_P   <=  0 ;
            CUS_P   <=  0 ;
            CCC_P   <=  0 ;
        end
        else begin
            LUI_P   <=  LUI;   
            AUIPC_P <=  AUIPC; 
            JAL_P   <=  JAL;   
            JALR_P  <=  JALR;  

            BCC_P   <=  BCC; 
            LCC_P   <=  LCC; 
            SCC_P   <=  SCC; 
            MCC_P   <=  MCC; 
  
            RCC_P   <=  RCC; 
            CUS_P   <=  CUS; 
            CCC_P   <=  CCC; 
        end
    end

    //单独同步SCC到第四流水级
    reg SCC_PP;
    always@(posedge CLK)    SCC_PP <= SCC_P;


`ifdef __THREADS__
    `ifdef __3STAGE__
        reg [31:0] NXPC2 [0:(2**`__THREADS__)-1];       // 32-bit program counter t+2
    `endif

    `ifdef __RV32E__
        reg [31:0] REGS [0:16*(2**`__THREADS__)-1];	// general-purpose 16x32-bit registers (s1)
    `else
        reg [31:0] REGS [0:32*(2**`__THREADS__)-1];	// general-purpose 32x32-bit registers (s1)
    `endif
`else
    `ifdef __3STAGE__
        reg [31:0] NXPC2;       // 32-bit program counter t+2(也就是第三流水级的PC)
        reg [31:0] NXPC3;
    `endif

    `ifdef __RV32E__
        reg [31:0] REGS [0:15];	// general-purpose 16x32-bit registers (s1)
    `else
        reg [31:0] REGS [0:31];	// general-purpose 32x32-bit registers (s1)
    `endif
`endif

    reg [31:0] NXPC;        // 32-bit program counter t+1(第二流水级的PC)
    reg [31:0] PC;		    // 32-bit program counter t+0(当前流水级的PC)

`ifdef SIMULATION
    integer i;
    
    initial for(i=0;i!=16;i=i+1) REGS[i] = 0;
`endif

    // source-1 and source-1 register selection

    wire [31:0] RD_VAL; //准备写入RD寄存器的计算结果

    wire          [31:0] U1REG = S1_CONF ? RD_VAL : REGS[S1PTR];
    wire          [31:0] U2REG = S2_CONF ? RD_VAL : REGS[S2PTR];
    //wire          [31:0] U1REG =  REGS[S1PTR];
    //wire          [31:0] U2REG =  REGS[S2PTR];

    wire signed   [31:0] S1REG = U1REG;
    wire signed   [31:0] S2REG = U2REG;

    wire signed [31:0] S2REGX = XMCC ? SIMM : S2REG;
    wire        [31:0] U2REGX = XMCC ? UIMM : U2REG;

    //引入分支预测机制(在第一流水级取指后立刻进行)
`ifdef __RV32E__
        wire [3:0] S1PTR_BP  = IDATA[18:15];
`else 
        wire [4:0] S1PTR_BP  = IDATA[19:15];
`endif

    wire    BXX = (IDATA[6:0] == 7'b1100011);
    wire    JALXX = (IDATA[6:0] == 7'b1101111);
    wire    JALRXX = (IDATA[6:0] == 7'b1100111);

    wire branch_bad_addr = (IDATA[6:0] == 7'b1100111) & ((S1PTR_BP == DPTR) | (S1PTR_BP == DPTR_ST3) | (S1PTR_BP == DPTR_ST4));

    wire branch_taken_pre = (!FLUSH) & ((IDATA[6:0] == 7'b1101111) | ((IDATA[6:0] == 7'b1100011) & (IDATA[31])));// | ((IDATA[6:0] == 7'b1100111) & (!branch_bad_addr))
    
    reg  branch_taken_status;
    always@(posedge CLK) branch_taken_status <= branch_taken_pre;

    wire branch_taken = branch_taken_pre & (!branch_taken_status);

    wire [31:0] branch_addr_op1 = (IDATA[6:0] == 7'b1100011) ? {(IDATA[31] ? ALL1[31:13]:ALL0[31:13]), IDATA[31], IDATA[7], IDATA[30:25], IDATA[11:8], ALL0[0]} :  //BXX
                       (IDATA[6:0] == 7'b1101111) ? {(IDATA[31] ? ALL1[31:21]:ALL0[31:21]), IDATA[31], IDATA[19:12], IDATA[20], IDATA[30:21], ALL0[0]}: //JAL
                       ((IDATA[6:0] == 7'b1100111) && (S1PTR_BP == 0)) ? ALL0 : //JALR  //对x0寄存器单独处理
                       ((IDATA[6:0] == 7'b1100111) && (S1PTR_BP != DPTR) && (S1PTR_BP != DPTR_ST3) && (S1PTR_BP != DPTR_ST4)) ? REGS[S1PTR_BP]:
                       //如果rs1不与任何流水级待写回寄存器冲突，就直接拉取rs1对应寄存器的值
                       ALL0;

    wire [31:0] branch_addr_op2 = ((IDATA[6:0] == 7'b1100011) || (IDATA[6:0] == 7'b1101111)) ? NXPC2:    //BXX与JAL给出的都是地址偏移量
                       (IDATA[6:0] == 7'b1100111) ? ALL0 : 
                       ALL0;     //JLAR给出的是绝对跳转地址

    wire [31:0] branch_addr = branch_addr_op1 + branch_addr_op2;     //计算出最终的偏移地址

    //将branch_taken打一拍同步到decode流水级
    reg branch_taken_p;
    always@(posedge CLK) branch_taken_p <= branch_taken;


    //mini-decode: 提前处理分支跳转指令
    wire         BMUX;
    wire         JREQ;
    wire  [31:0] JVAL;
    wire  [31:0] PCSIMM;

    assign BMUX     =  FCT3==7 && U1REG>=U2REG  || // bgeu
                       FCT3==6 && U1REG< U2REGX || // bltu
                       FCT3==5 && S1REG>=S2REG  || // bge
                       FCT3==4 && S1REG< S2REGX || // blt
                       FCT3==1 && U1REG!=U2REGX || // bne
                       FCT3==0 && U1REG==U2REGX; // beq

    assign PCSIMM = NXPC + SIMM;    //auipc指令的立即数
    assign JREQ = JAL||JALR||(BCC && BMUX);    //J型指令和B型指令同时处理
    assign JVAL = JALR ? DADDR : PCSIMM; // SIMM + (JALR ? U1REG : PC);

    wire   branch_wrongly_taken = branch_taken_p ^ JREQ;   //表示分支预测错误

    //同步分支预测错误信号
    reg    NJEQ_BT;
    always@(posedge CLK) NJEQ_BT <= ((!JREQ) && branch_wrongly_taken);  //表示不应跳却跳

    //单独对PCSIMM和SIMM打拍同步到第三、四流水级
    reg  [31:0]   PCSIMM_POS, PCSIMM_PP; 
    always@(posedge CLK)    PCSIMM_POS <= PCSIMM;

    always@(posedge CLK)    PCSIMM_PP <= PCSIMM_POS;

    reg  [31:0]   SIMM_POS, SIMM_PP;
    always@(posedge CLK)    SIMM_POS <= SIMM;   //同步SIMM到stage-3

    always@(posedge CLK)    SIMM_PP <= SIMM_POS;   //同步SIMM到stage-3

/*------------------stage 3------------------*/

    // L-group of instructions (OPCODE==7'b0000011)
    //XDATAI在DATAI基础上延后一拍，处于stage-3
    reg [31:0] LDATA;

    //不做时序处理，DADDR打一拍
    always@(posedge CLK) begin
`ifdef __FLEXBUZZ__
		//FLEXBUZZ bus requires little endian
    LDATA <= FCT3[1:0]==0 ? { FCT3[2]==0&&DATAI[ 7] ? ALL1[31: 8]:ALL0[31: 8] , DATAI[ 7: 0] } :
                        FCT3[1:0]==1 ? { FCT3[2]==0&&DATAI[15] ? ALL1[31:16]:ALL0[31:16] , DATAI[15: 0] } :
                                        DATAI;
`else	//operation below is to determine lb/lh/lw
		//i.e.	FCT3 == 0 --->lb	FCT3 == 5 ---> lbu  ...
		//DADDR's low 3bits may uesd to determine the location of required data in data bus
    LDATA <= FCT3==0||FCT3==4 ? ( DADDR[1:0]==3 ? { FCT3==0&&DATAI[31] ? ALL1[31: 8]:ALL0[31: 8] , DATAI[31:24] } :
                                             DADDR[1:0]==2 ? { FCT3==0&&DATAI[23] ? ALL1[31: 8]:ALL0[31: 8] , DATAI[23:16] } :
                                             DADDR[1:0]==1 ? { FCT3==0&&DATAI[15] ? ALL1[31: 8]:ALL0[31: 8] , DATAI[15: 8] } :
                                                             { FCT3==0&&DATAI[ 7] ? ALL1[31: 8]:ALL0[31: 8] , DATAI[ 7: 0] } ):
                        FCT3==1||FCT3==5 ? ( DADDR[1]==1   ? { FCT3==1&&DATAI[31] ? ALL1[31:16]:ALL0[31:16] , DATAI[31:16] } :
                                                             { FCT3==1&&DATAI[15] ? ALL1[31:16]:ALL0[31:16] , DATAI[15: 0] } ) :
                                             DATAI;  
     //在Load的HLT阶段，整条流水线这时会根据前一个ADDR对应的DATAI来继续运行，但由于下个周期指令不变，因此下个周期
     //会用新Load进来的数据再次覆盖写入之前的RD寄存器  
`endif
    end

    // S-group of instructions (OPCODE==7'b0100011)
//    reg [31:0] SDATA;
//    always@(posedge CLK) begin
`ifdef __FLEXBUZZ__

    wire    [31:0]  SDATA = U2REG; /* FCT3==0 ? { ALL0 [31: 8], U2REG[ 7:0] } :
                        FCT3==1 ? { ALL0 [31:16], U2REG[15:0] } :
                                    U2REG;*/
`else
	//logic distinguishing DADDR's low 3bits is corressponding with that for LDATA
    wire    [31:0]  SDATA = FCT3==0 ? ( DADDR[1:0]==3 ? { U2REG[ 7: 0], ALL0 [23:0] } :
                                    DADDR[1:0]==2 ? { ALL0 [31:24], U2REG[ 7:0], ALL0[15:0] } :
                                    DADDR[1:0]==1 ? { ALL0 [31:16], U2REG[ 7:0], ALL0[7:0] } :
                                                    { ALL0 [31: 8], U2REG[ 7:0] } ) :
                        FCT3==1 ? ( DADDR[1]==1   ? { U2REG[15: 0], ALL0 [15:0] } :
                                                    { ALL0 [31:16], U2REG[15:0] } ) :
                                    U2REG;
`endif
//    end

    // C-group: CSRRW

`ifdef __INTERRUPT__

    reg [31:0] MEPC  = 0;
    reg [31:0] MTVEC = 0;
    reg        MIE   = 0;
    reg        MIP   = 0;
    reg [31:0] CDATA = 0;
    reg MRET;
    reg CSRW;
    reg CSRR;

    always@(posedge CLK) begin

        CDATA <= XIDATA[31:20]==12'h344 ? MIP  : // machine interrupt pending
                        XIDATA[31:20]==12'h304 ? MIE   : // machine interrupt enable
                        XIDATA[31:20]==12'h341 ? MEPC  : // machine exception PC
                        XIDATA[31:20]==12'h305 ? MTVEC : // machine vector table
                                                 0;	 // unknown

        MRET  <= CCC && FCT3==0 && S2PTR==2;
        CSRW  <= CCC && FCT3==1;
        CSRR  <= CCC && FCT3==2;

    end
`endif
    wire EBRK = CCC && FCT3==0 && S2PTR==1;

    // RM-group of instructions (OPCODEs==7'b0010011/7'b0110011), merged! src=immediate(M)/register(R)

    //区分有符号和无符号的REG
    //wire signed [31:0] S2REGX = XMCC ? SIMM : S2REG;
    //wire        [31:0] U2REGX = XMCC ? UIMM : U2REG;
    reg [31:0] RMDATA;
    always@(posedge CLK) begin
    //进行计算
    RMDATA <= FCT3==7 ? U1REG&S2REGX :
                         FCT3==6 ? U1REG|S2REGX :
                         FCT3==4 ? U1REG^S2REGX :
                         FCT3==3 ? U1REG<U2REGX : // unsigned
                         FCT3==2 ? S1REG<S2REGX : // signed
                         FCT3==0 ? (XRCC&&FCT7[5] ? U1REG-S2REGX : U1REG+S2REGX) :	//didn't use Complement
                         FCT3==1 ? S1REG<<U2REGX[4:0] :
                         //FCT3==5 ?
                         !FCT7[5] ? S1REG>>U2REGX[4:0] :
`ifdef MODEL_TECH
                                   -((-S1REG)>>U2REGX[4:0]); // workaround for modelsim
`else
                                   $signed(S1REG)>>>U2REGX[4:0];  // (FCT7[5] ? U1REG>>>U2REG[4:0] :
`endif
    end
`ifdef __MAC16X16__

    // MAC instruction rd += s1*s2 (OPCODE==7'b1111111)
    //
    // 0000000 01100 01011 100 01100 0110011 xor a2,a1,a2
    // 0000000 01010 01100 000 01010 0110011 add a0,a2,a0
    // 0000000 01100 01011 000 01010 0001011 mac a0,a1,a2
    //
    // 0000 0000 1100 0101 1000 0101 0000 1011 = 00c5850b

    wire MAC = CUS && FCT3==0;

    wire signed [15:0] K1TMP = S1REG[15:0];
    wire signed [15:0] K2TMP = S2REG[15:0];
    wire signed [31:0] KDATA = K1TMP*K2TMP;

`endif

//    reg BMUX;
//    reg  [31:0] PCSIMM;
//    reg         JREQ;
//    reg  [31:0] JVAL;
  // J/B-group of instructions (OPCODE==7'b1100011)
//    always@(posedge CLK) begin
//        BMUX       = FCT3==7 && U1REG>=U2REG  || // bgeu
//                      FCT3==6 && U1REG< U2REGX || // bltu
//                      FCT3==5 && S1REG>=S2REG  || // bge
//                      FCT3==4 && S1REG< S2REGX || // blt
//                      FCT3==1 && U1REG!=U2REGX || // bne
//                      FCT3==0 && U1REG==U2REGX; // beq
//
//        PCSIMM = PC+SIMM;
//        JREQ = JAL||JALR||(BCC && BMUX);    //J型指令和B型指令同时处理
//        JVAL = JALR ? DADDR : PCSIMM; // SIMM + (JALR ? U1REG : PC);
//    end

//调整REGS[DPTR]的写回时序(不针对多线程)
`ifdef __RV32E__
    reg [3:0] DPTR_POS; 
`else   
    reg [4:0] DPTR_POS; 
`endif

    always@(posedge CLK) begin
`ifdef __RV32E__
        DPTR_POS   <= XRES ? 0 : XIDATA[10: 7]; 
`else   
        DPTR_POS   <= XRES ? 0 : XIDATA[11: 7]; 
`endif
    end

    //引入旁路机制
    //在所有执行/访存单元执行完成后直接拉取待写入RD寄存器的数值(第三流水级)
    `ifdef __RV32E__
        assign  RD_VAL =  XRES||DPTR_POS[3:0]==0 ? 0  :        // reset sp
`else
        assign  RD_VAL =  XRES||DPTR_POS[4:0]==0 ? 0  :        // reset sp
`endif
                       HLT_P ? REGS[DPTR_POS] :        // halt
                       LCC_P ? LDATA :
                     AUIPC_P ? PCSIMM_POS :
                 JAL_P||JALR_P ? NXPC : //暂时先不管，未处理
                       LUI_P ? SIMM_POS :
                  MCC_P||RCC_P ? RMDATA:
`ifdef __MAC16X16__
                       MAC ? REGS[DPTR_POS]+KDATA :
`endif
`ifdef __INTERRUPT__
                       CSRR ? CDATA :
`endif
                             REGS[DPTR_POS];


    //XRES和FLUSH的更新实际是在第三流水级(Decode后直接更新)
    always@(posedge CLK) begin

`ifdef __THREADS__
        RESMODE <= RES ? -1 : RESMODE ? RESMODE-1 : 0;	//if externel RES=1, system reset in next cycle
														//if just reseted last cycle, remove RES requist
        XRES <= |RESMODE;
`else
        XRES <= RES;    //RES信号在clk时钟下同步
`endif

`ifdef __3STAGE__
	    FLUSH <= XRES ? 2 : HLT ? FLUSH :        // reset and halt
        //RES给2的原因是3级流水线需要额外两个周期才能完成冲刷
        //因为提前了BXX指令的处理，因此实际只需要两个周期的冲刷就可以
	                       FLUSH ? FLUSH-1 :    //如果FLUSH不为0则每次自减1，直到减为0停止冲刷
    `ifdef __INTERRUPT__
                            MRET ? 2 :          //MRET指令需要冲刷当前流水线中正在执行的指令
    `endif
	//MRET和JREQ都在stage-2
           //branch_wrongly_taken ? 2 :
        (JREQ && branch_wrongly_taken) ? 2 :    //该跳却不跳，恢复JREQ的模式
                        NJEQ_BT ? 1 :           //不该跳却跳，恢复正确地址后取消掉错误加载的指令
        (branch_taken_p && (!branch_wrongly_taken)) ? 1 : 0;    //跳转地址需要下一个周期才能被加载，当前周期的指令需要被冲刷掉
	                       //JREQ ? 2 : 0;  // flush the pipeline!
						   //分支跳转都是默认不跳(J型指令也一样)，先按默认的PC+4执行，判断发生跳转后冲刷流水线
                           //此处的FLUSH需要等两个周期后，NXPC2、NXPC和PC对应取来的指令排空后才能取到正确PC对应的地址
                           //PC--->NXPC--->NXPC2(Act)
`else
        FLUSH <= XRES ? 1 : HLT ? FLUSH :        // reset and halt
                       JREQ;  // flush the pipeline!
`endif
    end

//PC指针的更新并不属于某一流水级，持续递增就行
    always@(posedge CLK) begin
`ifdef __3STAGE__

    `ifdef __THREADS__

        NXPC <= /*XRES ? `__RESETPC__ :*/ HLT ? NXPC : NXPC2[XMODE];
		//for multi-threads only manipulate current thread's NXPC2
        NXPC2[XRES ? RESMODE : XMODE] <=  XRES ? `__RESETPC__ : HLT ? NXPC2[XMODE] :   // reset and halt
                                      JREQ ? JVAL :                            // jmp/bra
	                                         NXPC2[XMODE]+4;                   // normal flow

        XMODE <= XRES ? 0 : HLT ? XMODE :        // reset and halt
                            /*JAL*/ JREQ ? XMODE+1 : XMODE;
	             //XMODE==0/*&& IREQ*/&&JREQ ? 1 :         // wait pipeflush to switch to irq
                 //XMODE==1/*&&!IREQ*/&&JREQ ? 0 : XMODE;  // wait pipeflush to return from irq

    `else
		//in 3-stages mode, NXPC always changes 1 cycle later than NXPC2
        NXPC <= /*XRES ? `__RESETPC__ :*/ HLT ? NXPC : NXPC2;

        NXPC2 <= HLT ? NXPC2 : NXPC3;

	    NXPC3 <=  XRES ? `__RESETPC__ : HLT ? NXPC3 :   // reset and halt
        //`ifdef __INTERRUPT__
        //             MRET ? MEPC :
        //            MIE&&MIP&&JREQ ? MTVEC : // pending interrupt + pipeline flush
        //`endif
	                 //JREQ ? JVAL :                    // jmp/bra
                          branch_taken ? branch_addr :
        (JREQ && branch_wrongly_taken) ? JVAL :
     ((!JREQ) && branch_wrongly_taken) ? NXPC2+4:
                            //AUIPC_P ? PCSIMM_POS:               
	                                     NXPC3+4;                   // normal flow

    `endif

`else
        NXPC <= XRES ? `__RESETPC__ : HLT ? NXPC :   // reset and halt
        `ifdef __INTERRUPT__
                     MRET ? MEPC :
                    MIE&&MIP&&JREQ ? MTVEC : // pending interrupt + pipeline flush
        `endif
              JREQ ? JVAL :                   // jmp/bra
                     NXPC+4;                   // normal flow
`endif
        PC   <= /*XRES ? `__RESETPC__ :*/ HLT ? PC : NXPC; // current program counter
    end

/*------------------stage 4------------------*/

reg    [31:0]  WB_VAL;
`ifdef __RV32E__
    reg [3:0] DPTR_POS_VAL; 
`else   
    reg [4:0] DPTR_POS_VAL; 
`endif   

    always@(posedge CLK)
    begin

`ifdef __INTERRUPT__
        if(XRES)
        begin
            MTVEC <= 0;
            MEPC  <= 0;
            MIP   <= 0;
            MIE   <= 0;
        end
        else
        if(MIP&&MIE&&JREQ)
        begin
            MEPC <= JVAL;
            MIP  <= 1;
            MIE  <= 0;
        end
        else
        if(CSRW)
        begin
            case(XIDATA[31:20])
                12'h305: MTVEC <= U1REG;
                12'h341: MEPC  <= U1REG;
                12'h304: MIE   <= U1REG;
            endcase
        end
        else
        if(MRET)
        begin
            MIP <= 0;
            MIE <= 1;
        end
        else
        if(INT==1&&MIE==1)
        begin
            MIP <= 1;
        end
`endif

//写回在第四流水级，待写回的各信号均同步到第三流水级下

`ifdef __RV32E__
        WB_VAL <=   XRES||DPTR_POS[3:0]==0 ? 0  :        // reset sp
`else
        WB_VAL <=   XRES||DPTR_POS[4:0]==0 ? 0  :        // reset sp
`endif
                       HLT_P   ? REGS[DPTR_POS] :        // halt
                       LCC_P ? LDATA :
                     AUIPC_P ? PCSIMM_POS :
                      JAL_P||JALR_P ? NXPC :     //NXPC应该对应到具体的流水级中去
                       LUI_P ? SIMM_POS :
                  MCC_P||RCC_P ? RMDATA:    //这块语句的各个信号更新顺序需要调整

`ifdef __MAC16X16__
                       MAC ? REGS[DPTR_POS]+KDATA :
`endif
`ifdef __INTERRUPT__
                       CSRR ? CDATA :
`endif
                             REGS[DPTR_POS];

DPTR_POS_VAL <= DPTR_POS;

`ifdef __RV32E__
        REGS[DPTR_POS] <=   XRES||DPTR_POS[3:0]==0 ? 0  :        // reset sp
`else
        REGS[DPTR_POS] <=   XRES||DPTR_POS[4:0]==0 ? 0  :        // reset sp
`endif
                       HLT_P   ? REGS[DPTR_POS] :        // halt
                       LCC_P ? LDATA :
                     AUIPC_P ? PCSIMM_POS :
                      JAL_P||
                      JALR_P ? NXPC :     //NXPC应该对应到具体的流水级中去
                       LUI_P ? SIMM_POS :
                  MCC_P||RCC_P ? RMDATA:    //这块语句的各个信号更新顺序需要调整

`ifdef __MAC16X16__
                       MAC ? REGS[DPTR_POS]+KDATA :
`endif
`ifdef __INTERRUPT__
                       CSRR ? CDATA :
`endif
                             REGS[DPTR_POS];

`ifndef __YOSYS__

        if(EBRK)
        begin
            $display("breakpoint at %x",PC);	//set breakpoint
            $stop();
        end
        
        if(!FLUSH && IDATA===32'dx)
        begin
            $display("invalid IDATA at %x",PC);	//instruction error
            $stop();  
        end

        //if(LCC && !HLT && DATAI===32'dx)
        if(XLCC && !HLT && DATAI===32'dx)
        begin
            $display("invalid DATAI@%x at %x",DADDR,PC);	//load data error
            $stop();
        end
`endif

    end

//下面的写法认为外设接口访问发生在第二流水级(Decode之后)
   // IO and memory interface

    assign DATAO = SDATA; // SCC ? SDATA : 0;
    
    //DADDR和DATAO对齐(SCC延迟一个时钟周期，LCC直接给)
    reg [31:0] XDADDR, XADDR_R;
    always@(posedge CLK)   XADDR_R <= U1REG + SIMM;
    always@(posedge CLK)   XDADDR <= XADDR_R;

    assign DADDR =(U1REG + SIMM); // (SCC||LCC) ? U1REG + SIMM : 0; // SCC ? XDADDR : 
    //assign DADDR = XDADDR; // (SCC||LCC) ? U1REG + SIMM : 0;

    // based on the SCC and LCC

`ifdef __FLEXBUZZ__
    assign RW      = !XSCC;
    assign DLEN[0] = (XSCC||XLCC)&&FCT3[1:0]==0;
    assign DLEN[1] = (XSCC||XLCC)&&FCT3[1:0]==1;
    assign DLEN[2] = (XSCC||XLCC)&&FCT3[1:0]==2;
`else
    assign RD = XLCC;
    assign WR = XSCC;
    assign BE = FCT3==0||FCT3==4 ? ( DADDR[1:0]==3 ? 4'b1000 : // sb/lb
                                     DADDR[1:0]==2 ? 4'b0100 :
                                     DADDR[1:0]==1 ? 4'b0010 :
                                                     4'b0001 ) :
                FCT3==1||FCT3==5 ? ( DADDR[1]==1   ? 4'b1100 : // sh/lh
                                                     4'b0011 ) :
                                                     4'b1111; // sw/lw
`endif

`ifdef __3STAGE__
    `ifdef __THREADS__
        assign IADDR = NXPC3[XMODE];    //由于SRAM总线访问需要两个时钟周期，因此仍将总线地址给成第二流水级的NXPC2
    `else
        assign IADDR = NXPC3;
    `endif
`else
    assign IADDR = NXPC;
`endif

    assign IDLE = |FLUSH;       //用IDLE来反压取指模块
`ifdef __INTERRUPT__
    assign DEBUG = { INT, MIP, MIE, MRET };
`else
    assign DEBUG = { XRES, IDLE, XSCC, XLCC };
`endif

endmodule
