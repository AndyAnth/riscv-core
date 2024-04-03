/*
 * Copyright (c) 2018, Marcelo Samsoniuk
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */

`timescale 1ns / 1ps
`include "../rtl/config.vh"

// clock and reset logic

module darksimv;

    reg CLK = 0;
    
    reg RES = 1;

`define CORE soc0.core0

    wire [31:0] IDATA = `CORE.IDATA;
    wire [31:0] IADDR = `CORE.IADDR;
    wire [1:0]  FLUSH = `CORE.FLUSH;
    wire [31:0] NXPC3 = `CORE.NXPC3;
    wire [31:0] NXPC2 = `CORE.NXPC2;
    wire [31:0] NXPC = `CORE.NXPC;
    wire [31:0] PC = `CORE.PC;
    wire        JREQ = `CORE.JREQ;
    wire [31:0] JVAL = `CORE.JVAL;
    //wire [31:0] NXPC3_PRE = `CORE.NXPC3_PRE;
    wire [31:0] DPTR_POS = `CORE.DPTR_POS;
    wire        BMUX = `CORE.BMUX;


    wire    S1_CONF = `CORE.S1_CONF;
    wire    S2_CONF = `CORE.S2_CONF;
    wire    [31:0]  U1REG = `CORE.U1REG;
    wire    [31:0]  U2REG = `CORE.U2REG;
    wire [3:0] DPTR   = `CORE.DPTR; // set SP_RESET when RES==1
    wire [3:0] S1PTR  = `CORE.S1PTR;
    wire [3:0] S2PTR  = `CORE.S2PTR;
    wire [3:0] DPTR_ST3 = `CORE.DPTR_ST3;
    wire [3:0] DPTR_ST4 = `CORE.DPTR_ST4;
    wire [31:0] RD_VAL = `CORE.RD_VAL;
    wire [31:0] RMDATA = `CORE.RMDATA;
    wire [31:0] WB_VAL = `CORE.WB_VAL;
    wire [31:0] DPTR_POS = `CORE.DPTR_POS;
    wire [4:0] DPTR_POS_VAL = `CORE.DPTR_POS_VAL;

    wire [31:0] REGS_1 = `CORE.REGS[0];
    wire [31:0] REGS_2 = `CORE.REGS[1];
    wire [31:0] REGS_3 = `CORE.REGS[2];
    wire [31:0] REGS_4 = `CORE.REGS[3];
    wire [31:0] REGS_5 = `CORE.REGS[4];
    wire [31:0] REGS_6 = `CORE.REGS[5];
    wire [31:0] REGS_7 = `CORE.REGS[6];
    wire [31:0] REGS_8 = `CORE.REGS[7];
    wire [31:0] REGS_9 = `CORE.REGS[8];
    wire [31:0] REGS_0 = `CORE.REGS[9];
    wire [31:0] REGS_10 = `CORE.REGS[10];
    wire [31:0] REGS_11 = `CORE.REGS[11];
    wire [31:0] REGS_12 = `CORE.REGS[12];
    wire [31:0] REGS_13 = `CORE.REGS[13];
    wire [31:0] REGS_14 = `CORE.REGS[14];
    wire [31:0] REGS_15 = `CORE.REGS[15];

    wire    XLUI = `CORE.XLUI;
    wire    XAUIPC = `CORE.XAUIPC;
    wire    XJAL = `CORE.XJAL;
    wire    XJALR = `CORE.XJALR;
    wire    XBCC = `CORE.XBCC;
    wire    XLCC = `CORE.XLCC;
    wire    XSCC = `CORE.XSCC;
    wire    XMCC = `CORE.XMCC;
    wire    XRCC = `CORE.XRCC;
    wire    XCUS = `CORE.XCUS;
    
    wire    LUI = `CORE.LUI;
    wire    AUIPC = `CORE.AUIPC;
    wire    JAL = `CORE.JAL;
    wire    JALR = `CORE.JALR;
    wire    BCC = `CORE.BCC;
    wire    LCC = `CORE.LCC;
    wire    SCC = `CORE.SCC;
    wire    MCC = `CORE.MCC;
    wire    RCC = `CORE.RCC;

    wire    HLT = `CORE.HLT;
    wire    HLT_P = `CORE.HLT_P;
    wire    HLT_PP = `CORE.HLT_PP;

    wire [31:0] XSIMM = `CORE.XSIMM;
    wire [31:0] XUIMM = `CORE.XUIMM;

    wire [31:0] DATAI = `CORE.DATAI;
    wire [31:0] DATAO = `CORE.DATAO;
    wire [31:0] DADDR = `CORE.DADDR;

    //wire [31:0] U1REG = `CORE.U1REG;
    //wire [31:0] U2REG = `CORE.U2REG;

    wire [3:0]  DEBUG = `CORE.DEBUG;


    initial while(1) #(500e6/`BOARD_CK) CLK = !CLK; // clock generator w/ freq defined by config.vh

    integer i;

    initial
    begin
`ifdef __ICARUS__
        $dumpfile("darksocv.vcd");
        $dumpvars();

    `ifdef __REGDUMP__
        for(i=0;i!=`RLEN;i=i+1)
        begin
            $dumpvars(0,soc0.core0.REGS[i]);
        end
    `endif
`endif
        $display("reset (startup)");
        #1e3    RES = 0;            // wait 1us in reset state
        //#1000e3 RES = 1;            // run  1ms
        //$display("reset (restart)");
        //#1e3    RES = 0;            // wait 1us in reset state
        //#1000e3 $finish();          // run  1ms
    end

    wire TX;
    wire RX = 1;

    darksocv soc0
    (
        .XCLK(CLK),
        .XRES(|RES),
        .UART_RXD(RX),
        .UART_TXD(TX)
    );

endmodule
