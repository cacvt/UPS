/*
 * hw_dac.c
 *
 *  Created on: Nov 7, 2013
 *      Author: Sleepy
 *  Modified by Jianghui Yu
 *  Description:
 *    DAC initialization, needs to be changed based on additional ADC
 */
#include <Init_dac.h>
#include "device.h"
/*******************************************************************************
 * Public variable definition
 ******************************************************************************/
struct DAC dac;

/*******************************************************************************
 * Public function definition
 ******************************************************************************/
interrupt void dac_isr(void)
{
	/* Disable PINT DMA triggering */
	EALLOW;
	DmaRegs.CH3.MODE.bit.PERINTE = PERINT_DISABLE;
	EDIS;
	/* Return from interrupt */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

void HW_dac_init(void)
{
	EALLOW;

	/* McBSP initialization */
	SysCtrlRegs.PCLKCR0.bit.MCBSPBENCLK = 1;							/* Enable clock for McBSPB */
	SysCtrlRegs.LOSPCP.bit.LSPCLK = 0;									/* LSPCLK = SYSCLK (200 MHz)*/

	McbspbRegs.SPCR2.all = 0x0000;										/* Reset FS generator, sample rate generator & transmitter */
	McbspbRegs.SPCR1.all = 0x0000;										/* Reset Receiver, Right justify word, Digital loopback disables. */
//	McbspbRegs.SPCR1.all = 0x0180;										/* Reset Receiver, Right justify word, Digital loopback disables. */

	McbspbRegs.MFFINT.all=0x0;			// Disable all interrupts

	McbspbRegs.RCR2.all=0x0;			// Single-phase frame, 1 word/frame, No companding	(Receive)
	McbspbRegs.RCR1.all=0x0;

	McbspbRegs.XCR2.all=0x0;			// Single-phase frame, 1 word/frame, No companding	(Transmit)
	McbspbRegs.XCR1.all=0x0;

	McbspbRegs.RCR1.bit.RWDLEN1=3;      // 20-bit word
	McbspbRegs.RCR1.bit.RFRLEN1=1;      // 1 word per frame
	McbspbRegs.XCR1.bit.XWDLEN1=3;      // 20-bit word
	McbspbRegs.XCR1.bit.XFRLEN1=1;      // 1 word per frame

	McbspbRegs.SRGR1.bit.FWID = 16;     // Frame Width = 17 CLKG period
	McbspbRegs.SRGR1.bit.CLKGDV = 249;	// CLKG frequency = LSPCLK/250=800kHz
	McbspbRegs.SRGR2.bit.CLKSM = 1;		// CLKSM=1 (If SCLKME=0, i/p clock to SRG is LSPCLK)
	McbspbRegs.SRGR2.bit.FPER = 19;		// FPER = 20 CLKG periods (800kHz/20=40kHz)
	McbspbRegs.SRGR2.bit.FSGM = 1;

	McbspbRegs.PCR.all = 0x0F08;           								/* (CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = 1) */
//	McbspbRegs.SPCR1.bit.CLKSTP = 2;     								/* Together with CLKXP/CLKRP determines clocking scheme */
//	McbspbRegs.PCR.bit.CLKXP = 0;										/* Clock stop mode, transfer on falling edge			*/
//	McbspbRegs.PCR.bit.CLKRP = 0;										/* receiving on rising edge (doesn't matter )			*/
//	McbspbRegs.RCR2.all = 0x81;											/* FSX setup time 1 in master mode */
//	McbspbRegs.XCR2.all = 0x81;											/* FSX setup time 1 in master mode */
//	McbspbRegs.RCR1.all = 0x80;											/* 24-bit word */
//	McbspbRegs.XCR1.all = 0x80;											/* 24-bit word */
//	McbspbRegs.SRGR2.all = 0x2000;										/* CLKSM=1, FPER = 1 CLKG periods */
//	McbspbRegs.SRGR1.all = 0x4;											/* Frame Width = 1 CLKG period, CLKGDV=4 (XCLK = LSPCLK/5) */
	DELAY_US(1.0E7/CPU_FREQ);											/* Delay 2 McBSP clock cycle */
	McbspbRegs.SPCR2.bit.GRST=1;										/* Enable the sample rate generator */
	DELAY_US(1.0E7/CPU_FREQ);											/* Delay 2 McBSP clock cycle */
	McbspbRegs.SPCR2.bit.XRST=1;										/* Release TX from Reset */
	McbspbRegs.SPCR1.bit.RRST=1;										/* Release RX from Reset */
	McbspbRegs.SPCR2.bit.FRST=1;										/* Frame Sync Generator reset */
//	McbspbRegs.SPCR2.bit.XINTM = 0;										/* Send interrupt when XREADY change from 0 to 1 */

	/* DMA initialization */
	SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 1;                   			/* Enable clock of DMA module */

	DmaRegs.CH3.SRC_BEG_ADDR_SHADOW = (Uint32)dac.data;     			/* Point to beginning of source buffer */
	DmaRegs.CH3.SRC_ADDR_SHADOW =     (Uint32)dac.data;     			/* Point to beginning of source buffer */
	DmaRegs.CH3.DST_BEG_ADDR_SHADOW = (Uint32)&McbspbRegs.DXR2.all;     /* Point to beginning of destination buffer */
	DmaRegs.CH3.DST_ADDR_SHADOW =     (Uint32)&McbspbRegs.DXR2.all;
	DmaRegs.CH3.BURST_SIZE.all = 1;                        				/* 2 words per burst */
	DmaRegs.CH3.SRC_BURST_STEP = 1;                        			 	/* Move to next data */
	DmaRegs.CH3.DST_BURST_STEP = 1;                     			    /* Move to DXR1 */
	DmaRegs.CH3.TRANSFER_SIZE = 0;                          			/* Number of bursts per transfer, DMA interrupt will occur after completed transfer */
	DmaRegs.CH3.SRC_TRANSFER_STEP = 0x0001;                 			/* Move to next data address */
	DmaRegs.CH3.DST_TRANSFER_STEP = 0xFFFF;                 			/* Move back to DXR2 */
	DmaRegs.CH3.SRC_WRAP_SIZE = 0xFFFF;                     			/* Larger than TRANSFER_STEP to disable WRAP */
	DmaRegs.CH3.SRC_WRAP_STEP = 0x0000;                     			/* Do not care */
	DmaRegs.CH3.DST_WRAP_SIZE = 0xFFFF;                     			/* Larger than TRANSFER_STEP to disable WRAP */
	DmaRegs.CH3.DST_WRAP_STEP = 0x0000;                     			/* Do not care */
	DmaRegs.CH3.MODE.bit.PERINTSEL = DMA_MXEVTB;            			/* Use McBSPb Xfer event to trigger DMA transfer */
	DmaRegs.CH3.MODE.bit.PERINTE = PERINT_DISABLE;           			/* Peripheral interrupt triggering enable */
	DmaRegs.CH3.MODE.bit.ONESHOT = ONESHOT_DISABLE;        			 	/* One shot mode disabled */
	DmaRegs.CH3.MODE.bit.CONTINUOUS = CONT_ENABLE;        				/* Continuous mode disable */
	DmaRegs.CH3.MODE.bit.DATASIZE = SIXTEEN_BIT;            			/* date size is set to 16-bits */
	DmaRegs.CH3.CONTROL.bit.RUN = 1;									/* Start DMA channel 3 */

	/* Interrupt for DMA to stop after a transfer */
    DmaRegs.CH3.MODE.bit.CHINTMODE = CHINT_END;     /* Interrupt at the end of transfer */
    DmaRegs.CH3.MODE.bit.CHINTE = CHINT_ENABLE;     /* Channel Interrupt to CPU enable */
    DmaRegs.CH3.CONTROL.bit.PERINTCLR = 1;          /* Clear any spurious interrupt flags */
    PieCtrlRegs.PIEIER7.bit.INTx3 = 1;              /* Enable interrupt DINTCH3 in PIE */
    PieVectTable.DINTCH3 = dac_isr;
    /* Enable global interrupt level for DMA interrupt */
    IER |= M_INT7;

	/* IO initialization */

	/* Configure McBSP-A pins using GPIO regs */
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 3;								/* GPIO24 is MDXB pin */
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 3;    							/* GPIO25 is MDRB pin */
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 3;								/* GPIO26 is MCLKXB pin */
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 3;								/* GPIO27 is MFSXB pin */

	/* Enable internal pull-up for the selected I/O pins. Disable pull-ups on output-only pins to reduce power consumption */
	GpioCtrlRegs.GPAPUD.bit.GPIO24 = 1;     							/* Disable pull-up on GPIO24 (MDXB) */
	GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;     							/* Enable pull-up on GPIO25 (MDRB)*/
	GpioCtrlRegs.GPAPUD.bit.GPIO26 = 1;     							/* Disable pull-up on GPIO26 (MCLKXB) */
	GpioCtrlRegs.GPAPUD.bit.GPIO27 = 1;     							/* Disable pull-up on GPIO27 (MFSXB) */

	EDIS;

	HW_dac_reset_buf();
}

void HW_dac_reset_buf(void)
{
	dac.dataptr = 0;
	dac.data_count = 0;
}
