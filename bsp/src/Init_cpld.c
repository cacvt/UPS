//==============================================================================
// Copyright �2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     hw_cpld.c
// Author:       Z Shen
// Description:
//   Access registers in CPLD for the functions implemented in CPLD
//
//==============================================================================
#define __HW_CPLD_C__
#include "device.h"
#include <Init_cpld.h>
#include <Func_CPLD.h>

//==============================================================================
// Private function declaration

//==============================================================================
// Public function definition

void HW_cpld_init(void)
{

	EALLOW;

	// McBSPA initialization
	SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK = 1;							// Enable clock for McBSPA
	SysCtrlRegs.LOSPCP.bit.LSPCLK = 0;									// LSPCLK = SYSCLK (200 MHz)*/

	McbspaRegs.SPCR2.all = 0x0000;		// Reset FS generator, sample rate generator & transmitter
	McbspaRegs.SPCR1.all = 0x1080;		// Reset Receiver, Right justify word, Digital loopback disables.

	McbspaRegs.PCR.all = 0x0F0F;        // (CLKXM=CLKRM=FSXM=FSRM= 1, FSXP = FSRP = 1)
	McbspaRegs.SPCR1.bit.CLKSTP = 2;    // Together with CLKXP/CLKRP determines clocking scheme
	McbspaRegs.PCR.bit.CLKXP = 1;		// Clock stop mode, transfer on falling edge			*/
	McbspaRegs.PCR.bit.CLKRP = 1;		// receiving on rising edge (doesn't matter )			*/
	McbspaRegs.RCR2.all = 0xA1;			// FSX setup time 1 in master mode
	McbspaRegs.XCR2.all = 0xA0;			// FSX setup time 1 in master mode
	McbspaRegs.RCR1.all = 0xA0;			// 32-bit word
	McbspaRegs.XCR1.all = 0xA0;			// 32-bit word
	McbspaRegs.SRGR2.all = 0x2000;										// CLKSM=1, FPER = 1 CLKG periods
	McbspaRegs.SRGR1.all = 0x4;											// Frame Width = 1 CLKG period, CLKGDV=5 (XCLK = LSPCLK/6)
	DELAY_US(1.0E7/CPU_FREQ);											// Delay 2 McBSP clock cycle
	McbspaRegs.SPCR2.bit.GRST=1;										// Enable the sample rate generator
	DELAY_US(1.0E7/CPU_FREQ);											// Delay 2 McBSP clock cycle
	McbspaRegs.SPCR2.bit.XRST=1;										// Release TX from Reset
	McbspaRegs.SPCR1.bit.RRST=1;										// Release RX from Reset
	McbspaRegs.SPCR2.bit.FRST=1;										// Frame Sync Generator reset
	McbspaRegs.SPCR2.bit.XINTM = 0;										// Send interrupt when XREADY change from 0 to 1
	McbspaRegs.SPCR2.bit.FREE = 1;

//	// DMA initialization
//	SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 1;                   			// Enable clock of DMA module
//
//	DmaRegs.CH4.SRC_BEG_ADDR_SHADOW = (Uint32)dac.data;     			// Point to beginning of source buffer
//	DmaRegs.CH4.SRC_ADDR_SHADOW =     (Uint32)dac.data;     			// Point to beginning of source buffer
//	DmaRegs.CH4.DST_BEG_ADDR_SHADOW = (Uint32)&McbspaRegs.DXR2.all;     // Point to beginning of destination buffer
//	DmaRegs.CH4.DST_ADDR_SHADOW =     (Uint32)&McbspaRegs.DXR2.all;
//	DmaRegs.CH4.BURST_SIZE.all = 1;                        				// 2 words per burst
//	DmaRegs.CH4.SRC_BURST_STEP = 1;                        			 	// Move to next data
//	DmaRegs.CH4.DST_BURST_STEP = 1;                     			    // Move to DXR1
//	DmaRegs.CH4.TRANSFER_SIZE = 0;                          			// Number of bursts per transfer, DMA interrupt will occur after completed transfer
//	DmaRegs.CH4.SRC_TRANSFER_STEP = 0x0001;                 			// Move to next data address
//	DmaRegs.CH4.DST_TRANSFER_STEP = 0xFFFF;                 			// Move back to DXR2
//	DmaRegs.CH4.SRC_WRAP_SIZE = 0xFFFF;                     			// Larger than TRANSFER_STEP to disable WRAP
//	DmaRegs.CH4.SRC_WRAP_STEP = 0x0000;                     			// Do not care
//	DmaRegs.CH4.DST_WRAP_SIZE = 0xFFFF;                     			// Larger than TRANSFER_STEP to disable WRAP
//	DmaRegs.CH4.DST_WRAP_STEP = 0x0000;                     			// Do not care
//	DmaRegs.CH4.MODE.bit.PERINTSEL = DMA_MXEVTB;            			// Use McBSPb Xfer event to trigger DMA transfer
//	DmaRegs.CH4.MODE.bit.PERINTE = PERINT_DISABLE;           			// Peripheral interrupt triggering enable
//	DmaRegs.CH4.MODE.bit.ONESHOT = ONESHOT_DISABLE;        			 	// One shot mode disabled
//	DmaRegs.CH4.MODE.bit.CONTINUOUS = CONT_ENABLE;        				// Continuous mode disable
//	DmaRegs.CH4.MODE.bit.DATASIZE = SIXTEEN_BIT;            			// date size is set to 16-bits
//	DmaRegs.CH4.CONTROL.bit.RUN = 1;									// Start DMA channel 3
//
//	// Interrupt for DMA to stop after a transfer
//    DmaRegs.CH4.MODE.bit.CHINTMODE = CHINT_END;     // Interrupt at the end of transfer
//    DmaRegs.CH4.MODE.bit.CHINTE = CHINT_ENABLE;     // Channel Interrupt to CPU enable
//    DmaRegs.CH4.CONTROL.bit.PERINTCLR = 1;          // Clear any spurious interrupt flags
//    PieCtrlRegs.PIEIER7.bit.INTx3 = 1;              // Enable interrupt DINTCH4 in PIE
//    PieVectTable.DINTCH4 = dac_isr;
//    // Enable global interrupt level for DMA interrupt
//    IER |= M_INT7;

	// IO initialization

	// Configure McBSP-A GPIO pins
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 2;	// GPIO20 is MDXA pin
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 2;	// GPIO21 is MDRA pin
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 2;	// GPIO22 is MCLKXA pin
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 2;	// GPIO23 is MFSXA pin

	// Enable internal pull-up for the selected I/O pins. Disable pull-ups on output-only pins to reduce power consumption
	GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;     // Disable pull-up on GPIO20 (MDXA)
	GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;     // Enable pull-up on GPIO20 (MDRA)
	GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;  	// Enable pull-up on GPIO22 (MCLKXA)
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;     // Enable pull-up on GPIO23 (MFSXA)

	// Send CS signal to SPI_CS pin on CPLD   -------------!!!---------------  49->23
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;	// GPIO23 as IO
	GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;		// As output
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1;		// Disable pull-up to save power
	GpioDataRegs.GPASET.bit.GPIO23 = 1;		// Set high to send MFSXA to SPI_CS

	// Set GPIO48 as reset for CPLD             49->48
	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;	// GPIO48 as IO
	GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;		// As output
	GpioCtrlRegs.GPBPUD.bit.GPIO48 = 1;		// Disable pull-up to save power
	GpioDataRegs.GPBSET.bit.GPIO48 = 1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;    // GPIO14 is err set pin
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;     // As output
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;     // Disable pull-up to save power

	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;    // GPIO15 is err clr pin
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;     // As output
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;     // Disable pull-up to save power

	GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;    // GPIO84
    GpioCtrlRegs.GPCDIR.bit.GPIO84 = 0;     // As input
    GpioCtrlRegs.GPCPUD.bit.GPIO84 = 1;     // Disable pull-up to save power

	GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;    // GPIO85
    GpioCtrlRegs.GPCDIR.bit.GPIO85 = 1;     // As output
    GpioCtrlRegs.GPCPUD.bit.GPIO85 = 1;     // Disable pull-up to save power

	GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 0;    // GPIO86
    GpioCtrlRegs.GPCDIR.bit.GPIO86 = 1;     // As output
    GpioCtrlRegs.GPCPUD.bit.GPIO86 = 1;     // Disable pull-up to save power

	GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 0;    // GPIO87
    GpioCtrlRegs.GPCDIR.bit.GPIO87 = 1;     // As output
    GpioCtrlRegs.GPCPUD.bit.GPIO87 = 1;     // Disable pull-up to save power

	EDIS;

	HW_cpld_reset();
	CPLD_NO_FAULT_SET;
	CPLD_FAULT_CLR;
}

void HW_cpld_reset(void)
{
	GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1;	// Set low
	DELAY_US(500000);
	GpioDataRegs.GPBSET.bit.GPIO48 = 1;		// Set high
}
void HW_cpld_reg_write_poll(unsigned int addr, unsigned int data)
{
	while(McbspaRegs.SPCR2.bit.XRDY == 0) {};
	McbspaRegs.DXR2.all = addr | 0x8000;
	McbspaRegs.DXR1.all = data;

}

unsigned int HW_cpld_reg_read_poll(unsigned int addr)
{
	unsigned int data;
	while(McbspaRegs.SPCR2.bit.XRDY == 0) {};	// Wait for the end of last transmission
	McbspaRegs.SPCR1.bit.RRST = 0;
	McbspaRegs.SPCR1.bit.RRST = 1;
	McbspaRegs.DXR2.all = addr & 0x7FFF;		// Write address
	McbspaRegs.DXR1.all = 0xFFFF;				// Dummy write
	while(McbspaRegs.SPCR1.bit.RRDY == 0) {};	// Wait RRDY bit
	data = McbspaRegs.DRR1.all;					// Read data
	return data;

}
//==============================================================================
// Private function definition
