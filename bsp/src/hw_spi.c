//==============================================================================
// Copyright � 2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     hw_spi.h
// author ${user}
// Description:
//   SPI-A hardware support for Ethernet module use only.
//   Not for general purpose use.
//==============================================================================

#define __HW_SPI_C__
#include "device.h"
#include "hw_spi.h"

//==============================================================================
// Public variable definition

//==============================================================================
// Private global variable definition

//==============================================================================
// Private function declaration

//==============================================================================
// Public  function definition

void HW_spi_init(void)
{
    EALLOW;

    //--------------------------------------------------------------------------
    // Enable clock for SPI-A module

    SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;

    //--------------------------------------------------------------------------
    // Put SPI into reset

    SpiaRegs.SPICCR.bit.SPISWRESET = 0x0;

    //--------------------------------------------------------------------------
	// Setup GPIO pins for SPIA
	//  CS ---------> GPIO19
	//  CLK --------> GPIO18
	//  MISO -------> GPIO17
	//  MOSI -------> GPIO16

    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0x1;	// Set GPIO pins as SPI IOs
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0x1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0x1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0x1;


    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 0x3;	// Set IO pins qualification as
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 0x3;	// asynchronous
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 0x3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 0x3;


    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0x1;	// Disable pull-up resistors to save
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0x1;	// power
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0x1;
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0x1;

    //--------------------------------------------------------------------------
    // Setup SPI module parameters

    SpiaRegs.SPICCR.bit.SPICHAR = 0xF;		// 16b character
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0x1;	// Data out @ falling edge
     	 	 	 	 	 	 	 	 	 	//   Data in @ rising edge

    SpiaRegs.SPICTL.bit.CLK_PHASE = 0x1;	// CLK signal is delayed by half
    										//   cycle
	SpiaRegs.SPICTL.bit.MASTER_SLAVE = 0x1;	// Master mode
	SpiaRegs.SPICTL.bit.TALK = 0x1;			// Enable talk

	SpiaRegs.SPIBRR =0x007F;				// Clock frequency

	SpiaRegs.SPIPRI.bit.FREE = 1;           // Free run mode. Breakpoints
											//   don't disturb transmission

	//--------------------------------------------------------------------------
	// Relinquish SPI from Reset

	SpiaRegs.SPICCR.bit.SPISWRESET =0x1;

	EDIS;
}

/*******************************************************************************
 * Private function definition
 ******************************************************************************/
