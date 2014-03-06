/*******************************************************************************
 * Copyright © 2014 Virginia Tech Center for Power Electronics Systems
 *
 * Filename:     hw_spi.h
 * author ${user}
 * Description:
 *   SPI-A hardware support for Ethernet module use only.
 *   Not for general purpose use.
 ******************************************************************************/
#define __HW_SPI_C__
#include "hw_spi.h"

/*******************************************************************************
 * Public variable definition
 ******************************************************************************/

/*******************************************************************************
 * Private global variable definition
 ******************************************************************************/


/*******************************************************************************
 * Private function declaration
 ******************************************************************************/

/*******************************************************************************
 * Public  function definition
 ******************************************************************************/

void HW_spi_init(void)
{
    EALLOW;

    /* Enable clock for SPI-A module */
    SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;

    /* Setup GPIO pins for SPIA
     * CS ---------> GPIO19
     * CLK --------> GPIO18
     * MISO -------> GPIO17
     * MOSI -------> GPIO16
     */

    /* Set GPIO pins as SPI IOs */
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0x1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0x1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0x1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0x1;

    /* Set IO pins qualification as asynchronous */
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 0x3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 0x3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 0x3;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 0x3;

    /* Disable pull-up resistors to save power */
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0x1;
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0x1;
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0x1;
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0x1;


    EDIS;
}

/*******************************************************************************
 * Private function definition
 ******************************************************************************/
