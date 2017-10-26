
/*
 * hw_CC_LED.c
 *
 *  Created on: Feb 7, 2014
 *      Author: shenzy
 */
#define __HW_CC_LED_C__
#include "device.h"
#include <Init_LED.h>
/*******************************************************************************
 * Public variable definition
 ******************************************************************************/
void HW_cc_led_init()
{
    /* Configure GPIOs as output to control Hex LED
     * 54, 55 */
	EALLOW;
	/* Set pins as IO */
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;
    /* Set IOs as output */
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 1;

    /* Disable pull-up resistor to save power */
	GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;
	GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;
    EDIS;

    /* Turn on LEDs */
    CC_LED0_ON;
    CC_LED1_ON;
}




