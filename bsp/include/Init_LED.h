/*
 * hw_CC_LED.h
 *
 *  Created on: Jan 10, 2014
 *      Author: Z Shen
 */

#ifndef __HW_CC_LED_H__
#define __HW_CC_LED_H__

/*******************************************************************************
 * Public function definitions
 ******************************************************************************/
#define CC_LED0_ON		GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1
#define CC_LED1_ON		GpioDataRegs.GPBCLEAR.bit.GPIO55 = 1
#define CC_LED0_OFF		GpioDataRegs.GPBSET.bit.GPIO54 = 1
#define CC_LED1_OFF		GpioDataRegs.GPBSET.bit.GPIO55 = 1
#define CC_LED0_TOOGLE	GpioDataRegs.GPBTOGGLE.bit.GPIO54 = 1
#define CC_LED1_TOOGLE	GpioDataRegs.GPBTOGGLE.bit.GPIO55 = 1

/*******************************************************************************
 * Public function definitions
 ******************************************************************************/
void HW_cc_led_init();


#endif /* __HW_CC_LED_H__ */
