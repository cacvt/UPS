/*
 * bsp.h
 *
 *  Created on: Sep 13, 2013
 *      Author: shenzy
 */

#ifndef __BSP_H__
#define __BSP_H__

#include "device.h"
#include "profile.h"
#include "hw_sys.h"
#include "hw_adc.h"
#include "hw_ethernet.h"
#include "hw_CC_LED.h"
#include "hw_cpld.h"
#include "hw_dac.h"
#include "hw_modulator.h"
#include "hw_pwm.h"
#include "hw_spi.h"

/*******************************************************************************
 * Public functions declarations
 ******************************************************************************/

/* Wrapper functions of lower level functions */
void HW_init(void);

#ifdef __HW_ADC_H__

void HW_set_ctrl_isr(CTRL_ISR_FUNC func);

#endif /* __HW_ADC_H__ */

#endif /* __BSP_H__ */
