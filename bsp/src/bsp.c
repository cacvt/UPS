/*
 * bsp.c
 *
 *  Created on: Sep 13, 2013
 *      Author: shenzy
 */
#include "bsp.h"

void HW_init(void)
{
	HW_sys_init();
	HW_pwm_init();

#ifdef __PROFILE_H__
	profile_init();
#endif /* __PROFILE_H__ */

#ifdef __HW_ADC_H__
	HW_adc_init();
#endif /* __HW_ADC_H__ */

#ifdef __HW_DAC_H__
	HW_dac_init();
#endif /* __HW_DAC_H__ */
}

#ifdef __HW_ADC_H__
void HW_set_ctrl_isr(ISR_FUNC func)
{
    HW_adc_set_isr(func);
}
#endif /* __HW_ADC_H__ */
