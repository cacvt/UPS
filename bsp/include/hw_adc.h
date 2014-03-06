/*
 * adc.h
 *
 *  Created on: Sep 12, 2013
 *      Author: shenzy
 */

#ifndef __ADC_H__
#define __ADC_H__
typedef interrupt void(*CTRL_ISR_FUNC)(void);

void HW_adc_init(void);
void HW_adc_set_isr(CTRL_ISR_FUNC func);

void HW_adc_scale(void);


#ifndef __ADC_C__
extern int16 adc_res[2][6];
extern float adc_val[2][6];
extern const float ADC_FS[2][6];
extern const float ADC_OS[2][6];
#endif /* __ADC_C__ */

#endif /* __ADC_H__ */
