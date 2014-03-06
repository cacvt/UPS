/*
 * adc.h
 *
 *  Created on: Sep 12, 2013
 *      Author: shenzy
 */

#ifndef __HW_ADC_H__
#define __HW_ADC_H__

typedef interrupt void(*ISR_FUNC)(void);

void HW_adc_init(void);
void HW_adc_set_isr(ISR_FUNC func);

void HW_adc_scale(void);


#ifndef __HW_ADC_C__
extern Uint16 adc_res[2][6];
extern float adc_val[2][6];
extern const float ADC_FS[2][6];
extern const float ADC_OS[2][6];
#endif /* __HW_ADC_C__ */

#endif /* __HW_ADC_H__ */
