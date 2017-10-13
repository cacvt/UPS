/*
 * adc.h
 *
 *  Created on: Sep 12, 2013
 *      Author: shenzy
 *  Modified by Jianghui Yu
 *  Description:
 *    ADC Initialization header file
 */

#ifndef __HW_ADC_H__
#define __HW_ADC_H__
typedef interrupt void (*CTRL_ISR_FUNC)(void);

void HW_adc_init(void);
void HW_adc_set_isr(CTRL_ISR_FUNC func);

typedef struct	//sampled data: 6 cap voltage, 6 arm current						--Jianghui
{

	float Ia1; 		//input AC current, phase A
	float Ib1; 		//input AC current, phase B
	float Ic1; 		//input AC current, phase C
	float Vab1;		//input AC line voltage, phase AB
	float Vbc1;		//input AC line voltage, phase BC
	float Vdcp;		//DC bus voltage, positive
	float Vdcn;		//DC bus voltage, negative
	float VBa;		//Battery voltage
	float IBa;		//Battery current
	float Ia2;		//output AC current, phase A
	float Ib2;		//output AC current, phase B
	float Ic2;		//output AC current, phase C
} SAMPLE_DATA;

#ifndef __HW_ADC_C__
extern int16 adc_res[2][6];
extern float adc_val[2][6];
extern const float ADC_FS[2][6];
extern const float ADC_OS[2][6];
#endif /* __HW_ADC_C__ */

#endif /* __HW_ADC_H__ */
