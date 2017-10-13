/*
 * hw_adc_constants.h
 *
 *  Created on: Oct 10, 2013
 *      Author: shenzy
 */

#ifndef __HW_ADC_CONSTANTS_H__
#define __HW_ADC_CONSTANTS_H__

/* ADC scaling function */
#define ADC_BIT_RES     (3.0/65535.0)


// the gain of SC board and Analog_Offset is (1+200k/R6)*0.15 ,R6=470k, measurement: 0.45
// the gain of ADC chip is 4095 bit / 3 V
// the gain of HW_adc_scale is 16*ADC_FS_VAL/65535
// ADC_FS_VAL is calculated so that overall gain is 1
// calculationg result is 14.033, need to be tuned                         --Jianghui
#define ADC_FS_VAL {{ \
        1 * ADC_BIT_RES,	/* ADC1 CH0 '12'*/	\
        1 * ADC_BIT_RES,	/* ADC1 CH1 '11'*/	\
		1 * ADC_BIT_RES,	/* ADC1 CH2 '8'*/			\
		1 * ADC_BIT_RES,	/* ADC1 CH3 '7'*/			\
		1 * ADC_BIT_RES,	/* ADC1 CH4 '4'*/			\
		1 * ADC_BIT_RES	/* ADC1 CH5 '3'*/			\
    }, { \
    	1 * ADC_BIT_RES,	/* ADC2 CH0 '10'*/			\
        1 * ADC_BIT_RES,	/* ADC2 CH1 '9'*/			\
	    1 * ADC_BIT_RES,	/* ADC2 CH2 '6'*/	\
        1 * ADC_BIT_RES,	/* ADC2 CH3 '5'*/	\
	    1 * ADC_BIT_RES,	/* ADC2 CH4 '2'*/	\
        1 * ADC_BIT_RES 	/* ADC2 CH5 '1'*/			\
}}

// actual offset after Analog_Offset is 1.3*1.15=1.495 V
// offset in ADC is 1.5 V
// compensate with 0.005/3*65536
#define ADC_OS_VAL {{ \
        -110,          		/* ADC1 CH0 '12'*/	\
		-110,					/* ADC1 CH1 '11'*/	\
        -110,					/* ADC1 CH2 '8'*/			\
		-110,					/* ADC1 CH3 '7'*/			\
		-110,					/* ADC1 CH4 '4'*/			\
		-110					/* ADC1 CH5 '3'*/			\
    }, { \
    	-110,				/* ADC2 CH0 '10'*/			\
        -110,					/* ADC2 CH1 '9'*/			\
        -110,					/* ADC2 CH2 '6'*/	\
        -110,					/* ADC2 CH3 '5'*/	\
        -110,					/* ADC2 CH4 '2'*/	\
		-110					/* ADC2 CH5 '1'*/			\
}}



#endif /* __HW_ADC_CONSTANTS_H__ */
