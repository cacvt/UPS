/*
 * hw_adc_constants.h
 *
 *  Created on: Oct 10, 2013
 *      Author: shenzy
 */

#ifndef __HW_ADC_CONSTANTS_H__
#define __HW_ADC_CONSTANTS_H__

/* ADC scaling function */
#define ADC_BIT_RES     (1.0/65535.0)

#define ADC_FS_VAL {{ \
        427.08333759 * ADC_BIT_RES,	/* ADC1 CH0 as Vbus*/	\
        -65.141298487 * ADC_BIT_RES,	/* ADC1 CH1 as Iac*/	\
		0.00000000E+00 * ADC_BIT_RES,	/* ADC1 CH2 */			\
		0.00000000E+00 * ADC_BIT_RES,	/* ADC1 CH3 */			\
		0.00000000E+00 * ADC_BIT_RES,	/* ADC1 CH4 */			\
		0.00000000E+00 * ADC_BIT_RES	/* ADC1 CH5 */			\
    }, { \
    	0.00000000E+00 * ADC_BIT_RES,	/* ADC2 CH0 */			\
        0.00000000E+00 * ADC_BIT_RES,	/* ADC2 CH1 */			\
	    -1076.876321173 * ADC_BIT_RES,	/* ADC2 CH2 as Vbat old: 1063.187215395*/	\
        -1063.187215395 * ADC_BIT_RES,	/* ADC2 CH3 as Vac*/	\
	    -55.039042682 * ADC_BIT_RES,	/* ADC2 CH4 as Ibat* old: -64.670875151*/	\
        0.00000000E+00 * ADC_BIT_RES 	/* ADC2 CH5 */			\
}}

#define ADC_OS_VAL {{ \
       -214.2806457170,          		/* ADC1 CH0 as Vbus*/	\
		18.0278438149,					/* ADC1 CH1 as Iac*/	\
        0.00000000E+00,					/* ADC1 CH2 */			\
		0.00000000E+00,					/* ADC1 CH3 */			\
		0.00000000E+00,					/* ADC1 CH4 */			\
		0.00000000E+00					/* ADC1 CH5 */			\
    }, { \
    	0.00000000E+00,					/* ADC2 CH0 */			\
        0.00000000E+00,					/* ADC2 CH1 */			\
        -0.470396861,					/* ADC2 CH2 as Vbat old:0.4644172399*/	\
        -0.4644172399,					/* ADC2 CH3 as Vac*/	\
        15.386548568,					/* ADC2 CH4 as Ibat old:18.0791945672*/	\
		0.00000000E+00					/* ADC2 CH5 */			\
}}

#endif /* __HW_ADC_CONSTANTS_H__ */
