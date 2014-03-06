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

#define ADC_FS_VAL {{                           \
        2.0 * ADC_BIT_RES,      /* ADC1 CH0 */  \
        3.0 * ADC_BIT_RES,      /* ADC1 CH1 */  \
        4.0 * ADC_BIT_RES,      /* ADC1 CH2 */  \
        5.0 * ADC_BIT_RES,      /* ADC1 CH3 */  \
        6.0 * ADC_BIT_RES,      /* ADC1 CH4 */  \
        7.0 * ADC_BIT_RES       /* ADC1 CH5 */  \
    }, {                                        \
        2.0 * ADC_BIT_RES,      /* ADC2 CH0 */  \
        3.0 * ADC_BIT_RES,      /* ADC2 CH1 */  \
        4.0 * ADC_BIT_RES,      /* ADC2 CH2 */  \
        5.0 * ADC_BIT_RES,      /* ADC2 CH3 */  \
        6.0 * ADC_BIT_RES,      /* ADC2 CH4 */  \
        7.0 * ADC_BIT_RES       /* ADC2 CH5 */  \
}}

#define ADC_OS_VAL {{                           \
        2.0,                    /* ADC1 CH0 */  \
        3.0,                    /* ADC1 CH1 */  \
        4.0,                    /* ADC1 CH2 */  \
        5.0,                    /* ADC1 CH3 */  \
        6.0,                    /* ADC1 CH4 */  \
        7.0                     /* ADC1 CH5 */  \
    }, {                                        \
        2.0,                    /* ADC2 CH0 */  \
        3.0,                    /* ADC2 CH1 */  \
        4.0,                    /* ADC2 CH2 */  \
        5.0,                    /* ADC2 CH3 */  \
        6.0,                    /* ADC2 CH4 */  \
        7.0                     /* ADC2 CH5 */  \
}}

#endif /* __HW_ADC_CONSTANTS_H__ */
