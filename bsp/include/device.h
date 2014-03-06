/*
 * device.h
 *
 *  Created on: Sep 16, 2013
 *      Author: Z Shen
 */

#ifndef __DEVICE_H__
#define __DEVICE_H__

/* Include device header files supplied by TI */
#include "DSP2834x_Device.h"
/* Include constant definition header files supplied by TI */
#include "DSP2834x_EPwm_Defines.h"
#include "DSP2834x_Dma_Defines.h"
#include "DSP2834x_I2c_Defines.h"

/* Declaration of functions provide by TI without header file */
extern void DSP28x_usDelay(Uint32 Count);
#define DELAY_US(A)  DSP28x_usDelay(((((long double) A * 1000.0L) / (long double)5.000L) - 9.0L) / 5.0L)

/********************************************************************************
 * Configuration constants
 */
#define CPU_FREQ            200000000L      // CPU frequency
#define CTRL_FREQ           50000L          // Control interrupt frequency
#define SW_PER_SAMPLE       10L              // Number of switching per control period

#define DEAD_TIME_DAB_PRI   10L             // Dead time of DAB primary side phase leg in CPU clock cycle
#define DEAD_TIME_DAB_SEC   10L             // Dead time of DAB secondary side phase leg in CPU clock cycle
#define DEAD_TIME_PFC_HF    10L             // Dead time of PFC high frequency leg in CPU clock cycle
#define DEAD_TIME_PFC_LF    10L             // Dead time of PFC low frequency leg in CPU clock cycle

#endif /* __DEVICE_H__ */
