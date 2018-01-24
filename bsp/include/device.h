/*
 * device.h
 *
 *  Created on: Sep 16, 2013
 *      Author: Z Shen
 */

#ifndef __DEVICE_H__
#define __DEVICE_H__

//==============================================================================
// Header files

#include "DSP2834x_Device.h"		// Peripheral register definition
#include "DSP2834x_EPwm_Defines.h"	// Peripheral configuration constants
#include "DSP2834x_Dma_Defines.h"
#include "DSP2834x_I2c_Defines.h"

//==============================================================================
// Declaration of functions provide by TI without header file
extern void DSP28x_usDelay(Uint32 Count);
#define DELAY_US(A)  DSP28x_usDelay(((((long double) A * 1000.0L) / (long double)5.000L) - 9.0L) / 5.0L)

//==============================================================================
// Custom data type definition
typedef interrupt void(*ISR_FUNC)(void);

//==============================================================================
// Configuration constants

#define Vc 300.0                            //Set cap voltage refernece
#define IBa0 10.0                             //Set Battery current reference

#define CPU_FREQ            200000000L      // CPU frequency                             200M
#define CTRL_FREQ           10000L          // Control interrupt frequency               10k    control calculation cannot finish in 10 ns
#define SW_PER_SAMPLE       1L              // Number of switching per control period    switching frequency: 10k
//#define CTRL_CLK            0.00005         // Control clock period                      50 us
#define CTRL_CLK            0.0001         // Control clock period                      100 us
#define DT                  0.04            // Deadtime                                  400 ns = 0.04 * Switching cycle
#define EPWM_DB   400 											//Set dead band 1us (DB= 0.05us*EPWM_DB)

#define DEAD_TIME_DAB_PRI   10L             // Dead time of DAB primary side phase leg in CPU clock cycle
#define DEAD_TIME_DAB_SEC   10L             // Dead time of DAB secondary side phase leg in CPU clock cycle
#define DEAD_TIME_PFC_HF    10L             // Dead time of PFC high frequency leg in CPU clock cycle
#define DEAD_TIME_PFC_LF    10L             // Dead time of PFC low frequency leg in CPU clock cycle

/*******************************************************************************
 * Constant derived from configuration constants
 ******************************************************************************/
#define SW_FREQ     (CTRL_FREQ * SW_PER_SAMPLE)
#define SW_PRD      (CPU_FREQ / SW_FREQ)
#define SW_PRD_HALF (SW_PRD / 2)

#endif /* __DEVICE_H__ */
