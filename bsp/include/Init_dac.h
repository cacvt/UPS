/*
 * hw_dac.h
 *
 *  Created on: Nov 7, 2013
 *      Author: Sleepy
 *  Modified by Jianghui Yu
 *  Description:
 *    DAC Initialization header file, needs to be changed based on additional ADC
 */

#ifndef __HW_DAC_H__
#define __HW_DAC_H__

/*******************************************************************************
 * Constant definition
 ******************************************************************************/
#define DAC_BIT_RESOLUTION		(5.0/4096.0)
#define DAC_FS		1/(1+200.0/470.0)/0.15 //gain for additional ADC channel
#define DAC_OS		2.5*1.15               //offset for additional ADC channel

enum DAC_ERR {
	DAC_ERR_BUSY = -32767,
	DAC_NO_ERR = 0
};

#define DAC_STORE		0x0000
#define DAC_UPDATE		0x0010
#define DAC_UPDATE_SIM	0x0020
#define DAC_UPDATE_ALL	0x0034

/*******************************************************************************
 * Macro definition
 ******************************************************************************/
#define DAC_BUSY_TANSFER (McbspbRegs.SPCR2.bit.XRDY == 0 || DmaRegs.CH3.BURST_COUNT.all !=0 || DmaRegs.CH3.TRANSFER_COUNT !=0)

/*******************************************************************************
 * Private type definition
 ******************************************************************************/
struct DAC {
	int data_count;		/* number of data to send */
	int	dataptr;		/* Buffer data pointer */
	int data[8];		/* Data buffer to load to DAC */
};

/*******************************************************************************
 * Public variable declaration
 ******************************************************************************/
#ifndef __HW_DAC_C__
extern struct DAC dac;
#endif /* __HW_DAC_C__ */

/*******************************************************************************
 * Public function declaration
 ******************************************************************************/
void HW_dac_init(void);
void HW_dac_reset_buf(void);

/*******************************************************************************
 * Inline functions for faster execution
 ******************************************************************************/
#include "hw_dac_inline.h"

#endif /* __HW_DAC_H__ */
