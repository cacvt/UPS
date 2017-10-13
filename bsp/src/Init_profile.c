/*
 * profile.c
 *
 *  Created on: Nov 19, 2013
 *      Author: shenzy
 */
#include <Init_profile.h>
#include "device.h"
/*******************************************************************************
 * Public variable definition
 ******************************************************************************/
volatile struct PROFILE profile;

/*******************************************************************************
 * Public function definition
 ******************************************************************************/
void profile_init(void)
{
	EALLOW;
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = 1;			/* Enable clock for CPU timer0 */
	EDIS;
    CpuTimer0Regs.TPR.all  = 0;							/* Set timer clock to SYSCLKOUT */
    CpuTimer0Regs.TPRH.all = 0;
    CpuTimer0Regs.TCR.bit.TSS = 1;						/* Stop timer */
    CpuTimer0Regs.PRD.all  = 0xFFFFFFFA;				/* Set timer period register */
    CpuTimer0Regs.TCR.bit.TRB = 1;						/* Reload counter register */
    CpuTimer0Regs.TCR.bit.TSS = 0;						/* Start timer */

    profile.cycle_isr = 0;
    profile.cycle_all = 0;
}



