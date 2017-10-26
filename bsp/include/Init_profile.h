/*
 * profile.h
 *
 *  Created on: Nov 19, 2013
 *      Author: shenzy
 */

#ifndef __PROFILE_H__
#define __PROFILE_H__
#include "device.h"
/*******************************************************************************
 * Type definition
 ******************************************************************************/
struct PROFILE {
	long cycle_isr;
	long cycle_all;
};
/*******************************************************************************
 * Macro definition
 ******************************************************************************/
#define PROFILE_ISR_START \
	profile.cycle_all;								/* Dummy line to keep variable information */ \
	asm("	MOVW	DP, #0x30 "); 					/* #_CpuTimer0Regs */ \
	asm("	MOVL	ACC, @0x0 ");					/* @_CpuTimer0Regs */ \
	asm("	OR		@0x4,#0x0020");					/* @_CpuTimer0Regs+4; CpuTimer0Regs.TCR */ \
	asm("	NEG		ACC "); \
	asm("	MOVW	DP,#_profile+2 "); 				/* profile.cycle_all */ \
	asm("	MOVL	@_profile+2,ACC ")

#define PROFILE_ISR_STOP \
	asm("	MOVB	ACC, #0");	\
	asm("	MOVW	DP, #0x30 "); 					/* #_CpuTimer0Regs */ \
	asm("	SUBL	ACC, @0x0 ");					/* @_CpuTimer0Regs */ \
	asm("	MOVW	DP, #_profile "); 				/* profile.cycle_isr */ \
	asm("	ADDB	ACC, #0x28 ");					/* Adjust to include MACRO time and function house keeping codes */ \
	asm("	MOVL	@_profile, ACC ")

/*******************************************************************************
 * Public variable declaration
 ******************************************************************************/
#ifndef __PROFILE_C__
extern volatile struct PROFILE profile;
#endif

/*******************************************************************************
 * Public function declaration
 ******************************************************************************/
void profile_init(void);

#endif /* __PROFILE_H__ */
