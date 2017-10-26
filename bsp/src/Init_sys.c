//
//  sys.c
//  Created on: Sep 13, 2013
//      Author: shenzy
#include <Init_sys.h>
#include "device.h"

//===============================================================================
// Private function declarations
 //===============================================================================
void sys_init_pll(void);
void sys_init_peripheral_clk(void);
void sys_init_pie_ctrl(void);
void sys_init_pie_vector(void);
interrupt void sys_rsvd_isr(void);

//===============================================================================
// Public function definitions
//===============================================================================

/*
 * Initialize basic hardwares
 */
void HW_sys_init(void)
{
    sys_init_pll();
    sys_init_peripheral_clk();
    sys_init_pie_ctrl();
    sys_init_pie_vector();
}

//===============================================================================
// Private function definitions
//===============================================================================

/*
 *  Initialize PLL
 * Modified from TI InitPll function
 */
void sys_init_pll(void)
{
	EALLOW;
	// DIVSEL MUST be 0 before PLLCR can be changed from 0x0000.
	SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
	// PLL multiplier is set to 20 to get 400M VCOCLK (20M * 20)
	SysCtrlRegs.PLLCR.bit.DIV = 19;
	EDIS;

	// Wait for PLL to lock
	while (SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1);

    /* Switch divider to /2 for 200M SYSCLK (400M / 2)
    //   The time required will depend on the system, this is only an example
	// * Then switch to 1/2*/
    EALLOW;
    // First go to /4 and let the power settle
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 1;
    DELAY_US(50L);
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 2;
    EDIS;
}

/*
 * Initialize peripheral clocks
 * Modified from TI InitPeripheralClocks function
 * Set high speed clock to XCLK/n (xxM)
 * Set low speed clock to XCLK/n (xxM)
 * Turn off all peripheral cocks, they will be enable during peripheral initialization
 */

void sys_init_peripheral_clk(void)
{
   EALLOW;

// HISPCP/LOSPCP prescale register settings, normally it will be set to default values
   SysCtrlRegs.HISPCP.all = 0x001F;
   SysCtrlRegs.LOSPCP.all = 0x0002;

   // Turn off all peripheral clocks but GPIO
   SysCtrlRegs.PCLKCR0.all = 0x0000;
   SysCtrlRegs.PCLKCR1.all = 0x0000;
   SysCtrlRegs.PCLKCR2.all = 0x0000;
   SysCtrlRegs.PCLKCR3.all = 0x2000;
   EDIS;
}

/*
 * This function initializes the PIE control registers to a known state.
 * Copied from TI InitPieCtrl function
 */
void sys_init_pie_ctrl(void)
{
    // Disable Interrupts at the CPU level:
    DINT;

    // Disable the PIE
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

    // Clear all PIEIER registers:
    PieCtrlRegs.PIEIER1.all = 0;
    PieCtrlRegs.PIEIER2.all = 0;
    PieCtrlRegs.PIEIER3.all = 0;
    PieCtrlRegs.PIEIER4.all = 0;
    PieCtrlRegs.PIEIER5.all = 0;
    PieCtrlRegs.PIEIER6.all = 0;
    PieCtrlRegs.PIEIER7.all = 0;
    PieCtrlRegs.PIEIER8.all = 0;
    PieCtrlRegs.PIEIER9.all = 0;
    PieCtrlRegs.PIEIER10.all = 0;
    PieCtrlRegs.PIEIER11.all = 0;
    PieCtrlRegs.PIEIER12.all = 0;

    // Clear all PIEIFR registers:
    PieCtrlRegs.PIEIFR1.all = 0;
    PieCtrlRegs.PIEIFR2.all = 0;
    PieCtrlRegs.PIEIFR3.all = 0;
    PieCtrlRegs.PIEIFR4.all = 0;
    PieCtrlRegs.PIEIFR5.all = 0;
    PieCtrlRegs.PIEIFR6.all = 0;
    PieCtrlRegs.PIEIFR7.all = 0;
    PieCtrlRegs.PIEIFR8.all = 0;
    PieCtrlRegs.PIEIFR9.all = 0;
    PieCtrlRegs.PIEIFR10.all = 0;
    PieCtrlRegs.PIEIFR11.all = 0;
    PieCtrlRegs.PIEIFR12.all = 0;
}

/*
 * Initialize PIE PIE vector table
 * Modified from TI InitPieVectTable function
 * Set all the ISR to rsvd_isr which halt the CPU
 */
void sys_init_pie_vector(void)
{
    int16   i;
    Uint32 *Dest = (void *) &PieVectTable;

    EALLOW;
    for(i=0; i < 128; i++)
        *Dest++ = (Uint32) sys_rsvd_isr;
    EDIS;

    // Enable the PIE Vector Table
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

}

/*
 * ISR function for unused interrupts
 * Halt CPU for error detection
 */
interrupt void sys_rsvd_isr(void)
{
    ESTOP0;
    for (;;);
}

