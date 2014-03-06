/*
 * main.c
 *
 *  Created on: Nov 7, 2013
 *      Author: Sleepy
 */
#include "bsp.h"

/*******************************************************************************
 * Global variable definitions
 ******************************************************************************/
unsigned long long tick;
/* for debug only */
volatile int tmp;
volatile int res1, res2;
volatile long cyc[1000];
volatile int ptr;
volatile unsigned long test;

/*******************************************************************************
 * Function declaration
 ******************************************************************************/
interrupt void ctrl_isr(void);
void main_loop(void) __attribute__((noreturn));

void main(void)
{
	HW_init();						/* Initialize hardware*/
	HW_set_ctrl_isr(ctrl_isr);      /* Set control ISR */
	HW_dac_update_all(0);
	DELAY_US(1000);
	tmp=20000;
	ptr = 0;
	for (ptr=0;ptr<1000;ptr++)
		cyc[ptr] = 0;
	ptr = 0;
	EINT;
	main_loop();					/* Start the super loop */

}

/*******************************************************************************
 * The super loop for everything
 ******************************************************************************/
void main_loop(void)
{
    do {
    	if( tick >= next) {

    	}
    	ams(" NOP");
    } while(1);
}

/*******************************************************************************
 * The interrupt routine for converter control
 * - Macro is preferred in this section
 * - Use global variable if the state of it is to be maintained during different
 *   calls
 * - Use tempoary variable declared in the function if the state does not matter
 *
 ******************************************************************************/
interrupt void ctrl_isr(void)
{
	PROFILE_ISR_START;					    /* Execution time monitoring */
    IER = M_INT2|M_INT3;					/* Trip zone interrupt for protection & PWM update ISR to update PWM registers */
    EINT;     								/* Re-enable high level interrupts */
    tick++;								    /* increase time tick */

/*******************************************************************************
 * User code begin
 ******************************************************************************/

    if(ptr<1000) {
		cyc[ptr++]=profile.cycle_all;
		cyc[ptr++]=profile.cycle_isr;
	}

    HW_dac_store_ch(0, tmp);
    HW_dac_store_ch(1, tmp);
    HW_dac_store_ch(2, tmp);
    HW_dac_store_ch(3, tmp);
    HW_dac_load_update();
    tmp=-tmp;

/*******************************************************************************
 * User code end
 ******************************************************************************/

    DINT;									    /* Disable global interrupt */
    PROFILE_ISR_STOP;						    /* Execution time monitoring */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;     /* Acknowledge interrupt to accept new ones */
}
