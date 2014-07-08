//==============================================================================
// Copyright © 2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     main.c
// Author:       Z Shen
// Description:
//   Sample entry point of the whole code
//==============================================================================

#include "bsp.h"

//==============================================================================
// Global variable definitions

// Variable for Ethernet submodule
unsigned int busy_sending;
unsigned int rtn_buf[400];

//test variables
unsigned long long led_tick;
unsigned long long hex_tick;
unsigned int hex_led_cntr;
unsigned int tmp;
//==============================================================================
// Function declarations
interrupt void ctrl_isr(void);
void main_loop(void) __attribute__((noreturn));

//==============================================================================
// Function definitions

void main(void)
{

	HW_init();						/* Initialize hardware*/
	HW_set_ctrl_isr(ctrl_isr);      /* Set control ISR */

	// System tick initialization
	tick = 0;
	led_tick = 0;
	hex_tick = 0;
	hex_led_cntr = 0;
	EINT;
	main_loop();					/* Start the super loop */

}


//=============================================================================
// The super loop
//
//=============================================================================
void main_loop(void)
{for (;;) {
//=============================================================================
// Starting point of task code

	//=========================================================================
	// Ethernet data processing task
	//   Task is executed every loop to maximize through output
	HW_eth_task();

	//=========================================================================
    // LED blinking task
    if( tick >= led_tick) {
		while (led_tick <= tick)
			led_tick += (0.5 * CTRL_FREQ);

		CC_LED0_TOOGLE;
		CC_LED1_TOOGLE;
	}

    //=========================================================================
    // HEX LED update blinking task
    if( tick >= hex_tick) {
		while (hex_tick <= tick)
		hex_tick += (0.1 * CTRL_FREQ);
    	tmp = HW_cpld_reg_read_poll(REG_CPLD_INPUT0);
    	HW_cpld_reg_read_poll(REG_CPLD_INPUT1);
    	HW_cpld_reg_write_poll(REG_HEX_LED, hex_led_cntr++);
    	HW_cpld_reg_write_poll(REG_CPLD_OUTPUT1, 0x40);
    	HW_cpld_reg_write_poll(REG_CPLD_OUTPUT1, 0x00);
    	HW_cpld_reg_write_poll(REG_CPLD_SET1, 0x40);
    	HW_cpld_reg_write_poll(REG_CPLD_CLEAR1, 0x40);
    	HW_cpld_reg_write_poll(REG_CPLD_TOGGLE1, 0x40);
    	HW_cpld_reg_write_poll(REG_CPLD_TOGGLE1, 0x40);
//    	HW_cpld_reg_write_poll(REG_CPLD_SET1, 0x40);

	}

// End point of task code
//=============================================================================
}}

//=============================================================================
// The interrupt routine for converter control
// - Macro is preferred in this section
// - Use global variable if the state of it is to be maintained during different
//   calls
// - Use tempoary variable declared in the function if the state does not matter
//=============================================================================
interrupt void ctrl_isr(void)
{
	PROFILE_ISR_START;					    // Execution time monitoring
    IER = M_INT2|M_INT3;					// Trip zone interrupt for protect-
    										//   -ion & PWM update ISR to update
    										//   PWM registers
    EINT;     								// Re-enable high level interrupts
    tick++;								    // increase time tick

//==============================================================================
// User code begin


//  User code end
//==============================================================================
    DINT;									    // Disable global interrupt
    PROFILE_ISR_STOP;						    // Execution time monitoring
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;     // Acknowledge interrupt
    											//   to accept new ones
}
