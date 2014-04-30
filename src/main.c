/*
 * main.c
 *
 *  Created on: Nov 7, 2013
 *      Author: Sleepy
 */
#include "bsp.h"
//==============================================================================
// Constant Definition
#define IPADDR0		10
#define IPADDR1		30
#define IPADDR2		81
#define IPADDR3		19

#define NETMASK0	255
#define NETMASK1	255
#define NETMASK2	255
#define NETMASK3	0

#define GATEWAY0	10
#define GATEWAY1	30
#define GATEWAY2	81
#define GATEWAY3	1

#define CTRL_PORT	2587
#define DATA_PORT	2588


//==============================================================================
// Global variable definitions
unsigned long long tick;

//test variables
unsigned int data[2000];
unsigned long long led_tick;
unsigned long long hex_tick;
unsigned long long eth_tick;
unsigned int hex_led_cntr;

//==============================================================================
// Function declarations

interrupt void ctrl_isr(void);
void startup_loop(void);
void main_loop(void) __attribute__((noreturn));

//==============================================================================
// Function definitions

void main(void)
{
	int i;

	HW_init();						/* Initialize hardware*/
	HW_set_ctrl_isr(ctrl_isr);      /* Set control ISR */
	HW_dac_update_all(0);

	// System tick initialization
	tick = 0;
	led_tick = 0;
	hex_tick = 0;
	eth_tick = 0;
	hex_led_cntr = 0;
	EINT;
	for (i = 0; i < 2000; i++)
		data[i] = i;
	startup_loop();
	main_loop();					/* Start the super loop */

}

void startup_loop(void)
{
	int step;
	unsigned long long startup_tick;

	step = 0;

	while (step < 14) {
		switch (step) {
		case 0:
			HW_eth_cmd(ETH_CMD_RESET, 0);
			startup_tick = tick + 5 * CTRL_FREQ;
			step++;
			break;

		case 2:
			HW_eth_cmd(ETH_CMD_SET_MASK, MAKE_IP(NETMASK0, NETMASK1, NETMASK2, NETMASK3));
			step++;
			break;
		case 4:
			HW_eth_cmd(ETH_CMD_SET_GW, MAKE_IP(GATEWAY0, GATEWAY1, GATEWAY2, GATEWAY3));
			step++;
			break;

		case 6:
			HW_eth_cmd(ETH_CMD_SET_IP, MAKE_IP(IPADDR0, IPADDR1, IPADDR2, IPADDR3));
			step++;
			break;

		case 8:
			HW_eth_cmd(ETH_CMD_SET_CTRL_PORT, CTRL_PORT);
			step++;
			break;

		case 10:
			HW_eth_cmd(ETH_CMD_SET_DATA_PORT, DATA_PORT);
			step++;
			break;

		case 12:
			HW_eth_cmd(ETH_CMD_START_SERVER, 0);
			step++;
			break;

		case 1:
			if (tick > startup_tick) {
				step--;
				break;
			}
		case 3:
		case 5:
		case 7:
		case 9:
		case 11:
		case 13:
			if (HW_eth_get_current_cmd() == ETH_CMD_NONE)
					step++;
			break;
		}
		HW_eth_task();
	}
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

	// Debug task, send data to host
    if( tick >= eth_tick) {
		while (eth_tick <= tick)
			eth_tick += (1 * CTRL_FREQ);
		HW_eth_send(data, 50);
	}

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
			hex_tick += (0.5 * CTRL_FREQ);
    	HW_cpld_reg_write_poll(0, hex_led_cntr++);
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
