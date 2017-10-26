//==============================================================================
// Copyright �2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     bsp.h
// Author:       Z Shen
//
// Modified by Jianghui Yu
// Description:
//   Initialization collector file for functions
//   Collector file for board supporting package
//==============================================================================
#include <Initialization.h>

//==============================================================================
// Public variable definitions
unsigned long long tick;
unsigned long long tick_eth;

//==============================================================================
// Public functions definitions

// Call initilization functions accoding whether header file is included in
// bsp.h
void HW_init(void)
{
	HW_sys_init();
	HW_pwm_init();

#ifdef __PROFILE_H__
	profile_init();
#endif // __PROFILE_H__

#ifdef __HW_ADC_H__
	HW_adc_init();
#endif // __HW_ADC_H__

//#ifdef __HW_DAC_H__
//	HW_dac_init();
//#endif // __HW_DAC_H__

#ifdef __HW_CC_LED_H__
	HW_cc_led_init();
#endif // __HW_CC_LED_H__

#ifdef __HW_CPLD_H__
	HW_cpld_init();
#endif // __HW_CPLD_H__

#ifdef __PARAM_TABLE_H__
	//param_table_init();
//#else  //__PARAM_TABLE_H__
//
//#ifdef __HW_ETHERNET_H__
//	HW_eth_init();                               //HW_eth_init() is called in param_table_init()
//#endif // __HW_ETHERNET_H__
#endif // __PARAM_TABLE_H__

	//Init_para_table();

	tick = 0;
	tick_eth = 0;

   	DELAY_US(10000);
}

#ifdef __HW_ADC_H__
void HW_set_ctrl_isr(CTRL_ISR_FUNC func)
{
    HW_adc_set_isr(func);
}
#endif // __HW_ADC_H__
