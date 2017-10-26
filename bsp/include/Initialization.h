//==============================================================================
// Copyright © 2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     bsp.h
// Author:       Z Shen
//
// Modified by Jianghui Yu
// Description:
//   Initialization collector file for head files
//   Main header file for board supporting package
//==============================================================================

#ifndef __BSP_H__
#define __BSP_H__

#include "device.h"
#include "Init_adc.h"
#include "Init_cpld.h"
#include "Init_dac.h"
#include "Init_ethernet.h"
#include "Init_LED.h"
#include "Init_param_table.h"
#include "Init_profile.h"
#include "Init_pwm.h"
#include "Init_sys.h"
#include "Func_HMI.h"

//==============================================================================
// Public variable declarations
extern unsigned long long tick;
extern unsigned long long tick_eth;

//==============================================================================
// Public functions declarations

// Wrapper functions of lower level functions
void HW_init(void);

#ifdef __HW_ADC_H__

void HW_set_ctrl_isr(CTRL_ISR_FUNC func);

#endif /* __HW_ADC_H__ */

#endif /* __BSP_H__ */
