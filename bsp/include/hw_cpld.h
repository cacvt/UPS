//==============================================================================
// Copyright © 2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     hw_cpld.h
// Author:       Z Shen
// Description:
//   Access registers in CPLD for the functions implemented in CPLD
//
//==============================================================================
#ifndef __HW_CPLD_H__
#define __HW_CPLD_H__
//==============================================================================
// Public constant definition
#define	REG_HEX_LED			0x0000
#define REG_CURRENT_ERROR0	0x0010
#define REG_CURRENT_ERROR1	0x0011
#define REG_LATCHED_ERROR0	0x0012
#define REG_LATCHED_ERROR1	0x0013
#define REG_CPLD_INPUT0		0x0020
#define REG_CPLD_INPUT1		0x0021
#define REG_CPLD_OUTPUT0	0x0022
#define REG_CPLD_OUTPUT1	0x0023

//==============================================================================
// Type definition

//==============================================================================
// Public function declaration
void HW_cpld_init(void);
void HW_cpld_reset(void);
void HW_cpld_reg_write_poll(unsigned int addr, unsigned int data);
unsigned int HW_cpld_reg_read_poll(unsigned int addr);

#endif//__HW_CPLD_H__
