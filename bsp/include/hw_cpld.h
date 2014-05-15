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

//==============================================================================
// Type definition

//==============================================================================
// Public function declaration
void HW_cpld_init(void);
void HW_cpld_reset(void);
void HW_cpld_reg_write_poll(unsigned int addr, unsigned int data);
unsigned int HW_cpld_reg_read_poll(unsigned int addr);

#endif//__HW_CPLD_H__
