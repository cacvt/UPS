//==============================================================================
// Copyright © 2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     param_table.h
// Author:       Z Shen
// Description:
//   A parameter tables that can be writed and read via Ethernet
//==============================================================================
#ifndef __PARAM_TABLE_H__
#define __PARAM_TABLE_H__
//==============================================================================
// Constant definition
#define PARAM_TABLE_LEN		32

//==============================================================================
// Macro definition
#define FLOAT_PARAM(id)		(*(float *)(&param_table[id]))
#define ULONG_PARAM(id)		(*(unsigned long *)(&param_table[id]))
#define LONG_PARAM(id)		(*(long *)(&param_table[id]))
#define UINT_PARAMT(id)		(*(unsigned int *)(&param_table[id]))
#define INT_PARAM(id)		(*(int *)(&param_table[id]))

//==============================================================================
// Public variable declaration
extern unsigned long param_table[];

//==============================================================================
// Public function declaration
void param_table_init(void);

#endif /* __PARAM_TABLE_H__ */
