// Filename:     Func_cpld.h
// Author:       Jianghui Yu
// Description:
//   CPLD functions header file
//
//==============================================================================
// Public function declaration
#define CPLD_FAULT_SET		GpioDataRegs.GPASET.bit.GPIO14 = 1;     //fault set for CPLD
#define CPLD_NO_FAULT_SET	GpioDataRegs.GPACLEAR.bit.GPIO14 = 1;   //No fault set for CPLD
#define CPLD_FAULT_CLR		{GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;asm(" nop");asm(" nop");asm(" nop");GpioDataRegs.GPASET.bit.GPIO15 = 1;}
                                                                    //rising edge clear fault for CPLD
