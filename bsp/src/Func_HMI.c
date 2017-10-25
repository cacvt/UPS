/*
 * Func_HMI.c
 *
 *  Created on: July 22, 2014
 *      Author: jiaoyang
 *      Modified by: Jianghui
 *      Description:
 *           Ethernet communication function file
 */

#define _HMI_FUNC_C_
#include "device.h"
#include "Init_param_table.h"
#include "Func_HMI.h"

void Init_para_table(void)
{
	UINT_PARAM(0)=0;
	UINT_PARAM(1)=8888;
	ULONG_PARAM(2)=0;
	UINT_PARAM(3)=8;
	ULONG_PARAM(4)=0;
	ULONG_PARAM(5)=0;
	UINT_PARAM(6)=8;

	UINT_PARAM(7)=8;
	FLOAT_PARAM(8)=0;
	FLOAT_PARAM(9)=0;
	FLOAT_PARAM(10)=0;
	FLOAT_PARAM(11)=0;
	FLOAT_PARAM(12)=0;
	FLOAT_PARAM(13)=0;

	FLOAT_PARAM(14)=0;
	FLOAT_PARAM(15)=0;
	FLOAT_PARAM(16)=0;
	FLOAT_PARAM(17)=0;
	FLOAT_PARAM(18)=0;
	FLOAT_PARAM(19)=0;
}


void update_var(void)
{
	UINT_PARAM(0) = HMI.Operation_State;        //Read, operation state
	UINT_PARAM(1) = HMI.Relay_State;            //Read, relay state
	HMI.Pre  = ULONG_PARAM(2);                  //Write, pre-charge command
	UINT_PARAM(3)  = HMI.Ch_State;              //Read, top/bottom PEBBs charged
	HMI.Rec  = ULONG_PARAM(4);                  //Write, start rectifier operation command
	HMI.Fault= ULONG_PARAM(20);                 //Write, start fault operation command
	HMI.Dis  = ULONG_PARAM(5);                  //Write, stop operation, start dis-chargepre-charge command
	UINT_PARAM(6)  = HMI.Dis_State;             //Read, top/bottom PEBBs dis-charged

	UINT_PARAM(7)  = HMI.flag;                  //Read, PLL is synchronized
	FLOAT_PARAM(8) = HMI.f;                     //Read, PLL frequency
	FLOAT_PARAM(9) = HMI.Vdc;                   //Read, average sensed dc voltage
	FLOAT_PARAM(21) = HMI.Vavg;                 //Read, average sensed average voltage
	FLOAT_PARAM(10) = HMI.Vd;                   //Read, average sensed ac voltage in dq
	FLOAT_PARAM(11) = HMI.Vq;
	FLOAT_PARAM(12) = HMI.Id;                   //Read, average sensed ac current in dq
	FLOAT_PARAM(13) = HMI.Iq;

	FLOAT_PARAM(14) = HMI.VAu;                  //Read, average sensed PEBB voltage
	FLOAT_PARAM(15) = HMI.VAl;
	FLOAT_PARAM(16) = HMI.VBu;
	FLOAT_PARAM(17) = HMI.VBl;
	FLOAT_PARAM(18) = HMI.VCu;
	FLOAT_PARAM(19) = HMI.VCl;
} // end of update_var
