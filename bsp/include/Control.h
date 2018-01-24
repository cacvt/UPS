// Filename:     Control.h
// Author:       Jianghui Yu
// Description:
//   Control collector file for header files

#include <Func_adc.h>
#include <Func_CPLD.h>
#include <Func_relay.h>
#include "math.h"
#include "pid_reg3.h"
#include "pr.h"
#include "constdef.h"
#include "device.h"
#include "HW_pll.h"

#ifndef __CONTROL_H__
#define __CONTROL_H__

typedef struct
{
	Uint32 Relay_State;       //2 relay state: 8-off,1-on. 5 relays
	Uint16 Ch_State[3];       //2 charging state: 8-not charged, 1-charged. [0] for top, [1] for bottom, [2] for overall
	Uint16 Dis_State[3];      //2 discharging state: 8-not-discharged, 1-discharged. [0] for top, [1] for bottom, [2] for overall
	Uint16 Operation_State;   //5 operation modes: 0-off, 1-precharge, 2-rectifier, 3-discharge, 4-fault
	Uint16 Ch_Dis_State;      //3 charging or discharging state: 0-top, 1-bottom, 2-off
} ctrl_state_data;

typedef struct	//calculated data: 3 average voltage, 3 circulating current, 3 ac current, 2 alpha-beta current, 2 dq current
{
	float VavgA;		//average voltage of phase A
	float VavgB;		//average voltage of phase B
	float VavgC;		//average voltage of phase C
	float Vavg;         //overall average voltage
	float IcirA;		//circulating current of phase A
	float IcirB;		//circulating current of phase B
	float IcirC;		//circulating current of phase C
	float IA;           //AC current of phase A
	float IB;           //AC current of phase B
	float IC;           //AC current of phase C
	float Ialpha;       //alpha axis current
	float Ibeta;        //beta axis current
	float Id;           //d axis current
	float Iq;           //q axis current
	float Idavg;           //d axis current
	float Iqavg;           //q axis current
	float Idtt;           //d axis current
	float Iqtt;           //q axis curren
	float Van;       //an axis voltage
	float Vbn;        //bn axis voltage
	float Vcn;        //cn axis voltage
	float Valpha;       //alpha axis voltage
	float Vbeta;        //beta axis voltage
	float Vd;           //d axis voltage
	float Vdavg;           //d axis voltage
	float Vq;           //q axis voltage
	float Vqavg;           //d axis voltage
	float Vdtt;           //d axis current
	float Vqtt;           //q axis curren
} CALCULATE_DATA;

#endif
