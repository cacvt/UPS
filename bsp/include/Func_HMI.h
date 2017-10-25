/*
 * Func_HMI.h
 *
 *  Created on: July 22, 2014
 *      Author: jiaoyang
 *      Modified by: Jianghui
 *      Description:
 *           Ethernet communication header file
 */

#ifndef _HMI_FUNC_H_
#define _HMI_FUNC_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Type definition
 ******************************************************************************/

struct HMI_data {
	Uint16 Operation_State;
	Uint16 Relay_State;
	Uint32 Pre;
	Uint16 Ch_State;
	Uint32 Rec;
	Uint32 Fault;
	Uint32 Dis;
	Uint16 Dis_State;
	Uint16 flag;
	float f;
	float Vdc;
	float Vavg;
	float Vd;
	float Vq;
	float Id;
	float Iq;
	float VAu;
	float VAl;
	float VBu;
	float VBl;
	float VCu;
	float VCl;
};

#ifdef  _HMI_FUNC_C_
struct HMI_data HMI;
#else
extern struct HMI_data HMI;
#endif

//************************
//declaration of control functions
void Init_para_table(void);
void update_var(void);
//************************


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */


#endif


