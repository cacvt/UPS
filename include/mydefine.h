/*
 * mydefine.h
 *
 *  Created on: 2014Äê10ÔÂ22ÈÕ
 *      Author: R.F. Yang
 */
#include "DSP2834x_Device.h"

#ifndef MYDEFINE_H_
#define MYDEFINE_H_

////state machine - predefine
//#define STS_INIT 	(0)
//#define STS_CHARGE 	(1)
//#define STS_READY 	(2)
//#define STS_RUNNING (3)

////state machine -charge
//#define STS_CH_INIT 	(0)
//#define STS_CH_TOGGLE1 	(1)
//#define STS_CH_TOGGLE2 	(2)
//#define STS_CH_FINISH 	(3)

//#define CELL_NUM (8)




//typedef union
//{
//	struct
//	{
//		unsigned char ERR_OCIu:	1;	//over current-up
//		unsigned char ERR_OCId:	1;	//over current-down
//		unsigned char ERR_SMDCOV:	1;	//over voltage
//		unsigned char ERR_DRV:	1;	//drive error
//		unsigned char reserved:	12;	//over current
//	}ERRBITs;
//	int16 ALL;
//}CTRLSYSERR;		//operation system err


#endif /* MYDEFINE_H_ */
