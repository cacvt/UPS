/*
 * hw_sync.h
 *
 *  Created on: Dec 1, 2014
 *      Author: Chi Li
 *      Modified by: Jianghui
 *      Description:
 *           PLL header file
 */

#ifndef __HW_SYNC_H__
#define __HW_SYNC_H__

#include "device.h"
#include "constdef.h"

struct  SYNC {
   float   		theta;
   float   		omega;
};

/*******************************************************************************
 * Public function declarations
 ******************************************************************************/
void HW_sync_init(struct SYNC *sync);
void HW_pll(struct SYNC *sync, float vq);



#endif /* __HW_SYNC_H__ */
