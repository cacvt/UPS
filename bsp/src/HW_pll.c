/*
 * hw_sync.c
 *
 *  Created on: Dec 1, 2014
 *      Author: Chi Li
 *      Modified by: Jianghui
 *      Description:
 *           PLL function file
 */

#define  __HW_SYNC__

#include "HW_pll.h"

/*******************************************************************************
 * Private variables definitions
 ******************************************************************************/
// Control parameters in s domain
const float Kp = 0.05/Vc*300, Ki = 10/Vc*300;

// States
float Sum_vq = 0;
int	cntsync = 0;

/*******************************************************************************
 * Public function definitions
 ******************************************************************************/

void HW_sync_init(struct SYNC *syn) {
	syn->theta = 0;
	syn->omega = 2*PI*60*1.01;       //set the initial value to be outside the range to make sure it is synchronized
//  Initialize integrators
	Sum_vq = syn->omega/Ki/CTRL_CLK;
	// Constants in difference equations
}

void HW_pll(struct SYNC *syn, float vq) {
	Sum_vq += vq;
	syn->omega = Kp*vq + Ki*CTRL_CLK*Sum_vq;
	syn->theta += syn->omega*CTRL_CLK;
}
