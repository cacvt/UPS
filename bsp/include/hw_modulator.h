/*
 * hw_modulator.h
 *
 *  Created on: Oct 29, 2013
 *      Author: shenzy
 */

#ifndef __HW_MODULATOR_H__
#define __HW_MODULATOR_H__

/*******************************************************************************
 * Configuration
 ******************************************************************************/
#define PFC_MAX_DUTY 			1.0
/*******************************************************************************
 * Public function declaration
 ******************************************************************************/
void HW_modulator_pfc(float duty);
void HW_modulator_dab(float phase);

#endif /* __HW_MODULATOR_H__ */
