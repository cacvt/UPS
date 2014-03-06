/*******************************************************************************
 *         Filename: HW_pwm.h
 *           Author: Z Shen
 *       Created on: 9/16/2013
 * Last modified on:
 ******************************************************************************/


#ifndef __HW_PWM_H__
#define __HW_PWM_H__

/*******************************************************************************
 * Public constant definition
 ******************************************************************************/
#define AQ_PFC1_LF			0
#define AQ_PFC1_HF			1
#define CMP_PFC1_HF			2
#define PHS_DAB_MAIN    	3
#define PWM_REG_BUF_LEN 	4

#define PHASE_LOAD_DELAY	2


/*******************************************************************************
 * Constant derived from configuration constants
 ******************************************************************************/
#define SW_FREQ     (CTRL_FREQ * SW_PER_SAMPLE)
#define SW_PRD      (CPU_FREQ / SW_FREQ)
#define SW_PRD_HALF (SW_PRD / 2)
/*******************************************************************************
 * Data type definition
 ******************************************************************************/
typedef interrupt void(*TZ_ISR_FUNC)(void);

/*******************************************************************************
 * Public function declarations
 ******************************************************************************/
void HW_pwm_init(void);
void HW_pwm_enable_pfc(void);
void HW_pwm_enable_dab(void);
void HW_pwm_disable_pfc(void);
void HW_pwm_disable_dab(void);
void HW_pwm_force_protection(void);
void HW_pwm_reset_protection(void);
void HW_pwm_set_tzisr(TZ_ISR_FUNC func);

/*******************************************************************************
 * Public global variables
 ******************************************************************************/
#ifndef __HW_PWM_C__

extern Uint32  pwm_reg_buf_ptr_ctrl;
#define PWM_REGS(r, n)	*((Uint16*)(pwm_reg_buf_ptr_ctrl + n * PWM_REG_BUF_LEN + r))

#endif /* __HW_PWM_C__ */

#endif /* __HW_PWM_H__ */
