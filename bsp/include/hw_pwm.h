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
#define PHS_DELAY       2
#define CMP_PFC_HF      0
#define CMP_PFC_LF      1
#define PHS_DAB_MAIN    2
#define PWM_REG_BUF_LEN 3


/*******************************************************************************
 * Constant derived from configuration constants
 ******************************************************************************/
#define SW_FREQ     (CTRL_FREQ * SW_PER_SAMPLE)
#define SW_PRD      (CPU_FREQ / SW_FREQ)
#define SW_PRD_HALF (SW_PRD / 2)

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

/*******************************************************************************
 * Public global variables
 ******************************************************************************/
#ifndef __HW_PWM_C__

extern Uint32  pwm_reg_buf_ptr_ctrl;

#endif /* __HW_PWM_C__ */

#endif /* __HW_PWM_H__ */
