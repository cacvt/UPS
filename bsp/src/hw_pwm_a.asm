;
; hw_pwm_reg_update_isr.asm
;
;  Created on: Sep 19, 2013
;      Author: shenzy

	.cdecls   C, LIST, "..\include\device.h", "..\include\hw_pwm.h"

	.def _pwm_reg_update_isr

	.ref _pwm_reg_buf_ptr_isr
	.ref _pwm_reg_buf_ptr_isr_start
	.ref _pwm_reg_buf_ptr_isr_end

	.text
    .global  __pwm_reg_update_isr
_pwm_reg_update_isr:
	ASP
	MOVL         *SP++, XAR7
	MOVW		 DP, #_EPwm1Regs.ETSEL.all
	AND          @_EPwm1Regs.ETSEL.all, #0x77FF
	MOVW         DP, #_pwm_reg_buf_ptr_isr
	MOVL		 ACC, @_pwm_reg_buf_ptr_isr
	CMPL         ACC, @_pwm_reg_buf_ptr_isr_end
	SB           HW_PWM_REG_UPDATE_NORMAL, LT
	MOVW         DP, #_EPwm1Regs.ETSEL.all
	OR           @_EPwm1Regs.ETSEL.all, #0x8800
	MOVW         DP, #_pwm_reg_buf_ptr_isr
	MOVL         XAR7, @_pwm_reg_buf_ptr_isr_start
	MOVL		 ACC, @_pwm_reg_buf_ptr_ctrl
	MOVL         @_pwm_reg_buf_ptr_isr_start, ACC
	MOVL		 @_pwm_reg_buf_ptr_isr, ACC
	MOVL         @_pwm_reg_buf_ptr_ctrl, XAR7
	ADDB		 ACC, #(SW_PER_SAMPLE * PWM_REG_BUF_LEN)
	MOVL         @_pwm_reg_buf_ptr_isr_end, ACC
	MOVL         XAR7, @_pwm_reg_buf_ptr_isr
	MOVW         DP, #_EPwm2Regs.AQCTLA
	MOV          AL, *+XAR7[0]
	MOV          @_EPwm2Regs.AQCTLA, AL
	MOVW         DP, #_pwm_reg_buf_ptr_isr
HW_PWM_REG_UPDATE_NORMAL:
	MOVL         XAR7, @_pwm_reg_buf_ptr_isr
	MOVB         ACC, #PWM_REG_BUF_LEN
	ADDL         @_pwm_reg_buf_ptr_isr, ACC
	MOVW         DP, #_EPwm3Regs.CMPA.half.CMPA
	MOV          AH, *+XAR7[1]
	MOV          AL, *+XAR7[2]
	MOV          @_EPwm3Regs.AQCTLA, AH
	MOV          @_EPwm3Regs.CMPA.half.CMPA, AL
	MOV          AL, *+XAR7[3]
	MOVW         DP, #_EPwm5Regs.TBPHS.half.TBPHS
	MOV          @_EPwm5Regs.TBPHS.half.TBPHS, AL
	MOVW         DP, #_EPwm1Regs.ETCLR.all
	MOVB         @_EPwm1Regs.ETCLR.all, #0x01, UNC
	MOVW         DP, #_PieCtrlRegs.PIEACK.all
	MOVB         @_PieCtrlRegs.PIEACK.all, #PIEACK_GROUP3, UNC
	MOVL         XAR7, *--SP
	NASP
	IRET

