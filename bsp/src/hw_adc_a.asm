;
;              File: hw_adc_a.asm
;            Author: Z Shen
;        Created on: 9/24/2013
;  Last modified on: 9/24/2013
;
	.cdecls   C, LIST, "device.h", "hw_adc.h"

	.def _adc_soc_isr

	.text
    .global  _adc_soc_isr
_adc_soc_isr:
	.if         SW_PER_SAMPLE > 1
	MOVW        DP, #_EPwm1Regs.ETSEL.all
	AND         @_EPwm1Regs.ETSEL.all, #0x77FF
	.endif
	MOVW        DP, #_PieCtrlRegs.PIEACK.all
	MOV        	@_PieCtrlRegs.PIEACK.all, #PIEACK_GROUP7
	IRET

