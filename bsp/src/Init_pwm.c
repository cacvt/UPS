/*
 * hw_pwm.c
 *
 *  Created on: Sep 16, 2013
 *      Author: Z Shen
 */

/* Modified by Jianghui Yu
 * Description:
 *    For each channel, only PWM signals for top switches are generaed in DSP
 *    PWM signals for bottom switches are generated in CPLD
 *    EPWM1 is for phase A upper arm, EPWM4 is for phase A lower arm
 *    EPWM2 is for phase B upper arm, EPWM5 is for phase B lower arm
 *    EPWM3 is for phase C upper arm, EPWM6 is for phase C lower arm
 */
#define __HW_PWM_C__

#include "device.h"
#include <Init_pwm.h>

/*******************************************************************************
 * Constants used within current file
 ******************************************************************************/
#define REG_LOAD_CYC        195      /* Cycles for PWM ISR to response and update regs    */
                                    /* ~16 to response, ~40 to update, plus 24 to be     */
                                    /* prepared for interrupt is disabled for short time */

/*******************************************************************************
 * Global variables used by any file
 ******************************************************************************/
Uint32  pwm_reg_buf_ptr_ctrl;

/*******************************************************************************
 * Global variables used within current file
 ******************************************************************************/
Uint32 pwm_reg_buf_ptr_isr;
Uint32 pwm_reg_buf_ptr_isr_start;
Uint32 pwm_reg_buf_ptr_isr_end;
#pragma DATA_ALIGN(pwm_reg_bufs, 128)
Uint16 pwm_reg_bufs[2][SW_PER_SAMPLE][PWM_REG_BUF_LEN];

/*******************************************************************************
 * Private function declaration
 ******************************************************************************/
void pwm_init_io();
void pwm_init_regs();
void pwm_init_clk();
interrupt void pwm_default_tz_isr(void);
extern interrupt void pwm_reg_update_isr(void);

void PWM_Regs_Conf(volatile struct EPWM_REGS *pPwmReg, Uint16 nPer, Uint32 nShift, unsigned char ADC_EN);//yrf 2014/11/1

/*******************************************************************************
 * Public function definitions
 ******************************************************************************/
void HW_pwm_init(void)
{
    //EALLOW;
    pwm_init_clk();
    pwm_init_io();
    pwm_init_regs();
    //EDIS;
}
void HW_pwm_enable_pfc(void)
{
    EALLOW;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.INT = 1;
    EPwm2Regs.TZEINT.bit.OST = 1;
    EDIS;
}

void HW_pwm_enable_dab(void)
{
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm8Regs.TZCLR.bit.OST = 1;
    EPwm4Regs.TZCLR.bit.OST = 1;
    EPwm5Regs.TZCLR.bit.OST = 1;
    EDIS;
    EPwm1Regs.AQCSFRC.all = 0;
}

void HW_pwm_disable_pfc(void)
{
    EALLOW;
    EPwm2Regs.TZEINT.bit.OST = 0;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm8Regs.TZFRC.bit.OST = 1;
    EDIS;
}

void HW_pwm_disable_dab(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm5Regs.TZFRC.bit.OST = 1;
    EPwm4Regs.TZFRC.bit.OST = 1;
    EDIS;
    EPwm1Regs.AQCSFRC.all = 0x2;
}

void HW_pwm_force_protection(void)
{
    /* Use IO to force set CPLD protection */
	GpioDataRegs.GPASET.bit.GPIO14 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO14 = 1;

}

void HW_pwm_reset_protection(void)
{
    /* Use IO to clear CPLD protection */
	GpioDataRegs.GPASET.bit.GPIO15 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;
}

void HW_pwm_set_tzisr(TZ_ISR_FUNC func)
{
    EALLOW;
    PieVectTable.EPWM2_TZINT = func;
    EDIS;
}

/********************************************************************************
 * Private function definitions
 */
void pwm_init_clk()
{
    /* Enable module clocks */
//	SysCtrlRegs.PCLKCR1.all |= 0x97;			// ePWM1 ~ ePWM3, ePWM5 ePWM8
	//SysCtrlRegs.PCLKCR1.all |= 0xFF;            // ePWM1~8		//yrf-2014/11/1
	//SysCtrlRegs.PCLKCR2.all |= 0x01;            // ePWM9		//yrf-2014/11/1

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // Disable TBCLK within the ePWM
    SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // ePWM1
    SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // ePWM2
    SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;  // ePWM3
    SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 1;  // ePWM4
    SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1;  // ePWM5
    SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1;  // ePWM6
    SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 1;  // ePWM7
    SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 1;  // ePWM8
    SysCtrlRegs.PCLKCR2.bit.EPWM9ENCLK = 1;  // ePWM9
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Enable TBCLK within the ePWM
    EDIS;

}

void pwm_init_io()
{
	/* Set pins as PWM output */
    EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;			// ePWM1A
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;			// ePWM1B //yrf//2014/11/1
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;			// ePWM2A
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;			// ePWM2B
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;			// ePWM3A
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;			// ePWM3B
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;			// ePWM4A
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;			// ePWM4B
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;			// ePWM5A
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;			// ePWM5B
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;		// ePWM6A //yrf//2014/11/1
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;		// ePWM6B //yrf//2014/11/1
	GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;		// ePWM7A //yrf//2014/11/1
	GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;		// ePWM7B //yrf//2014/11/1
////
////	GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3;		// ePWM8A
////	GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3;		// ePWM8B

	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0;		// ePWM9A	//yrf//2014/11/1  -used for I/o
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0;		// ePWM9B	//yrf//2014/11/1  -used for I/o


	/* Disable internal pull-up for the selected output pins to
     * reduce power consumption */
	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;			// ePWM1A
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;          // ePWM1A
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;			// ePWM2A
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;			// ePWM2B
	GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;			// ePWM3A
	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;			// ePWM3B
	GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;			// ePWM4A
	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;			// ePWM4B
	GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;			// ePWM5A
	GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;			// ePWM5B
	GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;         // ePWM6B
	GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;         // ePWM6B
	GpioCtrlRegs.GPBPUD.bit.GPIO58 = 1;         // ePWM7A
	GpioCtrlRegs.GPBPUD.bit.GPIO59 = 1;         // ePWM7A
////	GpioCtrlRegs.GPBPUD.bit.GPIO60 = 1;			// ePWM8A
////	GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1;			// ePWM8B

	GpioCtrlRegs.GPBDIR.bit.GPIO62 = 1;         // As output
	GpioCtrlRegs.GPBDIR.bit.GPIO63 = 1;         // As output
	//GpioCtrlRegs.GPBPUD.bit.GPIO62 = 1;         // ePWM9A
	//GpioCtrlRegs.GPBPUD.bit.GPIO63 = 1;         // ePWM9A

	/* Set GPIO12 as trip zone 1 input */
    //GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;		// Set as TZ1n
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;		// Set as async
    EDIS;



}


void PWM_Regs_Conf(volatile struct EPWM_REGS *pPwmReg, Uint16 nPer, Uint32 nShift, unsigned char ADC_EN)
{
    pPwmReg->TBPRD = nPer;                   // Set timer period
    pPwmReg->TBPHS.half.TBPHS = nShift;      // Set phase shift
    pPwmReg->TBCTL.bit.PHSDIR = TB_UP;       // phase shift count up
    pPwmReg->TBCTR = 0x0000;                 // Clear counter

    // Setup counter mode
    pPwmReg->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count up down
    pPwmReg->TBCTL.bit.PHSEN = TB_ENABLE;          // ENABLE phase loading
    pPwmReg->TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    pPwmReg->TBCTL.bit.CLKDIV = TB_DIV1;
    pPwmReg->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

    // Setup shadowing
    pPwmReg->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    pPwmReg->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    pPwmReg->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
    pPwmReg->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    pPwmReg->CMPA.half.CMPA = 0;     // Set compare A value
    pPwmReg->CMPB = 0;               // Set Compare B value

    // Set actions
    /* Setup action qualifier */
    pPwmReg->AQCTLA.bit.CAD = AQ_SET;           // Set high when counter decreases
    pPwmReg->AQCTLA.bit.CAU = AQ_CLEAR;         // Set zero when counter increases

    pPwmReg->AQCTLB.bit.CBD = AQ_SET;           // Set high when counter decreases
    pPwmReg->AQCTLB.bit.CBU = AQ_CLEAR;         // Set zero when counter increases

    // Setup Deadband
    pPwmReg->DBCTL.bit.OUT_MODE = 0x0;         // Disabled

    pPwmReg->ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    pPwmReg->ETSEL.bit.SOCBSEL = ET_CTR_ZERO;//ET_CTR_PRD

    pPwmReg->ETPS.bit.SOCAPRD = ET_1ST;
    pPwmReg->ETPS.bit.SOCBPRD = ET_1ST;


    // Interrupt where we will change the Compare Values
    //pPwmReg->ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    //pPwmReg->ETSEL.bit.INTEN = 1;                // Enable INT
    //pPwmReg->ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    if(ADC_EN == 1)
            pPwmReg->ETSEL.all |= 0x8800;


}


void pwm_init_regs()
{
    /* Stop all the TB clocks */
    //SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    Uint32 nPer = SW_PRD_HALF;  //up and down
    Uint32 nShift[2];

    nShift[0]=0;
    nShift[1]=nPer/2;		        //phase shift control: 90 degree shift

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    PWM_Regs_Conf(&EPwm1Regs, nPer, nShift[0], 0);
    PWM_Regs_Conf(&EPwm2Regs, nPer, nShift[0], 0);
    PWM_Regs_Conf(&EPwm3Regs, nPer, nShift[0], 0);
    PWM_Regs_Conf(&EPwm4Regs, nPer, nShift[0], 0);
    PWM_Regs_Conf(&EPwm5Regs, nPer, nShift[0], 0);
    PWM_Regs_Conf(&EPwm6Regs, nPer, nShift[0], 0);
    PWM_Regs_Conf(&EPwm7Regs, nPer, nShift[0], 0);


    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; //master


    PWM_Regs_Conf(&EPwm9Regs, nPer*SW_PER_SAMPLE, 0, 1);
    EPwm9Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Enable INT on Zero event
    EPwm9Regs.ETSEL.bit.INTEN = 1;   // Enable INT
    EPwm9Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st event

    EPwm9Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // ENABLE phase loading
    EPwm9Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    //PWM_Regs_Conf(&EPwm9Regs, nPer, nShift[8]);


    /* Setup PWM register update ISR
     * All registers are updated using ePWM1 CMPB event
     * The event is generated just before the counter go to zero
     */
    //PieVectTable.EPWM1_INT = pwm_reg_update_isr;
    //EPwm1Regs.CMPA.half.CMPA = REG_LOAD_CYC;

    //EPwm1Regs.ETSEL.bit.INTSEL = ET_CTRD_CMPA;      /* Select INT on CMPA event */
    //EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;             /* Generate INT on 1st event */
    //EPwm1Regs.ETSEL.bit.INTEN = 1;                  /* Enable INT */
    //PieCtrlRegs.PIEIER3.bit.INTx1 = 1;              /* Enable interrupt in PIE */
   // IER |= M_INT3;

    /* Setup trip zone ISR
     * The trip zone interrupt from ePWM1 is used
     */
    /* Setup interrupt vectors */
    //PieVectTable.EPWM2_TZINT = pwm_default_tz_isr;

    /* Enable TZ interrupt for protection, only one shot from TZ1 triggers interrupt */
    /* Interrupt generation will be enabled when PWM output is enables */
    //PieCtrlRegs.PIEIER2.bit.INTx2 = 1;
    //IER |= M_INT2;

    /* Start all the timers synchronized */
    //SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
}

interrupt void pwm_default_tz_isr(void)
{
	/* Halt CPU here for debug, no error reset is provided */
    ESTOP0;
//    /* Never return from ISR */
//    for (;;);
    // Return from interrupt to keep running
}
