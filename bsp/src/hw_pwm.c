/*
 * hw_pwm.c
 *
 *  Created on: Sep 16, 2013
 *      Author: Z Shen
 */

/*
 * PWM channel assignment:
 *   ePWM2: PFC high frequency phase leg
 *   ePWM3: PFC low frequency phase leg
 *   ePWM1: DAB primary left phase leg
 *   ePWM4: DAB primary right phase leg
 *   ePWM5: DAB secondary left phase leg
 *   ePWM6: DAB secondary right phase leg
 */
#define __HW_PWM_C__

#include "device.h"
#include "hw_pwm.h"

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
void pwm_init_reg_buf();
interrupt void pwm_default_tz_isr(void);
extern interrupt void pwm_reg_update_isr(void);

/*******************************************************************************
 * Public function definitions
 ******************************************************************************/
void HW_pwm_init(void)
{
    EALLOW;
    pwm_init_clk();
    pwm_init_regs();
    pwm_init_io();
    EDIS;
    pwm_init_reg_buf();
}
void HW_pwm_enable_pfc(void)
{
    EALLOW;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.INT = 1;
    EPwm2Regs.TZEINT.bit.OST = 1;
    EDIS;
}

void HW_pwm_enable_dab(void)
{
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm5Regs.TZCLR.bit.OST = 1;
    EPwm8Regs.TZCLR.bit.OST = 1;
    EDIS;
    EPwm1Regs.AQCSFRC.all = 0;
}

void HW_pwm_disable_pfc(void)
{
    EALLOW;
    EPwm2Regs.TZEINT.bit.OST = 0;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EDIS;
}

void HW_pwm_disable_dab(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm5Regs.TZFRC.bit.OST = 1;
    EPwm8Regs.TZFRC.bit.OST = 1;
    EDIS;
    EPwm1Regs.AQCSFRC.all = 0x2;
}

void HW_pwm_force_protection(void)
{
    /* Use IO to force set CPLD protection */
	GpioDataRegs.GPASET.bit.GPIO13 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;

}

void HW_pwm_reset_protection(void)
{
    /* Use IO to clear CPLD protection */
	GpioDataRegs.GPASET.bit.GPIO14 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO14 = 1;
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
	SysCtrlRegs.PCLKCR1.all |= 0x97;			// ePWM1 ~ ePWM3, ePWM5 ePWM8


}

void pwm_init_io()
{
    /* Congigure ePWM2AB , ePWM3AB for PFC1 */
	/* Set pins as PWM output */
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;			// ePWM1A
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;			// ePWM2A
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;			// ePWM2B
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;			// ePWM3A
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;			// ePWM3B
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;			// ePWM5A
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;			// ePWM5B
	GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 3;		// ePWM8A
	GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 3;		// ePWM8B

	/* Disable internal pull-up for the selected output pins to
     * reduce power consumption */
	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;			// ePWM1A
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;			// ePWM2A
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;			// ePWM2B
	GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;			// ePWM3A
	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;			// ePWM3B
	GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;			// ePWM5A
	GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;			// ePWM5B
	GpioCtrlRegs.GPBPUD.bit.GPIO60 = 1;			// ePWM8A
	GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1;			// ePWM8B

	/* Set GPIO12 as trip zone 1 input */
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;		// Set as TZ1n
    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;		// Set as async

    /* Set GPIO for CPLD protection */
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;		// As IO
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;			// As output
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;		// As IO
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;			// As output
}

void pwm_init_reg_buf()
{
    int i, j;

    for (i = 0; i<2; i++) {
        for (j = 0; j < SW_PER_SAMPLE; j++) {
            pwm_reg_bufs[i][j][AQ_PFC1_LF] = 0x0001;
            pwm_reg_bufs[i][j][AQ_PFC1_HF] = 0x00D1;
            pwm_reg_bufs[i][j][CMP_PFC1_HF] = 0;
            pwm_reg_bufs[i][j][PHS_DAB_MAIN] = 0.5 * SW_PRD_HALF - PHASE_LOAD_DELAY;
        }
    }

    pwm_reg_buf_ptr_isr = (Uint32)(&pwm_reg_bufs[0][0][0]);
    pwm_reg_buf_ptr_isr_start = pwm_reg_buf_ptr_isr;
    pwm_reg_buf_ptr_isr_end = pwm_reg_buf_ptr_isr_start + ((SW_PER_SAMPLE - 1) * PWM_REG_BUF_LEN);
    pwm_reg_buf_ptr_ctrl = (Uint32)(&pwm_reg_bufs[1][0][0]);

}

void pwm_init_regs()
{
    /* Stop all the TB clocks */
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;

    /* Clock base setup*/

    /* ePWM1 setup, used as clock base, no PWM output */
    /* Setup TBCLK */
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;		/* PWMCLK = SYSCLKOUT */
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    /* Disable register shadows */
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
    EPwm1Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;

    /* Setup PWM period */
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /* Count up-down */
    EPwm1Regs.TBPRD = SW_PRD_HALF;                  /* Set timer period */
    EPwm1Regs.TBCTR = 0x0000;                       /* Clear counter */

    /* Disable Dead time*/
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

    /* Initialize internal ePWMA signal to high */
    EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_SET;
    EPwm1Regs.AQSFRC.bit.OTSFA = 1;

    /* Setup action qualifier */
    EPwm1Regs.AQCTLA.bit.PRD = AQ_CLEAR;			// Set when CTR=PRD
    EPwm1Regs.AQSFRC.bit.RLDCSF = CC_CTR_PRD;		// Set software force load on zero
    EPwm1Regs.AQCSFRC.bit.CSFA= AQ_SET;				// Keep forcing ePWMA signal high using continuous software forcing

    /* Setup Sync */
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         /* Disable sync input for master */
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     /* Generate SYNC for other channel signal when CTR=0 */

    /* Setup trip zone */
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm1Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm1Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */

#if SW_PER_SAMPLE == 1
    EPwm1Regs.ETSEL.all |= 0x8800;
#endif

    /* PFC1 PWMs */
    /* ePWM2 setup */

    /* Setup TBCLK */
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;		/* PWMCLK = SYSCLKOUT */
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    /* Set counter mode */
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /* Count up-down */

    /* Set timer period */
    EPwm2Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
    EPwm2Regs.TBPRD = SW_PRD_HALF;

    /* Setup Sync */
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;      /* Bypass sync input signal to output */
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;         /* Enable phase loading */

    /* Initialize comparator */
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPA.half.CMPA = SW_PRD_HALF+1;     /* Set to 0% duty cycle for DAB */

    /* Initialize counter */
    EPwm2Regs.TBCTR = 0x0000;                       /* Clear counter */

    /* Setup Dead time - Active Low PWMs */
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED = DEAD_TIME_PFC_LF;
    EPwm2Regs.DBFED = DEAD_TIME_PFC_LF;

    /* Initialize internal ePWMA signal to low */
    EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
    EPwm2Regs.AQSFRC.bit.OTSFA = 1;

    /* Set time amount LF switch is toggled before HF switch */
    EPwm2Regs.CMPB = 110;

    /* Setup trip zone */
    EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_HI;          /* Force PWMB output to high when triggered */
    EPwm2Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm2Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */

    /* ePWM3 setup */

    /* Setup TBCLK */
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;		/* PWMCLK = SYSCLKOUT */
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    /* Set counter mode */
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /* Count up-down */


    /* Set timer period */
    EPwm3Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
    EPwm3Regs.TBPRD = SW_PRD_HALF;

    /* Setup Sync */
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_DISABLE;		/* Disable sync output signal, no more block in the chain */
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;         /* Disable phase loading */

    /* Initialize comparator */
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPA.half.CMPA = SW_PRD_HALF + 1;     /* Set to 0% duty cycle for DAB */

    /* Initialize counter */
    EPwm3Regs.TBCTR = 0x0000;                       /* Clear counter */

    /* Setup Dead time - Active Low PWMs */
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED = DEAD_TIME_PFC_HF;
    EPwm3Regs.DBFED = DEAD_TIME_PFC_HF;

    /* Initialize internal ePWMA signal to low */
    EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
    EPwm3Regs.AQSFRC.bit.OTSFA = 1;

    /* Setup trip zone */
    EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_HI;          /* Force PWMB output to high when triggered */
    EPwm3Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm3Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */

    /* DAB1 PWMs */
    /* ePWM5 setup, DAB1 Primary */

    /* Setup TBCLK */
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;		/* PWMCLK = SYSCLKOUT */
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    /* Set counter mode */
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;		// Count up


    /* Set timer period */
    EPwm5Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
    EPwm5Regs.TBPRD = SW_PRD -1;

    /* Setup Sync */
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;		// Bypass sync input signal to output
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;         	// Enable phase loading

    /* Initialize comparator */
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPA.half.CMPA = SW_PRD_HALF;    	/* Set to 50% duty cycle for DAB */

    /* Initialize counter */
    EPwm5Regs.TBCTR = 0x0000;                       /* Clear counter */

    /* Setup action qualifier */
    EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;				/* Set when CTR=CMPA*/
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_CLEAR;			/* Clear when CTR=ZERO */

    /* Setup Dead time - Active Low PWMs */
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm5Regs.DBRED = DEAD_TIME_DAB_PRI;
    EPwm5Regs.DBFED = DEAD_TIME_DAB_PRI;

    /* Initialize internal ePWMA signal to low */
    EPwm5Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
    EPwm5Regs.AQSFRC.bit.OTSFA = 1;

    /* Setup trip zone */
    EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_HI;          /* Force PWMB output to high when triggered */
    EPwm5Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm5Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */

    /* ePWM8 setup, DAB1 secondary*/

    /* Setup TBCLK */
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;		/* PWMCLK = SYSCLKOUT */
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    /* Set counter mode */
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /* Count up-down */


    /* Set timer period */
    EPwm8Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
    EPwm8Regs.TBPRD = SW_PRD_HALF;

    /* Setup Sync */
    EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;		/* Bypass sync input signal to output */
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;         /* Disable phase loading, sync to PFC */

    /* Initialize comparator */
    EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm8Regs.CMPA.half.CMPA = SW_PRD_HALF/2;     /* Set to 50% duty cycle for DAB */

    /* Initialize counter */
    EPwm8Regs.TBCTR = 0x0000;                       /* Clear counter */

    /* Setup action qualifier */
    EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;				/* Set when CTR=CMPA up counting */
    EPwm8Regs.AQCTLA.bit.CAD = AQ_CLEAR;			/* Clear when CTR=CMPA up counting */

    /* Setup Dead time - Active Low PWMs */
    EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm8Regs.DBRED = DEAD_TIME_DAB_PRI;
    EPwm8Regs.DBFED = DEAD_TIME_DAB_PRI;

    /* Initialize internal ePWMA signal to low */
    EPwm8Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
    EPwm8Regs.AQSFRC.bit.OTSFA = 1;

    /* Setup trip zone */
    EPwm8Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm8Regs.TZCTL.bit.TZB = TZ_FORCE_HI;          /* Force PWMB output to high when triggered */
    EPwm8Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm8Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */



    /* Setup PWM register update ISR
     * All registers are updated using ePWM1 CMPB event
     * The event is generated just before the counter go to zero
     */
    PieVectTable.EPWM1_INT = pwm_reg_update_isr;
    EPwm1Regs.CMPA.half.CMPA = REG_LOAD_CYC;
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTRD_CMPA;      /* Select INT on CMPA event */
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;             /* Generate INT on 1st event */
    EPwm1Regs.ETSEL.bit.INTEN = 1;                  /* Enable INT */
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;              /* Enable interrupt in PIE */
    IER |= M_INT3;

    /* Setup trip zone ISR
     * The trip zone interrupt from ePWM1 is used
     */
    /* Setup interrupt vectors */
    PieVectTable.EPWM2_TZINT = pwm_default_tz_isr;

    /* Enable TZ interrupt for protection, only one shot from TZ1 triggers interrupt */
    /* Interrupt generation will be enabled when PWM output is enables */
    PieCtrlRegs.PIEIER2.bit.INTx2 = 1;
    IER |= M_INT2;

    /* Start all the timers synchronized */
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
}

interrupt void pwm_default_tz_isr(void)
{
	/* Halt CPU here for debug, no error reset is provided */
    ESTOP0;
//    /* Never return from ISR */
//    for (;;);
    // Return from interrupt to keep running
}
