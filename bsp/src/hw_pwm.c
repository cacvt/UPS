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
#define REG_LOAD_CYC        80      /* Cycles for PWM ISR to response and update regs    */
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
Uint16 pwm_reg_bufs[2][SW_PER_SAMPLE][PWM_REG_BUF_LEN];

/*******************************************************************************
 * Private function declaration
 ******************************************************************************/
void pwm_init_io();
void pwm_init_regs();
void pwm_init_clk();
void pwm_init_reg_buf();
interrupt void pwm_tz_isr_dab(void);
interrupt void pwm_tz_isr_pfc(void);
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
    EPwm2Regs.TZFRC.bit.CBC = 1;
    EPwm3Regs.TZFRC.bit.CBC = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.INT = 1;
    EPwm2Regs.TZEINT.bit.OST = 1;
    EDIS;
}

void HW_pwm_enable_dab(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.CBC = 1;
    EPwm4Regs.TZFRC.bit.CBC = 1;
    EPwm5Regs.TZFRC.bit.CBC = 1;
    EPwm6Regs.TZFRC.bit.CBC = 1;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm4Regs.TZCLR.bit.OST = 1;
    EPwm5Regs.TZCLR.bit.OST = 1;
    EPwm6Regs.TZCLR.bit.OST = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EPwm1Regs.TZEINT.bit.OST = 1;
    EDIS;
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
    EPwm1Regs.TZEINT.bit.OST = 0;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm4Regs.TZFRC.bit.OST = 1;
    EPwm5Regs.TZFRC.bit.OST = 1;
    EPwm6Regs.TZFRC.bit.OST = 1;
    EDIS;
}

void HW_pwm_force_protection(void)
{
    /* Use IO to force set CPLD protection */

}

void HW_pwm_reset_protection(void)
{
    /* Use IO to clear CPLD protection */

}
/********************************************************************************
 * Private function definitions
 */
void pwm_init_clk()
{
    /* Enable module clocks of ePWM1 ~ ePWM6 */
    SysCtrlRegs.PCLKCR1.all |= 0x3F;

}

void pwm_init_io()
{
    /* Disable internal pull-up for the selected output pins to
     * reduce power consumption */
    GpioCtrlRegs.GPAPUD.all |= 0x0FFF;

    /* Set GPIO0~11 as ePWM output pins (ePWM1 ~ ePWM6) */
    GpioCtrlRegs.GPAMUX1.all &= 0xFF000000;
    GpioCtrlRegs.GPAMUX1.all |= 0xFF555555;

    /* Set GPIO for CPLD protection */
}


void pwm_init_reg_buf()
{
    int i, j;

    for (i = 0; i<2; i++) {
        for (j = 0; j < SW_PER_SAMPLE; j++) {
            pwm_reg_bufs[i][j][CMP_PFC_HF] = 0;
            pwm_reg_bufs[i][j][CMP_PFC_LF] = SW_PRD_HALF;
            pwm_reg_bufs[i][j][PHS_DAB_MAIN] = PHS_DELAY;
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

    /* DAB PWMs */
    /* ePWM1 setup */
    /* Setup TBCLK */
    /* PWMCLK = SYSCLKOUT */
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    /* Enable register shadows */
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;

    /* Setup PWM period */
    EPwm1Regs.TBPRD = SW_PRD_HALF;                  /* Set timer period */
    EPwm1Regs.TBCTR = 0x0000;                       /* Clear counter */
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /* Count up-down */

    /* Setup Sync */
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         /* Disable sync input for master */
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     /* Generate SYNC for other channel signal when CTR=0 */

    /* Initialize comparator */
    EPwm1Regs.CMPA.half.CMPA = SW_PRD_HALF / 2;     /* Set to 50% duty cycle for DAB */



    /* Set EPWM1a actions, this signal in internal and active high */
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;                  /* Set when PRD=CMPA, CU */
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;              /* Clear when PRD=CMPA, CD */


    /* Setup Dead time - Active Low PWMs */
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED = DEAD_TIME_DAB_PRI;
    EPwm1Regs.DBFED = DEAD_TIME_DAB_PRI;

    /* Setup trip zone */
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_HI;          /* Force PWMB output to high when triggered */
    EPwm1Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm1Regs.TZSEL.bit.CBC2 = 1;                   /* TZ2 is set as cycle by cycle for synchronized PWM enable */
    EPwm1Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */

    /* ePWM4 setup */

    /* Setup TBCLK */
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       /* PWMCLK = SYSCLKOUT */
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm4Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;       /* Set registers to immediate access first */
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;

    EPwm4Regs.TBPRD = SW_PRD_HALF;                  /* Set timer period */
    EPwm4Regs.TBCTR = 0x0000;                       /* Clear counter */
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /* Count up-down */
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;         /* Enable phase loading */

    /* Setup Sync */
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;      /* Bypass sync input signal to output */
    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;          /* Enable phase loading */
    EPwm4Regs.TBPHS.half.TBPHS = PHS_DELAY;         /* Compensation internal delay to get zero phase shift */

    /* Initialize comparator */
    EPwm4Regs.CMPA.half.CMPA = SW_PRD_HALF / 2;     /* Set to 50% duty cycle for DAB */


    /* Enable register shadows */
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;

    /* Set EPWMa actions, this signal in internal and active high
     * Action is reversed from ePWM1, produces 180 degree phase
     *  shift by default */
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;            /* Clear when PRD=CMPA, CU */
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;              /* Set when PRD=CMPA, CD */


    /* Setup Dead time - Active Low PWMs */
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm4Regs.DBRED = DEAD_TIME_DAB_PRI;
    EPwm4Regs.DBFED = DEAD_TIME_DAB_PRI;

    /* Setup trip zone */
    EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_HI;          /* Force PWMB output to high when triggered */
    EPwm4Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm4Regs.TZSEL.bit.CBC2 = 1;                   /* TZ2 is set as cycle by cycle for synchronized PWM enable */
    EPwm4Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */

    /* ePWM5 setup */

    /* Setup TBCLK */
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       /* PWMCLK = SYSCLKOUT */
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm5Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;       /* Set registers to immediate access first */
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;

    EPwm5Regs.TBPRD = SW_PRD_HALF;                  /* Set timer period */
    EPwm5Regs.TBCTR = 0x0000;                       /* Clear counter */
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /* Count up-down */
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;         /* Enable phase loading */

    /* Setup Sync */
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     /* Generate sync output for the next channels when CTR=0 */
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;          /* Enable phase loading */
    EPwm5Regs.TBPHS.half.TBPHS = PHS_DELAY;         /* Compensation internal delay to get zero phase shift */

    /* Initialize comparator */
    EPwm5Regs.CMPA.half.CMPA = SW_PRD_HALF / 2;     /* Set to 50% duty cycle for DAB */

    /* Enable register shadows */
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;

    /* Set EPWMa actions, this signal in internal and active high */
    EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;              /* Set when PRD=CMPA, CU */
    EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;            /* Clear when PRD=CMPA, CD */


    /* Setup Dead time - Active Low PWMs */
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm5Regs.DBRED = DEAD_TIME_DAB_SEC;
    EPwm5Regs.DBFED = DEAD_TIME_DAB_SEC;

    /* Setup trip zone */
    EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_HI;          /* Force PWMB output to high when triggered */
    EPwm5Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm5Regs.TZSEL.bit.CBC2 = 1;                   /* TZ2 is set as cycle by cycle for synchronized PWM enable */
    EPwm5Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */

    /* ePWM6 setup */

    /* Setup TBCLK */
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       /* PWMCLK = SYSCLKOUT */
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm6Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;       /* Set registers to immediate access first */
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;

    EPwm6Regs.TBPRD = SW_PRD_HALF;                  /* Set timer period */
    EPwm6Regs.TBCTR = 0x0000;                       /* Clear counter */
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /* Count up-down */

    /* Setup Sync */
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_DISABLE;      /* No more ePWM in the chain */
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;          /* Enable phase loading */
    EPwm6Regs.TBPHS.half.TBPHS = PHS_DELAY;         /* Compensation internal delay to get zero phase shift */

    /* Initialize comparator */
    EPwm6Regs.CMPA.half.CMPA = SW_PRD_HALF / 2;     /* Set to 50% duty cycle for DAB */

    /* Enable register shadows */
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;

    /* Set EPWMa actions, this signal in internal and active high
     * Action is reversed from ePWM5, produces 180 degree phase
     *  shift by default */
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;            /* Set when PRD=CMPA, CU */
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;              /* Clear when PRD=CMPA, CD */


    /* Setup Dead time - Active Low PWMs */
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm6Regs.DBRED = DEAD_TIME_DAB_SEC;
    EPwm6Regs.DBFED = DEAD_TIME_DAB_SEC;

    /* Setup trip zone */
    EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_HI;          /* Force PWMB output to high when triggered */
    EPwm6Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm6Regs.TZSEL.bit.CBC2 = 1;                   /* TZ2 is set as cycle by cycle for synchronized PWM enable */
    EPwm6Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */

    /* PFC PWMs */
    /* ePWM2 setup */

    /* Setup TBCLK */
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        /* PWMCLK = SYSCLKOUT */
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;       /* Set registers to immediate access first */
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;

    EPwm2Regs.TBPRD = SW_PRD_HALF;                  /* Set timer period */
    EPwm2Regs.TBCTR = 0x0000;                       /* Clear counter */
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /* Count up-down */

    /* Setup Sync */
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;         /* Disable sync for PFC PWMs */
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_DISABLE;     /* Generate SYNC for other channel signal when CTR=0 */

    /* Initialize comparator */
    EPwm2Regs.CMPA.half.CMPA = 0;                   /* Set to 100% duty cycle for PFC HF leg */

    /* Enable register shadows */
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;

    /* Set EPWMa actions, this signal in internal and active high */
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;              /* Clear when PRD=CMPA, CU */
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;            /* Set when PRD=CMPA, CD */


    /* Setup Dead time - Active Low PWMs */
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED = DEAD_TIME_PFC_HF;
    EPwm2Regs.DBFED = DEAD_TIME_PFC_HF;

    /* Setup trip zone */
    EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_HI;          /* Force PWMB output to high when triggered */
    EPwm2Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm2Regs.TZSEL.bit.CBC2 = 1;                   /* TZ2 is set as cycle by cycle for synchronized PWM enable */
    EPwm2Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */

    /* ePWM3 setup */

    /* Setup TBCLK */
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        /* PWMCLK = SYSCLKOUT */
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;       /* Set registers to immediate access first */
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;

    EPwm3Regs.TBPRD = SW_PRD_HALF;                  /* Set timer period */
    EPwm3Regs.TBCTR = 0x0000;                       /* Clear counter */
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /* Count up-down */

    /* Setup Sync */
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;         /* Disable sync for PFC PWMs */
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_DISABLE;     /* Generate SYNC for other channel signal when CTR=0 */

    /* Initialize comparator */
    EPwm3Regs.CMPA.half.CMPA = SW_PRD_HALF;         /* Set to 0% duty cycle for PFC LF leg */

    /* Enable register shadows */
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;

    /* Set EPWMa actions, this signal in internal and active high */
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;              /* Set when PRD=CMPA, CU */
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;            /* Clear when PRD=CMPA, CD */


    /* Setup Dead time - Active Low PWMs */
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED = DEAD_TIME_PFC_LF;
    EPwm3Regs.DBFED = DEAD_TIME_PFC_LF;

    /* Setup trip zone */
    EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_HI;          /* Force PWMA output to high when triggered */
    EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_HI;          /* Force PWMB output to high when triggered */
    EPwm3Regs.TZSEL.bit.OSHT1 = 1;                  /* TZ1 is set as one shot for protection */
    EPwm3Regs.TZSEL.bit.CBC2 = 1;                   /* TZ2 is set as cycle by cycle for synchronized PWM enable */
    EPwm3Regs.TZFRC.bit.OST = 1;                    /* Force output to off state */

    /* Setup PWM register update ISR
     * All registers are updated using ePWM1 CMPB event
     * The event is generated just before the counter go to zero
     */
    PieVectTable.EPWM1_INT = pwm_reg_update_isr;
    EPwm1Regs.CMPB = REG_LOAD_CYC;
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTRD_CMPB;      /* Select INT on CMPB event */
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;             /* Generate INT on 1st event */
    EPwm1Regs.ETSEL.bit.INTEN = 1;                  /* Enable INT */
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;              /* Enable interrupt in PIE */
    IER |= M_INT3;

    /* Setup trip zone ISR
     * The trip zone interrupt from ePWM1 is used
     */

    /* Setup interrupt vectors */
    PieVectTable.EPWM1_TZINT = pwm_tz_isr_dab;
    PieVectTable.EPWM2_TZINT = pwm_tz_isr_pfc;

    /* Enable TZ interrupt for protection, only one shot from TZ1 triggers interrupt */
    /* Interrupt generation will be enabled when PWM output is enables */
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;              /* Enable interrupt in PIE */
    PieCtrlRegs.PIEIER2.bit.INTx2 = 1;
    IER |= M_INT2;

    /* Start all the timers synchronized */
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
}

interrupt void pwm_tz_isr_pfc(void)
{
    /* Halt CPU here for debug, no error reset is provided */
    ESTOP0;
    for (;;);

    /* Turn off all the PWM immediately during normal operation */
    HW_pwm_force_protection();
    /* Set converter state to error */

    /* Return from ISR */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}

interrupt void pwm_tz_isr_dab(void)
{
    /* Halt CPU here for debug, no error reset is provided */
    ESTOP0;
    for (;;);

    /* Turn off all the PWM immediately during normal operation */
    HW_pwm_force_protection();
    /* Set converter state to error */

    /* Return from ISR */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}
