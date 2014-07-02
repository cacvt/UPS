/*
 * adc.c
 *
 *  Created on: Sep 12, 2013
 *      Author: shenzy
 */
#define __HW_ADC_C__
#include "device.h"
#include "hw_adc_constants.h"
#include "hw_adc.h"

/*******************************************************************************
 * Private constants definition
 ******************************************************************************/
#define ADC1_REG        ((volatile Uint16 *)0x4000)
#define ADC2_REG        ((volatile Uint16 *)0x200000)

const float ADC_FS[2][6] = ADC_FS_VAL;
const float ADC_OS[2][6] = ADC_OS_VAL;

/*******************************************************************************
 * Private function declaration
 ******************************************************************************/
interrupt void adc_default_isr(void);
void adc_init_dma(void);
void adc_init_io(void);
void adc_init_isr();
void adc_init_reg(void);
void adc_init_xintf(void);
void adc_soc_isr(void);
/*******************************************************************************
 * Public variable definitions
 ******************************************************************************/
#pragma SET_DATA_SECTION("adcdata")     /* Put adc data into dedicated RAM for DMA efficiency */
int16 adc_res[2][6];
float adc_val[2][6];
#pragma SET_DATA_SECTION(".ebss")       /* Restore data section assignment */

/*******************************************************************************
 * Public function definitions
 ******************************************************************************/
void HW_adc_init(void)
{
	adc_init_io();
	adc_init_xintf();
	adc_init_reg();
	adc_init_dma();
	adc_init_isr();
}

void HW_adc_scale(void)
{
    adc_val[0][0] = (float)(adc_res[0][0]<<4) * ADC_FS[0][0] - ADC_OS[0][0];
    adc_val[0][1] = (float)(adc_res[0][1]<<4) * ADC_FS[0][1] - ADC_OS[0][1];
    adc_val[0][2] = (float)(adc_res[0][2]<<4) * ADC_FS[0][2] - ADC_OS[0][2];
    adc_val[0][3] = (float)(adc_res[0][3]<<4) * ADC_FS[0][3] - ADC_OS[0][3];
    adc_val[0][4] = (float)(adc_res[0][4]<<4) * ADC_FS[0][4] - ADC_OS[0][4];
    adc_val[0][5] = (float)(adc_res[0][5]<<4) * ADC_FS[0][5] - ADC_OS[0][5];
    adc_val[1][0] = (float)(adc_res[1][0]<<4) * ADC_FS[1][0] - ADC_OS[1][0];
    adc_val[1][1] = (float)(adc_res[1][1]<<4) * ADC_FS[1][1] - ADC_OS[1][1];
    adc_val[1][2] = (float)(adc_res[1][2]<<4) * ADC_FS[1][2] - ADC_OS[1][2];
    adc_val[1][3] = (float)(adc_res[1][3]<<4) * ADC_FS[1][3] - ADC_OS[1][3];
    adc_val[1][4] = (float)(adc_res[1][4]<<4) * ADC_FS[1][4] - ADC_OS[1][4];
    adc_val[1][5] = (float)(adc_res[1][5]<<4) * ADC_FS[1][5] - ADC_OS[1][5];

}

void HW_adc_set_isr(CTRL_ISR_FUNC func)
{
    EALLOW;
    PieVectTable.DINTCH2 = func;
    EDIS;
}
/*******************************************************************************
 * Private function definitions
 ******************************************************************************/
interrupt void adc_default_isr(void)
{
    /* Disable SOC generation from ePWM1 */
    EALLOW;
    EPwm1Regs.ETSEL.all &= (~0x8800);
    EDIS;

    /* User codes */

    /* Re-arm DMA */

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}

void adc_init_dma(void)
{
    EALLOW;

    SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 1;                   /* Enable clock of DMA module */

    GpioIntRegs.GPIOXINT3SEL.all = 2;                       /* Use GPIO34 as XINT3 to trigger DMA */
    XIntruptRegs.XINT3CR.bit.POLARITY = 0;                  /* Interrupt on falling edge */
    XIntruptRegs.XINT3CR.bit.ENABLE = 1;                    /* Interrupt is enabled */

    DmaRegs.PRIORITYCTRL1.bit.CH1PRIORITY = 0;              /* CH1 high priority mode */

    DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32)ADC1_REG;     /* Point to beginning of source buffer */
    DmaRegs.CH1.SRC_ADDR_SHADOW =     (Uint32)ADC1_REG;     /* Point to beginning of source buffer */
    DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32)(adc_res[0]+2);     /* Point to beginning of destination buffer */
    DmaRegs.CH1.DST_ADDR_SHADOW =     (Uint32)(adc_res[0]+2);
    DmaRegs.CH2.SRC_BEG_ADDR_SHADOW = (Uint32)ADC2_REG;     /* Point to beginning of source buffer */
    DmaRegs.CH2.SRC_ADDR_SHADOW =     (Uint32)ADC2_REG;     /* Point to beginning of source buffer */
    DmaRegs.CH2.DST_BEG_ADDR_SHADOW = (Uint32)(adc_res[1]+2);     /* Point to beginning of destination buffer */
    DmaRegs.CH2.DST_ADDR_SHADOW =     (Uint32)(adc_res[1]+2);
    DmaRegs.CH1.BURST_SIZE.all = 1;                         /* n+1 words per burst */
    DmaRegs.CH1.SRC_BURST_STEP = 0;                         /* ADC register is at fixed address */
    DmaRegs.CH1.DST_BURST_STEP = 3;                         /* Increment dest addr between each word x-ferred */
    DmaRegs.CH1.TRANSFER_SIZE = 2;                          /* Number of bursts per transfer, DMA interrupt will occur after completed transfer */
    DmaRegs.CH2.BURST_SIZE.all = 1;                         /* n+1 words per burst */
    DmaRegs.CH2.SRC_BURST_STEP = 0;                         /* ADC register is at fixed address */
    DmaRegs.CH2.DST_BURST_STEP = 3;                         /* Increment dest addr between each word x-ferred */
    DmaRegs.CH2.TRANSFER_SIZE = 2;                          /* Number of bursts per transfer, DMA interrupt will occur after completed transfer */
    DmaRegs.CH1.SRC_TRANSFER_STEP = 0x0000;                 /* ADC register is at fixed address */
    DmaRegs.CH1.DST_TRANSFER_STEP = 0xFFFC;                 /* Move to next address after burst */
    DmaRegs.CH2.SRC_TRANSFER_STEP = 0x0000;                 /* ADC register is at fixed address */
    DmaRegs.CH2.DST_TRANSFER_STEP = 0xFFFC;                 /* Move to next address after burst */
    DmaRegs.CH1.SRC_WRAP_SIZE = 0xFFFF;                     /* Larger than TRANSFER_STEP to disable WRAP */
    DmaRegs.CH1.SRC_WRAP_STEP = 0x0000;                     /* Do not care */
    DmaRegs.CH1.DST_WRAP_SIZE = 0xFFFF;                     /* Larger than TRANSFER_STEP to disable WRAP */
    DmaRegs.CH1.DST_WRAP_STEP = 0x0000;                     /* Do not care */
    DmaRegs.CH2.SRC_WRAP_SIZE = 0xFFFF;                     /* Larger than TRANSFER_STEP to disable WRAP */
    DmaRegs.CH2.SRC_WRAP_STEP = 0x0000;                     /* Do not care */
    DmaRegs.CH2.DST_WRAP_SIZE = 0xFFFF;                     /* Larger than TRANSFER_STEP to disable WRAP */
    DmaRegs.CH2.DST_WRAP_STEP = 0x0000;                     /* Do not care */
    DmaRegs.CH1.MODE.bit.PERINTSEL = DMA_XINT3;             /* Use XINT3 to trigger DMA transfer */
    DmaRegs.CH2.MODE.bit.PERINTSEL = DMA_XINT3;             /* Use XINT3 to trigger DMA transfer */
    DmaRegs.CH1.MODE.bit.PERINTE = PERINT_ENABLE;           /* Peripheral interrupt triggering enable */
    DmaRegs.CH2.MODE.bit.PERINTE = PERINT_ENABLE;           /* Peripheral interrupt triggering enable */
    DmaRegs.CH1.MODE.bit.ONESHOT = ONESHOT_ENABLE;          /* Oneshot mode disabled */
    DmaRegs.CH2.MODE.bit.ONESHOT = ONESHOT_ENABLE;          /* Oneshot mode disabled */
    DmaRegs.CH1.MODE.bit.CONTINUOUS = CONT_ENABLE;          /* Continous mode enabled */
    DmaRegs.CH2.MODE.bit.CONTINUOUS = CONT_ENABLE;          /* Continous mode enabled */
    DmaRegs.CH1.MODE.bit.DATASIZE = SIXTEEN_BIT;            /* date size is set to 16-bits */
    DmaRegs.CH2.MODE.bit.DATASIZE = SIXTEEN_BIT;            /* date size is set to 16-bits */

    DmaRegs.CH1.CONTROL.bit.RUN = 1;                        /* Start DMA channel 1 */
    DmaRegs.CH2.CONTROL.bit.RUN = 1;                        /* Start DMA channel 2 */
    EDIS;
}

void adc_init_io(void)
{
    EALLOW;

    /* Configure XINTF pins
     * Enable used pins only! Keep others as input!
     *  D0-D11, CS0, CS7, WE0 RD */
    GpioCtrlRegs.GPBMUX1.all = 0x00003FC0;
    GpioCtrlRegs.GPCMUX1.all = 0xAAAAAA00;

    /* Disable pull-ups on used XINTF control pins */
    GpioCtrlRegs.GPBPUD.all |= 0x00000078;

    /* Configure gpios for manual conversion start
     * GPIO56: ADC1; GPIO57: ADC2 */
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;    // 0=GPIO,
    GpioCtrlRegs.GPBDIR.bit.GPIO56 = 1;     // 1=OUTput,  0=INput
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;    // 0=GPIO,
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;     // 1=OUTput,  0=INput

    /* Set both IO to low to enable ePWM triggering */
    GpioDataRegs.GPBCLEAR.bit.GPIO56 = 1;       // Set high for manual CONVST triggering
    GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;       // Set low to allow ePWM triggered CONVST

    /* Setup ExtSOC signal generation */
    /* Event generation is controlled by pwm update isr & adc isr */

    /* SOC is generated when TBCTR = 0 */
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    EPwm1Regs.ETSEL.bit.SOCBSEL = ET_CTR_ZERO;

    /* SOC is generated on first event */
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;
    EPwm1Regs.ETPS.bit.SOCBPRD = ET_1ST;

    // Set ExtSOC  polarity
    SysCtrlRegs.EXTSOCCFG.bit.EXTSOC1APOLSEL= 0x1;  // Set inverted polarity (CONVST is active low)
    SysCtrlRegs.EXTSOCCFG.bit.EXTSOC1BPOLSEL= 0x1;  // Set inverted polarity (CONVST is active low)

    /* Enable ExtSOC output 1A & 1B */
    SysCtrlRegs.EXTSOCCFG.bit.EXTSOC1AEN = 0x1;     // Enable extsoc1a
    SysCtrlRegs.EXTSOCCFG.bit.EXTSOC1BEN = 0x1;     // Enable extsoc1a

    /* Set HSPCLK to 50M to generate SOC width of 640ns
     * SOC width is 32 cyc of HSPCLK */
    SysCtrlRegs.HISPCP.bit.HSPCLK = 3;//0x7;

    /* Set GPIO34 as input to access BUSY signal */
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;    // 0=GPIO,
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0;     // 1=OUTput,  0=INput

    EDIS;
}

void adc_init_isr()
{
    EALLOW;

    /* Setup DMA CH2 interrupt for control
     * ISR function is given later by HW_adc_set_isr function call */
    DmaRegs.CH2.MODE.bit.CHINTMODE = CHINT_END;     /* Interrupt at the end of transfer */
    DmaRegs.CH2.MODE.bit.CHINTE = CHINT_ENABLE;     /* Channel Interrupt to CPU enable */
    DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1;          /* Clear any spurious interrupt flags */
    PieCtrlRegs.PIEIER7.bit.INTx2 = 1;              /* Enable interrupt DINTCH2 in PIE */

    /* Enable global interrupt level for DMA interrupt */
    IER |= M_INT7;

    EDIS;
}

void adc_init_reg(void)
{
    /* Reset devices */
    *ADC1_REG = 0x105;
    *ADC2_REG = 0x105;

    /* Setup Vref */
    /* Setup next access to Vfef value write */
    *ADC1_REG = 0x101;
    *ADC2_REG = 0x101;

    /* Set Vref = 3.0V/2 */
    *ADC1_REG = 0x266;
    *ADC2_REG = 0x266;

    /* Setup sequencer */
    /* Setup next access to sequencer write*/
    *ADC1_REG = 0x104;
    *ADC2_REG = 0x104;

    /* Configure sequencer */
    // Setup SEQUENCER to convert pseudo-diff mode in the following sequence:
    // Convert CH(A/B)0+, on single CONVERT and BUSY for entire sequence (though only one in sequence anyway)
    *ADC1_REG = 0xF24;
    *ADC2_REG = 0xF24;
}

void adc_init_xintf(void)
{
    EALLOW;

    SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;           /* Enable clock of XINTF module */

    /* Fast waitstates and the write buffer is set to buffer 3 writes. */
    XintfRegs.XINTCNF2.bit.WRBUFF = 3;

    /* XCLKOUT to SYSCLKOUT ratio
     * By default XCLKOUT = 1/8 SYSCLKOUT (200MHz) = (25MHz) */
    /* XTIMCLK = SYSCLKOUT/2 = (100 MHz) */
    XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
    /* XCLKOUT = XTIMCLK/2 = (50 MHz) */
    XintfRegs.XINTCNF2.bit.CLKMODE = 1;
    /* XCLKOUT = XTIMCLK/4 = (25 MHz) */
    XintfRegs.XINTCNF2.bit.BY4CLKMODE = 1;
    /* Enable XCLKOUT */
    XintfRegs.XINTCNF2.bit.CLKOFF = 0;

    /* Set up Zone 0 for ADC1 */

    /* Do not double all Zone read/write lead/active/trail timing */
    XintfRegs.XTIMING0.bit.X2TIMING = 0;

    /* Zone read timing */
    XintfRegs.XTIMING0.bit.XRDLEAD = 2;     // minimum required by DSP, 20ns
    XintfRegs.XTIMING0.bit.XRDACTIVE = 6;   // minimum required by DSP, 60ns
    XintfRegs.XTIMING0.bit.XRDTRAIL = 0;    //1

    /* Zone write timing */
    XintfRegs.XTIMING0.bit.XWRLEAD = 3;  //3
    XintfRegs.XTIMING0.bit.XWRACTIVE = 1;  //1
    XintfRegs.XTIMING0.bit.XWRTRAIL = 3; //3

    /* Zone will not sample READY */
    XintfRegs.XTIMING0.bit.USEREADY = 0;
    XintfRegs.XTIMING0.bit.READYMODE = 0;

    /* Set XINTF width to 16 bits */
    XintfRegs.XTIMING0.bit.XSIZE = 3;

    /* Set up Zone 7 for ADC2 */

    /* Do not double all Zone read/write lead/active/trail timing */
    XintfRegs.XTIMING7.bit.X2TIMING = 0;

    /* Zone read timing */
    XintfRegs.XTIMING7.bit.XRDLEAD = 2;
    XintfRegs.XTIMING7.bit.XRDACTIVE = 6;
    XintfRegs.XTIMING7.bit.XRDTRAIL = 0;

    /* Zone write timing */
    XintfRegs.XTIMING7.bit.XWRLEAD = 3;
    XintfRegs.XTIMING7.bit.XWRACTIVE = 1;
    XintfRegs.XTIMING7.bit.XWRTRAIL = 3;


    /* Zone will not sample READY */
    XintfRegs.XTIMING7.bit.USEREADY = 0;
    XintfRegs.XTIMING7.bit.READYMODE = 0;

    /* Set XINTF width to 16 bits */
    XintfRegs.XTIMING7.bit.XSIZE = 3;

    /* No bank switching delay */
    XintfRegs.XBANK.bit.BCYC = 0;
    EDIS;
}
