//==============================================================================
// Copyright �2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     main.c
// Author:       Z Shen
// Modified by Jianghui Yu
//
// Modified by Ming Lu, 06/09/2017
// Modified by chien-an chen, 10/24/17
//
// Description:
//   7 PWM modules are used, complementary signals and deadtime are generated in CPLD
//       1-3 for AC input, 4-6 for AC output. A and B for 1st and 2nd device from the top
//       7 for DC stage.                      A and B for 1st and 4th device from the top
//   Dc stage output current control enabled
//   Dc stage voltage balancing enabled
//   12 sensing feedback channels are used, current sensors are compensated
//
// To be achived:
//   Communication with computer to receive commands and send states
//   GUI displays:
//
// Comment:
//   Remember to check the current reference!!!
//==============================================================================

#include <Initialization.h>
#include "DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include <Control.h>

//==============================================================================
// Function declarations
interrupt void ctrl_isr(void);

void main_loop(void) __attribute__((noreturn));
interrupt void epwm9_timer_isr(void);

void HW_op(void);
void DELAY_MS(Uint32 t);
void control_init(void);
void control_function(void);
void ADCCalibration(void);
void init_D(void);
void update_D(void);
void update_PWM(void);

//==============================================================================
// Global variable definitions

//Test task variables
unsigned long long led_tick;
unsigned long long hex_tick;
unsigned int hex_led_cntr;
unsigned int tmp;
unsigned int ad_ptr;

unsigned long long ethernet_tick;

//----------control reference--------------------//feel free to change
//#define Vc 300.0                        //Set cap voltage refernece, move it to device.h
//const float Vc_th[3]={10,0.95*Vc,Vc};     //Set cap voltage charge/discharge threshold
//const float I_th=Vc/60*1.5;               //Set fault detection threshold
//const float Vac=0.3919*Vc;                //Set desired ouput voltage, 480 V, line-line, RMS

//----------state variables----------------------//need new definition
//ctrl_state_data ctrl_state;

//----------time related variables---------------//just a few counters
Uint32 counter_1[3]={0,0,0};              //counter for delay in main
Uint32 counter_2=0;                       //counter for delay in interrupt (for DC stage)
//Uint32 counter_3=0;                       //counter for delay in interrupt (in fault operation)
//Uint32 counter_4=0;                       //counter for mean value calculation

//----------sensing data variables---------------
SAMPLE_DATA  	VI_S;

//----------calculation variable-----------------
float Duty_Ratio[7][2];                   //Phase leg duty ratio
Uint32 D[7][2];                           //two duty ratio for each phase leg
CALCULATE_DATA	VI_C;		          //calculated data

//for PLL, no need to change
float theta=0,sintheta=0, costheta=0;     //angle information
struct SYNC syn;                          //PLL
unsigned int startsync = 0;               //Flag for start PLL
unsigned int flagSync = 8;		          //Flag for PLL synchronization ready
unsigned int cntSync = 0;                 //counter for PLL synchronization

//control parameters
PIDREG3 PI_DC_I=PIDREG1_DEFAULTS;           //PI controller for DC stage battery current control
PIDREG3 PI_DC_V=PIDREG2_DEFAULTS;           //PI controller for DC stage voltage balancing control

//const float Kp6=0.03;                     //P controller gain for individual voltage control
//float P_6[6]={0,0,0,0,0,0};               //P controller output for individual voltage control, 6 PEBBs

//float vdq[2]={0,0};                       //dq frame voltage reference
//float vdq_lim[2]={0,0.45*Vc/SQRT2OVER3};  //dq frame voltage reference limit, (0.45*Vc/SQRT2OVER3)
//float vab[2]={0,0};                       //alpha-beta frame voltage reference
//float vac[3]={0,0,0};                     //AC voltage reference, 3 phases

//display on GUI
//float Vc_av[13]={0,0,0,0,0,0,0,0,0,0,0,0};//a set of average sensed signals and control signals

//==============================================================================
// Function definitions


void main(void) {

    HW_init(); /* Initialize hardware*/
    control_init();//Initilize control variable and output

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM9_INT = &epwm9_timer_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    IER |= M_INT11;
    PieCtrlRegs.PIEIER11.bit.INTx1 = 1;

	HW_set_ctrl_isr(ctrl_isr); /* Set control ISR */

	// System tick initialization
	led_tick = 0;
	hex_tick = 0;
	hex_led_cntr = 0;
	ad_ptr = 0;

	ethernet_tick=0;

	EINT;
	main_loop(); /* Start the super loop */

}

//=============================================================================
// The super loop
//
//=============================================================================
void main_loop(void) {
	for (;;) {
//=============================================================================
// Starting point of task code

		//=========================================================================
		// Ethernet data processing task
		//   Task is executed every loop to maximize through output
//		HW_eth_task();
//
//		if (tick_eth >= ethernet_tick)    // update parameters from HMI and CPLD every 50ms
//		   {
//			   while (ethernet_tick <= tick_eth)
//			   {ethernet_tick += (0.05 * CTRL_FREQ) ;}   //time is 0.5s=50ms
//
//			   update_var();              //update parameters from HMI
//		   }

		//=========================================================================
		// LED blinking task
		if (tick >= led_tick) {
			while (led_tick <= tick)
				led_tick += (0.5 * CTRL_FREQ);

			CC_LED0_TOOGLE;
			CC_LED1_TOOGLE;
		}

		//=========================================================================
		// Operation mode processing task
		HW_op();
		CPLD_FAULT_CLR;

		//=========================================================================
		// HEX LED update blinking task
		if (tick >= hex_tick) {
			while (hex_tick <= tick)
				hex_tick += (0.1 * CTRL_FREQ);
			tmp = HW_cpld_reg_read_poll(REG_CPLD_INPUT0);
			HW_cpld_reg_read_poll(REG_CPLD_INPUT1);
			HW_cpld_reg_write_poll(REG_HEX_LED, hex_led_cntr++);
			HW_cpld_reg_write_poll(REG_CPLD_OUTPUT1, 0x40);
			HW_cpld_reg_write_poll(REG_CPLD_OUTPUT1, 0x00);
			HW_cpld_reg_write_poll(REG_CPLD_SET1, 0x40);
			HW_cpld_reg_write_poll(REG_CPLD_CLEAR1, 0x40);
			HW_cpld_reg_write_poll(REG_CPLD_TOGGLE1, 0x40);
			HW_cpld_reg_write_poll(REG_CPLD_TOGGLE1, 0x40);
		}
// End point of task code
//=============================================================================
	}
}

//=============================================================================
// The interrupt routine for converter control
// - Macro is preferred in this section
// - Use global variable if the state of it is to be maintained during different
//   calls
// - Use tempoary variable declared in the function if the state does not matter
//=============================================================================
interrupt void ctrl_isr(void)
{
	PROFILE_ISR_START
	;
	// Execution time monitoring
	IER = M_INT2 | M_INT3;					// Trip zone interrupt for protect-
											//   -ion & PWM update ISR to update
											//   PWM registers
	EINT;
	// Re-enable high level interrupts
	tick++;								    // increase time tick for LED display
	tick_eth++;                             // increase time tick for ethernet communication

//==============================================================================
// User code begin

	Sample_Data();  // load and calibrate ADC data into VI_S
	ADCCalibration(); // manual calibration for ADC zero
	control_function();

//==============================================================================

	DINT;  // Disable global interrupt
	PROFILE_ISR_STOP;  // Execution time monitoring
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;     // Acknowledge interrupt  to accept new ones
}

interrupt void epwm9_timer_isr(void)
{




	GpioDataRegs.GPBSET.bit.GPIO56 = 1;
    GpioDataRegs.GPBSET.bit.GPIO57 = 1;
    asm(" nop");
    asm(" nop");
    asm(" nop");
    GpioDataRegs.GPBCLEAR.bit.GPIO56 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;

    EPwm9Regs.ETCLR.bit.INT = 1;
     PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;

}

void control_init(void)
{
	CPLD_FAULT_SET;                                  //CPLD error initialization, set error, all switches off

	init_D();                                    //PWM duty ratio initilization

	HW_sync_init(&syn);

	VI_S.Ia1=0;VI_S.Ib1=0;VI_S.Ic1=0;                //Sensed data initilization
	VI_S.Vab1=0;VI_S.Vbc1=0;
	VI_S.Vdcp=0;VI_S.Vdcn=0;
	VI_S.VBa=0;VI_S.IBa=0;
	VI_S.Ia2=0;VI_S.Ib2=0;VI_S.Ic2=0;
	VI_C.VavgA=0;VI_C.VavgB=0;VI_C.VavgC=0;          //calculated data intilization
	//RELAY_PRE;                                       //connect DC source for pre-charge

	CPLD_NO_FAULT_SET;                   //clear DSP set error
	CPLD_FAULT_CLR;
}

void HW_op(void)                                     //Process off/pre-charge/rectifier/discharge operation mode
{

}

void DELAY_MS(Uint32 t)
{   //delay t ms without interrupt additional ADC or HMI communication
	//additioanl ADC is called every 25 us, HMI is called every 100 ms
	counter_1[0]=t*40;
	counter_1[1]=0;counter_1[2]=4000;
	while (counter_1[1]<counter_1[0])
	{
		DELAY_US(25);
		counter_1[1]=counter_1[1]+1;

		if (counter_1[1]>counter_1[2])
		{
			update_var();
			HW_eth_task();
			counter_1[2]=counter_1[2]+4000;
		}
	}
}

void control_function(void)
{
//  PLL, leave it here for now
//	if (startsync==1)
//	{
//		if (flagSync==8)                             //if not synchronized yet
//		{
//			if (cntSync<10000)
//			{
//				if (syn.omega > 2*60*PI*0.995 && syn.omega < 2*60*PI*1.005)//if error is smaller than 0.5%
//					cntSync++;
//				else cntSync = 0;
//			}
//			else                                     //if omega is in range for 10000 consecutive control cycles
//			{
//				flagSync = 1;
//				cntSync = 0;
//			}
//		}
//		else
//		{}
//
//		HW_pll(&syn,VI_C.Vq);                        //calculate PLL
//		syn.theta = syn.theta>=2*PI ? syn.theta-2*PI : syn.theta; //theta should be within [0,2*PI]
//		syn.theta = syn.theta<=0 ? syn.theta+2*PI : syn.theta;
//
		theta=syn.theta;                             //use angle from PLL
		sintheta=sin(theta);
		costheta=cos(theta);
//
		VI_C.Valpha=(VI_S.Vab-VI_S.Vbc*0.5-VI_S.Vca*0.5)*SQRT2OVER3; //calculate dq frame phase voltage using line-line voltage
		VI_C.Vbeta =SQRT3_2*(VI_S.Vbc- VI_S.Vca)*SQRT2OVER3;
		VI_C.Vd    =(costheta*VI_C.Valpha+sintheta*VI_C.Vbeta)*DIV1_SQRT3;
		VI_C.Vq    =(-sintheta*VI_C.Valpha+costheta*VI_C.Vbeta)*DIV1_SQRT3;
//	}

    if (GpioDataRegs.GPCDAT.bit.GPIO84 == 1) //if current loop closed
    {
    	PI_DC_I.Fdb=VI_S.IBa;PI_DC_I.calc(&PI_DC_I);      //calculate DC stage battery current control command
    }
    else
    {
    	PI_DC_I.Ui=0;                        //reset controller
    	PI_DC_I.Out=0;
    }

    if (GpioDataRegs.GPCDAT.bit.GPIO85 == 1) //if voltage loop closed
    {
    	PI_DC_V.Fdb=VI_S.Vdcp-VI_S.Vdcn;PI_DC_V.calc(&PI_DC_V);      //calculate DC stage voltage balancing control command
    }
    else
    {
    	PI_DC_V.Ui=0;                        //reset controller
    	PI_DC_V.Out=0;
    }

    //For Buck Operation
    Duty_Ratio[6][0]=0.5+PI_DC_I.Out+PI_DC_V.Out;
	Duty_Ratio[6][1]=0.5+PI_DC_I.Out-PI_DC_V.Out;

    //For Boost Operation
//    Duty_Ratio[6][0]=0.675+PI_DC_I.Out-PI_DC_V.Out;
//	  Duty_Ratio[6][1]=0.675+PI_DC_I.Out+PI_DC_V.Out;

	update_D();


//  For communication, leave it here
//	counter_4++;                                     //update time counter for average calculation
//	Vc_av[0]=Vc_av[0]+syn.omega/2/PI/CTRL_FREQ;      //calculate mean value
//	Vc_av[1]=Vc_av[1]+VI_S.Vdc/CTRL_FREQ;
//	Vc_av[2]=Vc_av[2]+VI_C.Vd/CTRL_FREQ;
//	Vc_av[3]=Vc_av[3]+VI_C.Vq/CTRL_FREQ;
//	Vc_av[4]=Vc_av[4]+VI_C.Id/CTRL_FREQ;
//	Vc_av[5]=Vc_av[5]+VI_C.Iq/CTRL_FREQ;
//	Vc_av[6]=Vc_av[6]+VI_S.VAu/CTRL_FREQ;
//	Vc_av[7]=Vc_av[7]+VI_S.VAl/CTRL_FREQ;
//	Vc_av[8]=Vc_av[8]+VI_S.VBu/CTRL_FREQ;
//	Vc_av[9]=Vc_av[9]+VI_S.VBl/CTRL_FREQ;
//	Vc_av[10]=Vc_av[10]+VI_S.VCu/CTRL_FREQ;
//	Vc_av[11]=Vc_av[11]+VI_S.VCl/CTRL_FREQ;
//	Vc_av[12]=Vc_av[12]+VI_C.Vavg/CTRL_FREQ;

//	if (counter_4>CTRL_FREQ)                         //update mean value every 1 sec
//	{
//		counter_4=counter_4-CTRL_FREQ;
//
//		HMI.flag=flagSync;
//
//		Vc_av[0]=0;
//	}
}

void init_D(void)
{
	int n=0;
	while (n<6)                                      //initiliaze half-bridge switch duty ratio
	{
		Duty_Ratio[n][0]=0.5;
		Duty_Ratio[n][1]=0.5;
		n++;
	}

	Duty_Ratio[6][0]=0.5;
	Duty_Ratio[6][1]=0.5;

	update_D();
}


void update_D(void)
{
	int n=0;
	while (n<7)                                      //calculate half-bridge switch duty ratio
	{
		D[n][0]=Duty_Ratio[n][0]*SW_PRD_HALF;
		D[n][1]=Duty_Ratio[n][1]*SW_PRD_HALF;
		n++;
	}

	update_PWM();
}


void update_PWM(void)
{
	EPwm1Regs.CMPA.half.CMPA = D[0][0];              //update output PWM duty ratio
	EPwm1Regs.CMPB =D[0][1];

	EPwm2Regs.CMPA.half.CMPA = D[1][0];
	EPwm2Regs.CMPB =D[1][1];

	EPwm3Regs.CMPA.half.CMPA = D[2][0];
	EPwm3Regs.CMPB =D[2][1];

	EPwm4Regs.CMPA.half.CMPA = D[3][0];
	EPwm4Regs.CMPB =D[3][1];

	EPwm5Regs.CMPA.half.CMPA = D[4][0];
	EPwm5Regs.CMPB =D[4][1];

	EPwm6Regs.CMPA.half.CMPA = D[5][0];
	EPwm6Regs.CMPB =D[5][1];

	EPwm7Regs.CMPA.half.CMPA = D[6][0];
	EPwm7Regs.CMPB =D[6][1];
}

