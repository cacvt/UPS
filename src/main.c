//==============================================================================
// Copyright �2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     main.c
// Author:       Z Shen
// Modified by Jianghui Yu
//
// Modified by Ming Lu, 06/09/2017
// Modified by chien-an chen, 10/26/17
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
void SVM(void);
void I_loop_VSI(void);
void O_loop_VSI(void);
void update_compare_PWM1(float DutyA_1, float DutyB_1, float DutyC_1);


//==============================================================================
// Global variable definitions

//Test task variables
unsigned long long led_tick;
unsigned long long hex_tick;
unsigned int hex_led_cntr;
unsigned int tmp;
unsigned int ad_ptr;

unsigned long long ethernet_tick;

#define PWM_Mode 1;											//set PWM mode

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

unsigned int EXSW_CTRL = 0;                 //Control signal from external switch, input from IO85_D
unsigned int SSR_CTRL = 0;                 //Control signal from DSP to SSR, output to IO87_D
unsigned int CPLD_ERRCLR = 0;				// Clear error in CPLD, output to IO14_D
unsigned int CPLD_ERRSET = 0;				// Clear error in CPLD, output to IO14_D
unsigned int CPLD_ERR = 0;					// error signal from CPLD, input from IO15_D


float out_vector_alpha_VSI 		= 0;
float out_vector_beta_VSI		= 0;
float Vdc_VSI					= 80;    // setup for DC side voltage
float Vd_out_VSI				= 40;
float Vq_out_VSI				= 0;

// control variables
float Vd_out_VSI_S				= 0;
float Vq_out_VSI_S				= 0;
float delta_id_antiwind_VSI		= 0;
float delta_iq_antiwind_VSI		= 0;
float Ka						= 1;
float In_uq_VSI					= 0;
float In_ud_VSI					= 0;
float Kii_VSI					= -122.7;
float Kpi_VSI					= -0.0604;
float Tsw_VSI					= CTRL_CLK;
float idslop_VSI				= 0;
float iqslop_VSI				= 0;
float idref_VSI					= -1;
float iqref_VSI					= 0;


//float sensor1_Ia_average 		=0;
//float sensor1_Ib_average 		=0;
//float sensor1_Ic_average 		=0;
//float sensor1_Vab_average 		=0;
//float sensor1_Vbc_average 		=0;

//float sensor1_Ia_total 		=0;
//float sensor1_Ib_total 		=0;
//float sensor1_Ic_total 		=0;
//float sensor1_Vab_total 		=0;
//float sensor1_Vbc_total 		=0;
int32 sensor1_cnt = 0;
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

//--------------PWM variable-------------------------
float Ma						= 0;
float Mk						= 0;
float duration1_1 				= 0;					//for intermediate converter 1 duty cycle calculation
float duration2_1 				= 0;					//for intermediate converter 1 duty cycle calculation
float duration0_1 				= 0;					//for intermediate converter 1 duty cycle calculation
int region_SVPWM_1 				= 0;					// location region of reference voltage for SVPWM
float region_angle_SVPWM_1 		= 0;					// angle in location region of reference voltage for SVPWM
int region_DPWM_1 				= 0;					// location region of reference voltage for DPWM
float region_angle_DPWM_1 		= 0;					// angle in location region of reference voltage for DPWM
float DutyA_1 					= 0;					//phase A duty cycle of converter 1
float DutyB_1 					= 0;					//phase B duty cycle of converter 1
float DutyC_1 					= 0;					//phase C duty cycle of converter 1
//==============================================================================
// Function definitions


//EPWM parameters

//Interleaving Configuration
#define EPWM_INTER_ANGLE1 0					//Set Interleaving Angle for Converter 1, maintain zero
#define EPWM_INTER_ANGLE2 0				//Set Interleaving Angle for Converter 2, change as needed (=interleave angle/2/pi from 0 to 0.5)
//6.4 for no L : 0.01785714285714285714285714285714
//13 for add L @20kHz: 0.03846153846153846153846153846154
//16 for add L @ 25kHz: 0.04545454545454545454545454545455
//20 for add L @ 30kHz: 0.05555555555555555555555555555556
//25 for add L @ 40kHz: 0.07142857142857142857142857142857

typedef struct
{
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwm_CMPA_Direction;
   Uint16 EPwm_CMPB_Direction;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmMaxCMPA;
   Uint16 EPwmMinCMPA;
   Uint16 EPwmMaxCMPB;
   Uint16 EPwmMinCMPB;
}EPWM_INFO;

//------------------------------------------Calculate Global variables based on system parameter------------------------
#define EPWM_TIMER_TBPRD   200*1000000/2/SW_FREQ	// Calculate Period register Value (for 200MHz/DIV4)
#define EPWM1_TIMER_TBPRD  EPWM_TIMER_TBPRD  		// send to each Period register
#define EPWM2_TIMER_TBPRD  EPWM_TIMER_TBPRD
#define EPWM3_TIMER_TBPRD  EPWM_TIMER_TBPRD
#define EPWM4_TIMER_TBPRD  EPWM_TIMER_TBPRD
#define EPWM5_TIMER_TBPRD  EPWM_TIMER_TBPRD
#define EPWM6_TIMER_TBPRD  EPWM_TIMER_TBPRD
#define EPWM7_TIMER_TBPRD  EPWM_TIMER_TBPRD
#define EPWM8_TIMER_TBPRD  EPWM_TIMER_TBPRD
#define EPWM9_TIMER_TBPRD  EPWM_TIMER_TBPRD
#define EPWM_CMPA_INITIAL 0										//Set Initial CMPA register value
// limitation on duty cycle minmumpuls witdth =2*DB
#define EPWM_MAX_CMP      EPWM_TIMER_TBPRD-EPWM_DB
#define EPWM_MIN_CMP      EPWM_DB
// Enable/Disable PWM interupt
#define PWM_INT_ENABLE  1
#define PWM_INT_DISABLE  0
// Enable/Disable PWM TZ interupt
#define PWM_TZEINT_ENABLE  1
#define PWM_TZEINT_DISABLE  0
// Global variables used in this example
Uint32  EPwm1TZIntCount;									// counter of TZ interrupt
Uint16  TZIntCount_Statu_CBC = 0;							// TZ interrupt statues Cycle By Cycle
Uint16  TZIntCount_Statu_OST = 0;							// TZ interrupt statues One Shot
Uint16  EPwm_Int_Counter = 0;								// PWM interrupt counter
Uint16  EPwm_Int_LedCounter = 0;							// PWM interrupt counter for LED indication
// Enable/Disable PWM TZ interupt
#define PWM_TZEINT_ENABLE  1
#define PWM_TZEINT_DISABLE  0

// indicate compare conuter direction for test
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0
EPWM_INFO epwm1_info;										// Information of each PWM module

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
	CPLD_FAULT_CLR;
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
		//CPLD_FAULT_CLR;

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
	IER = M_INT2 | M_INT3;			// Trip zone interrupt for protect-
									//   -ion & PWM update ISR to update
									//   PWM registers
	EINT;
	// Re-enable high level interrupts
	tick++;								    // increase time tick for LED display
	tick_eth++;                             // increase time tick for ethernet communication

//==============================================================================
// User code begin

	Sample_Data();  // load and calibrate ADC data into VI_S
	//ADCCalibration(); // manual calibration for ADC zero
    //----------------ADC calibration for ManualZERO purpose-----------------
    // revised by Chien-An
//    if (sensor1_cnt <= 500)
//   {

//    	sensor1_Ia_total = sensor1_Ia_total + VI_S.Ia1;
//    	sensor1_Ib_total = sensor1_Ib_total + VI_S.Ib1;
//     	sensor1_Ic_total = sensor1_Ic_total + VI_S.Ic1;
//    	sensor1_Vab_total = sensor1_Vab_total + VI_S.Vab1;
//    	sensor1_Vbc_total = sensor1_Vbc_total + VI_S.Vbc1;

    	//sensor1_cnt++;
    //}
    //else
    //{
    	//sensor1_Ia_average = sensor1_Ia_total * 0.002;
    	//sensor1_Ib_average = sensor1_Ib_total * 0.002;
    	//sensor1_Ic_average = sensor1_Ic_total * 0.002;
    	//sensor1_Vab_average = sensor1_Vab_total * 0.002;
     	//sensor1_Vbc_average = sensor1_Vbc_total * 0.002;

    	//sensor1_cnt = 0;

    	//sensor1_Ia_total = 0;
    	//sensor1_Ib_total = 0;
    	//sensor1_Ic_total = 0;
    	//sensor1_Vab_total = 0;
    	//sensor1_Vbc_total = 0;
    //}
    //----------------ADC calibration-----------------

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
	VI_S.Vab1=0;VI_S.Vbc1=0;VI_S.Vca1=0;
	VI_S.Vdcp=0;VI_S.Vdcn=0;
	VI_S.VBa=0;VI_S.IBa=0;
	VI_S.Ia2=0;VI_S.Ib2=0;VI_S.Ic2=0;
	VI_C.VavgA=0;VI_C.VavgB=0;VI_C.VavgC=0;          //calculated data intilization
	//RELAY_PRE;                                       //connect DC source for pre-charge

	CPLD_NO_FAULT_SET;                   //clear DSP set error
	//CPLD_FAULT_CLR;
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
		HW_pll(&syn,VI_C.Vq);                        //calculate PLL
		syn.theta = syn.theta>=2*PI ? syn.theta-2*PI : syn.theta; //theta should be within [0,2*PI]
		syn.theta = syn.theta<=0 ? syn.theta+2*PI : syn.theta;
//
		theta=syn.theta;                             //use angle from PLL
		sintheta=sin(theta);
		costheta=cos(theta);
// 		power invariant d-q frame

		VI_S.Vca1=-VI_S.Vab1-VI_S.Vbc1;
//		VI_C.Valpha=(VI_S.Vab1-VI_S.Vbc1*0.5-VI_S.Vca1*0.5)*SQRT2OVER3; //calculate dq frame phase voltage using line-line voltage
//		VI_C.Vbeta =SQRT3_2*(VI_S.Vbc1- VI_S.Vca1)*SQRT2OVER3;
//		VI_C.Vd    =(costheta*VI_C.Valpha+sintheta*VI_C.Vbeta)*DIV1_SQRT3;	   // convert into rms value
//		VI_C.Vq    =(-sintheta*VI_C.Valpha+costheta*VI_C.Vbeta)*DIV1_SQRT3;   // convert into rms value

		VI_C.Ialpha=(VI_S.Ia1-VI_S.Ib1*0.5-VI_S.Ic1*0.5)*SQRT2OVER3; //calculate dq frame phase voltage using line-line voltage
		VI_C.Ibeta =SQRT3_2*(VI_S.Ib1- VI_S.Ic1)*SQRT2OVER3;
		VI_C.Id    =(costheta*VI_C.Ialpha+sintheta*VI_C.Ibeta)*DIV1_SQRT3;	   // convert into rms value
		VI_C.Iq    =(-sintheta*VI_C.Ialpha+costheta*VI_C.Ibeta)*DIV1_SQRT3;   // convert into rms value

		VI_C.Van=DIV1_3*(VI_S.Vab1-VI_S.Vca1);
		VI_C.Vbn=DIV1_3*(VI_S.Vbc1-VI_S.Vab1);
		VI_C.Vcn=DIV1_3*(VI_S.Vca1-VI_S.Vbc1);
		VI_C.Valpha=(VI_C.Van-VI_C.Vbn*0.5-VI_C.Vcn*0.5)*SQRT2OVER3; //calculate dq frame phase voltage using line-line voltage
		VI_C.Vbeta =SQRT3_2*(VI_C.Vbn- VI_C.Vcn)*SQRT2OVER3;
		VI_C.Vd    =(costheta*VI_C.Valpha+sintheta*VI_C.Vbeta);	   // convert into rms value
		VI_C.Vq    =(-sintheta*VI_C.Valpha+costheta*VI_C.Vbeta);   // convert into rms value

		VI_C.Vdtt = VI_C.Vdtt + VI_C.Vd;
		VI_C.Vqtt = VI_C.Vqtt + VI_C.Vq;

		VI_C.Idtt = VI_C.Idtt + VI_C.Id;
		VI_C.Iqtt = VI_C.Iqtt + VI_C.Iq;

		if (sensor1_cnt++>=500)
		{
			VI_C.Vdavg = VI_C.Vdtt*0.002;
			VI_C.Vqavg = VI_C.Vqtt*0.002;
			VI_C.Idavg = VI_C.Idtt*0.002;
			VI_C.Iqavg = VI_C.Iqtt*0.002;
			VI_C.Vdtt=0;VI_C.Vqtt=0;VI_C.Idtt=0;VI_C.Iqtt=0;
			sensor1_cnt=0;
		}
		else
		{
		}

//	}
		EXSW_CTRL = GpioDataRegs.GPCDAT.bit.GPIO85;
		CPLD_ERR = GpioDataRegs.GPBDAT.bit.GPIO62;
		GpioDataRegs.GPCDAT.bit.GPIO87 = SSR_CTRL;
		GpioDataRegs.GPADAT.bit.GPIO14 = CPLD_ERRSET;
		GpioDataRegs.GPADAT.bit.GPIO15 = CPLD_ERRCLR;

// manual controlled by DIPSwitch SW1 on control board
// GPIO84 = SW1.pin1; GPIO85 = SW1.pin2; GPIO86 = SW1.pin3; GPIO87 = SW1.pin4
    if (GpioDataRegs.GPCDAT.bit.GPIO84 == 1) //if current loop closed
    {
    	PI_DC_I.Fdb=VI_S.IBa;PI_DC_I.calc(&PI_DC_I);      //calculate DC stage battery current control command
    }
    else
    {
    	PI_DC_I.Ui=0;	//reset controller //PID integral output
    	PI_DC_I.Out=0;	//PID output
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
//    Duty_Ratio[6][0]=0.25+PI_DC_I.Out+PI_DC_V.Out;
//		Duty_Ratio[6][1]=0.25+PI_DC_I.Out-PI_DC_V.Out;

    //For Boost Operation
//    Duty_Ratio[6][0]=0.675+PI_DC_I.Out-PI_DC_V.Out;
//	  Duty_Ratio[6][1]=0.675+PI_DC_I.Out+PI_DC_V.Out;

	//Vd_out_VSI = 40;
	//Vq_out_VSI = 0;

	//update_D();       // original pwm update in UPS

	//I_loop_VSI();
    if (EXSW_CTRL==0)
    {
	O_loop_VSI();
    }
    else
    {
    I_loop_VSI();
    }
	// open loop VSI
	SVM();
	update_compare_PWM1(DutyA_1, DutyB_1, DutyC_1);

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


void I_loop_VSI(void)
{
	delta_id_antiwind_VSI = Ka*(Vd_out_VSI_S-Vd_out_VSI);
	delta_iq_antiwind_VSI = Ka*(Vq_out_VSI_S-Vq_out_VSI);

	idslop_VSI				= idref_VSI;
	iqslop_VSI				= iqref_VSI;
	In_ud_VSI				= In_ud_VSI+Kii_VSI*(idslop_VSI-VI_C.Id-delta_id_antiwind_VSI)*Tsw_VSI;
	In_uq_VSI				= In_uq_VSI+Kii_VSI*(iqslop_VSI-VI_C.Iq-delta_iq_antiwind_VSI)*Tsw_VSI;
//	Vd_out_VSI				= K_decoup_VSI*Two_PI*f_line*Lout_VSI*Iq_VSI+Kpi_VSI*(idslop_VSI-Id_VSI)+In_ud_VSI;
	Vd_out_VSI				= Kpi_VSI*(idslop_VSI-VI_C.Id)+In_ud_VSI;
//	Vq_out_VSI				= -K_decoup_VSI*Two_PI*f_line*Lout_VSI*Id_VSI+Kpi_VSI*(iqslop_VSI-Iq_VSI)+In_uq_VSI;
	Vq_out_VSI				= Kpi_VSI*(iqslop_VSI-VI_C.Iq)+In_uq_VSI;

	if (Vd_out_VSI< -Vdc_VSI)
	{
		Vd_out_VSI_S = -Vdc_VSI;
	}
	else
	{
		if (Vd_out_VSI> Vdc_VSI)
		{
		Vd_out_VSI_S = Vdc_VSI;
		}
		else
		{
			Vd_out_VSI_S = Vd_out_VSI;
		}
	}
	if (Vq_out_VSI< -Vdc_VSI)
	{
		Vq_out_VSI_S = -Vdc_VSI;
	}
	else
	{
		if (Vq_out_VSI> Vdc_VSI)
		{
		Vq_out_VSI_S = Vdc_VSI;
		}
		else
		{
			Vq_out_VSI_S = Vq_out_VSI;
		}
	}

	out_vector_alpha_VSI	= costheta*Vd_out_VSI_S-sintheta*Vq_out_VSI_S;
	out_vector_beta_VSI		= sintheta*Vd_out_VSI_S+costheta*Vq_out_VSI_S;
	Ma						= out_vector_alpha_VSI*SQRT3over2/Vdc_VSI;							// Valpha*sqrt(3/2)/Vdc
	Mk						= out_vector_beta_VSI*SQRT1over2/Vdc_VSI;							// Vbeta*sqrt(3/2)/Vdc/sqrt(3)

//	output_vector_angle_2	= atan2(out_vector_beta_VSI, out_vector_alpha_VSI);

//	if(output_vector_angle_2<0)
//	{
//		output_vector_angle_2=output_vector_angle_2+2*PI;
//	}

}

void O_loop_VSI(void)
{
	out_vector_alpha_VSI = costheta*Vd_out_VSI-sintheta*Vq_out_VSI;
	out_vector_beta_VSI	= sintheta*Vd_out_VSI+costheta*Vq_out_VSI;

	Ma = out_vector_alpha_VSI*SQRT3over2/Vdc_VSI;
	Mk = out_vector_beta_VSI*SQRT1over2/Vdc_VSI;
}


void SVM(void)
{
//	switch (PWM_Mode)
//	{
//		case 1:						// center aligned continious PWM
//		{
			/*deterimine the region and the phase_angle in the region for SVPWM*/
			/*     010    110
					|_  II _|
				 III  |___|	 I
			  011_______|________100
					   _|_
				 IV  _|	  |_ VI
					|   V   |
				   001     101
			*/

			// find the section
			if (Ma > 0)
			{
				if (Mk > 0)
				{
					if (Ma > Mk)
					{
						region_SVPWM_1	= 1;
						duration1_1		= Ma-Mk;
						duration2_1		= 2*Mk;
					}
					else
					{
						region_SVPWM_1	= 2;
						duration1_1		= Ma+Mk;
						duration2_1		= -Ma+Mk;
					}
				}
				else
					{
						if (Ma > -Mk)
						{
							region_SVPWM_1	= 6;
							duration1_1		= -2*Mk;
							duration2_1		= Ma+Mk;
						}
						else
						{
							region_SVPWM_1	= 5;
							duration1_1		= -Ma-Mk;
							duration2_1		= Ma-Mk;
						}
					}
			}
			else
			{
				if (Mk > 0)
				{
					if (-Ma > Mk)
					{
						region_SVPWM_1	= 3;
						duration1_1		= 2*Mk;
						duration2_1		= -Ma-Mk;
					}
					else
					{
						region_SVPWM_1	= 2;
						duration1_1		= Ma+Mk;
						duration2_1		= -Ma+Mk;
					}
				}
				else
				{
					if (Ma < Mk)
					{
						region_SVPWM_1	= 4;
						duration1_1		= -Ma+Mk;
						duration2_1		= -2*Mk;
					}
					else
					{
						region_SVPWM_1	= 5;
						duration1_1		= -Ma-Mk;
						duration2_1		= Ma-Mk;
					}
				}
			}

			duration0_1=1-duration1_1-duration2_1;

			//calculate duty cycle based on location of output vector
			switch (region_SVPWM_1)
			{
				case 1:
				{
					DutyA_1=1-duration0_1/2;
					DutyB_1=duration2_1+duration0_1/2;
					DutyC_1=duration0_1/2;
					break;
				}
				case 2:
				{
					DutyA_1=duration1_1+duration0_1/2;
					DutyB_1=1-duration0_1/2;
					DutyC_1=duration0_1/2;
					break;
				}
				case 3:
				{
					DutyA_1=duration0_1/2;
					DutyB_1=1-duration0_1/2;
					DutyC_1=duration2_1+duration0_1/2;
					break;
				}
				case  4:
				{
					DutyA_1=duration0_1/2;
					DutyB_1=duration1_1+duration0_1/2;
					DutyC_1=1-duration0_1/2;
					break;
				}
				case  5:
				{
					DutyA_1=duration2_1+duration0_1/2;
					DutyB_1=duration0_1/2;
					DutyC_1=1-duration0_1/2;
					break;
				}
				case 6:
				{
					DutyA_1=1-duration0_1/2;
					DutyB_1=duration0_1/2;
					DutyC_1=duration1_1+duration0_1/2;
					break;
				}
			}
}


void update_compare_PWM1(float DutyA_1, float DutyB_1, float DutyC_1)
{
   int CMP_Origin_A, CMP_Origin_B, CMP_Origin_C;
   CMP_Origin_A=DutyA_1*EPWM1_TIMER_TBPRD;
   CMP_Origin_B=DutyB_1*EPWM1_TIMER_TBPRD;
   CMP_Origin_C=DutyC_1*EPWM1_TIMER_TBPRD;


   	EPwm1Regs.CMPA.half.CMPA = D[0][0];              //update output PWM duty ratio
   	EPwm1Regs.CMPB =D[0][1];

   if (CMP_Origin_A > EPWM_MAX_CMP)
   {
//   		EPwm1Regs.CMPA.half.CMPA=EPWM_MAX_CMP;
   		EPwm1Regs.CMPA.half.CMPA=EPWM1_TIMER_TBPRD;
   }
   else
   {
   		if (CMP_Origin_A < EPWM_MIN_CMP)
   		{
//   			EPwm1Regs.CMPA.half.CMPA=EPWM_MIN_CMP;
				EPwm1Regs.CMPA.half.CMPA=0;
   		}
   		else
   		{
   			EPwm1Regs.CMPA.half.CMPA=CMP_Origin_A;
   		}
   }
   if (CMP_Origin_B > EPWM_MAX_CMP)
   {
//   		EPwm2Regs.CMPA.half.CMPA=EPWM_MAX_CMP;
   		EPwm2Regs.CMPA.half.CMPA=EPWM1_TIMER_TBPRD;
   }
   else
   {
   		if (CMP_Origin_B < EPWM_MIN_CMP)
   		{
//   			EPwm2Regs.CMPA.half.CMPA=EPWM_MIN_CMP;
   			EPwm2Regs.CMPA.half.CMPA=0;
   		}
   		else
   		{
   			EPwm2Regs.CMPA.half.CMPA=CMP_Origin_B;
   		}
   }
   if (CMP_Origin_C > EPWM_MAX_CMP)
   {
//   		EPwm3Regs.CMPA.half.CMPA=EPWM_MAX_CMP;
   		EPwm3Regs.CMPA.half.CMPA=EPWM1_TIMER_TBPRD;
   }
   else
   {
   		if (CMP_Origin_C < EPWM_MIN_CMP)
   		{
//   			EPwm3Regs.CMPA.half.CMPA=EPWM_MIN_CMP;
   			EPwm3Regs.CMPA.half.CMPA=0;

   		}
   		else
   		{
   			EPwm3Regs.CMPA.half.CMPA=CMP_Origin_C;
   		}
   }
   return;
}
//========================================================Update PWM1 CMP Register value end========================================================
