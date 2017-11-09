// Filename:     Func_adc.h
// Author:       Jianghui Yu
// Description:
//   ADC functions function file
//   Sense voltage and current from 12 original channels

// Modified by Ming Lu, 06/09/2017
// Sense voltage and current from 5 original channels

#include "device.h"
#include "hw_adc_constants.h"
#include <Init_adc.h>
#include <Func_adc.h>

void HW_adc_scale(void);

const float Curr_lpnum=8;
const float Ia_ADCgain= -137.4987;
const float Ib_ADCgain= -138.105;
const float Ic_ADCgain= -138.36; // 1/0.15/1.65/0.03 = 134.68
const float Vab_ADCgain= 333;
const float Vbc_ADCgain= 333; //1/0.15/2/0.01 = 333.03
const float Vab_ADCOS= -2.2; //-1.538;
const float Vbc_ADCOS= -1.2025; //-3.923;
const float Ia_ADCOS= 0.0268609; //-1.7;
const float Ib_ADCOS= 0.304100515; //-1.3401;
const float Ic_ADCOS= 0.401628912; //-0.8659;
//----------ADC calibration-------------------
// revised by Ming Lu
int32 sensor_cnt;

float sensor_Ia_total;
float sensor_Ia_average;

float sensor_Ib_total;
float sensor_Ib_average;

float sensor_Ic_total;
float sensor_Ic_average;

float sensor_Vab_total;
float sensor_Vab_average;

float sensor_Vbc_total;
float sensor_Vbc_average;
//----------ADC calibration-------------------

void ADCCalibration()
{

    //----------------ADC calibration for ManualZERO purpose-----------------
    // revised by Chien-An
    if (sensor_cnt <= 500)
    {

    	sensor_Ia_total = sensor_Ia_total + VI_S.Ia1;
    	sensor_Ib_total = sensor_Ib_total + VI_S.Ib1;
     	sensor_Ic_total = sensor_Ic_total + VI_S.Ic1;
    	sensor_Vab_total = sensor_Vab_total + VI_S.Vab1;
    	sensor_Vbc_total = sensor_Vbc_total + VI_S.Vbc1;

    	sensor_cnt++;
    }
    else
    {
    	sensor_Ia_average = sensor_Ia_total * 0.002;
    	sensor_Ib_average = sensor_Ib_total * 0.002;
    	sensor_Ic_average = sensor_Ic_total * 0.002;
    	sensor_Vab_average = sensor_Vab_total * 0.002;
     	sensor_Vbc_average = sensor_Vbc_total * 0.002;

    	sensor_cnt = 0;

    	sensor_Ia_total = 0;
    	sensor_Ib_total = 0;
    	sensor_Ic_total = 0;
    	sensor_Vab_total = 0;
    	sensor_Vbc_total = 0;
    }
    //----------------ADC calibration-----------------

}

void Sample_Data()
{
	HW_adc_scale();                        //calculate sensor output based on ADC results

	// Input AC currents
	// Ia1 -> A5, AD: A5, MCU card Pin 69
	// Ib1 -> A4
	// Ic1 -> B3
	// Input AC voltages
	// Vab -> B5
	// Vbc -> B4

	// For all currents, the gain is 1/(1/1000*R7), R7=50, calculation result is 20
	// Ia: measured gain = 18.1

	// For AC voltages, the gain is 1/(1/25e3*2.5*R7), R7=100, calculation result is 100
	// Vab: measured gain = 111.1

	//measurement on sensor 2 has opposite direction

	//The gain and offset are tuned based on actual tests



	VI_S.Ia1=(adc_val[0][5])*Ia_ADCgain-Ia_ADCOS;
	VI_S.Ib1=(adc_val[1][4])*Ib_ADCgain-Ib_ADCOS;
	VI_S.Ic1=(adc_val[1][5])*Ic_ADCgain-Ic_ADCOS;
	VI_S.Vab1=(adc_val[1][3])*Vab_ADCgain-Vab_ADCOS;
	VI_S.Vbc1=(adc_val[0][4])*Vbc_ADCgain-Vbc_ADCOS;

	//VI_S.Ic1 = ((float)(adc_res[1][5]<<4)- ADC_OS[1][5])*ADC_FS[1][5];
	//VI_S.Ia1=adc_val[0][0]*15.41+0.326;
	//VI_S.Ib1=adc_val[0][3]*15.41+0.274;
	//VI_S.Ic1=adc_val[1][0]*15.41+0.357;

	//VI_S.Vab1=adc_val[1][3]*180.45;
	//VI_S.Vbc1=adc_val[0][2]*180.45;

//	VI_S.Vdcp=adc_val[0][1]*129.87+9.067;
//	VI_S.Vdcn=adc_val[1][1]*129.87+6.067;

//	VI_S.VBa=(adc_val[1][5]*111.11);
//	VI_S.IBa=-(adc_val[0][4]*15.41)*1.0461-0.1752;

}

void HW_adc_scale(void)
{

	adc_val[0][0] = ((float)(adc_res[0][0]<<4)- ADC_OS[0][0])*ADC_FS[0][0];
	adc_val[0][1] = ((float)(adc_res[0][1]<<4)- ADC_OS[0][1])*ADC_FS[0][1];
	adc_val[0][2] = ((float)(adc_res[0][2]<<4)- ADC_OS[0][2])*ADC_FS[0][2];
	adc_val[0][3] = ((float)(adc_res[0][3]<<4)- ADC_OS[0][3])*ADC_FS[0][3];
	adc_val[0][4] = ((float)(adc_res[0][4]<<4)- ADC_OS[0][4])*ADC_FS[0][4];
	adc_val[0][5] = ((float)(adc_res[0][5]<<4)- ADC_OS[0][5])*ADC_FS[0][5];

	adc_val[1][0] = ((float)(adc_res[1][0]<<4)- ADC_OS[1][0])*ADC_FS[1][0];
	adc_val[1][1] = ((float)(adc_res[1][1]<<4)- ADC_OS[1][1])*ADC_FS[1][1];
	adc_val[1][2] = ((float)(adc_res[1][2]<<4)- ADC_OS[1][2])*ADC_FS[1][2];
	adc_val[1][3] = ((float)(adc_res[1][3]<<4)- ADC_OS[1][3])*ADC_FS[1][3];
	adc_val[1][4] = ((float)(adc_res[1][4]<<4)- ADC_OS[1][4])*ADC_FS[1][4];
	adc_val[1][5] = ((float)(adc_res[1][5]<<4)- ADC_OS[1][5])*ADC_FS[1][5];

}
