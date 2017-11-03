//pr.h
/*
File name:   pr.h                   
                    
Originator:	 Digital Control Systems Group
			 RXPE Corp. Ltd

Description: Header file containing constants, data type definitions, and 
             function prototypes for the pid.
Author:
History:
             24-06-2011		Release	Rev 1.01
-------------------------------------------------------------------------------------                
 */  
#ifndef PR_H
#define PR_H


//PR¿ØÖÆ²ÎÊý
typedef   struct
{
    float       fKp                             ;//proportion parameter
    float       fKr                             ;//resonant parameter
    float       fWc                             ;//bandwidth, wc
	float       fWo                             ;//resonant frequency
    float       fPrOutMax                       ;//output upper limit
    float       fPrOutMin                       ;//output lower limit
    float       fReOutMax                       ;//resonat upper limit
    float       fReOutMin                       ;//resonat lower limit
    float       fReUr                           ;//initial value of resonant
    float       fResave1                        ;//resave

}sPrPara;  

#define  PR_DEFAULTS   {       0.1,\
                               100,\
							   10,\
							   2*60*PI*2,\
							   0.05*Vc,\
                               -0.05*Vc,\
                               0.05*Vc,\
                               -0.05*Vc,\
                               0,\
                               0  }
//second order frequency
//wc=10
//PR output limit: 0.05 in duty ratio
//R output limit: 0.05 in duty ratio

typedef  struct 
{
    float       fSampleIn                       ;//	input error
    float       fSampleOut                      ;//	output
   	void        (*calc)()		                ;//	interface
	void        (*init)()		                ;//	interface
    float       fe1                             ;// error of last time
    float       fe2                             ;// error of last two times
	float       fur0                            ;// output of resonant of this time
	float       fur1                            ;// output of resonant of last time
	float       fur2                            ;// output of resonant of last two times
	sPrPara*	pPara                           ;// parameter pointer
}sPrCtrl;

 
void CalcPrCtrl(sPrCtrl* pPrCtrl);
void InitPrCtrl(sPrCtrl* pPrCtrl);
                    
#define PR_CTRL_DEFAULTS {	   0, \
                               0, \
                               (void (*)(  unsigned long)) CalcPrCtrl,\
              		    	   (void (*)(  unsigned long)) InitPrCtrl,\
							   0, \
							   0, \
							   0, \
							   0, \
							   0, \
							   &Para_PR_5 }

#endif 
