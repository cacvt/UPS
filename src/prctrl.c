//prctrl.c
#include "pr.h"
#include "device.h"
/*
Function:     PR controller
Interface:    input:error; output: control output
Description:  U=Up+Ur; Up is propotion output, Ur is resonant output
      Ur/e=(br-br*Z^(-2))/(1+a1*Z^(-1)+a2*Z^(-2))
      br=bo-kp. Ur(k)=br*e(k)-br*e(k-2)-a1*u(k-1)-a2*u(k-2)£»Up(k)=Kp*e(k)  /tustin approximation, continous to discrete
*/
static float   a1,a2,br;  //PR parameters of differential equations
void CalcPrCtrl(sPrCtrl* pPrCtrl)
{
	float upur_reg;
	float upu_reg;

	upur_reg = br * pPrCtrl->fSampleIn - br * pPrCtrl->fe2 
	           - a1 * pPrCtrl->fur1 - a2 * pPrCtrl->fur2;
	           
    
	if( upur_reg - pPrCtrl->fur1 < -1)
		upur_reg = pPrCtrl->fur1 - 1;
	else
	if( upur_reg - pPrCtrl->fur1 >= 1)
		upur_reg = pPrCtrl->fur1 + 1;
    
	if( upur_reg > pPrCtrl->pPara->fReOutMax )
	    upur_reg = pPrCtrl->pPara->fReOutMax;
	else
	if( upur_reg <= pPrCtrl->pPara->fReOutMin)
        upur_reg = pPrCtrl->pPara->fReOutMin;
        	           
    
    pPrCtrl->fur0 = upur_reg;
    	           
	pPrCtrl->fur2 = pPrCtrl->fur1;
    pPrCtrl->fur1 = pPrCtrl->fur0;
    
	pPrCtrl->fe2 = pPrCtrl->fe1;
    pPrCtrl->fe1 = pPrCtrl->fSampleIn;

	upu_reg = pPrCtrl->pPara->fKp * pPrCtrl->fSampleIn + pPrCtrl->fur0;

	if( upu_reg > pPrCtrl->pPara->fPrOutMax )
	    pPrCtrl->fSampleOut = pPrCtrl->pPara->fPrOutMax;
	else
	if( upu_reg < pPrCtrl->pPara->fPrOutMin)
        pPrCtrl->fSampleOut = pPrCtrl->pPara->fPrOutMin;
	else
		pPrCtrl->fSampleOut = upu_reg;
}

void InitPrCtrl(sPrCtrl* pPrCtrl)
{
    float Ts = CTRL_CLK;           //SAMPLE_TIME;
	float Kr = pPrCtrl->pPara->fKr;
	float Wc = pPrCtrl->pPara->fWc;
	float Wo = pPrCtrl->pPara->fWo;

//	b0 = ( Kp * Wo*Wo * Ts*Ts + 4 * Kr * Wc * Ts + 4 * Kp * Wc * Ts + 4 * Kp)
//	     / ( Wo*Wo * Ts*Ts + 4 * Wc * Ts + 4);
//	b1 = ( 2 * Kp * Wo*Wo * Ts*Ts - 8 * Kp ) / ( Wo*Wo * Ts*Ts + 4 * Wc * Ts + 4 );
//	b2 = ( Kp * Wo*Wo * Ts*Ts + 4 * Kp - 4 * Kr * Wc * Ts - 4 * Kp * Wc * Ts )
//		   / ( Wo*Wo * Ts*Ts + 4 * Wc * Ts + 4 );
//	br = b0 - Kp;
	a1 = ( 2 * Wo*Wo * Ts*Ts -8 ) / ( Wo*Wo * Ts*Ts + 4 * Wc * Ts + 4 );
	a2 = ( Wo*Wo * Ts*Ts - 4 * Wc * Ts + 4 ) / ( Wo*Wo * Ts*Ts + 4 * Wc * Ts + 4);
	br = ( 4 * Kr * Wc * Ts) / ( Wo*Wo * Ts*Ts + 4 * Wc * Ts + 4);
	pPrCtrl->fur1 = pPrCtrl->pPara->fReUr;
	pPrCtrl->fur2 = pPrCtrl->pPara->fReUr;
	pPrCtrl->fSampleOut = pPrCtrl->fur1;
}

