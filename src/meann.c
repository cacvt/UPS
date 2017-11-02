//meann.c
#include "meann.h"

void  InitMeanN(sMeanN* pMeanN)
{
	unsigned int uiCount;
	unsigned int uiDeg;  //Î¬Êý
	for(uiDeg=0;uiDeg<pMeanN->uiNums;uiDeg++)
    {
        pMeanN->pfReg[uiDeg].cfsum.re=0.0;
	    pMeanN->pfReg[uiDeg].cfsum.im=0.0;
	    pMeanN->pfReg[uiDeg].pcfdata=pMeanN->pfReg[uiDeg].pcfbuf;

	    for(uiCount=0;uiCount<pMeanN->uiN;uiCount++)
		{
           pMeanN->pfReg[uiDeg].pcfbuf[uiCount].re=0.0;
		   pMeanN->pfReg[uiDeg].pcfbuf[uiCount].im=0.0;
		}
	}
}

void  CalcMeanN(sMeanN* pMeanN)
{
	unsigned int uiDeg;  //Î¬Êý
    pMeanN->uiTicCount++;
	if(pMeanN->uiTicCount>=pMeanN->uiTicN)
    {
        pMeanN->uiTicCount=0;
        for(uiDeg=0;uiDeg<pMeanN->uiNums;uiDeg++)
        {
            pMeanN->pfReg[uiDeg].cfsum.re+=pMeanN->pcfSampleIn[uiDeg*2];
		    pMeanN->pfReg[uiDeg].cfsum.re-=pMeanN->pfReg[uiDeg].pcfdata->re;
			pMeanN->pfReg[uiDeg].pcfdata->re=pMeanN->pcfSampleIn[uiDeg*2];
            pMeanN->pcfSampleOut[uiDeg*2]=pMeanN->pfReg[uiDeg].cfsum.re * pMeanN->fNinv;
            

            pMeanN->pfReg[uiDeg].cfsum.im+=pMeanN->pcfSampleIn[uiDeg*2+1];
		    pMeanN->pfReg[uiDeg].cfsum.im-=pMeanN->pfReg[uiDeg].pcfdata->im;
			pMeanN->pfReg[uiDeg].pcfdata->im=pMeanN->pcfSampleIn[uiDeg*2+1];
            pMeanN->pcfSampleOut[uiDeg*2+1]=pMeanN->pfReg[uiDeg].cfsum.im * pMeanN->fNinv;
            

            pMeanN->pfReg[uiDeg].pcfdata++;
		    if(pMeanN->pfReg[uiDeg].pcfdata>=pMeanN->pfReg[uiDeg].pcfbuf+pMeanN->uiN)
               pMeanN->pfReg[uiDeg].pcfdata-=pMeanN->uiN;
		}
    }
	else
	{   
		for(uiDeg=0;uiDeg<pMeanN->uiNums;uiDeg++)
		{
            pMeanN->pcfSampleOut[uiDeg*2]=pMeanN->pfReg[uiDeg].cfsum.re * pMeanN->fNinv;
		    pMeanN->pcfSampleOut[uiDeg*2+1]=pMeanN->pfReg[uiDeg].cfsum.im * pMeanN->fNinv;
		}
	}
}
