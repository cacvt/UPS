/*
File name:   meann.h (complex float N-point version )            
                    
Originator:	 Digital Control Systems Group
			 RXPE Corp. Ltd

Description: Header file containing constants, data type definitions, and 
             function prototypes for the meann .
Author:
History:
             26-12-2011		Release	Rev 1.01   
-------------------------------------------------------------------------------------
*/
#ifndef MEANN_H
#define MEANN_H

#include "constdef.h"

typedef struct
{
    complex_float   cfsum                            ;// 数据和
	complex_float*  pcfbuf                           ;// 数据缓冲区 
	complex_float*  pcfdata                          ;// 数据指针		
}sMeanReg;	

typedef struct
{
    float*          pcfSampleIn                      ;// 数据输入
    float*          pcfSampleOut                     ;// 数据输出
   	void            (*calc)()		                 ;// 对外接口
	void            (*init)()		                 ;// 对外接口
	unsigned int	uiNums	                         ;// 维数
	unsigned int    uiN                              ;// 数据点数
    float           fNinv                            ;// 1.0/N
	sMeanReg*       pfReg                            ;// 数据缓冲区
	unsigned int    uiTicCount                       ;// 隔点采样计数器
	unsigned int    uiTicN                           ;// 隔点采样周期	
}sMeanN;


void  InitMeanN(sMeanN* pMeanN);
void  CalcMeanN(sMeanN* pMeanN);  

#define MEANN_DEFAULTS { NULL,\
                         NULL,\
                         (void (*)( void))CalcMeanN,\
                         (void (*)( void))InitMeanN,\
                         30,\
                         10,\
                         1.0/10,\
                         NULL,\
                         0,\
                         1 }                  
#endif

