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
    complex_float   cfsum                            ;// ���ݺ�
	complex_float*  pcfbuf                           ;// ���ݻ����� 
	complex_float*  pcfdata                          ;// ����ָ��		
}sMeanReg;	

typedef struct
{
    float*          pcfSampleIn                      ;// ��������
    float*          pcfSampleOut                     ;// �������
   	void            (*calc)()		                 ;// ����ӿ�
	void            (*init)()		                 ;// ����ӿ�
	unsigned int	uiNums	                         ;// ά��
	unsigned int    uiN                              ;// ���ݵ���
    float           fNinv                            ;// 1.0/N
	sMeanReg*       pfReg                            ;// ���ݻ�����
	unsigned int    uiTicCount                       ;// �������������
	unsigned int    uiTicN                           ;// �����������	
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

