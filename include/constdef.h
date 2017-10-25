//constdef.h
#ifndef CONSTDEF_H
#define CONSTDEF_H


#if defined  NULL
     #undef NULL
#endif     
#define NULL   0

    
typedef struct
{
	float   re;
	float   im;
}complex_float;	   	 

#define COMPLEX_DEFAULTS  {0,0}
#define SAMPLE_NUMS     400
#define  PI            3.1415926535897932384626433832795
#define  PI2           (PI*2) 
#define  DIV1_3        0.33333333333333333333333333333333  
#define  DIV1_6        0.16666666666666666666666666666667
#define  DIV1_SQRT3    (1.0/1.7320508075688772935274463415059)
#define  SQRT2OVER3    0.8164965809277260327324
#define  SQRT3_2 	   0.8660254037844386467637

#define     WORD_H(x)  ((x)>>8)
#define     WORD_L(x)  ((x)&0xFF)

#define     DWORD_LL(x)  ((x)&0xFF)
#define     DWORD_LH(x)  (((x)>>8)&0xFF)
#define     DWORD_HL(x)  (((x)>>16)&0xFF)
#define     DWORD_HH(x)  (((x)>>24)&0xFF)

#define BIT_SIZE        32                 //32λϵͳ
#define BYTE_INT        (BIT_SIZE/8)       //4Byte
#define BIT_LEN         5                  //32=(2<<5)
#define BIT_MASK        (BIT_SIZE-1)       //31=(2<<5)-1
#define BIT_MASK16      15                 //0x0F
#define BIT_MASK8       7                  //0x07

#define DWORD_OR_WORD   1
    

   

#define    COS_PI2_3  -0.5
#define    SIN_PI2_3  -0.86602540378443864676372317075294
#define    COSPI2_3   -0.5
#define    SINPI2_3   0.86602540378443864676372317075294

    
#define BIT0    0x0001
#define BIT1    0x0002
#define BIT2    0x0004
#define BIT3    0x0008
#define BIT4    0x0010
#define BIT5    0x0020
#define BIT6    0x0040
#define BIT7    0x0080
#define BIT8    0x0100
#define BIT9    0x0200
#define BIT10   0x0400
#define BIT11   0x0800
#define BIT12   0x1000
#define BIT13   0x2000
#define BIT14   0x4000
#define BIT15   0x8000

#endif

