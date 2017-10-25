/* =================================================================================
File name:       PID_REG3.H                     
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type definitions, and 
function prototypes for the PIDREG3.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
------------------------------------------------------------------------------*/

typedef struct {  float32  Ref;   		// Input: Reference input 
				  float32  Fdb;   		// Input: Feedback input 
				  float32  Err;			// Variable: Error 
				  float32  Kp;			// Parameter: Proportional gain
				  float32  Up;			// Variable: Proportional output 
				  float32  Ui;			// Variable: Integral output 
				  float32  Ud;			// Variable: Derivative output	
				  float32  OutPreSat;	// Variable: Pre-saturated output
				  float32  OutMax;		// Parameter: Maximum output 
				  float32  OutMin;		// Parameter: Minimum output
				  float32  Out;   		// Output: PID output 
				  float32  SatErr;		// Variable: Saturated difference
				  float32  Ki;			// Parameter: Integral gain
				  float32  Kc;			// Parameter: Integral correction gain
				  float32  Kd; 			// Parameter: Derivative gain 
				  float32  Up1;			// History: Previous proportional output
		 	 	  void  (*calc)();	  	// Pointer to calculation function
				 } PIDREG3;	            

typedef PIDREG3 *PIDREG3_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/                     
#define PIDREG1_DEFAULTS { IBa0, \
                           0, \
                           0, \
                           0.0017, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0.4, \
                           -0.4, \
                           0, \
                           0, \
                           45000*CTRL_CLK, \
                           0.5, \
                           0, \
                           0, \
              			  (void (*)(Uint32))pid_reg3_calc }
//Kp=0.0017, Ki=76.34=0.0017*45000
//Output limit: +/- 0.2

#define PIDREG2_DEFAULTS { 0, \
                           0, \
                           0, \
                           0.0075, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0.05, \
                           -0.05, \
                           0, \
                           0, \
                           66*CTRL_CLK, \
                           0.5, \
                           0, \
                           0, \
              			  (void (*)(Uint32))pid_reg3_calc }
//Kp=0.0075, Ki=0.50=0.0075*66
//Output limit: +/- 0.1


/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pid_reg3_calc(PIDREG3_handle);
