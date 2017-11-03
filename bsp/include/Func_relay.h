// Filename:     Func_relay.h
// Author:       Jianghui Yu
// Description:
//   relay functions header file
//   GPIO 84 for pre-charge relay, GPIO 85 for "no-load" relay (use to be dis-charge relay), GPIO 86 for AC load/source relay, GPIO 87 for DC load/source relay, GPTIO 20 for fault relay
//   Designed for rectifier operation
//   Update relay state before every delay
//   may need HW_eth_task() if delay is too long (1s?) to avoid lost connection with computer (solved in main.c)
//   may need update HMI and update_var() if real time state is needed for communication
//==============================================================================
// Public function declaration
#define RELAY_OFF		{GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;GpioDataRegs.GPCCLEAR.bit.GPIO86 = 1;GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1;GpioDataRegs.GPCCLEAR.bit.GPIO85 = 1;ctrl_state.Relay_State=88888;DELAY_MS(50);}
                        //End of operation: all off, delay three cycles
#define RELAY_PRE       {GpioDataRegs.GPCSET.bit.GPIO84 = 1;ctrl_state.Relay_State=18888;DELAY_MS(1);}
                        //Pre-charge: only precharge relay is on
#define RELAY_REC_1     {DELAY_US(1);GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;DELAY_MS(1);GpioDataRegs.GPCSET.bit.GPIO85 = 1;DELAY_MS(1);GpioDataRegs.GPCSET.bit.GPIO86 = 1;ctrl_state.Relay_State=81188;}
                        //Rectifier operation: first delay is to wait for the switches to be turned off,
                        //then turn off the precharge relay, wait for it to turn off,
                        //then turn on "no-load" relay, wait for it to turn on,
                        //then turn on AC source relay
#define RELAY_REC_2     {DELAY_MS(1);HMI.Relay_State=ctrl_state.Relay_State;DELAY_MS(5000);GpioDataRegs.GPCSET.bit.GPIO87 = 1;ctrl_state.Relay_State=81118;}
                        //Rectifier operation: delay 10s to balance in no-load condition, then turn on DC load relay
                        //because of interrupt, the delay is longer than what is set to be. Total is about 10s
#define RELAY_FAULT_1   {GpioDataRegs.GPASET.bit.GPIO20 = 1;ctrl_state.Relay_State=81111;}
                        //turn on fault relay
#define RELAY_FAULT_2   {GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;ctrl_state.Relay_State=81118;}
                        //turn off fault relay

//#define RELAY_DIS       {DELAY_US(50000);GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1;GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;GpioDataRegs.GPCCLEAR.bit.GPIO86 = 1;ctrl_state.Relay_State=8888;DELAY_US(50000);GpioDataRegs.GPCSET.bit.GPIO85 = 1;ctrl_state.Relay_State=8188;DELAY_US(500);}
                        //Discharge: first delay is to wait for all current to decrease, second delay is to wait for three realys to turn off, then turn on only discharge relay
//#define RELAY_INV_1     {DELAY_US(1);GpioDataRegs.GPCSET.bit.GPIO84 = 1;DELAY_US(250);ctrl_state.Relay_State=1888;}
//                        //Inverter operation: first delay is to wait for the switches to be turned off,
//                        //then turn off the DC source relay, wait for it to turn off,//
//                        //then turn on precharge resistor relays, wait for them to turn on
//#define RELAY_INV_2     {DELAY_US(100000);HMI.Relay_State=ctrl_state.Relay_State;update_var();HW_eth_task();DELAY_US(100000);HW_eth_task();DELAY_US(100000);HW_eth_task();DELAY_US(100000);HW_eth_task();DELAY_US(100000);HW_eth_task();DELAY_US(100000);HW_eth_task();DELAY_US(100000);HW_eth_task();DELAY_US(100000);HW_eth_task();DELAY_US(100000);HW_eth_task();DELAY_US(100000);HW_eth_task();DELAY_US(100000);HW_eth_task();DELAY_US(100000);HW_eth_task();GpioDataRegs.GPCSET.bit.GPIO86 = 1;ctrl_state.Relay_State=1818;}
//                        //Inverter operation: delay 10s to balance in no-load condition, then turn on AC relay
//                        //because of interrupt, the delay is longer than what is set to be. 100000 is about 0.9s, total is about 10s
