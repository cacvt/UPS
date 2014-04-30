/*******************************************************************************
 * Copyright © 2014 Virginia Tech Center for Power Electronics Systems
 *
 * Filename:     hw_ethernet.h
 * Author:       Z Shen
 * Description:
*
 ******************************************************************************/
#ifndef __HW_ETHERNET_H__
#define __HW_ETHERNET_H__
#include "device.h"
#include "error_id.h"
//==============================================================================
// Public constant declaration
#define ETH_ACK					0x4000
#define ETH_NACK				0x2000
// For sending data out
#define ETH_SEND_CTRL			0x0000
#define ETH_SEND_DATA			0x0001

// For command to module
typedef enum {
	ETH_CMD_NONE =			0xFFFF,
	ETH_CMD_RESET =			0x8000,
	ETH_CMD_SET_IP	=		0x8001,
	ETH_CMD_SET_CTRL_PORT =	0x8002,
	ETH_CMD_SET_DATA_PORT =	0x8003,
	ETH_CMD_START_SERVER =	0x8004,
	ETH_CMD_KEEP_ALIVE =	0x8005,
	ETH_CMD_SET_MASK = 		0x8006,
	ETH_CMD_SET_GW = 		0x8007,

} ETH_CMD;

//==============================================================================
// Macro declaration
#define MAKE_IP(IP0, IP1, IP2, IP3) 	((((unsigned long)IP0) & 0xFF)	\
										| (((unsigned long)IP1) & 0xFF) * 0x100	\
										| (((unsigned long)IP2) & 0xFF) * 0x10000	\
										| (((unsigned long)IP3) & 0xFF) * 0x1000000)

//==============================================================================
// Public type declaration
typedef void (*ETH_SENT_CB)(unsigned int *p, unsigned int len);
//==============================================================================
// Public function declaration
void HW_eth_init(void);
void HW_eth_reset(void);
ETH_CMD HW_eth_get_current_cmd(void);
void HW_eth_sent(ETH_SENT_CB func);
ERR_ID HW_eth_cmd(ETH_CMD cmd, unsigned long param);
void HW_eth_task(void);
ERR_ID HW_eth_send(unsigned int *p, unsigned int len);
void HW_eth_pkt_recv(ETH_SENT_CB func);
void HW_eth_pkt_recvd(unsigned int *p);
//==============================================================================
// Public variables declaration
#ifndef __HW_ETH_C__
#endif /* __HW_ETH_C__ */



#endif /* __HW_ETHERNET_H__ */
