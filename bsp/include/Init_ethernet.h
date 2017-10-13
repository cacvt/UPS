//==============================================================================
// Copyright © 2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     hw_ethernet.h
// Author:       Z Shen
// Description:
//   SPI-A hardware support for Ethernet module use only.
//   Not for general purpose use.
//
// Used resource:
//   SPI-A Module
//   GPIO 16, 17, 18, 19, 32, 33
//   External interrupt 7
//
// Usage:
//   - Set ip information in HW_ethernet_addr.h
//   - Call HW_eth_init() before enableing interrupt
//   - Call HW_eth_task() regularlly in main loop
//   - Call HW_eth_send() to send data
//==============================================================================
#ifndef __HW_ETHERNET_H__
#define __HW_ETHERNET_H__
#include "device.h"
#include "error_id.h"
//==============================================================================
// Public constants definition
#define ETH_MAX_PKT_LEN				200				// word

//==============================================================================
// Public type declaration
typedef void (*ETH_CB)(unsigned int *p, unsigned int len);

//==============================================================================
// Public function declaration
void HW_eth_init(void);
void HW_eth_task(void);
ERR_ID HW_eth_send(unsigned int *p, unsigned int len);
void HW_eth_sent(ETH_CB func);
void HW_eth_pkt_recv(ETH_CB func);

#endif /* __HW_ETHERNET_H__ */
