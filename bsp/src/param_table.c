//==============================================================================
// Copyright © 2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     param_table.c
// Author:       Z Shen
// Description:
//   A parameter tables that can be writed and read via Ethernet
//==============================================================================
#include "bsp.h"
#include "param_table.h"
//==============================================================================
// Constant definition
#define PKT_PROCESS_TIMEOUT	100			//ms
#define PKT_TYPE_CMD		0x8000
#define CMD_PARAM_READ		0x0000
#define CMD_PARAM_WRITE		0x0001
//==============================================================================
// Public variable definition
unsigned long param_table[PARAM_TABLE_LEN];

//==============================================================================
// Private variable definition
static unsigned int busy_sending;
unsigned int reply_buf[ETH_MAX_PKT_LEN];

//==============================================================================
// Private function declaration
static void eth_data_cb(unsigned int *p, unsigned int len);
static void eth_data_sent_cb(unsigned int *p, unsigned int len);


//==============================================================================
// Public function definition
void param_table_init(void)
{
	// Initialize internal variables
	busy_sending = 0;

	// Initialize Ehternet
	HW_eth_init();

	// Setup callback functions
	HW_eth_pkt_recv(eth_data_cb);
	HW_eth_sent(eth_data_sent_cb);
}

//==============================================================================
// Private function definition
static void eth_data_cb(unsigned int *p, unsigned int len)
{
	int i, id;
	unsigned long long tick_timeout;

	// Packet information check
	if (len%5 != 1)	return;
	if (p[0] != PKT_TYPE_CMD) return;

	tick_timeout = tick + (0.001 * PKT_PROCESS_TIMEOUT * CTRL_FREQ);

	while (busy_sending == 1) {		// Wait until last packet is sent
		if (tick > tick_timeout)	// when timeout
			return;					// direct return without processing data
	}

	// Copy all packet into reply buffer
	for(i = 0; i < len; i++)
		reply_buf[i] = p[i];

	// Start to process packet
	for (i = 1; i < len; i += 5) {
		id = reply_buf[i + 1] | reply_buf[i + 2] * 0x10000;
		if (id < PARAM_TABLE_LEN) {
			switch (p[i]) {
			case CMD_PARAM_READ:
				reply_buf[i + 3] = param_table[id] & 0xFFFF;
				reply_buf[i + 4] = param_table[id] >> 16;
				break;
			case CMD_PARAM_WRITE:
				param_table[id] = reply_buf[i + 3] | reply_buf[i + 4] * 0x10000;
				break;
			}
		}
	}

	HW_eth_send(reply_buf, len);
	busy_sending = 1;

}

static void eth_data_sent_cb(unsigned int *p, unsigned int len)
{
	busy_sending = 0;
}
