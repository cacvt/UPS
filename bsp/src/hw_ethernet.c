//==============================================================================
// Copyright © 2014 Virginia Tech Center for Power Electronics Systems
//
// Filename:     hw_ethernet.c
// Author:       Z Shen
// Description:
//   SPI-A hardware support for Ethernet module use only.
//   Not for general purpose use.
//==============================================================================
#define __HW_ETHERNET_C__

#include "hw_ethernet.h"
//==============================================================================
// Private constants definition
#define ETH_RECV_BUF_LEN			256			// Must be power of 2
#define ETH_SEND_QUEUE_LEN			10
#define ETH_PKT_BUF_SIZE			4
#define ETH_MAX_PKT_LEN				400
#define ETH_MAX_CMD_LEN				4
#define ETH_ACK_LEN					2
#define ETH_RECV_BUF_START			(&recv_buf[0])
#define ETH_RECV_BUF_END			(&recv_buf[ETH_RECV_BUF_LEN - 1])
#define ETH_RECV_BUF_MASK			(ETH_RECV_BUF_LEN- 1)

typedef enum {
	RM_ACK,
	RM_ACK_INFO,
	RM_DATA,
	RM_DATA_ESCAPE
} RECEVING_MODE;
//==============================================================================
// Private Type definition
struct ETH_SEND_ITEM {
	unsigned int own;
	unsigned int *p;
	unsigned int *p_sending;
	unsigned int len;
	unsigned int header[2];
	void *next;
};

struct ETH {
	RECEVING_MODE rm;
	unsigned int receiving;
	ETH_CMD cmd;
	unsigned int *r_rptr;
	unsigned int *r_wptr;
	unsigned int *cmd_wptr;
	unsigned int *pkt_wptr;
	unsigned int pkt_id;
	unsigned int *pkt_buf_start;
	unsigned int *pkt_buf_end;
	struct ETH_SEND_ITEM  *t_rptr;
	struct ETH_SEND_ITEM  *t_wptr;
	struct ETH_SEND_ITEM  *t_sent_ptr;
	ETH_SENT_CB sent;
	ETH_SENT_CB pkt_recvd;
};

//==============================================================================
// Public variable definition
struct ETH eth;
//==============================================================================
// Private global variable definition
unsigned int recv_buf[ETH_RECV_BUF_LEN] __attribute((aligned(ETH_RECV_BUF_LEN)));
unsigned int pkt_buf[ETH_PKT_BUF_SIZE][ETH_MAX_PKT_LEN + 1];
static struct ETH_SEND_ITEM send_queue[ETH_SEND_QUEUE_LEN];
unsigned int cmd_buf[ETH_MAX_CMD_LEN];
unsigned int ack_buf[ETH_ACK_LEN];
//==============================================================================
// Private function declaration
static void low_level_init(void);
static void process_ack(void);
static interrupt void spi_isr(void);
static interrupt void gpio_isr(void);

//==============================================================================
// Public  function definition
ETH_CMD HW_eth_get_current_cmd(void)
{
	return eth.cmd;
}
void HW_eth_init(void)
{
	int i;

	// Initialize internal data structures
	// Receiving pointers
	eth.r_rptr = ETH_RECV_BUF_START;
	eth.r_wptr = ETH_RECV_BUF_START;

	for (i = 0; i < ETH_RECV_BUF_LEN; i++) {
		recv_buf[i]=0xFFFF;

	}
	// Transmission queue
	for (i = 0; i < ETH_SEND_QUEUE_LEN; i++) {
		send_queue[i].own = 0;
		send_queue[i].p = 0;
		send_queue[i].p_sending = 0;
		send_queue[i].len = 0;
		send_queue[i].header[0] = 0xFFFF;
		send_queue[i].header[1] = 0x0;
		send_queue[i].next = (void*)(&send_queue[i+1]);
	}
	send_queue[ETH_SEND_QUEUE_LEN - 1].next = (void*)(send_queue);

	// Transmission pointers
	eth.t_rptr = send_queue;
	eth.t_wptr = send_queue;
	eth.t_sent_ptr = send_queue;
	// NULL call back functions
	eth.sent = 0;
	eth.pkt_recvd = 0;
	// Receiving mode is idle
	eth.rm = RM_ACK;
	eth.cmd = ETH_CMD_NONE;
	eth.pkt_id = 0;
	eth.pkt_wptr = pkt_buf[0];
	eth.pkt_buf_start = pkt_buf[0];
	eth.pkt_buf_end = &pkt_buf[0][ETH_MAX_PKT_LEN - 1];
	// Initialize hardware communication to ETH modules
	low_level_init();
}

ERR_ID HW_eth_cmd(ETH_CMD cmd, unsigned long param)
{
	unsigned int pkt_len;
	ERR_ID err;

	if (eth.cmd != ETH_CMD_NONE)
		return ERR_BUSY;

	eth.cmd = cmd;
	cmd_buf[0] = cmd;
	switch (cmd) {
	case ETH_CMD_NONE:
		return ERR_OK;

	case ETH_CMD_RESET:
		pkt_len = 1;
		break;

	case ETH_CMD_SET_IP:
		cmd_buf[1] = (param & 0xFFFF0000) >> 16;
		cmd_buf[2] = param & 0xFFFF;
		pkt_len = 3;
		break;

	case ETH_CMD_SET_CTRL_PORT:
		cmd_buf[1] = 0x0000FFFF & param;
		pkt_len = 2;
		break;

	case ETH_CMD_SET_DATA_PORT:
		cmd_buf[1] = 0x0000FFFF & param;
		pkt_len = 2;
		break;

	case ETH_CMD_START_SERVER:
		pkt_len = 1;
		break;


	case ETH_CMD_SET_MASK:
		cmd_buf[1] = (param & 0xFFFF0000) >> 16;
		cmd_buf[2] = param & 0xFFFF;
		pkt_len = 3;
		break;

	case ETH_CMD_SET_GW:
		cmd_buf[1] = (param & 0xFFFF0000) >> 16;
		cmd_buf[2] = param & 0xFFFF;
		pkt_len = 3;
		break;

	case ETH_CMD_KEEP_ALIVE:
	default:
		eth.cmd = ETH_CMD_NONE;
		return ERR_INVALID_PARAM;
	}

	err = HW_eth_send(cmd_buf, pkt_len);

	return err;
}

ERR_ID HW_eth_send(unsigned int *p, unsigned int len)
{
	if(eth.t_rptr->own == 0) {
		// Record data information
		eth.t_wptr->p = p;
		eth.t_wptr->p_sending = p;
		eth.t_wptr->len = len;
		eth.t_wptr->own = 1;
		eth.t_wptr = eth.t_rptr->next;
		// Initialize data transferring if there is nothing in queue
		if (SpiaRegs.SPIFFTX.bit.TXFFST == 0) {
			SpiaRegs.SPITXBUF = 0xFFFF;
		}		// Clear interrupt flag
		SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;

		return ERR_OK;
	}else {
		return ERR_MEM;
	}
}

void HW_eth_pkt_recv(ETH_SENT_CB func)
{
	if (func != 0)
		eth.pkt_recvd = func;
}

void HW_eth_sent(ETH_SENT_CB func)
{
	if (func != 0)
		eth.sent = func;
}

void HW_eth_task(void)
{
	unsigned int *r_ptr;

	while(eth.r_rptr != eth.r_wptr) {

		r_ptr = eth.r_rptr;

		// Re-sync if SOP is seen
		if (*r_ptr == 0xFFAA) {
			eth.rm = RM_DATA;
			eth.pkt_wptr = eth.pkt_buf_start;
		}else {
			switch (eth.rm) {
			case RM_ACK:
				eth.rm = RM_ACK_INFO;
				ack_buf[0] = *eth.r_rptr;
				break;

			case RM_ACK_INFO:
				eth.rm = RM_ACK;
				ack_buf[1] = *eth.r_rptr;
				process_ack();
				break;

			case RM_DATA:
				if (*r_ptr == 0xFF55) {
					eth.rm = RM_ACK;

					if (++eth.pkt_id >= ETH_PKT_BUF_SIZE)
						eth.pkt_id = 0;

					if (eth.pkt_recvd != 0)
						eth.pkt_recvd(eth.pkt_buf_start, eth.pkt_wptr - eth.pkt_buf_start);

					eth.pkt_buf_start = pkt_buf[eth.pkt_id];
					eth.pkt_buf_end = &pkt_buf[eth.pkt_id][ETH_MAX_PKT_LEN - 1];

				}else if (*r_ptr == 0x7FFF){
					eth.rm = RM_DATA_ESCAPE;
				}else {
					if (eth.pkt_wptr <= eth.pkt_buf_end)
						*eth.pkt_wptr++ = *eth.r_rptr;
				}
				break;

			case RM_DATA_ESCAPE:
				eth.rm = RM_DATA;
				if (eth.pkt_wptr <= eth.pkt_buf_end)
					*eth.pkt_wptr++ = (*eth.r_rptr) ^ 0x4000;
				break;
			}
		}

		eth.r_rptr = (unsigned int *)((unsigned long)ETH_RECV_BUF_START + ((unsigned long)(++eth.r_rptr) & ETH_RECV_BUF_MASK));

	}

	if(eth.t_sent_ptr != eth.t_wptr && eth.t_sent_ptr->own == 0) {
		if (eth.t_sent_ptr->p !=  cmd_buf) {
			if (eth.sent != 0)
				eth.sent(eth.t_sent_ptr->p, eth.t_sent_ptr->len);
		}
		eth.t_sent_ptr = eth.t_sent_ptr->next;
	}
}
/*******************************************************************************
 * Private function definition
 ******************************************************************************/
static void process_ack(void)
{
	eth.cmd = ETH_CMD_NONE;
}


static void low_level_init(void)
{
	   EALLOW;

	    // Enable clock for SPI-A module
	    SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;

	    // Put SPI into reset
	    SpiaRegs.SPICCR.bit.SPISWRESET = 0x0;

		// Setup GPIO pins for SPIA
		//  CS ---------> GPIO19
		//  CLK --------> GPIO18
		//  MISO -------> GPIO17
		//  MOSI -------> GPIO16
	    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0x1;	// Set GPIO pins as SPI IOs
	    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0x1;
	    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0x1;
	    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0x1;


	    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 0x3;	// Set IO pins qualification as
	    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 0x3;	// asynchronous
	    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 0x3;
	    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 0x3;


	    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0x1;	// Disable pull-up resistors to save
	    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0x1;	// power
	    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0x1;
	    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0x1;

	    // Setup SPI module parameters
	    SpiaRegs.SPICCR.bit.SPICHAR = 0xF;		// 16b character
	    SpiaRegs.SPICTL.bit.CLK_PHASE = 0x1;	// CLK signal is delayed by half
												// cycle
	    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0x0;	// Data out @ rising edge
	     	 	 	 	 	 	 	 	 	 	// Data in @ falling edge
		SpiaRegs.SPICTL.bit.MASTER_SLAVE = 0x1;	// Master mode
		SpiaRegs.SPICTL.bit.TALK = 0x1;			// Enable talk
		SpiaRegs.SPIFFTX.bit.TXFFIENA = 1;		// Enable Transmit interrupt
		SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;		// Enable FIFO
		SpiaRegs.SPIFFCT.bit.TXDLY = 1;			// One clock cycle between words
		SpiaRegs.SPIBRR =0x000A;				// SPI_CLK = LOSPCLK/11 = 18.2 MHz

		SpiaRegs.SPIPRI.bit.FREE = 1;           // Free run mode. Breakpoints
												//   don't disturb transmission

		// Relinquish SPI from Reset
		SpiaRegs.SPICCR.bit.SPISWRESET =0x1;

		// Set GPIO32 as external interrupt source
	    GpioIntRegs.GPIOXINT4SEL.all = 0;                       // Use GPIO32 as XINT4 to trigger receviing
	    XIntruptRegs.XINT4CR.bit.POLARITY = 0;                  // Interrupt on falling edge
	    XIntruptRegs.XINT4CR.bit.ENABLE = 1;                    // Interrupt is enabled

		// Setup SPI interrupt
		PieVectTable.SPITXINTA = spi_isr;		// Setup interrupt vector
	    PieCtrlRegs.PIEIER6.bit.INTx2 = 1;		// Enable SPI TX interrupt in PIE

		// Setup GPIO interrupt
		PieVectTable.XINT4 = gpio_isr;			// Setup interrupt vector
	    PieCtrlRegs.PIEIER12.bit.INTx2 = 1;		// Enable GPIO interrupt in PIE

	    // Enable global interrupt level
	    IER |= M_INT6 | M_INT12;				// Enable global interrupt level for SPI interrupt

		//--------------------------------------------------------------------------
		// Configure GPIO as external interrupthh
		EDIS;

}

static interrupt void spi_isr(void)
{
	int i, l, len, r_len;
	unsigned int tmp;
	unsigned int *ps;

	// Read received data
	r_len = SpiaRegs.SPIFFRX.bit.RXFFST;

	for (i = 0; i < r_len; i++) {
		tmp = SpiaRegs.SPIRXBUF;
		if (tmp != 0xFFFF)
			*eth.r_wptr++ =  tmp;
		eth.r_wptr = (unsigned int *)((unsigned long)ETH_RECV_BUF_START + ((unsigned long)eth.r_wptr & ETH_RECV_BUF_MASK));
	}

	eth.receiving = !GpioDataRegs.GPBDAT.bit.GPIO32;

	if (eth.t_rptr->own == 1) {				// If there is more data to send
		if (eth.t_rptr->len == 0) {
			eth.t_rptr->own = 0;
			eth.t_rptr = eth.t_rptr->next;
			SpiaRegs.SPITXBUF =  0xFFFF;
		}else {
			len = eth.t_rptr->len;
			ps = eth.t_rptr->p_sending;

			l = __min(len, 16);

			for (i = 0; i < l; i++)
				SpiaRegs.SPITXBUF = *(ps++);

			len -= l;
			eth.t_rptr->p_sending = ps;
			eth.t_rptr->len = len;
		}
		SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;
	}else if(eth.receiving != 0) {			// If there is more data to receive
		for (i = 0; i < 16; i++) {
			SpiaRegs.SPITXBUF = 0xFFFF;
		}
		SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;
	}else {									// Send IDLE as last character
		SpiaRegs.SPITXBUF = 0xFFFF;
	}

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;	// Acknowledge interrupt
}

static interrupt void gpio_isr(void)
{
	eth.receiving = 1;
	SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;	// Acknowledge interrupt
}
