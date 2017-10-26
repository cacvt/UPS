
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

#include <Init_ethernet.h>
#include "HW_ethernet_addr.h"

//==============================================================================
// Private constants definition
#define ETH_CMD_TIMEOUT				5000L			// ms
#define ETH_RECV_BUF_LEN			2048			// Must be power of 2
#define ETH_SEND_QUEUE_LEN			10				// item
#define ETH_MAX_CMD_LEN				4				// word
#define ETH_ACK_LEN					2				// word

// Buffer address
#define ETH_RX_BUF_START			(&recv_buf[0])
#define ETH_RX_BUF_END				(&recv_buf[ETH_RECV_BUF_LEN - 1])
#define ETH_RECV_BUF_MASK			(ETH_RECV_BUF_LEN- 1)
#define ETH_PKT_BUF_START			pkt_buf
#define ETH_PKT_BUF_END				(&pkt_buf[ETH_MAX_PKT_LEN])

// Submodule interrupt line
#define ETH_RECV_INT_PIN			(GpioDataRegs.GPBDAT.bit.GPIO32)
#define ETH_RESET_PIN				(GpioDataRegs.GPBDAT.bit.GPIO33)

// Bits for CMD to submodule
#define ETH_CMD_ACK					0x4000
#define ETH_CMD_NACK				0x2000

// Receiving state
enum RECEVING_MODE{
	RM_ACK,						// Command acknowledgement word
	RM_ACK_DATA,				// Command acknowledgement data
	RM_DATA,					// Receiving data
	RM_DATA_ESCAPE,				// Next data is escaped
};

// Command words
enum ETH_CMD{
	ETH_CMD_NONE 			= 0xFFFF,
	ETH_CMD_RESET 			= 0x8000,
	ETH_CMD_SET_IP			= 0x8001,
	ETH_CMD_SET_MASK		= 0x8002,
	ETH_CMD_SET_GW			= 0x8003,
	ETH_CMD_SET_PORT		= 0x8004,
	ETH_CMD_START_SERVER	= 0x8005,
};

//==============================================================================
// Private Type definition
struct ETH_SEND_ITEM {
	unsigned int own;
	unsigned int *p;
	unsigned int *p_sending;
	int len;
	int len_orig;
	void *next;
};

struct ETH {
	enum RECEVING_MODE rm;
	enum ETH_CMD cmd;
	unsigned int spi_idle;
	unsigned int *r_rptr;
	unsigned int *r_wptr;
	unsigned int *pkt_wptr;
	struct ETH_SEND_ITEM  *t_rptr;
	struct ETH_SEND_ITEM  *t_wptr;
	ETH_CB sent;
	ETH_CB pkt_recvd;
};

//==============================================================================
// Macro definition
#define MAKE_IP(IP0, IP1, IP2, IP3) 	((((unsigned long)IP0) & 0xFF)	\
										| (((unsigned long)IP1) & 0xFF) * 0x100	\
										| (((unsigned long)IP2) & 0xFF) * 0x10000	\
										| (((unsigned long)IP3) & 0xFF) * 0x1000000)

#define PREV_RX_BUF_PTR(p)	((unsigned int *)((unsigned long)ETH_RX_BUF_START + ((unsigned long)(p - 1) & ETH_RECV_BUF_MASK)))
#define NEXT_RX_BUF_PTR(p)	((unsigned int *)((unsigned long)ETH_RX_BUF_START + ((unsigned long)(p + 1) & ETH_RECV_BUF_MASK)))

//==============================================================================
// Private global variable definition
static struct ETH eth;
unsigned int recv_buf[ETH_RECV_BUF_LEN] __attribute((aligned(ETH_RECV_BUF_LEN)));
unsigned int pkt_buf[ETH_MAX_PKT_LEN + 1];
static struct ETH_SEND_ITEM send_queue[ETH_SEND_QUEUE_LEN];
unsigned int cmd_buf[ETH_MAX_CMD_LEN];
unsigned int ack_buf[ETH_ACK_LEN];

//==============================================================================
// Private function declaration
static void 	low_level_init(void);
static void 	process_ack(void);
static ERR_ID 	exec_cmd(enum ETH_CMD cmd, unsigned long param);
static ERR_ID 	send_cmd(enum ETH_CMD cmd, unsigned long param);
static void		set_addresses(void);

//==============================================================================
// ISR function declaration
interrupt static void 	spi_isr(void);
interrupt static void 	gpio_isr(void);

//==============================================================================
// Public  function definition
void HW_eth_init(void)
{
	int i;

	//--------------------------------------------------------------------------
	// Initialize internal variables & data structures

	// Receiving pointers
	eth.r_rptr = ETH_RX_BUF_START;
	eth.r_wptr = ETH_RX_BUF_START;

	// Transmission queue
	for (i = 0; i < ETH_SEND_QUEUE_LEN; i++) {
		send_queue[i].own = 0;
		send_queue[i].p = 0;
		send_queue[i].p_sending = 0;
		send_queue[i].len = 0;
		send_queue[i].len_orig = 0;
		send_queue[i].next = (void*)(&send_queue[i+1]);
	}
	send_queue[ETH_SEND_QUEUE_LEN - 1].next = (void*)(send_queue);

	// Transmission pointers
	eth.t_rptr = send_queue;
	eth.t_wptr = send_queue;

	// NULL call back functions
	eth.sent = 0;
	eth.pkt_recvd = 0;

	// Receiving mode is idle
	eth.rm = RM_ACK;

	// No command is executing
	eth.cmd = ETH_CMD_NONE;

	// Set spi status to idle
	eth.spi_idle = 1;

	//--------------------------------------------------------------------------
	// Initialize hardware
	low_level_init();

	//--------------------------------------------------------------------------
	// Setup network parameters
	set_addresses();
}


void HW_eth_pkt_recv(ETH_CB func)
{
	eth.pkt_recvd = func;
}

ERR_ID HW_eth_send(unsigned int *p, unsigned int len)
{
	if(eth.t_wptr->own == 0) {
		eth.t_wptr->p = p;							// Record data information
		eth.t_wptr->p_sending = p;
		eth.t_wptr->len = len;
		eth.t_wptr->len_orig = len;
		eth.t_wptr->own = 1;						// Give descriptor to ISR
		eth.t_wptr = eth.t_wptr->next;				// Move to the next descriptor

		if (eth.spi_idle == 1)						// Clear interrupt flag if there is nothing being sent
			SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;
		return ERR_OK;
	}else {											// Out of queue space
		return ERR_MEM;
	}
}

void HW_eth_sent(ETH_CB func)
{
	eth.sent = func;
}

void HW_eth_task(void)
{
	int i;
	unsigned int *r_ptr;

	i = 0;

	while(eth.r_rptr != eth.r_wptr && i++ <ETH_MAX_PKT_LEN) {	// Process at most ETH_MAX_PKT_LEN words a time to limit function call time

		r_ptr = eth.r_rptr;

		if (*r_ptr == 0xFFFA) {									// Re-sync if SOP is seen
			eth.rm = RM_DATA;
			eth.pkt_wptr = ETH_PKT_BUF_START;
		}else {
			switch (eth.rm) {
			case RM_ACK:
				eth.rm = RM_ACK_DATA;
				ack_buf[0] = *r_ptr;
				break;

			case RM_ACK_DATA:
				eth.rm = RM_ACK;
				ack_buf[1] = *r_ptr;
				process_ack();
				break;

			case RM_DATA:
				if (*r_ptr == 0xFFF5) {
					eth.rm = RM_ACK;

					if (eth.pkt_recvd != 0)
						eth.pkt_recvd(ETH_PKT_BUF_START, eth.pkt_wptr - ETH_PKT_BUF_START);

				}else if (*r_ptr == 0x7FFF){
					eth.rm = RM_DATA_ESCAPE;
				}else {
					if (eth.pkt_wptr <= ETH_PKT_BUF_END)
						*eth.pkt_wptr++ = *eth.r_rptr;
				}
				break;

			case RM_DATA_ESCAPE:
				eth.rm = RM_DATA;
				if (eth.pkt_wptr <= ETH_PKT_BUF_END)
					*eth.pkt_wptr++ = (*eth.r_rptr) ^ 0x4000;
				break;
			}
		}

		eth.r_rptr = NEXT_RX_BUF_PTR(eth.r_rptr);
	}

	if (eth.spi_idle == 1 && SpiaRegs.SPIFFRX.bit.RXFFST > 0)	// If SPI is idle but there is data in receiving buffer
		SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;					// Trigger SPI interrupt again
}

//==============================================================================
// Private function definitions

static ERR_ID exec_cmd(enum ETH_CMD cmd, unsigned long param)
{
	ERR_ID err;
	int tick;

	err = send_cmd(cmd, param);

	if (err == ERR_OK) {

		tick = 0;

		while(eth.cmd != ETH_CMD_NONE && err == ERR_OK) {
			HW_eth_task();
			DELAY_US(10000L);
			if (tick++ > (ETH_CMD_TIMEOUT/10))
				err = ERR_TIMEOUT;
		}
	}
	return err;
}

static ERR_ID send_cmd(enum ETH_CMD cmd, unsigned long param)
{
	ERR_ID err;
	unsigned int pkt_len;

	if (eth.cmd != ETH_CMD_NONE)
		return ERR_BUSY;

	eth.cmd = cmd;
	cmd_buf[0] = cmd;
	switch (cmd) {
	case ETH_CMD_NONE:
		return ERR_OK;

	case ETH_CMD_RESET:
		ETH_RESET_PIN = 1;
		DELAY_US(10000);
		ETH_RESET_PIN = 0;
		return ERR_OK;

	case ETH_CMD_SET_IP:
		cmd_buf[1] = (param & 0xFFFF0000) >> 16;
		cmd_buf[2] = param & 0xFFFF;
		pkt_len = 3;
		break;

	case ETH_CMD_SET_PORT:
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

	default:
		eth.cmd = ETH_CMD_NONE;
		return ERR_INVALID_PARAM;
	}

	err = HW_eth_send(cmd_buf, pkt_len);

	return err;
}

static void low_level_init(void)
{
	EALLOW;

	// Enable clock for SPI-A module
	SysCtrlRegs.LOSPCP.bit.LSPCLK = 0;
	SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;

	// Put SPI into reset
	SpiaRegs.SPICCR.bit.SPISWRESET = 0x0;

	// Setup GPIO pins for SPI-A
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
	SpiaRegs.SPIBRR =0x0007;				// SPI_CLK = LOSPCLK/8 = 25 MHz

	SpiaRegs.SPIPRI.bit.FREE = 1;           // Free run mode. Breakpoints
											//   don't disturb transmission
	// Relinquish SPI from Reset
	SpiaRegs.SPICCR.bit.SPISWRESET =0x1;
	//--------------------------------------------------------------------------
	// Configure GPIO 33 as output to control submodule reset
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0x0;	// Set as IO
	GpioDataRegs.GPBDAT.bit.GPIO33 = 0x0;	// Set state to low
	GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0x1;	// Set as ouput
	GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0x1;	// Disable pull-up resistor

	//--------------------------------------------------------------------------
	// Configure GPIO as external interrupthh
	// Set GPIO32 as external interrupt source
	GpioIntRegs.GPIOXINT7SEL.all = 0;                       // Use GPIO32 as XINT4 to trigger receviing
	XIntruptRegs.XINT7CR.bit.POLARITY = 0;                  // Interrupt on falling edge
	XIntruptRegs.XINT7CR.bit.ENABLE = 1;                    // Interrupt is enabled

	//--------------------------------------------------------------------------
	// Interrupt setup
	// Setup SPI interrupt
	PieVectTable.SPITXINTA = spi_isr;		// Setup interrupt vector
	PieCtrlRegs.PIEIER6.bit.INTx2 = 1;		// Enable SPI TX interrupt in PIE

	// Setup GPIO interrupt
	PieVectTable.XINT7 = gpio_isr;			// Setup interrupt vector
	PieCtrlRegs.PIEIER12.bit.INTx5 = 1;		// Enable GPIO interrupt in PIE

	// Enable global interrupt level
	IER |= M_INT6 | M_INT12;				// Enable global interrupt level for SPI interrupt

	EDIS;

}

static void process_ack(void)
{
	if (ack_buf[0] == (eth.cmd | ETH_CMD_ACK))
		eth.cmd = ETH_CMD_NONE;
}

static void set_addresses(void)
{
	unsigned int ST1_o, IER_o, pie_ier_6_o, pie_ier_12_o;

	// Disable interrupts
	ST1_o = _disable_interrupts();

	// Preserve interrupt configuration
	IER_o = IER;
	pie_ier_6_o = PieCtrlRegs.PIEIER6.all;
	pie_ier_12_o = PieCtrlRegs.PIEIER12.all;

	// Enable only ethernet related interrupt
	PieCtrlRegs.PIEIER6.all = 0x2;
	PieCtrlRegs.PIEIER12.all = 0x10;
	IER = M_INT6 | M_INT12;

	// Enable global interrupt
	EINT;

SUBMODULE_RESET:
	eth.cmd = ETH_CMD_NONE;		// Clear pending command to be able to restart

	if (ERR_OK != exec_cmd(ETH_CMD_RESET, 0))	goto SUBMODULE_RESET;
	if (ERR_OK != exec_cmd(ETH_CMD_SET_MASK, MAKE_IP(NETMASK0, NETMASK1, NETMASK2, NETMASK3)))	goto SUBMODULE_RESET;
	if (ERR_OK != exec_cmd(ETH_CMD_SET_GW, MAKE_IP(GATEWAY0, GATEWAY1, GATEWAY2, GATEWAY3)))	goto SUBMODULE_RESET;
	if (ERR_OK != exec_cmd(ETH_CMD_SET_IP, MAKE_IP(IPADDR0, IPADDR1, IPADDR2, IPADDR3)))	goto SUBMODULE_RESET;
	if (ERR_OK != exec_cmd(ETH_CMD_SET_PORT, TCP_PORT))	goto SUBMODULE_RESET;
	if (ERR_OK != exec_cmd(ETH_CMD_START_SERVER, 0))	goto SUBMODULE_RESET;

	// Restore interrupt configuration
	DINT;
	IER = IER_o;
	PieCtrlRegs.PIEIER6.all = pie_ier_6_o;
	PieCtrlRegs.PIEIER12.all = pie_ier_12_o;

	// Restore interrupt status
	_restore_interrupts(ST1_o);
}

//==============================================================================
// ISRs
interrupt static void spi_isr(void)
{
	int i, l;
	int buf_len;
	int has_data;
	unsigned int tmp;
	unsigned int last;
	unsigned int *ps;
	unsigned int *last_read;

	PieCtrlRegs.PIEIER12.bit.INTx2 = 0;		// Mask receiving interrupt;
	eth.spi_idle = 0;
	EINT;									// Enable interrupt for PWM

	last = 0;
	has_data = eth.t_rptr->own;
	//--------------------------------------------------------------------------
	// Receive data
	last_read = PREV_RX_BUF_PTR(eth.r_rptr);			// Get pointer to the previous word read
														//   1 word space is put between read and write pointer
	if (!has_data && eth.r_wptr == last_read) {			// Check rx buffer space before proceeding
		last = 1;										//   Stop if buffer is full and no data to send
		goto ISR_RETURN;
	}

	while(SpiaRegs.SPIFFRX.bit.RXFFST > 0) {			// If there are data in the buffer

		tmp = SpiaRegs.SPIRXBUF;						// Read one word

		if (tmp != 0xFFFF) {							// Store only none IDLE word

			*eth.r_wptr =  tmp;
			eth.r_wptr = NEXT_RX_BUF_PTR(eth.r_wptr);	// Increase write pointer

			if (!has_data && eth.r_wptr == last_read) {	// Stop receiving if loop buffer is full and no data to send
				last = 1;
				goto ISR_RETURN;
			}


		}
	}

	//--------------------------------------------------------------------------
	// Send data
	//   Put at most 15 words into FIFO, considering one more word in SPIDAT register
	//   16 words may be received into FIFO before next ISR call
	if (has_data) {									// If there is more data to send
		if (eth.t_rptr->len == 0) {								// Current packet is sent

			if (eth.t_rptr->p !=  cmd_buf) {					// This is data packet

				if (eth.sent != 0)								// Data sent callback function
					eth.sent(eth.t_rptr->p, eth.t_rptr->len_orig);

				SpiaRegs.SPITXBUF =  0xFFF5;					// Add EOP
			}
			SpiaRegs.SPITXBUF =  0xFFFF;						// Send IDLE as last WORD

			eth.t_rptr->own = 0;								// Give descriptor back to queue
			eth.t_rptr = eth.t_rptr->next;						// Move to the next descriptor
		}else {
			// Send at most 14 words. the last FIFO space is reserved for word escaping expansion
			if (eth.t_rptr->len == eth.t_rptr->len_orig			// a new data packet
					&& eth.t_rptr->p !=  cmd_buf ){
					buf_len = 11;								// 3 words are sent below, be able to send 11 more
					SpiaRegs.SPITXBUF =  0xFFFF;				// IDLE
					SpiaRegs.SPITXBUF =  0x0;					// Channel selection
					SpiaRegs.SPITXBUF =  0xFFFA;				// SOP

			}else {
				buf_len = 14;									// All 14 spaces are available
			}

			ps = eth.t_rptr->p_sending;						// Get pointer & length of data to send
			l = eth.t_rptr->len;

			for (i = 0; i < buf_len; i++) {						// Send at most buf_len words
				if((*ps & 0xFFF0) != 0xFFF0 && *ps != 0x7FFF) {	//Normal word
					SpiaRegs.SPITXBUF = *(ps++);
				}else {											// Word to escape
					SpiaRegs.SPITXBUF = 0x7FFF;
					SpiaRegs.SPITXBUF = 0x4000 ^ *(ps++);
					i++;
				}
				if(--l == 0)									// stop if this is last word
					break;
			}

			eth.t_rptr->p_sending = ps;							// Record new pointer & length
			eth.t_rptr->len = l;
		}

		SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;				// Clear interrupt flag for next interrupt

	}else if(ETH_RECV_INT_PIN == 0) {			// If there is more data to receive

		for (i = 0; i < 15; i++)				// Read 15 words
			SpiaRegs.SPITXBUF = 0xFFFF;

		SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;	// Clear interrupt flag for next interrupt
	}else {
		last = 1;
	}


ISR_RETURN:
	DINT;										// Disable interrupt

	if (last == 1)								// Set idle flag if this is the last ISR
		eth.spi_idle = 1;

	PieCtrlRegs.PIEIER12.bit.INTx2 = 1;			// Un-mask receiving interrupt;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;		// Acknowledge interrupt
}

interrupt static void gpio_isr(void)
{
	if (eth.spi_idle == 1)
		SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;	// Acknowledge interrupt
}
