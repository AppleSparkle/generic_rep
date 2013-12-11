/*
 * main.h
 *
 *  Created on: 18.11.2013
 *      Author: R2D2
 */



#include <time.h>
#include <stdlib.h>
#include "uart_irda_cir.h"
#include "hw_uart_irda_cir.h"
#include "beaglebone.h"
#include "uartStdio.h"
#include "soc_AM335x.h"
#include "hw_types.h"
#include "mcspi.h"
#include "stdio.h"
#include "hw_types.h"
#include "hw_mcspi.h"
#include "hw_control_AM335x.h"
#include "hw_cm_per.h"
#include "gpio_v2.h"


#include "mux.h"



#include <string.h> /* memset */
#include <math.h>




void Idle_loop(void);
extern void Uart0_Setup(void);
extern void McSPI0_Setup(void);
extern void Pin_multiplexing_Setup(void);

extern void Reset_W5200(void);
extern void SET_MAC(void);
extern unsigned char LISTEN_TCP (unsigned char s);
extern void close(unsigned char s);
extern void ESTABLISH_SOCKET(unsigned char s, unsigned char protocol,
							   unsigned int port, unsigned char flag);


extern void WIZNET_INIT( unsigned char * tx_size, unsigned char * rx_size	);
extern void SETRETRANSMISSION(unsigned int timeout, unsigned char retry);
extern void SETSOURCEIP(void);
extern void SETGATEWAY(void);

extern void Delay_us( unsigned char time_us );
extern void Delay_ms( unsigned int time_ms );

extern void send_data_processing(unsigned char s, unsigned char *data, unsigned int len);
extern void write_data(unsigned char s, volatile unsigned char * src, volatile unsigned char * dst, unsigned int len);
extern unsigned int wiz_write_buf(unsigned int addr,unsigned char* buf,unsigned int len);
extern unsigned int getSn_TX_FSR(unsigned char s);
extern unsigned int send(unsigned char s, /*const*/ unsigned char * buf, unsigned int len, unsigned char retry);
extern void read_data(unsigned char s, volatile unsigned char * src, volatile unsigned char * dst,
		unsigned int len);
extern void recv_data_processing(unsigned char s, unsigned char *data, unsigned int len);
extern unsigned int WNET_RECIEVE(unsigned char s, unsigned char * buf, unsigned int len);
extern unsigned int getSn_RX_RSR(unsigned char s);
extern void disconnect(unsigned char s);

extern unsigned char IINCHIP_READ(unsigned int addr);
extern unsigned char IINCHIP_WRITE(unsigned int addr,unsigned char data);


extern void PP_SEND_DATA32(long address, long data);

extern void Pin_multiplexing_Setup(void);

unsigned int SPI_transmit_check_read(unsigned int data);

void Uart0_HWI(void);

extern long PP_READ_DATA32(long address);

void BRES_DONE_HWI(void);
