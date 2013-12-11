/*
 * wiz820io.c
 *
 *  Created on: 08.11.2013
 *      Author: R2D2
 */




#include "mcspi.h"
#include "gpio_v2.h"
#include "soc_AM335x.h"

#include "wiz820io.h"



#define SOC_wiz_reset SOC_GPIO_0_REGS
#define PIN_wiz_reset 13



extern unsigned int chNum;
extern unsigned int addr_spi;
extern unsigned char rw_flag;
extern unsigned char data_spi;
extern unsigned char RX_buffer;

extern unsigned char mac[6];
extern unsigned char gate[4];
extern unsigned char ip[4];








void Delay_ms( unsigned int time_ms )
{
  register unsigned int i;
  for( i=0;i<time_ms;i++ )
  {
    Delay_us(250);
    Delay_us(250);
    Delay_us(250);
    Delay_us(250);
  }
}

void Delay_us( unsigned char time_us )
{
	register unsigned char i;
	register unsigned char j;
  for( i=0;i<time_us;i++ )
  {
    for( j=0;j<200;j++ )          // 25CLK
    {

    }
  }                              // 25CLK*0.04us=1us
}

void Reset_W5200(void)
{
	GPIOPinWrite(SOC_wiz_reset, PIN_wiz_reset, 0);
	Delay_ms(10);
	GPIOPinWrite(SOC_wiz_reset, PIN_wiz_reset, 1);
	Delay_ms(1000);
}


/**
@brief	This function writes the data into W5200 registers.
*/

unsigned char IINCHIP_WRITE(unsigned int addr,unsigned char data)
{
	rw_flag = 0;

	data_spi = data;
	addr_spi = addr;
	McSPIChannelEnable(SOC_SPI_0_REGS, chNum);
	McSPICSAssert(SOC_SPI_0_REGS, chNum);
	McSPIIntEnable(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(chNum));
	McSPICSDeAssert(SOC_SPI_0_REGS, chNum);
	McSPIChannelDisable(SOC_SPI_0_REGS, chNum);

	return 1;
}
/**
@brief	This function reads the value from W5200 registers.
*/
unsigned char IINCHIP_READ(unsigned int addr)
{

	rw_flag = 1;

	addr_spi = addr;
	data_spi = 0;
	McSPIChannelEnable(SOC_SPI_0_REGS, chNum);
	McSPICSAssert(SOC_SPI_0_REGS, chNum);
	McSPIIntEnable(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(chNum));
	RX_buffer = McSPIReceiveData (SOC_SPI_0_REGS, chNum);
	McSPICSDeAssert(SOC_SPI_0_REGS, chNum);
	McSPIChannelDisable(SOC_SPI_0_REGS, chNum);

	return RX_buffer;

}

void SET_MAC(void)
{
	IINCHIP_WRITE((SHAR0 + 0),mac[0]);
	IINCHIP_WRITE((SHAR0 + 1),mac[1]);
	IINCHIP_WRITE((SHAR0 + 2),mac[2]);
	IINCHIP_WRITE((SHAR0 + 3),mac[3]);
	IINCHIP_WRITE((SHAR0 + 4),mac[4]);
	IINCHIP_WRITE((SHAR0 + 5),mac[5]);
}

void SETGATEWAY(void)
{
	IINCHIP_WRITE((GAR0 + 0),gate[0]);
	IINCHIP_WRITE((GAR0 + 1),gate[1]);
	IINCHIP_WRITE((GAR0 + 2),gate[2]);
	IINCHIP_WRITE((GAR0 + 3),gate[3]);
}

void SETSOURCEIP(void)
{
	IINCHIP_WRITE((SIPR0 + 0),ip[0]);
	IINCHIP_WRITE((SIPR0 + 1),ip[1]);
	IINCHIP_WRITE((SIPR0 + 2),ip[2]);
	IINCHIP_WRITE((SIPR0 + 3),ip[3]);
}

void ESTABLISH_SOCKET(unsigned char s, unsigned char protocol,
							   unsigned int port, unsigned char flag)
{


	close(s);

	IINCHIP_WRITE(Sn_MR(s), protocol | flag);

	IINCHIP_WRITE(Sn_PORT0(s),(unsigned char)((port & 0xff00) >> 8));
	IINCHIP_WRITE((Sn_PORT0(s) + 1),(unsigned char)(port & 0x00ff));

	IINCHIP_WRITE(Sn_CR(s), Sn_CR_OPEN); // run sockinit Sn_CR

	/* wait to process the command... */
	while( IINCHIP_READ(Sn_CR(s)) )
		;

}


void close(unsigned char s)
{

	IINCHIP_WRITE(Sn_CR(s),Sn_CR_CLOSE);

	/* wait to process the command... */
	while( IINCHIP_READ(Sn_CR(s)) )
		;
	/* ------- */
                /* all clear */
		IINCHIP_WRITE(Sn_IR(s), 0xFF);
}

unsigned char LISTEN_TCP (unsigned char s)
{
	unsigned char ret;

	if (IINCHIP_READ(Sn_SR(s)) == SOCK_INIT)
	{
		IINCHIP_WRITE(Sn_CR(s),Sn_CR_LISTEN);
		/* wait to process the command... */
		while( IINCHIP_READ(Sn_CR(s)) )
			;
		/* ------- */
		ret = 1;
	}
	else
	{
		ret = 0;

	}
	return ret;
}

/**
 Retransmittion
 **/

/**
@brief	This function sets up Retransmission time.

If there is no response from the peer or delay in response then retransmission
will be there as per RTR (Retry Time-value Register)setting
*/
void SETRETRANSMISSION(unsigned int timeout, unsigned char retry)
{
	IINCHIP_WRITE(RTR,(unsigned char)((timeout & 0xff00) >> 8));
	IINCHIP_WRITE((RTR + 1),(unsigned char)(timeout & 0x00ff));


/**
@brief	This function set the number of Retransmission.

If there is no response from the peer or delay in response then recorded time
as per RTR & RCR register seeting then time out will occur.
*/
	IINCHIP_WRITE(RCR,retry);
}

void WIZNET_INIT( unsigned char * tx_size, unsigned char * rx_size	)
{
	int i;
	int ssum,rsum;


	ssum = 0;
	rsum = 0;

	SBUFBASEADDRESS[0] = (int)(__DEF_IINCHIP_MAP_TXBUF__);		/* Set base address of Tx memory for channel #0 */
	RBUFBASEADDRESS[0] = (int)(__DEF_IINCHIP_MAP_RXBUF__);		/* Set base address of Rx memory for channel #0 */

	for (i = 0 ; i < 8; i++)       // Set the size, masking and base address of Tx & Rx memory by each channel
	{
    IINCHIP_WRITE((Sn_TXMEM_SIZE(i)),tx_size[i]);
    IINCHIP_WRITE((Sn_RXMEM_SIZE(i)),rx_size[i]);



		SSIZE[i] = (int)(0);
		RSIZE[i] = (int)(0);

		if (ssum <= 16384)
		{
         switch( tx_size[i] )
			{
			case 1:
				SSIZE[i] = (int)(1024);
				SMASK[i] = (int)(0x03FF);
				break;
			case 2:
				SSIZE[i] = (int)(2048);
				SMASK[i] = (int)(0x07FF);
				break;
			case 4:
				SSIZE[i] = (int)(4096);
				SMASK[i] = (int)(0x0FFF);
				break;
			case 8:
				SSIZE[i] = (int)(8192);
				SMASK[i] = (int)(0x1FFF);
				break;
			case 16:
				SSIZE[i] = (int)(16384);
				SMASK[i] = (int)(0x3FFF);
			break;
			}
		}

		if (rsum <= 16384)
		{
         switch( rx_size[i] )
			{
			case 1:
				RSIZE[i] = (int)(1024);
				RMASK[i] = (int)(0x03FF);
				break;
			case 2:
				RSIZE[i] = (int)(2048);
				RMASK[i] = (int)(0x07FF);
				break;
			case 4:
				RSIZE[i] = (int)(4096);
				RMASK[i] = (int)(0x0FFF);
				break;
			case 8:
				RSIZE[i] = (int)(8192);
				RMASK[i] = (int)(0x1FFF);
				break;
			case 16:
				RSIZE[i] = (int)(16384);
				RMASK[i] = (int)(0x3FFF);
				break;
			}
		}
		ssum += SSIZE[i];
		rsum += RSIZE[i];

        if (i != 0)             // Sets base address of Tx and Rx memory for channel #1,#2,#3
		{
			SBUFBASEADDRESS[i] = SBUFBASEADDRESS[i-1] + SSIZE[i-1];
			RBUFBASEADDRESS[i] = RBUFBASEADDRESS[i-1] + RSIZE[i-1];
		}

	}
}

void disconnect(unsigned char s)
{

	IINCHIP_WRITE(Sn_CR(s),Sn_CR_DISCON);

	/* wait to process the command... */
	while( IINCHIP_READ(Sn_CR(s)) )
		;
	/* ------- */
}

unsigned int getSn_RX_RSR(unsigned char s)
{
	unsigned int val=0,val1=0;
	do
	{
		val1 = IINCHIP_READ(Sn_RX_RSR0(s));
		val1 = (val1 << 8) + IINCHIP_READ(Sn_RX_RSR0(s) + 1);
      if(val1 != 0)
		{
   			val = IINCHIP_READ(Sn_RX_RSR0(s));
   			val = (val << 8) + IINCHIP_READ(Sn_RX_RSR0(s) + 1);
		}
	} while (val != val1);
   return val;
}

unsigned int WNET_RECIEVE(unsigned char s, unsigned char * buf, unsigned int len)
{
	unsigned int ret=0;

	if ( len > 0 )
	{
		recv_data_processing(s, buf, len);

		IINCHIP_WRITE(Sn_CR(s),Sn_CR_RECV);
		/* wait to process the command... */
		while( IINCHIP_READ(Sn_CR(s)));
		/* ------- */

		ret = len;
	}
	return ret;
}

void recv_data_processing(unsigned char s, unsigned char *data, unsigned int len)
{
	unsigned int ptr;
	ptr = IINCHIP_READ(Sn_RX_RD0(s));
	ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ(Sn_RX_RD0(s) + 1);

	read_data(s, (unsigned char *)ptr, data, len); // read data
	ptr += len;
	IINCHIP_WRITE(Sn_RX_RD0(s),(unsigned char)((ptr & 0xff00) >> 8));
	IINCHIP_WRITE((Sn_RX_RD0(s) + 1),(unsigned char)(ptr & 0x00ff));
}

void read_data(unsigned char s, volatile unsigned char * src, volatile unsigned char * dst,
		unsigned int len)
{
	unsigned int size;
	unsigned int src_mask;
	unsigned char * src_ptr;

	src_mask = (unsigned long)src & getIINCHIP_RxMASK(s);
	src_ptr = (unsigned char *)(getIINCHIP_RxBASE(s) + src_mask);

	if( (src_mask + len) > getIINCHIP_RxMAX(s) )
	{
		size = getIINCHIP_RxMAX(s) - src_mask;
		wiz_read_buf((unsigned long)src_ptr, (unsigned char *)dst,size);
		dst += size;
		size = len - size;
		src_ptr = (unsigned char *)(getIINCHIP_RxBASE(s));
		wiz_read_buf((unsigned long)src_ptr, (unsigned char *) dst,size);
	}
	else
	{
		wiz_read_buf((unsigned long)src_ptr, (unsigned char *) dst,len);
	}
}

unsigned int wiz_read_buf(unsigned int addr, unsigned char * buf, unsigned int len)
{
	unsigned int idx = 0;
   	unsigned char test = 0;

	McSPIChannelEnable(SOC_SPI_0_REGS, chNum);
	McSPICSAssert(SOC_SPI_0_REGS, chNum);

	McSPITransmitData(SOC_SPI_0_REGS, ((addr+idx) & 0xFF00) >> 8, chNum);
	test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	while(test != MCSPI_CH_STAT_EOT)
	{
		test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	}
	RX_buffer = McSPIReceiveData (SOC_SPI_0_REGS, chNum);

	McSPITransmitData(SOC_SPI_0_REGS, ((addr+idx) & 0x00FF), chNum);
	test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	while(test != MCSPI_CH_STAT_EOT)
	{
		test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	}
	RX_buffer = McSPIReceiveData (SOC_SPI_0_REGS, chNum);

	McSPITransmitData(SOC_SPI_0_REGS, (0x00 | ((len & 0x7F00) >> 8)), chNum);
	test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	while(test != MCSPI_CH_STAT_EOT){test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);}
	RX_buffer = McSPIReceiveData (SOC_SPI_0_REGS, chNum);

	McSPITransmitData(SOC_SPI_0_REGS, (len & 0x00FF), chNum);
	test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	while(test != MCSPI_CH_STAT_EOT)
	{
		test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	}
	RX_buffer = McSPIReceiveData (SOC_SPI_0_REGS, chNum);

	for(idx = 0; idx < len; idx++)                          // Read data in loop
	{
		McSPITransmitData(SOC_SPI_0_REGS, 0x0, chNum);
		test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
		while(test != MCSPI_CH_STAT_EOT)
		{
			test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
		}
		buf[idx] = McSPIReceiveData (SOC_SPI_0_REGS, chNum);
	}

	McSPICSDeAssert(SOC_SPI_0_REGS, chNum);
	McSPIChannelDisable(SOC_SPI_0_REGS, chNum);

	return len;
}

//************************//
//***SENDING OPERATIONS***//
//************************//

unsigned int send(unsigned char s, /*??*//*const*/ unsigned char * buf, unsigned int len, unsigned char retry)
{
	unsigned char status=0;
	unsigned int ret=0;
	unsigned int freesize=0;
	unsigned int txrd, txrd_before_send;

	if(retry) ;
	else {
		if (len > getIINCHIP_TxMAX(s)) ret = getIINCHIP_TxMAX(s); // check size not to exceed MAX size.
		else ret = len;

		// if freebuf is available, start.
		do
		{
			freesize = getSn_TX_FSR(s);
			status = IINCHIP_READ(Sn_SR(s));
			if ((status != SOCK_ESTABLISHED) && (status != SOCK_CLOSE_WAIT))
			{
				ret = 0;
				break;
			}
		} while (freesize < ret);

		// copy data
		send_data_processing(s, (unsigned char *)buf, ret);
	}


//	if(ret != 0) // error code
// 2013-07-30 wiznet fix the code to add condition "retry"
	if(retry || ret != 0)
	{
		txrd_before_send = IINCHIP_READ(Sn_TX_RD0(s));
		txrd_before_send = (txrd_before_send << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);

		IINCHIP_WRITE(Sn_CR(s),Sn_CR_SEND);

		// wait to process the command...
		while( IINCHIP_READ(Sn_CR(s)) );

		while ( (IINCHIP_READ(Sn_IR(s)) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
                {
			if(IINCHIP_READ(Sn_IR(s)) == SOCK_CLOSED)
			{
				close(s);
				return 0;
			}
		}
		IINCHIP_WRITE(Sn_IR(s), Sn_IR_SEND_OK);

		txrd = IINCHIP_READ(Sn_TX_RD0(s));
		txrd = (txrd << 8) + IINCHIP_READ(Sn_TX_RD0(s) + 1);

		if(txrd > txrd_before_send) {
			ret = txrd - txrd_before_send;
		} else {
			ret = (0xffff - txrd_before_send) + txrd + 1;
		}
	}

	return ret;
}

unsigned int getSn_TX_FSR(unsigned char s)
{
	unsigned int val=0,val1=0;
	do
	{
		val1 = IINCHIP_READ(Sn_TX_FSR0(s));
		val1 = (val1 << 8) + IINCHIP_READ(Sn_TX_FSR0(s) + 1);
      if (val1 != 0)
		{
   			val = IINCHIP_READ(Sn_TX_FSR0(s));
   			val = (val << 8) + IINCHIP_READ(Sn_TX_FSR0(s) + 1);
		}
	} while (val != val1);
   return val;
}


void send_data_processing(unsigned char s, unsigned char *data, unsigned int len)
{

	unsigned int ptr;
	ptr = IINCHIP_READ(Sn_TX_WR0(s));
	ptr = (ptr << 8) + IINCHIP_READ(Sn_TX_WR0(s) + 1);
	write_data(s, data, (unsigned char *)(ptr), len);
	ptr += len;

	IINCHIP_WRITE(Sn_TX_WR0(s),(unsigned char)((ptr & 0xff00) >> 8));
	IINCHIP_WRITE((Sn_TX_WR0(s) + 1),(unsigned char)(ptr & 0x00ff));

}

void write_data(unsigned char s, volatile unsigned char * src, volatile unsigned char * dst, unsigned int len)
{
	unsigned int size;
	unsigned int dst_mask;
	unsigned char * dst_ptr;

	dst_mask = (unsigned long)dst & getIINCHIP_TxMASK(s);
	dst_ptr = (unsigned char *)(getIINCHIP_TxBASE(s) + dst_mask);

	if (dst_mask + len > getIINCHIP_TxMAX(s))
	{
		size = getIINCHIP_TxMAX(s) - dst_mask;
		wiz_write_buf((unsigned long)dst_ptr, (unsigned char*)src, size);
		src += size;
		size = len - size;
		dst_ptr = (unsigned char *)(getIINCHIP_TxBASE(s));
		wiz_write_buf((unsigned long)dst_ptr, (unsigned char*)src, size);
	}
	else
	{
		wiz_write_buf((unsigned long)dst_ptr, (unsigned char*)src, len);
	}
}

unsigned int wiz_write_buf(unsigned int addr,unsigned char* buf,unsigned int len)
{
	unsigned int idx = 0;

   	unsigned char test = 0;

	if(len == 0)
	{

	  return 0;
	}


	McSPIChannelEnable(SOC_SPI_0_REGS, chNum);
	McSPICSAssert(SOC_SPI_0_REGS, chNum);

	McSPITransmitData(SOC_SPI_0_REGS, ((addr+idx) & 0xFF00) >> 8, chNum);
	test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	while(test != MCSPI_CH_STAT_EOT){test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);	}
	RX_buffer = McSPIReceiveData (SOC_SPI_0_REGS, chNum);

	McSPITransmitData(SOC_SPI_0_REGS, ((addr+idx) & 0x00FF), chNum);
	test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	while(test != MCSPI_CH_STAT_EOT)	{test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);}
	RX_buffer = McSPIReceiveData (SOC_SPI_0_REGS, chNum);

	McSPITransmitData(SOC_SPI_0_REGS, (0x80 | ((len & 0x7F00) >> 8)), chNum);
	test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	while(test != MCSPI_CH_STAT_EOT){test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);}
	RX_buffer = McSPIReceiveData (SOC_SPI_0_REGS, chNum);

	McSPITransmitData(SOC_SPI_0_REGS, (len & 0x00FF), chNum);
	test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
	while(test != MCSPI_CH_STAT_EOT){test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);}
	RX_buffer = McSPIReceiveData (SOC_SPI_0_REGS, chNum);

	for(idx = 0; idx < len; idx++)
	{
		McSPITransmitData(SOC_SPI_0_REGS, buf[idx], chNum);
		test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);
		while(test != MCSPI_CH_STAT_EOT){test = (McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT);}
		RX_buffer = McSPIReceiveData (SOC_SPI_0_REGS, chNum);
	}

	McSPICSDeAssert(SOC_SPI_0_REGS, chNum);
	McSPIChannelDisable(SOC_SPI_0_REGS, chNum);

	return len;
}

unsigned char windowfull_retry_cnt[MAX_SOCK_NUM];

unsigned char incr_windowfull_retry_cnt(unsigned char s)
{
  return windowfull_retry_cnt[s]++;
}

void init_windowfull_retry_cnt(unsigned char s)
{
  windowfull_retry_cnt[s] = 0;
}

unsigned int pre_sent_ptr, sent_ptr;

unsigned char getISR(unsigned char s)
{
	return I_STATUS[s];
}
void putISR(unsigned char s, unsigned char val)
{
   I_STATUS[s] = val;
}
unsigned int getIINCHIP_RxMAX(unsigned char s)
{
   return RSIZE[s];
}
unsigned int getIINCHIP_TxMAX(unsigned char s)
{
   return SSIZE[s];
}
unsigned int getIINCHIP_RxMASK(unsigned char s)
{
   return RMASK[s];
}
unsigned int getIINCHIP_TxMASK(unsigned char s)
{
   return SMASK[s];
}
unsigned int getIINCHIP_RxBASE(unsigned char s)
{
   return RBUFBASEADDRESS[s];
}
unsigned int getIINCHIP_TxBASE(unsigned char s)
{
   return SBUFBASEADDRESS[s];
}
