/*
 *  ======== main.c ========
 */

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include <stdlib.h>

#include "main.h"

#include "sys_mmu.h"


long i = 0;
int ping = 0;
unsigned int chNum = 0;	//	SPI channel number

extern void UART_cont_Read(unsigned char *rxBuff, unsigned char rxByte);
extern void Parse_command(unsigned char *cmdBuffer);
extern void Execute_command(void);

void PARSE_FRAMES_SWI(void);

extern unsigned char ASCIIToDigit(unsigned char byte, unsigned int base);

//	MAC Address
unsigned char mac[6] = {0x00, 0x08, 0xDC, 0x01, 0x02, 0x03};
//	Gateway IP
unsigned char gate[4] = {192, 168, 1, 100};
//	Module IP
unsigned char ip[4] = {192, 168, 1, 88};

//	Set buffer size of socket 1 to maximum, others - to zero
unsigned char txsize[8] = {0,16,0,0,0,0,0,0};
unsigned char rxsize[8] = {0,16,0,0,0,0,0,0};

//	Huge buffers
unsigned char TrX_BUF[4096] = {0};
unsigned char RcvX_BUF[17000] = {0};
//unsigned char length_BUF[4096] = {0};
long lng = 0;
unsigned int length_test = 8;

long Frame_BUF[1048575] = {0};
long frame = 0;
unsigned long shift_frame = 0;
long frame_exec = 0;

unsigned char info_BUF[4096];
unsigned char OK_BUF[] = "OK\n\r";
unsigned char test_BUF[32] = "0\n\r";


unsigned char data_spi = 0;
unsigned int addr_spi = 0;
unsigned char RX_buffer = 0;
unsigned char rw_flag = 0;


long byteprev = 0;
long length = 0;
int test = 0;
unsigned char rcv_flag = 0;
unsigned char parseready_flag = 0;


unsigned char wiznet_conf_flag = 0;
long test_val_1 = 900;
unsigned char send_pos_data = 1;

struct
	{
		unsigned int command;
		long value1;
		long value2;
	} cmdStruct ;

SYS_MMU_ENTRY applMmuEntries[] = {
	    {(void*)0x48300000,(void*)0x48300000,0,0},  //PWM - Non bufferable| Non Cacheable
	    {(void*)0x48200000,(void*)0x48200000,0,0},  //INTCPS,MPUSS - Non bufferable| Non Cacheable
	    {(void*)0x48100000,(void*)0x48100000,0,0},  //I2C2,McSPI1,UART3,UART4,UART5, GPIO2,GPIO3,MMC1 - Non bufferable| Non Cacheable
	    {(void*)0x48000000,(void*)0x48000000,0,0},  //UART1,UART2,I2C1,McSPI0,McASP0 CFG,McASP1 CFG,DMTIMER,GPIO1 -Non bufferable| Non Cacheable
	    {(void*)0x44E00000,(void*)0x44E00000,0,0},  //Clock Module, PRM, GPIO0, UART0, I2C0, - Non bufferable| Non Cacheable
	    {(void*)0x4A300000,(void*)0x4A300000,0,0},  //PRUSS1 - Non bufferable| Non Cacheable
	    {(void*)0x4830E000,(void*)0x4830E000,0,0},  //LcdDMA
	    {(void*)0x4A100000,(void*)0x4A100000,0,0},  //CPSW - Non bufferable| Non Cacheable
	    {(void*)0xFFFFFFFF,(void*)0xFFFFFFFF,0xFF,0xFF}
};



//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\
//										MAIN													\\
//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\

void main(void)
{

	chNum = 0;

	mmuInit(applMmuEntries);
	//	Enable GPIO2 and 1 module and its clocks.
	HWREG(SOC_CM_PER_REGS + CM_PER_GPIO2_CLKCTRL)
		= (CM_PER_GPIO2_CLKCTRL_MODULEMODE & CM_PER_GPIO2_CLKCTRL_MODULEMODE_ENABLE);
	HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL)
		= (CM_PER_GPIO1_CLKCTRL_MODULEMODE & CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE);

	Pin_multiplexing_Setup();

	//	Do the necessary setup for UART0.
	Uart0_Setup();

	//	Do the necessary setup for McSPI0.
	McSPI0_Setup();

	//	Enable interrupts and start SYS/BIOS.
    BIOS_start();

}



//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\
//										IDLE													\\
//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\

void Idle_loop(void)
{
	//long k;
	//long X_array[8192];
	//long Y_array[8192];
	//long D_array[8192];


	Reset_W5200();

	IINCHIP_WRITE(0x16, 0x2);				// Interrupt reset/set
	IINCHIP_WRITE(0x4000+0x0100+0x002C, 0xE);

	SET_MAC();
	SETGATEWAY();
	SETSOURCEIP();

	SETRETRANSMISSION(6000, 3);
	WIZNET_INIT(txsize, rxsize);

	ESTABLISH_SOCKET(1, 0x01, 10005, 0x0);

	LISTEN_TCP(1);

	IINCHIP_WRITE(0x4000+0x0100+0x0002, 0xff);

	GPIOPinIntEnable(SOC_GPIO_0_REGS, GPIO_INT_LINE_1, 12);

	wiznet_conf_flag = 1;


	while(1)
	{
		if (HWREG(SOC_UART_0_REGS + UART_IER) != UART_INT_RHR_CTI)
		{
			UARTIntEnable(SOC_UART_0_REGS, UART_INT_RHR_CTI);
		}
	}
}



//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\
//										WIZNET INTERRUPT										\\
//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\

void W_INT_pin_hwi(void)
{
	int status;
	long bytecnt = 0;

	IINCHIP_WRITE(0x16, 0x0);						// Disable interrupts

	status = IINCHIP_READ(0x4000+0x0100+0x0002);	// Get pending interrupts

	IINCHIP_WRITE(0x4000+0x0100+0x0002, 0xFF);		// Clear pending interrupts (all)

	length = getSn_RX_RSR(1);						// Read length of available data

	if (length >= 4)
	{
		WNET_RECIEVE(1, RcvX_BUF + shift_frame, length);

		shift_frame = 0;

		while(1)
		{
			Frame_BUF[frame] = 0;
			Frame_BUF[frame] += RcvX_BUF[bytecnt];
			Frame_BUF[frame] += RcvX_BUF[bytecnt+1]<<8;
			Frame_BUF[frame] += RcvX_BUF[bytecnt+2]<<16;
			Frame_BUF[frame] += RcvX_BUF[bytecnt+3]<<24;

			bytecnt += 4;

			frame++;

			length -= 4;

			//if (rcv_flag == 0)
			//{
				switch (Frame_BUF[frame-1])
				{
					case (0xF0F0F0F0):
						{
							rcv_flag = 1;			// start of transmission
							frame = 0;
							break;
						}
					case (0xFFFFFFFF):
						{
							rcv_flag = 0;			// end of transmission
							Swi_post(parseframesswi);
							GPIOPinIntEnable(SOC_GPIO_2_REGS, GPIO_INT_LINE_1, 25);
							parseready_flag = 1;
							break;
						}
					default:{break;}
				}
			//}



			if(length < 4)
			{
				shift_frame = length;
				bytecnt = 0;
				break;
			}
		}

	}

	status = IINCHIP_READ(0x4000 + 0x0100 + 0x0003);

	//	check following:
	if ((status == 0x1C) || (status == 0x00))		// If disconnect happened ->
													// reestablish socket
	{
		disconnect(1);
		close(1);
		ESTABLISH_SOCKET(1, 0x01, 10005, 0x0);
		LISTEN_TCP(1);
	}

	IINCHIP_WRITE(0x16, 0x2);								// Enable interrupts of
															// wiznet device

	GPIOPinIntClear(SOC_GPIO_0_REGS, GPIO_INT_LINE_1, 12);	// Clear pin interrupt

}



//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\
//										BRES DONE INTERRUPT										\\
//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\

void BRES_DONE_HWI(void)
{

	send_pos_data = 0;

	if(PP_READ_DATA32(0x7) == 0x0)
	{
		PP_SEND_DATA32(0x2, 0xFA0);	// Vel
		if(frame_exec < frame-3)
		{
			PP_SEND_DATA32(0x3, Frame_BUF[frame_exec]);		// Xcmd
			PP_SEND_DATA32(0x4, Frame_BUF[frame_exec+1]);	// Ycmd
			PP_SEND_DATA32(0x1, Frame_BUF[frame_exec+2]);	// Refdist
			frame_exec += 3;
			PP_SEND_DATA32(0x7, 0xFFFFFFFF);    // Exec
		}
		else
		{
			frame_exec = 0;
			frame = 0;
			GPIOPinIntDisable(SOC_GPIO_2_REGS, GPIO_INT_LINE_1, 25);
			send_pos_data = 1;
		}
	}

	GPIOPinIntClear(SOC_GPIO_2_REGS, GPIO_INT_LINE_1, 25);	// Clear pin interrupt
}



//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\
//										SPI(SEND) INTERRUPT										\\
//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\

void spi_HWI(void)
{
	//	SPI interrupt routine
	//	This HWI should be enabled from IINCHIPx and wiz_buf_x functions
	//	Interrupt on TX empty

	//	Using global variables: addr_spi, data_spi

	//	Clear pending status
	McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(chNum));

	//	Switch to one of the modes: read register, write to register, read buffer, write to buffer
	switch (rw_flag)
		{
			case 0:
				{
					//	Writing to register
					SPI_transmit_check_read((addr_spi & 0xFF00) >> 8);
					SPI_transmit_check_read(addr_spi & 0x00FF);
					SPI_transmit_check_read(0x80);
					SPI_transmit_check_read(0x01);
					SPI_transmit_check_read(data_spi);
					break;
				}
			case 1:
				{
					//	Reading from register
					SPI_transmit_check_read((addr_spi & 0xFF00) >> 8);
					SPI_transmit_check_read(addr_spi & 0x00FF);
					SPI_transmit_check_read(0x00);
					SPI_transmit_check_read(0x01);
					SPI_transmit_check_read(data_spi);
					break;
				}
			case 2:
				{
					// Writing data buffer to be transmitted
					break;
				}
			case 3:
				{
					// Reading data buffer to be received
					break;
				}
			default:	{break;}
		}

	McSPIIntDisable(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(chNum));
	McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(chNum));
}

unsigned int SPI_transmit_check_read(unsigned int data)
{
	int check;

	McSPITransmitData(SOC_SPI_0_REGS, data, chNum);
	do
	{
		check = McSPIChannelStatusGet(SOC_SPI_0_REGS, chNum) & MCSPI_CH_STAT_EOT;
	}	while (check != MCSPI_CH_STAT_EOT);

	return McSPIReceiveData(SOC_SPI_0_REGS, chNum);
}



//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\
//										UART(Receive) INTERRUPT									\\
//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\

void Uart0_HWI(void)
{
	UART_cont_Read(info_BUF, 0x0);

	Parse_command(info_BUF);

	Execute_command();

	UARTIntDisable(SOC_UART_0_REGS, UART_INT_RHR_CTI);
}



//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\
//							Timer (every ~0.2 sec) INTERRUPT									\\
//////////////****************************************************************\\\\\\\\\\\\\\\\\\\\

void timr_tcp_comm_out(void)
{
	unsigned char test_buf_in[32];
	int j_in = 0;

	if (send_pos_data == 1)
	{
		ltoa(PP_READ_DATA32(0x3), test_BUF);
		while (test_BUF[j_in] != 0x0)
		{
			test_buf_in[j_in] = test_BUF[j_in];
			j_in++;
		}
		test_buf_in[j_in] = ';';
		j_in++;
		ltoa(PP_READ_DATA32(0x4), (test_BUF+j_in));
		while (test_BUF[j_in] != 0x0)
		{
			test_buf_in[j_in] = test_BUF[j_in];
			j_in++;
		}
		test_buf_in[j_in] = '\n';
		j_in++;
		test_buf_in[j_in] = '\r';

		send(1, test_buf_in, j_in, 0);
	}
}

void PARSE_FRAMES_SWI(void)
{
	int ttt = 0;

	ttt++;
}
