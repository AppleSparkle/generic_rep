/*
 * COM_SYSTEM.c
 *
 *  Created on: 04.12.2013
 *      Author: R2D2
 */

#define NULL 0x0

#include <stdlib.h>
#include <string.h>
#include "uart_irda_cir.h"
#include "hw_uart_irda_cir.h"
#include "beaglebone.h"
#include "uartStdio.h"
#include "stdio.h"

#include "gpio_v2.h"
#include "hw_types.h"
#include "soc_AM335x.h"

void UART_cont_Read(unsigned char *rxBuff, unsigned char rxByte);
void Parse_command(unsigned char *cmdBuffer);
void Execute_command(void);
extern long PP_READ_DATA32(long address);
extern void PP_SEND_DATA32(long address, long data);

unsigned char info_Byte;


extern struct
	{
		unsigned int command;
		long value1;
		long value2;
	} cmdStruct ;




void Parse_command(unsigned char *cmdBuffer)
{
	int n = 0;
	char * pch;
	unsigned char temp_Buf[32] = {0};
	unsigned char temp_Buf2[32] = {0};

	cmdStruct.command = 0;
	cmdStruct.value1 = 0;
	cmdStruct.value2 = 0;

	switch (*cmdBuffer)
	{
		case ('s'):
			{
				pch = strstr (cmdBuffer,"set");
				if(pch == NULL){
									cmdStruct.command = 0xFF;
									return;
							   }

				cmdBuffer = pch+sizeof("set");
				cmdStruct.command = 0x2;


				if(*cmdBuffer != 'x'){
										cmdStruct.command = 0xFF;
										return;
									 }
				cmdBuffer++;
				while (*cmdBuffer != ' ')
				{
					temp_Buf[n] = *cmdBuffer;
					cmdBuffer++;
					n++;
				}
				cmdStruct.value1 = atol(temp_Buf);

				n=0;
				cmdBuffer++;

				if(*cmdBuffer != 'y'){
										cmdStruct.command = 0xFF;
										return;
									 }
				cmdBuffer++;
				while (*cmdBuffer != '\r')
				{
					temp_Buf2[n] = *cmdBuffer;
					cmdBuffer++;
					n++;
				}
				cmdStruct.value2 = atol(temp_Buf2);


				break;
			}
		case ('r'):
			{
				pch = strstr (cmdBuffer,"read");
				if(pch == NULL)
				{
				}
				else
				{
					cmdStruct.command = 0x1;
					cmdStruct.value1 = 0;
					cmdStruct.value2 = 0;
					break;
				}

				pch = strstr (cmdBuffer,"resume");
				if(pch == NULL)
				{
					cmdStruct.command = 0xFF;
					return;
				}
				else
				{
					cmdStruct.command = 0x4;
					cmdStruct.value1 = 0;
					cmdStruct.value2 = 0;
					break;
				}

			}
		case ('p'):
			{
				pch = strstr (cmdBuffer,"pause");
				if(pch == NULL){
									cmdStruct.command = 0xFF;
									return;
								}

				cmdStruct.command = 0x3;
				cmdStruct.value1 = 0;
				cmdStruct.value2 = 0;
				break;
			}
		default:
			{
				cmdStruct.command = 0xFF;
				return;
			}


	}

}

void UART_cont_Read(unsigned char *rxBuff, unsigned char rxByte)
{
    unsigned int inputCount = 0u;

    /*
    ** Check whether the byte entered is not either the carriage
    ** return or space, if yes then break from the loop.
    */
    while(rxByte != '\r')
    {
    	rxByte = UARTGetc();
        UARTPutc(rxByte);

        /* Account for the backspace to allow user to edit the input */
        if(((0x7F == rxByte)||('\b' == rxByte)) && (inputCount > 0))
        {
            rxBuff--;
            inputCount--;
        }
        else
        {
            *rxBuff++ = rxByte;
            inputCount++;
            info_Byte = rxByte;
        }

    }
    /* Add the delimiting character at the end of the buffer */
    *rxBuff = rxByte;

}


void Execute_command(void)
{
	long dist;
	long aInt1 = cmdStruct.value1;
	char str1[255];
	sprintf(str1, "%d", aInt1);
	long aInt2 = cmdStruct.value2;
	char str2[255];
	sprintf(str2, "%d", aInt2);


	switch(cmdStruct.command)
	{
	case 0x1:
		{
			UARTPuts("\n\rReading... \n\r", -1);

			sprintf(str1, "%d", PP_READ_DATA32(0x1));
			UARTPuts("\n\rXpos: ", -1);
			UARTPuts(str1, -1);
			UARTPuts("\n\r", -1);

			sprintf(str1, "%d", PP_READ_DATA32(0x2));
			UARTPuts("\n\rYpos: ", -1);
			UARTPuts(str1, -1);
			UARTPuts("\n\r", -1);

			sprintf(str1, "%d", PP_READ_DATA32(0x3));
			UARTPuts("\n\rXcmd: ", -1);
			UARTPuts(str1, -1);
			UARTPuts("\n\r", -1);

			sprintf(str1, "%d", PP_READ_DATA32(0x4));
			UARTPuts("\n\rYcmd: ", -1);
			UARTPuts(str1, -1);
			UARTPuts("\n\r", -1);

			sprintf(str1, "%d", PP_READ_DATA32(0x5));
			UARTPuts("\n\rKprop (float32): ", -1);
			UARTPuts(str1, -1);
			UARTPuts("\n\r", -1);

			sprintf(str1, "%d", PP_READ_DATA32(0x6));
			UARTPuts("\n\rKinteg (float32): ", -1);
			UARTPuts(str1, -1);
			UARTPuts("\n\r", -1);

			break;
		}
	case 0x2:
		{
			UARTPuts("\n\rSetting position: X ", -1);
			UARTPuts(str1, -1);
			//PP_SEND_DATA32(0x3, aInt1);
			UARTPuts(" Y ", -1);
			UARTPuts(str2, -1);
			//PP_SEND_DATA32(0x4, aInt2);
			UARTPuts(" \n\r", -1);

			if(PP_READ_DATA32(0x7) == 0x0)
			{
				PP_SEND_DATA32(0x2, 0xFA0);	// Vel
				dist = (aInt1 > aInt2) ? (aInt1/3) : (aInt2/3);
				PP_SEND_DATA32(0x1, dist);	// Refdist
				PP_SEND_DATA32(0x3, aInt1);		// Xcmd
				PP_SEND_DATA32(0x4, aInt2);	// Ycmd


				PP_SEND_DATA32(0x7, 0xFFFFFFFF);    // Exec
				UARTPuts("\n\rSet... ", -1);
			}
			else
			{
				UARTPuts("\n\rFailed... ", -1);
			}
			break;
		}
	case 0x3:
		{
			UARTPuts("\n\rPausing... ", -1);
			GPIOPinIntDisable(SOC_GPIO_2_REGS, GPIO_INT_LINE_1, 25);
			break;
		}
	case 0x4:
		{
			UARTPuts("\n\rResuming... ", -1);
			GPIOPinIntEnable(SOC_GPIO_2_REGS, GPIO_INT_LINE_1, 25);
			break;
		}
	case 0xFF:
		{
			UARTPuts("\n\rWrong syntax!\n\r", -1);
			break;
		}
	default: {UARTPuts("Shamefuru erroru! \n\r", -1);break;}

	}
}
