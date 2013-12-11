/*
 * parallel_port.c
 *
 *  Created on: 18.11.2013
 *      Author: R2D2
 */

#include "gpio_v2.h"
#include "hw_types.h"
#include "soc_AM335x.h"

#define PP_MODE_TRANSMIT 0x1
#define PP_MODE_RECEIVE  0x2

void PP_SEND_DATA32(long address, long data);
long PP_READ_DATA32(long address);
extern void PP_SET_PORT_MODE (unsigned char mode);

void PP_SEND_DATA32(long address, long data)
{
	long data_shift;

	data_shift = data>>16;

	PP_SET_PORT_MODE (PP_MODE_TRANSMIT);

    HWREG(SOC_GPIO_0_REGS + GPIO_CLEARDATAOUT) = 0xF00;
    HWREG(SOC_GPIO_0_REGS + GPIO_SETDATAOUT) = (address&0xF000)>>4;
    HWREG(SOC_GPIO_2_REGS + GPIO_CLEARDATAOUT) = 0x3FFC0;
    HWREG(SOC_GPIO_2_REGS + GPIO_SETDATAOUT) = (address&0xFFF)<<6;

    GPIOPinWrite(SOC_GPIO_2_REGS,22,0);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,1);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,0);

    HWREG(SOC_GPIO_0_REGS + GPIO_CLEARDATAOUT) = 0xF00;
    HWREG(SOC_GPIO_0_REGS + GPIO_SETDATAOUT) = (data_shift&0xF000)>>4;
    HWREG(SOC_GPIO_2_REGS + GPIO_CLEARDATAOUT) = 0x3FFC0;
    HWREG(SOC_GPIO_2_REGS + GPIO_SETDATAOUT) = (data_shift&0xFFF)<<6;

    GPIOPinWrite(SOC_GPIO_2_REGS,22,1);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,1);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,0);

    HWREG(SOC_GPIO_0_REGS + GPIO_CLEARDATAOUT) = 0xF00;
    HWREG(SOC_GPIO_0_REGS + GPIO_SETDATAOUT) = (data&0xF000)>>4;
    HWREG(SOC_GPIO_2_REGS + GPIO_CLEARDATAOUT) = 0x3FFC0;
    HWREG(SOC_GPIO_2_REGS + GPIO_SETDATAOUT) = (data&0xFFF)<<6;

    GPIOPinWrite(SOC_GPIO_2_REGS,22,1);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,1);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,0);

}

long PP_READ_DATA32(long address)
{
	long data32;
	int data16_hi;
	int data16_low;
	int data16_1;
	int data16_2;

	PP_SET_PORT_MODE (PP_MODE_TRANSMIT);

    HWREG(SOC_GPIO_0_REGS + GPIO_CLEARDATAOUT) = 0xF00;
    HWREG(SOC_GPIO_0_REGS + GPIO_SETDATAOUT) = (address&0xF000)>>4;
    HWREG(SOC_GPIO_2_REGS + GPIO_CLEARDATAOUT) = 0x3FFC0;
    HWREG(SOC_GPIO_2_REGS + GPIO_SETDATAOUT) = (address&0xFFF)<<6;

    GPIOPinWrite(SOC_GPIO_2_REGS,22,0);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,1);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,0);

    //Receiving operations

    PP_SET_PORT_MODE (PP_MODE_RECEIVE);

    GPIOPinWrite(SOC_GPIO_2_REGS,22,0);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,1);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,0);

   // while (!GPIOPinRead(SOC_GPIO_2_REGS,24));

    data16_2 = ((HWREG(SOC_GPIO_0_REGS + GPIO_DATAIN))&0xF00)<<4;
    data16_1 = ((HWREG(SOC_GPIO_2_REGS + GPIO_DATAIN))&0x3FFC0)>>6;//3FFFC

    data16_hi = data16_1 + data16_2;

    GPIOPinWrite(SOC_GPIO_2_REGS,22,0);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,1);
    GPIOPinWrite(SOC_GPIO_2_REGS,23,0);

    //while (!GPIOPinRead(SOC_GPIO_2_REGS,24));

    data16_2 = ((HWREG(SOC_GPIO_0_REGS + GPIO_DATAIN))&0xF00)<<4;
    data16_1 = ((HWREG(SOC_GPIO_2_REGS + GPIO_DATAIN))&0x3FFC0)>>6;//3FFFC

    data16_low = data16_1 + data16_2;

    data32 = (data16_hi<<16) + data16_low;

    return data32;

}

