/*
 * systems_setup.c
 *
 *  Created on: 18.11.2013
 *      Author: R2D2
 */

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

#include "interrupt.h"

#include "mux.h"

#define PINMUXMODE_1	0x00
#define PINMUXMODE_2	0x01
#define PINMUXMODE_3	0x02
#define PINMUXMODE_4	0x03
#define PINMUXMODE_5	0x04
#define PINMUXMODE_6	0x05
#define PINMUXMODE_7	0x06
#define PINMUXMODE_8	0x07

#define MCSPI_OUT_FREQ                   (2000000u)
#define MCSPI_IN_CLK                     (48000000u)


void Uart0_Setup(void);
void McSPI0_Setup(void);
void Pin_multiplexing_Setup(void);
void PP_SET_PORT_MODE (unsigned char mode);

extern unsigned int chNum;
unsigned int divisorValue = 0;


void Uart0_Setup(void)
{

	UART0ModuleClkConfig();

    /* Performing the Pin Multiplexing for UART0 instance. */
    UARTPinMuxSetup(0);

    /* Performing a module reset. */
    UARTModuleReset(SOC_UART_0_REGS);

    /* Performing Baud Rate settings. */


    /* Computing the Divisor Value. */
    divisorValue = UARTDivisorValCompute(48000000,
    									 115200,
                                         UART16x_OPER_MODE,
                                         UART_MIR_OVERSAMPLING_RATE_42);

    /* Programming the Divisor Latches. */
    UARTDivisorLatchWrite(SOC_UART_0_REGS, divisorValue);

    /* Switching to Configuration Mode B. */
    UARTRegConfigModeEnable(SOC_UART_0_REGS, UART_REG_CONFIG_MODE_B);

    /* Programming the Line Characteristics. */
    UARTLineCharacConfig(SOC_UART_0_REGS,
                         (UART_FRAME_WORD_LENGTH_8 | UART_FRAME_NUM_STB_1),
                         UART_PARITY_NONE);

    /* Disabling write access to Divisor Latches. */
    UARTDivisorLatchDisable(SOC_UART_0_REGS);

    /* Disabling Break Control. */
    UARTBreakCtl(SOC_UART_0_REGS, UART_BREAK_COND_DISABLE);

    /* Switching to UART16x operating mode. */
    UARTOperatingModeSelect(SOC_UART_0_REGS, UART16x_OPER_MODE);

 /*   unsigned int fifoConfig = 0;

    fifoConfig = UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_4,
                                  UART_TRIG_LVL_GRANULARITY_1,
                                  UART_FCR_TX_TRIG_LVL_56,
                                  1,
                                  1,
                                  1,
                                  UART_DMA_EN_PATH_SCR,
                                  UART_DMA_MODE_0_ENABLE);


    UARTFIFOConfig(SOC_UART_0_REGS, fifoConfig);
*/
    UARTIntEnable(SOC_UART_0_REGS, UART_INT_RHR_CTI);


}



void McSPI0_Setup(void)
{
	/* Reset the McSPI instance.*/
    McSPIReset(SOC_SPI_0_REGS);
    /* Enable chip select pin.*/
    McSPICSEnable(SOC_SPI_0_REGS);
    /* Enable master mode of operation.*/
    McSPIMasterModeEnable(SOC_SPI_0_REGS);
    /* Perform the necessary configuration for master mode.*/
    McSPIMasterModeConfig(SOC_SPI_0_REGS, MCSPI_SINGLE_CH,
                          MCSPI_TX_RX_MODE, MCSPI_DATA_LINE_COMM_MODE_6, //****************
                          chNum);
    /* Configure the McSPI bus clock depending on clock mode. */
    McSPIClkConfig(SOC_SPI_0_REGS, MCSPI_IN_CLK, MCSPI_OUT_FREQ, chNum,
                   MCSPI_CLK_MODE_0);
    /* Configure the word length.*/
    McSPIWordLengthSet(SOC_SPI_0_REGS, MCSPI_WORD_LENGTH(8), chNum);
    /* Set polarity of SPIEN to low.*/
    McSPICSPolarityConfig(SOC_SPI_0_REGS, MCSPI_CS_POL_LOW, chNum);
    /* Enable the transmitter FIFO of McSPI peripheral.*/
    McSPITxFIFOConfig(SOC_SPI_0_REGS, MCSPI_TX_FIFO_ENABLE, chNum);
    /* Enable the receiver FIFO of McSPI peripheral.*/
    McSPIRxFIFOConfig(SOC_SPI_0_REGS, MCSPI_RX_FIFO_ENABLE, chNum);



    HWREG(SOC_SPI_0_REGS + MCSPI_CHCTRL(chNum)) = 0x180603CC;

    MUX_VAL(CONTROL_PADCONF_SPI0_SCLK, (IDIS | PU | MODE0 ));
    MUX_VAL(CONTROL_PADCONF_SPI0_D0, (IDIS | PU | MODE0 ));
    MUX_VAL(CONTROL_PADCONF_SPI0_D1, (IEN | OFF | MODE0 ));
    MUX_VAL(CONTROL_PADCONF_SPI0_CS0, (IDIS | PU | MODE0 ));
    MUX_VAL(CONTROL_PADCONF_SPI0_CS1, (IDIS | PU | MODE0 ));

    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_SPI0_SCLK) =	PINMUXMODE_1 | CONTROL_CONF_SPI0_SCLK_CONF_SPI0_SCLK_RXACTIVE;
    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_SPI0_D0 ) = 	PINMUXMODE_1 | CONTROL_CONF_SPI0_D0_CONF_SPI0_D0_RXACTIVE;
    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_SPI0_D1 ) = 	PINMUXMODE_1 | CONTROL_CONF_SPI0_D1_CONF_SPI0_D1_PUTYPESEL |
    			CONTROL_CONF_SPI0_D1_CONF_SPI0_D1_RXACTIVE;

    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_SPI0_CS0) =	PINMUXMODE_1 |
    				CONTROL_CONF_SPI0_D1_CONF_SPI0_D1_PUTYPESEL |
    			CONTROL_CONF_SPI0_CS0_CONF_SPI0_CS0_RXACTIVE;

    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_SPI0_CS1) =  PINMUXMODE_1 |
    			CONTROL_CONF_SPI0_CS1_CONF_SPI0_CS1_PUDEN |
   			CONTROL_CONF_SPI0_CS1_CONF_SPI0_CS1_RXACTIVE;
    HWREG(SOC_PRCM_REGS + CM_PER_SPI0_CLKCTRL) =
    				CM_PER_SPI0_CLKCTRL_MODULEMODE_ENABLE;


    HWREG(SOC_SPI_0_REGS + MCSPI_SYST) = 0x00000200;

    while ((HWREG(SOC_PRCM_REGS + CM_PER_L4LS_CLKSTCTRL) & CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_SPI_GCLK) !=
    						(CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_SPI_GCLK_ACT <<
    								CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_SPI_GCLK_SHIFT));

    while(HWREG(SOC_PRCM_REGS + CM_PER_SPI1_CLKCTRL) & CM_PER_SPI1_CLKCTRL_IDLEST != CM_PER_SPI1_CLKCTRL_IDLEST_FUNC);

    McSPIIntDisable(SOC_SPI_0_REGS,  MCSPI_INT_TX_EMPTY(chNum) | MCSPI_INT_TX_UNDERFLOW(chNum)
    									|MCSPI_INT_RX_FULL(chNum)|MCSPI_INT_RX0_OVERFLOW
    									|MCSPI_INT_EOWKE);

}

void Pin_multiplexing_Setup(void)
{
	int pin;

	//	Parallel port GPIO -> 16 A/D + 3 (TRDY, RECD, A/D Switch)

	MUX_VAL(CONTROL_PADCONF_LCD_DATA0,	(IDIS | PU | MODE7 ));	//gpio2_6 bit_0
	MUX_VAL(CONTROL_PADCONF_LCD_DATA1, 	(IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA2, 	(IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA3, 	(IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA4, 	(IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA5, 	(IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA6, 	(IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA7, 	(IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA8, 	(IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA9, 	(IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA10, (IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA11, (IDIS | PU | MODE7 ));	//gpio2_17	bit_11
	MUX_VAL(CONTROL_PADCONF_LCD_DATA12, (IDIS | PU | MODE7 ));	//gpio0_8	bit_12
	MUX_VAL(CONTROL_PADCONF_LCD_DATA13, (IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA14, (IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_LCD_DATA15, (IDIS | PU | MODE7 ));	//gpio0_11	bit_15

	MUX_VAL(CONTROL_PADCONF_LCD_VSYNC, 		(IDIS | PU | MODE7 ));	//gpio2_22	A/D Switch
	MUX_VAL(CONTROL_PADCONF_LCD_HSYNC, 		(IDIS | PU | MODE7 ));	//gpio2_23	TRDY
	MUX_VAL(CONTROL_PADCONF_LCD_PCLK,  		(IEN  | PU | MODE7 ));	//gpio2_24	RECD
	MUX_VAL(CONTROL_PADCONF_LCD_AC_BIAS_EN, (IEN  | PU | MODE7 ));	//gpio2_25	BRES_DONE


	GPIOModuleEnable(SOC_GPIO_0_REGS);
	GPIOModuleEnable(SOC_GPIO_2_REGS);

	for (pin = 8; pin<12; pin++)
	{
		GPIODirModeSet(SOC_GPIO_0_REGS, pin, GPIO_DIR_OUTPUT);
	}

	for (pin = 6; pin<18; pin++)
	{
		GPIODirModeSet(SOC_GPIO_2_REGS, pin, GPIO_DIR_OUTPUT);
	}

	GPIODirModeSet(SOC_GPIO_2_REGS, 22, GPIO_DIR_OUTPUT);
	GPIODirModeSet(SOC_GPIO_2_REGS, 23, GPIO_DIR_OUTPUT);
	GPIODirModeSet(SOC_GPIO_2_REGS, 24, GPIO_DIR_INPUT);
	GPIODirModeSet(SOC_GPIO_2_REGS, 25, GPIO_DIR_INPUT);

	//	WizNet pins -> nINT & Reset

	MUX_VAL(CONTROL_PADCONF_UART1_RTSN, (IDIS | PU | MODE7 ));
	MUX_VAL(CONTROL_PADCONF_UART1_CTSN, (IEN | PU | MODE7 ));


	GPIOModuleEnable(SOC_GPIO_0_REGS);

	//	Interrupts
	GPIOIntTypeSet(SOC_GPIO_0_REGS, 12, GPIO_INT_TYPE_LEVEL_LOW);
	GPIOIntTypeSet(SOC_GPIO_2_REGS, 25, GPIO_INT_TYPE_LEVEL_HIGH);

	GPIODirModeSet(SOC_GPIO_0_REGS, 13, GPIO_DIR_OUTPUT);
	GPIODirModeSet(SOC_GPIO_0_REGS, 12, GPIO_DIR_INPUT);

	//sysb/dat & r/w switch pins

	GPIOModuleEnable(SOC_GPIO_1_REGS);
	MUX_VAL(CONTROL_PADCONF_GPMC_A3, (IDIS | PU | MODE7 ));
	GPIODirModeSet(SOC_GPIO_1_REGS, 19, GPIO_DIR_OUTPUT);

	GPIOPinWrite(SOC_GPIO_1_REGS, 19, 1);


	MUX_VAL(CONTROL_PADCONF_GPMC_A0, (IDIS | PU | MODE7 ));
	GPIODirModeSet(SOC_GPIO_1_REGS, 16, GPIO_DIR_OUTPUT);

	GPIOPinWrite(SOC_GPIO_1_REGS, 16, 1);


}



void PP_SET_PORT_MODE (unsigned char mode)
{
	// PP_MODE_TRANSMIT 0x1
	// PP_MODE_RECIEVE  0x2

	unsigned char pin;

	if (mode == 0x1)
	{
		MUX_VAL(CONTROL_PADCONF_LCD_DATA0,	(IDIS | PU | MODE7 ));	//gpio2_6 bit_0
		MUX_VAL(CONTROL_PADCONF_LCD_DATA1, 	(IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA2, 	(IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA3, 	(IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA4, 	(IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA5, 	(IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA6, 	(IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA7, 	(IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA8, 	(IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA9, 	(IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA10, (IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA11, (IDIS | PU | MODE7 ));	//gpio2_17	bit_11
		MUX_VAL(CONTROL_PADCONF_LCD_DATA12, (IDIS | PU | MODE7 ));	//gpio0_8	bit_12
		MUX_VAL(CONTROL_PADCONF_LCD_DATA13, (IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA14, (IDIS | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA15, (IDIS | PU | MODE7 ));	//gpio0_11	bit_15

		for (pin = 8; pin<12; pin++)
		{
			GPIODirModeSet(SOC_GPIO_0_REGS, pin, GPIO_DIR_OUTPUT);
		}

		for (pin = 6; pin<18; pin++)
		{
			GPIODirModeSet(SOC_GPIO_2_REGS, pin, GPIO_DIR_OUTPUT);
		}

		GPIOPinWrite(SOC_GPIO_1_REGS, 16, 1);		// R/W to 'write' (1)
	}
	else if (mode == 0x2)
	{
		MUX_VAL(CONTROL_PADCONF_LCD_DATA0,	(IEN | PU | MODE7 ));	//gpio2_6 bit_0
		MUX_VAL(CONTROL_PADCONF_LCD_DATA1, 	(IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA2, 	(IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA3, 	(IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA4, 	(IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA5, 	(IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA6, 	(IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA7, 	(IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA8, 	(IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA9, 	(IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA10, (IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA11, (IEN | PU | MODE7 ));	//gpio2_17	bit_11
		MUX_VAL(CONTROL_PADCONF_LCD_DATA12, (IEN | PU | MODE7 ));	//gpio0_8	bit_12
		MUX_VAL(CONTROL_PADCONF_LCD_DATA13, (IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA14, (IEN | PU | MODE7 ));
		MUX_VAL(CONTROL_PADCONF_LCD_DATA15, (IEN | PU | MODE7 ));	//gpio0_11	bit_15

		for (pin = 8; pin<12; pin++)
		{
			GPIODirModeSet(SOC_GPIO_0_REGS, pin, GPIO_DIR_INPUT);
		}

		for (pin = 6; pin<18; pin++)
		{
			GPIODirModeSet(SOC_GPIO_2_REGS, pin, GPIO_DIR_INPUT);
		}

		GPIOPinWrite(SOC_GPIO_1_REGS, 16, 0);		// R/W to 'read' (0)
	}
	else
	{
		//wrong mode, do nothing
	}
}
