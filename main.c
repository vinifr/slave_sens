//*****************************************************************************
//
// timers.c - Timers example.
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"

#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Timer (timers)</h1>
//!
//! This example application demonstrates the use of the timers to generate
//! periodic interrupts.  One timer is set up to interrupt once per second and
//! the other to interrupt twice per second; each interrupt handler will toggle
//! its own indicator throught the UART.
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//
//*****************************************************************************

#define V_HIGH			3	// 3 Volts
#define V_REF			(4096*V_HIGH) / 3.3
#define NIVEL_1V		3723	// Valor referente a 3V depois da conversao

#define INTERT_1MS		5
#define INTERT_2MS		10

#define MAX_SENSORES	96
#define ID_FIM			MAX_SENSORES/8
//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;
//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags;
uint32_t f_timer;

char f_adc, f_totalSens;
char buff[32];
uint16_t id_tx;
uint8_t leituras[ID_FIM+8];
uint32_t ui32ADC0Value[8];
float value;

//, ui32TempAvg, ui32TempValueC;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void I2C0IntHandler(void)
{
	volatile unsigned int uiData = 0;
	unsigned int uiStatus;

	//
	// Clear the I2C0 interrupt flag.
	//
	I2CSlaveIntClear(I2C0_BASE);

	uiStatus = I2CSlaveStatus(I2C0_BASE);


	if(uiStatus & I2C_SLAVE_ACT_TREQ)
	{
		if (id_tx < ID_FIM)
		{
			// based on status, master is requesting data
			I2CSlaveDataPut(I2C0_BASE, leituras[id_tx++]);
		}
	}

	if(uiStatus & I2C_SLAVE_ACT_RREQ)
	{
		// based on status, master is pushing data
		uiData = I2CSlaveDataGet(I2C0_BASE);
		//if(++uiData > 'Z')
			//uiData='A';
	}
}

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void Timer0IntHandler(void)
{
    //char cOne, cTwo;

    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    HWREGBITW(&g_ui32Flags, 0) ^= 1;
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_ui32Flags);
    //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, g_ui32Flags);

    f_timer++;
}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void Timer1IntHandler(void)
{
    //char cOne, cTwo;

    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    //
    // Toggle the flag for the second timer.
    //
    HWREGBITW(&g_ui32Flags, 1) ^= 1;
    //
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, g_ui32Flags);
    //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, g_ui32Flags);
    //
    // Update the interrupt status.
    // 
    /*ROM_IntMasterDisable();
    cOne = HWREGBITW(&g_ui32Flags, 0) ? '1' : '0';
    cTwo = HWREGBITW(&g_ui32Flags, 1) ? '1' : '0';
    UARTprintf("\rT1: %c  T2: %c", cOne, cTwo);
    ROM_IntMasterEnable();*/
}

void ADCISRHandler(void)
{
    while (!ADCIntStatus(ADC0_BASE, 1, false)){};

    ADCIntClear(ADC0_BASE, 1);
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);

    f_adc = 1;
    //ui32TempAvg = ui32ADC0Value[3];
    //ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096) / 10;
}

void ADCConfig(void)
{
	SysCtlClockSet(
	SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_SYSDIV_5);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);

	ADCSequenceDisable(ADC0_BASE, 1);
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

	GPIOPinTypeADC(GPIO_PORTE_BASE,
	GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	//SysCtlADCSpeedSet(SYSCTL_ADCSPEED_250KSPS);

	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH3 |
			ADC_CTL_IE | ADC_CTL_END);

	IntEnable(INT_ADC0SS1);
	ADCIntEnable(ADC0_BASE, 1);
	ADCSequenceEnable(ADC0_BASE, 1);
}

int print_sensores()
{
	memset(buff, 0, sizeof(buff));
	value = ((float)ui32ADC0Value[0] * 3.3) / 4096;
	sprintf( buff, "AN1:%2f", value);
	UARTprintf("%s\n", buff);

	value = ((float)ui32ADC0Value[1] * 3.3) / 4096;
	sprintf(buff, "AN2:%2f", value);
	UARTprintf("%s\n", buff);

	value = ((float)ui32ADC0Value[2] * 3.3) / 4096;
	sprintf(buff, "AN3:%2f", value);
	UARTprintf("%s\n", buff);

	value = ((float)ui32ADC0Value[3] * 3.3) / 4096;
	sprintf(buff, "AN4:%2f", value);
	UARTprintf("%s\n\n", buff);
	return 0;
}

void InitI2C(void)
{
	//
	// For this example I2C0 is used with PortB[3:2].  The actual port and
	// pins used may be different on your part, consult the data sheet for
	// more information.  GPIO port B needs to be enabled so these pins can
	// be used.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
	// The I2C0 peripheral must be enabled before use.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

	//
	// Configure the pin muxing for I2C0 functions on port B2 and B3.
	// This step is not necessary if your part does not support pin muxing.
	//
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	//
	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	//
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);


	// Set GPIO Pins for Open-Drain operation (I have two Rpulls=10K Ohm to 5V on the SCL and SDA lines)
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

	// Give control to the I2C0 Module
	GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_DIR_MODE_HW);
	GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_DIR_MODE_HW);

}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

void TimerConfig(uint32_t periodT0, uint32_t periodT1)
{
	//
	// Enable the peripherals used by this example.
	//
	if (periodT0)
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	if (periodT1)
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	//
	// Configure the two 32-bit periodic timers.
	//
	if (periodT0)
	{
		ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
		ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, periodT0);
		ROM_IntEnable(INT_TIMER0A);
		ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	}

	if (periodT1)
	{
		ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
		ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, periodT1 / 4);
		ROM_IntEnable(INT_TIMER1A);
		ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
		ROM_TimerEnable(TIMER1_BASE, TIMER_A);
	}
}

int comuta_mux()
{
	return 0;
}

//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int
main(void)
{
	//char state;
	unsigned char mask;
	uint16_t id;
    //
    // Set the clocking to run directly from the crystal at 120MHz.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);

    //
    // Initialize the UART and write status.
    //
    ConfigureUART();
    f_timer = 0;

    UARTprintf("Sensores usando TivaC\n");
    //
    // Enable the GPIO port that is used for the on-board LEDs.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    //ROM_GPIODirModeSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_DIR_MODE_OUT);
    //
    // Enable the GPIO pins for the LEDs (PN0 & PN1).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);


    TimerConfig(0, g_ui32SysClock/2);	// 10000=200us; 100000=20us
    ADCConfig();
    //
	// Enable processor interrupts.
	//
	ROM_IntMasterEnable();

    ADCProcessorTrigger(ADC0_BASE, 1);
    //read_sensor();
    f_adc = 0;
    id_tx = 0;
    f_totalSens = 1;
    mask = 0x01;

    //
    // Loop forever while the timers run.
    //
    while(1)
    {
    	if (f_adc & f_totalSens)
    	{
    		f_adc = 0;

    		if (id >= ID_FIM)
    		{
    			f_totalSens = 0;
    			id = 0;
    			id_tx = 0;
    			continue;
    		}

    		if (mask == 0x80)
    		{
    			mask = 0x01;
    			id++;
    		}
    		leituras[id] = (ui32ADC0Value[0] >= NIVEL_1V ? 1: 0) & mask;
    		mask <<= 1;
    		leituras[id] = (ui32ADC0Value[1] >= NIVEL_1V ? 1: 0) & mask;
    		mask <<= 1;
    		leituras[id] = (ui32ADC0Value[2] >= NIVEL_1V ? 1: 0) & mask;
    		mask <<= 1;
    		leituras[id] = (ui32ADC0Value[3] >= NIVEL_1V ? 1: 0) & mask;
    		mask <<= 1;
    		comuta_mux();
    		ADCProcessorTrigger(ADC0_BASE, 1);
    	}
    }
}
