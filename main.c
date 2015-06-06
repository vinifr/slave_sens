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

#define I2C_SLAVE_ADDR		0x30
#define V_HIGH				3	// 3 Volts
#define V_REF				(4096*V_HIGH) / 3.3
#define NIVEL_1V			3723	// Valor referente a 3V depois da conversao

#define INTERT_1MS			5
#define INTERT_2MS			10

#define MAX_SENSORES		96
#define ID_FIM				MAX_SENSORES/8

#define A4_LOW			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0)
#define A3_LOW 			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0)
#define A2_LOW 			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0)
#define A1_LOW 			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0)
#define A0_LOW 			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0)
#define EN_LOW 			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0)
#define CS_LOW 			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0)
#define WR_LOW 			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0)

#define A4_HIGH			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 1)
#define A3_HIGH			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 1)
#define A2_HIGH			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 1)
#define A1_HIGH			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 1)
#define A0_HIGH			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 1)
#define EN_HIGH			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 1)
#define CS_HIGH			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 1)
#define WR_HIGH			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 1)
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

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);

	ADCSequenceDisable(ADC0_BASE, 1);
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2); // PE1(D3) e PE2(D1)
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3); // PD3(D2)

	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1);	// D1
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH4); // D2
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END); // D3
	//ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH3 |	ADC_CTL_IE | ADC_CTL_END);

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
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);


	// Set GPIO Pins for Open-Drain operation (I have two Rpulls=10K Ohm to 5V on the SCL and SDA lines)
	//GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
	//GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

	// Give control to the I2C0 Module
	GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_DIR_MODE_HW);
	GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_DIR_MODE_HW);

	ROM_IntEnable(INT_I2C0);
	I2CSlaveInit(I2C0_BASE, I2C_SLAVE_ADDR);
	I2CSlaveIntEnable(I2C0_BASE);
	I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);
	I2CSlaveEnable(I2C0_BASE);
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

void GPIOConfig()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// Habilita PA5(A1) e PA6(A0) como saida
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6);
	// Habilita PB0(WR), PB1(CS), PB4(A2) e PB5(EN) como saida
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
			GPIO_PIN_4 | GPIO_PIN_5);
	// Habilita PE4(A4) e PE5(A3) como saida
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
}

int comuta_mux(int pos)
{
	//aciona S1 do mux, dai le D1 (sinal mux1), D2 (sinal mux 2), D3 (sinal mux3)

	switch(pos){
		case 0:
			A4_LOW; A3_LOW; A2_LOW; A1_LOW; A0_LOW;
			break;
		case 1:
			A4_LOW; A3_LOW; A2_LOW; A1_LOW; A0_HIGH;
			break;
		case 2:
			A4_LOW; A3_LOW; A2_LOW; A1_HIGH; A0_LOW;
			break;
		case 3:
			A4_LOW; A3_LOW; A2_LOW; A1_HIGH; A0_HIGH;
			break;
		case 4:
			A4_LOW; A3_LOW; A2_HIGH; A1_LOW; A0_LOW;
			break;
		case 5:
			A4_LOW; A3_LOW; A2_HIGH; A1_LOW; A0_HIGH;
			break;
		case 6:
			A4_LOW; A3_LOW; A2_HIGH; A1_HIGH; A0_LOW;
			break;
		case 7:
			A4_LOW; A3_LOW; A2_HIGH; A1_HIGH; A0_HIGH;
			break;
		case 8:
			A4_LOW; A3_HIGH; A2_LOW; A1_LOW; A0_LOW;
			break;
		case 9:
			A4_LOW; A3_HIGH; A2_LOW; A1_LOW; A0_HIGH;
			break;
		case 10:
			A4_LOW; A3_HIGH; A2_LOW; A1_HIGH; A0_LOW;
			break;
		case 11:
			A4_LOW; A3_HIGH; A2_LOW; A1_HIGH; A0_HIGH;
			break;
		case 12:
			A4_LOW; A3_HIGH; A2_HIGH; A1_LOW; A0_LOW;
			break;
		case 13:
			A4_LOW; A3_HIGH; A2_HIGH; A1_LOW; A0_HIGH;
			break;
		case 14:
			A4_LOW; A3_HIGH; A2_HIGH; A1_HIGH; A0_LOW;
			break;
		case 15:
			A4_LOW; A3_HIGH; A2_HIGH; A1_HIGH; A0_HIGH;
			break;
		case 16:
			A4_HIGH; A3_LOW; A2_LOW; A1_LOW; A0_LOW;
			break;
		case 17:
			A4_HIGH; A3_LOW; A2_LOW; A1_LOW; A0_HIGH;
			break;
		case 18:
			A4_HIGH; A3_LOW; A2_LOW; A1_HIGH; A0_LOW;
			break;
		case 19:
			A4_HIGH; A3_LOW; A2_LOW; A1_HIGH; A0_HIGH;
			break;
		case 20:
			A4_HIGH; A3_LOW; A2_HIGH; A1_LOW; A0_LOW;
			break;
		case 21:
			A4_HIGH; A3_LOW; A2_HIGH; A1_LOW; A0_HIGH;
			break;
		case 22:
			A4_HIGH; A3_LOW; A2_HIGH; A1_HIGH; A0_LOW;
			break;
		case 23:
			A4_HIGH; A3_LOW; A2_HIGH; A1_HIGH; A0_HIGH;
			break;
		case 24:
			A4_HIGH; A3_HIGH; A2_LOW; A1_LOW; A0_LOW;
			break;
		case 25:
			A4_HIGH; A3_HIGH; A2_LOW; A1_LOW; A0_HIGH;
			break;
		case 26:
			A4_HIGH; A3_HIGH; A2_LOW; A1_HIGH; A0_LOW;
			break;
		case 27:
			A4_HIGH; A3_HIGH; A2_LOW; A1_HIGH; A0_HIGH;
			break;
		case 28:
			A4_HIGH; A3_HIGH; A2_HIGH; A1_LOW; A0_LOW;
			break;
		case 29:
			A4_HIGH; A3_HIGH; A2_HIGH; A1_LOW; A0_HIGH;
			break;
		case 30:
			A4_HIGH; A3_HIGH; A2_HIGH; A1_HIGH; A0_LOW;
			break;
		case 31:
			A4_HIGH; A3_HIGH; A2_HIGH; A1_HIGH; A0_HIGH;
			break;
	}

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
	unsigned char mask1, mask2, mask3, mask4;
	uint16_t id, mux_cont;

	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	ROM_FPULazyStackingEnable();

	//
	// Set the clocking to run directly from the crystal.
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
					   SYSCTL_XTAL_16MHZ);
	g_ui32SysClock = ROM_SysCtlClockGet();


    //
    // Initialize the UART and write status.
    //
    ConfigureUART();
    f_timer = 0;

    UARTprintf("Sensores usando TivaC\n");

    GPIOConfig();
    InitI2C();
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
    mux_cont = 0;
    f_totalSens = 1;
    mask1 = 0x01;
    mask2 = 0x01;
    mask3 = 0x01;
    mask4 = 0x01;

    CS_LOW;
    WR_LOW;
    //
    // Loop forever while the timers run.
    //
    while(1)
    {
    	if (id_tx >= ID_FIM)
		{
			id_tx = 0;
			f_totalSens = 1;
		}

    	if (f_adc & f_totalSens)
    	{
    		f_adc = 0;

    		if(mux_cont >= 31)
    			mux_cont = 0;
    		if (id >= ID_FIM)
    		{
    			f_totalSens = 0;
    			id = 0;
    			id_tx = 0;
    			continue;
    		}
    		if (mask4 == 0x80)
    		{
    			mask1 = 0x01; mask2 = 0x01; mask3 = 0x01; mask4 = 0x01;
    			id++;
    		}
    		leituras[id] = (ui32ADC0Value[0] >= NIVEL_1V ? 1: 0) & mask1;
    		mask1 <<= 1;
    		leituras[id+3] = (ui32ADC0Value[1] >= NIVEL_1V ? 1: 0) & mask2;
    		mask2 <<= 1;
    		leituras[id+6] = (ui32ADC0Value[2] >= NIVEL_1V ? 1: 0) & mask3;
    		mask3 <<= 1;
    		leituras[id+9] = (ui32ADC0Value[3] >= NIVEL_1V ? 1: 0) & mask4;
    		mask4 <<= 1;
    		comuta_mux(mux_cont++);
    		ADCProcessorTrigger(ADC0_BASE, 1);
    	}
    }
}
