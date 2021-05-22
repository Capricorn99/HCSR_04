/*
 * main.c
 *
 *  Created on: May 19, 2021
 *      Author: Admin
 */




#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_timer.h"                   // Defines and macros used when accessing the timer


#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"                  // Defines and macros for ROM API of driverLib

#define UART0_BAUDRATE  115200              // Macro for UART0 Baud rate

/* -----------------------      Global Variables        --------------------- */
volatile bool boolTrigCondition = 1;        // Variable to control the Trigger Pin Switching
volatile uint32_t ui32EchoDuration = 0;     // Variable to store duration for which Echo Pin is high
volatile uint32_t ui32ObstacleDist = 0;     // Variable to store distance of the Obstacle

uint8_t ui8WelcomeText[] = {"\n\rDistance: "};


void GPIOIntHandler()
{
    // The ISR for GPIO PortA Interrupt Handling

    // Clear the GPIO Hardware Interrupt
    GPIOIntClear(GPIO_PORTA_BASE , GPIO_INT_PIN_2);

    // Condition when Echo Pin (PA2) goes high
    if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == GPIO_PIN_2){
        // Initialize Timer2 with value 0
        HWREG(TIMER2_BASE + TIMER_O_TAV) = 0;
        // Enable Timer2 to start measuring duration for which Echo Pin is High
        TimerEnable(TIMER2_BASE, TIMER_A);
    }
    else{
        ui32EchoDuration = TimerValueGet(TIMER2_BASE, TIMER_A);
        // Disable Timer2 to stop measuring duration for which Echo Pin is High
        TimerDisable(TIMER2_BASE, TIMER_A);
        // Convert the Timer Duration to Distance Value according to Ultrasonic's formula
        ui32ObstacleDist = (ui32EchoDuration /2353) ;
        // Convert the Distance Value from Integer to Array of Characters
        char chArrayDistance[8];
        ltoa(ui32ObstacleDist, chArrayDistance,10);

        // Transmit the distance reading to the terminal
        uint8_t iter;
        for (iter = 0; iter<sizeof(chArrayDistance); iter++ ) UARTCharPut(UART0_BASE, chArrayDistance[iter]);
        for (iter = 0; iter<sizeof(ui8WelcomeText); iter++ ) UARTCharPut(UART0_BASE, ui8WelcomeText[iter]);

        // Enable condition for Trigger Pulse
        boolTrigCondition = 1;
    }

}
void Timer0IntHandler()
{
    // The ISR for Timer0 Interrupt Handling
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Disable the timer
    TimerDisable(TIMER0_BASE, TIMER_A);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);

}

int main(void)
{
    SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ | SYSCTL_USE_PLL | SYSCTL_SYSDIV_5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);


    // Configure PA0 as UART0_Rx and PA1 as UART0_Tx
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Configure the baud rate and data setup for the UART0
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), UART0_BAUDRATE, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE );
    // Enable the UART0
    UARTEnable(UART0_BASE);

    //echo pinA2
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
    //IntPrioritySet(INT_GPIOA, 1);
    IntRegister(INT_GPIOA, GPIOIntHandler);
    IntEnable(INT_GPIOA);
    IntMasterEnable();

    //trigger A3
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);

    //Kich hoat timer0
    TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
    //IntPrioritySet(INT_TIMER0A, 0);
    IntRegister(INT_TIMER0A, Timer0IntHandler);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // timer2
    TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT_UP);
    //IntPrioritySet(INT_TIMER0A, 1);


    while(1) {
        if (boolTrigCondition)
        {
            //nap timer0 gia tro 10us
            TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()/100000) - 1);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            TimerEnable(TIMER0_BASE, TIMER_A);
            boolTrigCondition = 0;
        }
        SysCtlDelay(2000000);

    }
}




