//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <limits.h> //uint max
#include "inc/hw_ints.h" // Definitions for the interrupt and register assignments.
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "MyQueue.h"
#include "display_float.h"

#define AVG_STEPS 16

#define HEARTBEAT_LENGTH 1000000

#define MENU_STRING "\n\rMenu Selection :\n\rM - Print This Menu\n\rA - toggle alpha\n\rB - toggle beta\n\rP - Print Clock Speed\n\rC - Clear Terminal Window\n\r\n\r"

// global variables
bool alphaEnable = false, betaEnable = false, alphaCanPrint = true, betaCanPrint = true;
uint32_t sys_clock;
uint32_t startAlpha[AVG_STEPS] = {0}, endAlpha[AVG_STEPS] = {0}, startBeta[AVG_STEPS] = {0}, endBeta[AVG_STEPS] = {0};
uint8_t alphaPos = 0, betaPos = 0;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

//*****************************************************************************
//
// Create function prototypes
//
//*****************************************************************************

int32_t UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);

void heartbeat(void);                   // Function for heartbeat

void processMenu(char input);           // Function to process menu input

void UARTIntHandler(void);              // UART interrupt handler

void SytemConfiguration(void);          // Configure the system

void peripheralEnable(void);            // Enable the peripherals

void UARTConfig(void);                  // Configure the UART

void LEDEnable(void);                   // Enable the user LED

void printMenu(void);                   // Print the menu over UART

void clearMenu(void);                   //Clear the menu over UART

void initTimer(void);                  //initialize timer for uart edge capture

void alphaMesure();                     //measure the position as a function of pwm period and duty cycle for alpha

void betaMesure();                      //measure the position as a function of pwm period and duty cycle for beta

//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
int main(void)
{
  char x;
    
  FPUEnable();
  FPULazyStackingEnable();
   
  SytemConfiguration();

  IntMasterDisable();
  
  sys_clock = SysCtlClockGet();

  peripheralEnable();

  UARTConfig();
  
  LEDEnable();
    
  QueueInit();
  
  initTimer();

  //
  // Enable processor interrupts.
  //
  IntMasterEnable();
  
  clearMenu();
  
  printMenu();

  //
  // Loop forever.
  //
  while(1)
  {
    while (!QueueEmpty()){
      QueueGet(&x);
      processMenu(x);
    }
    // heartbeat();
  }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
int32_t UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
  int32_t charsSent = 0;
  //
  // Loop while there are more characters to send.
  //
  while(ui32Count--)
  {
    charsSent++;
    //
    // Write the next character to the UART.
    //
    UARTCharPut(UART0_BASE, *pui8Buffer++);
  }
  return charsSent;
}

void heartbeat(){
  //
  // Turn on the LED.
  //
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

  //
  // Delay for a bit.
  //
  for(volatile uint32_t ui32Loop = 0; ui32Loop < HEARTBEAT_LENGTH; ui32Loop++)
  {
  }

  //
  // Turn off the LED.
  //
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

  //
  // Delay for a bit.
  //
  for(volatile uint32_t ui32Loop = 0; ui32Loop < HEARTBEAT_LENGTH; ui32Loop++)
  {
  }
}

//*****************************************************************************
//
// Process the menu selection
//
//*****************************************************************************

void processMenu(char input) {
  char badInput[] = "\n\r\n\rYou Did Not Enter A Valid Character";
  uint32_t badInputCharCnt = (uint32_t)sizeof(badInput) / (uint32_t)sizeof(badInput);

  IntMasterDisable();
    switch(input)
    {
    case 'M':
    case 'm':
      printMenu();
      break;
    case 'A':
    case 'a':
      if(!alphaEnable)
      {
        alphaEnable = true;
        UARTSend((uint8_t *) "Alpha Read Enabled\n\r", 22);
        // The specified interrupt is enabled in the interrupt controller.
        IntEnable(INT_TIMER0B);
      }
      else {
        alphaEnable = false;
        UARTSend((uint8_t *) "Alpha Read Disabled\n\r", 22);
        //The specified interrupt is enabled in the interupt controller.
        IntDisable(INT_TIMER0B);
      }
      break;
    case 'B':
    case 'b':
      if(!betaEnable)
      {
        betaEnable = true;
        UARTSend((uint8_t *) "Beta Read Enabled\n\r", 20);
        // The specified interrupt is enabled in the interrupt controller.
        IntEnable(INT_TIMER2B);
      }
      else {
        betaEnable = false;
        UARTSend((uint8_t *) "Beta Read Disabled\n\r", 21);
        //The specified interrupt is enabled in the interupt controller.
        IntDisable(INT_TIMER2B);
      }
      break;
    case 'P':
    case 'p':
      char buffer[32];
      sprintf(buffer, "Timer Speed %d\n\r", sys_clock);
      UARTSend((uint8_t *)buffer, 32);
      break;
    case 'C':
    case 'c':
      clearMenu();
      break;
    default:
      UARTSend((uint8_t *) badInput, badInputCharCnt);
      break;
    }
    //TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/timerRate);
  IntMasterEnable();
 }

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
      char a = UARTCharGetNonBlocking(UART0_BASE);
      QueuePut(a);
    }
}

//*****************************************************************************
//
// Configure the system
//
//*****************************************************************************

void SytemConfiguration(void){
  //
  // Enable lazy stacking for interrupt handlers.  This allows floating-point
  // instructions to be used within interrupt handlers, but at the expense of
  // extra stack usage.
  //

  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
		 SYSCTL_OSC_MAIN);
}

//*****************************************************************************
//
// Enable the peripherals
//
//*****************************************************************************

void peripheralEnable(void){
  //
  // Enable the peripherals used by this example.
  //

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
}

//*****************************************************************************
//
// Configure UART
//
//*****************************************************************************

void UARTConfig(void){
  //
  // Set GPIO A0 and A1 as UART pins.
  //
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                   (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                    UART_CONFIG_PAR_NONE));

  //
  // Enable the UART interrupt.
  //
  IntEnable(INT_UART0);
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

//*****************************************************************************
//
// Enable the user LED
//
//*****************************************************************************

void LEDEnable(void){
  //
  // Enable the GPIO port that is used for the on-board LED.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  //
  // Check if the peripheral access is enabled.
  //
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
  {
  }
  //
  // Enable the GPIO pin for the LED (PG2).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  //
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
}

//*****************************************************************************
//
// Print the menu over UART
//
//*****************************************************************************

void printMenu()
{
  uint32_t charCnt = (uint32_t)sizeof(MENU_STRING) / (uint32_t)sizeof(char);

  UARTSend((uint8_t *)MENU_STRING, charCnt);
}

//*****************************************************************************
//
// Clear the menu over UART
//
//*****************************************************************************

void clearMenu()
{
  UARTSend((uint8_t *) "\033[2J\n\r", 7);
}

//*****************************************************************************
//
// Initialize the timer for pwm edge capture
//
//*****************************************************************************
void initTimer()
{
  // Enable and configure Timer0 peripheral.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

  // Initialize timer A and B to count up in edge time mode
  TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_CAP_TIME_UP));
  TimerConfigure(TIMER2_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_CAP_TIME_UP));

  // Timer a records pos edge time and Timer b records neg edge time
  TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
  TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  TimerControlEvent(TIMER2_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);

  //set the value that the timers count to (0x9C3F = 39999)
  //CO2 sensor outputs 1khz pwm so with mcu at 40Mhz, timers should stay in sync with CO2 output
  TimerLoadSet(TIMER0_BASE, TIMER_BOTH, sys_clock / 1000);
  TimerLoadSet(TIMER2_BASE, TIMER_BOTH, sys_clock / 1000);

  //Configure the pin that the timer reads from (PB6)
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  GPIOPinConfigure(GPIO_PB6_T0CCP0);
  GPIOPinConfigure(GPIO_PB7_T0CCP1);
  GPIOPinConfigure(GPIO_PB0_T2CCP0);
  GPIOPinConfigure(GPIO_PB1_T2CCP1);
  GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);
  GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_7);
  GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0);
  GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_1);

  // Registers a interrupt function to be called when timer b hits a neg edge event
  IntRegister(INT_TIMER0B, alphaMesure);
  IntRegister(INT_TIMER2B, betaMesure);
  // Makes sure the interrupt is cleared
  TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);
  TimerIntClear(TIMER2_BASE, TIMER_CAPB_EVENT);
  // Enable the indicated timer interrupt source.
  TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);
  TimerIntEnable(TIMER2_BASE, TIMER_CAPB_EVENT);
  
  //
  // Enable the timers.
  //
  TimerEnable(TIMER0_BASE, TIMER_BOTH);
  TimerEnable(TIMER2_BASE, TIMER_BOTH);
}

//*****************************************************************************
//
//measure the position as a function of pwm period and duty cycle for alpha
//
//*****************************************************************************

void alphaMesure()
{
  TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);
  
  uint32_t lengthPeriod;
  uint32_t lengthSignal;
  float degrees;
  
  startAlpha[alphaPos] = TimerValueGet(TIMER0_BASE, TIMER_A);
  endAlpha[alphaPos] = TimerValueGet(TIMER0_BASE, TIMER_B);
  lengthPeriod = (startAlpha[alphaPos] - startAlpha[(alphaPos + 1) % 16])/15;
  lengthSignal = (startAlpha[alphaPos] - endAlpha[alphaPos]);
  
  if(alphaCanPrint){
    char buffer[32];
    sprintf(buffer, "position = %d\n\n\r", lengthSignal);
    UARTSend((uint8_t *)buffer, 64);
  } else if (!alphaCanPrint && alphaPos == 15) {
    alphaCanPrint = true;
  }
  
  alphaPos = (alphaPos + 1) % 16;
}

//*****************************************************************************
//
//measure the position as a function of pwm period and duty cycle for beta
//
//*****************************************************************************

void betaMesure()
{
  TimerIntClear(TIMER2_BASE, TIMER_CAPB_EVENT);
  
  uint32_t lengthPeriod;
  uint32_t lengthSignal;
  float degrees;
  
  startBeta[betaPos] = TimerValueGet(TIMER0_BASE, TIMER_A);
  endBeta[betaPos] = TimerValueGet(TIMER0_BASE, TIMER_B);
  lengthPeriod = (startBeta[betaPos] - startBeta[(betaPos + 1) % 16])/15;
  lengthSignal = (startBeta[betaPos] - endBeta[betaPos]);
  
  if(betaCanPrint){
    char buffer[32];
    sprintf(buffer, "position = %d\n\n\r", lengthSignal);
    UARTSend((uint8_t *)buffer, 32);
  } else if (!betaCanPrint && betaPos == 15) {
    betaCanPrint = true;
  }
  
  betaPos = (betaPos + 1) % 16;
}