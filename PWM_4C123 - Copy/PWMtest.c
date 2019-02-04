// PWMtest.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
  Program 6.7, section 6.3.2

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value

#include <stdint.h>
#include "PLL.h"
#include "PWM.h"

void WaitForInterrupt(void);  // low power mode

void DisableInterrupts(void);
void PortF_Init(void);
void EnableInterrupts(void);

int main(void){
	
	DisableInterrupts();
	
  PLL_Init();                      // bus clock at 80 MHz
	
	EnableInterrupts();
	PortF_Init();

  // PWM1A_Init(40000, 30000);         // initialize PWM0, 1000 Hz, 75% duty
  PWM1B_Init(2, 1);         // initialize PWM1_3_B, 1000 Hz, 25% duty
//  PWM0A_Duty(4000);    // 10%
//  PWM0A_Duty(10000);   // 25%
//  PWM0A_Duty(30000);   // 75%
//  PWM0B_Duty(4000);    // 10%
//  PWM0B_Duty(10000);   // 25%
//  PWM0B_Duty(30000);   // 75%


//  PWM0A_Init(4000, 2000);         // initialize PWM0, 10000 Hz, 50% duty
//  PWM0A_Init(1000, 900);          // initialize PWM0, 40000 Hz, 90% duty
//  PWM0A_Init(1000, 100);          // initialize PWM0, 40000 Hz, 10% duty
//  PWM0A_Init(40, 20);             // initialize PWM0, 1 MHz, 50% duty
//  PWM0B_Init(4000, 2000);         // initialize PWM0, 10000 Hz, 50% duty
//  PWM0B_Init(1000, 900);          // initialize PWM0, 40000 Hz, 90% duty
//  PWM0B_Init(1000, 100);          // initialize PWM0, 40000 Hz, 10% duty
//  PWM0B_Init(40, 20);             // initialize PWM0, 1 MHz, 50% duty
	
	// on reset, Mode 1 = LED is Red
	GPIO_PORTF_DATA_R = 0x02;  // LED is RED
	
  while(1){
		// inverse every second to blink
		GPIO_PORTF_DATA_R = ~GPIO_PORTF_DATA_R;
		
    WaitForInterrupt();
  }
}

void PortF_Init(void){
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     		// 1) F clock
  delay = SYSCTL_RCGC2_R;           		// 		delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   		// 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1E;           		// 		allow changes to PF 4-1       
	// GPIO_PORTF_AMSEL_R = 0x00;         // 3) disable analog function
	// GPIO_PORTF_PCTL_R = 0x00000000;    // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          		// 5) PF4 input, PF3,PF2,PF1 output   
	// GPIO_PORTF_AFSEL_R = 0x00;         // 6) no alternate function
	GPIO_PORTF_PUR_R = 0x10;          	// 		enable pullup resistors on PF4  
  GPIO_PORTF_DEN_R = 0x1E;          		// 7) enable digital pins PF4-PF1 
	
	GPIO_PORTF_ICR_R = 0X11;
	GPIO_PORTF_IM_R |= 0x11;
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)
								|0x00400000; 						// (g) priority 2
  NVIC_EN0_R = 0x40000000;      				// (h) enable interrupt 30 in NVIC
	
} // end of PortF_Init

void GPIOPortF_Handler(void){ // called on touch of either SW1 (PF4)
  
	if(GPIO_PORTF_RIS_R&0x10){  // SW1 click
		
		// change mode
		GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R << 1; // shift one to the left
		
		// if shift left goes over the green signal, come back to Mode 1
		if(GPIO_PORTF_DATA_R >0x08) GPIO_PORTF_DATA_R = 0x02; 
  } // end of if

	GPIO_PORTF_ICR_R = 0x10;  // acknowledge PF4
} // end of GPIOPortF_Handler

