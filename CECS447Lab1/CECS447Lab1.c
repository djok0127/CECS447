// CECS447Lab1.c
// Runs on LM4F120/TM4C123
// Test the switch initialization functions by setting the LED
// color according to the status of the switches.
// Daniel and Jonathan Valvano
// May 23, 2014

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Section 4.2    Program 4.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// negative logic switches connected to PF0 and PF4 on the Launchpad
// red LED connected to PF1 on the Launchpad
// blue LED connected to PF2 on the Launchpad
// green LED connected to PF3 on the Launchpad
// NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
// the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
// and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
// is written to the Port F GPIO Lock Register.  After Port F is
// unlocked, bit 0 of the Port F GPIO Commit Register must be set to
// allow access to PF0's control registers.  On the LM4F120, the other
// bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
// that the rest of Port F can always be freely re-configured at any
// time.  Requiring this procedure makes it unlikely to accidentally
// re-configure the JTAG pins as GPIO, which can lock the debugger out
// of the processor and make it permanently unable to be debugged or
// re-programmed.

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
#define SW1       0x01                      // on the left side of the Launchpad board
#define SW2       0x10                      // on the right side of the Launchpad board
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOB			0x00000002	// port B Clock Gating Control
#define SYSCTL_TCGC2_FPIOA			0x00000001	// port A Clock Gating Control
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
// initialize global variables
uint8_t mode = 1; 
uint8_t toggle_c = 1;
uint8_t color = RED;
uint8_t color_sw = 1;


void PortF_Init(void){
	unsigned volatile delay;
	SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
  delay = SYSCTL_RCGCTIMER_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}

void PortF_Inter_Init(void){
	GPIO_PORTF_IS_R &= ~0x01;					// setting for edge trigger
	GPIO_PORTF_IBE_R &= ~0x01;				// setting for one edge trigger
	GPIO_PORTF_IEV_R &= ~0x01;				// setting for falling edge trigger
	GPIO_PORTF_ICR_R |= 0x01;					// set the flags for interrupt
	GPIO_PORTF_IM_R |= 0x01;					// arm interrupt of PF4
	NVIC_PRI7_R = 0 << 21;						// setting the priority of 0
	NVIC_EN0_R = 0x40000000;					// enable the interrupt
}

void Timer0A_Init(void){
	unsigned volatile delay;
	SYSCTL_RCGCTIMER_R |= 0x01;				// activate timer 0
	delay = SYSCTL_RCGCTIMER_R;
	TIMER0_CTL_R = 0;									// disable the timer for setup
	TIMER0_CFG_R = 0x00;							// configure for a 16-bit timer mode
	TIMER0_TAMR_R |= 0x02;						// configure for a periodic timer mode
	TIMER0_TAILR_R = 16000000-1;				// reload time for 1 second 
	TIMER0_ICR_R = 0x01;							// clear timer0 timeout flag
	TIMER0_IMR_R = 0x01;							// arm timerout interrupt
	NVIC_PRI4_R = 1 << 29;						// setting the priority of 1
	NVIC_EN0_R = 1 << 19;							// enable IQR 19(timer0A) in NVIC
	TIMER0_CTL_R |= 0x01;							// enable timer0
}

int main(void){
	//PLL_Init();												// setting the clock frequency of 4MHz
	PortF_Init();
	PortF_Inter_Init();
	Timer0A_Init();
	GPIO_PORTF_DATA_R = RED;
	while(1);
}

void GPIOPortF_Handler(void){
	volatile int readback;
		mode++;
		readback = GPIO_PORTF_DATA_R;
		if(mode > 4)
			mode = 1;
		switch(mode){
			case 1  : color = RED;
								break;
			case 2  : color = BLUE;
								break;
			case 3  : color = GREEN;
								break;
			default : color = RED;
								break;
		}
	GPIO_PORTF_DATA_R = color;
	GPIO_PORTF_ICR_R |= SW1;
	readback = GPIO_PORTF_ICR_R;
}

void Timer0A_Handler(void){
	volatile int readback;
	if(mode == 4){
		color_sw++;
		if(color_sw > 3)
			color_sw = 1;
		switch(color_sw){
			case 1  : color = RED;
								break;
			case 2  : color = BLUE;
								break;
			case 3  : color = GREEN;
								break;
			default : color = RED;
								break;
		}
		GPIO_PORTF_DATA_R = color;
		toggle_c = 0;

	} else {
		GPIO_PORTF_DATA_R ^= color; 
	}
	TIMER0_ICR_R = 0x01;
	readback = TIMER0_ICR_R;
}


// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06
