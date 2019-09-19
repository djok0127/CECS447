// InputOutput.c
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
#include "inc/tm4c123gh6pm.h"
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
#define SW1       0x10                      // on the left side of the Launchpad board
#define SW2       0x01                      // on the right side of the Launchpad board
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOB			0x00000002	// port B Clock Gating Control
#define SYSCTL_TCGC2_FPIOA			0x00000001	// port A Clock Gating Control
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
uint32_t count;


void PortF_Init(void){ volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
  delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
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

void PortC_Init(void){ volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= 0x00000004;  // 1) activate clock for Port C
  delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
  GPIO_PORTC_DIR_R = 0x1F;          // 5) PC7-0 out
  GPIO_PORTC_DEN_R = 0xFF;          // 7) enable digital I/O on PC7-0
	
	GPIO_PORTC_IS_R &= 0xF0;
	GPIO_PORTC_IBE_R |= 0xF0;			     // trigger control
	GPIO_PORTC_IEV_R |= 0xF0;
	GPIO_PORTC_ICR_R |= 0xF0;				 // clear any prior interrupts
	GPIO_PORTC_IM_R |= 0xF0;				   // unmask interrupt
	// enable interrupt in NVIC and set priority to 5
	NVIC_PRI0_R = 5 << 5;					         // set interrupt priority to 5
	NVIC_EN0_R |= 0x04;				             // enable IRQ1 for port c
}

void PortB_Init(void){ volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= 0x00000002;  // 1) activate clock for Port B
  delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
  GPIO_PORTB_DIR_R = 0xFF;          // 5) PB7-0 out
  GPIO_PORTB_DEN_R = 0xFF;          // 7) enable digital I/O on PB7-0
}

uint32_t PortF_Input(void){     
  return (GPIO_PORTF_DATA_R&0x11);  // read PF4,PF0 inputs
}

void PortF_Output(uint32_t data){ // write PF3-PF1 outputs
  GPIO_PORTF_DATA_R = data;      
}

uint32_t PortC_Input(void){     
  return (GPIO_PORTC_DATA_R&0xF0);  // read PC4-PC7 inputs
}






void Timer1A_Init(void){
	// Timer1A configuration
	SYSCTL_RCGCTIMER_R |= 0x02;	  // enable clock to timer Block 1
	TIMER1_CTL_R = 0;					             	// disable Timer1 during configuration
	TIMER1_CFG_R = 0x04;				          // 16-bit timer
	TIMER1_TAMR_R = 0x02;				        // periodic timer
	TIMER1_TAPR_R = 250;		              // 16MHz/250 = 64kHz
	TIMER1_TAILR_R = 6400;			          // 6.4kHz/64kHz = 1Hz
	TIMER1_ICR_R = 0x01;				            // clear Timer1A timeout flag
	TIMER1_IMR_R |= 0x01;				          // enable Timer1A timeout interrupt 
	TIMER1_CTL_R |= 0x01;				          // enable Timer1A
	NVIC_PRI5_R = (NVIC_PRI5_R & 0xffffff) | 0xc0000000;
	NVIC_EN0_R |= 0x00200000;	        	// enable IRQ21
}

void Timer2A_Init(void){	
	// Timer2A configuration
	SYSCTL_RCGCTIMER_R |= 0x04;	  // enable clock to Timer Block 2
	TIMER2_CTL_R = 0;					          	  // disable Tier2 during configuration
	TIMER2_CFG_R = 0x04;				          // 16-bit timer
	TIMER2_TAMR_R = 0x01;				        // one-shot timer
	TIMER2_TAPR_R = 25;	      	        // 16MHz/250 = 64kHz
	TIMER2_TAILR_R = 64/10;		              	// 64Hz/64kHz
	TIMER2_ICR_R = 0x01;			          	  // clear Timer2A timeout flag
	TIMER2_IMR_R |= 0x01;				          // enable Timer2A timeout interrupt ds.p745
	NVIC_PRI5_R = (NVIC_PRI5_R & 0xffff00ff) | 0x0000a000;
	NVIC_EN0_R |= 0x00800000;		        // enable IRQ23
}
	
void Timer3A_Init(void){
	// Configure Timer3
	SYSCTL_RCGCTIMER_R |= 0x08;	// enable clock to Timer 2
	TIMER3_CTL_R = 0;					// disable Timer2 during config
	TIMER3_CFG_R = 0x04;			// 16-bit mode
	TIMER3_TAMR_R = 0x02;			// periodic up-counter
	TIMER3_TAPR_R = 25;			// 16MHz/250 = 64kHz
	TIMER3_TAILR_R = 64/10;		//10 microseconds
	TIMER3_ICR_R = 0x01;			// clear Timer3A timeout flag
	TIMER3_IMR_R |= 0x01;			// enable Timer3A timeout interrupt
	TIMER3_CTL_R |= 0x01;			// enable Timer3A after config
	NVIC_EN1_R |= 0x00000008;
}


int main(void){
	//Port initialization
	PortB_Init();
	PortC_Init();
	PortF_Init();
	
	// Timer initialization
	Timer1A_Init();
	Timer2A_Init();
	Timer3A_Init();
	
	count = 0; 
	
	
  SYSCTL_RCGCPWM_R |= 1;
	SYSCTL_RCGCGPIO_R |= 0x02;										// start clock for Port B
	SYSCTL_RCC_R |= 0x00160000;										// start the clock divider by 16Mhz/16, 1Mhz clock 
	PortF_Init();
	
	GPIO_PORTB_AFSEL_R = 0x10;										// activate analog of PortB4
	GPIO_PORTB_PCTL_R &= ~0x000F0000;							// clear PCTL for PWM
	GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_M0PWM2;		// setting PCTL
	
	GPIO_PORTB_DEN_R |= 0x10;											// input/output of PortB4
	
	PWM0_1_CTL_R = 0;															// PWM stop counter
	PWM0_1_GENA_R = 0x0000008C;										// M0PWM2 output set when reload
	PWM0_1_LOAD_R = 5000;													// reaload value for 50Hz, 20ms
	PWM0_1_CTL_R = 1;															// start timer
	PWM0_ENABLE_R = 0x04;													// start PWM0 ch2
	
	while(1){
		switch(PortF_Input()){
			case 0x01 : PWM0_1_CMPA_R = 4400;					// match comparator value to make signal low, 90 degrees left
									break;
			case 0x10 : PWM0_1_CMPA_R = 4850;					// match comparator value to make signal low, 90 degrees right
									break;
			default  	: PWM0_1_CMPA_R = 4625;					// match comparator value to make signal low, center
									break;
		}
		
	}
    
}

void GPIOPortC_Handler(void){
	volatile int32_t readback;
	if(((count*10)>0) & ((count*10)<950)){
		PortF_Output(RED);
	}
	else if(((count*10)>=950) & ((count*10)<1750)){
		PortF_Output(GREEN);
	}else if((count*10)>=1750){
		PortF_Output(BLUE);
	}else{}
	GPIO_PORTC_ICR_R |= 0xF0;       //clear the interrupt flag
	readback = GPIO_PORTC_ICR_R;  //read to force interrupt flag clear
}


void Timer1A_Handler(void){
	volatile uint32_t readback;
	GPIO_PORTC_DATA_R |= 0x10;
	TIMER2_CTL_R |= 0x01;				          // enable Timer2A
	TIMER1_ICR_R = 0x01;
	readback = TIMER1_ICR_R;
}

void Timer2A_Handler(void){
	volatile uint32_t readback;
	GPIO_PORTC_DATA_R &= ~(0x10);
	count = 0;
	TIMER2_ICR_R = 0x01;
	readback = TIMER2_ICR_R;
}
void Timer3A_Handler(void){
	volatile uint32_t readback;
	count++;
	TIMER3_ICR_R = 0x01;			// clear interrupt flag	
	readback = TIMER3_ICR_R;
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

