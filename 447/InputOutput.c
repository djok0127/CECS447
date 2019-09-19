// InputOutput.c
// Runs on LM4F120/TM4C123
// Test the switch initialization functions by setting the LED
// color according to the status of the switches.
// Daniel and Jonathan Valvano
// October 24, 2018

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

// Demo Lab 4 on 10/29/18
#include <stdint.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "Nokia5110.h"
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
#define YELLOW    0x0A
#define CYAN      0x0C
#define PINK			0x06
#define STOP_SPEED 4998

//-----------------------------------------------------------------------------
// global variables
//-----------------------------------------------------------------------------
uint32_t period_r = 0;
uint32_t period_l = 0;
uint32_t period_c = 0;
uint32_t first_r = 0;
uint32_t first_l = 0;
uint32_t first_c = 0;
uint32_t distance_r = 0;
uint32_t distance_l = 0;
uint32_t distance_c = 0;
uint32_t first_r_time = 0;
uint32_t first_l_time = 0;
uint32_t first_c_time = 0;
uint32_t second_r_time = 0;
uint32_t second_l_time = 0;
uint32_t second_c_time = 0;
uint32_t first_read=0, second_read=0;
uint16_t speed = 4998;
uint8_t done_r = 0, done_l = 0, done_c = 0, timeout = 0;
uint8_t OutOfRange_r = 0, OutOfRange_l = 0, OutOfRange_c = 0;
uint8_t e_stop = 0;
uint8_t test_dcm = 0;

//--------------------------------------------------------------------------------
// Initialize switch 1 and 2 and LEDs
//--------------------------------------------------------------------------------
void PortF_Init(void){
	unsigned volatile delay;
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
	
	// configure PortF4 for falling edge trigger interrupt
	GPIO_PORTF_IS_R &= ~0x11;
	GPIO_PORTF_IBE_R &= ~0x11;
	GPIO_PORTF_IEV_R &= ~0x11;
	GPIO_PORTF_ICR_R |= 0x11;
	GPIO_PORTF_IM_R |= 0x11;
	
	// enable interrupt in NVIC and set priority to 4
	NVIC_PRI30_R |= 0 << 5;						// setting the priority of 0
	NVIC_EN0_R |= 0x40000000;					// setting 30 IQR 
}

//-------------------------------------------------------------------------------
// setting the pwm of the four wheels
//-------------------------------------------------------------------------------
void PWM_Init(void){
	unsigned volatile delay;
	SYSCTL_RCGCPWM_R |= 0x01;							// activate clock for PWM0 module
	SYSCTL_RCGCGPIO_R |= 0x02;						// activate clock for Port B
	SYSCTL_RCGCGPIO_R |= 0x10;						// activate clock for Port E
	delay = SYSCTL_RCGCGPIO_R;
	delay = SYSCTL_RCGCGPIO_R;
	SYSCTL_RCC_R |= 0x00160000;						// enaable clock divider, 16MHz/64 = 250kHz
	
	GPIO_PORTB_AFSEL_R |= 0x30;						// enable alternate function of PB4,5
	GPIO_PORTB_PCTL_R &= ~0x00FF0000;			// clear PCTL for PWM
	GPIO_PORTB_PCTL_R |= 0x00440000;			// setting PCTL
	GPIO_PORTB_DEN_R |= 0x30;							// output of PB4,5
	
	//PWM0_2_CTL_R = 0;											// disable PWM0 module 2 for setup
	PWM0_1_CTL_R = 0;											// disable PWM0 module 1 for setup
	//PWM0_2_GENA_R = 0x0000008C;			
	PWM0_1_GENA_R = 0x0000008C;
	//PWM0_2_GENB_R = 0x0000080C;
	PWM0_1_GENB_R = 0x0000080C;
	//PWM0_2_LOAD_R = 5000 - 1;						// reload value for 50Hz, 20ms
	PWM0_1_LOAD_R = 5000 - 1;						// reload value for 50Hz, 20ms
	//PWM0_2_CMPA_R = speed - 1;						// setting the duty cycle high for almost entire period
	PWM0_1_CMPA_R = speed - 1;						// setting the duty cycle high for almost entire period
	//PWM0_2_CMPB_R = speed - 1;						// setting the duty cycle high for almost entire period
	PWM0_1_CMPB_R = speed - 1;						// setting the duty cycle high for almost entire period
	//PWM0_2_CTL_R = 1;											// start timer for PWM0 module 0
	PWM0_1_CTL_R = 1;											// start timer for PWM0 module 1
	PWM0_ENABLE_R = 0x3C;									// enable PWM0 Channel 2, 3, 4, & 5
}

//---------------------------------------------------------------------------
// use for direction of the wheels
//---------------------------------------------------------------------------
void PortC_Init(void){
	volatile unsigned long delay;
	SYSCTL_RCGCGPIO_R |= 0x04;
	delay = SYSCTL_RCGCGPIO_R;
	GPIO_PORTC_AMSEL_R &= ~0xC0;
	GPIO_PORTC_PCTL_R &= ~0xFF000000;
	GPIO_PORTC_DIR_R |= 0xC0;
	GPIO_PORTC_AFSEL_R &= ~0xC0;
	GPIO_PORTC_DEN_R |= 0xC0;
	GPIO_PORTC_DATA_R &= ~0xC0;													// change back to |= 0xF0
}

//---------------------------------------------------------------------------
// initialize Pin E3 for ADC
//---------------------------------------------------------------------------
void ADC_InitSeq3Ch9(void){
	volatile unsigned long delay;
	SYSCTL_RCGCGPIO_R |= 0x10;																// activate Port E clock
	while((SYSCTL_PRGPIO_R & 0x10) == 0);
	GPIO_PORTE_DIR_R &= ~0x08;																// make PE3 input
	GPIO_PORTE_AFSEL_R |= 0x08;																// enable alternate function of PE3
	GPIO_PORTE_DEN_R &= ~0x08;																// disable PE4 digital I/O
	GPIO_PORTE_AMSEL_R |= 0x08;																// enable analog function on PE3
	SYSCTL_RCGCADC_R |= 0x01;																	// activate ADC0
	delay = SYSCTL_RCGCADC_R;
	delay = SYSCTL_RCGCADC_R;
	delay = SYSCTL_RCGCADC_R;
	delay = SYSCTL_RCGCADC_R;
	ADC0_PC_R = 0x01;																					// configure for 125k sample rate
	ADC0_SSPRI_R = 0x0123;																		// Seq 3 is highest priority
	ADC0_ACTSS_R &= ~0x008;																		// disable sample seq 3
	ADC0_EMUX_R &= ~0xF000;																		// seq3 is software trigger
	ADC0_SSMUX3_R = (ADC0_SSMUX3_R & 0xFFFFFFF0) + 0;					// setting channel 0 
	ADC0_SSCTL3_R = 0x0006;																		// no on TS0 & D0, yes on IE0 & END0 
	ADC0_IM_R &= ~0x0008;																			// disable SS3 interrupts
	ADC0_ACTSS_R |= 0x0008;																		// enable sample sequencer 3
}

//-------------------------------------------------------------------------
// function to capture the ADC value
//-------------------------------------------------------------------------
uint32_t ADC0_InSeq3(void){
	uint32_t result;
	ADC0_PSSI_R = 0x0008;
	while((ADC0_RIS_R & 0x08) == 0);
	result = ADC0_SSFIFO3_R & 0xFFF;
	ADC0_ISC_R = 0x0008;
	return result;
}

//-----------------------------------------------------------------------------
// use to set a one shot timer
//-----------------------------------------------------------------------------
void Timer1A_Init(void){
	unsigned volatile delay;
	SYSCTL_RCGCTIMER_R |= 0x02;						// activate timer1A block
	delay = SYSCTL_RCGCTIMER_R;
	TIMER1_CTL_R = 0;											// disable timer1A for setup
	TIMER1_CFG_R |= 0x04;									// configure for a 16-bit mode
	TIMER1_TAMR_R |= 0x01;								// configure for one shot mode, count down
}

//------------------------------------------------------------------------------
// function to create a delay in microseconds
//------------------------------------------------------------------------------
void Timer1A_delayUs(uint32_t delay){
	uint32_t x;
	unsigned volatile clock_d;
	SYSCTL_RCGCTIMER_R |= 0x02;						// activate timer1A block
	clock_d = SYSCTL_RCGCTIMER_R;
	for(x=0; x<delay; x++){
		TIMER1_TAILR_R = 16-1;							// set for 1us
		TIMER1_ICR_R = 0x01;								// setting the flag
		TIMER1_CTL_R = 0x01;								// activate the timer
		while((TIMER1_RIS_R & 0x01) == 0);	// polling until the timerflag is set off
		TIMER1_ICR_R = 0x01;								// resetting the flag
	}
}

//------------------------------------------------------------------------------
// to timeout the ultrasonic every 100ms
//------------------------------------------------------------------------------
void Systick_Inter_Init(void){
	NVIC_ST_CTRL_R = 0;								// disable systick for setup
	NVIC_ST_RELOAD_R = 320000;				// set the load for 200Hz,5ms
	NVIC_SYS_PRI3_R = 1 <<29;					// set the priority to level to 3
	NVIC_ST_CTRL_R = 0;								// enable count down, interrupt, systick
}

//-----------------------------------------------------------------------------------------------
// main function
//-----------------------------------------------------------------------------------------------
int main(void){
	PLL_Init();														// initialize PLL for 80MHz
	unsigned volatile delay;
	uint32_t ADC_value;
	uint16_t test_speed;
	
	PortF_Init();
	PWM_Init();														// initilize B4,5 and PE4,5 for speed of DC motors
	PortC_Init();													// initilize PC4-7 for direction of DC motors
	ADC_InitSeq3Ch9();										// initialize PE4 for ADC
	Timer1A_Init();												// initialize timer1A block for polling delays
	//Systick_Inter_Init();									// initialize system tick for 20Hz, 50ms
	
	// i do not know if this is necessary
	//SYSCTL_RCGCGPIO_R |= 0x10;						// activate Port E
	//SYSCTL_RCGCGPIO_R |= 0x20;						// activate Port F
	//SYSCTL_RCGCGPIO_R |= 0x04;						// activate Port C
	//delay = SYSCTL_RCGCGPIO_R;
	
	while(1){
		//-----------------------------------------------------------------
		// Stops the car from moving
		//-----------------------------------------------------------------
		if((distance_c > 200) && (distance_r >200) && (distance_l > 200)){
			PWM0_1_CMPA_R = STOP_SPEED - 1;						// stops the motors
			PWM0_1_CMPB_R = STOP_SPEED - 1;						// stops the motors
			GPIO_PORTF_DATA_R = CYAN;
			while(1);																	// stops the car from moving
		} // end of stops the car from moving

		//----------------------------------------------------------------
		// moving the car backwards
		//----------------------------------------------------------------
		if((distance_c < 11) && (distance_c > 2)){
			GPIO_PORTC_DATA_R = ~GPIO_PORTC_DATA_R;
			PWM0_1_CMPA_R = 1500 - 1;						// setting the duty cycle high for almost entire period
			PWM0_1_CMPB_R = 1500 - 1;						// setting the duty cycle high for almost entire period
			GPIO_PORTF_DATA_R = YELLOW;
			Timer1A_delayUs(500000);
			GPIO_PORTC_DATA_R = ~GPIO_PORTC_DATA_R;
		} // end of going backwards
		
		//----------------------------------------------
		// if E-Stop is activated
		//----------------------------------------------
		while(e_stop){
			GPIO_PORTF_DATA_R = RED;									// set the LED red
			PWM0_1_CMPA_R = STOP_SPEED - 1;						// stops the motors
			PWM0_1_CMPB_R = STOP_SPEED - 1;						// stops the motors
		} // end of e stop
		
		//----------------------------------------------
		// testing DC motors
		//----------------------------------------------
		while(test_dcm){
			GPIO_PORTF_DATA_R = PINK;									// set the LED pink
			ADC_value = ADC0_InSeq3();
			test_speed = ADC_value *0.819;
			if(test_speed >= 4998)
				test_speed = 4998;
			else if(test_speed <= 3)
				test_speed = 3;
			Timer1A_delayUs(100000);
			PWM0_1_CMPA_R = test_speed - 1;						// testting pwm of motors
			PWM0_1_CMPB_R = test_speed - 1;						// testting pwm of motors
		} //  end of test dcm while
	} // end of super loop
	
} // end of main

// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06
