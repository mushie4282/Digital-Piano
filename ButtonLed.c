// ButtonLed.c: starter file for CECS447 Project 1 Part 2
// Runs on TM4C123,
// Dr. Min He
// September 10, 2022
// Port B bits 5-0 outputs to the 6-bit DAC
// Port D bits 3-0 inputs from piano keys: CDEF:doe ray mi fa,negative logic connections.
// Port F is onboard LaunchPad switches and LED
// SysTick ISR: PF3 is used to implement heartbeat

#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "ButtonLed.h"

// Constants
#define NVIC_EN0_PORTF 0x40000000 // enable PORTF edge interrupt
#define NVIC_EN0_PORTD 0x00000008 // enable PORTD edge interrupt
#define NVIC_EN0_PORTA 0x00000001 // enable PORTA edge interrupt
#define PIANO_KEYS_PORTD 0x4F	  // PD3 - PD0 and PD6 (0100 1111)
#define PIANO_KEYS_PORTA 0x0C	  // PA2 & PA3 (0000 1100)

// Gobals
volatile uint8_t curr_mode = PIANO; // 0: piano mode, 1: auto-play mode

//---------------------Switch_Init---------------------
// initialize onboard switch and LED interface
// Input: none
// Output: none
void ButtonLed_Init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
	while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOF) != SYSCTL_RCGC2_GPIOF)
		;

	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock GPIO Port F
	GPIO_PORTF_CR_R |= 0x1F;		   // allow changes to PF4-0
	GPIO_PORTF_AMSEL_R &= ~0x1F;	   // disable analog on PF4-0
	GPIO_PORTF_PCTL_R &= ~0x000FFFFF;  // PCTL GPIO on PF4-0
	GPIO_PORTF_DIR_R &= ~0x11;		   // PF4,PF0 in, PF3-1 out
	GPIO_PORTF_AFSEL_R &= ~0x1F;	   // disable alt funct on PF4-0
	GPIO_PORTF_PUR_R |= 0x11;		   // enable pull-up on PF0 and PF4
	GPIO_PORTF_DEN_R |= 0x1F;		   // enable digital I/O on PF4-0

	GPIO_PORTF_IS_R &= ~0x11;							   // PF4,PF0 is edge-sensitive
	GPIO_PORTF_IBE_R |= ~0x11;							   // PF4,PF0 is both edges
														   // PF4,PF0 falling edge event
	GPIO_PORTF_ICR_R = 0x11;							   // clear flag 0 & flag 4
	GPIO_PORTF_IM_R |= 0x11;							   // enable interrupt on PF0 & PF4
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF1FFFFF) | 0x00A00000; // (g) bits:23-21 for PORTF, set priority to 5
	NVIC_EN0_R |= NVIC_EN0_PORTF;						   // (h) enable interrupt 30 in NVIC
}

//---------------------PianoKeys_Init---------------------
// initialize onboard Piano keys interface: PORT D 0 - 3 are used to generate
// tones: CDEFG:doe ray mi fa so
// initialized extended piano keys interface : PORT A 2-3  are used to generate
// tones: AB:
// No need to unlock. Only PD7 needs to be unlocked.
// Input: none
// Output: none
void PianoKeys_Init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD; // activate port D
	while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOD) != SYSCTL_RCGC2_GPIOD)
		;

	GPIO_PORTD_AMSEL_R &= ~PIANO_KEYS_PORTD; // disable analog on PD3-0
	GPIO_PORTD_PCTL_R &= ~0xFF00FFFF;		 // PCTL GPIO on PD3-0
	GPIO_PORTD_DIR_R &= ~PIANO_KEYS_PORTD;	 // PD3-0 input
	GPIO_PORTD_AFSEL_R &= ~PIANO_KEYS_PORTD; // disable alt funct on PD3-0
	GPIO_PORTD_PUR_R |= PIANO_KEYS_PORTD;	 // enable pull-up internal resistors on PD3-0
	GPIO_PORTD_PDR_R &= ~PIANO_KEYS_PORTD;	 // disable pull-down internal resistors on PD3-0
	GPIO_PORTD_DEN_R |= PIANO_KEYS_PORTD;	 // enable digital I/O on PD3-0

	GPIO_PORTD_IS_R &= ~PIANO_KEYS_PORTD;					// PD3-0 is edge-sensitive
	GPIO_PORTD_IBE_R |= PIANO_KEYS_PORTD;					// PD3-0 is not both edges
	GPIO_PORTD_ICR_R = PIANO_KEYS_PORTD;					// clear flags PD3-0
	GPIO_PORTD_IM_R |= PIANO_KEYS_PORTD;					// enable interrupt on PD3-0
	NVIC_PRI0_R = (NVIC_PRI0_R & 0x1FFFFFFF) | 0x600000000; // (g) bits:31-29 for PORTD, set priority to 3
	NVIC_EN0_R |= NVIC_EN0_PORTD;							// (h) enable interrupt 3 in NVIC

	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
	while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOA) != SYSCTL_RCGC2_GPIOA)
		;
	GPIO_PORTA_LOCK_R = GPIO_LOCK_KEY; // unlock port A
	GPIO_PORTA_CR_R |= PIANO_KEYS_PORTA;
	GPIO_PORTA_AMSEL_R &= ~PIANO_KEYS_PORTA; // disable analog on PA2-3
	GPIO_PORTA_PCTL_R &= ~0x0000FF00;		 // PCTL GPIO on PA2-3
	GPIO_PORTA_DIR_R &= ~PIANO_KEYS_PORTA;	 // PA2-3 input
	GPIO_PORTA_AFSEL_R &= ~PIANO_KEYS_PORTA; // disable alt funct on PD3-0
	GPIO_PORTA_PUR_R |= PIANO_KEYS_PORTA;	 // enable pull-up internal resistors on PA2-3
	GPIO_PORTA_PDR_R &= ~PIANO_KEYS_PORTA;	 // disable pull-down internal resistors on PA2-3
	GPIO_PORTA_DEN_R |= PIANO_KEYS_PORTA;	 // enable digital I/O on PA2-3

	GPIO_PORTA_IS_R &= ~PIANO_KEYS_PORTA;				   // PA2-3 is edge-sensitive
	GPIO_PORTA_IBE_R |= PIANO_KEYS_PORTA;				   // PA2-3 is not both edges
	GPIO_PORTA_ICR_R = PIANO_KEYS_PORTA;				   // clear flags PA2-3
	GPIO_PORTA_IM_R |= PIANO_KEYS_PORTA;				   // enable interrupt on PA2-3
	NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFFFF1F) | 0x00000060; // (g) bits:31-29 for PORTD, set priority to 3
	NVIC_EN0_R |= NVIC_EN0_PORTA;						   // (h) enable interrupt 3 in NVIC
}

uint8_t get_current_mode(void)
{
	return curr_mode;
}