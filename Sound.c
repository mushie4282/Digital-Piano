// Sound.c
// This is the starter file for CECS 447 Project 1 Part 2
// By Dr. Min He
// September 10, 2022
// Port B bits 5-0 outputs to the 6-bit DAC
// Port D bits 3-0 inputs from piano keys: CDEF:doe ray mi fa,negative logic connections.
// Port F is onboard LaunchPad switches and LED
// SysTick ISR: PF3 is used to implement heartbeat

#include "tm4c123gh6pm.h"
#include "sound.h"
#include <stdint.h>

// define bit addresses for Port B bits 0,1,2,3,4,5 => DAC inputs
#define DAC (*((volatile unsigned long *)0x400050FC))

// 6-bit: value range 0 to 2^6-1=63, 64 samples
const uint8_t SineWave[64] = {32, 35, 38, 41, 44, 47, 49, 52, 54, 56, 58, 59, 61, 62, 62, 63, 63, 63, 62, 62,
							  61, 59, 58, 56, 54, 52, 49, 47, 44, 41, 38, 35, 32, 29, 26, 23, 20, 17, 15, 12,
							  10, 8, 6, 5, 3, 2, 2, 1, 1, 1, 2, 2, 3, 5, 6, 8, 10, 12, 15, 17,
							  20, 23, 26, 29};

// initial values for piano major tones.
// Assume SysTick clock frequency is 16MHz.
const uint32_t tonetab[] =
	// C, D, E, F, G, A, B
	// 1, 2, 3, 4, 5, 6, 7
	// lower C octave:130.813, 146.832,164.814,174.614,195.998, 220,246.942
	// calculate reload value for the whole period:Reload value = Fclk/Ft = 16MHz/Ft
	{	//     C         D         E           F         G           A          B
		30534 * 2, 27211 * 2, 24242 * 2, 22923 * 2, 20408 * 2, 18182 * 2, 16194 * 2, // C4 Major notes
		15289 * 2, 13621 * 2, 12135 * 2, 11454 * 2, 10204 * 2, 9091 * 2, 8099 * 2,	 // C5 Major notes
		7645 * 2, 6810 * 2, 6067 * 2, 5727 * 2, 5102 * 2, 4545 * 2, 4050 * 2};		 // C6 Major notes

// Constants
// index definition for tones used in happy birthday.
enum note_names
{
	C4,
	D4,
	E4,
	F4,
	G4,
	A4,
	B4,
	C5,
	D5,
	E5,
	F5,
	G5,
	A5,
	B5,
	C6,
	D6,
	E6,
	F6,
	G6,
	A6,
	B6
};

#define MAX_NOTES 255 // maximum number of notes for a song to be played in the program
// #define NUM_SONGS 3				  // number of songs in the play list.
#define SILENCE MAX_NOTES		  // use the last valid index to indicate a silence note. The song can only have up to 254 notes.
#define NUM_OCT 3				  // number of octave defined in tontab[]
#define NUM_NOTES_PER_OCT 7		  // number of notes defined for each octave in tonetab
#define NVIC_EN0_PORTF 0x40000000 // enable PORTF edge interrupt
#define NVIC_EN0_PORTD 0x00000008 // enable PORTD edge interrupt
#define NUM_SAMPLES 64			  // number of sample in one sin wave period
#define SW1 0x10				  // bit position for onboard switch 1(left switch)
#define SW2 0x01				  // bit position for onboard switch 2(right switch)
#define PIANO 0					  // piano mode: press a key to play a tone
#define AUTO_PLAY 1				  // auto play mode: automatic playing a song
#define PORTB5_B0 0x3F			  // PB5 - PB0 (0011 1111)

// note table for Happy Birthday
// doe ray mi fa so la ti
// C   D   E  F  G  A  B
NTyp playlist[][MAX_NOTES] =
	{
		// Mary Had A Little Lamb
		{E4, 4, D4, 4, C4, 4, D4, 4, E4, 4, E4, 4, E4, 8,
		 D4, 4, D4, 4, D4, 8, E4, 4, G4, 4, G4, 8,
		 E4, 4, D4, 4, C4, 4, D4, 4, E4, 4, E4, 4, E4, 8,
		 D4, 4, D4, 4, E4, 4, D4, 4, C4, 8, 0, 0},

		// Twinkle Twinkle Little Stars
		{C4, 4, C4, 4, G4, 4, G4, 4, A4, 4, A4, 4, G4, 8, F4, 4, F4, 4, E4, 4, E4, 4, D4, 4, D4, 4, C4, 8,
		 G4, 4, G4, 4, F4, 4, F4, 4, E4, 4, E4, 4, D4, 8, G4, 4, G4, 4, F4, 4, F4, 4, E4, 4, E4, 4, D4, 8,
		 C4, 4, C4, 4, G4, 4, G4, 4, A4, 4, A4, 4, G4, 8, F4, 4, F4, 4, E4, 4, E4, 4, D4, 4, D4, 4, C4, 8, 0, 0},

		// Happy Birthday
		{// so   so   la   so   doe' ti
		 G4, 2, G4, 2, A4, 4, G4, 4, C5, 4, B4, 4,
		 // pause so   so   la   so   ray' doe'
		 SILENCE, 4, G4, 2, G4, 2, A4, 4, G4, 4, D5, 4, C5, 4,
		 // pause so   so   so'  mi'  doe' ti   la
		 SILENCE, 4, G4, 2, G4, 2, G5, 4, E5, 4, C5, 4, B4, 4, A4, 8,
		 // pause fa'  fa'   mi'  doe' ray' doe'  stop
		 SILENCE, 4, F5, 2, F5, 2, E5, 4, C5, 4, D5, 4, C5, 8, SILENCE, 0},
};

// File scope golbal
volatile uint8_t curr_song = 0; // 0: Happy Birthday, 1: Mary Had A Little Lamb. 2: Twinkle Twinkle Little Stars
volatile uint8_t stop_play = 0; // 0: continue playing a song, 1: stop playing a song
volatile uint8_t skip_song = 0; // 0: continue playing a song, 1: skip song
volatile uint8_t octave = 0;	// 0: lower C, 1: middle C, 2: upper C
uint8_t Index; // DAC SineWave array index
extern volatile uint8_t curr_mode;

// **************DAC_Init*********************
// Initialize 6-bit DAC
// Input: none
// Output: none
void DAC_Init(void)
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; // activate Port B
	while ((SYSCTL_RCGC2_R & SYSCTL_RCGC2_GPIOB) != SYSCTL_RCGC2_GPIOB)
		;

	GPIO_PORTB_AMSEL_R &= ~PORTB5_B0; // disable analog on PB5-0
	GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // PCTL GPIO on PB5-0
	GPIO_PORTB_DIR_R |= PORTB5_B0;	  // PB5-0 output (0011 1111)
	GPIO_PORTB_AFSEL_R &= ~PORTB5_B0; // disable alt funct on PB5-0
	GPIO_PORTB_DEN_R |= PORTB5_B0;	  // enable digital I/O on PB5-0
	GPIO_PORTB_DR8R_R |= PORTB5_B0;	  // enable 8 mA drive on PB5-0

	// Sound Initialization: Initialize SysTick periodic interrupts
	Index = 0;
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;						   // disable SysTick during setup
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x1FFFFFFF) | 0x20000000; // priority 1
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN;   // enable SysTick with core clock and interrupts
}

// **************Sound_Start*********************
// Set reload value and enable systick timer
// Input: time duration to be generated in number of machine cycles
// Output: none
void Sound_Start(uint32_t period)
{
	NVIC_ST_RELOAD_R = period - 1; // reload value (frequency)
	NVIC_ST_CURRENT_R = 0;		   // any write to current clears it
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
}

void Sound_stop(void)
{
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
}

// Interrupt service routine
// Executed based on number of sampels in each period
void SysTick_Handler(void)
{
	GPIO_PORTF_DATA_R ^= 0x08; // toggle PF3, debugging
	Index = (Index + 1) % NUM_SAMPLES;
	DAC = SineWave[Index]; // output to DAC: 6-bit data
}

/**
 * @brief Handles the onboard push buttons that will change the system's mode or octive
 *
 * SW1: change system mode
 * SW2: change octave or song
 */
void GPIOPortF_Handler(void)
{
	// simple debouncing code: generate 20ms to 30ms delay
	for (uint32_t time = 0; time < 72724; time++)
	{
	}

	if (GPIO_PORTF_MIS_R & SW1) // change system mode
	{
		GPIO_PORTF_ICR_R = SW1; // acknowledge flag 0
		if (curr_mode == PIANO)
		{
			stop_play = 0; // start playing a song
			curr_mode = AUTO_PLAY;
		}
		else if (curr_mode == AUTO_PLAY)
		{
			stop_play = 1; // stop play a song
			curr_mode = PIANO;
			curr_song = 0; // reset playlist index
		}
	}
	else if (GPIO_PORTF_MIS_R & SW2) // change song or octave
	{
		GPIO_PORTF_ICR_R = SW2; // acknowledge flag 4
		if (curr_mode == AUTO_PLAY) // skip song
		{
			skip_song = 1;
		}
		else if (curr_mode == PIANO) // change octave
		{
			if (octave < 2)
				octave++;
			else
				octave = 0;
		}
	}
}

//---------------------Delay10ms---------------------
// wait 10ms for switches to stop bouncing
// Input: none
// Output: none
void Delay10ms(void)
{
	unsigned long volatile time;
	time = 14545; // 10msec
	while (time)
	{
		time--;
	}
}

// Dependency: Requires PianoKeys_Init to be called first, assume at any time only one key is pressed
// Inputs: None
// Outputs: None
// Description: Rising/Falling edge interrupt on PD6-PD0. Whenever any
// button is pressed, or released the interrupt will trigger.
void GPIOPortD_Handler(void)
{
	// simple debouncing code: generate 20ms to 30ms delay
	for (uint32_t time = 0; time < 72724; time++)
	{
	}

	if (curr_mode == AUTO_PLAY)
		return;

	if (GPIO_PORTD_MIS_R & 0x01) // PD0
	{
		GPIO_PORTD_ICR_R = 0x01;
		if (!(GPIO_PORTD_DATA_R & 0x01))
		{
			Sound_Start(tonetab[(NUM_NOTES_PER_OCT * octave)] / NUM_SAMPLES); // C
		}
		else
		{
			Sound_stop();
		}
	}
	else if (GPIO_PORTD_MIS_R & 0x02) // PD1
	{
		GPIO_PORTD_ICR_R = 0x02;
		if (!(GPIO_PORTD_DATA_R & 0x02))
		{
			Sound_Start(tonetab[(NUM_NOTES_PER_OCT * octave) + 1] / NUM_SAMPLES); // D
		}
		else
		{
			Sound_stop();
		}
	}
	else if (GPIO_PORTD_MIS_R & 0x04) // PD2
	{
		GPIO_PORTD_ICR_R = 0x04;
		if (!(GPIO_PORTD_DATA_R & 0x04))
		{
			Sound_Start(tonetab[(NUM_NOTES_PER_OCT * octave) + 2] / NUM_SAMPLES); // E
		}
		else
		{
			Sound_stop();
		}
	}
	else if (GPIO_PORTD_MIS_R & 0x08) // PD3
	{
		GPIO_PORTD_ICR_R = 0x08;
		if (!(GPIO_PORTD_DATA_R & 0x08))
		{
			Sound_Start(tonetab[(NUM_NOTES_PER_OCT * octave) + 3] / NUM_SAMPLES); // F
		}
		else
		{
			Sound_stop();
		}
	}
	else if (GPIO_PORTD_MIS_R & 0x40) // PD6
	{
		GPIO_PORTD_ICR_R = 0x40;
		if (!(GPIO_PORTD_DATA_R & 0x40))
		{
			Sound_Start(tonetab[(NUM_NOTES_PER_OCT * octave) + 4] / NUM_SAMPLES); // G
		}
		else
		{
			Sound_stop();
		}
	}
	else
	{
		Sound_stop();
	}
	// pause after each play
	Delay10ms();
}
// ****** Piano extended buttons using Port A ********
// Dependancy: Requires PianoKeys_Init to be called first, assume at any time only one key is pressed
// Inputs: None
// Outputs: None
// Description: Rising/Falling edge interrupt on PA2-3. Whenever any
// button is pressed, or released the interrupt will trigger.
void GPIOPortA_Handler(void)
{
	// simple debouncing code: generate 20ms to 30ms delay
	for (uint32_t time = 0; time < 72724; time++)
	{
	}

	if (curr_mode == AUTO_PLAY)
		return;

	if (GPIO_PORTA_MIS_R & 0x04) // PA2
	{
		GPIO_PORTA_ICR_R = 0x04;
		if (!(GPIO_PORTA_DATA_R & 0x04))
		{
			Sound_Start(tonetab[(NUM_NOTES_PER_OCT * octave) + 5] / NUM_SAMPLES); // A
		}
		else
		{
			Sound_stop();
		}
	}
	else if (GPIO_PORTA_MIS_R & 0x08) // PA3
	{
		GPIO_PORTA_ICR_R = 0x08;
		if (!(GPIO_PORTA_DATA_R & 0x08))
		{
			Sound_Start(tonetab[(NUM_NOTES_PER_OCT * octave) + 6] / NUM_SAMPLES); // B
		}
		else
		{
			Sound_stop();
		}
	}
	else
	{
		Sound_stop();
	}
	// pause after each play
	Delay10ms();
}

// Subroutine to wait 0.1 sec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay(void)
{
	uint32_t volatile time;
	time = 727240 * 20 / 91; // 0.1sec
	while (time)
	{
		time--;
	}
}

void play_a_song()
{
	unsigned char i = 0, j;

	while (playlist[curr_song][i].delay && curr_mode == AUTO_PLAY && skip_song == 0)
	{
		// Mode changed from AUTO_PLAY to PIANO in the middle of a note
		if (stop_play)
		{
			Sound_stop();
			return;
		}

		if (playlist[curr_song][i].tone_index == MAX_NOTES)
			Sound_stop(); // silence tone, turn off SysTick timer
		else
		{
			Sound_Start(tonetab[playlist[curr_song][i].tone_index + (octave * NUM_NOTES_PER_OCT)] / NUM_SAMPLES);
		}

		// tempo control: play current note for specified duration
		for (j = 0; j < playlist[curr_song][i].delay; j++)
			Delay();

		Sound_stop();
		i++; // move to the next note
	}

	if (curr_mode == AUTO_PLAY && skip_song == 0)
	{
		// pause after each play
		for (j = 0; j < 15; j++)
			Delay();
	}

	if (skip_song) // check if skip song flag has been set
	{
		if (curr_song < 2)
			curr_song++;
		else
			curr_song = 0;

		skip_song = 0;
	}
}
