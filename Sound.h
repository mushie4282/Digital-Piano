// Sound.h,
// This is the starter file for CECS 447 Project 1 Part 2
// By Dr. Min He
// September 10, 2022
// Port B bits 5-0 outputs to the 6-bit DAC
// Port D bits 3-0 inputs from piano keys: CDEF:doe ray mi fa,negative logic connections.
// Port F is onboard LaunchPad switches and LED
// SysTick ISR: PF3 is used to implement heartbeat

#include <stdint.h>

// define music note data structure
struct Note
{
  uint32_t tone_index;
  uint8_t delay;
};
typedef const struct Note NTyp;

// Constants

// **************DAC_Init*********************
// Initialize 3-bit DAC
// Input: none
// Output: none
void DAC_Init(void);

// **************Sound_Start*********************
// Set reload value and enable systick timer
// Input: time duration to be generated in number of machine cycles
// Output: none
void Sound_Start(uint32_t period);
void Sound_stop(void);
void play_a_song();