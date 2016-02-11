 /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  system.h - system-wide header file for common things
 *
 *  Part of the RCM-BFin GCC v2.0 firmware for the RCM-BFin robot
 *    
 *  Copyright (C) 2015  Engineering^3
 *
 *  Based on the original Surveyor firmware, written and copyrighted
 *  by Surveyor Corp. 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details (www.gnu.org/licenses)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  
/*
 * system.h
 *
 * This header file takes the values that the user has set up in config.h and
 * uses them to turn stuff on or off, or to set the actual values used in
 * the code. This header file needs to be included in every .c file used
 * in the RCM-Bfin firmware.
 */

#ifndef __RCM_BFIN_SYSTEM_H__
#define __RCM_BFIN_SYSTEM_H__

//--------------------------------------------------------------------------//
// Header files
//--------------------------------------------------------------------------//
#include <blackfin.h>
#include <cdefBF537.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "intuart.h"

/*****************************************************************
   System wide defines
   
   These defines are used throughout all of the RCM-Bfin fimrware
*****************************************************************/

#define CPU_BF537    37

#define BOARD_NAME          "RCM-Bfin Blackfin Camera Board"
#define CPU                 CPU_BF537

#if defined(DEBUG)
	#define DEBUG_INIT()        *pPORTHIO_DIR |= PH8 | PH9 | PH10 | PH11 | PH12 | PH13 | PH14 | PH15;
	#define DEBUG_H15_HIGH()    *pPORTHIO_SET = 0x8000;
	#define DEBUG_H14_HIGH()    *pPORTHIO_SET = 0x4000;    
	#define DEBUG_H13_HIGH()    *pPORTHIO_SET = 0x2000;
	#define DEBUG_H12_HIGH()    *pPORTHIO_SET = 0x1000;    
	#define DEBUG_H11_HIGH()    *pPORTHIO_SET = 0x0800;    
	#define DEBUG_H10_HIGH()    *pPORTHIO_SET = 0x0400;    
	#define DEBUG_H9_HIGH()     *pPORTHIO_SET = 0x0200;    
	#define DEBUG_H8_HIGH()     *pPORTHIO_SET = 0x0100;
	#define DEBUG_H15_LOW()     *pPORTHIO_CLEAR = 0x8000;    
	#define DEBUG_H14_LOW()     *pPORTHIO_CLEAR = 0x4000;   
	#define DEBUG_H13_LOW()     *pPORTHIO_CLEAR = 0x2000;    
	#define DEBUG_H12_LOW()     *pPORTHIO_CLEAR = 0x1000;   
	#define DEBUG_H11_LOW()     *pPORTHIO_CLEAR = 0x0800;   
	#define DEBUG_H10_LOW()     *pPORTHIO_CLEAR = 0x0400;   
	#define DEBUG_H9_LOW()      *pPORTHIO_CLEAR = 0x0200;   
	#define DEBUG_H8_LOW()      *pPORTHIO_CLEAR = 0x0100;
	#define DEBUG_H15_TOGGLE()  *pPORTHIO_TOGGLE = 0x8000;
	#define DEBUG_H14_TOGGLE()  *pPORTHIO_TOGGLE = 0x4000;
	#define DEBUG_H13_TOGGLE()  *pPORTHIO_TOGGLE = 0x2000;
	#define DEBUG_H12_TOGGLE()  *pPORTHIO_TOGGLE = 0x1000;
	#define DEBUG_H11_TOGGLE()  *pPORTHIO_TOGGLE = 0x0800;
	#define DEBUG_H10_TOGGLE()  *pPORTHIO_TOGGLE = 0x0400;
	#define DEBUG_H9_TOGGLE()   *pPORTHIO_TOGGLE = 0x0200;
	#define DEBUG_H8_TOGGLE()   *pPORTHIO_TOGGLE = 0x0100;
#else
	#define DEBUG_INIT()
	#define DEBUG_H15_HIGH()
	#define DEBUG_H14_HIGH()
	#define DEBUG_H13_HIGH()
	#define DEBUG_H12_HIGH()
	#define DEBUG_H11_HIGH()
	#define DEBUG_H10_HIGH()
	#define DEBUG_H9_HIGH()
	#define DEBUG_H8_HIGH()
	#define DEBUG_H15_LOW() 
	#define DEBUG_H14_LOW() 
	#define DEBUG_H13_LOW() 
	#define DEBUG_H12_LOW() 
	#define DEBUG_H11_LOW() 
	#define DEBUG_H10_LOW() 
	#define DEBUG_H9_LOW()
	#define DEBUG_H8_LOW()
	#define DEBUG_H15_TOGGLE()
	#define DEBUG_H14_TOGGLE()
	#define DEBUG_H13_TOGGLE()
	#define DEBUG_H12_TOGGLE()
	#define DEBUG_H11_TOGGLE()
	#define DEBUG_H10_TOGGLE()
	#define DEBUG_H9_TOGGLE() 
	#define DEBUG_H8_TOGGLE() 
#endif

// LED defines
#define LED0_BIT        (0x0100)
#define LED1_BIT        (0x0200)

#define INIT_LED_IO()     		*pPORTGIO_DIR = (LED0_BIT | LED1_BIT);   // LEDs (PG8 and PG9)
#define LED0_OFF() 				*pPORTGIO_SET = LED0_BIT;
#define LED1_OFF() 				*pPORTGIO_SET = LED1_BIT;
#define LED0_ON()  				*pPORTGIO_CLEAR = LED0_BIT;
#define LED1_ON()  				*pPORTGIO_CLEAR = LED1_BIT; 
#define LED0_TOGGLE() 			*pPORTGIO_TOGGLE = LED0_BIT;
#define LED1_TOGGLE() 			*pPORTGIO_TOGGLE = LED1_BIT;

#define disable_interrupts(x)	asm volatile("cli %0" : "=r"(x))
#define enable_interrupts(x)	asm volatile("sti %0" : : "r"(x))

	
// FLOW_CTRL_IN (RCM2) = PORTH0 (Blackfin) = RTS0 (Matchport). When low, OK to send bytes out. When high, stop.
#define BIT_RTS0        (1 << 0)
// FLOW_CTRL_OUT (RCM2) = PORTH6 (Blackfin) = CTS0 (Matchport). Clear to allow data to come in from Radio to Blackfin.
#define BIT_CTS0        (1 << 6)

#define ALLOW_RX()       		*pPORTHIO_CLEAR = BIT_CTS0;  // allow incoming data 
#define DISALLOW_RX()    		*pPORTHIO_SET = BIT_CTS0;

#ifndef TRUE
#define TRUE    true
#define FALSE   false
#endif

#ifndef NULL
#define NULL    null
#endif

#ifndef min
#define min(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef BOOL
#define BOOl    bool
#endif


/*
 * UART based defines/formulas

// With Divider of 3, baud rate will be 2,419,156 baud
#define UART0_DIVIDER   (CORE_CLOCK / 16 / UART0_BAUDRATE)
*/

#define UART0_DIVIDER   (1)
#define UART0_FAST_DIVIDER   (CORE_CLOCK / 16 / UART0_FAST_BAUDRATE)
#define UART_DIVIDER   UART0_DIVIDER
#define UART1_DIVIDER   (CORE_CLOCK / 16 / UART1_BAUDRATE)


/*
 * PLL and clock definitions
 */

#define MASTER_CLOCK        22118000	// Crystal speed 22.118 MHz
#define CCLK_DIVIDER        1			// NOTE: not actually used anywhere - default value
 
#define CONFIG_PLL_LOCKCNT_VAL  0x0300
#define CONFIG_VR_CTL_VAL       0x40fb

// Set up the VCO and SCLK values to give us the best baud rate match possible
// for the UARTs.
#if defined(ROUTERBOARD_RADIO_2500000)
	#define VCO_MULTIPLIER  	20
	#define SCLK_DIVIDER    	4
#elif defined(ROUTERBOARD_RADIO_921600)
	#define VCO_MULTIPLIER  	20
	#define SCLK_DIVIDER    	5
#elif defined(MATCHPORT_RADIO)
	#define VCO_MULTIPLIER  	22
	#define SCLK_DIVIDER    	4
#elif defined(MATCHPORT_RADIO_19200)
	#define VCO_MULTIPLIER  	22
	#define SCLK_DIVIDER    	4
#else
	#error You need to define a radio type in config.h
#endif

// MSEL[5:0] = 010110 means VCO = CLKIN * 22
// BYPASS = 0 means do not bypass PLL
// Ouptut Delay = 0 means no output delay
// Input Delay  = 0 means no input delay
// PDWN = 0 means all internal clocks on
// STOPCK = 0 means CLCK on
// PLL_OFF = 0 means enable power to PLL
// DF = 0 means pass CLKIN to PLL
#define CONFIG_PLL_CTL_VAL		(VCO_MULTIPLIER << 9)

// CSEL[1:0] = 00 means CCLK = VCO
// SSEL[3:0] = 0100 means SCLK = VCO/4
#define CONFIG_PLL_DIV_VAL      (SCLK_DIVIDER)

// Important: VCO must stay below the nominal operation frequency of the core
// (normally 500 MHz for the BF parts used on RCM-Bfin)
//
// VCO = (VCO_MULTIPLIER * MASTER_CLOCK) / MCLK_DIVIDER
// where MCLK_DIVIDER = 1 when DF bit = 0,  (default)
//                      2               1
// CCLK = VCO / CCLK_DIVIDER
// SCLK = VCO / SCLK_DIVIDER

// These two values are used throughout the rest of the code to set up
// clock rates for various things (like baud rates, or timers, etc.)
#define CORE_CLOCK (MASTER_CLOCK * VCO_MULTIPLIER / CCLK_DIVIDER)
#define PERIPHERAL_CLOCK  (CORE_CLOCK / SCLK_DIVIDER)


//--------------------------------------------------------------------------//
// Global variables
//--------------------------------------------------------------------------//
extern volatile uint32_t Counter0;
extern volatile uint32_t LEDCounterMS;
extern volatile uint32_t XMODEMTimeoutMS;
extern volatile uint32_t I2CTimeoutMS;
extern volatile uint32_t StreamShutdownTimeout;
extern volatile uint32_t UART0LastByteRX;
extern volatile uint32_t PicoCDelay;

//--------------------------------------------------------------------------//
// System wide function prototypes
//--------------------------------------------------------------------------//
// in file interrupts.c
void Timer0_ISR () __attribute__((interrupt_handler, nesting));
//void Timer0_ISR () __attribute__((interrupt_handler));

void interrupts_init(void);
void exception_init(void);
void hang(void) __attribute__((noreturn));
int picoc(char *SourceStr, unsigned char AllowBackgroundMode);
void httpd_request(char firstChar);
void hang(void);
uint32_t get_seqstat(void);
void ProcessPriorityCommand(uint8_t CharIn);
void MainLoop(bool CalledFromPicoC);


void run_leds(void);

//
// strReplace: replace a pattern in a string with another string (i.e., delete the pattern and insert the new string)
//
extern bool strReplace (        // Returns TRUE if pattern found and replaced successfully
char *      dest,               // Destination string
int         destMax,            // Maximum size of destination string buffer, to accommodate replacement
char *      pattern,            // Pattern to search for
char *      replacement);       // Replacement string



//
// getUnaligned32: get a 32-bit value from an unaligned address
//
extern unsigned getUnaligned32 (    // Returns value
void *          p);                 // Pointer to 32-bit value, need not be 32-bit aligned

extern unsigned int atoi_b16(char *s);
extern unsigned int atoi_b10(char *s);

//
// strnstr: search a buffer of a given size for a string
//
// Similar to strstr, except the string to search is instead a buffer with a count, and embedded zeroes
// in the buffer do not terminate the buffer
//
extern char * strnstr (         // Returns pointer to found string in str1 or NULL
const char * str1,              // Buffer to search. Null characters do NOT terminate the buffer (i.e., it may
                                //  contain embedded null chars that are considered characters)
const char * str2,              // Pattern to search for, null-terminated string
int n);                         // Size of str1 buffer, in characters

#endif



