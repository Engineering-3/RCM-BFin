/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  sysinit.c - system intialization routines
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
 *
 *
 */
 
#include <blackfin.h>
#include <cdefBF537.h>
#include "system.h"
#include "config.h"
#include "sysinit.h"


//--------------------------------------------------------------------------//
// Function:	init_program_clocks											//
//																			//
// Parameters:	None														//
//																			//
// Return:		None														//
//																			//
// Description:	Resets the clock PLL. This sets up the core clock as well	//
//				as the peripheral clock										//
//--------------------------------------------------------------------------//
void init_program_clocks(void)
{
	/* Disable all peripheral wakeups except for the PLL event. */
	*pSIC_IWR = 1;

	*pPLL_LOCKCNT = CONFIG_PLL_LOCKCNT_VAL;

	/* Only reprogram when needed to avoid triggering unnecessary
	 * PLL relock sequences.
	 */
	if (*pVR_CTL != CONFIG_VR_CTL_VAL) {
		*pVR_CTL = CONFIG_VR_CTL_VAL;
		asm("idle;");
	}

	*pPLL_DIV = CONFIG_PLL_DIV_VAL;

	/* Only reprogram when needed to avoid triggering unnecessary
	 * PLL relock sequences.
	 */
	if (*pPLL_CTL != CONFIG_PLL_CTL_VAL) {
		*pPLL_CTL = CONFIG_PLL_CTL_VAL;
		asm("idle;");
	}

	/* Restore all peripheral wakeups. */
	*pSIC_IWR = -1;
}


//--------------------------------------------------------------------------//
// Function:	Init_Timers													//
//																			//
// Parameters:	None														//
//																			//
// Return:		None														//
//																			//
// Description:	This function initialises Timer0 for PWM mode.				//
//				It is used as reference for the 'shift-clock'.				//
//--------------------------------------------------------------------------//
void InitTMR0(void)
{
	// Timers count at SCLK rate. (PERIPHERAL_CLOCK)
	*pTIMER0_CONFIG		= 0x0059;
	*pTIMER0_PERIOD		= (PERIPHERAL_CLOCK / 1000);		// Should go every 1ms
	*pTIMER0_WIDTH		= *pTIMER0_PERIOD / 2;
	*pTIMER_ENABLE		= 0x0001;
}
//__attribute__((interrupt_handler, nesting))
__attribute__((interrupt_handler))
void hardware_error_handler(void)
{	
	// Read the register that tells us why we got here
	uint32_t Why = (get_seqstat() & 0x0007C000) >> 14;

	// Now communicate this to the outside world.
	printf("Hardware Error: %02X\r\n", (unsigned int)Why);
//	hang();
}

//--------------------------------------------------------------------------//
// Function:	Init_Interrupts												//
//																			//
// Parameters:	None														//
//																			//
// Return:		None														//
//																			//
// Description:	This function initialises the interrupts for Timer0 and		//
//				PORTF_IntA (PORTF2).										//
//--------------------------------------------------------------------------//
void Init_Interrupts(void)
{
    uint32_t temp;
    // assign core IDs to interrupts - leave everything alone except what we want to tweak
    *pSIC_IAR1 = (*pSIC_IAR1 & 0xFFF00FFF) | (3 << 16) | (3 << 12);     // Uart0 RX and TX -> ID3
    *pSIC_IAR2 = (*pSIC_IAR2 & 0xFFFF0FFF) | (4 << 12);					// Timer0 -> ID4

    // assign ISRs to interrupt vectors
    *pEVT5 = hardware_error_handler;
    *pEVT10 = uart0_ISR;
    *pEVT11 = Timer0_ISR;

    // Enable each interrupt in IMASK register by setting their respective bits
    asm volatile ("cli %0;" : "=d"(temp));
    asm volatile ("bitset (%0, 5);" : "+d"(temp));		// Hardware Error
    asm volatile ("bitset (%0, 10);" : "+d"(temp));		// UART0
    asm volatile ("bitset (%0, 11);" : "+d"(temp));		// TIMER0
    asm volatile ("sti %0; csync;" : : "d"(temp));

    // enable Timer0 interrupt
    *pSIC_IMASK = 0x00080000;   // Timer0 interrupt
}

