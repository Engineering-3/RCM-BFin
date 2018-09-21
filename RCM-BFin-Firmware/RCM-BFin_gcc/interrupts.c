/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  interrupts.c - some of the interrupt handlers
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
#include "intuart.h"
#include "rcm-bfin.h"
#include "streaming.h"
#include <stdint.h>

volatile uint32_t Counter0 = 0;
volatile uint32_t CounterStreamVideoTimout = 0;
volatile uint32_t I2CDelayTimeoutMS = 0;
volatile uint32_t LEDCounterMS = 0;
volatile uint32_t XMODEMTimeoutMS = 0;
volatile uint32_t I2CTimeoutMS = 0;
volatile uint32_t PicoCDelay = 0;
volatile uint32_t StreamShutdownTimeout = 0;
volatile uint32_t UART0LastByteRX = 0;
volatile uint32_t I2CAutoincTimeoutMS = 0;

//----------------------------------------------------------------------------//
// Function:    Timer0_ISR                                                    //
//                                                                            //
// Parameters:    None                                                        //
//                                                                            //
// Return:        None                                                        //
//                                                                            //
// Description:    This ISR is executed every time Timer0 expires.            //
//                The old LED pattern is shifted by one; the direction        //
//                depends on the state of sLight_Move_Direction, which is     //
//                changed in PORTF_IntA_ISR.                                  //
//----------------------------------------------------------------------------//

__attribute__((interrupt_handler,nesting))
//__attribute__((interrupt_handler))
void Timer0_ISR (void)
{
    volatile uint32_t IntTemp;

    // confirm interrupt handling
    *pTIMER_STATUS = 0x0001;

    // Countdown all registered countdown timer variables
    /// TODO: Make this some type of registered callback thing
    if (Counter0) {Counter0--;}
    if (CounterStreamVideoTimout) {CounterStreamVideoTimout--;}
    if (LEDCounterMS) {LEDCounterMS--;}
    if (XMODEMTimeoutMS) {XMODEMTimeoutMS--;}
    if (I2CTimeoutMS) {I2CTimeoutMS--;}
    if (PicoCDelay) {PicoCDelay--;}
    if (I2CDelayTimeoutMS) {I2CDelayTimeoutMS--;}
    if (I2CAutoincTimeoutMS) {I2CAutoincTimeoutMS--;}
    UART0LastByteRX++;
    StreamingLastTickMS++;
    
    // If we are not transmitting a byte right now,
    // then start things off.
    disable_interrupts(IntTemp);
    if (TXBufCount && (*pUART0_LSR & THRE) && !CTS_TRIGGERED)
    {
        TXSendNextByte();
    }
    enable_interrupts(IntTemp);    

    // Go take care of the times for any enabled streaming sensors
    StreamingTickTrigger = true;
}
