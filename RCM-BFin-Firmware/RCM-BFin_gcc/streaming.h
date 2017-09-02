/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  streaming.h - 
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

#ifndef __STREAMING_H__
#define __STREAMING_H__

#include <blackfin.h>
#include <cdefBF537.h>
#include <stdint.h>
#include <stdbool.h>

#define STREAMING_MAX_ANALOGS           64
#define STREAMING_MAX_DIGITAL_BITS      64
#define STREAMING_MAX_DIGITAL_BYTES     64
#define STREAMING_MAX_PICOCVARS         768 // Max is 256 for string, float and int, or 256 * 3

/* I2C read transactions can either be normal (stop after write, start before read phase)
 * or they can be repeated-start type. This enum allows functions the choice. */
typedef enum {
  I2C_NORMAL = 0,
  I2C_REPEATED_START
} I2CRead_Type;

// Global variables
/// TODO: Put this in a local function
extern volatile bool StreamingTickTrigger;
extern volatile uint32_t StreamingLastTickMS;

void StreamingParseAnalogBit(void);
void StreamingParseDigitalBit(I2CRead_Type I2CReadMethod);
void StreamingParseDigitalByte(I2CRead_Type I2CReadMethod);
void StreamingParseHokuyo(void);
void StreamingParsePicoC(void);
uint32_t StreamingProcess(void);
void StreamingTick(void);
void StreamingShutdown(void);
void StopAllStreaming(void);

#endif
