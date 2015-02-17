/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  hokuyo.h - 
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
 
#ifndef HOKUYO_H
#define HOKUYO_H

#define HOKUYO_PRINT_ASCII 0
#define HOKUYO_PRINT_BINARY 1

/* Function prototypes */
void processHokuyo(unsigned int);
void HokuyoCalibrateStart(void);
void HokuyoSetDeviation(void);
void HokuyoCalibrateEnd(void);
void HokuyoSetVariation(void);
void HokuyoPrintEEPROM(void);
void HokuyoPringHeading(void);
void HokuyoCompassEEPROMClear(void);

/* Globals */

#endif

