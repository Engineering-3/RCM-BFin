/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  options.h - 
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

#ifndef __OPTIONS_H__
#define __OPTIONS_H__

#include <blackfin.h>
#include <cdefBF537.h>
#include <stdint.h>

typedef enum {
	OPT_STREAM_VIDEO_ON = 0,
	OPT_PACKET_MODE,
	OPT_VIDEO_TEST_MODE_1,
	OPT_MAX				// This has to be the last one, and is not really an option

} OptionType;

// Returns an option's current value as int
extern int32_t GetOption(OptionType Option);

// Sets an option's value
extern int32_t SetOption(OptionType Option, int32_t NewValue);

#endif
