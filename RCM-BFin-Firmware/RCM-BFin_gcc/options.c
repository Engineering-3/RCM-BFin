/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  options.c -
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
 * Run-time option control for RCM-Bfin firmware
 *
 * Super simple (for now) array access for run time options
 *
 */

#include "options.h"
 
static uint32_t OptionArray[OPT_MAX];
 
// Returns an option's current value as int
int32_t GetOption(OptionType Option)
{
	if (Option < OPT_MAX)
	{
		return (OptionArray[Option]);
	}
	else
	{
		return 0;
	}
}

// Sets an option's value
int32_t SetOption(OptionType Option, int32_t NewValue)
{
	if (Option < OPT_MAX)
	{
		OptionArray[Option] = NewValue;
		return (0);
	}
	else
	{
		return (-1);
	}
}
