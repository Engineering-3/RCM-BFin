/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  config.h - Configuration options for building the RCM-Bfin firmware
 *
 *  Part of the RCM-BFin GCC v2.0 firmware for the RCM-BFin robot
 *    
 *  Copyright (C) 2015  Engineering^3
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
 *  System Setup Options
 *  
 *  Use these #defines to set up options for compiling the
 *  firmware.
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <blackfin.h>
#include <cdefBF537.h>

/*
 * Debug options
 */

// Define PICOC2_DEBUG to print out memory copy operation debug information
//#define PICOC2_DEBUG

// Define DEBUG if you want H1-H5 and H7 through H15 (I/O pins) to be available as debug outputs. See system.h for the defines.
#define DEBUG

// Define UART1_PACKET_DEBUG if you want packet mode debug to come out UART1
//#define UART1_PACKET_DEBUG

/*
 * Radio config
 */

// Choose one of the following radio options. You must have one and exactly one defined!
#define MATCHPORT_RADIO
//#define ROUTERBOARD_RADIO_2500000
//#define ROUTERBOARD_RADIO_921600
//#define MATCHPORT_RADIO_19200


/*
 * UART config - normally no need to modify
 */
// Make file may define __RCM_BFIN_UART_BAUDRATE_115200 for one of the builds, so this 
// overrides everything else.
// Otherwise, the choose of radio define (See above) choose the baudrate of UART0. 
#if defined(__RCM_BFIN_UART0_BAUDRATE_115200)
	#define UART0_BAUDRATE 115200
#else
	#if defined(MATCHPORT_RADIO)
		#define UART0_BAUDRATE 2500000
	#elif defined(ROUTERBOARD_RADIO_2500000)
		#define UART0_BAUDRATE 2500000
	#elif defined(ROUTERBOARD_RADIO_921600)
		#define UART0_BAUDRATE 921600
	#elif defined(MATCHPORT_RADIO_19200)
		#define UART0_BAUDRATE 19200
	#endif
#endif

/*
 * Version string
 */
#define RCM_BFIN_GCC_V1_VERSION_STRING "RCM_BFIN GCC Blackfin w/PicoC " PICOC_VERSION " built:" __TIME__ " - " __DATE__ " v2.0test56"

/*
 * I2C speed
 */
// Define this to stay slow, undefine to default to 400Khz
#define TWI_100KHZ


#endif