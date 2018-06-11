/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  init.c - initalize memory before the rest of the program is loaded
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
 
/* init: initialize external memory
 *
 * The CONFIG_* settings are board specific, but the method for programming
 * them is board independent.  You should be able to copy and paste this file
 * into your project pretty easily.
 *
 * Each function is independent of the other.  You can take only what you need.
 * For more information on each register setting, consult the HRM.
 */

#include <blackfin.h>
#include <cdefBF537.h>
#include "system.h"

/* External memory settings for 32meg chip */
#define CONFIG_EBIU_SDRRC_VAL   0x0817
#define CONFIG_EBIU_SDBCTL_VAL  0x0013
#define CONFIG_EBIU_SDGCTL_VAL  0x0091998d

/* By default, during boot, the SPI baud rate register is set to 133, */
/* which for us will give us 121,649,000/(2 * 133) =  457,327Hz clock.*/
/* This is about 100 times slower than our SPI flash can handle, so we */
/* should bump the divider up to 2, which will then give us a SPI clock */
/* of 121,649,000/(2 * 2) = 30,412,250Hz which is as fas as we can go */
/* without going faster than the 40MHz speed of the part. */
#define CONFIG_SPI_BAUD_VAL     0x0002

static inline void init_program_clocks(void)
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

static inline void init_program_memory_controller(void)
{
    /* Program the external memory controller. */
    *pEBIU_SDRRC  = CONFIG_EBIU_SDRRC_VAL;
    *pEBIU_SDBCTL = CONFIG_EBIU_SDBCTL_VAL;
    *pEBIU_SDGCTL = CONFIG_EBIU_SDGCTL_VAL;
}

__attribute__((saveall))
void initcode(void)
{
    DEBUG_INIT()
    DEBUG_H15_HIGH()    // System Init
    DEBUG_H14_LOW()	    // System Init
    DEBUG_H13_HIGH()    // System Init
    DEBUG_H12_LOW()	    // System Init

    init_program_clocks();
    init_program_memory_controller();

    // Set SPI Baud rate for maximum boot speed
    *pSPI_BAUD = CONFIG_SPI_BAUD_VAL;

    // Turn LED1 on, and LED0 off to show that we're booting
    INIT_LED_IO()
    LED0_OFF()
    LED1_ON()
}
