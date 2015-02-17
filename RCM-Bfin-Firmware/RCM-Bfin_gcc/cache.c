/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  cache.c - 
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
 
 /** \file cache.c
 *
 * Caching support for bare metal apps
 *
 * If you are not calling the bare metal code from a cache managing
 * boot loader such as u-boot, you may want to use these setup routines.
 *
 * IMPORTANT: you can make your system throw core faults if you try
 * to access memory areas that are not covered by the CPLB configuration.
 * Also, when configuring ICACHE for a specific SDRAM region, you can not
 * set breakpoints through the debugger, or you will get an unrecoverable
 * core fault.
 *
 * Note that you will have to use a specific linker script (baremetal.x)
 * and possibly adapt the memory map to your needs.
 * The cacheable and non cacheable memory regions are normally defined
 * in the linker script, so that maximum control over the code relocation
 * is provided. The standard cache initialization uses a data and a program
 * block in sdram memory, see also symbols _sdram_prog and _sdram_data.
 *
 */

#include <cdef_LPBlackfin.h>

#include "cache.h"

#define MB * 0x100000

extern void _sdram_nocache;
extern void _sdram_data;
extern void _sdram_prog;

void dcache_reg_set(unsigned short i, uint32_t addr, uint32_t val)
{
	volatile uint32_t *cfg = pDCPLB_DATA0;
	void * volatile *add = pDCPLB_ADDR0;
	cfg[i] = val;
	add[i] = (void *) addr;
}

void icache_reg_set(unsigned short i, uint32_t addr, uint32_t val)
{
	volatile uint32_t *cfg = pICPLB_DATA0;
	void * volatile *add = pICPLB_ADDR0;
	cfg[i] = val;
	add[i] = (void *) addr;
}

int icache_setup(struct cache_page *ctab)
{
	int i = 0;
	while (ctab->config != 0) {
		icache_reg_set(i, ctab->addr, ctab->config);
		ctab++; i++;
	}
	return i;
}

int dcache_setup(struct cache_page *ctab)
{
	int i = 0;
	while (ctab->config != 0) {
		dcache_reg_set(i, ctab->addr, ctab->config);
		ctab++; i++;
	}
	return i;
}

#define DCACHE_DEFAULT (CPLB_VALID | CPLB_USER_RD \
	| CPLB_WT \
	| CPLB_USER_WR \
	| CPLB_SUPV_WR \
	| CPLB_L1_CHBL )

#define ICACHE_DEFAULT (CPLB_VALID | CPLB_USER_RD \
	| CPLB_L1_CHBL )

#define NOCACHE_DEFAULT (CPLB_VALID \
	| CPLB_USER_RD \
	| CPLB_SUPV_WR \
	)

static struct cache_page
g_default_dcache[] = {
	// These are bloody important. If they're missing, we'll get core faults.
	{ 0xff800000, DCACHE_DEFAULT | CPLB_WT | PAGE_SIZE_4MB },
	{ 0, 0 }
};

static struct cache_page
g_default_icache[] = {
	// These are bloody important. If they're missing, we'll get core faults.
	{ 0xffa00000, CPLB_VALID | CPLB_USER_RD | CPLB_LOCK | PAGE_SIZE_1MB },
	{ 0, 0 }
};

void cache_init(void)
{
	int d, i;
	uint32_t addr;
	d = dcache_setup(g_default_dcache);
	i = icache_setup(g_default_icache);

	// Get addresses from linker script:
//	addr = (uint32_t) &_sdram_prog;
	addr = 0x00000000;
	icache_reg_set(i++, addr, ICACHE_DEFAULT | PAGE_SIZE_1MB);
	// Important: In order to debug with software breaks, we must set the
	// following dcache entry for the above memory range:
	dcache_reg_set(d++, addr, DCACHE_DEFAULT |
		CPLB_L1_AOW | CPLB_WT | PAGE_SIZE_1MB);

	addr = 0x00100000;
//	addr = (uint32_t) &_sdram_data;
	// Cached SDRAM data is from 0x00100000 to 0x01000000 (1MB to 16MB)
	dcache_reg_set(d++, addr, DCACHE_DEFAULT | PAGE_SIZE_1MB); addr += 1 MB;
	dcache_reg_set(d++, addr, DCACHE_DEFAULT | PAGE_SIZE_1MB); addr += 1 MB;
	dcache_reg_set(d++, addr, DCACHE_DEFAULT | PAGE_SIZE_1MB); addr += 1 MB;
	dcache_reg_set(d++, addr, DCACHE_DEFAULT | PAGE_SIZE_4MB); addr += 4 MB;
	dcache_reg_set(d++, addr, DCACHE_DEFAULT | PAGE_SIZE_4MB); addr += 4 MB;
	dcache_reg_set(d++, addr, DCACHE_DEFAULT | PAGE_SIZE_4MB); addr += 4 MB;

	// Non-cached SDRAM data is from 0x01000000 to 0x02000000 (16MB to 32MB)
	addr = 0x01000000;
//	addr = (uint32_t) &_sdram_nocache;
	dcache_reg_set(d++, addr, NOCACHE_DEFAULT | PAGE_SIZE_4MB); addr += 4 MB;
	dcache_reg_set(d++, addr, NOCACHE_DEFAULT | PAGE_SIZE_4MB); addr += 4 MB;
	dcache_reg_set(d++, addr, NOCACHE_DEFAULT | PAGE_SIZE_4MB); addr += 4 MB;
	dcache_reg_set(d++, addr, NOCACHE_DEFAULT | PAGE_SIZE_4MB); addr += 4 MB;

	// All other memory ranges will fire a CPLB exception.
	icache_enable(1);
	dcache_enable(1);
}

void dcache_enable(int which)
{
	if (which) {
		*pDMEM_CONTROL = ACACHE_BCACHE | ENDCPLB | PORT_PREF0;
	} else {
		*pDMEM_CONTROL = 0;
	}
	asm("ssync;");
}

void icache_enable(int which)
{
	if (which) {
		*pIMEM_CONTROL = IMC | ENICPLB;
	} else {
		*pIMEM_CONTROL = 0;
	}
	asm("ssync;");
}

int cache_status(void)
{
	return ((*pIMEM_CONTROL & IMC) << 1 | (*pDMEM_CONTROL & ACACHE_BCACHE));
}
