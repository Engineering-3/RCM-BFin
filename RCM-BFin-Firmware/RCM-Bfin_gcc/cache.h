/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  cache.h -
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
 
#include <stdint.h>

struct cache_page {
	uint32_t addr;
	uint32_t config;
};

int icache_setup(struct cache_page *ctab);

int dcache_setup(struct cache_page *ctab);

void cache_init(void);

void dcache_enable(int which);

void icache_enable(int which);

int cache_status(void);
