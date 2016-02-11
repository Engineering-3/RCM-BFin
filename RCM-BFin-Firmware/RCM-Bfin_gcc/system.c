/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  system.c - 
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
 
#include <malloc.h>
#include <stdlib.h>
#include <stdbool.h>
#include "system.h"
#include "rcm-bfin.h"

void run_leds(void)
{
	if (!LEDCounterMS)
	{
		LED0_TOGGLE();
		LEDCounterMS = 500;
	}
}

bool        strReplace ( 
char *      dest,
int         destMax,     
char *      pattern,     
char *      replacement)
{
    char * targ = strstr (dest, pattern);
    if (targ != NULL) {
        int destLen = strlen (dest);
        int patLen = strlen (pattern);
        int repLen = strlen (replacement);
        if (destLen + repLen - patLen + 1 > destMax)
            return FALSE;
        memmove (targ + repLen, targ + patLen, (destLen - (targ - dest) - patLen + 1) * sizeof (char));
        memcpy (targ, replacement, repLen);
        return true;
    }        
    else
        return false;
}


unsigned    getUnaligned32 (
void *      p)
{
    unsigned char * c = p;
#ifdef _BIG_ENDIAN
    return c[3] + (c[2] << 8) + (c[1] << 16) + (c[0] << 24);
#else
    return c[0] + (c[1] << 8) + (c[2] << 16) + (c[3] << 24);
#endif
}

unsigned int atoi_b16(char *s) {
    // Convert two ascii hex characters to an int
    unsigned int result = 0;
    int i;
    for(i = 0; i < 2; i++,s++)  {
        if (*s >= 'a')
            result = 16 * result + (*s) - 'a' + 10;
        else if (*s >= 'A')
            result = 16 * result + (*s) - 'A' + 10;
        else
            result = 16 * result + (*s) - '0';
    }
    return result;
}

unsigned int atoi_b10(char *s) {
    // Convert two ascii decimal characters to an int
    unsigned int result = 0;
    int i;
    for(i = 0; i < 2; i++,s++)
        result = 10 * result + (*s) - '0';
    return result;
}

char * strnstr (
        const char * str1,
        const char * str2,
        int n
        )
{
        char *cp = (char *) str1;
        char *s1, *s2;

        if ( !*str2 )
            return(char *)str1;

        while (n > 0)
        {
                s1 = cp;
                s2 = (char *) str2;

                int n2 = n;
                while ( n2 > 0  && *s2 && !(*s1-*s2) )
                        s1++, s2++, --n2;

                if (!*s2)
                        return cp;

                cp++;
                --n;
        }

        return 0;

}

unsigned int ctoi(unsigned char c) {
    if (c > '9')
        return (unsigned int)(c & 0x0F) + 9;
    else
        return (unsigned int)(c & 0x0F);
}

int
_kill (int n, int m)
{
	puts("Called kill()");
	return 0;
}

int
_getpid (void)
{
	return 1;
}

 /* If we get here, something bad happened, so just try to signal the JTAG
 * ICE as hard as we can until someone notices.
 */
void hang(void)
{
	while (1)
		asm("emuexcpt;");
}

uint32_t get_seqstat(void)
{
	uint32_t ret;
	asm("%0 = seqstat;" : "=d"(ret));
	return ret;
}





