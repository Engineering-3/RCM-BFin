/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  rcm-bfin.c - routines to interface with the RCM-Bfin Blackfin robot.
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
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "system.h"
#include "config.h"
#include "intuart.h"
#include "i2c.h"
#include "ov9655.h"
#include "ov7725.h"
#include "camera.h"
#include "jpeg.h"
#include "xmodem.h"
#include "stm_m25p32.h"
#include "font8x8.h"
#include "colors.h"
#include "edit.h"
#include "neural.h"
#include "sdcard.h"
#include "gps.h"
#include "picoc\picoc.h"
#include "options.h"
#include "streaming.h"

#ifdef STEREO
#include "stereo.h"
#endif

#include "rcm-bfin.h"

void _motionvect(unsigned char *, unsigned char *, char *, char *, int, int, int);
extern void PlatformExit(int ExitVal);

// Countdown timer for I2C delay function
extern uint32_t I2CDelayTimeoutMS;

// Countdown timer for I2C autoincriment function
extern uint32_t I2CAutoincTimeoutMS;

/* PicoC state variables */
unsigned int PicoCRunning = FALSE;
signed int pico_int[PICOC_SHARED_ARRAY_SIZE];
char pico_char[PICOC_SHARED_ARRAY_SIZE][PICOC_SHARED_ARRAY_STRING_LENGTH];
double pico_float[PICOC_SHARED_ARRAY_SIZE];

/* PicoC printf() stream buffer */
unsigned int PicoCStreamBufferIn = 0;
unsigned int PicoCStreamBufferOut = 0;
unsigned int PicoCStreamBufferLength = 0;
unsigned char PicoCStreamBuffer[PICOC_STREAM_BUFFER_SIZE];
unsigned char PicoCStreamBufferEnabled = TRUE;

/* Size of frame */
unsigned int imgWidth, imgHeight;

/* stereo vision globals */
#ifdef STEREO
int svs_calibration_offset_x, svs_calibration_offset_y;
int svs_centre_of_disortion_x, svs_centre_of_disortion_y;
int svs_scale_num, svs_scale_denom, svs_coeff_degree;
unsigned int stereo_processing_flag, stereo_sync_flag;
long* svs_coeff;
int svs_right_turn_percent;
int svs_turn_tollerance_percent;
int svs_sensor_width_mmx100;
int svs_width, svs_height;
int svs_enable_horizontal;
int svs_ground_y_percent;
int svs_ground_slope_percent;
int svs_enable_ground_priors;
int svs_disp_left, svs_disp_right, svs_steer;
unsigned int enable_stereo_flag;
int svs_enable_mapping;
#endif /* STEREO */

/* motor move times */
int move_start_time, move_stop_time, move_time_mS;
int robot_moving;

/* Version */
unsigned char version_string[] = RCM_BFIN_GCC_V1_VERSION_STRING;

/* Frame count output string */
unsigned char frame[] = "000-deg 000-f 000-d 000-l 000-r";
//unsigned char frame[] = "frame     ";

/* Camera globals */
unsigned int quality, framecount, ix, overlay_flag;
unsigned int segmentation_flag, edge_detect_flag, frame_diff_flag, horizon_detect_flag;
unsigned int obstacle_detect_flag;
unsigned int blob_display_flag;
unsigned int blob_display_num;
unsigned int edge_thresh;
unsigned int invert_flag;
unsigned int qr_code_detect_flag;
char QRValue[41];   /* Holds the decoded string of the QR code */
unsigned char *output_start, *output_end; /* Framebuffer addresses */
unsigned int image_size; /* JPEG image size */
char imgHead[11]; /* image frame header for I command */

/* Motor globals */
int lspeed, rspeed, lcount, rcount, lspeed2, rspeed2, base_speed, base_speed2, err1;
int pwm1_mode, pwm2_mode, pwm1_init, pwm2_init, xwd_init;
int encoder_flag;

/* IMU globals */
int x_acc, x_acc0, x_center;
int y_acc, y_acc0, y_center;
int compass_init, compass_continuous_calibration, tilt_init;
short cxmin, cymin, cxmax, cymax, czmin, czmax;

/* Failsafe globals */
int failsafe_mode = 0;
int lfailsafe, rfailsafe;
int failsafe_clock;

/* Sonar globals */
int sonar_data[5], sonar_flag = 0;

/* random number generator globals */
unsigned int rand_seed = 0x55555555;

/* General globals */
unsigned char *cp;
unsigned int i, j; // Loop counter.
unsigned int master;  // SVS master or slave ?
unsigned int uart1_flag = 0;

#define PICOC2

#ifdef PICOC2
extern unsigned int _picoc_data_start;
extern unsigned int _picoc_data_end;
extern unsigned int _picoc_bss_start;
extern unsigned int _picoc_bss_end;
extern unsigned int _picoc_rodata_start;
extern unsigned int _picoc_rodata_end;

static unsigned int PicoCCount = 0;
static unsigned int * PicoCDataInitEnd = NULL;
static unsigned int * PicoCDataEnd = NULL;
static unsigned int * PicoCBSSEnd = NULL;
#endif

/// TODO: Get rid of this since we DO not have %p
// Print out a pointer in hex, since we don't have a working %p in printf
void PrintP(unsigned int Ptr)
{
    printf("%04X", (Ptr >> 16));
    printf("%04X", (Ptr & 0xFFFF));
}

void PrintMemCpy(char * Label, unsigned int * InP, unsigned int * OutP, unsigned int Length)
{
    printf(Label);
    printf(" from ");
    PrintP((unsigned int)InP);
    printf(" to ");
    PrintP((unsigned int)OutP);
    printf(" length ");
    PrintP((unsigned int)Length);
    printf("\r\n");
}

// In RAM, we use the area from 0x00080000 to 0x00090000 to store the data and bss sections
// of PICOC. But, we also have to store the initialization data for the data section before
// we execute PICOC for the first time.
// Returns the number of levels deep we are (ie 0 for first level, 1 for 2nd level, etc.)
// Returns negative numbers for error codes

int CopyPicoCOut(void)
{
#ifdef PICOC2
    unsigned int * outp = (unsigned int *)PICOC1_BUF_DATA;
    unsigned int * inp = &_picoc_data_start;
    unsigned int length = 0;
    unsigned int i;
    
    // If this is the first time we've been called, then copy out the PicoC data section
    // to save it off for the future
    if (PicoCCount == 0)
    {
        inp = &_picoc_data_start;
        outp = (unsigned int *)PICOC1_BUF_DATA;
        length = &_picoc_data_end - &_picoc_data_start;
        #ifdef PICOC2_DEBUG
        PrintMemCpy("->Level1 : data", inp, outp, length * 4);
        #endif
        for (i=0; i < length; i++)
        {
            *outp = *inp;
            outp++;
            inp++;
        }
        #ifdef PICOC2_DEBUG
        printf("CopyPicoCOut Level 1 done\r\n");
        #endif
        PicoCCount++;
        PicoCDataInitEnd = outp;
        return(0);
    }

    if (PicoCCount == 1)
    {
        // Copy the whole data section over
        inp = &_picoc_data_start;
        outp = PicoCDataInitEnd;
        length = (&_picoc_data_end - &_picoc_data_start);
        #ifdef PICOC2_DEBUG
        PrintMemCpy("->Level2 : data ", inp, outp, length * 4);
        #endif
        for (i=0; i < length; i++)
        {
            *outp = *inp;
            outp++;
            inp++;
        }

        PicoCDataEnd = outp;

        // And copy the bss section over
        inp = &_picoc_bss_start;
        length = (&_picoc_bss_end - &_picoc_bss_start);
        #ifdef PICOC2_DEBUG
        PrintMemCpy("->Level2 : BSS", inp, outp, length * 4);
        #endif
        for (i=0; i < length; i++)
        {
            *outp = *inp;
            outp++;
            inp++;
        }

        PicoCBSSEnd = outp;
    
        // Then initialize the data section
        // And copy the bss section over
        inp = (unsigned int *)PICOC1_BUF_DATA;
        outp = &_picoc_data_start;
        length = ((unsigned int)PicoCDataInitEnd - PICOC1_BUF_DATA)/4;
        #ifdef PICOC2_DEBUG
        PrintMemCpy("->Level2 : data init", inp, outp, length * 4);
        #endif
        for (i=0; i < length; i++)
        {
            *outp = *inp;
            outp++;
            inp++;
        }
        
        // And clear out the bss section
        // And copy the bss section over
        outp = &_picoc_bss_start;
        length = (&_picoc_bss_end - &_picoc_bss_start);
        #ifdef PICOC2_DEBUG
        PrintMemCpy("->Level2 : bss clear", inp, outp, length * 4);
        #endif
        for (i=0; i < length; i++)
        {
            *outp = 0x0000000;
            outp++;
            inp++;
        }
        
        // Copy the PICOC heap over too
        inp = (unsigned int *)C_HEAPSTART;
        outp = (unsigned int *)PICOC1_BUF_HEAP;
        length = (C_HEAPSIZE/4);
        #ifdef PICOC2_DEBUG
        PrintMemCpy("->Level2 : heap", inp, outp, length * 4);
        #endif
        for (i=0; i < length; i++)
        {
            *outp = *inp;
            outp++;
            inp++;
        }

        #ifdef PICOC2_DEBUG
        printf("CopyPicoCOut level 2 done.\r\n");
        #endif
        
        PicoCCount++;
        return(1);
    }
    #ifdef PICOC2_DEBUG
    printf("CopyPicoCOut error\r\n");
    #endif

#endif    
    // If we're at more than 1, then this is an error and not allowed. Return an error.
    return(-1);
}

void CopyPicoCBack(void)
{
#ifdef PICOC2
    unsigned int * outp = NULL;
    unsigned int * inp = NULL;
    unsigned int length, i;
    
    if (PicoCCount == 2)
    {
        // Copy the old data section back
        inp = PicoCDataInitEnd;
        outp = &_picoc_data_start;
        length = (&_picoc_data_end - &_picoc_data_start);
        #ifdef PICOC2_DEBUG
        PrintMemCpy("<-Level2 : data ", inp, outp, length * 4);
        #endif
        for (i=0; i < length; i++)
        {
            *outp = *inp;
            outp++;
            inp++;
        }

        // And copy the bss section back
        inp = PicoCDataEnd;
        outp = &_picoc_bss_start;
        length = (&_picoc_bss_end - &_picoc_bss_start);
        #ifdef PICOC2_DEBUG
        PrintMemCpy("<-Level2 : bss ", inp, outp, length * 4);
        #endif
        for (i=0; i < length; i++)
        {
            *outp = *inp;
            outp++;
            inp++;
        }

        // Finally, put the heap back
        inp = (unsigned int *)PICOC1_BUF_HEAP;
        outp = (unsigned int *)C_HEAPSTART;
        length = (C_HEAPSIZE/4);
        #ifdef PICOC2_DEBUG
        PrintMemCpy("<-Level2 : heap ", inp, outp, length * 4);
        #endif
        for (i=0; i < length; i++)
        {
            *outp = *inp;
            outp++;
            inp++;
        }
        PicoCCount--;
        #ifdef PICOC2_DEBUG
        printf("CopyPicoCBack - 2 done\r\n");
        #endif
    }
    else
    {
        #ifdef PICOC2_DEBUG
        printf("CopyPicoCBack - 1 done\r\n");
        #endif
        if (PicoCCount)
        {
            PicoCCount--;
        }
    }
#endif
}

void init_io() {
    *pPORTGIO_DIR = 0x0300;   // LEDs (PG8 and PG9)
    *pPORTH_FER = 0x0000;     // set for GPIO
    *pPORTHIO_DIR |= 0x0040;  // set PORTH6 to output for serial flow control
    *pPORTHIO = 0x0000;       // set output low 
    *pPORTHIO_INEN |= 0x000D; // enable inputs: Matchport RTS0 (H0), battery (H2), master/slave (H3)
    *pPORTHIO_DIR |= 0x0380;  // set up lasers - note that GPIO-H8 is used for SD SPI select on RCM board
    
    //*pPORTHIO |= 0x0100;      // set GPIO-H8 high in case it's used for SD SPI select 

    #ifdef STEREO
    if (*pPORTHIO & 0x0008) {  // check SVS master/slave bit
        master = 0;
        *pPORTHIO_DIR &= 0xFEFF;
        *pPORTHIO_INEN |= 0x0100; // enable GPIO-H8 as input on slave for stereo sync
        stereo_sync_flag = *pPORTHIO & 0x0100;
    } else {
        master = 1;
        *pPORTHIO_DIR |= 0x0100;  // set GPIO-H8 as output on master for stereo sync        
    } 
    #endif /* STEREO */

    pwm1_mode = PWM_OFF;
    pwm2_mode = PWM_OFF;
    pwm1_init = 0;
    pwm2_init = 0;
    xwd_init = 0;
    tilt_init = 0;
    sonar_flag = 0;
    edge_detect_flag = 0;
    horizon_detect_flag = 0;
    edge_thresh = 3200;
    obstacle_detect_flag = 0;
    segmentation_flag = 0;
    blob_display_flag = 0;
    qr_code_detect_flag = 0;
    invert_flag = 0;
    encoder_flag = 0;

    /* compass calibration */
    cxmin = cymin = 9999;
    cxmax = cymax = -9999;
    compass_init = 0;
    compass_continuous_calibration = 1;
    
    #ifdef STEREO
    stereo_processing_flag = 0;
    #endif /* STEREO */
}

/* reset CPU */
void __attribute__((section(".l1code"))) reset_cpu() 
//void reset_cpu() 
{
    uint32_t IntTemp;

    delayMS(100);
    
    disable_interrupts(IntTemp);
    SSYNC;
    asm(
		/* Issue system soft reset */ 
		"P0.L = 0x0100 ; "
		"P0.H = 0xFFC0 ; "
		"R0.L = 0x0007 ; "
		"W[P0] = R0 ; "
		"SSYNC ; "
		/*********** 
		"Wait for System reset to complete (needs to be 5 SCLKs). Assuming a worst case CCLK:SCLK ratio (15:1), use 5*15 = 75 as the loop count. 
		"***********/
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		"      NOP ; "
		/* Clear system soft reset */
		"R0.L = 0x0000 ; "
		"W[P0] = R0 ; "
		"SSYNC ; "
		/* Core reset - forces reboot */
		"RAISE 1 ;"
	);
}

/* clear SDRAM */
void clear_sdram() 
{
    memset((uint8_t *)SDRAM_CLEAR_START, 0x00, SDRAM_END - SDRAM_CLEAR_START);
}

void show_stack_ptr() 
{
    uint32_t x = 0;
    asm("%0 = SP;" : "=r"(x) : "0"(x));
    printf("stack_ptr = 0x%p\r\n", (void *)x);
    return;
}

unsigned int stack_remaining() 
{
    unsigned int x = 0;
    asm("%0 = SP" : "=r"(x) : "0"(x));
    return (x - (unsigned int)STACK_BOTTOM);
}

void show_heap_ptr() 
{
//    printf("heap_ptr  = 0x%x\r\n", (int)heap_ptr);
    printf("heap_ptr  = NOT AVAILABLE IN NEW VERSION\r\n");
}

/* RCM-Bfin Firmware Version Request
   Serial protocol char: V */
void serial_out_version(void)
{
    char s[200];
    char * sp = s;
    
    sp += sprintf(sp, "##Version - %s", version_string);
    #ifdef STEREO
    if (master)  // check master/slave bit
        sp += sprintf(sp, " (stereo master)");     
    else
        sp += sprintf(sp, " (stereo slave)");     
    #endif /* STEREO */
    if (GetOption(OPT_PACKET_MODE))
    {
        sp += sprintf(sp, " PKT");
    }
    sp += sprintf(sp, "\r\n");
    SendPacket('V', SENDER_RCM_BFIN, strlen(s), (uint8_t *)s);
}

void serial_out_version_no_packet(void)
{
    char s[200];
    char * sp = s;
    
    sp += sprintf(sp, "##Version - %s", version_string);
    #ifdef STEREO
    if (master)  // check master/slave bit
        sp += sprintf(sp, " (stereo master)");     
    else
        sp += sprintf(sp, " (stereo slave)");     
    #endif /* STEREO */
    if (GetOption(OPT_PACKET_MODE))
    {
        sp += sprintf(sp, " PKT");
    }
    sp += sprintf(sp, "\r\n");
    PacketBegin();
    printf(s);
    PacketEnd(false);
}

/* Get current time
   Serial protocol char: t */
void serial_out_time (void) 
{
    printf("##time - millisecs:  %d\r\n", readRTC());
}

/* load flash sector 4 into flash buffer on startup.  
   If "autorun" is found at beginning of buffer, launch picoC */
void check_for_autorun(void) 
{
    char *cp;
   
    printf("##checking for autorun() in flash sect#4 ... ");
    // Previous to 09/01/2011, we would read in sectors #4 and #5, and then check to see
    // if "autorun" was at the beginning. Instead, now we just read in a few bytes (much faster)
    // and only read in the rest of the sectors if the first part says "autorun". Smart!
    spi_read(USER_FLASH, (unsigned char *)FLASH_BUFFER, 0xFF);  // read 256 bytes to check for first commands
    
    cp = (char *)FLASH_BUFFER;
    if (strncmp("autorun", cp, 7) == 0) 
    {
        printf("autorun() found - launching picoC\r\n");
        // We found something interesting, so read in the entire 2 sectors
        spi_read(USER_FLASH, (unsigned char *)FLASH_BUFFER, TWO_SECTORS);  // read flash sector #4 & #5
        picoc((char *)FLASH_BUFFER, FALSE);
    } 
    else 
    {
        if (strncmp("runback", cp, 7) == 0) 
        {
            // We found something interesting, so read in the entire 2 sectors
            spi_read(USER_FLASH, (unsigned char *)FLASH_BUFFER, TWO_SECTORS);  // read flash sector #4 & #5
            printf("autorun() found - launching picoC\r\n");
            picoc((char *)FLASH_BUFFER, TRUE);
        } 
        else 
        {
            printf("no autorun() found\r\n");
            // clear FLASH_BUFFER. Since we only read in two sectors, we only need to clear two sectors
            memset((uint8_t *)FLASH_BUFFER, 0x00, TWO_SECTORS);
        }
    }
}

/* Dump flash buffer to serial
   Serial protocol char: z-d */
void serial_out_flashbuffer (void) 
{
    printf("##zdump:\r\n");
    cp = (unsigned char *)FLASH_BUFFER;
    for (i = 0; i < FLASH_BUFFER_SIZE; i++) 
    {
        if (*cp == 0)
        {
            return;
        }
        if (*cp == 0x0A)
        {
            uart0SendChar(0x0D);
        }
        uart0SendChar(*cp++);
    }
    // Always end with a CR
    printf("\r\n");
}

/* Dump http buffer to serial
   Serial protocol char: z-d */
void serial_out_httpbuffer (void) 
{
    printf("##zhttp: \r\n");
    cp = (unsigned char *)HTTP_BUFFER;
    for (i = 0; i < HTTP_BUFFER_SIZE; i++) 
    {
        if ((*cp == 0) && (*(cp+1) == 0)) // skips the 0's that were inserted by strtok()
        {
            return;
        }
        if (*cp == 0x0A)
        {
            uart0SendChar(0x0D);
        }
        uart0SendChar(*cp++);
    }
}

/* Turn lasers on
   Serial protocol char: l */
void lasers_on (void) 
{
    *pPORTHIO |= 0x0280;
    printf("#l");
}

/* Turn lasers off
   Serial protocol char: L */
void lasers_off (void) 
{
    *pPORTHIO &= 0xFD7F;
    printf("#L");
}

/* Show laser range
   Serial protocol char: R */
void show_laser_range(int flag) 
{
    printf("##Range(cm) = %d\r\n", laser_range(flag));
}

/* Compute laser range 
    turn off lasers
    stop motors
    delay 250ms
    grab reference frame
    do right laser, then left laser
        turn on laser
        delay 250ms
        grab frame
        turn lasers off 
        compute difference
        set color #16
        use adaptive threshold to filter blobs
        range (inches) = imgWidth*2 / vblob(16)
    compare results, return best measurement
*/
unsigned int laser_range(int dflag) {
    unsigned int ix, rrange, lrange, rconf, lconf;
    
    rrange = lrange = 9999;
    rconf = lconf = 0;    // confidence measure
    
    *pPORTHIO &= 0xFC7F;    // lasers off
    if (pwm1_mode == PWM_PWM) setPWM(0, 0);    // motors off
    else setPPM1(50, 50);
    delayMS(250);
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab reference frame
            (unsigned char *)FRAME_BUF2, imgWidth, imgHeight); 
    *pPORTHIO |= 0x0200;      // right laser on
    delayMS(250);
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    compute_frame_diff((unsigned char *)FRAME_BUF,             // compute difference        
                (unsigned char *)FRAME_BUF2, imgWidth, imgHeight);
    *pPORTHIO &= 0xFC7F;    // lasers off
    umin[16] = 80; umax[16]=144; vmin[16] = 100; vmax[16]=255; ymax[16]=255;   // set color bin #16
    for(ymin[16]=200; ymin[16]>0; ymin[16]-=10) {
        vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, 16);  // use the brightest blob
        if (dflag)
            printf("right blobs: ymin=%d   %d %d %d %d %d  %d %d %d %d %d  %d %d %d %d %d\r\n",
              ymin[16], 
              blobcnt[0], blobx1[0], blobx2[0], bloby1[0], bloby2[0],
              blobcnt[1], blobx1[1], blobx2[1], bloby1[1], bloby2[1],
              blobcnt[2], blobx1[2], blobx2[2], bloby1[2], bloby2[2]);
        ix = blobcnt[0];
        if (!ix)
            continue;
        if (blobx1[0] < (imgWidth/2)) // make certain that blob is on right
            continue;
        break;
    }
    rrange = (6*imgWidth) / (blobx2[0]+blobx1[0]-imgWidth+1); // right blob
    rconf = (100 * ix) / ((blobx2[0]-blobx1[0]+1) * (bloby2[0]-bloby1[0]+1));

    *pPORTHIO |= 0x0080;      // left laser on
    delayMS(250);
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    compute_frame_diff((unsigned char *)FRAME_BUF,             // compute difference        
                (unsigned char *)FRAME_BUF2, imgWidth, imgHeight);
    *pPORTHIO &= 0xFC7F;    // lasers off
    umin[16] = 80; umax[16]=144; vmin[16] = 100; vmax[16]=255; ymax[16]=255;   // set color bin #16
    for(ymin[16]=200; ymin[16]>0; ymin[16]-=10) {
        vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, 16);  // use the brightest blob
        if (dflag)
            printf("left blobs: ymin=%d   %d %d %d %d %d  %d %d %d %d %d  %d %d %d %d %d\r\n",
              ymin[16], 
              blobcnt[0], blobx1[0], blobx2[0], bloby1[0], bloby2[0],
              blobcnt[1], blobx1[1], blobx2[1], bloby1[1], bloby2[1],
              blobcnt[2], blobx1[2], blobx2[2], bloby1[2], bloby2[2]);
        ix = blobcnt[0];
        if (!ix)
            continue;
        if (blobx2[0] > (imgWidth/2)) // make certain that blob is on left
            continue;
        break;
    }
    lrange = (6*imgWidth) / (imgWidth-blobx2[0]-blobx1[0]+ 1); // left blob
    lconf = (100 * ix) / ((blobx2[0]-blobx1[0]+1) * (bloby2[0]-bloby1[0]+1));
    
    if (dflag)
        printf("lconf %d lrange %d rconf %d rrange %d\r\n", lconf, lrange, rconf, rrange);
    if ((lrange==9999) && (rrange==9999))
        return 9999;
    if (lconf > rconf)
        return lrange;
    return rrange;
}

void check_battery() { // 'D' command
    if (*pPORTHIO & 0x0004)
        printf("##D - low battery voltage detected\r\n");
    else
        printf("##D - battery voltage okay\r\n");
}

/* init LIS3LV02DQ 3-axis tilt sensor with i2c device id 0x1D */
void init_tilt()
{
    unsigned char i2c_data[3], device_id;
    
    device_id = 0x1D;
    i2c_data[0] = 0x20;
    i2c_data[1] = 0x87;
    i2cwritex(device_id, (unsigned char *)i2c_data, 2, SCCB_OFF);
    delayMS(10);
    tilt_init = 1;
}

unsigned int tilt(unsigned int channel)
{
    unsigned char i2c_data[2], ch1;
    unsigned int ix;
    
    if (!tilt_init)
        init_tilt();
    switch(channel) {
        case 1:  // x axis
            ch1 = 0x28;
            break;  
        case 2:  // y axis
            ch1 = 0x2A;
            break;  
        case 3:  // z axis
            ch1 = 0x2C;
            break;  
        default:
            return 0;  // invalid channel
    }
    i2c_data[0] = ch1;
    i2cread(0x1D, (unsigned char *)i2c_data, 1, SCCB_ON);
    ix = (unsigned int)i2c_data[0];
    i2c_data[0] = ch1 + 1;
    delayUS(1000);
    i2cread(0x1D, (unsigned char *)i2c_data, 1, SCCB_ON);
    ix += (unsigned int)i2c_data[0] << 8;
    return ix;
}

void read_tilt()
{
    unsigned int channel;
    channel = (unsigned int)(getch() & 0x0F);
    printf("##$T%d %4d\r\n", channel, tilt(channel));
}

void show_compass2x()
{
    unsigned char i2c_data[3];
    unsigned int ix;

    i2c_data[0] = 0x41;  // read compass twice to clear last reading
    i2cread(0x22, (unsigned char *)i2c_data, 2, SCCB_ON);
    delayUS(20000);
    i2c_data[0] = 0x41;
    i2cread(0x22, (unsigned char *)i2c_data, 2, SCCB_ON);
    ix = (((unsigned int)i2c_data[0] << 8) + i2c_data[1]) / 10;
    printf("##$C %3d\r\n", ix);
}

void show_compass3x() {
    short x, y, z, head;
    
    head = read_compass3x(&x, &y, &z);
    printf("##c heading=%d  x=%d y=%d z=%d  xmin=%d xmax=%d ymin=%d ymax=%d\r\n", 
       head, x, y, z, cxmin, cxmax, cymin, cymax);
}

short read_compass3x(short *x, short *y, short *z) {
    unsigned char i2c_data[12];
    short i;
    unsigned char addr;
    short xx, yy, sy, sx, ang;
    
    delayMS(50);  // limit updates to 20Hz

    // HMC5843
    addr = 0x1E;
    if (compass_init == 0) {
        /// TODO: Should this be a i2cwritex() call? We only seem to need to write 2 bytes, but yet we're asking i2cwrite() to write 4 (2 paris)
        i2c_data[0] = 0x00; i2c_data[1] = 0x20; i2cwrite(addr, (unsigned char *)i2c_data, 2, SCCB_OFF);
        i2c_data[0] = 0x01; i2c_data[1] = 0x20; i2cwrite(addr, (unsigned char *)i2c_data, 2, SCCB_OFF);
        i2c_data[0] = 0x02; i2c_data[1] = 0x00; i2cwrite(addr, (unsigned char *)i2c_data, 2, SCCB_OFF);
        delayMS(10);
        compass_init = 1;
    }
    for(i = 0; i < 6; i++) i2c_data[i] = 0x00;
    i2c_data[0] = 0x03;
    i2cread(addr, (unsigned char *)i2c_data, 6, SCCB_ON);
    *x = ((short) (i2c_data[0] * 256 + i2c_data[1]));
    *y = ((short) (i2c_data[2] * 256 + i2c_data[3]));
    *z = ((short) (i2c_data[4] * 256 + i2c_data[5]));
    if ((*x == (short)0xFFFF) && (*y == (short)0xFFFF) && (*z == (short)0xFFFF))  // compass read failed - return 999
        return (999);  
    if (compass_continuous_calibration) {  // this is turned off by compassxcal() C function or $y console function
        if (cxmin > *x)
            cxmin = *x;
        if (cymin > *y)
            cymin = *y;
        if (cxmax < *x)
            cxmax = *x;
        if (cymax < *y)
            cymax = *y;
    }

    /* now compute heading */
    sx = sy = 0;  // sign bits

    yy = *y - (cymax + cymin) / 2;
    if (yy < 0) { sy = 1; yy = -yy; }

    xx = *x - (cxmax + cxmin) / 2;
    if (xx < 0) { sx = 1; xx = -xx; }
    
    ang = (short)GPSatan((int)yy, (int)xx);
    if ((sx==0) && (sy==0))
        ang = 90 - ang;
    if ((sx==0) && (sy==1))
        ang = 90 + ang;
    if ((sx==1) && (sy==1))
        ang = 270 - ang;
    if ((sx==1) && (sy==0))
        ang = 270 + ang;
    i = 360 - ang;  // compass angles go in opposite direction of trig angles
    i += 90;  // shift by 90-deg for compass placement
    if (i >= 360)
        i -= 360;
    return (i); 
}

/* 
 * Reads analog channel 'ix'
 * Returns 12 bit analog value (10-bit result from
 *   RCM2 is left shifted by 2 bits)
 *
 * NAV board AD7998 at I2C 0x20 is channels 01-08
 * RCM AD7998 at I2C 0x23 is channels 11-18 
 * RCM AD7988 at I2C 0x24 is channels 21-28 
 * RCM2 Left Analog 1-4 are channels 31-34
 * RCM2 Right Analog 1-4 are channels 35-38
 *
 * Returns 0xFFFF on error.
 */
unsigned int analog(unsigned int ix)
{
    unsigned char i2c_data[3], device_id;
    uint32_t RetVal = 0;
    unsigned int channel;
    unsigned char mask[] = { 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0 };
    
    // decide which i2c device based on channel range
    if ((ix < 1) || (ix > 38))
    {
        return 0xFFFF;  // invalid channel
    }
    
    if (ix < 30)
    {
        // Handle AD7998 I2C chips
        device_id = 0;
        switch (ix / 10) 
        {
            case 0:
                device_id = 0x20;  // channels 1-8
                break;
            case 1:
                device_id = 0x23;  // channels 11-18
                break;
            case 2:
                device_id = 0x24;  // channels 21-28
                break;
        }
        channel = ix % 10;
        if ((channel<1) || (channel>8))
        {
            return 0xFFFF;  // invalid channel
        }

        // set analog channel 
        i2c_data[0] = mask[channel-1];
        i2c_data[1] = 0;
        i2cwritex(device_id, (unsigned char *)i2c_data, 2, SCCB_OFF);

        // small delay
        delayUS(10);

        // read data
        i2c_data[0] = 0x00;
        i2cread(device_id, (unsigned char *)i2c_data, 2, SCCB_OFF);
        RetVal = (((i2c_data[0] & 0x0F) << 8) + i2c_data[1]);
    }
    else
    {
        // Handle RCM2 ARM on I2C bus
        // We map analog 'channels' 31-38 to I2C register reads 0x30 to 0x37
        i2c_data[0] = (uint8_t)(ix + 0x11);
        i2cread(RCM2_I2C_ADDRESS, (uint8_t *)i2c_data, 2, SCCB_OFF);
        RetVal = ((i2c_data[0] << 8) + i2c_data[1]) << 2;
    }
    return RetVal;
}

void read_analog()
{
    unsigned int channel;
    channel = ((unsigned int)(getch() & 0x0F) * 10) + (unsigned int)(getch() & 0x0F);
    printf("##$A%d %04d\r\n", channel, analog(channel));
}

unsigned int analog_4wd(unsigned int ix)
{
    int t0, ii, val;
    unsigned char ch;
    
    if (xwd_init == 0) {
        xwd_init = 1;
        init_uart1(115200);
        delayMS(10);
    }
    uart1SendChar('a');
    uart1SendChar((char)(ix + 0x30));

    ii = 1000;
    val = 0;
    while (ii) {
        t0 = readRTC();
        while (!uart1GetChar(&ch))
            if ((readRTC() - t0) > 10)  // 10msec timeout
                return 0;
        if ((ch < '0') || (ch > '9'))
            continue;
        val += (unsigned int)(ch & 0x0F) * ii;        
        ii /= 10;
    }
    while (1) {  // flush the UART1 receive buffer
        t0 = readRTC();
        while (!uart1GetChar(&ch))
            if ((readRTC() - t0) > 10)  // 10msec timeout
                return 0;
        if (ch == '\n')
            break;
    }
    return val;
}

void read_analog_4wd()
{
    unsigned int channel;
    channel = (unsigned int)(getch() & 0x0F);
    printf("##$a%d %04d\r\n", channel, analog_4wd(channel));
}

/* use GPIO H10 (pin 27), H11 (pin 28), H12 (pin 29), H13 (pin 30) as sonar inputs -
    GPIO H1 (pin 18) is used to trigger the sonar reading (low-to-high transition) */
void init_sonar() {  
    *pPORTHIO_INEN &= 0xC3FF;
    *pPORTHIO_INEN |= 0x3C00;  // enable H27, H28, H29, H30 as inputs
    *pPORTHIO_DIR |= 0x0002;  // set up sonar trigger
    *pPORTHIO &= 0xFFFD;       // force H1 low
    initTMR4();
}

void ping_sonar() {
    sonar();
    printf("##ping %d %d %d %d\r\n", sonar_data[1], sonar_data[2], sonar_data[3], sonar_data[4]);
}

void sonar() {
    int t0, t1, t2, t3, t4, x1, x2, x3, x4, imask;
    
    if (!sonar_flag) {
        sonar_flag = 1;
        init_sonar();
    }
    
    imask = *pPORTHIO & 0x3C00;
    *pPORTHIO |= 0x0002;       // force H1 high to trigger sonars
    t0 = readRTC();
    t1 = t2 = t3 = t4 = *pTIMER4_COUNTER;
    x1 = x2 = x3 = x4 = 0;
    
    while ((readRTC() - t0) < 40) {
        if ((*pPORTHIO & 0x0400)) 
            x1 = *pTIMER4_COUNTER;
        if ((*pPORTHIO & 0x0800)) 
            x2 = *pTIMER4_COUNTER;
        if ((*pPORTHIO & 0x1000)) 
            x3 = *pTIMER4_COUNTER;
        if ((*pPORTHIO & 0x2000)) 
            x4 = *pTIMER4_COUNTER;
    }

    *pPORTHIO &= 0xFFFD;       // force H1 low to disable sonar
    if (imask & 0x0400) {
        x1 = 0;
    }else {
        if (x1 < t1)
            t1 -= PERIPHERAL_CLOCK;
        x1 = (x1 - t1) / 200;
    }
    if (imask & 0x0800) {
        x2 = 0;
    } else {
        if (x2 < t2)
            t2 -= PERIPHERAL_CLOCK;
        x2 = (x2 - t2) / 200;
    }
    if (imask & 0x1000) {
        x3 = 0;
    } else {
        if (x3 < t3)
            t3 -= PERIPHERAL_CLOCK;
        x3 = (x3 - t3) / 200;
    }
    if (imask & 0x2000) {
        x4 = 0;
    } else {
        if (x4 < t4)
            t1 -= PERIPHERAL_CLOCK;
        x4 = (x4 - t4) / 200;
    }

    sonar_data[0] = sonar_data[1] = x1;  // should fix this - we are counting 1-4, not 0-3
    sonar_data[2] = x2;
    sonar_data[3] = x3;
    sonar_data[4] = x4;
}

void enable_frame_diff() {
    frame_diff_flag = 1;
    grab_reference_frame();
    printf("##g0");
}

void enable_segmentation() {
    segmentation_flag = 1;
    printf("##g1");
}

void enable_edge_detect() {
    edge_detect_flag = 1;
    edge_thresh = 3200;
    printf("##g2");
}

void enable_horizon_detect() {
    horizon_detect_flag = 1;
    edge_thresh = 3200;
    printf("##g3");
}

void enable_obstacle_detect() {
    obstacle_detect_flag = 1;
    edge_thresh = 3200;
    printf("##g4");
}

void enable_blob_display() {
    unsigned int ix;
    char cbuf[2];
    
    cbuf[0] = getch();
    cbuf[1] = 0;
    ix = atoi(cbuf);  // find out which color bin to use with vblob()
    printf("##g6 bin# %x\r\n", ix);
    blob_display_flag = 1;
    blob_display_num = ix;
}

void enable_qr_code_detection() {
  printf("##g7");
  qr_code_detect_flag = 1;
  overlay_flag = 1;
}

#ifdef STEREO
void enable_stereo_processing() {
    if (master) {
        stereo_processing_flag = 1;
        printf("##g5");
    }
}

unsigned int check_stereo_sync() {
    return (*pPORTHIO & 0x0100);
}
#endif /* STEREO */

void set_edge_thresh () {
    unsigned char ch;
    ch = getch();
    edge_thresh = (unsigned int)(ch & 0x0f) * 800;
    printf("#T");
}

void disable_frame_diff() {  // disables frame differencing, edge detect and color segmentation
    frame_diff_flag = 0;
    segmentation_flag = 0;
    edge_detect_flag = 0;
    horizon_detect_flag = 0;
    obstacle_detect_flag = 0;
    blob_display_flag = 0;
    qr_code_detect_flag = 0;
    #ifdef STEREO
    stereo_processing_flag = 0;
    #endif /* STEREO */
    printf("#g_");
}

void grab_frame() {
  unsigned int vect[16];
  int slope, intercept;
  unsigned int ix, ii;
  
  #ifdef STEREO
  if (stereo_processing_flag != 0) {
    svs_stereo(0);
    return;
  }
  #endif /* STEREO */

  if (invert_flag)
    move_inverted((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab and flip new frame
        (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
  else
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
        (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
  if (frame_diff_flag) {
    compute_frame_diff((unsigned char *)FRAME_BUF, 
            (unsigned char *)FRAME_BUF2, imgWidth, imgHeight);
  } else if (segmentation_flag) {
    color_segment((unsigned char *)FRAME_BUF);
  } else if (edge_detect_flag) {
    svs_segcode((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh);
    svs_segview((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF);
  } else if (horizon_detect_flag) {
    vhorizon((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh, 
            16, &vect[0], &slope, &intercept, 5);
    addline((unsigned char *)FRAME_BUF, slope, intercept);
  } else if (obstacle_detect_flag) {
    vscan((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh, 16, &vect[0]);
    addvect((unsigned char *)FRAME_BUF, 16, &vect[0]);
  } else if (blob_display_flag) {
    ix = vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, blob_display_num);
    if (ix > 7)  // only show 8 largest blobs
        ix = 7;
    for (ii=0; ii<ix; ii++) {
      addbox((unsigned char *)FRAME_BUF, blobx1[ii], blobx2[ii], bloby1[ii], bloby2[ii]);
    }
  } else if (qr_code_detect_flag) {
    if (process_qr_detect((unsigned char *)FRAME_BUF, QRValue)) {
      // We found a valid QR code!
      addbox((unsigned char *)FRAME_BUF, blobx1[0], blobx2[0], bloby1[0], bloby2[0]);
      PacketBegin();
      printf("##g:7 '%s'", QRValue);
      PacketEnd(true);
    }
    else
    {
      // No QR code found
      QRValue[0] = 0x00;    // Clear out the overlay text string
    }
  }
  ///// JUST FOR TESTING ////
  // I don't know why these next three lines are necessary. But right before going to Australia 
  // (6/24/2019) while adding the QR code functions, we had to move the BFin stack from the fast
  // scratch pad RAM (4K) to normal SDRAM (512K) because we needed more of it. This somehow
  // affected the video buffer coming from the camera, where it would get out of sync and
  // the images would get shifted to the right and up. Adding these three lines appears to 
  // solve that problem. It is unknown if this actually affects performance, but I'm guessing not.
  camera_stop();
  camera_init((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, imgWidth, imgHeight);
  camera_start();
}


void grab_reference_frame () {
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, 
            (unsigned char *)FRAME_BUF2, imgWidth, imgHeight); 
}

void motion_vect_test (int srange) {  
    char hvect[300], vvect[300];
    int ix, iy, hb, vb;
    
    if (srange < 1) srange = 1;
    if (srange > 3) srange = 3;
    hb = imgWidth / 16;
    vb = imgHeight / 16;
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    copy_image((unsigned char *)FRAME_BUF4, (unsigned char *)FRAME_BUF3, imgWidth, imgHeight);
    move_yuv422_to_planar((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF4, imgWidth, imgHeight);
    _motionvect((unsigned char *)FRAME_BUF4, (unsigned char *)FRAME_BUF3, 
        vvect, hvect, (int)imgWidth, (int)imgHeight, 3);
    for (iy=0; iy<vb; iy++) {
        for (ix=0; ix<hb; ix++) {
            printf("%2d %2d  ", hvect[iy*hb + ix], vvect[iy*hb + ix]);
        }
        printf("\r\n");
    }
}

void motion_vect80x64 () {  
    char hvect[20], vvect[20], hsum[20], vsum[20];
    int ix, iy;
    
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    copy_image((unsigned char *)FRAME_BUF4, (unsigned char *)FRAME_BUF3, 80, 192);
    scale_image_to_80x64_planar ((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF4, imgWidth, imgHeight);
    _motionvect((unsigned char *)FRAME_BUF4, (unsigned char *)FRAME_BUF3, vvect, hvect, 80, 64, 3);
    for (ix=0; ix<20; ix++) {
        hsum[ix] = hvect[ix];
        vsum[ix] = vvect[ix];
    }
    _motionvect((unsigned char *)FRAME_BUF4+5120, (unsigned char *)FRAME_BUF3+5120, vvect, hvect, 80, 64, 3);
    for (ix=0; ix<20; ix++) {
        hsum[ix] += hvect[ix];
        vsum[ix] += vvect[ix];
    }
    _motionvect((unsigned char *)FRAME_BUF4+10240, (unsigned char *)FRAME_BUF3+10240, vvect, hvect, 80, 64, 3);
    for (ix=0; ix<20; ix++) {
        hsum[ix] += hvect[ix];
        vsum[ix] += vvect[ix];
    }
    for (iy=0; iy<4; iy++) {
        for (ix=0; ix<5; ix++) {
            printf("%2d %2d  ", hsum[iy*5 + ix], vsum[iy*5 + ix]);
        }
        printf("\r\n");
    }
}

/*  compute frame difference between two frames 
     U and V are computed by U1 + 128 - U2
     Y is computed as abs(Y1 - Y2) 
     fcur is current frame
     fref is reference frame*/
void compute_frame_diff(unsigned char *fcur, unsigned char *fref, int w1, int h1) {
    int ix, ipix;
    
    ipix = w1*h1*2;
    for (ix=0; ix<ipix; ix+=2) {
        fcur[ix] = (unsigned char)((unsigned int)fcur[ix] - (unsigned int)fref[ix] + 128);
        if (fcur[ix+1] < fref[ix+1])
            fcur[ix+1] = fref[ix+1] - fcur[ix+1];
        else
            fcur[ix+1] = fcur[ix+1] - fref[ix+1];
    }
}

/* JPEG compress and send frame captured by grab_frame()
   Serial protocol char: I */
void send_frame() {
#if 0
    unsigned char i2c_data[2];
    unsigned int ix;
#endif

    if (overlay_flag) {
        //frame[9] = (framecount % 10) + 0x30;
        //frame[8] = ((framecount/10)% 10) + 0x30;
        //frame[7] = ((framecount/100)% 10) + 0x30;

#if 0 // We don't need any of this 'old' stuff to be on the display rihgt now
        i2c_data[0] = 0x41;  // read compass twice to clear last reading
        i2cread(0x22, (unsigned char *)i2c_data, 2, SCCB_ON);
        i2c_data[0] = 0x41;
        i2cread(0x22, (unsigned char *)i2c_data, 2, SCCB_ON);
        ix = ((i2c_data[0] << 8) + i2c_data[1]) / 10;

        frame[2] = (ix % 10) + 0x30;
        frame[1] = ((ix/10)% 10) + 0x30;
        frame[0] = ((ix/100)% 10) + 0x30;

        sonar();
        ix = sonar_data[1] / 100;
        frame[10] = (ix % 10) + 0x30;
        frame[9] = ((ix/10)% 10) + 0x30;
        frame[8] = ((ix/100)% 10) + 0x30;
        ix = sonar_data[2] / 100;
        frame[16] = (ix % 10) + 0x30;
        frame[15] = ((ix/10)% 10) + 0x30;
        frame[14] = ((ix/100)% 10) + 0x30;
        ix = sonar_data[3] / 100;
        frame[22] = (ix % 10) + 0x30;
        frame[21] = ((ix/10)% 10) + 0x30;
        frame[20] = ((ix/100)% 10) + 0x30;
        ix = sonar_data[4] / 100;
        frame[28] = (ix % 10) + 0x30;
        frame[27] = ((ix/10)% 10) + 0x30;
        frame[26] = ((ix/100)% 10) + 0x30;
#endif
        strncpy((char *)frame, (char *)QRValue, 40);
        set_caption(frame, imgWidth);
    }
    output_start = (unsigned char *)JPEG_BUF;
    output_end = encode_image((unsigned char *)FRAME_BUF, output_start, quality, 
            FOUR_TWO_TWO, imgWidth, imgHeight); 
    image_size = (unsigned int)(output_end - output_start);

    framecount++;

    // We can't use PacketBegin() and PacketEnd() here because only the main
    // TX buffer is large enough to store a frame, and we don't want to have
    // to copy it over twice anyway. So we use SendPacket() here.
    if (GetOption(OPT_VIDEO_TEST_MODE_1))
    {
        image_size = 10000;
        imgHead[6] = (unsigned char)(image_size & 0x000000FF);
        imgHead[7] = (unsigned char)((image_size & 0x0000FF00) >> 8);
        imgHead[8] = (unsigned char)((image_size & 0x00FF0000) >> 16);
        imgHead[9] = 0x00;

        
        PacketBegin();
        for (i=0; i<10; i++) {
            //while (*pPORTHIO & 0x0001)  // hardware flow control
                //continue;
           uart0SendChar(imgHead[i]);
        }
        //cp = (unsigned char *)JPEG_BUF;
        for (i=0; i<image_size; i++)
        {
            uart0SendChar(0x80);
        }
        PacketEnd(true);
    }
    else
    {
        imgHead[6] = (unsigned char)(image_size & 0x000000FF);
        imgHead[7] = (unsigned char)((image_size & 0x0000FF00) >> 8);
        imgHead[8] = (unsigned char)((image_size & 0x00FF0000) >> 16);
        imgHead[9] = 0x00;

        PacketBegin();
        for (i=0; i<10; i++) {
            //while (*pPORTHIO & 0x0001)  // hardware flow control
                //continue;
           uart0SendChar(imgHead[i]);
        }
        cp = (unsigned char *)JPEG_BUF;
        for (i=0; i<image_size; i++)
        {
            uart0SendChar(*cp++);
        }
        PacketEnd(true);
    }
}

void send_80x64planar () {
    unsigned char *cp;
    unsigned int i;
    
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    scale_image_to_80x64_planar ((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, 
            imgWidth, imgHeight);


    printf("P5\n80\n192\n255\n");  // send pgm header
    cp = (unsigned char *)FRAME_BUF2;
    for (i=0; i<80*64*3; i++) 
        uart0SendChar(*cp++);
}

/* Turn image overlay on.
   Serial protocol char: o */
void overlay_on () {
    overlay_flag = 1;
    printf("#o");
}


/* Turn image overlay off.
   Serial protocol char: O */
void overlay_off () {
    overlay_flag = 0;
    printf("#O");
}

/* Camera initial setup */
void camera_setup () {
    
    /* Initialize camera-related globals */
    framecount = 0;
    overlay_flag = 0;
    quality = 4; // Default JPEG quality - range is 1-8 (1 is highest)
    frame_diff_flag = 0;
    segmentation_flag = 0;
    imgWidth = 320;
    imgHeight = 240;
    strcpy(imgHead, "##IMJ5    ");
    
    i2cwrite(0x21, ov7725_qvga, sizeof(ov7725_qvga)>>1, SCCB_ON);
    delayMS(2);
    i2cwrite(0x21, ov7725_qvga, sizeof(ov7725_qvga)>>1, SCCB_ON);
    delayMS(2);
    i2cwrite(0x30, ov9655_qvga, sizeof(ov9655_qvga)>>1, SCCB_ON);
    delayMS(2);
    i2cwrite(0x30, ov9655_qvga, sizeof(ov9655_qvga)>>1, SCCB_ON);

    camera_init((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, imgWidth, imgHeight);
    camera_start();
}

void invert_video() {  // flip video for upside-down camera
    invert_flag = 1;
    i2cwrite(0x21, ov7725_invert, sizeof(ov7725_invert)>>1, SCCB_ON);  // flip UV on OV7725
    i2cwrite(0x30, ov9655_invert, sizeof(ov9655_invert)>>1, SCCB_ON);  // flip UV on OV9655
    printf("#y");
}

void restore_video() {  // restore normal video orientation
    invert_flag = 0;
    i2cwrite(0x21, ov7725_restore, sizeof(ov7725_restore)>>1, SCCB_ON); // restore UV on OV7725
    i2cwrite(0x30, ov9655_restore, sizeof(ov9655_restore)>>1, SCCB_ON); // restore UV on OV9655
    printf("#Y");
}

/* Refactored out, code to reset the camera after a frame size change. */
void camera_reset (unsigned int width) {
    if (width == 160) {
        imgWidth = width;
        imgHeight = 120;
        strcpy(imgHead, "##IMJ3    ");
        camera_stop();
        i2cwrite(0x21, ov7725_qqvga, sizeof(ov7725_qqvga)>>1, SCCB_ON);
        i2cwrite(0x30, ov9655_qqvga, sizeof(ov9655_qqvga)>>1, SCCB_ON);
        printf("#a");
    } else if (width == 320) {
        imgWidth = width;
        imgHeight = 240;
        strcpy(imgHead, "##IMJ5    ");
        camera_stop();
        i2cwrite(0x21, ov7725_qvga, sizeof(ov7725_qvga)>>1, SCCB_ON);
        i2cwrite(0x30, ov9655_qvga, sizeof(ov9655_qvga)>>1, SCCB_ON);
        printf("#b");
    } else if (width == 640) {
        imgWidth = width;
        imgHeight = 480;
        strcpy(imgHead, "##IMJ7    ");
        camera_stop();
        i2cwrite(0x21, ov7725_vga, sizeof(ov7725_vga)>>1, SCCB_ON);
        i2cwrite(0x30, ov9655_vga, sizeof(ov9655_vga)>>1, SCCB_ON);
        printf("#c");
    } else if (width == 1280) {
        imgWidth = width;
        imgHeight = 1024;
        strcpy(imgHead, "##IMJ9    ");
        camera_stop();
        i2cwrite(0x30, ov9655_sxga, sizeof(ov9655_sxga)>>1, SCCB_ON);
        printf("#d");
    }
    camera_init((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, imgWidth, imgHeight);
    camera_start();
}

/* Change image quality.
   Serial protocol char: q */
void change_image_quality () {
    unsigned char ch;
    ch = getch();
    quality = (unsigned int)(ch & 0x0f);
    if (quality < 1) {
        quality = 1;
    } else if (quality > 8) {
        quality = 8;
    }
    printf("##quality - %c\r\n", ch);
}

// write caption string of up to 40 characters to frame buffer 
void set_caption(unsigned char *str, unsigned int width) {
    unsigned char *fbuf, *fcur, *str1, cc;
    int len, ix, iy, iz, w2;
    
    w2 = width * 2;
    str1 = str;
    
    for (len=0; len<40 && *str1++; len++);          // find string length
    fbuf = FRAME_BUF + (unsigned char *)((width * 17) - (len * 8));  // point to 1st char
    
    for (ix=0; ix<len; ix++) {
        fcur = fbuf;
        for (iy=0; iy< 8; iy++) {
            cc = font8x8[str[ix]*8 + iy];
            for (iz=0; iz<8; iz++) {
                if (cc & fontmask[iz]) {
                    fcur[0] = 0x80;
                    fcur[1] = 0xff;
                }
                fcur += 2;
            }
            fcur += (width * 2) - 16;          // move to next line
        }    
        fbuf += 16;  // move to next char position
    }
}


void move_image(unsigned char *src1, unsigned char *src2, unsigned char *dst, unsigned int width, unsigned int height) 
{
    unsigned char *src;
    unsigned short *isrc, *idst;
    int ix;
        
    if (*pDMA0_CURR_ADDR < (void *)src2)
        src = src2;
    else
        src = src1;
    
    isrc = (unsigned short *)src;
    idst = (unsigned short *)dst;
    for (ix = 0; ix < (width * height); ix++)
        *idst++ = *isrc++;
}

void move_inverted(unsigned char *src1, unsigned char *src2, unsigned char *dst, unsigned int width, unsigned int height) 
{
    unsigned char *src;
    unsigned short *isrc, *idst;
    int ix;
        
    if (*pDMA0_CURR_ADDR < (void *)src2)
        src = src2;
    else
        src = src1;
    
    isrc = (unsigned short *)src;
    idst = (unsigned short *)(dst + imgWidth*imgHeight*2 - 2);
    for (ix = 0; ix < (width * height); ix++)
        *idst-- = *isrc++;
}

/* copy frame buffer 
   - works whether pixels are interleaved or planar
*/
void copy_image(unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height) 
{
    int ix, xy;
    unsigned short *isrc, *idst;
        
    xy = width * height;
    isrc = (unsigned short *)src;
    idst = (unsigned short *)dst;
    for (ix=0; ix<xy; ix++)
        *idst++ = *isrc++;
}


/* move YUV422 interleaved pixels to separate Y, U and V planar buffers 
   - incoming pixels are UYVY
   - Y buffer is twice the size of U or V buffer 
*/
void move_yuv422_to_planar (unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height)
{
    unsigned char *py, *pu, *pv;
    int ix, xy, xy2;
    
    xy = width * height;
    xy2 = xy / 2;
    py = dst;
    pu = dst + xy;
    pv = pu + xy2;
    
    for (ix=0; ix<xy2; ix++) {
        *pu++ = *src++;
        *py++ = *src++;
        *pv++ = *src++;
        *py++ = *src++;
    }
}

/* scale YUV422 interleaved pixels to separate 80x64 Y, U and V planar buffers 
   - incoming pixels are UYVY, so Y pixels will be averaged
*/
void scale_image_to_80x64_planar (unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height)
{
    unsigned char *py, *pu, *pv;
    int ix, iy, xskip, yskip; 
    unsigned int y1, y2;
    
    xskip = (width / 40) - 4;
    yskip = ((height / 60) - 1) * width * 2;
    py = dst;
    pu = dst + 5120;  // 80*64
    pv = pu + 5120;
    
    for (iy=0; iy<64; iy++) {
        if ((iy==1) || (iy==2) || (iy==62) || (iy==63)) {  // duplicate first 2 and last 2 lines 
            for (ix=0; ix<80; ix++) {                      // to fill out 60 lines to 64
                *pu = *(pu-80);
                *pv = *(pv-80);
                *py = *(py-80);
                py++; pu++; pv++;                
            }
            continue;
        }
        for (ix=0; ix<80; ix++) {
            *pu++ = *src++;
            y1 = (unsigned int)*src++;
            *pv++ = *src++;
            y2 = (unsigned int)*src++;
            *py++ = (unsigned char)((y1+y2)>>1);
            src += xskip;
        }
        src += yskip;
    }
}


/* XModem Receive.
   Serial protocol char: X */
void xmodem_receive (unsigned char * BufferPtr, unsigned int BufferSize) 
{
    unsigned char * ix;
    PauseStreaming();
    for (ix = BufferPtr; ix < (BufferPtr + BufferSize); ix++)
    {
        *ix = 0;   // clear the read buffer
    }
    err1 = xmodemReceive(BufferPtr, BufferSize);
    if (err1 < 0) {
        PacketBegin();
        printf("##Xmodem receive error: %d\r\n", err1);
        PacketEnd(true);
    } else {
        PacketBegin();
        printf("##Xmodem success. Count: %d\r\n", err1);
        PacketEnd(true);
    }
    UnpauseStreaming();
}

void launch_editor() {
    edit((unsigned char *)FLASH_BUFFER);
}

/* Clear flash buffer
   Serial protocol char: z-c */
void clear_flash_buffer () {
    for (ix = FLASH_BUFFER; ix < (FLASH_BUFFER  + FLASH_BUFFER_SIZE); ix++)
    {
        *((unsigned char *)ix) = 0;   // clear the read buffer
    }
    printf("##zclear buffer\r\n");
}

/* crc flash buffer using crc16_ccitt()
   Serial protocol char: z-C */
void crc_flash_buffer () 
{
    unsigned int ix;
    ix = (unsigned int)crc16_ccitt((void *)FLASH_BUFFER, 0x0001fff8);  // don't count last 8 bytes
    printf("##zCRC: 0x%x\r\n", ix);
}

/* Read user flash sector into flash buffer
   Serial protocol char: z-r */
void read_user_flash () 
{
    int ix;
    memset((uint8_t *)FLASH_BUFFER, 0x00, FLASH_BUFFER_SIZE);
    ix = spi_read(USER_FLASH, (unsigned char *)FLASH_BUFFER, ONE_SECTOR);
    printf("##zread count: %d\r\n", ix);
}

void read_user_sector (int isec) 
{
    int ix;
    memset((uint8_t *)FLASH_BUFFER, 0x00, FLASH_BUFFER_SIZE);
    printf("##zRead ");
    if ((isec < USER_FLASH_SECTOR_BEGIN) || (isec > USER_FLASH_SECTOR_END)) 
    {
        printf(" - sector %d not accessible\r\n", isec);
        return;
    }
    ix = spi_read((isec * ONE_SECTOR), (unsigned char *)FLASH_BUFFER, ONE_SECTOR);
    printf (" - loaded %d bytes from flash sector %d\r\n", ix, isec);   
}

void read_double_sector (int isec, int quiet) 
{
    int ix;
    memset((uint8_t *)FLASH_BUFFER, 0x00, FLASH_BUFFER_SIZE);
    if (!quiet)
    {
        printf("##zA ");
    }
    if ((isec < USER_FLASH_SECTOR_BEGIN) || (isec > USER_FLASH_SECTOR_END)) 
    {
        if (!quiet)
        {
            printf(" - sector %d not accessible\r\n", isec);
        }
        return;
    }
    ix = spi_read((isec * ONE_SECTOR), (unsigned char *)FLASH_BUFFER, TWO_SECTORS);
    if (!quiet)
        printf (" - loaded %d bytes from flash sector %d->%d\r\n", ix, isec, isec+1);   
}

/* Write user flash sector from flash buffer
   Serial protocol char: z-w */
void write_user_flash () 
{
    int ix;
    ix = spi_write(
        USER_FLASH, 
        (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + ONE_SECTOR), 
        ONE_SECTOR
    );
    printf("##zwrite count: %d\r\n", ix);
}

void write_user_sector (int isec) 
{
    int ix;
    printf("##zWrite ");
    if ((isec < USER_FLASH_SECTOR_BEGIN) || (isec > USER_FLASH_SECTOR_END)) 
    {
        printf(" - sector %d not accessible\r\n", isec);
        return;
    }
    ix = spi_write(
        (isec * ONE_SECTOR), 
        (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + ONE_SECTOR), 
        ONE_SECTOR
    );
    printf (" - saved %d bytes to flash sector %d\r\n", ix, isec);   
}

// save 128kB to consecutive sectors
void write_double_sector (int isec, int quiet) 
{
    int ix;
    if (!quiet)
    {
        printf("##zB ");
    }
    if ((isec < USER_FLASH_SECTOR_BEGIN) || (isec > USER_FLASH_SECTOR_END)) 
    {
        if (!quiet)
        {
            printf(" - sector %d not accessible\r\n", isec);
        }
        return;
    }
    ix = spi_write(
        (isec * ONE_SECTOR), 
        (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + TWO_SECTORS), 
        TWO_SECTORS
    );
    if (!quiet)
    {
        printf (" - saved %d bytes to flash sectors %d->%d\r\n", ix, isec, isec+1);   
    }
}

/* Write boot flash sectors (1-3) from flash buffer
   should also work with writing u-boot.ldr
   Serial protocol char: zZ */
void write_boot_flash () 
{
    unsigned int *ip;
    int ix;
    
    ip = (unsigned int *)FLASH_BUFFER;  // look at first 4 bytes of LDR file - should be 0xFFA00000 or 0xFF800000
    if ((*ip != 0xFFA00000) && (*ip != 0xFF800000)) 
    {
        printf("##zZ boot image - invalid header, %p\r\n", (void *)*ip);
        return;
    }                        
    ix = spi_write(
        BOOT_FLASH, 
        (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + BOOT_FLASH_SIZE), 
        BOOT_FLASH_SIZE
    );
    printf("##zZ boot image write count: %d\r\n", ix);
}

// Process the $P31 or $P30 command
// Turn PicoC printf() buffer stream mode on and off
// Reply with #P3
void PicoCSetPrintfBuffer(void)
{
    unsigned char c;
    c = getch();

    if (c == '1')
    {
        PicoCStreamBufferEnabled = TRUE;
        printf("#P3");
    }
    if (c == '0')
    {
        PicoCStreamBufferEnabled = FALSE;
        printf("#P3");
    }

    return;
}

/* Process PicoC Memory command
    The basic idea is to allow RCM-Bfin Firmware commands (this one) to access a special group of PicoC global variables.
    This allows a running PicoC program to send/receive data with an application running on a computer over
    the radio. This, in turn, allows real-time values to be see on the PC, and real-time control over the PicoC program
    from the PC (if the PC application is set up to use these commands properly).

    There are three arrays, each with 256 elements in it:
    1) string array pico_str[256][256]; where each string has 256 bytes of space
    2) signed int pico_int[256]; to store signed 32 bit integers
    3) double pico_char[256]; to store floating point values

    // Writing \\
    "*W,S,<index>,<value><CR>" Write string to <index>, where <value> is a string from one to 256 bytes long. First <CR> terminates the string.
        RCM-Bfin responds with "#WS"
    "*W,I,<index>,<value><CR>" Write signed 32 bit integer to <index>, where <value> is a signed 32 bit decimal integer. <CR> terminates command.
        RCM-Bfin responds with "#WI"
    "*W,F,<index>,<value><CR>" Write floating point value to <index>, where <value> is a normal double floating point value (exponential allowed). <CR> terminates command.
        RCM-Bfin responds with "#WF"
    
    // Reading \\
    "*R,S,<index><CR>" Read string from <index>.
        RCM-Bfin responds with "##R,S,<index>,<string><CR><LF>" where <string> is a 1 to 256 byte string from <index>. First <CR> terminates the string and the response.
    "*R,I,<index><CR>" Read signed 32 bit integer from <index>.
        RCM-Bfin responds with "##R,I,<index>,<value><CR><LF>" where <value> is a signed 32 bit integer from <index>. <CR> terminates the response.
    "*R,F,<index><CR>" Read floating point value from <index>.
        RCM-Bfin responds with "##R,F,<index>,<value><CR><LF>" where <value> is a floating point double value from <index>. <CR> terminates the response.
        
    All accesses to reading and writing these PicoC shared variable/memory are guaranteed to be atomic.
	
	"*B[CR]" Read out entire printf() stream buffer and clear
		RCM-Bfin responds with "#B,[length],[buffer_contents]" where [length] is length of [buffer_contents], and [buffer_contents]
		is the entire contents of the printf stream buffer.
*/
void processPicoCMemory()
{
    unsigned char Direction, Type = 0;
    unsigned int Index = 0;
    char Command[300];
    unsigned int i = 0;
    char * Rest = NULL;
    unsigned int CommaCount = 0;
 
    PacketBegin();
    // Pull in all characters until <CR>
    while (i < 300)
    {
        // Have we found a <CR>?
        if ((Command[i] = getch()) == '\r')
        {
            i++;
            Command[i] = 0x00;
            break;
        }
        i++;
    }
    if (i == 300)
    {
        // Getting hear means that there was not a <CR> after 300 characters
        // Do nothing. The goggles, they do nothing. 
        PrintUnknownCommand(Command[0]);
        PacketEnd(true);
        return;
    }

    // Check for *B command
    if (Command[0] == 'B')
    {
        char temp[20];
        int len = PicoCStreamBufferLength;
        sprintf(temp, "#B,%5d,", len);
        i = 0;
        while (temp[i] != 0x00)
        {
            putchar(temp[i]);
            i++;
        }
        // Now output the entire buffer
        while (len)
        {
            putchar(PicoCStreamBuffer[PicoCStreamBufferOut]);
            PicoCStreamBufferOut++;
            if (PicoCStreamBufferOut >= PICOC_STREAM_BUFFER_SIZE)
            {
                PicoCStreamBufferOut = 0; 
            }
            len--;
        }
        // And clear it out
        PicoCStreamBufferOut = 0;
        PicoCStreamBufferIn = 0;
        PicoCStreamBufferLength = 0;
        PacketEnd(true);
        return;
    }

    // Parse the string, at least as far as we can without knowing Type
    i = sscanf(Command, "%c,%c,%u", &Direction, &Type, &Index); 

    // Did we get enough parameters?
    if (i != 3)
    {
        PrintUnknownCommand(Command[0]);
        PacketEnd(true);
        return;
    }
        
    // Limit the index value
    if (Index > PICOC_SHARED_ARRAY_SIZE)
    {
        Index = PICOC_SHARED_ARRAY_SIZE - 1;
    }
    
    // Get the R or W
    switch (Direction) 
    {
        case 'R':
            // Now the type
            switch (Type)
            {
                // Read in a string
                case 'S':
                    pico_char[Index][PICOC_SHARED_ARRAY_STRING_LENGTH-1] = 0x00;
                    printf("##R,S,%d,%s\r\n", Index, pico_char[Index]);
                    break;
                // Read in an integer
                case 'I':
                    printf("##R,I,%d,%d\r\n", Index, pico_int[Index]);
                    break;
                // Read in a float
                case 'F':
                    printf("##R,F,%d,%f\r\n", Index, pico_float[Index]);
                    break;

                default:
                    break;
            }
            break;

        case 'W':
            // We depend on Rest to point to character after the 3th comma
            for (i = 0; i < 300; i++)
            {
                if (Command[i] == ',')
                {
                    CommaCount++;
                    if (CommaCount == 3)
                    {
                        break;
                    }
                }
            }
            if (i == 300)
            {
                PrintUnknownCommand(Command[0]);
                PacketEnd(true);
                return;
            }
            Rest = &(Command[i+1]);
            switch (Type)
            {
                case 'S':
                    strncpy(pico_char[Index], Rest, PICOC_SHARED_ARRAY_STRING_LENGTH);
                    printf("#WS\r\n");
                    break;
                
                case 'I':
                    pico_int[Index] = strtol(Rest, NULL, 10);
                    printf("#WI\r\n");
                    break;
                    
                case 'F':
                    pico_float[Index] = strtof(Rest, NULL);
                    printf("#WF\r\n");
                    break;
                    
                default:
                
                    break;
            }
            break;
            
        default:
            break;
    }
    PacketEnd(true);
}

/* Targeted I2C Delay
This function inerts a delay if this function is called with an argument
of i2c_device = 0x61 (RCM SC Servo chip - SD20) within 2ms of the last
call with i2c_device = 0x61 so that the second call doesn't happen until
2ms after the first one.

The reason this has to exist is that if you send two I2C commands to the
RCM SD20 RC servo chip without the delay, it will ignore the second command.

This function takes an argument of i2c_device which is the I2C address of the
device you're writing to.

This function returns true if a delay was inserted, false otherwise.
*/
unsigned int i2c_targeted_delay(unsigned char i2c_device)
{
	unsigned int ret = false;

	if (i2c_device == 0x61)
	{
		if (I2CDelayTimeoutMS)
		{
			delayUS(2000);
			ret = true;
		}
		I2CDelayTimeoutMS = 2;
	}
	return ret;
}

/*
 * Autoincriment
 *
 * Feature documentation:
 *
 * The autoincriment feature allows a PC client to send a simple I2C command to the RCM-Bfin,
 * and have the RCM-Bfin send continuous (every 100ms) I2C commands out to an I2C slave.
 * This allows a simple 'motor control' button on the PC (that just sends a single I2C
 * command on press, and another on release) to 'increment' and 'decrement' the value sent
 * to an I2C slave, in this case normally an LED dimmer.
 *
 * 
 */


#define I2C_RCM_BFIN_ADDRESS                     0x55    // I2C address for all autoincriment registers
#define I2C_RCM_BFIN_AUTOINC1_COMMAND            0x01    // Main command register for autoinc1 - send new data value here (>0x80 to to up, <0x80 to go down)
#define I2C_RCM_BFIN_AUTOINC1_DEST_REGISTER      0x02    // Set the destination register value here
#define I2C_RCM_BFIN_AUTOINC1_DEST_ADDRESS       0x03    // Set the destination address value here
#define I2C_RCM_BFIN_AUTOINC1_LOW_DATA_REGISTER  0x04    // Set the lowest value data value can take on (other than 0x00) here
#define I2C_RCM_BFIN_AUTOINC1_HIGH_DATA_REGISTER 0x05    // Set the highest value data value can take on here
#define I2C_RCM_BFIN_AUTOINC2_COMMAND            0x11
#define I2C_RCM_BFIN_AUTOINC2_DEST_REGISTER      0x12
#define I2C_RCM_BFIN_AUTOINC2_DEST_ADDRESS       0x13
#define I2C_RCM_BFIN_AUTOINC2_LOW_DATA_REGISTER  0x14
#define I2C_RCM_BFIN_AUTOINC2_HIGH_DATA_REGISTER 0x15
#define I2C_RCM_BFIN_AUTOINC_RATE_REGISTER       0x20    // Defaults to 200ms, can change to 
#define I2C_RCM_BFIN_AUTOINC_DELAY_MS            200     // Normally use 200ms between updates to the dimmer
#define I2C_RCM_BFIN_AUTOINC_LOWEST_DATA         0x0F    // Lowest value (other than 0x00) the data value can take
#define I2C_RCM_BFIN_AUTOINC_HIGHEST_DATA        0xC0    // Highest value the data value can take

 /* Block of static data for the autoincriment function */
static unsigned char autoinc1_curent_command = 0x00;
static unsigned char autoinc1_curent_value = 0x00;
static unsigned char autoinc1_dest_register = 0x18;
static unsigned char autoinc1_dest_address = 0x2C;
static unsigned char autoinc1_lowest_data = I2C_RCM_BFIN_AUTOINC_LOWEST_DATA;
static unsigned char autoinc1_highest_data = I2C_RCM_BFIN_AUTOINC_HIGHEST_DATA;

static unsigned char autoinc2_curent_command = 0x00;
static unsigned char autoinc2_curent_value = 0x00;
static unsigned char autoinc2_dest_register = 0x18;
static unsigned char autoinc2_dest_address = 0x2D;
static unsigned char autoinc2_lowest_data = I2C_RCM_BFIN_AUTOINC_LOWEST_DATA;
static unsigned char autoinc2_highest_data = I2C_RCM_BFIN_AUTOINC_HIGHEST_DATA;

static unsigned char autoinc_delay = I2C_RCM_BFIN_AUTOINC_DELAY_MS;

/* 
This function filters writes. It intercepts certain I2C addresses. When these addresses are
written from the PC, they do 'special' things. This is where that code lives. Normally, this
function doesn't do anything - it simply returns true (meaning that no special processing happed).
If it sees a write to one of the special I2C addresses, then it will handle those writes itself
and then return false, meaning that no more 'real' I2C processing should happen.
*/
bool i2c_write_filter(
    unsigned char i2c_device, 
    unsigned char data_length, 
    unsigned char * data
) {
    
    if (i2c_device == I2C_RCM_BFIN_ADDRESS) {
        if (data_length < 1) {
            return false;
        }
        switch (data[0]) {
            case I2C_RCM_BFIN_AUTOINC1_COMMAND:
                if (autoinc1_dest_register != 0 && autoinc1_dest_address != 0) {
                    // Start the process of sending out new values depending on how fast the change is supposed to happen
                    autoinc1_curent_command = data[1];
                }
                break;
            
            case I2C_RCM_BFIN_AUTOINC1_DEST_REGISTER:
                if (data_length >= 2) {
                    autoinc1_dest_register = data[1];
                }
                break;
                
            case I2C_RCM_BFIN_AUTOINC1_DEST_ADDRESS:
                if (data_length >= 2) {
                    autoinc1_dest_address = data[1];
                }
                break;

            case I2C_RCM_BFIN_AUTOINC1_HIGH_DATA_REGISTER:
                if (data_length >= 2) {
                    autoinc1_highest_data = data[1];
                }
                break;

            case I2C_RCM_BFIN_AUTOINC1_LOW_DATA_REGISTER:
                if (data_length >= 2) {
                    autoinc1_lowest_data = data[1];
                }
                break;

            case I2C_RCM_BFIN_AUTOINC2_COMMAND:
                if (autoinc2_dest_register != 0 && autoinc2_dest_address != 0) {
                    // Start the process of sending out new values depending on how fast the change is supposed to happen
                    autoinc2_curent_command = data[1];
                }
                break;
            
            case I2C_RCM_BFIN_AUTOINC2_DEST_REGISTER:
                if (data_length >= 2) {
                    autoinc2_dest_register = data[1];
                }
                break;
                
            case I2C_RCM_BFIN_AUTOINC2_DEST_ADDRESS:
                if (data_length >= 2) {
                    autoinc2_dest_address = data[1];
                }
                break;

            case I2C_RCM_BFIN_AUTOINC2_HIGH_DATA_REGISTER:
                if (data_length >= 2) {
                    autoinc2_highest_data = data[1];
                }
                break;

            case I2C_RCM_BFIN_AUTOINC2_LOW_DATA_REGISTER:
                if (data_length >= 2) {
                    autoinc2_lowest_data = data[1];
                }
                break;

            case I2C_RCM_BFIN_AUTOINC_RATE_REGISTER:
                if (data_length >= 2) {
                    autoinc_delay = data[1];
                }
                break;
        }
    
        return false;
    }
    else {
        return true;
    }
}

/* This function needs to get called from the main loop when there's idle time.
It keeps track of the time, and if enough time has passed, it checks to see
if there is any autoincriment I2C data that needs to get sent out. If so,
it takes care of handling that.
*/
void process_autoinc_i2c(void) {
    static bool have_initalized = false;
    unsigned char i2c_data[4];
    signed int new_value = 0;

    // On boot, we need to turn the LED off.
    if (have_initalized == false) {
        i2c_data[0] = autoinc1_dest_register;
        autoinc1_curent_value = 0x00;
        i2c_data[1] = autoinc1_curent_value;
        i2cwrite(autoinc1_dest_address, (unsigned char *)i2c_data, 1, SCCB_ON);

        i2c_data[0] = autoinc2_dest_register;
        autoinc2_curent_value = 0x00;
        i2c_data[1] = autoinc2_curent_value;
        i2cwrite(autoinc1_dest_address, (unsigned char *)i2c_data, 1, SCCB_ON);
        
        have_initalized = true;
    }
    
    // We only check this stuff every I2C_RCM_BFIN_AUTOINC_DELAY_MS millisconds.
    if (I2CAutoincTimeoutMS == 0) {
        // Yup. Time to do something. Reload the timer.
        I2CAutoincTimeoutMS = autoinc_delay;        
        
        // For Autoinc1
        // If our current value is 0x00 or 0x80, then don't send any new I2C commands to dimmer.
        // Also no need to send a command if we're already at max or min.
        if (
            autoinc1_curent_command != 0x00 
            && 
            autoinc1_curent_command != 0x80 
            && 
            !(autoinc1_curent_value >= autoinc1_highest_data && autoinc1_curent_command > 0x80)
            && 
            !(autoinc1_curent_value == 0x00 && autoinc1_curent_command < 0x80)
        ) {
            // We need to send out a new dimming command to the LED. We take the current dimming level,
            // and then add or subtract from it based on how far away the PC command value is from 0x80.
            if (autoinc1_curent_command > 0x80) {
                // If we're at 0 right now, then just jump up to lowest data value
                if (autoinc1_curent_value == 0x00 && autoinc1_lowest_data != 0x00) {
                    new_value = autoinc1_lowest_data;
                }
                else {
                    // Otherwise, start marching up normally
                    new_value = (int)autoinc1_curent_value + (int)(autoinc1_curent_command - 0x80);
                    if (new_value > autoinc1_highest_data) {
                        new_value = autoinc1_highest_data;
                    }
                }
                autoinc1_curent_value = (unsigned char)new_value;
            }
            else if (autoinc1_curent_command < 0x80) {
                new_value = (int)autoinc1_curent_value - (int)(0x80 - autoinc1_curent_command);
                if (new_value < autoinc1_lowest_data) {
                    new_value = 0x00;
                }
                autoinc1_curent_value = (unsigned char)new_value;
            }
            // Now send off the new value
            i2c_data[0] = autoinc1_dest_register;
            i2c_data[1] = autoinc1_curent_value;
            i2cwrite(autoinc1_dest_address, (unsigned char *)i2c_data, 1, SCCB_ON);
        }

        // For Autoinc2
        // If our current value is 0x00 or 0x80, then don't send any new I2C commands to dimmer.
        // Also no need to send a command if we're already at max or min.
        if (
            autoinc2_curent_command != 0x00 
            && 
            autoinc2_curent_command != 0x80 
            && 
            !(autoinc2_curent_value >= autoinc2_highest_data && autoinc2_curent_command > 0x80)
            && 
            !(autoinc2_curent_value == 0x00 && autoinc2_curent_command < 0x80)
        ) {
            // We need to send out a new dimming command to the LED. We take the current dimming level,
            // and then add or subtract from it based on how far away the PC command value is from 0x80.
            if (autoinc2_curent_command > 0x80) {
                // If we're at 0 right now, then just jump up to lowest data value
                if (autoinc2_curent_value == 0x00 && autoinc2_lowest_data != 0x00) {
                    new_value = autoinc2_lowest_data;
                }
                else {
                    // Otherwise, start marching up normally
                    new_value = (int)autoinc2_curent_value + (int)(autoinc2_curent_command - 0x80);
                    if (new_value > autoinc2_highest_data) {
                        new_value = autoinc2_highest_data;
                    }
                }
                autoinc2_curent_value = (unsigned char)new_value;
            }
            else if (autoinc2_curent_command < 0x80) {
                new_value = (int)autoinc2_curent_value - (int)(0x80 - autoinc2_curent_command);
                if (new_value < autoinc2_lowest_data) {
                    new_value = 0x00;
                }
                autoinc2_curent_value = (unsigned char)new_value;
            }
            // Now send off the new value
            i2c_data[0] = autoinc2_dest_register;
            i2c_data[1] = new_value;
            i2cwrite(autoinc2_dest_address, (unsigned char *)i2c_data, 1, SCCB_ON);
        }
    }
}

/* Process i2c command:  
        irxy  - i2c read device x, register y, return '##ir value'
        iRxy  - i2c read device x, register y, return 2-byte '##iR value'
        iMxyz - i2c read device x, register y, count z, return z-byte '##iM values'
        iwxyz - i2c write device x, register y, value z, return '##iw'
        iWabcd - i2c write device a, data b, c, d, return '##ix'
        idabcde - i2c dual write device a, register1 b, data1 c, register2 d, data2 e, return '##ix'
        iAa - i2c read analog channel on NAV board, RCM or RCM2 boards
        
        Read One Bit From I2C Register
            Command:
                “ibxxyyz”
                xx is a two character ASCII hexadecimal value from 00 to FF representing the I2C address
                yy is a two character ASCII hexadecimal value from 00 to FF representing the I2C register
                z is a one character ASCII hexadecimal value from 0 to 7 representing the bit to be read
                example: "ib327F3" would request a read of the 3rd bit of register 0x7F from I2C address 0x32
            Response:
                “##ib xx yy z v\r\n”
                xx is a two character ASCII hexadecimal value from 00 to FF representing the I2C address
                yy is a two character ASCII hexadecimal value from 00 to FF representing the I2C register
                z is a one character ASCII hexadecimal value from 0 to 7 representing the bit to be read
                v is a one character ASCII ‘0’ or ‘1’ representing the value of the bit that just got read
                example: "##ib 32 7F 3 0\n\r" would be indicating a 0 from bit 3 of register 0x7F of address 0x32 

            Command:
                “icxxyyz”
                Exactly the same as "ib" above, but using repeated start I2C transaction

            Response:
                “##ic xx yy z v\r\n”
                Exactly the same as "ib" above, but using repeated start I2C transaction
                

        Read One Byte From I2C Register
            Command:
                “ilxxyy”
                xx is a two character ASCII hexadecimal value from 00 to FF representing the I2C address
                yy is a two character ASCII hexadecimal value from 00 to FF representing the I2C register
                example: "il327F" would request a read of register 0x7F from I2C address 0x32
            Response:
                “##il xx yy vv\r\n”
                xx is a two character ASCII hexadecimal value from 00 to FF representing the I2C address
                yy is a two character ASCII hexadecimal value from 00 to FF representing the I2C register
                vv is a two character ASCII hexadecimal value from 00 to FF representing the value just read
                example: "##il 32 7F F2\n\r" would be indicating a value of 0xF2 from register 0x7F of address 0x32 

            Command:
                “inxxyyz”
                Exactly the same as "il" above, but using repeated start I2C transaction

            Response:
                “##in xx yy z v\r\n”
                Exactly the same as "il" above, but using repeated start I2C transaction


                Read Two Bytes From I2C Register
            Command:
                “iLxxyy”
                xx is a two character ASCII hexadecimal value from 00 to FF representing the I2C address
                yy is a two character ASCII hexadecimal value from 00 to FF representing the I2C register
                example: "il327F" would request a read of register 0x7F from I2C address 0x32
            Response:
                “##iL xx yy vvvv\r\n”
                xx is a two character ASCII hexadecimal value from 00 to FF representing the I2C address
                yy is a two character ASCII hexadecimal value from 00 to FF representing the I2C register
                vvvv is a four character ASCII hexadecimal value from 0000 to FFFF representing the value just read
                example: "##il 32 7F 4F2D\n\r" would be indicating a value of 0x4F2D from register 0x7F of address 0x32 

            Command:
                “iNxxyyz”
                Exactly the same as "iL" above, but using repeated start I2C transaction

            Response:
                “##iN xx yy z v\r\n”
                Exactly the same as "iL" above, but using repeated start I2C transaction

        Serial protocol char: i */
void process_i2c() {
    unsigned char i2c_device, i2c_data[16], cx, count, c1, c2, i2c_register, i2c_bit;
    uint16_t i2c_id;
    char buf[100];
    unsigned char cmd;
    
    PacketBegin();
    cmd = (unsigned char)getch();
    switch (cmd) {
        case 'r':   // 'irab'
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2cread(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
            printf("##ir%2x %d\r\n", i2c_device, i2c_data[0]);
            break;
        case 'R':   // 'iRab'
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2cread(i2c_device, (unsigned char *)i2c_data, 2, SCCB_ON);
            printf("##iR%2x %d\r\n",i2c_device, (i2c_data[0] << 8) + i2c_data[1]);
            break;
        case 'M':   // 'iMabc'
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            count = (unsigned char)getch() & 0x0F;
            i2cread(i2c_device, (unsigned char *)i2c_data, (unsigned int)count, SCCB_ON);
            printf("##iM%2x  ", i2c_device);
            for (cx=0; cx<count; cx++) {
                printf("%d ", i2c_data[cx]);
            }
            printf("\r\n");
            break;
        case 'w':   // 'iwabc'
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2c_data[1] = (unsigned char)getch();
            i2c_targeted_delay(i2c_device);
            if (i2c_write_filter(i2c_device, 2, i2c_data)) {
                i2cwrite(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
            }
            printf("##iw%2x\r\n", i2c_device);
            break;
        case 'W':  // 'iWabcd' multi-write
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2c_data[1] = (unsigned char)getch();
            i2c_data[2] = (unsigned char)getch();
            i2c_targeted_delay(i2c_device);
            if (i2c_write_filter(i2c_device, 3, i2c_data)) {
                i2cwritex(i2c_device, (unsigned char *)i2c_data, 3, SCCB_ON);
            }
            printf("##iW%2x", i2c_device);
            break;
        case 'd':  // 'idabcef' dual channel single byte I2C write
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2c_data[1] = (unsigned char)getch();
            c1 = (unsigned char)getch();
            c2 = (unsigned char)getch();
            i2c_targeted_delay(i2c_device);
            if (i2c_write_filter(i2c_device, 3, i2c_data)) {
                i2cwrite(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
            }
            i2c_data[0] = c1;
            i2c_data[1] = c2;
            i2c_targeted_delay(i2c_device);
            if (i2c_write_filter(i2c_device, 2, i2c_data)) {
                i2cwrite(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
            }
            printf("##id%2x", i2c_device);
            break;
        case 'b': // 'ibxxyyz'
        case 'c': // 'icxxyyz'
            buf[0] = (unsigned char)getch();
            buf[1] = (unsigned char)getch();
            buf[2] = 0x00;
            i2c_device = (unsigned char)strtol(buf, NULL, 16);
            buf[0] = (unsigned char)getch();
            buf[1] = (unsigned char)getch();
            buf[2] = 0x00;
            i2c_register = (unsigned char)strtol(buf, NULL, 16);
            i2c_data[0] = i2c_register;
            buf[0] = (unsigned char)getch();
            buf[1] = 0x00;
            i2c_bit = (unsigned char)strtol(buf, NULL, 16);
            if (i2c_bit > 7) {
                i2c_bit = 7;
            }
            if (cmd == 'b')
            {
                i2cread(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
                if (i2c_data[0] & (1 << i2c_bit)) {
                    printf("##ib %02x %02x %01x 1\r\n", i2c_device, i2c_register, i2c_bit);
                }
                else {
                    printf("##ib %02x %02x %01x 0\r\n", i2c_device, i2c_register, i2c_bit);
                }
            }
            else
            {
                i2creadrs(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
                if (i2c_data[0] & (1 << i2c_bit)) {
                    printf("##ic %02x %02x %01x 1\r\n", i2c_device, i2c_register, i2c_bit);
                }
                else {
                    printf("##ic %02x %02x %01x 0\r\n", i2c_device, i2c_register, i2c_bit);
                }
            }
            break;
        case 'l':   // 'ilxxyy'
        case 'n':   // 'inxxyy'
            buf[0] = (unsigned char)getch();
            buf[1] = (unsigned char)getch();
            buf[2] = 0x00;
            i2c_device = (unsigned char)strtol(buf, NULL, 16);
            buf[0] = (unsigned char)getch();
            buf[1] = (unsigned char)getch();
            buf[2] = 0x00;
            i2c_register = (unsigned char)strtol(buf, NULL, 16);
            i2c_data[0] = i2c_register;
            if (cmd == 'l')
            {
                i2cread(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
                printf("##il %02x %02x %02x\r\n", i2c_device, i2c_register, i2c_data[0]);
            }
            else
            {
                i2creadrs(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
                printf("##in %02x %02x %02x\r\n", i2c_device, i2c_register, i2c_data[0]);
            }
            break;
        case 'L':   // 'iLxxyy'
        case 'N':   // 'iNxxyy'
            buf[0] = (unsigned char)getch();
            buf[1] = (unsigned char)getch();
            buf[2] = 0x00;
            i2c_device = (unsigned char)strtol(buf, NULL, 16);
            buf[0] = (unsigned char)getch();
            buf[1] = (unsigned char)getch();
            buf[2] = 0x00;
            i2c_register = (unsigned char)strtol(buf, NULL, 16);
            i2c_data[0] = i2c_register;
            if (cmd == 'L')
            {
                i2cread(i2c_device, (unsigned char *)i2c_data, 2, SCCB_ON);
                printf("##iL %02x %02x %04x\r\n", i2c_device, i2c_register, (i2c_data[0] << 8) + i2c_data[1]);
            }
            else
            {
                i2creadrs(i2c_device, (unsigned char *)i2c_data, 2, SCCB_ON);
                printf("##iN %02x %02x %04x\r\n", i2c_device, i2c_register, (i2c_data[0] << 8) + i2c_data[1]);
            }
            break;
        case '$':   // For commands with ID word parameters
            cmd = (unsigned char)getch();
            switch (cmd) {
                case 'l':   // 'i$lddddxxyy
                case 'n':   // 'i$nddddxxyy
                    buf[0] = (unsigned char)getch();
                    buf[1] = (unsigned char)getch();
                    buf[2] = (unsigned char)getch();
                    buf[3] = (unsigned char)getch();
                    buf[4] = 0x00;
                    i2c_id = (uint16_t)strtol(buf, NULL, 16);
                    buf[0] = (unsigned char)getch();
                    buf[1] = (unsigned char)getch();
                    buf[2] = 0x00;
                    i2c_device = (unsigned char)strtol(buf, NULL, 16);
                    buf[0] = (unsigned char)getch();
                    buf[1] = (unsigned char)getch();
                    buf[2] = 0x00;
                    i2c_register = (unsigned char)strtol(buf, NULL, 16);
                    i2c_data[0] = i2c_register;
                    if (cmd == 'l')
                    {
                        i2cread(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
                        printf("##i$l %04x %02x %02x %02x\r\n", i2c_id, i2c_device, i2c_register, i2c_data[0]);
                    }
                    else if (cmd == 'n')
                    {
                        i2creadrs(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
                        printf("##i$n %04x %02x %02x %02x\r\n", i2c_id, i2c_device, i2c_register, i2c_data[0]);
                    }
                    break;
                
                case 'L':   // 'i$Lddddxxyy
                case 'N':   // 'i$Nddddxxyy
                    buf[0] = (unsigned char)getch();
                    buf[1] = (unsigned char)getch();
                    buf[2] = (unsigned char)getch();
                    buf[3] = (unsigned char)getch();
                    buf[4] = 0x00;
                    i2c_id = (uint16_t)strtol(buf, NULL, 16);
                    buf[0] = (unsigned char)getch();
                    buf[1] = (unsigned char)getch();
                    buf[2] = 0x00;
                    i2c_device = (unsigned char)strtol(buf, NULL, 16);
                    buf[0] = (unsigned char)getch();
                    buf[1] = (unsigned char)getch();
                    buf[2] = 0x00;
                    i2c_register = (unsigned char)strtol(buf, NULL, 16);
                    i2c_data[0] = i2c_register;
                    if (cmd == 'L')
                    {
                        i2cread(i2c_device, (unsigned char *)i2c_data, 2, SCCB_ON);
                        printf("##i$L %04x %02x %02x %04x\r\n", i2c_id, i2c_device, i2c_register, (i2c_data[0] << 8) + i2c_data[1]);
                    }
                    else if (cmd == 'N')
                    {
                        i2creadrs(i2c_device, (unsigned char *)i2c_data, 2, SCCB_ON);
                        printf("##i$N %04x %02x %02x %04x\r\n", i2c_id, i2c_device, i2c_register, (i2c_data[0] << 8) + i2c_data[1]);
                    }
                    break;

                default:
            
                    break;
            }
            
            break;
            
        default:
            return;
    }
    PacketEnd(true);
}


/* Motor command, three character command string follows.
   Serial protocol char: M */
void motor_command(void) 
{
    unsigned int mdelay;
    lspeed = (int)((signed char)getch());
    rspeed = (int)((signed char)getch());
    mdelay = (unsigned int)getch();
    if (!pwm1_init) 
    {
        initPWM();
        pwm1_init = 1;
        pwm1_mode = PWM_PWM;
        base_speed = 40;
        lspeed = rspeed = 0;
    }
    setPWM(lspeed, rspeed);
    if (mdelay) 
    {
        delayMS(mdelay * 10);
        setPWM(0, 0);
        lspeed = 0;
        rspeed = 0;
    }
    printf("#M");
}

/* Motor command for 2nd set of timers, three character command string follows.
   Serial protocol char: m */
void motor2_command(void) 
{
    unsigned int mdelay;
    lspeed2 = (int)((signed char)getch());
    rspeed2 = (int)((signed char)getch());
    mdelay = (unsigned int)getch();
    if (!pwm2_init) 
    {
        initPWM2();
        pwm2_init = 1;
        pwm2_mode = PWM_PWM;
        base_speed2 = 40;
    }
    setPWM2(lspeed2, rspeed2);
    if (mdelay) 
    {
        delayMS(mdelay * 10);
        setPWM2(0, 0);
        lspeed2 = 0;
        rspeed2 = 0;
    }
    printf("#m");
}

/* Increase motor base speed
   Serial protocol char: + */
void motor_increase_base_speed(void) 
{
    base_speed += 3;
    if (base_speed > 95) {
        base_speed = 95;
    }
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed + 3);
        rspeed = check_bounds_0_100(rspeed + 3);
        setPPM1(lspeed, rspeed);
    }
    printf("#+");
}

/* Decrease motor base speed
   Serial protocol char: - */
void motor_decrease_base_speed(void) 
{
    base_speed -= 3;
    if (base_speed < 0) {
        base_speed = 0;
    }
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed - 3);
        rspeed = check_bounds_0_100(rspeed - 3);
        setPPM1(lspeed, rspeed);
    }
    printf("#-");
}

void motor_trim_left(void) 
{
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed - 1);
        rspeed = check_bounds_0_100(rspeed + 1);
        setPPM1(lspeed, rspeed);
    }
}

void motor_trim_right(void) 
{
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed + 1);
        rspeed = check_bounds_0_100(rspeed - 1);
        setPPM1(lspeed, rspeed);
    }
}

/* Take motor action */
void motor_action(unsigned char ch) 
{

    #ifdef STEREO
    if (ch == '.') svs_right_turn_percent = 21;
    if (ch == '0') svs_right_turn_percent = -21;
    #endif
    
    motor_set(ch, base_speed, &lspeed, &rspeed);
    printf("#%c", ch);
}

/* General motor control code */
void motor_set(unsigned char cc, int speed, int *ls, int *rs)  
{
    int left_speed, right_speed;

    if (pwm1_mode != PWM_PWM) // only run the keypad commands in PWM mode
        return;
    
    /* record the time at which the motors started moving */
    if (cc != '5') 
    {
        move_start_time = readRTC();
        robot_moving = 1;
    }
    
    left_speed = right_speed = 0;
    switch (cc) 
    {
        case '7':     // drift left
            left_speed = speed-15;
            right_speed = speed+15;
            break;
        case '8':     // forward
            left_speed = speed; 
            right_speed = speed;
            break;
        case '9':     // drift right
            left_speed = speed+15;
            right_speed = speed-15;
            break;
        case '4':     // turn left
            left_speed = speed-30;
            right_speed = speed+30;
            break;
        case '5':        // stop
            left_speed = 0;
            right_speed = 0;
            move_stop_time = readRTC();
            move_time_mS = move_stop_time - move_start_time;
            robot_moving = 0;
            //printf("Move time at speed %d = %d mS\r\n", speed, move_time_mS);
            break;
        case '6':     // turn right
            left_speed = speed+30;
            right_speed = speed-30;
            break;
        case '1':     // back left
            left_speed = -(speed-30);
            right_speed = -(speed+30);
            break;
        case '2':     // back
            left_speed = -speed;
            right_speed = -speed;
            break;
        case '3':     // back right
            left_speed = -(speed+30);
            right_speed = -(speed-30);
            break;
        case '.':     // clockwise turn
            setPWM(70, -70);
            #ifdef STEREO
            delayMS(100);
            #else
            delayMS(200);
            #endif
            setPWM(0, 0);
            left_speed = 0;
            right_speed = 0;
            move_stop_time = readRTC();
            move_time_mS = move_stop_time - move_start_time;
            robot_moving = 0;
            break;
        case '0':     // counter clockwise turn
            setPWM(-70, 70);
            #ifdef STEREO
            delayMS(100);
            #else
            delayMS(200);
            #endif
            setPWM(0, 0);
            left_speed = 0;
            right_speed = 0;
            move_stop_time = readRTC();
            move_time_mS = move_stop_time - move_start_time;
            robot_moving = 0;
            break;
        default:
            return;
    }
    setPWM(left_speed, right_speed);

    *ls = left_speed;
    *rs = right_speed;
    return;
}

/* servo command, timers 2 and 3, two character command string follows.
   Serial protocol char: S */
void ppm1_command() {
    if (!pwm1_init) {
        initPPM1();
        pwm1_init = 1;
        pwm1_mode = PWM_PPM;
    }
    lspeed = (int)((signed char)getch());
    rspeed = (int)((signed char)getch());
    setPPM1(lspeed, rspeed);
    printf("#S");
}

/* servo command, timers 6 and 7, two character command string follows.
   Serial protocol char: s */
void ppm2_command() {
    if (!pwm2_init) {
        initPPM2();
        pwm2_init = 1;
        pwm2_mode = PWM_PPM;
    }
    lspeed2 = (int)((signed char)getch());
    rspeed2 = (int)((signed char)getch());
    setPPM2(lspeed2, rspeed2);
    printf("#s");
}

void initPWM() {
    // configure timers 2 and 3 for PWM (H-bridge interface)
    //*pPORT_MUX = 0;  // don't do this - it clobbers timers 6/7
    *pPORTF_FER |= 0x00C0;  // configure PF6 and PF7 as TMR3 and TMR2
    *pTIMER2_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER3_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER2_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER3_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100; 
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100;
    *pTIMER_ENABLE = TIMEN2 | TIMEN3;
    *pPORTHIO_DIR |= 0x0030;  // set PORTH4 and PORTH5 to output for direction control
    *pPORTHIO &= 0xFFCF;      // set output low 
    //*pPORTHIO |= 0x0030;  
}

void initPWM2() {
    // configure timers 6 and 7 for PWM
    *pPORT_MUX |= 0x0010;   // note that this reassigns UART1 signals as timers
    *pPORTF_FER |= 0x000C;  // configure PF2 and PF3 as TMR7 and TMR6
    *pTIMER6_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER7_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER6_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER7_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100; 
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100;
    *pTIMER_ENABLE |= TIMEN6 | TIMEN7;
}

void initTMR4() {
    // configure timer 4
    *pTIMER_ENABLE  &= ~TIMEN4;  // disable timer
    *pPORTF_FER |= 0x0020;  // configure PF5 TMR4
    *pTIMER4_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER4_PERIOD = PERIPHERAL_CLOCK;  // should be counting a 122MHz rate
    *pTIMER4_WIDTH = PERIPHERAL_CLOCK;
    *pTIMER_ENABLE |= TIMEN4;
}

void initPPM1() {
    // configure timers 2 and 3
    *pPORTF_FER |= 0x00C0;  // configure PF6 and PF7 as TMR3 and TMR2
    *pTIMER2_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER3_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER2_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER3_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 50) * 150) / 2000; // 1.5 millisec pulse
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 50) * 150) / 2000; // 1.5 millisec pulse
    *pTIMER_ENABLE |= TIMEN2 | TIMEN3;
}

void initPPM2() {
    // configure timers 6 and 7
    *pPORT_MUX |= 0x0010;   // note that this reassigns UART1 signals as timers
    *pPORTF_FER |= 0x000C;  // configure PF2 and PF3 as TMR7 and TMR6
    *pTIMER6_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER7_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER6_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER7_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 50) * 150) / 2000; // 1.5 millisec pulse
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 50) * 150) / 2000; // 1.5 millisec pulse
    *pTIMER_ENABLE |= TIMEN6 | TIMEN7;
}

void setPWM (int mleft, int mright) {
    if (mleft < 0) {
        *pPORTHIO = (*pPORTHIO & 0xFFEF);  // clear left direction bit
        mleft = -mleft;
    } else {
        *pPORTHIO = (*pPORTHIO & 0xFFEF) | 0x0010;  // turn on left direction bit
    }
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;

    if (mright < 0) {
        *pPORTHIO = (*pPORTHIO & 0xFFDF);  // clear right direction bit
        mright = -mright;
    } else {
        *pPORTHIO = (*pPORTHIO & 0xFFDF) | 0x0020;  // turn on right direction bit
    }
    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mleft) / 100;
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mright) / 100;
}

void setPWM2 (int mleft, int mright) {
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;

    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mleft) / 100;
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mright) / 100;
}

void setPPM1 (int mleft, int mright) {
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;
    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mleft)) / 2000;
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mright)) / 2000;
}

void setPPM2 (int mleft, int mright) {
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;
    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mleft)) / 2000;
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mright)) / 2000;
}

int check_bounds_0_100(int ix) {
    if (ix > 100)
        ix = 100;
    if (ix < 0)
        ix= 0;
    return ix;
}

/* Initialise the Real-time Clock */
void initRTC() {
    *pRTC_ICTL = 0;  // disable interrupts
    SSYNC;
    *pRTC_PREN = 0;  // disable prescaler - clock counts at 32768 Hz
    SSYNC;
    *pRTC_STAT = 0;  // clear counter
    SSYNC;
}

/* Read the RTC counter, returns number of milliseconds since reset */
int readRTC() {     
    int i1, i2;
    i1 = *pRTC_STAT;
    i2 = (i1 & 0x0000003F) + (((i1 >> 6) & 0x0000003F) * 60) +  
        (((i1 >> 12) & 0x0000001F) * 3600) + (((i1 >> 17) & 0x00007FFF) * 86400);
    return (i2 / 33);  // converts tick count to milliseconds
                       //    32,768 / 32.77 = 1,000
}

/* Clear the RTC counter value */
void clearRTC() {
    *pRTC_STAT = 0;
    SSYNC;
}

// delay up to 100000 millisecs (100 secs)
void delayMS(int delay) 
{
    int i0;

    if ((delay < 0) || (delay > 100000))
    {
        return;
    }
    i0 = readRTC();
    while (readRTC() < (i0 + delay))
    {
    }
}

// NOTE: Reentrant safe
void delayUS(int delay) {  // delay up to 100000 microseconds (.1 sec)
    // CORE_CLOCK (MASTER_CLOCK * VCO_MULTIPLIER / CCLK_DIVIDER) = 22,118,000 * 22
    // PERIPHERAL_CLOCK  (CORE_CLOCK / SCLK_DIVIDER) =  CORE_CLOCK / 4 = 121,649,000
    // *pTIMER4_PERIOD = PERIPHERAL_CLOCK, so TIMER4 should be counting a 121.649MHz rate
    int target, start;
    
    if ((delay < 0) || (delay > 100000))
        return;
    start = *pTIMER4_COUNTER;
    target = (((PERIPHERAL_CLOCK / 10000) * delay) / 100) + start;
    
    if (target > PERIPHERAL_CLOCK) {  // wait for timer to wrap-around
        target -= PERIPHERAL_CLOCK;
        while (*pTIMER4_COUNTER > target)
            continue;
    }
    while (*pTIMER4_COUNTER < target)
        continue;
}

void delayNS(int delay) {  // delay up to 100000 nanoseconds (.1 millisec)
    // minimum possible delay is approx 10ns
    int target, start;
    
    if ((delay < 10) || (delay > 100000))
        return;
    
    start = *pTIMER4_COUNTER;
    target = (((PERIPHERAL_CLOCK / 10000) * delay) / 100000) + start;
    
    if (target > PERIPHERAL_CLOCK) {  // wait for timer to wrap-around
        target -= PERIPHERAL_CLOCK;
        while (*pTIMER4_COUNTER > target)
            continue;
    }
    while (*pTIMER4_COUNTER < target)
        continue;
}

/* Enable failsafe - two character motor speeds follow
   Serial protocol char: F */
void enable_failsafe() {
    lfailsafe = (int)((signed char)getch());
        if (lfailsafe == 0)  // minimum PWM power setting is 0x01, not 0x00
            lfailsafe = 1;
    rfailsafe = (int)((signed char)getch());
        if (rfailsafe == 0)  // minimum PWM power setting is 0x01, not 0x00
            rfailsafe = 1;
    failsafe_mode = 1;
    printf("#F");
}

/* Disable failsafe - 
   Serial protocol char: f */
void disable_failsafe() {
    failsafe_mode = 0;
    printf("#f");
}

void reset_failsafe_clock() {
    failsafe_clock = readRTC();
}

void check_failsafe() {
    if (!failsafe_mode)
        return;
    if ((readRTC() - failsafe_clock) < 2000)  // 2 second timeout
        return;
    lspeed = lfailsafe;
    rspeed = rfailsafe;
    if (pwm1_mode == PWM_PWM)
        setPWM(lspeed, rspeed);
    if (pwm1_mode == PWM_PPM)
        setPPM1(lspeed, rspeed);
}

void process_colors() {
    unsigned char ch, ch1, ch2;
    unsigned int clr, x1, x2, y1, y2;
    unsigned int ix, iy, i1, i2, itot;
    unsigned int ulo[4], uhi[4], vlo[4], vhi[4];
    int vect[16];  // used by vscan()
    int slope, intercept;
    unsigned char i2c_data[2];
              // vision processing commands
                    //    va = enable/disable AGC / AWB / AEC camera controls
                    //    vb = find blobs matching color bin 
                    //    vc = set color bin ranges
                    //    vd = dump camera registers
                    //    vf = find pixels matching color bin
                    //    vh = histogram
                    //    vm = mean colors
                    //    vp = sample individual pixel
                    //    vr = recall color bin ranges
                    //    vs = scan for edges
                    //    vt = set edge detect threshold (0000-9999, default is 3200)
                    //    vz = zero all color settings
    ch = getch();
    switch (ch) {
        case 'a':  //    va = enable/disable AGC(4) / AWB(2) / AEC(1) camera controls
                   //    va7 = AGC+AWB+AEC on   va0 = AGC+AWB+AEC off
            ix = (unsigned int)getch() & 0x07;
            i2c_data[0] = 0x13;
            i2c_data[1] = 0xC0 + ix;
            i2cwrite(0x30, (unsigned char *)i2c_data, 1, SCCB_ON);  // OV9655
            i2cwrite(0x21, (unsigned char *)i2c_data, 1, SCCB_ON);  // OV7725
            printf("##va%d\r\n", ix);
            break;
        case 'b':  //    vb = find blobs for a given color
            ch1 = getch();
            ch2 = ch1;
            if (ch1 > '9')
                ch1 = (ch1 & 0x0F) + 9;
            else
                ch1 &= 0x0F;
            grab_frame();
            ix = vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, ch1);
            if (ix == 0xFFFFFFFF) {
                printf("##vb%c -1\r\n", ch2);
                break;  // too many blobs found
            }
            printf("##vb%c %d\r\n", ch2, ix);
            for (iy=0; iy<ix; iy++) {
                printf(" %d - %d %d %d %d  \r\n", 
                    blobcnt[iy], blobx1[iy], blobx2[iy], bloby1[iy], bloby2[iy]);
            }
            break;
        case 'c':  //    vc = set colors
            ix = (unsigned int)getch();
            if (ix > '9')
                ix = (ix & 0x0F) + 9;
            else
                ix &= 0x0F;
            ymin[ix] = intfromthreechars();
            ymax[ix] = intfromthreechars();
            umin[ix] = intfromthreechars();
            umax[ix] = intfromthreechars();
            vmin[ix] = intfromthreechars();
            vmax[ix] = intfromthreechars();
            printf("##vc %d\r\n", ix);
            break;
        case 'd':  //    vd = dump camera registers
            printf("##vdump\r\n");
            for(ix=0; ix<256; ix++) {
                i2c_data[0] = ix;
                i2cread(0x21, (unsigned char *)i2c_data, 1, SCCB_ON);
                printf("%x %x\r\n", ix, i2c_data[0]);
            }
            break;
        case 'f':  //    vf = find number of pixels in x1, x2, y1, y2 range matching color bin
            clr = getch() & 0x0F;
            x1 = intfromfourchars();
            x2 = intfromfourchars();
            y1 = intfromfourchars();
            y2 = intfromfourchars();
            grab_frame();
            printf("##vf %d\r\n", vfind((unsigned char *)FRAME_BUF, clr, x1, x2, y1, y2));
            break;
        case 'h':  //    vh = histogram
            grab_frame();
            vhist((unsigned char *)FRAME_BUF);
            printf("##vhist\r\n");
            iy = 0;
            itot = imgWidth * imgHeight / 2;
            printf("      0  16  32  48  64  80  96 112 128 144 160 176 192 208 224 240 (V-axis)\r\n");
            for (i1=0; i1<16; i1++) {
                printf("%d", i1*16);
                for (i2=0; i2<16; i2++) {
                    iy++;
                    ix = i1*16 + i2;
                    if (hist0[ix] > (itot>>2))
                        printf("****");
                    else if (hist0[ix] > (itot>>5))
                        printf(" ***");
                    else if (hist0[ix] > (itot>>8))
                        printf("  **");
                    else if (hist0[ix] > (itot>>11))
                        printf("   *");
                    else {
                        printf("    ");
                        iy--;
                    }
                }
                printf("\r\n");
            }
            printf("(U-axis)             %d regions\r\n", iy);
            break;
        case 'm':  //    vm = mean colors
            grab_frame();
            vmean((unsigned char *)FRAME_BUF);
            printf("##vmean %d %d %d\r\n", mean[0], mean[1], mean[2]);
            break;
        case 'p':  //    vp = sample individual pixel, print YUV value
            i1 = intfromfourchars();
            i2 = intfromfourchars();
            grab_frame();
            ix = vpix((unsigned char *)FRAME_BUF, i1, i2);
            printf("##vp %d %d %d\r\n",
                ((ix>>16) & 0x000000FF),  // Y1
                ((ix>>24) & 0x000000FF),  // U
                ((ix>>8) & 0x000000FF));   // V
            break;
        case 'r':  //    vr = recall colors
            ix = (unsigned int)getch();
            if (ix > '9')
                ix = (ix & 0x0F) + 9;
            else
                ix &= 0x0F;
            printf("##vr %d %d %d %d %d %d %d\r\n",
               ix, ymin[ix], ymax[ix], umin[ix], umax[ix], vmin[ix], vmax[ix]);
            break;
        case 's':  //    vs = scan for edges 
            x1 = (unsigned int)getch() & 0x0F;  // get number of columns to use
            grab_frame();
            ix = vscan((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh, (unsigned int)x1, (unsigned int *)&vect[0]);
            printf("##vscan = %d ", ix);
            for (i1=0; i1<x1; i1++)
                printf("%4d ", vect[i1]);
            printf("\r\n");
            break;
        case 't':  //    vt = set edge detect threshold (0000-9999, default is 3200)
            edge_thresh = intfromfourchars();
            printf("##vthresh %d\r\n", edge_thresh);
            break;
        case 'u':  //    vu = scan for horizon, 
            x1 = (unsigned int)getch() & 0x0F;  // get number of columns to use
            grab_frame();
            ix = vhorizon((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh, 
                        (unsigned int)x1, (unsigned int *)&vect[0], &slope, &intercept, 5);
            printf("##vhorizon = %d ", ix);
            for (i1=0; i1<x1; i1++)
                printf("%4d ", vect[i1]);
            printf("\r\n");
            break;
        case 'z':  //    vz = clear or segment colors
            ix = (unsigned int)getch() & 0x0F;
            printf("##vzero\r\n");
            switch (ix) {
                case 0:
                    for(ix = 0; ix<MAX_COLORS; ix++) 
                        ymin[ix] = ymax[ix] = umin[ix] = umax[ix] = vmin[ix] = vmax[ix] = 0;
                    break;
                case 1:
                    for(ix = 0; ix<MAX_COLORS; ix++) {
                        ymin[ix] = (ix / 4) * 64;
                        ymax[ix] = ymin[ix] + 63;
                        umin[ix] = (ix & 0x02) * 64;
                        umax[ix] = umin[ix] + 127;
                        vmin[ix] = (ix & 0x01) * 128;
                        vmax[ix] = vmin[ix] + 127;
                    }
                    break;
                case 2:
                    for(ix = 0; ix<MAX_COLORS; ix++) {
                        ymin[ix] = 0;
                        ymax[ix] = 255;
                        umin[ix] = (ix >> 2) * 64;
                        umax[ix] = umin[ix] + 63;
                        vmin[ix] = (ix & 0x03) * 64;
                        vmax[ix] = vmin[ix] + 63;
                    }
                    break;
                case 3:
                    ulo[0]=0; ulo[1]=96; ulo[2]=128; ulo[3]=160;
                    uhi[0]=ulo[1]-1; uhi[1]=ulo[2]-1; uhi[2]=ulo[3]-1; uhi[3]=255;
                    vlo[0]=0; vlo[1]=96; vlo[2]=128; vlo[3]=160;
                    vhi[0]=vlo[1]-1; vhi[1]=vlo[2]-1; vhi[2]=vlo[3]-1; vhi[3]=255;
                    for(ix = 0; ix<MAX_COLORS; ix++) {
                        i1 = ix >> 2;
                        i2 = ix & 0x03;
                        ymin[ix] = 0;
                        ymax[ix] = 255;
                        umin[ix] = ulo[i1];
                        umax[ix] = uhi[i1];
                        vmin[ix] = vlo[i2];
                        vmax[ix] = vhi[i2];
                    }
                    break;
                case 4:
                    for(ix = 0; ix<MAX_COLORS; ix++) {
                        ymin[ix] = ix << 4;
                        ymax[ix] = ymin[ix] + 15;
                        umin[ix] = 0;
                        umax[ix] = 255;
                        vmin[ix] = 0;
                        vmax[ix] = 255;
                    }
                    break;
           }
            break;
    }
}

void process_neuralnet() {
    unsigned char ch;
    unsigned int ix, i1, i2;
              // neural net processing commands
                    //    np = set pattern
                    //    nd = display pattern
                    //    ni = init network
                    //    nt = train for 10000 iterations
                    //    nx = test a pattern
                    //    nb = match blob to patterns
                    //    ng = create pattern from blob
    ch = getch();
    switch (ch) {
        case 'p':  //    np = set pattern
            ix = ctoi(getch());
            if (ix > NUM_NPATTERNS) {
                printf("##np - invalid index\r\n");
                break;
            }
            for (i1=0; i1<8; i1++)
                npattern[ix*8 + i1] = (ctoi(getch()) << 4) + ctoi(getch());
            printf("##np %d\r\n", ix);
            break;
        case 'd':  //    nd = display pattern
            ix = ctoi(getch());
            if (ix > NUM_NPATTERNS) {
                printf("##np - invalid index\r\n");
                break;
            }
            printf("##nd %d\r\n", ix);
            nndisplay(ix);
            break;
        case 'i':  //    ni = init network
            nninit_network();
            printf("##ni - init neural net\r\n");
            break;
        case 't':  //    nt = train network
            nntrain_network(10000);
            printf("##nt - train 10000 iterations\r\n");
            for (ix=0; ix<NUM_NPATTERNS; ix++) {
                nnset_pattern(ix);
                nncalculate_network();
                for (i1=0; i1<NUM_OUTPUT; i1++) 
                    printf(" %3d", N_OUT(i1)/10);
                printf("\r\n");
            }
            break;
        case 'x':  //    nx = test example pattern
            ix = 0;
            for (i1=0; i1<8; i1++) {   /// capture the test pattern and store in N_IN input neurons
                ch = (ctoi(getch()) << 4) + ctoi(getch());
                for (i2=0; i2<8; i2++) {
                    if (ch & nmask[i2])
                        N_IN(ix++) = 1024;
                    else
                        N_IN(ix++) = 0;
                }
            }
            nncalculate_network();
            printf("##nx\r\n");
            for (i1=0; i1<NUM_OUTPUT; i1++) 
                printf(" %3d", N_OUT(i1)/10);
            printf("\r\n");
            break;            
        case 'b':  //    nb = match blob to patterns
            ix = ctoi(getch());    // grab the blob #
            if (!blobcnt[ix]) { 
                printf("##nb - not a valid blob\r\n");
                break;
            }
            /* use data still in blob_buf[] (FRAME_BUF3)
               square the aspect ratio of x1, x2, y1, y2
               then subsample blob pixels to populate N_IN(0:63) with 0:1024 values
               then nncalculate_network() and display the N_OUT() results */
            nnscale8x8((unsigned char *)FRAME_BUF3, blobix[ix], blobx1[ix], blobx2[ix], 
                    bloby1[ix], bloby2[ix], imgWidth, imgHeight);
            nncalculate_network();
            printf("##nb\r\n");
            for (i1=0; i1<NUM_OUTPUT; i1++) 
                printf(" %3d", N_OUT(i1)/10);
            printf("\r\n");
            break;
        case 'g':  //     ng = create pattern from blob
            ix = ctoi(getch());    // grab the new pattern #
            if (!blobcnt[0]) { 
                printf("##ng - no blob to grab\r\n");
                break;
            }
            nnscale8x8((unsigned char *)FRAME_BUF3, blobix[0], blobx1[0], blobx2[0], 
                    bloby1[0], bloby2[0], imgWidth, imgHeight);
            nnpack8x8(ix);
            nndisplay(ix);
            break;
    }
}

/* use GPIO H14 and H15 for 2-channel wheel encoder inputs -
    H14 (pin 31) is left motor, H15 (pin 32) is right motor */
void init_encoders() {  
    if (encoder_flag)
        return;
    encoder_flag = 1;
    *pPORTHIO_INEN |= 0xC000;  // enable H14 and H15 as inputs
    *pPORTHIO_DIR &= 0x3FFF;   // set H14 and H15 as inputs
    initTMR4();
}

void read_encoders()
{
    encoders();
    printf("##$Encoders:  left = %d  right = %d\r\n", lcount, rcount);
}

/* read encoder pulses from GPIO-H14 and H15.  compute pulses per second, and
      pack left and right encoder counts into 32-bit unsigned */
unsigned int encoders() {
    int t0;
    unsigned int llast, rlast, lnew, rnew, ltime0, ltime1, rtime0, rtime1;
    
    init_encoders();

    t0 = readRTC();
    llast = *pPORTHIO & 0x4000;
    rlast = *pPORTHIO & 0x8000;
    ltime0 = ltime1 = rtime0 = rtime1 = 0;
    lcount = rcount = 0;
    
    while ((readRTC() - t0) < 40) {  
        lnew = *pPORTHIO & 0x4000;
        if (llast != lnew) {
            llast = lnew;
            lcount++;
            if (lcount == 1)
                ltime0 = *pTIMER4_COUNTER;
            if (lcount == 3)
                ltime1 = *pTIMER4_COUNTER;
        }
        rnew = *pPORTHIO & 0x8000;
        if (rlast != rnew) {
            rlast = rnew;
            rcount++;
            if (rcount == 1)
                rtime0 = *pTIMER4_COUNTER;
            if (rcount == 3)
                rtime1 = *pTIMER4_COUNTER;
        }
        if ((lcount>=3) && (rcount>=3))
            break;
    }
    if (!ltime1) ltime0 = 0;
    if (ltime1 < ltime0) ltime1 += PERIPHERAL_CLOCK;   // fix wraparound
    if (!rtime1) rtime0 = 0;
    if (rtime1 < rtime0) rtime1 += PERIPHERAL_CLOCK;   // fix wraparound

    ltime1 -= ltime0;  // compute pulse width
    if (ltime1) 
        lcount = PERIPHERAL_CLOCK / ltime1;  // compute pulses per second
    else 
        lcount = 0;

    rtime1 -= rtime0;  
    if (rtime1) 
        rcount = PERIPHERAL_CLOCK / rtime1;  // compute pulses per second
    else 
        rcount = 0;
    
    return ((unsigned int)lcount << 16) + (unsigned int)rcount;
}

unsigned int encoder_4wd(unsigned int ix)
{
    int t0, ii, val;
    unsigned char ch;
    
    if (xwd_init == 0) {
        xwd_init = 1;
        init_uart1(115200);
        delayMS(10);
    }
    uart1SendChar('e');
    uart1SendChar((char)(ix + 0x30));

    ii = 10000;  // range is 0 - 65535
    val = 0;
    while (ii) {
        t0 = readRTC();
        while (!uart1GetChar(&ch))
            if ((readRTC() - t0) > 10)  // 10msec timeout
                return 0;
        if ((ch < '0') || (ch > '9'))
            continue;
        val += (unsigned int)(ch & 0x0F) * ii;        
        ii /= 10;
    }
    while (1) {  // flush the UART1 receive buffer
        t0 = readRTC();
        while (!uart1GetChar(&ch))
            if ((readRTC() - t0) > 10)  // 10msec timeout
                return 0;
        if (ch == '\n')
            break;
    }
    return val;
}

void read_encoder_4wd()
{
    unsigned int channel;
    channel = (unsigned int)(getch() & 0x0F);
    printf("##$e%d %05d\r\n", channel, encoder_4wd(channel));
}

void testSD() {
    unsigned int numsec;
    
    InitSD();
    printf("CardInit() returns %d\r\n", CardInit());
    printf("GetCardParams() returns %d     ", GetCardParams(&numsec));
    printf("%d sectors found\r\n", numsec);
    CloseSD();
}

/* pseudo-random number generator based on Galois linear feedback shift register
     taps: 32 22 2 1; characteristic polynomial:  x^32 + x^22 + x^2 + x^1 + 1  */
#if 0
unsigned int rand() { 
    int ix, iy;
    if (rand_seed == 0x55555555) {  // initialize 
        iy = (readRTC() % 1000) + 1000;
        for (ix=0; ix<iy; ix++)
            rand_seed = (rand_seed >> 1) ^ (-(rand_seed & 0x00000001) & 0x80200003); 
    }
    for (ix=0; ix<19; ix++)  // use every 19th result
        rand_seed = (rand_seed >> 1) ^ (-(rand_seed & 0x00000001) & 0x80200003); 
    return (rand_seed);
}
#endif

unsigned int intfromthreechars() {
    unsigned char ch1, ch2, ch3;
    
    ch1 = getch() & 0x0F;
    ch2 = getch() & 0x0F;
    ch3 = getch() & 0x0F;
    return (ch1*100 + ch2*10 + ch3);
}

unsigned int intfromfourchars() {
    unsigned char ch1, ch2, ch3, ch4;
    
    ch1 = getch() & 0x0F;
    ch2 = getch() & 0x0F;
    ch3 = getch() & 0x0F;
    ch4 = getch() & 0x0F;

    return (ch1*1000 + ch2*100 + ch3*10 + ch4);
}

// Check for a new command from the PC
// Only works when PicoC is running
// Returns TRUE if the user wants to exit PicoC
unsigned int CheckForRCMBFINCommand(void)
{
    if (!gAllowBackgroundMode)
    {
        return(0);
    }
    
    // BPS: Adding ability to check for serial input to run RCM-Bfin firmware commands _while_ running a PICOC app
    MainLoop(true);

    // Check to see if the user typed a "get me out of PICOC" command ("|")
    if (!PicoCRunning)
    {
        printf("Halting PicoC and returning to RCM-Bfin command prompt.\r\n");
        PlatformExit(1);
        return(1);
    }
    else
    {
        return(0);
    }
}

// Allow user to send PicoC program as text over serial link, terminating with 0x1b (ESC, ^[)
// read in bytes to BufferPtr, max length BufferSize, until ESC is received. (ctrl-c also works)
void ReadPicoCProgram(char * BufferPtr, unsigned int BufferSize)
{
    int i = 0;
    char ch;
    bool done = false;
    
    while (!done) 
    {
        ch = getch();
        
        // ESC character or ctrl-c (to avoid problem with TeraTerm) - exit
        if (ch == 0x1B)
        { 
            BufferPtr[i] = 0x00;
            return;
        }
        else if (ch == 0x03) 
        { 
            BufferPtr[i] = 0x00;
            return;
        }
        // Backspace character has to be handled special
        else if (ch == 0x08)
        {
            // Remove the latest character from our buffer
            if (i > 0)
            {
                // Send a space and then backspace again
                putchar(' ');
                putchar(0x08);
                i--;
                BufferPtr[i] = 0x00;
            }
        }
        else
        {
            // Handle the normal storage of the byte
            BufferPtr[i] = ch;
            i++;
            if (i >= BufferSize)
            {
                BufferPtr[i] = 0x00;
                return;
            }
        }
    }
}


void ProcessPipeTest(void)
{
    int i = 0;
    char c;
    uint8_t Buffer[301];
    
    // Wait for ESC
    while ((c = getch()) != 0x1B)
    {
        if (i < 300)
        {
            Buffer[i] = c;
            i++;
        }
    }
    Buffer[i] = 0x00;
    
    printf("%s",(char *)Buffer);
}

// Print the string that indicates we don't understand the current command.
// In the future, we could make this more useful by sending some type of error
// code to this function and then it could print specific error messages based 
// on what type of error was encountered. (Unknown command, missing parameter, 
// parameter out of value, etc.)
void PrintUnknownCommand(char Command)
{
	// For debugging, print out the entire command that we just got
    printf("#?");  // unknown command
	printf(" %02X:'%c'", Command, Command);
}
