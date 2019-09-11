/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  colors.h - 
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
#include <stdbool.h>
 
#define MAX_BLOBS  63
#define MAX_COLORS 17  // reserve color #16 for internal use

#define index(xx, yy)  ((yy * imgWidth + xx) * 2) & 0xFFFFFFFC  // always a multiple of 4
// Return the X or Y co-ordinate of a pixel when given a byte index into the frame buffer
#define X_FROM_BYTE(b)  ((b/2) % imgWidth)
#define Y_FROM_BYTE(b)  (b/2/imgWidth)
// Return the byte index into a frame buffer given X and Y coordinates
#define INDEX_FROM_X_Y(x, y)  ((y * imgWidth + x) * 2)

//#define index(xx, yy)   (scaling_enabled > 0) ? ((imgWidth * (xx + ((63-yy) * imgWidth))) / 40) : ((yy * imgWidth + xx) * 2)

extern unsigned int vblob(unsigned char *, unsigned char *, unsigned int);
extern unsigned int vblob2(unsigned char *, unsigned char *, unsigned char *, unsigned int);
extern unsigned int vpix(unsigned char *, unsigned int, unsigned int);
extern unsigned int vfind(unsigned char *frame_buf, unsigned int clr, unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2);
extern void init_colors();
extern void vhist(unsigned char *frame_buf);
extern void vmean(unsigned char *frame_buf);
extern void color_segment(unsigned char *frame_buf);
extern void edge_detect(unsigned char *outbuf, unsigned char *inbuf, int threshold);

extern unsigned int ymax[], ymin[], umax[], umin[], vmax[], vmin[];
extern unsigned int blobx1[], blobx2[], bloby1[], bloby2[], blobcnt[], blobix[];
extern unsigned int hist0[], hist1[], hist2[], mean[];

unsigned int vscan(unsigned char *outbuf, unsigned char *inbuf, int thresh, 
           unsigned int columns, unsigned int *outvect);
unsigned int vhorizon(unsigned char *outbuf, unsigned char *inbuf, int thresh, 
           int columns, unsigned int *outvect, int *slope, int *intercept, int filter);

unsigned int svs_segcode(unsigned char *outbuf, unsigned char *inbuf, int thresh);
void svs_segview(unsigned char *inbuf, unsigned char *outbuf);
void addvect(unsigned char *outbuf, unsigned int columns, unsigned int *vect);
void addline(unsigned char *outbuf, int slope, int intercept);
void addbox(unsigned char *outbuf, unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2);
void addblackbox(unsigned char *outbuf, unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2);
bool process_qr_detect(unsigned char *frame_buf, char * output);
