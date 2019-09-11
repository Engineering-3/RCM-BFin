/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  colors.c - image processing routines for the RCM-Bfin Blackfin robot.
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
 
#include "system.h"
#include "colors.h"
#include <stdio.h>
#include "quirc\quirc.h"
#include "intuart.h"
#include "rcm-bfin.h"

extern unsigned int imgWidth, imgHeight;

unsigned int ymax[MAX_COLORS], ymin[MAX_COLORS], umax[MAX_COLORS], umin[MAX_COLORS], vmax[MAX_COLORS], vmin[MAX_COLORS];
unsigned int blobx1[MAX_BLOBS], blobx2[MAX_BLOBS], bloby1[MAX_BLOBS], bloby2[MAX_BLOBS], blobcnt[MAX_BLOBS], blobix[MAX_BLOBS];
unsigned int hist0[256], hist1[256], mean[3];

static struct quirc *qr;
int qr_result = 0;

void init_colors(void)
{
  unsigned int ii;
  
  for(ii = 0; ii < MAX_COLORS; ii++)
  {
    ymax[ii] = 0;
    ymin[ii] = 0;
    umax[ii] = 0;
    umin[ii] = 0;
    vmax[ii] = 0;
    vmin[ii] = 0;
  }
  
  qr = quirc_new();
  if (!qr)
  {
    printf("Failed to allocate QR struct");
    qr_result = 1;
  }
  else
  {
    if (quirc_resize(qr, 320, 240) < 0)
//    if (quirc_resize(qr, imgWidth, imgHeight) < 0)
    {
      printf("Failed to resize QR struct");
      qr_result = 2;
    }
    else
    {
      printf("quirc_resize() worked!");
      qr_result = 3;
    }
  }
}

unsigned int vpix(unsigned char *frame_buf, unsigned int xx, unsigned int yy) {
        unsigned int ix;
        ix = index(xx,yy); 
        return    ((unsigned int)frame_buf[ix] << 24) +    // returns UYVY packed into 32-bit word
                        ((unsigned int)frame_buf[ix+1] << 16) +
                        ((unsigned int)frame_buf[ix+2] << 8) +
                        (unsigned int)frame_buf[ix+3];
}


// return number of blobs found that match the search color
unsigned int vblob(unsigned char *frame_buf, unsigned char *blob_buf, unsigned int ii)
{
  unsigned int jj, ix, xx, yy, y, u, v, count, bottom, top, tmp;
  unsigned int maxx, maxy;
  unsigned char *bbp, ctmp;
  int itmp, jtmp;
  int y1, y2, u1, u2, v1, v2;
  
  y1 = ymin[ii];
  y2 = ymax[ii];
  u1 = umin[ii];
  u2 = umax[ii];
  v1 = vmin[ii];
  v2 = vmax[ii];
  
  // Initialize each blob with default (no blob) values
  for (ix = 0; ix < MAX_BLOBS; ix++)
  {
    blobcnt[ix] = 0;
    blobx1[ix] = imgWidth;
    blobx2[ix] = 0;
    bloby1[ix] = imgHeight;
    bloby2[ix] = 0;
    blobix[ix] = 0;
  }

  // Zero out the blob buffer (image buffer where blobs are recorded)
  bbp = blob_buf;
  for (ix = 0; ix < imgWidth*imgHeight*2; ix++)
  {
    *bbp++ = 0;
  }

  /* tag all pixels in blob_buf[]
       matching = 0xFF
       no color match = 0
     thus all matching pixels will belong to blob #1 */
  bbp = blob_buf;
  for (ix = 0; ix < (imgWidth*imgHeight*2); ix += 4)
  {
    y = (unsigned int)frame_buf[ix+1];
    u = (unsigned int)frame_buf[ix];
    v = (unsigned int)frame_buf[ix+2];

    if ((y >= y1) && (y <= y2) && (u >= u1) && (u <= u2) && (v >= v1) && (v <= v2))
    {
      *bbp = 0x80;
      *(bbp+1) = 0xFF;
    }
    bbp += 2;

    y = (unsigned int)frame_buf[ix+3];

    if ((y >= y1) && (y <= y2) && (u >= u1) && (u <= u2) && (v >= v1) && (v <= v2))
    {
      *bbp = 0x80;
      *(bbp+1) = 0xFF;
    }
    bbp += 2;
  }

  // Set blob pixels at outer edges of image to zero (no color match - no blobs)
  ix = (imgWidth*2) * (imgHeight-1);
  for (xx = 0; xx < imgWidth*2; xx++)
  {
    blob_buf[xx] = 0;
    blob_buf[xx+ix] = 0;
  }
  ix = (imgWidth*2)-1;
  for (yy = 0; yy < imgHeight; yy++)
  {
    blob_buf[yy*imgWidth*2] = 0;
    blob_buf[(yy*imgWidth*2) + ix] = 0;
  }

  /* Clear out orphan pixels. Walk through blob image. For each pixel,
   * count the number of pixels around it that are also part of a blob.
   * If less than four, then set this pixel to zero (i.e. no color match)
   */
  itmp = 0;
  jtmp = 0;
  for (ix = 0; ix < (imgWidth*imgHeight*2); ix += 4)
  {
    if (blob_buf[ix])
    {
      itmp++;
      ctmp = 
        blob_buf[(ix-imgWidth)-2] +
        blob_buf[ix-imgWidth] +
        blob_buf[(ix-imgWidth)+2] +
        blob_buf[ix-2] +
        blob_buf[ix+2] +
        blob_buf[(ix+imgWidth)-2] +
        blob_buf[ix+imgWidth] +
        blob_buf[(ix+imgWidth)+2];
      if (ctmp < 4)
      {
        jtmp++;
        blob_buf[ix] = 0;
      }
    }
  }
  //printf("cleared %d out of %d matching pixels\r\n", jtmp, itmp);

  /// For testing - copy over blob frame to display frame
  copy_image((unsigned char *)blob_buf, (unsigned char *)frame_buf, imgWidth, imgHeight);

  maxx = imgWidth;
  maxy = imgHeight;

  for (jj = 0; jj < MAX_BLOBS; jj++)
  {
    blobcnt[jj] = 0;      // Zero means this blob is not valid
    blobx1[jj] = maxx;    // Make each blob take up the entire image size by default
    blobx2[jj] = 0;
    bloby1[jj] = maxy;
    bloby2[jj] = 0;
  }
  
  jj = 0;    // jj indicates the current blob being processed
  for (xx = 0; xx < (maxx*2); xx += 2)    // xx is the first byte of the vertical column of pixels we are looking at
  {
    count = 0;
    bottom = maxy;
    top = 0;
    for (yy = 0; yy < maxy; yy++)     // yy is the horizontal row of pixels we are looking at
    {
      ix = xx + yy*imgWidth*2;
      if (blob_buf[ix])
      {
        count++;
        if (bottom > yy)
        {
          bottom = yy;
        }
        if (top < yy)
        {
          top = yy;
        }
      }
    }
    if (count)
    {
      if (bloby1[jj] > bottom)
      {
        bloby1[jj] = bottom;
      }
      if (bloby2[jj] < top)
      {
        bloby2[jj] = top;
      }
      if (blobx1[jj] > (xx/2))
      {
        blobx1[jj] = (xx/2);
      }
      if (blobx2[jj] < (xx/2))
      {
        blobx2[jj] = (xx/2);
      }
      blobcnt[jj] += count;
    }
    else
    {
      if (blobcnt[jj])    // move to next blob if a gap is found
      {
        jj++;
      }
      if (jj > (MAX_BLOBS-2))   // Break out of the loop if we've found too many blobs
      {
        goto blobbreak;
      }
    }
  }
blobbreak:     // now sort blobs by size, largest to smallest pixel count
  for (xx = 0; xx <= jj; xx++)
  {
    if (blobcnt[xx] == 0)    // no more blobs, so exit
    {
      return xx;
    }
    for (yy = xx; yy <= jj; yy++)
    {
      if (blobcnt[yy] == 0)
      {
        break;
      }
      if (blobcnt[xx] < blobcnt[yy])
      {
        tmp = blobcnt[xx];
        blobcnt[xx] = blobcnt[yy];
        blobcnt[yy] = tmp;
        tmp = blobx1[xx];
        blobx1[xx] = blobx1[yy];
        blobx1[yy] = tmp;
        tmp = blobx2[xx];
        blobx2[xx] = blobx2[yy];
        blobx2[yy] = tmp;
        tmp = bloby1[xx];
        bloby1[xx] = bloby1[yy];
        bloby1[yy] = tmp;
        tmp = bloby2[xx];
        bloby2[xx] = bloby2[yy];
        bloby2[yy] = tmp;
      }
    }
  }
  return xx;
}

// return number of blobs found that match the search color
// Use Brian's Enhanced Algorithm for finding blobs.
// This algorithm is really looking for the rectangular bounding box that has
// pixels of the given color within it. What this means is that it will keep 
// expanding the sides of the 'blob' (i.e. bounding box) until all of the pixels of
// the bounding box are NOT of the chosen color. This may result in some separate 
// looking blobs being put together into one bounding box because they overlap in
// x or y somewhere, but that's OK for this purpose.
// One of the requirements of this blob detection algorithm is to be able to bound
// a single pixel of the chosen color (i.e. no minimum blob size).
// The bounding box coordinates are chosen to be the outer rectangle outside the blob
// itself - i.e. the smallest rectangle that can be drawn around the blob where the 
// lines making up the rectangle are all NOT of the chosen color.
unsigned int vblob2(unsigned char *frame_buf, unsigned char *blob_buf, unsigned char *ignore_buf, unsigned int color_bin_index)
{
  unsigned int current_blob, ix, xx = 0, yy, y, u, v, count, bottom, top, tmp, x;
  unsigned char test_u, test_v, test_y1, test_y2;
  bool expand_top, expand_right, expand_bottom, expand_left;
  unsigned int maxx, maxy;
  unsigned char *bbp;
  int y1, y2, u1, u2, v1, v2;
  unsigned int test_pixel_byte;
  
  y1 = ymin[color_bin_index];
  y2 = ymax[color_bin_index];
  u1 = umin[color_bin_index];
  u2 = umax[color_bin_index];
  v1 = vmin[color_bin_index];
  v2 = vmax[color_bin_index];
  
  // Initialize each blob with default (no blob) values
  for (current_blob = 0; current_blob < MAX_BLOBS; current_blob++)
  {
    blobcnt[current_blob] = 0;
    blobx1[current_blob] = 0;
    blobx2[current_blob] = 0;
    bloby1[current_blob] = 0;
    bloby2[current_blob] = 0;
    blobix[current_blob] = 0;
  }

  // Zero out the blob buffer (image buffer where blobs are recorded)
  // And the ignore buffer (where we mark the pixels that are already part of blobs)
  bbp = blob_buf;
  for (ix = 0; ix < imgWidth*imgHeight*2; ix++)
  {
    *bbp++ = 0;
  }
  bbp = ignore_buf;
  for (ix = 0; ix < imgWidth*imgHeight*2; ix++)
  {
    *bbp++ = 0;
  }

 /* Walk through entire input frame. Look at the two pixels
  * together and if both of them match the selected bin 
  * (color_bin_index) in the input frame, then set the
  * blob detect frame pixel value to something other than zero.
  */
  bbp = blob_buf;
  for (ix = 0; ix < (imgWidth*imgHeight*2); ix += 4)
  {
    test_u  = (unsigned int)frame_buf[ix+0];
    test_y1 = (unsigned int)frame_buf[ix+1];
    test_v  = (unsigned int)frame_buf[ix+2];
    test_y2 = (unsigned int)frame_buf[ix+3];

    if (
      ((test_y1 >= y1) && (test_y1 <= y2))
      &&
      ((test_y2 >= y1) && (test_y2 <= y2))
      &&
      ((u >= u1) && (u <= u2))
      &&
      ((v >= v1) && (v <= v2))
    )
    {
      *(bbp+0) = 0x80;
      *(bbp+1) = 0x80;
      *(bbp+2) = 0x80;
      *(bbp+1) = 0x80;
    }
    bbp += 4;
  }

  // Set blob pixels at outer edges of image to zero (no color match - no blobs)
  addblackbox((unsigned char *)blob_buf, 0, imgWidth-1, 0, imgHeight-1);
#if 0
  /* Clear out orphan pixels. Walk through blob image. For each pixel,
   * count the number of pixels around it that are also part of a blob.
   * If less than four, then set this pixel to zero (i.e. no color match)
   */
  itmp = 0;
  jtmp = 0;
  for (ix = 0; ix < (imgWidth*imgHeight*2); ix += 4)
  {
    if (blob_buf[ix])
    {
      itmp++;
      ctmp = 
        blob_buf[(ix-imgWidth)-2] +
        blob_buf[ix-imgWidth] +
        blob_buf[(ix-imgWidth)+2] +
        blob_buf[ix-2] +
        blob_buf[ix+2] +
        blob_buf[(ix+imgWidth)-2] +
        blob_buf[ix+imgWidth] +
        blob_buf[(ix+imgWidth)+2];
      if (ctmp < 4)
      {
        jtmp++;
        blob_buf[ix] = 0;
      }
    }
  }
  //printf("cleared %d out of %d matching pixels\r\n", jtmp, itmp);
#endif

  /// For testing - copy over blob frame to display frame
  copy_image((unsigned char *)blob_buf, (unsigned char *)frame_buf, imgWidth, imgHeight);

  // Keep track of which blob we're on. Goes up to MAX_BLOBS then we stop looking.
  current_blob = 0;

  // Search for the next blob start pixel (test_pixel_byte is a byte index)
  for (test_pixel_byte = 0; test_pixel_byte < imgHeight*imgWidth*2; test_pixel_byte++)
  {
    // Test this pixel to see if it's in in our blob frame and not in the ignore frame
    if (blob_buf[test_pixel_byte] && !ignore_buf[test_pixel_byte])
    {
      // We've now found the start of a blob. Initialize this blob's values
      blobcnt[current_blob] = 1;                              // We have one pixel in this blob so far
      blobx1[current_blob] = X_FROM_BYTE(test_pixel_byte)-1;  // Draw a square bounding box around the pixel we've found
      blobx2[current_blob] = X_FROM_BYTE(test_pixel_byte)+1;
      bloby1[current_blob] = Y_FROM_BYTE(test_pixel_byte)-1;
      bloby2[current_blob] = Y_FROM_BYTE(test_pixel_byte)+1;
      // Mark this pixel in the ignore frame
      ignore_buf[test_pixel_byte] = 1;
      
      // Walk around the bounding box, looking for more pixels that are in the blob
      expand_top = false;
      expand_right = false;
      expand_bottom = false;
      expand_left = false;
      
      // First the top line (from x1 to x2 along y1)
      for (x = blobx1[current_blob]; x <= blobx2[current_blob]; x++)
      {
        // Always mark this pixel as "we've already checked it, so we can ignore it from now on"
        ignore_buf[INDEX_FROM_X_Y(x, bloby1[current_blob])] = 1;
        if (blob_buf[INDEX_FROM_X_Y(x, bloby1[current_blob])])
        {
          // Yup, found another pixel in the blob, so count it
          blobcnt[current_blob]++;
          // And mark this 'side' of the bounding box for expansion
          expand_top = true;
        }
      }
      
      // Then the right hand side (from y1 to y2 along x2)
      for (y = bloby1[current_blob]; y <= bloby2[current_blob]; y++)
      {
        // Always mark this pixel as "we've already checked it, so we can ignore it from now on"
        ignore_buf[INDEX_FROM_X_Y(blobx2[current_blob], y)] = 1;
        if (blob_buf[INDEX_FROM_X_Y(blobx2[current_blob], y)])
        {
          // Yup, found another pixel in the blob, so count it
          blobcnt[current_blob]++;
          // And mark this 'side' of the bounding box for expansion
          expand_right = true;
        }
      }
      
      // Next the bottom line (from x1 to x2 along y2)
      for (x = blobx1[current_blob]; x <= blobx2[current_blob]; x++)
      {
        // Always mark this pixel as "we've already checked it, so we can ignore it from now on"
        ignore_buf[INDEX_FROM_X_Y(x, bloby2[current_blob])] = 1;
        if (blob_buf[INDEX_FROM_X_Y(x, bloby2[current_blob])])
        {
          // Yup, found another pixel in the blob, so count it
          blobcnt[current_blob]++;
          // And mark this 'side' of the bounding box for expansion
          expand_top = true;
        }
      }
      
      // Then the left hand side (from y1 to y2 along x1)
      for (y = bloby1[current_blob]; y <= bloby2[current_blob]; y++)
      {
        // Always mark this pixel as "we've already checked it, so we can ignore it from now on"
        ignore_buf[INDEX_FROM_X_Y(blobx1[current_blob], y)] = 1;
        if (blob_buf[INDEX_FROM_X_Y(blobx1[current_blob], y)])
        {
          // Yup, found another pixel in the blob, so count it
          blobcnt[current_blob]++;
          // And mark this 'side' of the bounding box for expansion
          expand_right = true;
        }
      }
      
      // Do we need to expand any sides of the blob's bounding box?
      if (expand_top == true || expand_right == true || expand_bottom == true || expand_left == true)
      {
        // Yes. Based on which sides need expanding, expand them
        if (expand_top)
        {
          
        }
      }
      else
      {
        // No, so this blob is now completely bounded by empty space and is completely done.
      }
    }
  }
  
#if 0
  maxx = imgWidth;
  maxy = imgHeight;

  for (jj = 0; jj < MAX_BLOBS; jj++)
  {
    blobcnt[jj] = 0;      // Zero means this blob is not valid
    blobx1[jj] = maxx;    // Make each blob take up the entire image size by default
    blobx2[jj] = 0;
    bloby1[jj] = maxy;
    bloby2[jj] = 0;
  }

  
  jj = 0;    // jj indicates the current blob being processed
  for (xx = 0; xx < (maxx*2); xx += 2)    // xx is the first byte of the vertical column of pixels we are looking at
  {
    count = 0;
    bottom = maxy;
    top = 0;
    for (yy = 0; yy < maxy; yy++)     // yy is the horizontal row of pixels we are looking at
    {
      ix = xx + yy*imgWidth*2;
      if (blob_buf[ix])
      {
        count++;
        if (bottom > yy)
        {
          bottom = yy;
        }
        if (top < yy)
        {
          top = yy;
        }
      }
    }
    if (count)
    {
      if (bloby1[jj] > bottom)
      {
        bloby1[jj] = bottom;
      }
      if (bloby2[jj] < top)
      {
        bloby2[jj] = top;
      }
      if (blobx1[jj] > (xx/2))
      {
        blobx1[jj] = (xx/2);
      }
      if (blobx2[jj] < (xx/2))
      {
        blobx2[jj] = (xx/2);
      }
      blobcnt[jj] += count;
    }
    else
    {
      if (blobcnt[jj])    // move to next blob if a gap is found
      {
        jj++;
      }
      if (jj > (MAX_BLOBS-2))   // Break out of the loop if we've found too many blobs
      {
        goto blobbreak;
      }
    }
  }
blobbreak:     // now sort blobs by size, largest to smallest pixel count
  for (xx = 0; xx <= jj; xx++)
  {
    if (blobcnt[xx] == 0)    // no more blobs, so exit
    {
      return xx;
    }
    for (yy = xx; yy <= jj; yy++)
    {
      if (blobcnt[yy] == 0)
      {
        break;
      }
      if (blobcnt[xx] < blobcnt[yy])
      {
        tmp = blobcnt[xx];
        blobcnt[xx] = blobcnt[yy];
        blobcnt[yy] = tmp;
        tmp = blobx1[xx];
        blobx1[xx] = blobx1[yy];
        blobx1[yy] = tmp;
        tmp = blobx2[xx];
        blobx2[xx] = blobx2[yy];
        blobx2[yy] = tmp;
        tmp = bloby1[xx];
        bloby1[xx] = bloby1[yy];
        bloby1[yy] = tmp;
        tmp = bloby2[xx];
        bloby2[xx] = bloby2[yy];
        bloby2[yy] = tmp;
      }
    }
  }
#endif
  return xx;
}

// histogram function - 
//  hist0[] holds frequency of u|v combination  
//  hist1[] holds average luminance corresponding each u|v combination

void vhist(unsigned char *frame_buf) {
    unsigned int ix, iy, xx, yy, y1, u1, v1;

    for (ix=0; ix<256; ix++) {          hist0[ix] = 0;  // accumulator 
        hist1[ix] = 0;  
    }
    for (xx=0; xx<imgWidth; xx+=2) {   
        for (yy=0; yy<imgHeight; yy++) {
            ix = index(xx,yy);  
            y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            u1 = ((unsigned int)frame_buf[ix]);
            v1 = ((unsigned int)frame_buf[ix+2]);
            iy = (u1 & 0xF0) + (v1 >> 4);
            hist0[iy]++;
            hist1[iy] += y1;
        }
    }
    for (ix=0; ix<256; ix++)
        if (hist1[ix])
            hist1[ix] /= hist0[ix];  // normalize by number of hits
}

/* mean color function - computes mean value for Y, U and V
   mean[0] = Y mean, mean[1] = U mean, mean[2] = V mean */
void vmean(unsigned char *frame_buf) {
    unsigned int ix, xx, yy, y1, u1, v1;
    unsigned int my, mu, mv;
    my = mu = mv = 0;

    for (xx=0; xx<imgWidth; xx+=2) {   
        for (yy=0; yy<imgHeight; yy++) {
            ix = index(xx,yy);  // yx, uv, vx range from 0-63 (yuv value divided by 4)
            y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            u1 = ((unsigned int)frame_buf[ix]);
            v1 = ((unsigned int)frame_buf[ix+2]);
            my += y1;
            mu += u1;
            mv += v1;
        }
    }
    mean[0] = ((my*2) / imgWidth) / imgHeight;
    mean[1] = ((mu*2) / imgWidth) / imgHeight;
    mean[2] = ((mv*2) / imgWidth) / imgHeight;
}

void color_segment(unsigned char *frame_buf)
{
  unsigned int ix, xx, yy, y, u, v, clr;
  unsigned int ymid[MAX_COLORS], umid[MAX_COLORS], vmid[MAX_COLORS];

  for (ix = 0; ix < MAX_COLORS; ix++)
  {
    ymid[ix] = (ymax[ix] + ymin[ix]) >> 1;
    umid[ix] = (umax[ix] + umin[ix]) >> 1;
    vmid[ix] = (vmax[ix] + vmin[ix]) >> 1;
  }
  for (xx = 0; xx < imgWidth; xx += 2)
  {
    for (yy = 0; yy < imgHeight; yy++)
    {
      ix = index(xx,yy);
      y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
      //y = (unsigned int)frame_buf[ix+1];
      u = (unsigned int)frame_buf[ix];
      v = (unsigned int)frame_buf[ix+2];
      for (clr = 0; clr < MAX_COLORS; clr++)
      {
        if (ymax[clr] == 0)    // skip this color if not defined
        {
          continue;
        }
        if (
             (y >= ymin[clr])
          && (y <= ymax[clr])
          && (u >= umin[clr])
          && (u <= umax[clr])
          && (v >= vmin[clr])
          && (v <= vmax[clr])
        )
        {
          frame_buf[ix+1] = frame_buf[ix+3] = ymid[clr];
          frame_buf[ix] = umid[clr];
          frame_buf[ix+2] = vmid[clr];
          break;
        }
      }
      if (clr == MAX_COLORS)  // if no match, black out the pixel
      {
        frame_buf[ix+1] = frame_buf[ix+3] = 0;
        frame_buf[ix] = frame_buf[ix+2] = 128;
      }
    }
  }
}

/* count number of pixels matching 'clr' bin in range [x1,y1] to [x2,y2] */
unsigned int vfind (
  unsigned char *frame_buf,
  unsigned int clr,
  unsigned int x1,
  unsigned int x2,
  unsigned int y1,
  unsigned int y2
)
{
  unsigned int ix, xx, yy, y, u, v, count;
  
  count = 0;
  for (xx = x1; xx < x2; xx += 2)
  {
    for (yy = y1; yy < y2; yy++)
    {
      ix = index(xx,yy);
      y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
      //y = (unsigned int)frame_buf[ix+1];
      u = (unsigned int)frame_buf[ix];
      v = (unsigned int)frame_buf[ix+2];
      if (
           (y >= ymin[clr])
        && (y <= ymax[clr])
        && (u >= umin[clr])
        && (u <= umax[clr])
        && (v >= vmin[clr])
        && (v <= vmax[clr])
      )
      {
        count++;
      }
    }
  }
  return count;
}

void edge_detect(unsigned char *inbuf, unsigned char *outbuf, int thresh) {
    unsigned int ix, xx, yy, y2, u2, v2, skip;
    unsigned char *ip, *op;
    unsigned int *ip1, *op1;
    unsigned int gx, gy;
    
    skip = imgWidth*2;
    for (xx=2; xx<imgWidth-2; xx+=2) {   
        for (yy=1; yy<imgHeight-1; yy++) {
            gx = gy = 0;
            ix = index(xx, yy);
            ip = inbuf + ix;
            op = outbuf + ix;

            y2 = *(ip+5) + *(ip+7) - *(ip-3) - *(ip-1);
            u2 = *(ip+4) - *(ip-4);
            v2 = *(ip+6) - *(ip-2);
            gy = ((y2*y2)>>2) + u2*u2 + v2*v2;
            
            y2 = *(ip+1+skip) + *(ip+3+skip) - *(ip+1-skip) - *(ip+3-skip);
            u2 = *(ip+skip) - *(ip-skip); 
            v2 = *(ip+2+skip) - *(ip+2-skip); 
            gx = ((y2*y2)>>2) + u2*u2 + v2*v2;
 
            if ((gx > thresh) || (gy > thresh)) {
                *op = 128;
                *(op+2) = 128;
                *(op+1) = 255;
                *(op+3) = 255;
            } else {
                *op = (*(ip) & 0x80) | 0x40;
                *(op+2) = (*(ip+2) & 0x80) | 0x40;
                *(op+1) = (*(ip+1) & 0xC0) | 0x20;
                *(op+3) = (*(ip+3) & 0xC0) | 0x20;
            }
        }
    }

    op1 = (unsigned int *)outbuf;
    ip1 = (unsigned int *)inbuf;
    for (ix=0; ix<(imgWidth*imgHeight>>1); ix++)
        ip1[ix] = op1[ix];
}

unsigned int vscan(unsigned char *outbuf, unsigned char *inbuf, int thresh, 
           unsigned int columns, unsigned int *outvect) {
    int x, y;
    unsigned int ix, hits;
    unsigned char *pp;
    
    svs_segcode(outbuf, inbuf, thresh);  // find edge pixels

    hits = 0;
    pp = outbuf;
    for (ix=0; ix<columns; ix++)    // initialize output vector
        outvect[ix] = imgHeight;
        
    for (y=0; y<imgHeight; y++) {  // now search from top to bottom, noting each hit in the appropriate column
        for (x=0; x<imgWidth; x+=2) {  // note that edge detect used full UYVY per pixel position
            if (*pp & 0xC0) {   // look for edge hit 
                outvect[((x * columns) / imgWidth)] = imgHeight - y;
                hits++;
            }
            pp++;
        }
    }
    return hits;
}

/* search for image horizon.  similar to vscan(), but search is top-to-bottom rather than bottom-to-top */
unsigned int vhorizon(unsigned char *outbuf, unsigned char *inbuf, int thresh, 
           int columns, unsigned int *outvect, int *slope, int *intercept, int filter) {
    int x, y;
    int ix, hits;
    unsigned char *pp;
    static int sfilter[10], ifilter[10];
    
    if (filter > 10) filter = 10;  // max number of filter taps
    if (filter < 1) filter = 1;
    
    svs_segcode(outbuf, inbuf, thresh);  // find edge pixels

    hits = 0;
    for (ix=0; ix<columns; ix++)    // initialize output vector
        outvect[ix] = 0;
        
    for (y=imgHeight-1; y>=0; y--) {  // now search from top to bottom, noting each hit in the appropriate column
        pp = outbuf + (y * imgWidth / 2);
        for (x=0; x<imgWidth; x+=2) {  // note that edge detect used full UYVY per pixel position
            if (*pp & 0xC0) {   // look for edge hit 
                outvect[((x * columns) / imgWidth)] = y;
                hits++;
            }
            pp++;
        }
    }
    /*int sx, sy, sxx, sxy, dx, delta;
    sx = sy = sxx = sxy = 0;
    dx = imgWidth / columns;
    for (ix=0; ix<columns; ix++) {
        x = ix*dx + dx/2;
        y = (int)outvect[ix];
        sx += x;
        sy += y;
        sxx += x*x;
        sxy += x*y;
    }
    delta = columns*sxx - sx*sx;
    *slope = (1000 * (columns*sxy - sx*sx))/ delta;  // slope is scaled by 1000
    *intercept = (sxx*sy - sx*sxy) / delta; */
    int sx, sy, stt, sts, t, dx;
    sx = sy = stt = sts = 0;
    dx = imgWidth / columns;
    for (ix = 0; ix < columns; ix++) {
        sx += ix*dx + dx/2;
        sy += (int)outvect[ix];
    }
    for (ix = 0; ix < columns; ix++) {
        t = ix*dx + dx/2 - sx/columns;
        stt += t*t;
        sts += t*(int)outvect[ix];
    }
	*slope = (1000*sts)/stt;
	*intercept = (sy*1000 - sx*(*slope))/(columns*1000);
    if (*intercept > (imgHeight-1)) *intercept = imgHeight-1;
    if (*intercept < 0) *intercept = 0;

    if (filter > 1) {
        for (ix=filter; ix>1; ix--) {  // push old values to make room for latest
            sfilter[ix-1] = sfilter[ix-2];
            ifilter[ix-1] = ifilter[ix-2];
        }
        sfilter[0] = *slope;
        ifilter[0] = *intercept;
        sx = sy = 0;
        for (ix=0; ix<filter; ix++) {  // now average the data
            sx += sfilter[ix];
            sy += ifilter[ix];
        }
        *slope = sx / filter;
        *intercept = sy / filter;
    }
    //printf("vhorizon:  slope = %d   intercept = %d\r\n", *slope, *intercept);
    return hits;
}

unsigned int svs_segcode(unsigned char *outbuf, unsigned char *inbuf, int thresh) {
    unsigned int ix, xx, yy, y2, u2, v2, skip;
    unsigned char *ip, *op, cc;
    unsigned int gx, gy, edgepix;
    
    if (imgWidth > 640)   // buffer size limits this function to 640x480 resolution
        return 0;
    
    skip = imgWidth*2;
    op = outbuf;
    edgepix = 0;
    for (yy=0; yy<imgHeight; yy++) {
        for (xx=0; xx<imgWidth; xx+=2) {   
            if ((xx < 2) || (xx >= imgWidth-2) || (yy < 1) || (yy >= imgHeight-1)) {
                *op++ = 0;
                continue;
            }
            gx = gy = 0;
            ix = index(xx, yy);
            ip = inbuf + ix;

            y2 = *(ip+5) + *(ip+7) - *(ip-3) - *(ip-1);
            u2 = *(ip+4) - *(ip-4);
            v2 = *(ip+6) - *(ip-2);
            gy = ((y2*y2)>>2) + u2*u2 + v2*v2;
            
            y2 = *(ip+1+skip) + *(ip+3+skip) - *(ip+1-skip) - *(ip+3-skip);
            u2 = *(ip+skip) - *(ip-skip); 
            v2 = *(ip+2+skip) - *(ip+2-skip); 
            gx = ((y2*y2)>>2) + u2*u2 + v2*v2;
 
            cc = ((*ip >> 2) & 0x38)       // threshold U to 0x38 position
               + ((*(ip+2) >> 5) & 0x07);  // threshold V to 0x07 position
            if (gx > thresh)  
                cc |= 0x80;               // add 0x80 flag if this is a horizontal edge pixel
            if (gy > thresh)  
                cc |= 0x40;               // add 0x40 flag if this is a vertical edge pixel
            if (cc & 0xC0)
                edgepix++;
            *op++ = cc;
        }
    }
    return edgepix;
}

void svs_segview(unsigned char *inbuf, unsigned char *outbuf) {
    unsigned int ix;
    unsigned char *ip, *op;

    if (imgWidth > 640)   // buffer size limits this function to 640x480 resolution
        return;
    
    ip = inbuf;
    op = outbuf;
    for (ix=0; ix<imgWidth*imgHeight; ix+=2) {   
        if (*ip & 0xC0) {      // is this an edge pixel ?
            *(op+1) = *(op+3) = 0xFF;
            *op = *(op+2) = 0x80;
        } else {
            *(op+1) = *(op+3) = 0xA0;
            *op = ((*ip & 0x38) << 2) + 0x10;
            *(op+2) = ((*ip & 0x07) << 5) + 0x10;
        }
        op += 4;
        ip++;
    }
}

/* display vector as red pixels (YUV = 72 84 255) */
void addvect(unsigned char *outbuf, unsigned int columns, unsigned int *vect)
{
    unsigned int xx, yy, ix;

    if (imgWidth > 640)   // buffer size limits this function to 640x480 resolution
        return;
    for (xx=0; xx<imgWidth; xx++) {
        yy = (imgHeight) - (vect[(xx * columns) / imgWidth]);
        ix = index(xx, yy);
        outbuf[ix+1] =  outbuf[ix+3] = 72;
        outbuf[ix] = 84;
        outbuf[ix+2] = 255;
    }
}

/* display line as red pixels (red YUV = 72 84 255) (yellow YUV = 194 18 145)
   note that slope is scaled up by 1000 */
void addline(unsigned char *outbuf, int slope, int intercept)
{
    int xx, yy, ix;

    if (imgWidth > 640)   // buffer size limits this function to 640x480 resolution
        return;
    for (xx=0; xx<imgWidth; xx+=2) {
        yy = ((slope * xx) / 1000) + intercept; 
        if (yy > (imgHeight-1)) yy = imgHeight - 1;
        if (yy < 0) yy = 0; 
        ix = index(xx, yy);
        outbuf[ix+1] =  outbuf[ix+3] = 72;
        outbuf[ix] = 84;
        outbuf[ix+2] = 255;
    }
}

/* display a box as red pixels (YUV = 72 84 255) or yellow pixels (YUV = 194 18 145) */
void addbox(unsigned char *outbuf, unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2)
{
  unsigned int xx, yy, ix;

  for (xx=x1; xx<=x2; xx+=2) {
    ix = index(xx, y1);
    outbuf[ix+1] =  outbuf[ix+3] = 194;
    outbuf[ix] = 18;
    outbuf[ix+2] = 145;
    ix = index(xx, y2);
    outbuf[ix+1] =  outbuf[ix+3] = 194;
    outbuf[ix] = 18;
    outbuf[ix+2] = 145;
  }
  for (yy=y1; yy<=y2; yy++) {
    ix = index(x1, yy);
    outbuf[ix+1] =  outbuf[ix+3] = 194;
    outbuf[ix] = 18;
    outbuf[ix+2] = 145;
    ix = index(x2, yy);
    outbuf[ix+1] =  outbuf[ix+3] = 194;
    outbuf[ix] = 18;
    outbuf[ix+2] = 145;
  }
}

void addblackbox(unsigned char *outbuf, unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2)
{
  unsigned int xx, yy, ix;

  for (xx=x1; xx<=x2; xx+=2) {
    ix = index(xx, y1);
    outbuf[ix+1] =  outbuf[ix+3] = 0;
    outbuf[ix] = 0;
    outbuf[ix+2] = 0;
    ix = index(xx, y2);
    outbuf[ix+1] =  outbuf[ix+3] = 0;
    outbuf[ix] = 0;
    outbuf[ix+2] = 0;
  }
  for (yy=y1; yy<=y2; yy++) {
    ix = index(x1, yy);
    outbuf[ix+1] =  outbuf[ix+3] = 0;
    outbuf[ix] = 0;
    outbuf[ix+2] = 0;
    ix = index(x2, yy);
    outbuf[ix+1] =  outbuf[ix+3] = 0;
    outbuf[ix] = 0;
    outbuf[ix+2] = 0;
  }
}

// Return true if there's a QR code in frame_buf, and fill in output string with the decoded value
// Return false if there's not a QR code in frame_buf
bool process_qr_detect(unsigned char *frame_buf, char * output)
{
  uint8_t *image;
  int w,h;
  int num_codes;
  int i;
  int ix;
  unsigned int *ip1;
  bool RetVal = false;
  
  image = quirc_begin(qr, &w, &h);
  i = 0;
  // Fill out image here with greyscale pixels
  ip1 = (unsigned int *)frame_buf;
  for (ix=0; ix < ((imgWidth*imgHeight)/2); ix++)
  {
    *image = (uint8_t)(ip1[ix] >> 8);
    image++;
    *image = (uint8_t)(ip1[ix] >> 24);
    image++;
    // Set U and V to 128 and let Y pass through for both pixels
    //ip1[ix] = (ip1[ix] & 0xFF00FF00) | 0x00800080;
  }

  quirc_end(qr);

  /* We've previously fed an image to the decoder via
   * quirc_begin/quirc_end.
   */

  num_codes = quirc_count(qr);
  
  if (num_codes) {
    //PacketBegin();
    //printf("Found %d QR codes\n", num_codes);
    //PacketEnd(true);

    PacketBegin();
    // Assume only one QR code
    struct quirc_code code;
    struct quirc_data data;
    quirc_decode_error_t err;

    quirc_extract(qr, 0, &code);

    /* Decoding stage */
    err = quirc_decode(&code, &data);
    if (err)
    {
      printf("QR code found, but decode failed due to %s\n", quirc_strerror(err));
    }
    else
    {
      int maxX = 0, minX = 1000, maxY = 0, minY = 1000, i = 0;
      printf("QR code found, decoded = '%s'\n", data.payload);
      // Copy over the string that's contained in the decoded QR code
      strncpy((char *)output, (char *)data.payload, 40);
      // And set up the bounding box values to display on the image frame
      // Find the actual bounding box based on the four corners of the QR code
      for (i=0; i < 4; i++)
      {
        //printf("corner %d x=%d y=%d\n", i, code.corners[i].x, code.corners[i].y);
        if (code.corners[i].x > maxX) {
          maxX = code.corners[i].x;
        }
        if (code.corners[i].y > maxY) {
          maxY = code.corners[i].y;
        }
        if (code.corners[i].x < minX) {
          minX = code.corners[i].x;
        }
        if (code.corners[i].y < minY) {
          minY = code.corners[i].y;
        }
      }
      blobx1[0] = minX;
      blobx2[0] = maxX;
      bloby1[0] = minY;
      bloby2[0] = maxY;
      //printf("x1=%d y1=%d x2=%d y2=%d\n", blobx1[0], bloby1[0], blobx2[0], bloby2[0]);
      RetVal = true;
    }
    PacketEnd(true);
  }
  return(RetVal);
}
