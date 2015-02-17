/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  gps.h - 
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
 
void gps_show();
int gps_parse();
unsigned char read_ublox();
int GPScos(int);
int GPSsin(int);
int GPStan(int);
int GPSacos(int, int);
int GPSasin(int, int);
int GPSatan(int, int);
int gps_head(int, int, int, int);
int gps_dist(int, int, int, int);
unsigned int GPSisqrt(unsigned int);

typedef struct gps_data {
    int lat;  // lat degrees x 10^6
    int lon;  // lon degrees x 10^6
    int alt;
    int fix;
    int sat;
    int utc;
} gps_data;

extern struct gps_data gps_gga;

