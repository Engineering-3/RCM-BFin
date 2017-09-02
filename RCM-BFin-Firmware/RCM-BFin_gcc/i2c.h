/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  i2c.h -
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
 
#define SCCB_ON  1   // enables SCCB mode for I2C (used by OV9655 camera)
#define SCCB_OFF 0

#define RCM2_I2C_ADDRESS	0x18	// default I2C address on ARM on RCM2
#define I2C_TIMEOUT_RELOAD	20		// in ms

extern int InitI2C(void);
extern int i2cwrite(unsigned char i2c_device, unsigned char *i2c_data, unsigned int pair_count, int sccb_flag);
extern int i2cwritex(unsigned char i2c_device, unsigned char *i2c_data, unsigned int count, int sccb_flag);
extern int i2cread(unsigned char i2c_device, unsigned char *i2c_data, unsigned int pair_count, int sccb_flag);
extern int i2creadrs(unsigned char i2c_device, unsigned char *i2c_data, unsigned int pair_count, int sccb_flag);
extern int i2cslowread(unsigned char i2c_device, unsigned char *i2c_data, unsigned int pair_count, int sccb_flag);
extern int i2c_read_timed(unsigned char i2c_device, unsigned char *i2c_data, unsigned int data_count, int sccb_flag, int delay_us);
extern int i2c_read_timed2(
	unsigned char i2c_device, 
	unsigned char *i2c_data_out, 
	unsigned int data_count_out, 
	unsigned char *i2c_data_in, 
	unsigned int data_count_in, 
	int sccb_flag, int delay_us
);
