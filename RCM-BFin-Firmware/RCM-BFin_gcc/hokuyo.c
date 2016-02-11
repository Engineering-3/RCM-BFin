/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  hokuyo.c - routines for talking to a Hokuyo sensor
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
 
// Use this define to enable operation without a Hokuyo sensor (and with faked data)
//#define NO_HOKUYO_SENSOR
 
// Use this define to enable the alternate printout format (X,Y) each on separate 
// line, rather than polar all on one line)
#define HOKUYO_ALT_FORMAT
 
#define HOKUYO_OFFSET   (180)
 
#include "system.h"
#include <cdefBF537.h>
#include "config.h"
#include <stdio.h>
#include <string.h>
#include "rcm-bfin.h"
#include "i2c.h"
#include "hokuyo.h"
#include <math.h>
#include "gps.h"

#define HOKUYO_DATA_POINT_COUNT		1024		// Note this is after heading correction
#define CHAR_LF 					10
#define HOKUYO_TEMP_BUF_SIZE		2048

/* General globals */
// Global offset for compass heading, in degrees
signed int CompassOffset = 0;

/* Reads out all EEPROM contents of HMC6343 compass and prints them */
void HokuyoPrintEEPROM(void)
{
    unsigned char i2c_data_out[6];
    unsigned char i2c_data_in[6];
    short i;
    unsigned char addr = 0;

    // HMC6343
    addr = 0x19;

    // Read out and dump all EEPROM registers
    // Initalize the spot where we'll put the data
    for(i = 0; i < 6; i++) i2c_data_in[i] = 0x00;
    for(i = 0; i < 6; i++) i2c_data_out[i] = 0x00;

    i2c_data_out[0] = 0xE1;
    i2c_data_out[1] = 0x00;

    printf("\n");

    for (i=0; i < 22; i++)
    {
       i2c_data_out[1] = i;

        // Perform the read - 1 byte
        i2c_read_timed2(addr, (unsigned char *)i2c_data_out, 2, (unsigned char *)i2c_data_in, 1, SCCB_OFF, 12000);

        printf("EEPROM %x = %X\n", i2c_data_out[1], i2c_data_in[0]);
    }
}

/* Query a HMC6343 Honeywell compass over I2c and get the current
heading. Return a negative number if there was an error. Otherwise
return a positive integer from 0 to 359 in degrees.

The compass defaults to 0x32 as its address (0x32 for write, 0x33 for read) (so we use 0x19)
To read the data, we need to send it a 0x32 (write),0x40 (heading data) then reset the bus and
send a 0x33 (read), and then read out 6 bytes of data. The three heading values then come
back Heading, Pitch, Roll, with two bytes each, MSB first.

This code is based on the compass code in rcm-bfin.c
*/
signed int hmc6343_read(void)
{
    unsigned char i2c_data_in[6];
    short i;
    unsigned char addr = 0;
    short h, p, r;
    
    delayMS(50);  // limit updates to 20Hz

    // HMC6343
    addr = 0x19;

#ifdef NO_HOKUYO_SENSOR
	return(144);
#endif

    // Initalize the spot where we'll put the data
    for(i = 0; i < 6; i++) i2c_data_in[i] = 0x00;

    // 0x50 is the command code to read out the heading registers
    i2c_data_in[0] = 0x50;

    // Perform the read - 6 bytes
    i2c_read_timed(addr, (unsigned char *)i2c_data_in, 6, SCCB_OFF, 2000);

    // Pull apart the data
    h = ((short) (i2c_data_in[0] * 256 + i2c_data_in[1]));
    p = ((short) (i2c_data_in[2] * 256 + i2c_data_in[3]));
    r = ((short) (i2c_data_in[4] * 256 + i2c_data_in[5]));

//printf("Compass returned: h=%d p=%d r=%d\n", h, p, r);

    if ((h == (short)0xFFFF) && (p == (short)0xFFFF) && (r == (short)0xFFFF))  // compass read failed - return -999
        return (-999);  

    return (h/10);
}

// Use hmc6343_read() and just print out heading
void HokuyoPringHeading(void)
{
    printf("%03d\n", hmc6343_read());
}

// Clear out variation, deviation and hard iorn calibartion EEPROM loations to zero
void HokuyoCompassEEPROMClear(void)
{
    unsigned char i2c_data[6];
    unsigned char addr = 0;
    unsigned char i;

    // HMC6343
    addr = 0x19;

    // 0xF1 is the command code to write EEPROM, 0x0C is the LSB of the variation angle
    i2c_data[0] = 0xF1;
    i2c_data[2] = 0x00;

    // Loop through addresses 0x0A to 0x13 and write zeros to them
    for (i=0x0A; i <= 0x13; i++)
    {
        i2c_data[1] = i;
        // Perform the write
        i2cwritex(addr, (unsigned char *)i2c_data, 3, SCCB_ON);
        delayMS(10);
    }
}

/* Listen to UART1 for a complete response from the Hokuyo sensor.
This will consist of an echo of the command we just sent, plus
the sensor's response, terminated by two <LF> characters in a row.

Temp needs to be long enough to hold the maximum number of characters
that can come back.

timeout is in milliseonds

Return zero on success, 1 if timeout or too many bytes
*/
signed int hokuyo_receive(unsigned char Temp[HOKUYO_TEMP_BUF_SIZE], unsigned int timeout)
{
	unsigned int i = 0;
	unsigned char Char1 = 0, Char2 = 0;
	unsigned int StartTime = readRTC();
	unsigned char Done = FALSE;
		
	while (!Done)
	{
		// Check to see if there is a new character for us
		if((*pUART1_LSR & DR))
		{
			Temp[i] = *pUART1_RBR;
			Char2 = Char1;
			Char1 = Temp[i];

//printf("Got '%c' %d at %d\n", Char1, Char1, i);
			
			i++;
			if (i >= HOKUYO_TEMP_BUF_SIZE)
			{
				Temp[i] = 0x00;
				return(-1);
			}
			
			StartTime = readRTC();
		}
		
		// Check for timeout
		if (readRTC() > (StartTime + timeout))
		{
//printf("Timeout on read after %d\n", timeout);
			return(-2);
		}
		
		// Are we done with this response?
		if (Char1 == CHAR_LF && Char2 == CHAR_LF)
		{
			Done = TRUE;
		}
	}

	Temp[i] = 0x00;

	return(0);
}

// Take the two ASCII chars pointed to by Chars
// and convert them to a 12 bit number and return it
unsigned int hokuyo_convert(unsigned char * Chars)
{
    unsigned Char1 = 0, Char2 = 2;

    Char1 = *Chars - 0x30;
    Char2 = *(Chars+1) - 0x30;

    return (((int)(Char1 & 0x3F) << 6) | (Char2 & 0x3F));
}

// Take the raw string of ASCII data that the HOKUYO spits out, and parse it  into our unsigned int Data array.
// Return non-zero on any errors. Follow the URG_SCIP11.pdf doc for information.
// Also correct for heading - i.e. take the heading (in degrees) and slide all of the output array around
// such that sample 0 is always pointed directly north (heading zero), with all other samples following
// counter clockwise (from the top) all the way to sample 1023 which is 359.64 degrees.
signed int 	hokuyo_process(
	unsigned char Temp[HOKUYO_TEMP_BUF_SIZE], 
	unsigned int Data[HOKUYO_DATA_POINT_COUNT],
	unsigned int Heading
)
{
	unsigned int i;
	unsigned int DataPointIndex = 0;
	unsigned int TempIndex = 0;
	unsigned int LastByte = 0;
	unsigned int SampleCorrectionOffset = 0;

	// Calculate the SampleCorrectionOffset.
	// Since sample 384 represents zero degrees (to the sensor - straight ahead)
	// and each sample represents 360/1024 (or .35156) degrees, and we have a heading value that tells
	// what direction (relative to north) sample 384 is really pointing, we create this correction
	// value to 'slide' the data around the output array so that sample 0 of the Data array always holds
	// the sample that's at true north.
	SampleCorrectionOffset = (639 - ((Heading * 1024)/360) + 180) % HOKUYO_DATA_POINT_COUNT;
	if (SampleCorrectionOffset == 0)
	{
		SampleCorrectionOffset = 1;
	}

//printf("Sample correction = %d, heading = %d\n", SampleCorrectionOffset, Heading);
	
	// For now, we're going to ignore the 'G', StartingPoint, EndingPoint, and ClusterCount that the sensor sends back,
	// other than to check that those bytes all contain ASCII data.

	// Find the last byte of data in the stream
	for (i=0; i < HOKUYO_TEMP_BUF_SIZE; i++)
	{
		if (Temp[i] == 0x00)
		{
			LastByte = i;
			break;
		}
	}

	// First check to make sure everything is ASCII. All bytes coming from the sensor are supposed to fall into the range
	// 0x30 to 0x6F, or 0x0A (linefeed).
	for (i=0; i < LastByte; i++)
	{
		if (Temp[i] != CHAR_LF && (Temp[i] < 0x30 || Temp[i] > 0x6F))
		{
//printf("Bad data bytes\n");
		return(-1);
		}
	}

	// Make sure we have at least one set of real data
	if (LastByte < 16)
	{
//printf("Not enough bytes of data\n");
		return (-2);
	}
	
	// Skip ahead to the first byte of real data
	TempIndex = 10;
	
	// Grab the status value - it always goes in location zero of the data array
	Data[0] = atoi((char *)&Temp[TempIndex]);
//printf("got a %d for status\n", Data[0]);

	while (Temp[TempIndex] != CHAR_LF)
	{
		TempIndex++;
		if (TempIndex > LastByte)
		{
			return(-3);
		}
	}
	if (Temp[TempIndex] == CHAR_LF && Temp[TempIndex + 1] == CHAR_LF)
	{
		return(0);
	}
	TempIndex++;
	if (TempIndex > LastByte)
	{
		return(-3);
	}

	// Start recording data into the data array with our correction 
	// offset. Incriment from there.
	DataPointIndex = SampleCorrectionOffset;

	// Now grab all of the data elements
	while (TempIndex < LastByte)
	{
		if (Temp[TempIndex] != CHAR_LF)
		{
			// There is a dead zone from data 1 through 44, and from 725 through 768 so 
			// use zeros for that.
			if (TempIndex > (44 * 2) && TempIndex < (725 * 2))
			{
				Data[DataPointIndex] = hokuyo_convert(&Temp[TempIndex]);
			}
			else
			{
				hokuyo_convert(&Temp[TempIndex]);
			}
			
			DataPointIndex++;
			if (DataPointIndex >= HOKUYO_DATA_POINT_COUNT)
			{
				DataPointIndex = 1;		// Skip over data point zero
			}
			TempIndex += 2;
		}
		else
		{
			TempIndex++;
		}
	}
	
	return(0);
}

// Print out the array Data to the PC.
// Note that the first element in the array is the status, not a data point
signed int hokuyo_print(
	unsigned int Data[HOKUYO_DATA_POINT_COUNT], 
	unsigned int StartingPoint, 
	unsigned int EndingPoint, 
	unsigned int ClusterCount,
	unsigned int Heading,
	unsigned int PrintType
)
{
	int i;
	char Temp[100];
	int iX, iY;
	
	// For the alternate format, instead of putting everything on one line,
	// we put each data element on its own line, and have a double \n\n at the end
	// signaling the end of the data.
	// We also convert each distance given by the Hokuyo sensor into an X,Y pair
	// for easier plotting.
	double CurAngle = 0.0;
	double X, Y;

	if (PrintType == HOKUYO_PRINT_ASCII)
	{
		// Print the StartingPoint
		sprintf(Temp, "%04d\n", StartingPoint);
		uart0SendString((unsigned char *)Temp);

		// Print the EndingPoint
		sprintf(Temp, "%04d\n", EndingPoint);
		uart0SendString((unsigned char *)Temp);

		// Print the ClusterCount
		sprintf(Temp, "%04d\n", ClusterCount);
		uart0SendString((unsigned char *)Temp);

		// Print the status value
		sprintf(Temp, "%04d\n", Data[0]);
		uart0SendString((unsigned char *)Temp);

		// Print the current heading value
		sprintf(Temp, "%04d\n", Heading);
		uart0SendString((unsigned char *)Temp);
	}
	else
	{
		// Print the StartingPoint
		uart0SendChar((StartingPoint & 0xFF00) >> 8);
		uart0SendChar(StartingPoint & 0xFF);

		// Print the EndingPoint
		uart0SendChar((EndingPoint & 0xFF00) >> 8);
		uart0SendChar(EndingPoint & 0xFF);

		// Print the ClusterCount
		uart0SendChar((ClusterCount & 0xFF00) >> 8);
		uart0SendChar(ClusterCount & 0xFF);

		// Print the status value
		uart0SendChar(0x00);
		uart0SendChar(Data[0]);

		// Print the current heading value
		uart0SendChar((Heading & 0xFF00) >> 8);
		uart0SendChar(Heading & 0xFF);
	}
		
	if ((EndingPoint <= StartingPoint) || ClusterCount == 0)
	{
		return(-1);
	}

#ifdef HOKUYO_ALT_FORMAT
	X = 0.0;
	Y = 0.0;
	// Now print all of the data points - always print them all out
	// Note that GPScos() takes an integer number of degrees, and outputs
	// cos(degrees) * 1000
	for (i=1; i < HOKUYO_DATA_POINT_COUNT; i++)
	{
		if (Data[i] > 20)
		{
			X = Data[i] * (GPScos(CurAngle)/1000.0);
			Y = Data[i] * (GPSsin(CurAngle)/1000.0);
			if (PrintType == HOKUYO_PRINT_ASCII)
			{
				sprintf(Temp, "%7.2f,", X);
				uart0SendString((unsigned char *)Temp);
				sprintf(Temp, "%7.2f\n", Y);
				uart0SendString((unsigned char *)Temp);
			}
			else
			{
				// For BINARY mode, convert to integers and send 2 bytes for X and Y each
				iX = (int)X;
				iY = (int)Y;

				uart0SendChar((iX & 0xFF00) >> 8);
				uart0SendChar(iX & 0xFF);
				uart0SendChar((iY & 0xFF00) >> 8);
				uart0SendChar(iY & 0xFF);
			}
		}
		else
		{
			if (PrintType == HOKUYO_PRINT_ASCII)
			{
				sprintf(Temp, "0000.00,0000.00\n");
				uart0SendString((unsigned char *)Temp);
			}
			else
			{
				uart0SendChar(0x00);
				uart0SendChar(0x00);
				uart0SendChar(0x00);
				uart0SendChar(0x00);
			}
		}
		CurAngle += (360.0/1024.0);
	}

	if (PrintType == HOKUYO_PRINT_ASCII)
	{
		uart0SendString((unsigned char *)"\n\n");
	}
#else

	// Now print all of the data points - always print them all out
	for (i=1; i < HOKUYO_DATA_POINT_COUNT; i++)
	{
		sprintf(Temp, ",%04d", Data[i]);
		uart0SendString((unsigned char *)Temp);
	}

	uart0SendString((unsigned char *)"\n");
#endif
	return(0);
}


/* Perform the following steps:
1) Initalize UART1 at 19,200
2) Send a "V" command and get a valid response back from the Hokuyo
3) Send a "G" comamnd and ask for the whole sensor sweep
4) Process the data that comes back from the G command, and sent it out to the PC
    as a comma separated set of values, in text form.
*/
void processHokuyo(unsigned int PrintType)
{
#ifndef NO_HOKUYO_SENSOR
	int StartingPoint = 0;
	int EndingPoint = 768;
	int ClusterCount = 1;
	char TmpStr[40];
	// Temp buffer for strings coming back from sensor
	unsigned char Temp[HOKUYO_TEMP_BUF_SIZE];
#else
	int i;
#endif
	int Heading;
	
	// Stores value from the sensor data stream, then sent to PC.
	unsigned int Data[HOKUYO_DATA_POINT_COUNT];

	// Grab a compass heading first thing
	Heading = hmc6343_read();

	// Trap a compas error by just returning without printing anything
	if (Heading < 0 || Heading >= 360)
		return;

#ifndef NO_HOKUYO_SENSOR
	// Configure the serial port to the Hokuyo sensor
	init_uart1(19200);

	// The 'V' command establishes that we have a sensor on the serial port
	uart1SendString((unsigned char *)"V\n");
	// Check to see if the V command returned a valid result, or if we timed out.
	if (hokuyo_receive(Temp, 1000))
	{
		memset((unsigned char *)Data, 0x00, sizeof(Data));
		hokuyo_print(Data, 0, 0, 0, Heading, PrintType);
		return;
	}
	if (Temp[0] != 'V' || Temp[1] != CHAR_LF || Temp[2] != '0')
	{
		memset((unsigned char *)Data, 0x00, sizeof(Data));
		hokuyo_print(Data, 0, 0, 0, Heading, PrintType);
//printf("Failed to get valid 'V' response.\n");
		return;
	}

	// Clear our array that will hold the string from the sensor as well as our data array
	memset(Temp, 0x00, sizeof(Temp));
	memset((unsigned char *)Data, 0x00, sizeof(Data));

	// Generate and send the command to the sensor that tells it to send back the most recent
	// scan's worth of data. (The 'G' command.)
	sprintf(TmpStr, "G%03d%03d%02d\n", StartingPoint, EndingPoint, ClusterCount);
	uart1SendString((unsigned char *)TmpStr);
	// Check to see if the G command returned a valid result, or if we timed out.
	if (hokuyo_receive(Temp, 1000))
	{
		memset((unsigned char *)Data, 0x00, sizeof(Data));
		hokuyo_print(Data, 0, 0, 0, Heading, PrintType);
//printf("Failed to get valid 'G' response.\n");
		return;
	}

	// Convert the raw data into our data array of points, also correcting for compass heading
	hokuyo_process(Temp, Data, Heading);

	// Print the data point array back to the PC
	hokuyo_print(Data, StartingPoint, EndingPoint, ClusterCount, Heading, PrintType);
#else
	for (i=0; i < HOKUYO_DATA_POINT_COUNT; i++)
	{
		Data[i] = ((i * i) % 4000);
	}
	// Fill the data array with bogus values
	// Print the data point array back to the PC
	hokuyo_print(Data, 0, 768, 1, Heading, PrintType);
#endif
}

// Write a new deviation value to the compass EEPROM
// Must be either 4 digits or 4 digits with a preceeding negative sign
// All in ASCII
void HokuyoSetDeviation(void)
{
	signed int Offset = 0;
	unsigned char Byte = 0;
	unsigned int Negative = 0;
	unsigned char i2c_data[6];
	unsigned char addr = 0;
	
	Byte = getch();
	if (Byte == '-')
	{
		Negative = 1;
		Byte = getch();
	}
	
	if (Byte >= '0' && Byte <= '9')
	{
		Offset = (Byte - '0') * 1000;
	}
	
	Byte = getch();
	if (Byte >= '0' && Byte <= '9')
	{
		Offset = Offset + (Byte - '0') * 100;
	}

	Byte = getch();
	if (Byte >= '0' && Byte <= '9')
	{
		Offset = Offset + (Byte - '0') * 10;
	}
	
	Byte = getch();
	if (Byte >= '0' && Byte <= '9')
	{
		Offset = Offset + (Byte - '0');
	}

	if (Offset > 1800)
	{
		Offset = 1800;
	}
	
	if (Negative)
	{
		Offset = -Offset;
	}

#ifdef NO_HOKUYO_SENSOR
	return;
#endif
    
    // HMC6343
    addr = 0x19;

    // 0xF1 is the command code to write EEPROM, 0x0A is the LSB of the deviation angle
    i2c_data[0] = 0xF1;
	i2c_data[1] = 0x0A;
	i2c_data[2] = Offset & 0xFF;

    // Perform the write
    i2cwritex(addr, (unsigned char *)i2c_data, 3, SCCB_ON);

	delayMS(10);

    i2c_data[0] = 0xF1;
	i2c_data[1] = 0x0B;
	i2c_data[2] = (Offset >> 8) & 0xFF;

    // Perform the write
    i2cwritex(addr, (unsigned char *)i2c_data, 3, SCCB_ON);

    return;
}	
	
void HokuyoSetVariation(void)
{
	signed int Offset = 0;
	unsigned char Byte = 0;
	unsigned int Negative = 0;
	unsigned char i2c_data[6];
	unsigned char addr = 0;
	
	Byte = getch();
	if (Byte == '-')
	{
		Negative = 1;
		Byte = getch();
	}
	
	if (Byte >= '0' && Byte <= '9')
	{
		Offset = (Byte - '0') * 1000;
	}
	
	Byte = getch();
	if (Byte >= '0' && Byte <= '9')
	{
		Offset = Offset + (Byte - '0') * 100;
	}

	Byte = getch();
	if (Byte >= '0' && Byte <= '9')
	{
		Offset = Offset + (Byte - '0') * 10;
	}
	
	Byte = getch();
	if (Byte >= '0' && Byte <= '9')
	{
		Offset = Offset + (Byte - '0');
	}

	if (Offset > 1800)
	{
		Offset = 1800;
	}
	
	if (Negative)
    {
        Offset = -Offset;
    }

    // HMC6343
    addr = 0x19;

    // 0xF1 is the command code to write EEPROM, 0x0C is the LSB of the variation angle
    i2c_data[0] = 0xF1;
    i2c_data[1] = 0x0C;
    i2c_data[2] = Offset & 0xFF;

    // Perform the write
    i2cwritex(addr, (unsigned char *)i2c_data, 3, SCCB_ON);

    delayMS(10);

    i2c_data[0] = 0xF1;
    i2c_data[1] = 0x0D;
    i2c_data[2] = (Offset >> 8) & 0xFF;

    // Perform the write
    i2cwritex(addr, (unsigned char *)i2c_data, 3, SCCB_ON);

	return;
}
// Send the start calibration command to the compass
void HokuyoCalibrateStart(void)
{
    unsigned char i2c_data[6];
    unsigned char addr = 0;

#ifdef NO_HOKUYO_SENSOR
	return;
#endif
    
    // HMC6343
    addr = 0x19;

    // 0x50 is the command code to enter calibration mode
    i2c_data[0] = 0x71;

    // Perform the write - 1 byte
    i2cwritex(addr, (unsigned char *)i2c_data, 1, SCCB_OFF);

    return;
}

// Send the end calibration command to the compass
void HokuyoCalibrateEnd(void)
{
    unsigned char i2c_data[6];
    unsigned char addr = 0;

#ifdef NO_HOKUYO_SENSOR
	return;
#endif
    
    // HMC6343
    addr = 0x19;

    // 0x50 is the command code to exit calibration mode
    i2c_data[0] = 0x7E;

    // Perform the write - 1 byte
    i2cwritex(addr, (unsigned char *)i2c_data, 1, SCCB_OFF);

    return;
}


