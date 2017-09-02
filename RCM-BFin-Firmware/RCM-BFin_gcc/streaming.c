/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  streaming.c -
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
 * Implements sensor streaming (of all types) from RCM-Bfin back to PC
 *
 * 
 *
 */

#include "streaming.h"
#include "stdio.h"
#include "intuart.h"
#include "i2c.h"
#include "rcm-bfin.h"
#include "system.h"
#include "options.h"
#include <stdint.h>

/* Structure to hold digital sensor streaming info */
typedef struct 
{
    uint32_t I2CAddress;
    uint32_t I2CRegister;
    uint32_t Bit;
    uint32_t Rate;                // if zero, this stream is off
    uint32_t RateCounter;
    I2CRead_Type Type;            // I2C_NORMAL or I2C_REPEATED_START
} StreamingDigitalBitType;

typedef struct 
{
    uint32_t I2CAddress;
    uint32_t I2CRegister;
    uint32_t Bytes;
    uint32_t Rate;                // if zero, this stream is off
    uint32_t RateCounter;
    I2CRead_Type Type;            // I2C_NORMAL or I2C_REPEATED_START
} StreamingDigitalByteType;

/* Structure to hold analog sensor streaming into */
typedef struct 
{
    uint32_t AnalogChannel;
    uint32_t Rate;                // if zero, this stream is off
    uint32_t RateCounter;
} StreamingAnalogType;

/* Structure to hold information on streaming a PicoC shared variable */
typedef struct
{
    uint32_t Type;                  // Character 'F', 'S', or 'I'
    uint32_t StartIndex;            // Starting variable index
    uint32_t EndIndex;              // Ending variable index
    uint32_t Rate;                  // if zero, this stream is off
    uint32_t RateCounter;
} StreamingPicoCVariableType;
 
/* Main arrays holding structures */
static volatile StreamingPicoCVariableType   PicoCVars[STREAMING_MAX_PICOCVARS];
static volatile StreamingDigitalBitType      DigitalBits[STREAMING_MAX_DIGITAL_BITS];
static volatile StreamingDigitalByteType     DigitalBytes[STREAMING_MAX_DIGITAL_BYTES];
static volatile StreamingAnalogType          Analogs[STREAMING_MAX_ANALOGS];
 
volatile bool StreamingTickTrigger = false;
volatile uint32_t StreamingLastTickMS = 0;

/*
 * Handle the NA command.
 * Format is NActt
 * where
 *   c is a one byte binary analog channel
 *   tt is a two byte binary (LSB first) number, indicating the time
 *       in milliseconds between sensor streaming packet sends
 * This command will set up a read from analog channel c
 * every 'tt' milliseconds. Two bytes will be read
 * and returned in the sensor packet, and will always be a 12 bit result.
 */
void StreamingParseAnalogBit(void)
{
    uint32_t Channel, i, Rate;
    
    Channel = getch();
    Rate = getch() + (getch() * 256);
    
    // Now search to see if we've used this set of values before
    for (i = 0; i < STREAMING_MAX_ANALOGS; i++)
    {
        if (Channel == Analogs[i].AnalogChannel)
        {
            // All we need to do is update the rate
            // If Rate is zero, this will effectively shut this channel off
            Analogs[i].Rate = Rate;
            Analogs[i].RateCounter = 0;    // Force a reload
            // Now we're done
            printf("#NRA");
            return;
        }
    }

    // Didn't find it, so this one must be new.
    for (i = 0; i < STREAMING_MAX_ANALOGS; i++)
    {
        if (
            Analogs[i].Rate == 0
        )
        {
            // Copy all of our information over to the structure
            Analogs[i].AnalogChannel = Channel;
            Analogs[i].Rate = Rate;
            Analogs[i].RateCounter = 0;    // Force a reload
            // Now we're done
            printf("#NRA");
            return;
        }
    }
    
    // This is an error - we've used up all of the available slots
    // So fail silently
    printf("#NRA");
}

/*
 * Handle the ND command - streams a single bit
 * Format is NDarbtt (or Ndarbtt for repeated start)
 * where
 *   a is a one byte binary I2C address to read
 *   r is a one byte binary I2C register to read
 *   b is a one byte binary number from 0 to 7 indicating the bit to be read
 *   tt is a two byte binary (LSB first) number, indicating the time
 *       in milliseconds between sensor streaming packet sends
 *
 * If tt = 0 then streaming from this I2C address and register is turned off
 * Returns "#NRD" or "#NRd" for repeated-start when the ND or Nd command is received
 * And starts (or stops) #ND responses at the desired streaming interval
 */
void StreamingParseDigitalBit(I2CRead_Type I2CReadMethod)
{
    uint32_t Address, Register, Bit, Rate, i;
    
    Address = getch();
    Register = getch();
    Bit = getch();
    Rate = getch() + (getch() * 256);
    
    // Now search to see if we've used this set of values before
    for (i = 0; i < STREAMING_MAX_DIGITAL_BITS; i++)
    {
        if (
            Address == DigitalBits[i].I2CAddress
            &&
            Register == DigitalBits[i].I2CRegister
            &&
            Bit == DigitalBits[i].Bit
        )
        {
            // All we need to do is update the rate
            // If Rate is zero, this will effectively shut this channel off
            DigitalBits[i].Rate = Rate;
            DigitalBits[i].RateCounter = 0;    // Force a reload
            DigitalBits[i].Type = I2CReadMethod; // Always update the type
            // Now we're done
            if (I2CReadMethod == I2C_NORMAL)
            {
              printf("#NRD");
            }
            else
            {
              printf("#NRd");
            }
            return;
        }
    }

    // Didn't find it, so this one must be new.
    for (i = 0; i < STREAMING_MAX_DIGITAL_BITS; i++)
    {
        if (
            DigitalBits[i].Rate == 0
        )
        {
            // Copy all of our information over to the structure
            DigitalBits[i].I2CAddress = Address;
            DigitalBits[i].I2CRegister = Register;
            DigitalBits[i].Bit = Bit;
            DigitalBits[i].Rate = Rate;
            DigitalBits[i].RateCounter = 0;    // Force a reload
            DigitalBits[i].Type = I2CReadMethod; // Always update the type
            // Now we're done
            if (I2CReadMethod == I2C_NORMAL)
            {
              printf("#NRD");
            }
            else
            {
              printf("#NRd");
            }
            return;
        }
    }
    
    // This is an error - we've used up all of the available slots
    // So fail silently
    if (I2CReadMethod == I2C_NORMAL)
    {
      printf("#NRD");
    }
    else
    {
      printf("#NRd");
    }
}

/*
 * Handle the NB command - streams a byte or more than one byte from an I2C address and register
 * Format is NBarbtt (or Nbarbtt for repeated-start)
 * where
 *   a is a one byte binary I2C address to read
 *   r is a one byte binary I2C register to read
 *   b is a one byte binary number indicating the number of bytes to read
 *   tt is a two byte binary (LSB first) number, indicating the time
 *       in milliseconds between sensor streaming packet sends
 *
 * If tt = 0 then streaming from this I2C address and register is turned off
 * Returns "#NRB" (or "#NRb" for repeated-start) when the NB or Nb command is received
 * And starts (or stops) #NB responses at the desired streaming interval
 */
void StreamingParseDigitalByte(I2CRead_Type I2CReadMethod)
{
    uint32_t Address, Register, Bytes, Rate, i;
    
    Address = getch();
    Register = getch();
    Bytes = getch();
    // Always have at least 1 byte to read
    if (Bytes == 0)
    { 
        Bytes = 1;
    }
    Rate = getch() + (getch() * 256);
    
    // Now search to see if we've used this set of values before
    for (i = 0; i < STREAMING_MAX_DIGITAL_BYTES; i++)
    {
        if (
            Address == DigitalBytes[i].I2CAddress
            &&
            Register == DigitalBytes[i].I2CRegister
            &&
            Bytes == DigitalBytes[i].Bytes
        )
        {
            // All we need to do is update the rate
            // If Rate is zero, this will effectively shut this channel off
            DigitalBytes[i].Rate = Rate;
            DigitalBytes[i].RateCounter = 0;    // Force a reload
            DigitalBytes[i].Type = I2CReadMethod; // Always update the I2C read type
            // Now we're done
            if (I2CReadMethod == I2C_NORMAL)
            {
              printf("#NRB");
            }
            else
            {
              printf("#NRb");
            }
            return;
        }
    }

    // Didn't find it, so this one must be new.
    for (i = 0; i < STREAMING_MAX_DIGITAL_BYTES; i++)
    {
        if (
            DigitalBytes[i].Rate == 0
        )
        {
            // Copy all of our information over to the structure
            DigitalBytes[i].I2CAddress = Address;
            DigitalBytes[i].I2CRegister = Register;
            DigitalBytes[i].Bytes = Bytes;
            DigitalBytes[i].Rate = Rate;
            DigitalBytes[i].RateCounter = 0;    // Force a reload
            DigitalBytes[i].Type = I2CReadMethod; // Always update the I2C read type
            // Now we're done
            if (I2CReadMethod == I2C_NORMAL)
            {
              printf("#NRB");
            }
            else
            {
              printf("#NRb");
            }
            return;
        }
    }
    
    // This is an error - we've used up all of the available slots
    // So fail silently
    if (I2CReadMethod == I2C_NORMAL)
    {
      printf("#NRB");
    }
    else
    {
      printf("#NRb");
    }
}

/*
 * Handle the NP command - streams values from PicoC shared memory to PC
 * Format is NP,a,tt,b,c<CR>
 * where
 *   a is one byte type:'F' (for float), 'I' (for integer) or 'S' (for string)
 *   tt is a two byte ASCII number, from 0 to 65535, indicating the time
 *       in milliseconds between sensor streaming packet sends
 *   b is a one to three byte ASCII start value from 0 to 255 indicating which 
 *      variable to start reading from (index)
 *   c is a one to three byte ASCII end value from 0 to 255 indicating which 
 *      variable to end with (index) - this parameter is optional, and if not 
 *      present, will just use 'b' as the only variable streamed.
 *
 * Note that the commas are required between parameters, and that <CR> terminates
 * the command, and that the 'c' parameter is optional. 
 *
 * For example, this command
 *  "NP,F,2000,96<CR>"
 *  Would ask the RCM-Bfin to generate a streaming packet every 2 seconds consisting
 *      of the float variable 96
 *
 * If tt = 0 then streaming from this combination of type, start and end is 
 *      turned off
 * Returns "#NRP" when the NP command is received
 *
 * Streaming replies:
 * The packet sent every tt milliseconds will consist of the following:
 *   Floats
 *      "##NP,F,b,f1,f2,...<CR><LF>"
 *      Where 
 *          'b' is the starting index of v1
 *          'f1' is the first floating point value
 *          'f2' is the second floating point value (if present)
 *          'f3' through 'fn' are the rest of the floating point values (if present)
 *      So for a command "NPF,2000,96,101<CR>" a streaming response packet might be:
 *      "##NP,F,96,2.15,-69291.2131,.6,10.0,.000012,99.99<CR><LF>"
 *
 *   Integers
 *      "##NP,I,b,i1,i2,...<CR><LF>"
 *      Where 
 *          'b' is the starting index of v1
 *          'i1' is the first integer value
 *          'i2' is the second integer value (if present)
 *          'i3' through 'in' are the rest of the integer values (if present)
 *      So for a command "NPI,250,45,50<CR>" a streaming response packet might be:
 *      "##NP,I,45,5,-241,56789,-1,5544<CR><LF>"
 *
 *   Strings
 *      "##NP,S,b,s1,s2,...<CR><LF>"
 *      Where 
 *          'b' is the starting index of s1
 *          's1' is the first string
 *          's2' is the second string (if present)
 *          's3' through 'sn' are the rest of the strings (if present)
 *      So for a command "NPS,1000,15,16<CR>" a streaming response packet might be:
 *      "##NP,S,15,abc how are you? Fine!,Robot Has Died. Sensor 4 = 55.<CR><LF>"
 *      The main restriction  here is that you may not use commas or <LF> or <CR> in your
 *      strings or the PC parser will get confused.
 *
 */
void StreamingParsePicoC(void)
{
    uint32_t i;
    uint StartIndex = 0, EndIndex = 0, Rate = 0;
    uint8_t Type;
    char InStr[100];

    // For this command, we need to get at least 3, and up to 4 pieces of information.
    // First, we're going to read until we see a <CR> or <LF> so that we have the whole
    // input string at once.
    i = 0;
    
    while (i < 99)
    {
        InStr[i] = getch();
        if (InStr[i] == '\n' || InStr[i] == '\r')
        {
            i = 100;
        }
        else
        {
            i++;
        }
    }
    InStr[i] = 0x00;
    
    // Now pull out the parameters within the command one by one
    i = sscanf(InStr, ",%c,%6u,%3u,%3u", &Type, &Rate, &StartIndex, &EndIndex);
    
    // If the user did not send enough parameters, then bail out
    if (i < 3)
    {
        PrintUnknownCommand('=');
        return;
    }
    
    // If they did not have an EndIndex, then set it to StartIndex as we'll only stream one variable.
    if (i != 4)
    {
        EndIndex = StartIndex;
    }
    
    // Don't let end be less than start
    if (EndIndex < StartIndex)
    {
        EndIndex = StartIndex;
    }

    // Now search to see if we've used this set of values before
    for (i = 0; i < STREAMING_MAX_PICOCVARS; i++)
    {
        if (
            Type == PicoCVars[i].Type
            &&
            StartIndex == PicoCVars[i].StartIndex
            &&
            EndIndex == PicoCVars[i].EndIndex
        )
        {
            // All we need to do is update the rate
            // If Rate is zero, this will effectively shut this channel off
            PicoCVars[i].Rate = Rate;
            PicoCVars[i].RateCounter = 0;    // Force a reload
            // Now we're done
            printf("#NRP");
            return;
        }
    }

    // Didn't find it, so this one must be new.
    for (i = 0; i < STREAMING_MAX_DIGITAL_BYTES; i++)
    {
        if (
            PicoCVars[i].Rate == 0
        )
        {
            // Copy all of our information over to the structure
            PicoCVars[i].Type = Type;
            PicoCVars[i].StartIndex = StartIndex;
            PicoCVars[i].EndIndex = EndIndex;
            PicoCVars[i].Rate = Rate;
            PicoCVars[i].RateCounter = 0;    // Force a reload
            // Now we're done
            printf("#NRP");
            return;
        }
    }

    // This is an error - we've used up all of the available slots
    // So fail silently
    printf("#NRP");
}


void StreamingParseHokuyo(void)
{

    printf("#NH");
}

/* Handle the streaming shutdown command. Simply
read in the user's time value, in seconds, and
update the global that gets watched in the ISR. */
void StreamingShutdown(void)
{
    int timeout = 0;
    uint8_t Buffer[4];
    
    Buffer[0] = getch();
    Buffer[1] = getch();
    Buffer[2] = getch();
    Buffer[3] = 0x00;
    
    timeout = atoi((char*)Buffer);
    if (timeout > 255) 
    {
        timeout = 255;
    }
    
    StreamShutdownTimeout = timeout;
    
    printf("#$OK");
}


/*
 * Call this each time through the main loop.
 * It will check to see if there are any sensors that need to be
 * read and packets sent back, and if so, will take care of sending
 * them.
 *
 */
uint32_t StreamingProcess(void)
{
    uint32_t i,j, Result;
    uint8_t i2c_data[256];

    // Every 1ms (when StreamingTickTrigger fires), decrement all streaming variable's counters
    if (StreamingTickTrigger)
    {
        StreamingTickTrigger = false;
        StreamingTick();

        // Handle all of the digital bits
        for (i = 0; i < STREAMING_MAX_DIGITAL_BITS; i++)
        {
            // Is this one even turned on?
            if (DigitalBits[i].Rate != 0)
            {
                // Is this one ready to send?
                if (DigitalBits[i].RateCounter == 0)
                {
                    // Reload the time
                    DigitalBits[i].RateCounter = DigitalBits[i].Rate;
                    
                    // Set up the register to read from
                    i2c_data[0] = DigitalBits[i].I2CRegister;
                    // Perform the sensor read
                    if (DigitalBits[i].Type == I2C_NORMAL)
                    {
                      i2cread((uint8_t)DigitalBits[i].I2CAddress, (uint8_t *)i2c_data, 1, SCCB_ON);
                    }
                    else
                    {
                      i2creadrs((uint8_t)DigitalBits[i].I2CAddress, (uint8_t *)i2c_data, 1, SCCB_ON);
                    }
                    /// TODO: put in check for failed I2C call
                    if (i2c_data[0] & (1 << DigitalBits[i].Bit))
                    {
                        Result = 1;
                    }
                    else
                    {
                        Result = 0;
                    }
                    
                    // And send out a packet
                    PacketBegin();
                    if (DigitalBits[i].Type == I2C_NORMAL)
                    {
                      printf("#ND%c%c%c%c",
                          (uint8_t)DigitalBits[i].I2CAddress,
                          (uint8_t)DigitalBits[i].I2CRegister,
                          (uint8_t)DigitalBits[i].Bit,
                          (uint8_t)Result
                      );
                    }
                    else
                    {
                      printf("#Nd%c%c%c%c",
                          (uint8_t)DigitalBits[i].I2CAddress,
                          (uint8_t)DigitalBits[i].I2CRegister,
                          (uint8_t)DigitalBits[i].Bit,
                          (uint8_t)Result
                      );
                    }
                    PacketEnd(true);
                }
            }
        }

        // Handle all of the digital bytes
        for (i = 0; i < STREAMING_MAX_DIGITAL_BYTES; i++)
        {
            // Is this one even turned on?
            if (DigitalBytes[i].Rate != 0)
            {
                // Is this one ready to send?
                if (DigitalBytes[i].RateCounter == 0)
                {
                    // Reload the time
                    DigitalBytes[i].RateCounter = DigitalBytes[i].Rate;

                    // Set up the register to read from
                    i2c_data[0] = DigitalBytes[i].I2CRegister;
                    // Perform the sensor read
                    if (DigitalBytes[i].Type == I2C_NORMAL)
                    {
                      i2cread((uint8_t)DigitalBytes[i].I2CAddress, (uint8_t *)i2c_data, DigitalBytes[i].Bytes, SCCB_ON);
                    }
                    else
                    {
                      i2creadrs((uint8_t)DigitalBytes[i].I2CAddress, (uint8_t *)i2c_data, DigitalBytes[i].Bytes, SCCB_ON);
                    }
                    /// TODO: put in check for failed I2C call
                    
                    // And send out a packet
                    PacketBegin();
                    if (DigitalBytes[i].Type == I2C_NORMAL)
                    {
                      printf("#NB%c%c",
                          (uint8_t)DigitalBytes[i].I2CAddress,
                          (uint8_t)DigitalBytes[i].I2CRegister
                      );
                    }
                    else
                    {
                      printf("#Nb%c%c",
                          (uint8_t)DigitalBytes[i].I2CAddress,
                          (uint8_t)DigitalBytes[i].I2CRegister
                      );
                    }
                    for (j=0; j < DigitalBytes[i].Bytes; j++)
                    {
                        printf("%c", i2c_data[j]);
                    }
                    PacketEnd(true);
                }
            }
        }

        // And all of the analogs
        for (i = 0; i < STREAMING_MAX_ANALOGS; i++)
        {
            // Is this one even turned on?
            if (Analogs[i].Rate != 0)
            {
                // Is this one ready to send?
                if (Analogs[i].RateCounter == 0)
                {
                    // Reload the time
                    Analogs[i].RateCounter = Analogs[i].Rate;
                                    
                    // Perform the 16 bit sensor read
                    Result = analog(Analogs[i].AnalogChannel);
                
                    // And send out a packet
                    PacketBegin();
                    printf("#NA%c%c%c",
                        (uint8_t)Analogs[i].AnalogChannel,
                        (uint8_t)(Result),
                        (uint8_t)(Result >> 8)
                    );
                    PacketEnd(true);
                    
                }
            }
        }
        
        // And all of the PicoC Shared Variables
        for (i = 0; i < STREAMING_MAX_PICOCVARS; i++)
        {
            // Is this one even turned on?
            if (PicoCVars[i].Rate != 0)
            {
                // Is this one ready to send?
                if (PicoCVars[i].RateCounter == 0)
                {
                    // Reload the time
                    PicoCVars[i].RateCounter = PicoCVars[i].Rate;
                                    
                    // And send out a packet
                    PacketBegin();
                    
                    if (PicoCVars[i].Type == 'F')
                    {
                        printf("##NP,%c,%u",
                            (uint8_t)PicoCVars[i].Type,
                            (uint8_t)PicoCVars[i].StartIndex
                        );
                        for (j = PicoCVars[i].StartIndex; j <= PicoCVars[i].EndIndex; j++)
                        {
                            printf(",%f", (double)pico_float[j]);
                        }
                        printf("\r\n");
                    }
                    else if (PicoCVars[i].Type == 'S')
                    {
                        printf("##NP,%c,%u",
                            (uint8_t)PicoCVars[i].Type,
                            (uint8_t)PicoCVars[i].StartIndex
                        );
                        for (j = PicoCVars[i].StartIndex; j <= PicoCVars[i].EndIndex; j++)
                        {
                            printf(",%s", pico_char[j]);
                        }
                        printf("\r\n");
                    }
                    else if (PicoCVars[i].Type == 'I')
                    {
                        printf("##NP,%c,%u",
                            (uint8_t)PicoCVars[i].Type,
                            (uint8_t)PicoCVars[i].StartIndex
                        );
                        for (j = PicoCVars[i].StartIndex; j <= PicoCVars[i].EndIndex; j++)
                        {
                            printf(",%d", (int)pico_int[j]);
                        }
                        printf("\r\n");
                    }
                    PacketEnd(true);
                }
            }
        }
    }
    
    return 0;
}

/*
 * Turn off all streaming and flush all output buffers.
 */
void StopAllStreaming(void)
{
    uint32_t i;

    // Shut everybody down
    for (i = 0; i < STREAMING_MAX_DIGITAL_BITS; i++)
    {
        DigitalBits[i].Rate = 0;
    }
    for (i = 0; i < STREAMING_MAX_DIGITAL_BYTES; i++)
    {
        DigitalBytes[i].Rate = 0;
    }
    for (i = 0; i < STREAMING_MAX_ANALOGS; i++)
    {
        Analogs[i].Rate = 0;
    }
    for (i = 0; i < STREAMING_MAX_PICOCVARS; i++)
    {
        PicoCVars[i].Rate = 0;
    }
    SetOption(OPT_STREAM_VIDEO_ON, false);      // Video streaming off
    FlushAllTXBuffers();                        // Clear out any pending bytes
}

/*
 * This function gets called approximately every 1ms from main loop
 * and performs our timing for each sensor
 */
void StreamingTick(void)
{
    int i;
    
    // Check for timeout
    /// TODO: Optimize this by only doing it once, then setting a flag. The next time a byte comes in,
    /// trip the flag back. You don't want to be running through all of these every tick.
    if ((UART0LastByteRX > (StreamShutdownTimeout * 1000)) && (StreamShutdownTimeout != 0))
    {
        StopAllStreaming();
    }
    /// TODO: Optimize by only checking streaming 'channels' that are actually being used - have MaxUsed for each type
    
    // Handle all of the digital bits
    for (i = 0; i < STREAMING_MAX_DIGITAL_BITS; i++)
    {
        if (DigitalBits[i].RateCounter > StreamingLastTickMS)
        {
            DigitalBits[i].RateCounter -= StreamingLastTickMS;
        }
        else
        {
            DigitalBits[i].RateCounter = 0;
        }
    }

    // Handle all of the digital bytes
    for (i = 0; i < STREAMING_MAX_DIGITAL_BYTES; i++)
    {
        if (DigitalBytes[i].RateCounter > StreamingLastTickMS)
        {
            DigitalBytes[i].RateCounter -= StreamingLastTickMS;
        }
        else
        {
            DigitalBytes[i].RateCounter = 0;
        }
    }

    // And all of the analogs
    for (i = 0; i < STREAMING_MAX_ANALOGS; i++)
    {
        if (Analogs[i].RateCounter > StreamingLastTickMS)
        {
            Analogs[i].RateCounter -= StreamingLastTickMS;
        }
        else
        {
            Analogs[i].RateCounter = 0;
        }
    }

    // And all of the PicoC shared variables
    for (i = 0; i < STREAMING_MAX_PICOCVARS; i++)
    {
        if (PicoCVars[i].RateCounter > StreamingLastTickMS)
        {
            PicoCVars[i].RateCounter -= StreamingLastTickMS;
        }
        else
        {
            PicoCVars[i].RateCounter = 0;
        }
    }

    // Clear our "milliseconds since last call" timer
    StreamingLastTickMS = 0;
}
