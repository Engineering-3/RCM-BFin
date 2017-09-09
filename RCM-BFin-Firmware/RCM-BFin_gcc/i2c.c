/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  i2c.c - I2C routines for the blackfin
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
 
#include "config.h"
#include <cdefBF537.h>
#include "i2c.h"
#include "rcm-bfin.h"
#include "system.h"

#define PRESCALE120M 12         // factor = 12, 120MHz/10MHz
#define SSYNC asm("ssync;")

#if defined(TWI_100KHZ)
    // For 100KHz SCL speed: CLKDIV = (1/100KHz)/(1/10MHz) = 100
    // 2/3 high: CLKHI = 67, 1/3 low: CLKLOW = 33
    #define TWI_CLKDIV_VAL      (CLKLOW(67) | CLKHI(33))
//    #define TWI_CLKDIV_VAL      (CLKLOW(128) | CLKHI(128))
#else
    // For 400KHz SCL speed: CLKDIV = (1/400KHz)/(1/10MHz) = 25
    // 1/2 high: CLKHI = 13, 1/2 low: CLKLOW = 12
    #define TWI_CLKDIV_VAL      (CLKLOW(13) | CLKHI(2))
#endif


// Local variables
// Holds the number of times we will re-try an I2C message before giving up
/// TODO: make a function to modify this value
static int I2CMaxRetries = 10;

// Local function prototypes
static int WaitForIdleBus(void);
static int WaitForTX_FIFORoom(void);
static int WaitForXMTSERV(void);
static int WaitForMCOMP(void);

// Initialize the I2C hardware so we can use it
int InitI2C(void)
{
    *pTWI_CONTROL = 0x0000;                    // Turn off I2C if it was already on
    SSYNC;
    SSYNC;
    SSYNC;
    *pTWI_CONTROL = TWI_ENA | PRESCALE120M;    // PRESCALE = fsclk/10MHz
    *pTWI_CLKDIV = TWI_CLKDIV_VAL;             // Set clock div for 60% low and 30% high clock time
    *pTWI_MASTER_CTL = 0x0000;                 // Clear out master mode control register
    *pTWI_MASTER_ADDR = 0x0000;                // Clear out master address register
    *pTWI_FIFO_CTL |= XMTFLUSH | RCVFLUSH;     // Clear the TX and RX FIFO
    *pTWI_MASTER_STAT = BUFWRERR | BUFRDERR | DNAK | ANAK | LOSTARB; // Clear all status error
    *pTWI_FIFO_CTL = 0;                        // Clear the XMTFLUSH and RCVFLUSH bits manually (strange

    return(0);
}

// Wait until I2C bus is free, with timeout. If timeout, return -1 as error.
// Also check for any I2C errors, and return an error code if they're set
static int WaitForIdleBus(void)
{    
    int Ret = 0;
    
    // Delay so that the transaction can start, otherwise BUSBUSY might not be true yet
    delayUS(5);
    // This gets decremented every millisecond in our timer ISR
    I2CTimeoutMS = I2C_TIMEOUT_RELOAD;
    // Wait for completion
    while ((*pTWI_MASTER_STAT & (BUSBUSY | MPROG)) && I2CTimeoutMS)
    {
        SSYNC;
    }
    // Check for timeout
    if (I2CTimeoutMS == 0)
    {
        Ret = -1;
    }
    else
    {
        // Detect any errors
        if (*pTWI_MASTER_STAT & BUFWRERR)
        {
            Ret |= BUFWRERR;
        }
        if (*pTWI_MASTER_STAT & BUFRDERR)
        {
            Ret |= BUFRDERR;
        }
        if (*pTWI_MASTER_STAT & DNAK)
        {
            Ret |= DNAK;
        }
        if (*pTWI_MASTER_STAT & ANAK)
        {
            Ret |= ANAK;
        }
        if (*pTWI_MASTER_STAT & LOSTARB)
        {
            Ret |= LOSTARB;
        }
    }
    // Always clear errors
    *pTWI_MASTER_STAT = BUFWRERR | BUFRDERR | DNAK | ANAK | LOSTARB;
    // Always flush the TX FIFO
    *pTWI_FIFO_CTL |= XMTFLUSH;
    // And manually clear the XMTFLUSH bit
    *pTWI_FIFO_CTL = 0; 
    // Clear out the interrupt status bits (necessary?)
    *pTWI_INT_STAT = 0xFF;
        
    return (Ret);
}


// Spin on the XMTSERV bit while we wait for the address to be transmitted
// Return -1 if timed out, otherwise zero.
static int WaitForXMTSERV(void)
{
    int Ret = 0;
    
    I2CTimeoutMS = I2C_TIMEOUT_RELOAD;
    // wait to load the next sample into the TX FIFO
    while (!(*pTWI_INT_STAT & XMTSERV) && !(*pTWI_INT_STAT & MERR) && I2CTimeoutMS) 
    {
        SSYNC;
    }
    // Check for timeout
    if (I2CTimeoutMS == 0 || (*pTWI_INT_STAT & MERR))
    {
        Ret = -1;
    }
    else
    {
      // Write a 1 to XMTSERV to clear it
      *pTWI_INT_STAT = XMTSERV;
    }
    return (Ret);
}

// Spin on the MCOMP bit while we wait for the transaction to be finished
// Return -1 if timed out, otherwise zero.
static int WaitForMCOMP(void)
{
    int Ret = 0;
    
    I2CTimeoutMS = I2C_TIMEOUT_RELOAD;
    // wait to load the next sample into the TX FIFO
    while (!(*pTWI_INT_STAT & MCOMP) && I2CTimeoutMS) 
    {
        SSYNC;
    }
    // Check for timeout
    if (I2CTimeoutMS == 0)
    {
        Ret = -1;
    }
    return (Ret);
}


// Spin on the XMTSTAT bit to wait for room in our TX FIFO.
// Return -1 if timed out, otherwise zero.
static int WaitForTX_FIFORoom(void)
{
    int Ret = 0;
    
    I2CTimeoutMS = I2C_TIMEOUT_RELOAD;
    // wait to load the next sample into the TX FIFO
    while ((*pTWI_FIFO_STAT == XMTSTAT) && I2CTimeoutMS) 
    {
        SSYNC;
    }
    // Check for timeout
    if (I2CTimeoutMS == 0)
    {
        Ret = -1;
    }
    return (Ret);
}

// Transmit pairs of bytes out I2C. Smallest amount of data is 2 bytes (one pair)
// Return 0 on no error
// Return error code on timeout or other error
// NOTE: This routine sends 2 bytes of data at a time to address i2c_data. It then stop the transfer,
// and if pair_count is more than one, restarts the transfer for another pair of bytes. This is how the
// SCCB/camera need it. (i.e. not all one big long transmit stream)
int i2cwriteSingle(unsigned char i2c_device, unsigned char *i2c_data, unsigned int pair_count, int sccb_flag)
{
    int i;

    // Set/clear sccb flag
    if (sccb_flag)
        *pTWI_CONTROL |= SCCB;
    else
        *pTWI_CONTROL &= ~SCCB;

    // Target address - just 7-bit address, no read/write flag
    *pTWI_MASTER_ADDR = (i2c_device & 0x7F);

    
    // Loop across pair_count pairs of data
    for (i = 0; i < pair_count; i++) 
    { 
        // Load first byte of pair
        *pTWI_XMT_DATA8 = *i2c_data++;
        
        // Start transmission of address and 1st byte, tell I2C hardware we're sending 2 bytes of data
        *pTWI_MASTER_CTL = (2 << 6) | MEN; 
        if(WaitForTX_FIFORoom())
        {
            return (-1);
        }
        
        // Load second byte of data into FIFO
        *pTWI_XMT_DATA8 = *i2c_data++; 
        
        // Now wait until entire transfer is complete
        if (WaitForIdleBus())
        {
            return (-1);
        }
    }
    return (0);
}

// Wrapper routine for i2cwriteSingle(). Re-tries up to I2CMaxRetries times to send message out
int i2cwrite(unsigned char i2c_device, unsigned char *i2c_data, unsigned int pair_count, int sccb_flag)
{
    int i;

    // Try up to I2CMaxRetries to send the message out
    for (i = 0; i < I2CMaxRetries; i++) 
    { 
        if (i2cwriteSingle(i2c_device, i2c_data, pair_count, sccb_flag) == 0)
        {   
            // We were successful, so return PASS
            return (0);
        }
    }
    // We have failed, so return FAIL
    return (-1);
}

// write 'count' single bytes to i2c device
// Return 0 on no error
// Return error code on timeout or other error
int i2cwritexSingle(unsigned char i2c_device, unsigned char *i2c_data, unsigned int count, int sccb_flag)
{    
    int i;

    // Set/clear sccb flag
    if (sccb_flag)
        *pTWI_CONTROL |= SCCB;
    else
        *pTWI_CONTROL &= ~SCCB;
            
    // Target address - just 7-bit address, no read/write flag
    *pTWI_MASTER_ADDR = (i2c_device & 0x7F);
    
    // Loop across all data bytes to send out
    for (i = 0; i < count; i++) 
    {
        // Wait for space in the TX FIFO
        if(WaitForTX_FIFORoom())
        {
            return (-1);
        }

        // Load a byte to transmit into FIFO
        *pTWI_XMT_DATA8 = *i2c_data++;

        // If this is our first byte, tell I2C hardware to start sending
        if (i == 0)
        {
            *pTWI_MASTER_CTL = (count << 6) | MEN;
        }
    }
    return (WaitForIdleBus());
}

// Wrapper function for i2cread(). Tries up to I2CMaxRetries before giving up.
int i2cwritex(unsigned char i2c_device, unsigned char *i2c_data, unsigned int count, int sccb_flag)
{
    int i;

    // Try up to I2CMaxRetries to send the message out
    for (i = 0; i < I2CMaxRetries; i++) 
    { 
        if (i2cwritexSingle(i2c_device, i2c_data, count, sccb_flag) == 0)
        {   
            // We were successful, so return PASS
            return (0);
        }
    }
    // We have failed, so return FAIL
    return (-1);
}


// Read data_count number of bytes from I2C address i2c_device into i2c_data.
// First, send address and i2c_data[0] as register to read.
// Then read data_count bytes and put them in i2c_data starting with i2c_data[0].
// Return 0 on no error
// Return error code on timeout or other error
int i2creadSingle(unsigned char i2c_device, unsigned char *i2c_data, unsigned int data_count, int sccb_flag)
{
  int i, t;
  int retval = 0;

  // Set/clear sccb flag
  if (sccb_flag)
  {
    *pTWI_CONTROL |= SCCB;
  }
  else
  {
    *pTWI_CONTROL &= ~SCCB;
  }

   // Target address - just 7-bit address, no read/write flag
  *pTWI_MASTER_ADDR = (i2c_device & 0x7F);

  // Wait for space in the TX FIFO
  if(WaitForTX_FIFORoom())
  {
    retval = -1;
  }
  
  if (!retval)
  {
    // Put the register value into the TX FIFO
    *pTWI_XMT_DATA8 = *i2c_data;
    // Start master mode transmit, sending only 1 data byte (register value)
    *pTWI_MASTER_CTL = 0x40 | MEN;

    // Now wait until transfer is complete
    if (WaitForIdleBus())
    {
      retval = -2;
    }
  }
  
  if (!retval)
  {
    // Now start reading data_count bytes from the slave
    *pTWI_MASTER_CTL = (data_count << 6) | MEN | MDIR; 
    t = 0;
    for (i = 0; i < data_count; i++)
    {
      while (((*pTWI_FIFO_STAT & RCVSTAT) == 0) && (!retval))
      {
        SSYNC;
        delayUS(10);
        if (t++ >= 200)
        {
          retval = -1;
        }
      }
      if (!retval)
      {
        *i2c_data++ = *pTWI_RCV_DATA8; // Load the next sample into the TX FIFO.
        SSYNC;
      }
    }
  }
  if (!retval)
  {
    retval = WaitForIdleBus();
  }
  return (retval);
}

// Read data_count number of bytes from I2C address i2c_device into i2c_data, using repeated start.
// First, send address and i2c_data[0] as register to read.
// Then read data_count bytes and put them in i2c_data starting with i2c_data[0].
// Return 0 on no error
// Return error code on timeout or other error
int i2creadSinglers(unsigned char i2c_device, unsigned char *i2c_data, unsigned int data_count, int sccb_flag)
{
  int i, t;
  int retval = 0;

  // Set/clear sccb flag
  if (sccb_flag)
  {
    *pTWI_CONTROL |= SCCB;
  }
  else
  {
    *pTWI_CONTROL &= ~SCCB;
  }

  // Set FIFO control - set to generate RCVSERV and XTMSERV only when FIFO is completely empty
  *pTWI_FIFO_CTL = 0x000C0;
    
   // Target address - just 7-bit address, no read/write flag
  *pTWI_MASTER_ADDR = (i2c_device & 0x7F);

  // Wait for space in the TX FIFO
  if(WaitForTX_FIFORoom())
  {
    retval = -1;
  }

  if (!retval)
  {
    // Put the register value into the TX FIFO
    *pTWI_XMT_DATA8 = *i2c_data;
    // Start master mode transmit, sending only 1 data byte (register value)
    *pTWI_MASTER_CTL = 0x40 | MEN;

    // Now wait until XMTSERV bit is set, indicating that the address has been sent
    if (WaitForXMTSERV())
    {
      WaitForIdleBus();
      retval = -2;
    }
  }

  if (!retval)
  {
    // To cause repeated start condition, set RSTART bit and set MDIR bit to indicate a receive
    *pTWI_MASTER_CTL = RSTART | MDIR | MEN;
    
    // Now wait for MCOMP bit to be set, indicating that the write phase is complete
    if (WaitForMCOMP())
    {
      retval = -3;
    }
  }
 
  I2CTimeoutMS = I2C_TIMEOUT_RELOAD;

  if (!retval)
  {
    // Now start reading data_count bytes from the slave, clearing the RSTART bit in the process
    *pTWI_MASTER_CTL = (data_count << 6) | MEN | MDIR; 
    t = 0;
    for (i=0; i<data_count; i++)
    {
      while (((*pTWI_FIFO_STAT & RCVSTAT) == 0) && I2CTimeoutMS && (!retval))
      {
        SSYNC;
        delayUS(10);
        if (t++ >= 200)
        {
          retval = -4;
        }
      }
      if (I2CTimeoutMS == 0 && !retval)
      {
        WaitForIdleBus();
        retval = -5;
      }
      if (!retval)
      {
        *i2c_data++ = *pTWI_RCV_DATA8; // Read the received byte from the RX FIFO and store it
        SSYNC;
      }
    }
  }
  
  if (!retval)
  {
    retval = WaitForIdleBus();
  }
  
  return (retval);
}

// Wrapper function for i2cread(). Tries up to I2CMaxRetries before giving up.
int i2cread(unsigned char i2c_device, unsigned char *i2c_data, unsigned int data_count, int sccb_flag)
{
    int i;

    // Try up to I2CMaxRetries to send the message out
    for (i = 0; i < I2CMaxRetries; i++) 
    { 
        if (i2creadSingle(i2c_device, i2c_data, data_count, sccb_flag) == 0)
        {   
            // We were successful, so return PASS
            return (0);
        }
    }
    // We have failed, so return FAIL
    return (-1);
}


// Wrapper function for i2creadrs(). Tries up to I2CMaxRetries before giving up. Uses repeated start condition.
int i2creadrs(unsigned char i2c_device, unsigned char *i2c_data, unsigned int data_count, int sccb_flag)
{
    int i;

    // Try up to I2CMaxRetries to send the message out
    for (i = 0; i < I2CMaxRetries; i++) 
    { 
        if (i2creadSinglers(i2c_device, i2c_data, data_count, sccb_flag) == 0)
        {   
            // We were successful, so return PASS
            return (0);
        }
    }
    // We have failed, so return FAIL
    return (-1);
}

// Return 0 on no error
// Return error code on timeout or other error
/// TODO: Not fully cleaned up yet
int i2cslowread(unsigned char i2c_device, unsigned char *i2c_data, unsigned int data_count, int sccb_flag)
{
    int i;

    // Set/clear sccb flag
    if (sccb_flag)
        *pTWI_CONTROL |= SCCB;
    else
        *pTWI_CONTROL &= ~SCCB;

     // Target address - just 7-bit address, no read/write flag
    *pTWI_MASTER_ADDR = (i2c_device & 0x7F);

    while ((*pTWI_FIFO_STAT & XMTSTAT) == XMT_FULL)
        SSYNC;
    *pTWI_XMT_DATA8 = *i2c_data; // send the start register 
    *pTWI_MASTER_CTL = 0x40 | MEN;     
    I2CTimeoutMS = I2C_TIMEOUT_RELOAD;
    while (((*pTWI_INT_STAT & MCOMP) == 0) && I2CTimeoutMS) { // Wait until transmission complete and MCOMP is set
        SSYNC;        
    }
    *pTWI_INT_STAT = XMTSERV | MCOMP; // service TWI for next transmission

    SSYNC;
    *pTWI_MASTER_CTL = (data_count << 6) | MEN | MDIR; 
    for (i=0; i<data_count; i++) {
        I2CTimeoutMS = I2C_TIMEOUT_RELOAD;
        while (((*pTWI_FIFO_STAT & RCVSTAT) == 0) && I2CTimeoutMS) {
            SSYNC;
        }
        *i2c_data++ = *pTWI_RCV_DATA8; // Load the next sample into the TX FIFO. 
        SSYNC;
    }
    return (WaitForIdleBus());
}

// Just like i2cread except you get to specify the time between the write and read sections
// Just like i2cread except you get to specify the time between the write and read sections
// Return 0 on no error
// Return error code on timeout or other error
/// TODO: Not fully cleaned up yet

/// TODO: THis is only called from hokuyo.c - change to normal i2cread??

int i2c_read_timed(unsigned char i2c_device, unsigned char *i2c_data, unsigned int data_count, int sccb_flag, int delay_us)
{
    int i, t;

    // Set/clear sccb flag
    if (sccb_flag)
        *pTWI_CONTROL |= SCCB;
    else
        *pTWI_CONTROL &= ~SCCB;

     // Target address - just 7-bit address, no read/write flag
    *pTWI_MASTER_ADDR = (i2c_device & 0x7F);

    t = 0;
    while ((*pTWI_FIFO_STAT & XMTSTAT) == XMT_FULL)
    {
        SSYNC;
        delayUS(10);
        if (t++ >= 200)
            return(-1);
    }
    *pTWI_XMT_DATA8 = *i2c_data; // send the start register 
    *pTWI_MASTER_CTL = 0x40 | MEN;     

    t = 0;
    while ((*pTWI_INT_STAT & MCOMP) == 0) // Wait until transmission complete and MCOMP is set
    {
        SSYNC;
        delayUS(10);
        if (t++ >= 200)
            return(-2);
    }

    // delay the proper time  between transmit and receive
    delayUS(delay_us);

    *pTWI_INT_STAT = XMTSERV | MCOMP; // service TWI for next transmission

    SSYNC;
    *pTWI_MASTER_CTL = (data_count << 6) | MEN | MDIR; 
    t = 0;
    for (i=0; i<data_count; i++) {
        while ((*pTWI_FIFO_STAT & RCVSTAT) == 0)
        {
            SSYNC;
            delayUS(10);
            if (t++ >= 200)
                return(-3);
        }
        *i2c_data++ = *pTWI_RCV_DATA8; // Load the next sample into the TX FIFO. 
        SSYNC;
    }
       return (WaitForIdleBus());
}

// Performs read, with separate data buffers and length for the write and read portions of the transaction.
// Return 0 on no error
// Return error code on timeout or other error

/// TODO: THis is only called from hokuyo.c - change to normal i2cread??

int i2c_read_timed2(
    unsigned char i2c_device, 
    unsigned char *i2c_data_out, 
    unsigned int data_count_out, 
    unsigned char *i2c_data_in, 
    unsigned int data_count_in, 
    int sccb_flag, int delay_us
)
{
    int i, t;

    // Set/clear sccb flag
    if (sccb_flag)
        *pTWI_CONTROL |= SCCB;
    else
        *pTWI_CONTROL &= ~SCCB;

     // Target address - just 7-bit address, no read/write flag
    *pTWI_MASTER_ADDR = (i2c_device & 0x7F);

    t = 0;
    while ((*pTWI_FIFO_STAT & XMTSTAT) == XMT_FULL)
    {
        SSYNC;
        delayUS(10);
        if (t++ >= 200)
            return(-1);
    }

    SSYNC;
    t = 0;
    for (i = 0; i < data_count_out; i++) { // # of configurations to send to the sensor
/// TODO: REPLACE        WAIT_FOR_XMIT();
        *pTWI_XMT_DATA8 = *i2c_data_out++; // Load a value 
        if (i == 0)
            *pTWI_MASTER_CTL = (data_count_out << 6) | MEN; // Start transmission of n bytes
        SSYNC;
    }
    t = 0;
    while ((*pTWI_MASTER_STAT & MPROG) == 0) { // Wait for completion
        delayUS(10);
        if (t++ >= 200)
            return(-2);
        SSYNC;        
    }

    // delay the proper time  between transmit and receive
    delayUS(delay_us);

    *pTWI_INT_STAT = XMTSERV | MCOMP; // service TWI for next transmission

    SSYNC;
    *pTWI_MASTER_CTL = (data_count_in << 6) | MEN | MDIR; 
    t = 0;
    for (i=0; i<data_count_in; i++) {
        while ((*pTWI_FIFO_STAT & RCVSTAT) == 0)
        {
            SSYNC;
            delayUS(10);
            if (t++ >= 200)
                return(-3);
        }
        *i2c_data_in++ = *pTWI_RCV_DATA8; // Load the next sample into the TX FIFO. 
        SSYNC;
    }
    return (WaitForIdleBus());
}

