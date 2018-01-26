/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  intuart.c - interrupt driven UART routines for both hardware UARTs
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
#include <stdio.h>
#include <errno.h>
#include <cdefBF537.h>
#include <sys/stat.h>
#include <gcc.h>
#include <stdint.h>
#include <stdbool.h>
#include "system.h"
#include "intuart.h"
#include "config.h"
#include "rcm-bfin.h"
#include "options.h"
#include "edit.h"
#include "streaming.h"
#include "xmodem.h"

// Define this if you want to cause every 100th packet to become corrupt
//#define DEBUG_CORRUPT_PACKET_TEST   1

//-----------------------------------------
// Buffers for interrupt-enabled receive
//
volatile uint32_t RXBufIn;
volatile uint32_t RXBufOut;
volatile uint32_t RXBufCount;

volatile uint32_t RXPriorityBufIn;
volatile uint32_t RXPriorityBufOut;
volatile uint32_t RXPriorityBufCount;

volatile uint32_t RXAltBufIn;
volatile uint32_t RXAltBufOut;
volatile uint32_t RXAltBufCount;

volatile uint32_t TXBufIn;
volatile uint32_t TXBufOut;
volatile uint32_t TXBufCount;

volatile uint32_t TXPriorityBufIn;
volatile uint32_t TXPriorityBufOut;
volatile uint32_t TXPriorityBufCount;

volatile uint32_t TXAltBufIn;
volatile uint32_t TXAltBufOut;
volatile uint32_t TXAltBufCount;

volatile char processing_priority_command;
volatile uint32_t UseHardwareHandshaking = false;


//-----------------------------------------
// Buffers for interrupt-enabled receive
//
volatile uint8_t RXBuf[RX_BUF_SIZE];
volatile uint8_t RXPriorityBuf[RX_PRIORITY_BUF_SIZE];
volatile uint8_t RXAltBuf[RX_ALT_BUF_SIZE];
volatile uint8_t TXBuf[TX_BUF_SIZE];
volatile uint8_t TXPriorityBuf[TX_PRIORITY_BUF_SIZE];
volatile uint8_t TXAltBuf[TX_ALT_BUF_SIZE];

#ifdef UART1_DEBUG_ENABLE
volatile uint8_t RXDebugBuf[40];
volatile uint8_t TXDebugBuf[40];
volatile uint8_t RXDebugLen;
volatile uint8_t TXDebugLen;
#endif

//-----------------------------------------
// Local enums
//
typedef enum {
    NO_PACKET = 0,
    SENDING_PACKET,
} TXPacketStateType;

typedef enum {
    WAITING_FOR_PACKET = 0,
    RECEIVING_PACKET,
} RXReceiveStateType;

typedef enum {
    BUFFER_NORMAL = 0,
    BUFFER_ALT,
    BUFFER_PRIORITY,
} BufferType;

typedef enum {
    RX_STATE_SYNC_0 = 0,
    RX_STATE_SYNC_1,
    RX_STATE_SYNC_2,
    RX_STATE_SYNC_3,
    RX_STATE_COMMAND,
    RX_STATE_SENDER,
    RX_STATE_TIME_0,
    RX_STATE_TIME_1,
    RX_STATE_TIME_2,
    RX_STATE_TIME_3,
    RX_STATE_LENGTH_0,
    RX_STATE_LENGTH_1,
    RX_STATE_DATA,
    RX_STATE_CRC_0,
    RX_STATE_CRC_1,
} RXPacketStateType;

// These are for the BufferSel parameter of PacketComputeCRC()
#define MAINLINE_BUFFER 0
#define ISR_BUFFER 1

//------------------------------------------
// static function prototypes
//
static int32_t RXBufPush(uint8_t Data);
static int32_t RXAltBufPush(uint8_t Data);
//static int32_t RXPriorityBufPush(uint8_t Data);
static int32_t RXBufPull(uint8_t *Data);
static int32_t TXBufPush(uint8_t Data);
static int32_t TXAltBufPull(uint8_t *Data);
static inline void IncrimentRXInPtr(void);
static inline void IncrimentRXOutPtr(void);
static inline void IncrimentRXPriorityInPtr(void);
static inline void IncrimentRXPriorityOutPtr(void);
static inline void IncrimentTXInPtr(void);
static inline void IncrimentTXOutPtr(void);
static inline void ResetTXAltBuf(void);
static inline void IncrimentTXAltInPtr(void);
static inline void IncrimentTXAltOutPtr(void);
static inline void IncrimentTXPriorityInPtr(void);
static inline void IncrimentTXPriorityOutPtr(void);
static inline void IncrimentRXAltInPtr(void);
static inline void IncrimentRXAltOutPtr(void);
static inline void ResetRXAltBuf(void);
static int32_t TXPriorityBufPull(uint8_t *Data);
static void UART_waitForTransferCompletion(void);
static int32_t PacketNewByteRX(uint8_t Byte);
static void SelectTXBuffer(BufferType Type);
static BufferType GetTXBuffer(void);
static uint32_t PacketComputeCRC(uint8_t Command, PacketSenderType Sender, uint32_t Length, uint32_t Timestamp, uint8_t * Data, uint8_t BufferSel);
static int32_t GeneratePacket(uint8_t Command, PacketSenderType Sender, uint32_t Length, uint8_t * Data);

//------------------------------------------
// Static variables
volatile static uint32_t ProcessingPriorityCommand = false;
volatile static uint32_t UsePrimaryTXBuffer = true;
volatile static uint32_t UsePrimaryRXBuffer = true;
volatile static TXPacketStateType TXPacketState = NO_PACKET;
volatile static TXPacketStateType TXPriorityPacketState = NO_PACKET;
volatile static RXReceiveStateType RXReceiveState = WAITING_FOR_PACKET;
volatile static BufferType CurrentTXBuffer = BUFFER_NORMAL;
volatile static BufferType CurrentRXBuffer = BUFFER_NORMAL;
static bool SyncFieldActive = false;
volatile static uint32_t NewPacketReady = 0;
volatile static uint32_t DeferredPicoCsend = false;

//------------------------------------------
// Public functions
/// TODO: Reorganize functions
//

/*
 * PacketBegin() must be called before sending out the first byte of 
 * a packet. It doesn't matter if Packet Mode is turned on or not,
 * it still must be called. After the 'packet' (real packet or just
 * reply data for one command) is complete, then you must call
 * PacketEnd().
 *
 * Its purpose is to set up the buffer that bytes get sent to, and
 * to error check that two pieces of code are not trying to write
 * into the same packet, and to handle high priority command
 * responses.
 *
 * It returns 0 on success, or -1 for any failure.
 *
 * You do not need to call this (or PacketEnd()) if you call
 * CreatePacket() as it does it for you.
 */
int32_t PacketBegin(void)
{
    // If we are in the middle of a PicoC generated packet (which is all the time
    // a PicoC program is running) then close out that packet and start our new one.
    if (PicoCRunning && TXPacketState == SENDING_PACKET)
    {
		PacketEnd(true);
        DeferredPicoCsend = true;
    }
    // Are we already in the middle of sending one packet?
    if (TXPacketState == SENDING_PACKET)
    {
        // Yup, so see if we can delay this outgoing packet until the main packet is done
        if (TXPriorityPacketState == NO_PACKET)
        {
            // Switch over to Priority TX Buffer
            SelectTXBuffer(BUFFER_PRIORITY);
            
            // Update our state
            TXPriorityPacketState = SENDING_PACKET;
        }
        else
        {
            /// TODO: Generate error here
            while(1)
            {DEBUG_H15_TOGGLE();}
//            SendErrorPacket((uint8_t *)"Error: PacketBegin() already in SENDING_PACKET state.\r\n");
            return (-1);
        }
    }
    else
    {
        TXPacketState = SENDING_PACKET;

        // Direct all bytes to the ALT TX buffer (256KB)
        SelectTXBuffer(BUFFER_ALT);
    }
    return (0);
}


/*
 * This function should be called when a single unit of data is done (to PC).
 * For example, you can do
 *         PacketBegin();
 *         printf("some data");
 *         PacketEnd(true);
 *
 * PacketBegin() will make sure that printf() points to the right output buffer.
 * Then PacketEnd() will (if necessary) copy the data from that buffer to the real
 * output buffer.
 * 
 * The CreatePacket argument to this function needs to be set to true for all
 * calls (except from within CreatePacket()).
 * 
 * It will return 0 for success and -1 for failure.
*/
int32_t PacketEnd(bool CreatePacket)
{
    uint8_t c = 0;

    // We always need to start by sending all of the characters to the
    // output buffers.
    fflush(NULL);

    // Are we in the middle of sending a packet?
    if (TXPacketState == SENDING_PACKET)
    {
        // Are we done with storing a priority packet in the priority buffer
        if (TXPriorityPacketState == SENDING_PACKET)
        {
            TXPriorityPacketState = NO_PACKET;
            
            // Switch back to normal TX buffer
            UsePrimaryTXBuffer = true;
        }
        else
        {
            // If packet mode is turned on, then we need to create a packet from
            // the data that's been put in the ALT TX buffer
            if (CreatePacket && GetOption(OPT_PACKET_MODE))
            {
                SelectTXBuffer(BUFFER_NORMAL);
                if (TXAltBufCount)
                {
                    // If the second byte is '#' then use the third byte as the command
                    if (TXAltBuf[1] == '#')
                    {
                        GeneratePacket(TXAltBuf[2], SENDER_RCM_BFIN, TXAltBufCount, (uint8_t *)TXAltBuf);
                    }
                    else
                    {
                        // Otherwise the second byte will do just fine
                        GeneratePacket(TXAltBuf[1], SENDER_RCM_BFIN, TXAltBufCount, (uint8_t *)TXAltBuf);
                    }
                }
                ResetTXAltBuf();
            }
            else
            {
                SelectTXBuffer(BUFFER_NORMAL);
                // We do not need to create a 'packet' when transferring data to the main buffer
                while (TXAltBufCount)
                {
                    TXAltBufPull(&c);
                    TXBufPush(c);
                }
                ResetTXAltBuf();
            }

            // Check to see if we have any packets queued up in the priority buffer
            if (TXPriorityBufCount)
            {
                SelectTXBuffer(BUFFER_NORMAL);
                // Yup, so dump the priority buffer into the main buffer
                while (TXPriorityBufCount)
                {
                    TXPriorityBufPull(&c);
                    TXBufPush(c);
                }
            }
            
            TXPacketState = NO_PACKET;
        }
        // If we interrupted a PicoC program's packet with this send, then make sure
        // we clean up and begin a new packet for PicoC's output.
        if (DeferredPicoCsend)
        {
            PacketBegin();
            DeferredPicoCsend = false;
        }
    }
    else
    {
        while(1)
        {
            DEBUG_H15_TOGGLE()		// PacketEnd bad state error
        }
        /// TODO: Generate error here
        SendErrorPacket((uint8_t *)"Error: PacketEnd() already in NO_PACKET state.\r\n");
        return (-1);
    }
    return (0);
}

/*
 * Wrapper function to print a string as an error packet 
 */
int32_t SendErrorPacket(uint8_t * String)
{
    SendPacket('E', SENDER_RCM_BFIN_ERROR, strlen((char *)String), String);
    return(0);
}

/*
 * Private function
 * Writes out the various fields of an outgoing packet, including CRC.
 *
 */
int32_t GeneratePacket(uint8_t Command, PacketSenderType Sender, uint32_t Length, uint8_t * Data)
{
    uint32_t x;
    uint32_t CRC = 0;
    uint32_t time;
#if DEBUG_CORRUPT_PACKET_TEST
    static uint32_t corrpt_packet_count = 0;
#endif
    
#ifdef UART1_PACKET_DEBUG
    char tmp[20];
    
    // For debugging packet mode, spit out the entire packet
    // on UART1, as hex bytes
    uart1SendString((unsigned char *)"-<");
#endif
    // First the header
    SyncFieldActive = true;
    uart0SendChar(0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, (char *)"%02x:", (uint8_t)0xFF);
    uart1SendString((unsigned char *)tmp);
#endif
    uart0SendChar(0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)0xFF);
    uart1SendString((unsigned char *)tmp);
#endif
    uart0SendChar(0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)0xFF);
    uart1SendString((unsigned char *)tmp);
#endif
    uart0SendChar(0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)0xFF);
    uart1SendString((unsigned char *)tmp);
#endif
    SyncFieldActive = false;
    // Then the command
    uart0SendChar(Command);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)Command);
    uart1SendString((unsigned char *)tmp);
#endif
    // And the sender
    uart0SendChar(Sender);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)Sender);
    uart1SendString((unsigned char *)tmp);
#endif
    // Then the current timestamp
    time = readRTC();
#if DEBUG_CORRUPT_PACKET_TEST
    // If we compile with DEBUG_CORRUPT_PACKET_TEST turned on, then every 100th
    // packet will be sent out with a missing byte (the first byte of the time
    // field). But everything else will be normal.
    corrpt_packet_count++;
    if (corrpt_packet_count >= 100) {
        corrpt_packet_count = 0;
    }
    else {
        uart0SendChar((time >> 24) & 0xFF);
    }
#else
    uart0SendChar((time >> 24) & 0xFF);
#endif
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)((time >> 24) & 0xFF));
    uart1SendString((unsigned char *)tmp);
#endif
    uart0SendChar((time >> 16) & 0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)((time >> 16) & 0xFF));
    uart1SendString((unsigned char *)tmp);
#endif
    uart0SendChar((time >> 8) & 0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)((time >> 8) & 0xFF));
    uart1SendString((unsigned char *)tmp);
#endif
    uart0SendChar(time & 0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)(time & 0xFF));
    uart1SendString((unsigned char *)tmp);
#endif
    // Next the length
    uart0SendChar((Length >> 8) & 0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)((Length >> 8) & 0xFF));
    uart1SendString((unsigned char *)tmp);
#endif
    uart0SendChar(Length & 0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)(Length & 0xFF));
    uart1SendString((unsigned char *)tmp);
#endif
    // Then the data
    for (x = 0; x <  Length; x++)
    {
        uart0SendChar(Data[x]);
#ifdef UART1_PACKET_DEBUG
        sprintf(tmp, "%02x:", (uint8_t)Data[x]);
        uart1SendString((unsigned char *)tmp);
#endif
    }
    // And finally a checksum
    CRC = PacketComputeCRC(Command, Sender, Length, time, Data, MAINLINE_BUFFER);
    uart0SendChar((CRC >> 8) & 0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)((CRC >> 8) & 0xFF));
    uart1SendString((unsigned char *)tmp);
#endif
    uart0SendChar(CRC & 0xFF);
#ifdef UART1_PACKET_DEBUG
    sprintf(tmp, "%02x:", (uint8_t)(CRC & 0xFF));
    uart1SendString((unsigned char *)tmp);
    uart1SendString((unsigned char *)"    ");
    for (x=0; x < Length; x++)
    {
        sprintf(tmp, "%c", Data[x]);
        uart1SendString((unsigned char *)tmp);
    }
    uart1SendString((unsigned char *)"\r\n");
#endif
    return(0);
}

// This function will create a packet from the given info and store it
// in the TX buffer. If Packet Mode is not turned on, then it will just
// take the data and copy it into the output buffer like normal
int32_t SendPacket(uint8_t Command, PacketSenderType Sender, uint32_t Length, uint8_t * Data)
{
    uint32_t x;
    
    if (Length)
    {
        if (GetOption(OPT_PACKET_MODE))
        { 
            PacketBegin();
            GeneratePacket(Command, Sender, Length, Data);
            PacketEnd(false);
        }
        else
        {
            PacketBegin();
            for (x = 0; x < Length; x++)
            {
                uart0SendChar(*Data++);
            }
            PacketEnd(false);
        }
    }
    return 0;
}

int32_t ReceiveBegin(void)
{
    if (RXReceiveState == WAITING_FOR_PACKET)
    {
        RXReceiveState = RECEIVING_PACKET;
    }
    else
    {
        /// TODO: Generate error here
        SendErrorPacket((uint8_t *)"Error: ReceiveBegin() not in WAITING_FOR_PACKET state.\r\n");
    }
    return(0);
}

int32_t ReceiveEnd(void)
{
    if (RXReceiveState == RECEIVING_PACKET)
    {
        RXReceiveState = WAITING_FOR_PACKET;
    }
    else
    {
        /// TODO: Generate error here
        SendErrorPacket((uint8_t *)"Error: ReceiveEnd() not in RECIVING_PACKET state.\r\n");
    }
    return(0);
}

// Switch the uart0SendChar() call to point to different buffers
void SelectTXBuffer(BufferType Type)
{
    CurrentTXBuffer = Type;
}

BufferType GetTXBuffer(void)
{
    return CurrentTXBuffer;
}

// Compute the CRC-16 CCITT checksum on a single packet
// NOTE: This could be done much more efficiently if we had the entire packet consecutively in RAM
// Since this gets called in both mainline code and from the ISR, we need to make sure it's re-entrant safe.
// To do this, we have two buffers, one for mainline and one for interrupt, and select between them with the BufferSel parameter.
uint32_t PacketComputeCRC(
    uint8_t Command, 
    PacketSenderType Sender, 
    uint32_t Length, 
    uint32_t Timestamp, 
    uint8_t * Data, 
    uint8_t BufferSel)
{
    static uint8_t LocalDataArray[2][0xFFFF + 12];
    uint8_t * LocalData;
    uint16_t CRC = 0;
    uint32_t i = 0;

    if (BufferSel >= 1) {
        BufferSel = 1;
    }
    
    LocalData = &LocalDataArray[BufferSel][0];
    
    LocalData[0] = 0xFF;
    LocalData[1] = 0xFF;
    LocalData[2] = 0xFF;
    LocalData[3] = 0xFF;
    LocalData[4] = (uint8_t)Command;
    LocalData[5] = (uint8_t)Sender;
    LocalData[6] = (uint8_t)(Timestamp >> 24);
    LocalData[7] = (uint8_t)((Timestamp >> 16) & 0xFF);
    LocalData[8] = (uint8_t)((Timestamp >> 8) & 0xFF);
    LocalData[9] = (uint8_t)(Timestamp & 0xFF);
    LocalData[10] = (uint8_t)(Length >> 8);
    LocalData[11] = (uint8_t)(Length & 0xFF);
    
    // Limit length so we don't go beyond the end of our 64K packet size
    if (Length > 0xFFFF)
    {
        Length = 0xFFFF;
    }

    // Copy over the rest of the packet
    for (i = 0; i < Length; i++)
    {
        LocalData[i+12] = Data[i];
    }

    CRC = crc16_ccitt((void *)LocalData, Length + 12);

    return(CRC);
}


void init_uart0(uint32_t baudrate) {
    uint32_t divider = ((PERIPHERAL_CLOCK / 16) / baudrate);

    *pPORTF_FER |= 0x0003; // enable UART0 pins

    *pUART0_GCTL = UCEN;
    *pUART0_LCR = DLAB;
    *pUART0_DLL = divider;
    *pUART0_DLH = divider >> 8;
    *pUART0_LCR = WLS(8); // 8 bit, no parity, one stop bit

    // dummy reads to clear possible pending errors / irqs
    uint8_t dummy = *pUART0_RBR;
    dummy = *pUART0_LSR;
    dummy = *pUART0_IIR;
    SSYNC;
    
    TXBufCount = 0;
    TXBufIn = 0;
    TXBufOut = 0;
    TXPriorityBufIn = 0;
    TXPriorityBufOut = 0;
    TXPriorityBufCount = 0;
    TXAltBufIn = 0;
    TXAltBufOut = 0;
    TXAltBufCount = 0;
    RXBufCount = 0;
    RXBufIn = 0;
    RXBufOut = 0;
    RXPriorityBufCount = 0;
    RXPriorityBufIn = 0;
    RXPriorityBufOut = 0;
}

void init_uart1(uint32_t baudrate) {
    uint32_t divider = ((PERIPHERAL_CLOCK / 16) / baudrate);

    *pPORTF_FER |= 0x000C; // enable UART1 pins
    *pUART1_GCTL = UCEN;
    *pUART1_LCR = DLAB;
    *pUART1_DLL = divider;
    *pUART1_DLH = divider >> 8;
    *pUART1_LCR = WLS(8); // 8 bit, no parity, one stop bit

    uint8_t dummy = *pUART1_RBR;
    dummy = *pUART1_LSR;
    dummy = *pUART1_IIR;
    SSYNC;
}

// Zero out all pending data in all of our output buffers
void FlushAllTXBuffers(void)
{
    volatile uint32_t IntTemp;

    disable_interrupts(IntTemp);
    TXBufCount = 0;
    TXBufIn = 0;
    TXBufOut = 0;
    TXPriorityBufIn = 0;
    TXPriorityBufOut = 0;
    TXPriorityBufCount = 0;
    TXAltBufIn = 0;
    TXAltBufOut = 0;
    TXAltBufCount = 0;
    enable_interrupts(IntTemp);
}

// Call to send a byte out the UART
// Works in ether interrupt/buffered or not modes
void uart0SendChar(uint8_t c) 
{
    static uint32_t FFByteCount = 0;
    if (UART0_INTERRUPTS_ENABLED)
    {
        // If we ever see a byte to go out and we're not in the middle of a packet,
        // then we have an error and should report it.
        if (TXPacketState != SENDING_PACKET)
        {
            TXBufPush('!');
        }
        TXBufPush(c);

        // Perform byte stuffing (if we see 3 0xFFs in a row, insert a 0x00)
        // But only if we're writing out into the real output buffer
        if (GetOption(OPT_PACKET_MODE) && GetTXBuffer() == BUFFER_NORMAL)
        {
            if (c == 0xFF && !SyncFieldActive)
            {
                FFByteCount++;
                if (FFByteCount >= 3)
                {
                    FFByteCount = 0;
                    // Add the byte into our buffer
                    TXBufPush(0x00);
                }
            }
            else
            {
                FFByteCount = 0;
            }
        }
    }
    else
    {
//        if (UseHardwareHandshaking)
//        {
            while (*pPORTHIO & 0x0001) // hardware serial flow control -
                continue; //    S32 pin 17 should be grounded to disable
//        }
        UART_waitForTransferCompletion();
        
        // Wait until previous character is out of the buffer
        // (but not necessarily shifted out - we don't check
        // TEMT - which would indicate that it was done shifting)
        while (!(*pUART0_LSR & THRE))
            continue;
        
        // Now send this one
        *pUART0_THR = c;
    }
}

///////////////////////////////////////////////////////////////
//poll the TEMT bit in LSR reg and wait until all data shifted out.
///////////////////////////////////////////////////////////////
void UART_waitForTransferCompletion(void)
{
    while (!(*pUART0_LSR & TEMT)) { }; // wait
    ssync();
}

void uart0SendString(uint8_t *s) {
    char a;
    while ((a = *s++)) {
        uart0SendChar(a);
    }
}

void uart0SendChars(uint8_t *buf, uint32_t size) {
    while (size--) {
        uart0SendChar(*buf++);
    }
}
//-----------------------------------------------------------
// These functions have changed to accommodate both polled and
//    interrupt-driven receive
//


uint8_t uart0GetCh() // This one waits for input
{
    // if interrupts enabled, wait until something is in buffer
    //   and return it.

    if (UART0_INTERRUPTS_ENABLED) {
        unsigned char c;
        uart0GetBufferCharWait(&c);
        return c;
    }

    // if interrupts not enabled, wait for Data Ready bit, then
    //   get char and return it.

    while (!(*pUART0_LSR & DR))
        ;
    return *pUART0_RBR;

}

uint8_t uart0GetChar(uint8_t *a) // this one does not wait
{
    // if interrupts enabled, check for something in buffer
    //   and return it.

    if (UART0_INTERRUPTS_ENABLED) {
        return uart0GetBufferCharNoWait(a);
    }

    // if interrupts not enabled, check Data Ready bit, if not set, return
    //   otherwise, get char and return it.
    if (!(*pUART0_LSR & DR))
    {
        return 0;
    }
    *a = *pUART0_RBR;
    return 1;
}

///////////////////////////////////////////////////////////////
//receive an LF ('ENTER') by polling the DR bit in the LSR register.
///////////////////////////////////////////////////////////////
int32_t UART_gets(uint8_t *str, uint32_t max)
{
    int i;
    
    ALLOW_RX(); 
    
    for(i = 0; i < max; i++)
    {
        str[i] = uart0GetCh();
 
        if (str[i] == 13)
        {
            return i+1;
        }
    }
      
    return max;
}

unsigned char uart0Signal()
{
    if (!(*pUART0_LSR & DR))
        return 0;
    return 1;
}

//-------------------------------------------------------------

void uart1SendChar(uint8_t c) {
    while (!(*pUART1_LSR & THRE))
        ;
    *pUART1_THR = c;
}

void uart1SendString(uint8_t *s) {
    char a;
    while ((a = *s++)) {
        uart1SendChar(a);
    }
}

void uart1SendChars(uint8_t *buf, uint32_t size) {
    while (size--) {
        uart1SendChar(*buf++);
    }
}

uint8_t uart1GetCh() {
    while (!(*pUART1_LSR & DR))
        ;
    return *pUART1_RBR;
}

uint8_t uart1GetChar(uint8_t *a) {
    if (!(*pUART1_LSR & DR))
        return 0;
    *a = *pUART1_RBR;
    return 1;
}

unsigned char uart1Signal()
{
    if (!(*pUART1_LSR & DR))
        return 0;
    return 1;
}

//---------------------------------------------------------------
// functions to push and pull chars from ring buffers
//        Handles both regular mode and priority mode, using
//        separate buffers for each
//
// This function is only called from the ISR. Takes Data and
// pushes it into the RX buffer. If there's room, it returns 0.
// If there is no room, then it returns -1.
int32_t RXBufPush(uint8_t Data) 
{
    // Check for full buffer
    if (RXBufCount < RX_BUF_SIZE - 50)
    {
        // Add the byte into our buffer
        RXBuf[RXBufIn] = Data;
        // And move ahead one byte
        IncrimentRXInPtr();
        return (0);
    }
    // No room - just drop the character
    /// TODO: put in a flag here that we can check from the 
    // mainline code that says we lost a byte.
    // TODO: Add hardware handshaking details here
    return (-1);
}

//int32_t RXPriorityBufPush(uint8_t Data)
//{            
//    // Check for full buffer
//    if (RXPriorityBufCount < RX_PRIORITY_BUF_SIZE - 2)
//    {
//        // Add the byte into our buffer
//        RXPriorityBuf[RXPriorityBufIn] = Data;
//        // And move ahead one byte
//        IncrimentRXPriorityInPtr();
//        return (0);
//    } 
//    // No room - just drop the character
//    // TODO: put in a flag here that we can check from the 
//    // mainline code that says we lost a byte.
//    // TODO: Add hardware handshaking details here
//    return (-1);
//}

int32_t RXAltBufPush(uint8_t Data)
{            
    // Check for full buffer
    if (RXAltBufCount < RX_ALT_BUF_SIZE - 100)
    {
        // Add the byte into our buffer
        RXAltBuf[RXAltBufIn] = Data;
        // And move ahead one byte
        IncrimentRXAltInPtr();
        return (0);
    }

    // No room - just drop the character
    // TODO: put in a flag here that we can check from the 
    // mainline code that says we lost a byte.
    // TODO: Add hardware handshaking details here
    return (-1);
}

// If there is a byte in the RX buffer, pulls it off and puts it
// into *Data and returns 1. If nothing is there to pull, returns 0.
int32_t RXBufPull(uint8_t *Data) 
{
    volatile uint32_t IntTemp;

    if (UsePrimaryRXBuffer)
    {
        /// TODO: Can we just disable the RX interrupt here?
        disable_interrupts(IntTemp);
        // Check if there's any data in the buffer
        if (RXBufCount == 0)
        {
            enable_interrupts(IntTemp);
            return (0);
        }
        *Data = RXBuf[RXBufOut];

        IncrimentRXOutPtr();

        enable_interrupts(IntTemp);
    }
    else
    {
        if (RXPriorityBufCount == 0)
        {
            return (0);
        }
        *Data = RXPriorityBuf[RXPriorityBufOut];

        IncrimentRXPriorityOutPtr();
    }
    
    return (1);
}

int32_t RXAltBufPull(uint8_t *Data) 
{
    if (RXAltBufCount == 0)
    {
        return (0);
    }
    *Data = RXAltBuf[RXAltBufOut];

    IncrimentRXAltOutPtr();

    return (1);
}

int32_t TXPriorityBufPull(uint8_t *Data) 
{
    if (TXPriorityBufCount == 0)
    {
        return (0);
    }
    *Data = TXPriorityBuf[TXPriorityBufOut];

    IncrimentTXPriorityOutPtr();

    return (1);
}

int32_t TXAltBufPull(uint8_t *Data) 
{
    if (TXAltBufCount == 0)
    {
        return (0);
    }
    *Data = TXAltBuf[TXAltBufOut];

    IncrimentTXAltOutPtr();

    return (1);
}

// Takes Data and
// pushes it into the TX buffer. If there's room, it returns 1.
// If there is no room, then it returns 0.
// Also performs byte stuffing if packet mode is turned on
int32_t TXBufPush(uint8_t Data) 
{
    uint32_t IntTemp;

    if (GetTXBuffer() == BUFFER_NORMAL)
    {
        // Check for full buffer
        // If there is no room in the TX buffer, then wait
        // until there is.
        while(TXBufCount >= (TX_BUF_SIZE - 10))
        {
            DEBUG_H15_TOGGLE()        // TX Buffer Overflow
            StopAllStreaming();
        }

        /// TODO: Can we just disable the TX interrupt here?
        disable_interrupts(IntTemp);
        // Add the byte into our buffer
        TXBuf[TXBufIn] = Data;
        // And move ahead one byte
        IncrimentTXInPtr();
        // If we are not transmitting a byte right now,
        // then start things off.
        if (TXBufCount && (*pUART0_LSR & THRE) && !CTS_TRIGGERED)
        {
            TXSendNextByte();
        }
        enable_interrupts(IntTemp);
    }
    else if (GetTXBuffer() == BUFFER_PRIORITY)
    {
        // We are using the Priority buffer now

        // Check for full buffer
        // If there is no room in the TX buffer, then wait
        // until there is.
        if(TXPriorityBufCount >= (TX_PRIORITY_BUF_SIZE - 10))
        {
            /// TODO: Generate an error packet here - we have to drop the byte
            DEBUG_H15_TOGGLE()		// TX Buffer Overflow
            return(0);
        }

        // Add the byte into our buffer
        TXPriorityBuf[TXPriorityBufIn] = Data;
        // And move ahead one byte
        IncrimentTXPriorityInPtr();
    }
    else if (GetTXBuffer() == BUFFER_ALT)
    {
        // Send byte to alternate buffer

        // Check for full buffer
        // If there is no room in the TX buffer, then wait
        // until there is.
        if(TXAltBufCount >= (TX_ALT_BUF_SIZE - 10))
        {
            /// TODO: Generate an error packet here - we have to drop the byte
            DEBUG_H15_TOGGLE()		// TX Buffer Overflow
            return(0);
        }

        // Add the byte into our buffer
        TXAltBuf[TXAltBufIn] = Data;
        // And move ahead one byte
        IncrimentTXAltInPtr();
    }
    return (1);
}

void uart0TurnOnInterrupts(void)
{
    // Init buffer pointers
    TXBufCount = 0;
    TXBufIn = 0;
    TXBufOut = 0;
    RXBufCount = 0;
    RXBufIn = 0;
    RXBufOut = 0;
    RXPriorityBufCount = 0;
    RXPriorityBufIn = 0;
    RXPriorityBufOut = 0;
    processing_priority_command = false;

    // Enable UART interrupts on transmit, status, and receive
    *pUART0_IER |= (ERBFI | ELSI | ETBEI);

    // Unmask IVG10 interrupt
    *pIMASK |= EVT_IVG10;

    // Unmask UART interrupts (RX and TX)
    *pSIC_IMASK |= (IRQ_DMA9 | IRQ_DMA8);
}

void uart0TurnOffInterrupts(void)
{
    *pUART0_IER &= ~(ERBFI | ELSI | ETBEI);
}

// Move a TX buffer pointer
inline void IncrimentTXInPtr(void)
{
    TXBufIn++;
    if (TXBufIn >= TX_BUF_SIZE)
    {
        TXBufIn = 0;
    }
    TXBufCount++;
}

inline void IncrimentTXOutPtr(void)
{
    TXBufCount--;
    TXBufOut++;
    if (TXBufOut >= TX_BUF_SIZE)
    {
        TXBufOut = 0;
    }
}

inline void ResetTXAltBuf(void)
{
    TXAltBufIn = 0;
    TXAltBufOut = 0;
    TXAltBufCount = 0;
}

inline void IncrimentTXAltInPtr(void)
{
    TXAltBufIn++;
    if (TXAltBufIn >= TX_ALT_BUF_SIZE)
    {
        TXAltBufIn = 0;
    }
    TXAltBufCount++;
}

inline void IncrimentTXAltOutPtr(void)
{
    TXAltBufCount--;
    TXAltBufOut++;
    if (TXAltBufOut >= TX_ALT_BUF_SIZE)
    {
        TXAltBufOut = 0;
    }
}

// Move high priority response TX buffer pointer
inline void IncrimentTXPriorityInPtr(void)
{
    TXPriorityBufIn++;
    if (TXPriorityBufIn >= TX_PRIORITY_BUF_SIZE)
    {
        TXPriorityBufIn = 0;
    }
    TXPriorityBufCount++;
}

inline void IncrimentTXPriorityOutPtr(void)
{
    TXPriorityBufCount--;
    TXPriorityBufOut++;
    if (TXPriorityBufOut >= TX_PRIORITY_BUF_SIZE)
    {
        TXPriorityBufOut = 0;
    }
}

// Move RX buffer pointer
inline void IncrimentRXInPtr(void)
{
    RXBufIn++;
    if (RXBufIn >= RX_BUF_SIZE)
    {
        RXBufIn = 0;
    }
    RXBufCount++;
}

inline void IncrimentRXOutPtr(void)
{
    RXBufCount--;
    RXBufOut++;
    if (RXBufOut >= RX_BUF_SIZE)
    {
        RXBufOut = 0;
    }
}

// Move RX priority buffer pointer
inline void IncrimentRXPriorityInPtr(void)
{
    RXPriorityBufIn++;
    if (RXPriorityBufIn >= RX_PRIORITY_BUF_SIZE)
    {
        RXPriorityBufIn = 0;
    }
    RXPriorityBufCount++;
}

inline void IncrimentRXPriorityOutPtr(void)
{
    RXPriorityBufCount--;
    RXPriorityBufOut++;
    if (RXPriorityBufOut >= RX_PRIORITY_BUF_SIZE)
    {
        RXPriorityBufOut = 0;
    }
}

// Move RX Alt buffer pointer
inline void IncrimentRXAltInPtr(void)
{
    RXAltBufIn++;
    if (RXAltBufIn >= RX_ALT_BUF_SIZE)
    {
        RXAltBufIn = 0;
    }
    RXAltBufCount++;
}

inline void IncrimentRXAltOutPtr(void)
{
    RXAltBufCount--;
    RXAltBufOut++;
    if (RXAltBufOut >= RX_ALT_BUF_SIZE)
    {
        RXAltBufOut = 0;
    }
}

inline void ResetRXAltBuf(void)
{
    RXAltBufIn = 0;
    RXAltBufOut = 0;
    RXAltBufCount = 0;
}


// Call only when buffering/interrupts are enabled
uint8_t uart0GetBufferCharWait(uint8_t *data) {

    if (UsePrimaryRXBuffer)
    {
        while (RXBufCount == 0)
        {}
    }
    else
    {
        while (RXPriorityBufCount == 0)
        {}
    }

    RXBufPull(data);

    return 1;
}

// Call only when buffering/interrupts are enabled
uint8_t uart0GetBufferCharNoWait(uint8_t *data) {
    if (RXBufCount) 
    {
        RXBufPull(data);
        return 1;
    } else
        return 0;
}

// Send the next byte from the TX buffer out to UART0
inline void TXSendNextByte(void)
{
    *pUART0_THR = TXBuf[TXBufOut];
    IncrimentTXOutPtr();
}

void uart0_ISR()
{
    uint8_t identification_register = *pUART0_IIR;
    uint32_t CTSCounter = 0;
    while (identification_register != 0x01)
    {
        switch (identification_register)
        {
            // Is our TX buffer empty? (THRE bit in LSR set)
            case 0x02:
            {
                // Yup. If we have another character to send, send it
                // If we're being held off by the radio, then don't send
                // now but wait until the 1ms tick figures out to start
                // sending again.
                if (TXBufCount)
                {
                    CTSCounter = 0;
                    if (CTS_TRIGGERED)
                    {
                        while((CTSCounter < 10) && CTS_TRIGGERED)
                        {
                            CTSCounter++;
                            delayUS(1);

                            if(*pUART0_LSR & DR)
                            {
                                // Read data
                                uint8_t c = *pUART0_RBR;

                                // Set our last-seen RX byte time to zero ms. 1ms ISR increments this.
                                UART0LastByteRX = 0;

                                // If we're in packet mode, then handle that here
                                if (GetOption(OPT_PACKET_MODE))
                                {
                                  PacketNewByteRX(c);
                                }
                                else
                                {
                                  RXBufPush(c);
                                }
                            }
                        }
                    }
                    if (!CTS_TRIGGERED)
                    {
                        TXSendNextByte();
                    }
                }
                break;
            }

            // Did we get this interrupt because of a line status error?
            case 0x06:
            {
                int temp3 = *pUART0_LSR;
                /// TODO: Implement counting of the various line errors
                temp3++;
                break;
            }

            // Did we get this interrupt because of a receive interrupt? (DR bit in LSR set)
            case 0x04:
            {
                // We're going to loop as long as there are bytes coming in.
                // This is necessary at the 2.5Mbps rate if cache is turned off
                while(*pUART0_LSR & DR)
                {
                    // Read data
                    uint8_t c = *pUART0_RBR;

                    // Set our last-seen RX byte time to zero ms. 1ms ISR increments this.
                    UART0LastByteRX = 0;

                    // If we're in packet mode, then handle that here
                    if (GetOption(OPT_PACKET_MODE))
                    {
                        PacketNewByteRX(c);
                    }
                    else
                    {
//// TODO: We can't use RXReceiveState because that takes place in 'mainline' code time
//// we need to know, in ISR time (ie where the bytes go INTO the buffer) if we're at the
//// first byte of the packet or not.
//                        // Only check for priority commands if we're looking for the first byte of a packet
//                        if (RXReceiveState == WAITING_FOR_PACKET)
//                        {
//                            // Manage priority commands
//                            if (c == PRIORITY_COMMAND_HEADER) 
//                            {
//                                ProcessingPriorityCommand = 1;
//                            }
//                            else if (c == PRIORITY_COMMAND_TRAILER) 
//                            {
//                                uint8_t InChar;
//                                // Switch to RX Priority buffer
//                                UsePrimaryRXBuffer = false;
//                                
//                                if (RXBufPull(&InChar))
//                                {                            
//                                    ProcessPriorityCommand(InChar);
//                                    // Eat all extra bytes in RX Priority buffer
//                                    while (RXPriorityBufCount)
//                                    {
//                                        IncrimentRXPriorityOutPtr();
//                                    }
//                                }
//                                // Switch back to normal RX buffer
//                                UsePrimaryRXBuffer = true;
//                                
//                                ProcessingPriorityCommand = 0;
//                            }
//                            else
 //                           {
//                                if (ProcessingPriorityCommand)
//                                {
//                                    // Priority commands are not stored in our RX buffer
//                                    RXPriorityBufPush(c);
//                                }
//                                else
//                                {
//                                    RXBufPush(c);
//                                }
//                            }
//                        }
//                        else
//                        {
//                            if (ProcessingPriorityCommand)
//                            {
//                                // Priority commands are not stored in our RX buffer
//                                RXPriorityBufPush(c);
//                            }
//                            else
//                            {
                                RXBufPush(c);
//                            }
//                        }
                    }
                }
                break;
            }
            default:
                break;
        }

        identification_register = *pUART0_IIR;
    }
    return;
}

///////////////////////////////////////////////////////////////
//  LOW LEVEL STUBS                                          //
///////////////////////////////////////////////////////////////
 
int
_open (const char *name,
       int         flags,
       int         mode)
{
  errno = ENOSYS;
  return -1;                    /* Always fails */
 
}       /* _open () */
 
int
_close (int   file)
{
  errno = EBADF;
  return -1;                    /* Always fails */
 
}       /* _close () */
 
int
_write (int   file,
        char *buf,
        int   nbytes)
{
    int i;
    /* Output character at at time */
    for (i = 0; i < nbytes; i++)
    {
        uart0SendChar(*buf++);
    }
    
    // This is total a hack - if we're running PicoC, then
    // close out a packet (and send what's in the stream) any time
    // we write a CR character.
    if (PicoCRunning || EditorRunning)
    {
        PacketEnd(true);
        PacketBegin();
    }
    return nbytes; 
}       /* _write () */
 
int
_read (int   file,
       char *buf,
       int   nbytes)
{
 
  int result = UART_gets( (uint8_t *) buf, nbytes );
  
  return result;                          /* EOF */
 
}       /* _read () */
 
int
_fstat (int          file,
        struct stat *st)
{
  st->st_mode = S_IFCHR;
  return  0;
 
}       /* _fstat () */
 
int
_lseek (int   file,
        int   offset,
        int   whence)
{
  return  0;
 
}       /* _lseek () */
 
int
isatty (int   file)
{
  return  1;
}       /* _isatty () */
 
int
_isatty (int   file)
{
  return  1;
}       /* _isatty () */

///////////////////////////////////////////////////////////////


#define SSYNC    asm("ssync;")
#define SUART_SEND0 *pPORTHIO |= 0x4000
#define SUART_SEND1 *pPORTHIO &= 0xBFFF
#define SUART_RECV  (*pPORTHIO & 0x8000)

static int suart_timebase;

void waituntilNS(unsigned int target) {
    if (target > PERIPHERAL_CLOCK) {
        target -= PERIPHERAL_CLOCK;
        while (*pTIMER4_COUNTER > target)  // wait for timer to wrap-around
            continue;
    }
    while (*pTIMER4_COUNTER < target)
        continue;
}

void suartInit(int baud) {  // use GPIO-H14 and H15 for soft uart.  H14 = TX, H15 = RX
    suart_timebase = 1000000000 / baud;  // define bit period in nanoseconds
    *pPORTHIO_DIR &= 0x3FFF;   // set H14 as output, H15 as input
    SSYNC;
    *pPORTHIO_DIR |= 0x4000;   // enable H14 output
    SSYNC;
    *pPORTHIO_INEN |= 0x8000;  // set H15 as input
    SSYNC;
    SUART_SEND0;  // set H14 high
    SSYNC;
}

void suartPutChar(unsigned char ch)  // soft uart send - transmit on GPIO-H14
{
    int ix;
    unsigned int twait;
    
    twait = *pTIMER4_COUNTER + suart_timebase;
    SUART_SEND1;  // send start bit
    waituntilNS(twait);
    for (ix=0; ix<8; ix++) {
        if (ch & 0x01)
            SUART_SEND0;  // output is inverted
        else
            SUART_SEND1;
        twait += suart_timebase; 
        waituntilNS(twait);
        ch = ch >> 1;
    }
    SUART_SEND0;  // send 2 stop bits
    twait += suart_timebase * 2;
    waituntilNS(twait); 
}

unsigned short suartGetChar(int timeout)  // check for incoming character, wait for "timeout" milliseconds
{
    int t0;
    unsigned short sx, smask;
    unsigned int twait;
    
    t0 = readRTC();
    sx = 0;
    
    while ((readRTC()-t0) < timeout) {  // wait for start bit
        if (SUART_RECV)
            continue;
        twait = *pTIMER4_COUNTER + ((suart_timebase * 4) / 3);
        waituntilNS(twait);  // wait for completion of start bit, then go 30% into next bit
        for (smask=1; smask<256; smask*=2) {
            if (SUART_RECV)
                sx += smask;
            twait += suart_timebase;
            waituntilNS(twait); 
        }
        twait += suart_timebase;  // wait for first stop bit
        waituntilNS(twait); 
        return (sx + 0x8000);  // set high bit to indicate received character
    }
    return 0;
}

/*
 * Packet functions
 */
 
/*
 * PacketNewByteRX is called for each bytes that comes in from the PC when
 * in Packet Mode. This is called from the RX ISR context, so it needs to be
 * really fast in order to get the next byte.
 *
 * This function needs to implement the Packet Mode state machine.
 * It needs to handle byte un-stuffing.
 * It needs to store the packet in a temporary buffer until the entire packet
 * can be verified and CRC checked. Then, if all is OK, the the data field is 
 * actually put into the RX buffer for the mainline code to
 * to extract. In the future, more can be done where the mainline code can
 * re-sync with the data stream.
 * If the CRC is bad, then this function needs to discard the packet and
 * wait for a sync field before starting a new packet.
 * If a priority packet is detected, then it should be handled here.
 */ 
int32_t PacketNewByteRX(uint8_t Byte)
{
    static uint32_t Command;
    static uint32_t Sender;
    static uint32_t Length;
    static uint32_t CRC;
    static uint32_t PacketTime;
    uint32_t RealCRC;
    static bool ThrowAwayNextByte = false;
    static uint32_t RXFFCount = 0;
    static uint32_t SyncFieldCounter = 0;
    static RXPacketStateType RXPacketState = RX_STATE_SYNC_0;
    uint8_t c = 0;
#ifdef UART1_PACKET_DEBUG
    static uint32_t i;
    static char tmp[20];
#endif
        
    // Do a re-sync check. If we see four 0xFF bytes in a row,
    // at ANY point in the data stream, it means we just saw a sync
    // field, so, if we're in the middle of a packet, then record
    // the error and reset the counters so we start recording a packet again.
    if (Byte == 0xFF)
    {
        SyncFieldCounter++;
        if (SyncFieldCounter >= 4) 
        {
            // We have a sync field - are we in the right place?
            if (RXPacketState == RX_STATE_SYNC_3)
            {
                // Yup, so we don't really need to do anything
                SyncFieldCounter = 0;            
            }
            else
            {
                // Nope, we're in the wrong place in the packet
                /// TODO: Increment an error counter here
                // Put us in the right place in the packet stream
                RXPacketState = RX_STATE_SYNC_3;
                RXFFCount = 0;
                // Now reset our current packet data (if any)
                ResetRXAltBuf();
                Command = 0;
                Length = 0;
                Sender = 0;
                CRC = 0;
#ifdef UART1_PACKET_DEBUG
                for (RXDebugLen = 0; RXDebugLen < 20; RXDebugLen++)
                {
                    // With something known
                    RXDebugBuf[RXDebugLen] = 0xEE;
                }
                RXDebugLen = 0;
#endif
            }
        }
    }
    else
    {
        SyncFieldCounter = 0;
    }
    
#ifdef UART1_PACKET_DEBUG
    RXDebugBuf[RXDebugLen] = Byte;
    if (RXDebugLen < 20)
    {
        RXDebugLen++;
    }
#endif

    // Check to see if we need to discard this byte
    if (ThrowAwayNextByte)
    {
        ThrowAwayNextByte = false;
        // If it's not a zero, then there was some type of error or corruption
        if (Byte != 0x00)
        {
            // So go back to looking for the sync field again.
            RXPacketState = RX_STATE_SYNC_0;
            /// TODO: Increment some type of error counter here for link quality measurement
        }
        else
        {
            // We do nothing with this byte, as it was a 0x00 after three 0xFFs.
            return 0;
        }
    }

    // Perform byte unstuffing. If we're not in the sync field, then
    // drop the 0x00 byte that happens after three 0xFFs.
    if (
        RXPacketState != RX_STATE_SYNC_0 
        &&
        RXPacketState != RX_STATE_SYNC_1
        &&
        RXPacketState != RX_STATE_SYNC_2 
        &&
        RXPacketState != RX_STATE_SYNC_3 
    )
    {
        if (Byte == 0xFF)
        {
            RXFFCount++;
            if (RXFFCount >= 3)
            {
                // Set our reminder to throw away the next byte that comes in
                ThrowAwayNextByte = true;
                RXFFCount = 0;
            }
        }
        else
        {
            RXFFCount = 0;
        }
    }
    
    // Run a state machine based upon where we are in the packet
    switch (RXPacketState)
    {
        case RX_STATE_SYNC_0:
            if (Byte == 0xFF) 
            {
                RXPacketState = RX_STATE_SYNC_1;
                // Clear out our packet stuff
                Command = 0;
                Length = 0;
                Sender = 0;
                CRC = 0;
                PacketTime = 0;
            }
            
            // As a special case, we also accept a single 'V' here
            // so that you can always do a version command to get the 
            // current packet mode status
            if (Byte == 'V')
            {
                RXBufPush(0x1F);
                // And tell GetPacketCommand() that there's a new packet ready
                NewPacketReady++;
            }

            // As a special case, we also accept a single 'Z' here
            // so that you can always turn off packet mode no matter if you're in it or not
            if (Byte == 'Z')
            {
                RXBufPush(0x1E);
                // And tell GetPacketCommand() that there's a new packet ready
                NewPacketReady++;
            }
            break;
            
        case RX_STATE_SYNC_1:
            if (Byte == 0xFF) 
            {
                RXPacketState = RX_STATE_SYNC_2;
            }
            else
            {
                RXPacketState = RX_STATE_SYNC_0;
            }
            break;
            
        case RX_STATE_SYNC_2:
            if (Byte == 0xFF) 
            {
                RXPacketState = RX_STATE_SYNC_3;
            }
            else
            {
                RXPacketState = RX_STATE_SYNC_0;
            }
            break;
            
        case RX_STATE_SYNC_3:
            if (Byte == 0xFF) 
            {
                RXPacketState = RX_STATE_COMMAND;
            }
            else
            {
                RXPacketState = RX_STATE_SYNC_0;
            }
            break;
            
        case RX_STATE_COMMAND:
            Command = Byte;
            RXPacketState = RX_STATE_SENDER;
            break;

        case RX_STATE_SENDER:
            Sender = Byte;
            RXPacketState = RX_STATE_TIME_0;
            break;
            
        case RX_STATE_TIME_0:
            PacketTime = Byte << 24;
            RXPacketState = RX_STATE_TIME_1;
            break;
            
        case RX_STATE_TIME_1:
            PacketTime = PacketTime | (Byte << 16);
            RXPacketState = RX_STATE_TIME_2;
            break;

        case RX_STATE_TIME_2:
            PacketTime = PacketTime | (Byte << 8);
            RXPacketState = RX_STATE_TIME_3;
            break;

        case RX_STATE_TIME_3:
            PacketTime = PacketTime | Byte;
            RXPacketState = RX_STATE_LENGTH_0;
            break;

        case RX_STATE_LENGTH_0:
            Length = Byte;
            RXPacketState = RX_STATE_LENGTH_1;
            break;
            
        case RX_STATE_LENGTH_1:
            Length = (Length << 8) + Byte;
            RXPacketState = RX_STATE_DATA;
            break;
            
        case RX_STATE_DATA:
            RXAltBufPush(Byte);
            if (RXAltBufCount == Length)
            {
                RXPacketState = RX_STATE_CRC_0;
            }
            break;
            
        case RX_STATE_CRC_0:
            CRC = Byte;
            RXPacketState = RX_STATE_CRC_1;
            break;
            
        case RX_STATE_CRC_1:
            CRC = (CRC << 8) + Byte;
            // Compute CRC here and compare with CRC in packet
            RealCRC = PacketComputeCRC(Command, Sender, Length, PacketTime, (uint8_t *)RXAltBuf, ISR_BUFFER);
            
            if (CRC == RealCRC)
            {
                // Copy over the entire packet into the real RX buffer (now that
                // we know it's good) so that functions can get at the data.
                while(RXAltBufCount)
                {
                    RXAltBufPull(&c);
                    RXBufPush(c);
                }
                // And tell GetPacketCommand() that there's a new packet ready
                NewPacketReady++;
            }
            else
            {
                /// TODO: Record this error in an error counter
#ifdef UART1_PACKET_DEBUG
                // For debugging packet mode, spit out the entire packet
                // on UART1, as hex bytes
                uart1SendString((unsigned char *)"->");
                for(i = 0; i < RXDebugLen; i++)
                {
                    sprintf(tmp, "%02x:", RXDebugBuf[i]);
                    uart1SendString((unsigned char *)tmp);
                }
                uart1SendString((unsigned char *)"    ");
                for (i=12; i < (RXDebugLen-2); i++) 
                {
                    sprintf(tmp, "%c", RXDebugBuf[i]);
                    uart1SendString((unsigned char *)tmp);
                }
                uart1SendString((unsigned char *)"\r\n");
                RXDebugLen = 0;
                sprintf(tmp, "CRC error %04x != %04x\r\n", (unsigned int)CRC, (unsigned int)RealCRC);
                uart1SendString((unsigned char *)tmp);
#endif
            }
            // And clear it out of our alt buffer
            ResetRXAltBuf();
            Command = 0;
            Length = 0;
            Sender = 0;
            CRC = 0;
            PacketTime = 0;
            RXPacketState = RX_STATE_SYNC_0;
            break;
        
        default:
            break;
    }

    return 0;
}

// Call this function in the main loop
// It will return true when there is a new command
// ready from the PC (either in Packet Mode or not) and
// the command will be stored in Command.
bool GetPacketCommand(uint8_t * Command)
{
#ifdef UART1_PACKET_DEBUG
    uint8_t i;
    char tmp[10];
#endif

    if (GetOption(OPT_PACKET_MODE))
    {
        if (NewPacketReady)
        {
            NewPacketReady--;
            RXBufPull(Command);
#ifdef UART1_PACKET_DEBUG
            // For debugging packet mode, spit out the entire packet
            // on UART1, as hex bytes
            uart1SendString((unsigned char *)"->");
            for(i = 0; i < RXDebugLen; i++)
            {
                sprintf(tmp, "%02x:", RXDebugBuf[i]);
                uart1SendString((unsigned char *)tmp);
            }
            uart1SendString((unsigned char *)"    ");
            for (i=12; i < (RXDebugLen-2); i++) 
            {
                sprintf(tmp, "%c", RXDebugBuf[i]);
                uart1SendString((unsigned char *)tmp);
            }
            uart1SendString((unsigned char *)"\r\n");
            RXDebugLen = 0;
#endif
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        if (RXBufCount)
        {
            RXBufPull(Command);
            return true;
        }
        else
        {
            return false;
        }
    }
    return 0;
}

// Return how many bytes are in each of the buffers
void ReadBufferStates(
    uint32_t * RXBufferCount, 
    uint32_t * RXPriorityBufferCount, 
    uint32_t * RXAlternateBufferCount, 
    uint32_t * TXBufferCount, 
    uint32_t * TXPriorityBufferCount, 
    uint32_t * TXAlternateBufferCount
)
{
	*RXBufferCount = RXBufCount;
	*RXPriorityBufferCount = RXPriorityBufCount;
	*RXAlternateBufferCount = RXAltBufCount;
	*TXBufferCount = TXBufCount;
	*TXPriorityBufferCount = TXPriorityBufCount;
	*TXAlternateBufferCount = TXAltBufCount;
}








































