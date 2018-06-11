/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  intuart.h - header for interrtup driven UART driver
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
 
#ifndef INT_UART_H_
#define INT_UART_H_

#include <stdint.h>
#include <stdbool.h>

//#define putchar uart0SendChar
#define putchars uart0SendChars
#define getch uart0GetCh
//#define getchar uart0GetChar
#define getsignal uart0Signal


#define SSYNC    asm("ssync;")

// Comment out to use no flow control
#define UART0_USE_HARDWARE_FLOWCONTROL
#define RX_BUF_SIZE                     0x20000   // 128KB
#define RX_PRIORITY_BUF_SIZE            0x1000    // 4KB
#define RX_ALT_BUF_SIZE                 0x20000   // 128KB
#define TX_BUF_SIZE                     0x20000   // 128KB
#define TX_PRIORITY_BUF_SIZE            0x1000    // 4KB
#define TX_ALT_BUF_SIZE                 0x20000   // 128KB
#define RTS_ASSERT                      *pPORTHIO_CLEAR = PH6;  // Pin 23 of RCM-Bfin header, tied to CTS of radio
#define RTS_DEASSERT                    *pPORTHIO_SET   = PH6;    //    Force low to receive data from radio
#define CTS_TRIGGERED                   (*pPORTHIO & PH0)       // Pin 17 of RCM-Bfin header, tied to RTS of radio
#define CTS_CLEAR_TRIGGER               *pPORTHIO_CLEAR = PH0;  //    Radio will set low when OK to send data to it
#define UART0_INTERRUPTS_ENABLED        (*pUART0_IER & (ERBFI || ETBEI || ELSI))
#define PRIORITY_COMMAND_HEADER         '['
#define PRIORITY_COMMAND_TRAILER        ']'

typedef enum
{
  SENDER_PC = 0,
  SENDER_RCM_BFIN = 0x80,
  SENDER_RCM_BFIN_ERROR,
} PacketSenderType;

// Public function prototypes
void uart0TurnOnInterrupts();
void uart0TurnOffInterrupts();
uint8_t uart0GetBufferCharWait(uint8_t *data);
uint8_t uart0GetBufferCharNoWait(uint8_t *data);
//void uart0_ISR() __attribute__((interrupt_handler, nesting));
void uart0_ISR() __attribute__((interrupt_handler));
inline void TXSendNextByte(void);
void init_uart0(uint32_t baudrate);
void init_uart1(uint32_t baudrate);
void uart0SendChar(uint8_t s);
void uart0SendString(uint8_t *s);
void uart0SendChars(uint8_t *s, uint32_t count);
uint8_t uart0GetCh();
uint8_t uart0GetChar(uint8_t *s);
uint8_t uart0Signal();
void uart1SendChar(uint8_t s);
void uart1SendString(uint8_t *s);
void uart1SendChars(uint8_t *s, uint32_t count);
uint8_t uart1GetCh();
uint8_t uart1GetChar(uint8_t *s);
uint8_t uart1Signal();
void ReadBufferStates(
  uint32_t * RXBufferCount, 
  uint32_t * RXPriorityBufferCount, 
  uint32_t * RXAlternateBufferCount, 
  uint32_t * TXBufferCount, 
  uint32_t * TXPriorityBufferCount, 
  uint32_t * TXAlternateBufferCount
);
void FlushAllTXBuffers(void);

// Packet related functions
int32_t PacketBegin(void);
int32_t PacketEnd(bool CreatePacket);
int32_t ReceiveBegin(void);
int32_t ReceiveEnd(void);
int32_t SendErrorPacket(uint8_t * String);
int32_t SendPacket(uint8_t Command, PacketSenderType Sender, uint32_t Length, uint8_t * Data);
bool GetPacketCommand(uint8_t * Command);

// Public variables
extern volatile uint32_t TXBufCount;


// TODO: Remove these and put them in their own file
void suartInit(int baudrate);
void suartPutChar(unsigned char ch);
unsigned short suartGetChar(int timeout);


#endif /* UART_H */
