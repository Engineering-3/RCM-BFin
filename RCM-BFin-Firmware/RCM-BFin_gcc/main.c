/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Main.c - Initialization, main loop, and command parsing for RCM-Bfin
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
 
 
/*
 * Known issues:
 * 1) If a function needs to send data bigger than the alt TX buffer, it needs
 * to break up that data into multiple chunks. Data is not sent out as it is
 * generated, but rather once PacketEnd() is called. Currently buffer is at
 * 256K.
 *
 * 2) Operations on the flash buffer continue until they see a 0x00 in the buffer
 * then terminate. We should keep track of how many bytes are actually used in 
 * the buffer, and then do all operations to that size rather than some specific
 * piece of data (i.e. 0x00)
 *
 */
 
/*
 * TODO list:
 * 1) Add error reporting if I2C bus is being held by other device when I2C command
 * is to be run.
 *
 * 2) Improve intuart.c by making interrupt driven operation possible for both UARTs
 *
 */
 
#include <blackfin.h>
#include <cdefBF537.h>
#include <stdio.h>
#include <gcc.h>
#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "system.h"
#include "rcm-bfin.h"
#include "gps.h"
#include "myfunc.h"
#include "xmodem.h"
#include "hokuyo.h"
#include "intuart.h"
#include "sysinit.h"
#include "cache.h"
#include "options.h"
#include "edit.h"
#include "streaming.h"
#include "i2c.h"

#ifdef STEREO
#include "stereo.h"
#endif

// This should not be here - where is a better place?
extern uint32_t CounterStreamVideoTimout;
uint32_t TimeBetweenStreamingFramesMS;

// Local function prototypes
static void ProcessCommands(unsigned char CharIn);
 int superdebug = 0;

int main() 
{
    init_io(); // Initialize LED, GPIO, serial flow & lasers.

    // We do this just to test all of the debug I/O pins - normally these would
    // be shut off by undefined #debug in config.h
    DEBUG_INIT()
    DEBUG_H15_LOW()
    DEBUG_H14_LOW()
    DEBUG_H13_LOW()
    DEBUG_H12_LOW()
    DEBUG_H11_LOW()
    DEBUG_H10_LOW()
    DEBUG_H9_LOW()
    DEBUG_H8_LOW()
    DEBUG_H15_HIGH()
    DEBUG_H14_HIGH()
    DEBUG_H13_HIGH()
    DEBUG_H12_HIGH()
    DEBUG_H11_HIGH()
    DEBUG_H10_HIGH()
    DEBUG_H9_HIGH()
    DEBUG_H8_HIGH()
    DEBUG_H15_LOW()
    DEBUG_H14_LOW()
    DEBUG_H13_LOW()
    DEBUG_H12_LOW()
    DEBUG_H11_LOW()
    DEBUG_H10_LOW()
    DEBUG_H9_LOW()
    DEBUG_H8_LOW()

    // In init.c, LED1 is on and LED2 is off. Here we 
    // turn 1 off and 0 on.
    LED1_OFF();
    LED0_ON();

    init_program_clocks();

    // NOTE: You must run with cache on if you want serial receive not to drop bytes (i.e. XMODEM)
    cache_init();

    init_uart0(UART0_BAUDRATE);

#ifdef UART1_DEBUG_ENABLE
    init_uart1(115200);
    uart1SendString((unsigned char *)"uart1 configured \n\r");
    //superdebug = 1;
    //tmp_buf[0] = 0xFF;
    //tmp_buf[1] = 0xFF;
    //tmp_buf[2] = 0xFF;
    //tmp_buf[3] = 0xFF;
    //tmp_buf[4] = 0x54;
    //tmp_buf[5] = 0x80;
    //tmp_buf[6] = 0x00;
    //tmp_buf[7] = 0x00;
    //tmp_buf[8] = 0x00;
    //tmp_buf[9] = 0x2E;
    //tmp_buf[10] = 0x00;
    //tmp_buf[11] = 0x02;
    //tmp_buf[12] = 0x23;
    //tmp_buf[13] = 0x54;
    //sprintf(out_buf, "CRC = %02X\n\r", crc16_ccitt(tmp_buf, 14));
    //uart1SendString(out_buf);
    //superdebug = 0;
#endif

    InitI2C();
    InitTMR0();
    Init_Interrupts();
    uart0TurnOnInterrupts();
    initRTC();
    initTMR4();
    init_colors();
    disable_failsafe();
    clear_sdram(); // Clears from 0x00200000 to 0x02000000, at about 235 MB/s (max for this part)
    camera_setup(); // Sets up the camera to 320x240
    
    PicoCRunning = false;
    EditorRunning = false;
    
    #ifdef STEREO
    if (master)
        serial_out_version();
    init_svs();
    if (master)
        check_for_autorun();
    #else
    serial_out_version();
    PacketBegin();
    check_for_autorun(); // (58ms)
    PacketEnd(true);
    
    #endif /* STEREO */

    // Initialize options
    SetOption(OPT_STREAM_VIDEO_ON, false);      // Video streaming off
    SetOption(OPT_VIDEO_TEST_MODE_1, false);    // Video test mode 1 off
    SetOption(OPT_PACKET_MODE, true);           // Packet mode set to on
    TimeBetweenStreamingFramesMS = 250;         // Default to 4fps in streaming mode

    // Turning both LEDs on shows that the boot process has completed.
    LED1_ON();
    LED0_ON();
        
    while (1) 
    {
        MainLoop(false);
    }
}

// This is the main RCM-Bfin firmware loop. Here we check to see if we've gotten
// a new character from the PC, and if so, handle the command. Also we handle
// blinking the LEDs and taking care of any streaming sensors or streaming 
// video. This main loop gets called from main() as well as from within the PicoC
// command processor between commands.
void MainLoop(bool CalledFromPicoC)
{
    uint8_t ch;
    // Check to see if a new command has come in from the PC (packet or not)
    while (GetPacketCommand(&ch)) 
    {
        if (CalledFromPicoC)
        {
            ReceiveEnd();
        }
        ReceiveBegin();
        // This function does all firmware command processing
        ProcessCommands(ch);
        reset_failsafe_clock();
        ReceiveEnd();
        if (CalledFromPicoC)
        {
            ReceiveBegin();
        }
    }

    // Each time through the loop, if we're supposed to stream video,
    // check to see if it's time to send it out. If so, do so.
    if (GetOption(OPT_STREAM_VIDEO_ON))
    {
        if (CounterStreamVideoTimout == 0)
        {
            CounterStreamVideoTimout = TimeBetweenStreamingFramesMS;
            grab_frame();
            send_frame();
        }
    }
    
    // Handle sensor streaming
    StreamingProcess();
 
    // Handle the autoincriment I2C processing
    process_autoinc_i2c();
 
    #ifdef STEREO
    if (!master) {  // if slave processor on SVS, check whether stereo_sync_flag has triggered
        if ((stereo_sync_flag == 0x0100) && (check_stereo_sync() == 0)) {
            stereo_sync_flag = 0;
            continue;
        }
        if ((stereo_sync_flag == 0) && (check_stereo_sync() == 0x0100)) {
            //printf("SVS slave:  stereo sync toggle\r\n");
            stereo_sync_flag = 0x0100;
            svs_grab(svs_calibration_offset_x, svs_calibration_offset_y, 0, 0);
            svs_send_features();
        }
    }
    #endif /* STEREO */
    check_failsafe();

    // Handle the LED state machine from the main loop
    run_leds();
}


// We have a high priority command. This function is called
// within the RX interrupt handler, to handle these high
// priority commands. This means that ANY existing code could
// be interrupted. So we must make sure that this code doesn't
// step on the toes of anything else going on.
void ProcessPriorityCommand(uint8_t CharIn)
{
    switch (CharIn) {
        case 'i':   // i2c read / write
            process_i2c();
            break;
        default:
            PacketBegin();
            uart0SendString((uint8_t *)"#?'1'");      // unknown command
            PacketEnd(true);
            break;
    }
    /// TODO: Eat any remaining characters if the command was not completed
}


/*
 Type-able characters that have not yet been used in the commands below (as first letters of a command)
 KkUuWwe"#&'(),:;=?@[]\^_`{}~ (note: may not be accurate)
*/
void ProcessCommands(uint8_t CharIn)
{
    static unsigned char AllowPicoCBackground = true;
    unsigned char TestChar;
    unsigned char *cp;
    int ix;
    unsigned int t0, loop;
    unsigned short sx;

    switch (CharIn) {
        case 'I':
            // This command does nothing if streaming video is turned on
            if (!GetOption(OPT_STREAM_VIDEO_ON))
            {
                grab_frame();
                send_frame();
            }
            break;
        case 'J':
            PacketBegin();
            send_80x64planar();
            PacketEnd(true);
            break;
        case 'G':
        case 'P':
            PacketBegin();
            httpd_request(CharIn);
            PacketEnd(true);
            break;
        case 'y':
            PacketBegin();
            invert_video();
            PacketEnd(true);
            break;
        case 'Y':
            PacketBegin();
            restore_video();
            PacketEnd(true);
            break;
        case 'o':   // turn on overlay_flag
            PacketBegin();
            overlay_on();
            PacketEnd(true);
            break;
        case 'O':   // turn off overlay_flag
            PacketBegin();
            overlay_off();
            PacketEnd(true);
            break;
        case 'a':   // 160 x 120
            PacketBegin();
            camera_reset(160);
            PacketEnd(true);
            break;
        case 'b':   // 320 x 240
            PacketBegin();
            camera_reset(320);
            PacketEnd(true);
            break;
        case 'c':   // 640 x 480
            PacketBegin();
            camera_reset(640);
            PacketEnd(true);
            break;
        case 'd':   // 1280 x 1024
        case 'A':   // 1280 x 1024
            PacketBegin();
            camera_reset(1280);
            PacketEnd(true);
            break;
        case 'V':   // send version string
            serial_out_version();
            break;
        case 0x1F:  // send version string, but force no packet even in packet mode
                    // This value is pushed into the command queue from within the 
                    // PacketNewByteRX() state machine to indicate a 'V' was
                    // seen at the beginning of a packet.
            serial_out_version_no_packet();
            break;
        case 'D':   // check battery
            PacketBegin();
            check_battery();
            PacketEnd(true);
            break;
        case 'R':   // show laser range
            PacketBegin();
            show_laser_range(0);
            PacketEnd(true);
            break;
        case 'r':   // show laser range with debug
            PacketBegin();
            show_laser_range(1);
            PacketEnd(true);
            break;
        case 'q':  // change image quality
            PacketBegin();
            change_image_quality();
            PacketEnd(true);
            break;
        case 'X':  // xmodem receive
            xmodem_receive((unsigned char *)FLASH_BUFFER, FLASH_BUFFER_SIZE);
            break;
        case '|': // Break out of running PicoC program
        case 0x1B: // Also do the same for ESC character
            // When we return, PicoC will pick this up and exit
            PicoCRunning = false;
            break;
        case 'Q': // execute C program from flash buffer
           switch (CopyPicoCOut())
            {
                case 0:
                    PicoCRunning = true;
                    PacketBegin();
                    picoc((char *)FLASH_BUFFER, AllowPicoCBackground);
                    PacketEnd(true);
                    CopyPicoCBack();
                    PicoCRunning = false;
                    break;
                case 1:
                    PacketBegin();
                    picoc((char *)FLASH_BUFFER2, AllowPicoCBackground);
                    PacketEnd(true);
                    CopyPicoCBack();
                    break;
                case -1:
                    PacketBegin();
                    printf("Error - PicoCCopyOut failed\n");
                    PacketEnd(true);
                    break;
                default:
                    PacketBegin();
                    printf("Error - too many levels deep into PicoC\n");
                    PacketEnd(true);
            }
            break;
        case '!': // run C interpreter interactively
            switch (CopyPicoCOut())
            {
                case 0:
                    PicoCRunning = true;
                    PacketBegin();
                    picoc(NULL, AllowPicoCBackground);
                    PacketEnd(true);
                    CopyPicoCBack();
                    PicoCRunning = false;
                    break;
                case 1:
                    PacketBegin();
                    picoc(NULL, AllowPicoCBackground);
                    PacketEnd(true);
                    CopyPicoCBack();
                    break;
                case -1:
                    PacketBegin();
                    printf("Error - PicoCCopyOut failed\n");
                    PacketEnd(true);
                    break;
                default:
                    PacketBegin();
                    printf("Error - too many levels deep into PicoC\n");
                    PacketEnd(true);
            }
            break;
        case 'B': // Send down a short PicoC program and exectue it
            switch (CopyPicoCOut())
            {
                case 0:
                    PicoCRunning = true;
                    PacketBegin();
                    ReadPicoCProgram((char *)FLASH_BUFFER, FLASH_BUFFER_SIZE);
                    PacketEnd(true);
                    PacketBegin();
                    picoc((char *)FLASH_BUFFER, AllowPicoCBackground);
                    PacketEnd(true);
                    CopyPicoCBack();
                    PicoCRunning = false;
                    break;
                case 1:
                    PacketBegin();
                    ReadPicoCProgram((char *)FLASH_BUFFER2, FLASH_BUFFER2_SIZE);
                    PacketEnd(true);
                    PacketBegin();
                    picoc((char *)FLASH_BUFFER2, AllowPicoCBackground);
                    PacketEnd(true);
                    CopyPicoCBack();
                    break;
                case -1:
                    PacketBegin();
                    printf("Error - PicoCCopyOut failed\n");
                    PacketEnd(true);
                    break;
                default:
                    PacketBegin();
                    printf("Error - too many levels deep into PicoC\n");
                    PacketEnd(true);
            }
            break;
        case '$': // prototype zone
            switch (getch()) {
                case '!':  // reset processor
                    PacketBegin();
                    reset_cpu();
                    PacketEnd(true);
                case '7':  // test of SVS using suart instead of spi
                    PacketBegin();
                    suartInit(10000000);
                    *pPORTHIO_DIR |= 0x0100;
                    *pPORTHIO |= 0x0100;  // set GPIO-H8 high to signal slave
                    cp = (unsigned char *)FLASH_BUFFER;
                    t0 = readRTC();
                    ix = 0;
                    loop = 1;
                    while ((readRTC() < t0 + 5000) && loop) {
                        sx = suartGetChar(100);
                        if (sx)
                            cp[ix++] = (unsigned char)(sx & 0x00FF);
                        if (sx == 0x8000)
                            loop = 0;
                    }
                    printf("received %d characters\r\n", ix);
                    printf("%s", cp);
                    printf("\r\n");                                    
                    *pPORTHIO &= 0xFEFF;  // set GPIO-H8 low to signal slave
                    PacketEnd(true);
                    break;
                case '8':  // test of SVS using suart instead of spi
                    PacketBegin();
                    suartInit(10000000);
                    *pPORTHIO &= 0xFEFF;
                    *pPORTHIO_DIR &= 0xFEFF;  // set GPIO-H8 as input
                    *pPORTHIO_INEN |= 0x0100;  
                    t0 = readRTC();
                    loop = 1;
                    while ((readRTC() < t0 + 5000) && loop) {
                        if (*pPORTHIO & 0x0100) {
                            for (TestChar=0x20; TestChar<=0x7A; TestChar++)  {
                                for (ix=0; ix<320; ix++) {
                                    suartPutChar(TestChar);
                                    delayNS(100);  // add an extra bit period
                                }
                            }
                            suartPutChar(0);
                            printf("sent %d characters\r\n", 0x5B*320);
                            loop = 0;
                        }
                    }
                    PacketEnd(true);
                    break;
                #ifdef STEREO
                case 'X':
                    PacketBegin();
                    svs_master((unsigned short *)FLASH_BUFFER, 
                        (unsigned short *)(FLASH_BUFFER+131072), 131072);
                    ix = (unsigned int)crc16_ccitt(FLASH_BUFFER, 131064);
                    printf("     CRC-sent: 0x%x\r\n", ix);
                    PacketEnd(true);
                    break;
                case 'R':
                    PacketBegin();
                    svs_slave((unsigned short *)FLASH_BUFFER,
                        (unsigned short *)(FLASH_BUFFER+131072), 131072);
                    printf("##$R SPI Slave\r\n");
                    ix = (unsigned int)crc16_ccitt(FLASH_BUFFER, 131064);
                    printf("     CRC-received: 0x%x\r\n", ix);
                    PacketEnd(true);
                    break;
                case '1':
                    PacketBegin();
                    /* toggle mapping on or off */
                    svs_enable_mapping = 1 - svs_enable_mapping;
                    PacketEnd(true);
                    break;
                case '2':
                    PacketBegin();
                    /* right camera */
                    svs_grab(svs_calibration_offset_x, svs_calibration_offset_y, 0, 0);
                    svs_send_features();
                    PacketEnd(true);
                    break;
                case '3':
                    PacketBegin();
                    /* left camera */
                    if (svs_receive_features() > -1) {
                        svs_match(200, 40, 18, 7, 3, 10, 200, 0);
                    }                            
                    PacketEnd(true);
                    break;
                case '4':
                    PacketBegin();
                    /* read calibration params */
                    svs_read_calib_params();
                    PacketEnd(true);
                    break;
                case 's':  // stereo + send disparities, should be called on the master
                    PacketBegin();
                    svs_stereo(1);
                    PacketEnd(true);
                    break;
                #endif /* STEREO */
                case 'g':  // gps test
                    PacketBegin();
                    gps_show();
                    PacketEnd(true);
                    break;
                case 'm':  // 80x64 motion vect test
                    PacketBegin();
                    motion_vect80x64();
                    PacketEnd(true);
                    break;
                case 'M':  // full frame motion vect test
                    PacketBegin();
                    ix = getch() & 0x0F;
                    motion_vect_test(ix);
                    PacketEnd(true);
                    break;
                case 'A':  // read analog channel 1-8, 11-18 or 21-28
                    PacketBegin();
                    read_analog();
                    PacketEnd(true);
                    break;
                case 'a':  // read analog channels from SRV_4WD
                    PacketBegin();
                    read_analog_4wd();
                    PacketEnd(true);
                    break;
                case 'C':  // read HMC6352 compass on i2c port 0x22
                    PacketBegin();
                    show_compass2x();
                    PacketEnd(true);
                    break;
                case 'c':  // read HMC5843 compass on i2c port 0x1E
                    PacketBegin();
                    show_compass3x();
                    PacketEnd(true);
                    break;
                case 'E':  // read motor encoders on GPIO-H14 / H15
                    PacketBegin();
                    read_encoders();
                    PacketEnd(true);
                    break;
                case 'e':  // read motor encoders on SRV-4WD
                    PacketBegin();
                    read_encoder_4wd();
                    PacketEnd(true);
                    break;
                case 'T':  // read tilt sensor channel 1-3 (x, y or z axis)
                    PacketBegin();
                    read_tilt();
                    PacketEnd(true);
                    break;
                case 'S':  // test SD card interface on RCM
                    PacketBegin();
                    testSD();
                    PacketEnd(true);
                    break;
                case 'H':    // Hokyuo sensor commands
                    switch (getch())  {
                        case 's': // Start hard iron calibration mode for compass
                            PacketBegin();
                            HokuyoCalibrateStart();
                            PacketEnd(true);
                            break;
                        case 'e': // End hard iron calibration mode for compass
                            PacketBegin();
                            HokuyoCalibrateEnd();
                            PacketEnd(true);
                            break;
                        case 'd': // Set compass deviation - four numerals, in 10ths of a degree (-1800 to 1800)
                            PacketBegin();
                            HokuyoSetDeviation();
                            PacketEnd(true);
                            break;
                        case 'v': // Set compasss variation - four numerals, in 10ths of a degree (-1800 to 1800)
                            PacketBegin();
                            HokuyoSetVariation();
                            PacketEnd(true);
                            break;
                        case 'p': // Print entire compass EEPROM contents
                            PacketBegin();
                            HokuyoPrintEEPROM();
                            PacketEnd(true);
                            break;
                        case 'h': // Print just compass heading
                            PacketBegin();
                            HokuyoPringHeading();
                            PacketEnd(true);
                            break;
                        case 'c': // Clear deviation, variation and hard iron calibration EEPROM values to zero
                            PacketBegin();
                            HokuyoCompassEEPROMClear();
                            PacketEnd(true);
                            break;
                    }
                    break;
                case 'P':    // PicoC configuration commands
                    switch (getch()) 
                    {
                        case '?':   // Report number of PicoC programs running right now
                            PacketBegin();
                            if (PicoCRunning)
                            {
                                // At least one PicoC program is running
                                printf("#$P?1");
                            }
                            else
                            {
                                // None are running right now
                                printf("#$P?0");
                            }
                            PacketEnd(true);
                            break;
                            
                        case '1':    // Enable PicoC to parse serial input while running for firmware commands
                            AllowPicoCBackground = true;
                            PacketBegin();
                            printf("#$P1");
                            PacketEnd(true);
                            break;
                        case '2':    // Disable PicoC background mode
                            AllowPicoCBackground = false;
                            PacketBegin();
                            printf("#$P2");
                            PacketEnd(true);
                            break;
                        case '3':    // Enable/disable PicoC printf() buffering
                            PacketBegin();
                            PicoCSetPrintfBuffer();
                            PacketEnd(true);
                            break;
                    }
                    break;
                case 'x':   // XModem receive into alternate buffer
                    xmodem_receive((unsigned char *)FLASH_BUFFER2, FLASH_BUFFER2_SIZE);
                    break;
                case 'v':    // Alternate video setup commands
                    PacketBegin();
                    switch (getch()) {
                        case '1':    // Turns on streaming video
                            SetOption(OPT_STREAM_VIDEO_ON, true);
                            printf("#$v1");
                            break;
                        case '2':    // Turns off streaming video
                            SetOption(OPT_STREAM_VIDEO_ON, false);
                            printf("#$v2");
                            break;
                    }
                    PacketEnd(true);
                    break;
                case 't':    // Test commands, of all sorts
                    switch (getch())
                    {
                        uint32_t yy, ii;
                        uint8_t ch;
                        
                        case '1':
                            ch = '0';
                            for (yy = 0; yy < 100; yy++)
                            {
                                // Test the UART send function
                                for (ii=0; ii < 50; ii++)
                                {

                                    uart0SendChar((char)ch);
                                    ch++;
                                    if (ch == ':')
                                        ch = '0';
                                }
                                uart0SendChar('\n');
                                uart0SendChar('\r');
                            }
                            break;
                        case '2': // Wait 10 seconds, then send 10MB 
                            delayMS(10000);
                            
                            ch = 0;
                            for (ii = 0; ii < 10000000; ii++)
                            {
                                uart0SendChar(ch);
                                ch++;
                            }
                            break;
                        case 'b': // Send back current UART buffer fullness
                            {
                                uint32_t RXBuf1, RXBuf2, RXBuf3, TXBuf1, TXBuf2, TXBuf3;
                                ReadBufferStates(&RXBuf1, &RXBuf2, &RXBuf3, &TXBuf1, &TXBuf2, &TXBuf3);
                                PacketBegin();
                                printf("##tb,%d,%d,%d,%d,%d,%d\r\n", (int)RXBuf1, (int)RXBuf2, (int)RXBuf3, (int)TXBuf1, (int)TXBuf2, (int)TXBuf3);
                                PacketEnd(true);
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                case 'O':    // Set system-wide options
                    switch (getch())
                    {
                        case 'i':    // Stuff that has to do with I2C
                            switch (getch())
                            {
                                default:
                                    break;
                            }
                            break;
                        
                        case 'p':    // Stuff that has to do with packet mode
                            switch (getch())
                            {
                                case '1': // Turn packet mode on
                                    PacketBegin();
                                    printf("#$Op1");
                                    PacketEnd(true);
                                    SetOption(OPT_PACKET_MODE, true);
                                    break;
                                    
                                case '2': // Turn packet mode off
                                    PacketBegin();    
                                    printf("#$Op2");
                                    PacketEnd(true);
                                    SetOption(OPT_PACKET_MODE, false);
                                    break;

                                default:
                                    break;
                            }
                            break;
                        
                        case 'v':    // Stuff that has to do with video options
                            PacketBegin();
                            switch (getch())
                            {
                                uint32_t Temp = 0;
                                
                                case '1': // Turn test mode 1 on
                                    SetOption(OPT_VIDEO_TEST_MODE_1, true);
                                    printf("#$Ov1");
                                    break;
                                    
                                case '2': // Turn test mode 1 off
                                    SetOption(OPT_VIDEO_TEST_MODE_1, false);
                                    printf("#$Ov2");
                                    break;
                                    
                                case '3': // Set delay between frames for video streaming mode
                                    Temp = getch();
                                    Temp = Temp + (getch() << 8);
                                    Temp = Temp + (getch() << 16);
                                    Temp = Temp + (getch() << 24);
                                    TimeBetweenStreamingFramesMS = Temp;
                                    printf("#$Ov3");
                                    break;
                                    
                                default:
                                    break;
                            }
                            PacketEnd(true);
                            break;
                        
                        case 'K':   // Control the streaming shutdown command
                            PacketBegin();
                            StreamingShutdown();
                            PacketEnd(true);
                            break;
                            
                        default:
                            break;
                    
                    }
                    break;
                        break;

                default:
                    break;
            }
            break;
        case '%': // jump to user defined functions in myfunc.c
            PacketBegin();
            myfunc();
            PacketEnd(true);
            break;
        case 'E': // launch flash buffer editor
            PacketBegin();
            EditorRunning = true;
            launch_editor();
            EditorRunning = false;
            PacketEnd(true);
            break;
        case 't':
            PacketBegin();
            serial_out_time();
            PacketEnd(true);
            break;
        case 'T':
            PacketBegin();
            set_edge_thresh();
            PacketEnd(true);
            break;
        case 'z':  // read, write or dump flash sectors
            PacketBegin();
            switch (getch()) {  
                        // zAxx - read double sector xx, xx+1 to buffer
                        // zBxx - write buffer to double sector xx, xx+1
                        // zr   - read sector 4 to buffer
                        // zRxx - read sector xx to buffer
                        // zw   - write buffer to sector 4
                        // zWxx - write buffer to sector xx
                        // zZ   - write buffer to boot flash sectors
                        // zd   - display contents of flash buffer
                        // zh   - display contents of http buffer
                        // zc   - clear buffer
                        // zC   - compute checksum of buffer contents 
                case 'A':
                      // read double user sector 00 - 63  from flash buffer
                    ix = ((getch() & 0x0F) * 10) + (getch() & 0x0F);
                    read_double_sector(ix, 0);
                    break;
                case 'B':
                      // write double user sector 00 - 63  from flash buffer
                    ix = ((getch() & 0x0F) * 10) + (getch() & 0x0F);
                    write_double_sector(ix, 0);
                    break;
                case 'r':
                    // read user flash sector into flash buffer
                    read_user_flash();
                    break;
                case 'R':
                      // read user sector 00 - 63  to flash buffer
                    ix = ((getch() & 0x0F) * 10) + (getch() & 0x0F);
                    read_user_sector(ix);
                    break;
                case 'w':
                      // write user flash sector from flash buffer
                    write_user_flash();
                    break;
                case 'W':
                      // write user sector 00 - 63  from flash buffer
                    ix = ((getch() & 0x0F) * 10) + (getch() & 0x0F);
                    write_user_sector(ix);
                    break;
                case 'Z':
                      // write boot flash sectors (1-2) from flash buffer
                    write_boot_flash();
                    break;
                case 'd':
                      // dump flash buffer contents to console
                    serial_out_flashbuffer();
                    break;
                case 'h':
                      // dump HTTP buffer contents to console
                    serial_out_httpbuffer();
                    break;
                case 'c':
                      // clear flash buffer
                    clear_flash_buffer();
                    break;
                case 'C':
                      // compute flash buffer checksum
                    crc_flash_buffer();
                    break;
            }
            PacketEnd(true);
            break;
        case 'p':   // ping sonar - supports up to 4 transducers
            PacketBegin();
            ping_sonar();
            PacketEnd(true);
            break;
        case 'S':   // grab 2 character servo 2/3 command string (left, right)
            PacketBegin();
            ppm1_command();
            PacketEnd(true);
            break;
        case 's':   // grab 2 character servo 6/7 command string (left, right)
            PacketBegin();
            ppm2_command();
            PacketEnd(true);
            break;
        case 'M':   // grab 3 character motor command string (left, right, delay)
            PacketBegin();
            motor_command();
            PacketEnd(true);
            break;
        case 'm':   // use 2nd set of timers for PWM
            PacketBegin();
            motor2_command();
            PacketEnd(true);
            break;
        case 'x':
            if (xwd_init == 0) {
                xwd_init = 1;
                init_uart1(115200);
                delayMS(10);
            }
            PacketBegin();
            printf("#x");
            uart1SendChar('x');
            uart1SendChar(getch());
            uart1SendChar(getch());
            while (uart1GetChar(&TestChar))  // flush the receive buffer
                continue;
            PacketEnd(true);
            break;
        case '+':   // increase base motor speed
            PacketBegin();
            motor_increase_base_speed();
            PacketEnd(true);
            break;
        case '-':   // decrease base motor speed
            PacketBegin();
            motor_decrease_base_speed();
            PacketEnd(true);
            break;
        case '<':   // bias motor drive to left
            PacketBegin();
            motor_trim_left();
            PacketEnd(true);
            break;
        case '>':   // bias motor drive to right
            PacketBegin();
            motor_trim_right();
            PacketEnd(true);
            break;
        case '7':   // drift left
        case '8':   // forward
        case '9':   // drift right
        case '4':   // turn left
        case '5':   // stop
        case '6':   // turn right
        case '1':   // back left
        case '2':   // back
        case '3':   // back right
        case '0':   // counter clockwise turn
        case '.':   // clockwise turn
            PacketBegin();
            motor_action(CharIn);
            PacketEnd(true);
            break;
        case 'l':   // lasers on
            PacketBegin();
            lasers_on();
            PacketEnd(true);
            break; 
        case 'L':   // lasers off
            PacketBegin();
            lasers_off();
            PacketEnd(true);
            break; 
        case 'v':   // vision commands
            PacketBegin();
            process_colors();
            PacketEnd(true);
            break;
        case 'n':   // neural net commands
            PacketBegin();
            process_neuralnet();
            PacketEnd(true);
            break;
        case 'F':   // set failsafe mode
            PacketBegin();
            enable_failsafe();
            PacketEnd(true);
            break;
        case 'f':   // clear failsafe mode
            PacketBegin();
            disable_failsafe();
            PacketEnd(true);
            break;
        case 'g':   // g0 = grab a reference frame and enable differencing
                    // g1 = enable color segmentation
                    // g2 = enable edge detection
                    // g3 = enable horizon detection
                    // g4 = enable obstacle detection
                    // g5 = enable stereo processing (SVS only)
                    // g6 = enable blob display
                    // g7 = enable QR code detection
                    // g_ = anything else turns them all off
            PacketBegin();
            switch (getch()) {
                case '0':
                    enable_frame_diff();
                    break;
                case '1':
                    enable_segmentation();
                    break;
                case '2':
                    enable_edge_detect();
                    break;
                case '3':
                    enable_horizon_detect();
                    break;
                case '4':
                    enable_obstacle_detect();
                    break;
                #ifdef STEREO
                case '5':
                    enable_stereo_processing();
                    break;
                #endif /* STEREO */
                case '6':
                    enable_blob_display();
                    break;
                case '7':
                    enable_qr_code_detection();
                    break;
                default:  // no match - turn them all off
                    disable_frame_diff();
                    break;
            }
            PacketEnd(true);
            break;
        case 'i':   // i2c read / write
            process_i2c();
            break;
        case '*':   // Process all PicoC shared memory functions
            processPicoCMemory();
            break;

        case 'H':   // Fire up the Hokuyo laser range finder on UART1, grab a single set of distance data
                    // and send it back to the PC. (in ASCII)
            PacketBegin();
            processHokuyo(HOKUYO_PRINT_ASCII);
            PacketEnd(true);
            break;
        case 'h':    // Fire up the Hokuyo laser range finder on UART1, grab a single set of distance data
                    // and send it back to the PC. (in binary)
            PacketBegin();
            processHokuyo(HOKUYO_PRINT_BINARY);
            PacketEnd(true);
            break;
        case '/':    // Test command - you send it as much data as you want, anything at all
                    // that ends with an ESC character, and it will echo back exactly what you've sent it.
            PacketBegin();
            ProcessPipeTest();
            PacketEnd(true);
            break;
        case 'N':    // Setup streaming sensor packets back to PC command
            PacketBegin();
            switch (getch()) {
                case 'D':    // For single bit digital sensors - normal I2C
                    StreamingParseDigitalBit(I2C_NORMAL);
                    break;
                case 'd':    // For single bit digital sensors - repeated start I2C
                    StreamingParseDigitalBit(I2C_REPEATED_START);
                    break;
                case 'B':    // For byte (single or multiple) digital sensors - normal I2C, no ID word
                    StreamingParseDigitalByte(I2C_NORMAL, ID_NONE);
                    break;
                case 'b':    // For byte (single or multiple) digital sensors - repeated start I2C, no ID word
                    StreamingParseDigitalByte(I2C_REPEATED_START, ID_NONE);
                    break;
                case '$':    // For commands that use the ID word
                    switch (getch()) {
                        case 'B':    // For byte (single or multiple) digital sensors - normal I2C, no ID word
                            StreamingParseDigitalByte(I2C_NORMAL, ID_PRESENT);
                            break;
                        case 'b':    // For byte (single or multiple) digital sensors - repeated start I2C, no ID word
                            StreamingParseDigitalByte(I2C_REPEATED_START, ID_PRESENT);
                            break;
                        default:
                            break;
                    }
                    break;
                case 'A':    // For analog sensors
                    StreamingParseAnalogBit();
                    break;
                case 'H':    // For Hokuyo
                    StreamingParseHokuyo();
                    break;
                case 'P':    // For PicoC shared memory streaming
                    StreamingParsePicoC();
                    break;
                default:
                    break;
            }
            PacketEnd(true);
            break;
            
        case 'Z':   // Force packet mode off (we respond to this command in packet mode and we ignore it when not in packet mode)
        case 0x1E:  // This value is pushed into the command queue from within the PacketNewByteRX() state machine to indicate a 'Z' was
                    // seen at the beginning of a packet.
                    // Note that packet mode turns off BEFORE the response is sent (unlike #$Op2)
            SetOption(OPT_PACKET_MODE, false);
            PacketBegin();
            printf("##Z pkt mode off\r\n");
            PacketEnd(true);
            break;
            
        case 0x0D:  // <CR> Carriage Return
            // We just print out a "#" prompt so somebody at a terminal emulator knows we're alive
            PacketBegin();
            printf("#");
            PacketEnd(true);
            break;
            
        case 0x0A:  // <LF> Line Feed
            // We just silently ignore these
            break;
            
        default:
            PacketBegin();
            PrintUnknownCommand(CharIn);
            printf("%c", CharIn);
            PacketEnd(true);
            break;
    }
}
