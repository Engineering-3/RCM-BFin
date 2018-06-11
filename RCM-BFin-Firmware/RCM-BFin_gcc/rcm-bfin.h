/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  rcm-bfin.h -
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

#ifndef RCMBFIN_H
#define RCMBINF_H
#include "config.h"

#define SSYNC asm("ssync;")


/* general macros */

#define countof(a)      (sizeof(a)/sizeof(a[0]))

#define PWM_OFF 0
#define PWM_PWM 1
#define PWM_PPM 2
#define PWM_UART 3

#define FOUR_TWO_ZERO 1
#define FOUR_TWO_TWO  2

/* 
 * SPI Flash allocation
 * 
 * Flash is a 32Mbit (4MB) flash M25P32-VMF6P from 0x00000000 to 
 * 0x00400000 (SPI chip addresses -  not in processor's memory map)
 * Uses sectors that are 0xFFFF in size
 * 64 total sectors - first 4 are reserved for boot image (code)
 * Note that this chip can use a clock speed of 40MHz to 50Mhz 
 */
#define BOOT_FLASH          0x00000000  // address in SPI flash of boot image
#define BOOT_FLASH_SIZE     0x00040000  // 256KB - four sectors worth
#define USER_FLASH          0x00040000  // address in SPI flash of user flash sector
#define USER_FLASH_SIZE 	0x003C0000 	// 3840KB - entire rest of Flash chip
#define USER_FLASH_SECTOR_BEGIN 4		// first sector for users to use for their stuff
#define USER_FLASH_SECTOR_END 63		// Last sector users can use for their stuff

#define ONE_SECTOR			0x00010000	// One sector's size in bytes
#define TWO_SECTORS			(ONE_SECTOR * 2)
/* 
 * SDRAM allocation
 * 
 * SDRAM is 32MB in size, from 0x00000000 to 0x02000000
 * First part of SDRAM is where all of the code in Flash gets copied 
 *  to on boot (by BF boot ROM), and executes from there.
 */
 // This first 1MB is instruction cached
#define SDRAM_CODE          0x00000000  // Starts at zero
#define SDRAM_CODE_SIZE     0x00100000  // 1MB in size (NOTE: this is SMALLER than the Flash size, but much larger than the boot sector)

// Everything after this point is considered SDRAM data (not instructions)
// this is important for setting up the CPLBs in cache.c and the linker
// script needs to know this too. SDRAM_DATA is used by the linker for C variables and such.
// This next 15MB is data cached
#define SDRAM_DATA			0x00100000	// Holds data memory for code running in SDRAM_CODE section (cached)
#define SDRAM_DATA_SIZE		0x00100000	// 1MB in size

// From here on to the end of SDRAM is all managed by the RCM-Bfin firmware - NOT the compiler
#define HEAPSTART           0x00200000  // For malloc and such - heap grows UP from here
#define HEAPSIZE            0x00100000  // 1MB for now - leave 1MB for JPEG buffer
#define SDRAM_CLEAR_START	0x00300000	// Address to start SDRAM clear operation
#define FLASH_BUFFER        0x00300000	// address in SDRAM for buffering flash and xmodem  (primary PicoC program storage)
#define FLASH_BUFFER_SIZE   0x00080000  // 512KB in size
#define FLASH_BUFFER2       0x00380000  // address in SDRAM for buffering flash and xmodem  (second PicoC program storage)
#define FLASH_BUFFER2_SIZE  0x00080000  // 512KB in size
#define C_HEAPSTART         0x00400000  // Buffer for picoC
#define C_HEAPSIZE          0x00080000  // 512KB in size
#define SPI_BUFFER1         0x00480000  // Buffer for transfer of data via SPI bus
#define SPI_BUFFER1_SIZE	0x00040000	// 256KB
#define SPI_BUFFER2         0x004C0000  // Buffer for transfer of data via SPI bus
#define SPI_BUFFER2_SIZE	0x00040000	// 256KB
#define HTTP_BUFFER         0x00500000  // Buffer also used for receiving and sending HTTP messages
#define HTTP_BUFFER_SIZE    0x00040000	// 256KB
#define HTTP_BUFFER2        0x00540000  // additional 256kB buffer for HTTP content
#define HTTP_BUFFER2_SIZE   0x00040000	// 256KB
#define EMPTY_RAM           0x00580000  // Not used yet
#define EMPTY_RAM_SIZE      0x00280000  // 2.5MB for now
#define PICOC1_BUF          0x00800000  // C_HEAPSTART, PicoC Data section,  goes here when 2nd PicoC program runs
#define PICOC1_BUF_HEAP     PICOC1_BUF
#define PICOC1_BUF_HEAP_SZ  C_HEAPSIZE
#define PICOC1_BUF_DATA     (PICOC1_BUF + C_HEAPSIZE)
#define PICOC1_BUF_DATA_SZ  0x00080000
// The last 16MB is uncached
#define DMA_BUF1            0x01000000  // address in SDRAM for DMA transfer of frames from camera
#define DMA_BUF2            0x01280000  //   second DMA buffer for double buffering
#define FRAME_BUF           0x01500000  // address in SDRAM for staging images for processing/jpeg
#define FRAME_BUF2          0x01780000  //   second frame buffer for storing reference frame
#define FRAME_BUF3          0x01A00000  //   third frame buffer for edge data or YUV planar data
#define FRAME_BUF4          0x01C80000  //   fourth frame buffer 

#define JPEG_BUF            0x00F00000  // address in SDRAM for JPEG compressed image
#define DISP_BUF            0x00F00000  // buffer used to send disparity data
#define SDRAM_END			0x02000000	// End of SDRAM chip

/* Stack info (this is on-chip scratchpad RAM in the BF) 4KB*/
#define STACK_TOP    0xFFB01000
#define STACK_BOTTOM 0xFFB00000

/* PicoC shared array variable constants */
#define PICOC_SHARED_ARRAY_SIZE             256
#define PICOC_SHARED_ARRAY_STRING_LENGTH    256

int CopyPicoCOut(void);
void CopyPicoCBack(void);

/* Misc Init */
void init_io ();
void clear_sdram ();
void launch_editor ();
void init_heap ();
void show_stack_ptr ();
void show_heap_ptr ();
void reset_cpu ();
unsigned int stack_remaining();
unsigned int ctoi(unsigned char);
void check_for_autorun();
unsigned int CheckForRCMBFINCommand(void);

void led0_off();
void led1_off();
void led0_on();
void led1_on();
void led0_toggle();
void led1_toggle();
void ProcessPipeTest(void);

/* Serial outputs */
void serial_out_version ();
void serial_out_version_no_packet(void);
void serial_out_time ();
void serial_out_flashbuffer ();
void serial_out_httpbuffer ();

/* I2C */
void process_i2c();
void process_autoinc_i2c(void);

/* Analog */
void init_analog();
unsigned int analog(unsigned int);
void read_analog();
void read_analog_4wd();
unsigned int analog_4wd(unsigned int);

/* Tilt */
void init_tilt();
unsigned int tilt(unsigned int);
void read_tilt();

/* Compass */
void show_compass2x();
void show_compass3x();
short read_compass3x(short *, short *, short *);

/* Lasers */
void lasers_on ();
void lasers_off ();
void show_laser_range();
unsigned int laser_range(int);

/* Sonar */
void init_sonar();
void ping_sonar();
void sonar();

/* LED's */
void led0_on();
void led1_on();

/* Camera */
void grab_frame ();
void send_frame ();
void grab_reference_frame ();
void compute_frame_diff (unsigned char *, unsigned char *, int, int);
void enable_frame_diff ();
void enable_segmentation();
void enable_edge_detect();
void enable_horizon_detect();
void enable_obstacle_detect();
void enable_stereo_processing();
void enable_blob_display();
unsigned int check_stereo_sync();
void set_edge_thresh();
void grab_code_send();
void recv_grab_code();
void disable_frame_diff ();
void overlay_on ();
void overlay_off ();
void camera_setup ();
void camera_reset (unsigned int width);
void change_image_quality ();
void set_caption (unsigned char *str, unsigned int width);
void move_image (unsigned char *src1, unsigned char *src2, unsigned char *dst, unsigned int width, unsigned int height);
void move_inverted (unsigned char *src1, unsigned char *src2, unsigned char *dst, unsigned int width, unsigned int height);
void copy_image (unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height);
void move_yuv422_to_planar (unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height);
void send_80x64planar();
void scale_image_to_80x64_planar (unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height);
void invert_video(), restore_video();

/* Image Processing */
void process_colors(), init_colors();
void motion_vect_test();
void motion_vect80x64();
void process_neuralnet();

/* Failsafe */
void enable_failsafe();
void disable_failsafe();
void check_failsafe();
void reset_failsafe_clock();
void check_battery();

/* Transfer */
void xmodem_receive (unsigned char * BufferPtr, unsigned int BufferSize);
void ReadPicoCProgram(char * BufferPtr, unsigned int BufferSize);
                    
/* Flash */
void read_user_flash ();
void read_user_sector (int);
void read_double_sector (int, int);
void write_user_flash ();
void write_user_sector (int);
void write_double_sector (int, int);
void write_boot_flash ();
void clear_flash_buffer ();
void crc_flash_buffer ();
void testSD();

/* Motors */
void init_motors ();
void init_servos();
void init_encoders();
void read_encoders();
unsigned int encoders();
void read_encoder_4wd();
unsigned int encoder_4wd(unsigned int);
void update_servos();
void motor_command(), motor2_command();
void motor_increase_base_speed ();
void motor_decrease_base_speed ();
void motor_trim_left ();
void motor_trim_right ();
void motor_action (unsigned char ch);
void motor_set (unsigned char cc, int speed, int *ls, int *rs);
void ppm1_command ();
void ppm2_command ();
void initPWM ();
void initPWM2 ();
void initPPM1 ();
void initPPM2 ();
void setPWM (int mleft, int mright);
void setPWM2 (int mleft, int mright);
void setPPM1 (int mleft, int mright);
void setPPM2 (int mleft, int mright);
int check_bounds_0_100(int ix);

/* Misc */
//unsigned int rand();
unsigned int intfromthreechars();
unsigned int intfromfourchars();
void PrintUnknownCommand(char);

/* PicoC related commands */
void processPicoCMemory();
void PicoCSetPrintfBuffer(void);

/* Compass */
extern short cxmin, cxmax, cymin, cymax;
extern int compass_continuous_calibration, compass_init;

/* Clock */
void initRTC ();
int readRTC ();
void clearRTC ();
void initTMR4 ();
void delayMS (int delay);  // delay up to 100000 millisecs (100 secs)
void delayUS (int delay);  // delay up to 100000 microseconds (.1 sec)
void delayNS (int delay);  // delay up to 100000 nanoseconds (.0001 sec)

/* Globals */
extern int pwm1_mode, pwm2_mode, pwm1_init, pwm2_init, xwd_init, tilt_init, analog_init;
extern int lspeed, rspeed, lspeed2, rspeed2, base_speed, base_speed2, lcount, rcount;
extern int move_start_time, move_stop_time, move_time_mS, robot_moving;
extern int sonar_data[];
extern unsigned int imgWidth, imgHeight, frame_diff_flag, horizon_detect_flag, invert_flag, quality;
extern unsigned int uart1_flag, thumbnail_flag;
extern unsigned int segmentation_flag, edge_detect_flag, frame_diff_flag, horizon_detect_flag;
extern unsigned int obstacle_detect_flag;
extern unsigned int blob_display_flag;
extern unsigned int blob_display_num;
extern unsigned int edge_thresh;
extern unsigned int master;  // SVS master or slave ?
extern unsigned int stereo_sync_flag;
extern unsigned int stereo_processing_flag;
extern int svs_sensor_width_mmx100;
extern int svs_right_turn_percent;
extern int svs_turn_tollerance_percent;
extern int svs_calibration_offset_x, svs_calibration_offset_y;
extern int svs_centre_of_disortion_x, svs_centre_of_disortion_x;
extern int svs_scale_num, svs_scale_denom, svs_coeff_degree;
extern long* svs_coeff;
extern int svs_width, svs_height;
extern int svs_enable_horizontal;
extern int svs_ground_y_percent;
extern int svs_ground_slope_percent;
extern int svs_enable_ground_priors;
extern int svs_enable_mapping;
extern int svs_disp_left, svs_disp_right, svs_steer;
extern unsigned char version_string[];
extern unsigned int PicoCRunning;
extern unsigned int PicoCGoInteractive;
extern signed int pico_int[PICOC_SHARED_ARRAY_SIZE];
extern char pico_char[PICOC_SHARED_ARRAY_SIZE][PICOC_SHARED_ARRAY_STRING_LENGTH];
extern double pico_float[PICOC_SHARED_ARRAY_SIZE];
extern unsigned int PicoCStreamBufferIn;
extern unsigned int PicoCStreamBufferOut;
extern unsigned int PicoCStreamBufferLength;
extern unsigned char PicoCStreamBuffer[PICOC_STREAM_BUFFER_SIZE];
extern unsigned char PicoCStreamBufferEnabled;

#endif

