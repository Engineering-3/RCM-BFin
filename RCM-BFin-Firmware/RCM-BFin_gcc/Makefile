#/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# *  Makefile - Makefile for the RCM-Pi robot firmware
# *
# *  Part of the RCM-Pi GCC v2.0 firmware for the RCM-Pi robot
# *    
# *  Copyright (C) 2015-2020  Engineering^3
# *
# *  Based on the original Surveyor firmware, written and copyrighted
# *  by Surveyor Corp. 
# *
# *  This program is free software; you can redistribute it and/or modify
# *  it under the terms of the GNU General Public License as published by
# *  the Free Software Foundation; either version 2 of the License, or
# *  (at your option) any later version.
# *
# *  This program is distributed in the hope that it will be useful,
# *  but WITHOUT ANY WARRANTY; without even the implied warranty of
# *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# *  GNU General Public License for more details (www.gnu.org/licenses)
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
# *
# * Original version was for Blackfin processor, and
# * used Blackfin GCC toolchain, 2011R1 version
# * http://sourceforge.net/projects/adi-toolchain/files/2011R1/2011R1-RC4/GCC4.3/blackfin-toolchain-win32-2011R1.exe/download
# *
# * New version is running on Raspberry Pi, so build is done natively on the Pi using gcc.
# */

# OBJS = init.o rcm-bfin.o intuart.o camera.o jpeg.o i2c.o stm_m25p32.o httpd.o \
#  xmodem.o colors.o edit.o myfunc.o neural.o gps.o sdcard.o hokuyo.o \
#  picoc/picoc.o picoc/table.o picoc/lex.o picoc/parse.o picoc/expression.o \
#  picoc/heap.o picoc/type.o picoc/variable.o picoc/clibrary.o picoc/debug.o \
#  picoc/platform/library_surveyor.o picoc/platform.o picoc/platform/platform_surveyor.o \
#  picoc/include.o interrupts.o sysinit.o system.o \
#  r8x8dct.o motionvect.o cache.o basiccrt.o options.o streaming.o 
SOURCES = main.c packet.c
OBJECTS = $(SOURCES:%.c=%.o)

TARGET=rcm-pi

$(TARGET): $(OBJECTS)
	gcc $^ -o $@

%.o: %.c
	gcc -c $< -o $@
  
# I know this is a stupid way to do it (allow operation on both 64 and 32 bit Windows 
# systems)  but I couldn't come up with a better way. If you know of a more slick way to
# do this, and still allow multiple versions of the compiler on the machine at once,
# please let me know at brian@schmalzhaus.com
#CROSS_COMPILE = "/cygdrive/c/Program Files (x86)/Analog Devices/GNU Toolchain/2011R1/elf"
#CROSS_COMPILE = "/cygdrive/c/Program Files/Analog Devices/GNU Toolchain/2011R1/elf"
#AS = $(CROSS_COMPILE)/bin/bfin-elf-as
#CC = $(CROSS_COMPILE)/bin/bfin-elf-gcc
#LD = $(CROSS_COMPILE)/bin/bfin-elf-ld.real
#LDR = $(CROSS_COMPILE)/bin/bfin-elf-ldr

#CPUDEFINES = -D__ADSPBF537__ -D__ADSPLPBLACKFIN__ -DSURVEYOR_HOST
#ASMFLAGS = -x assembler-with-cpp $(CPUDEFINES) $(INCLUDES)

#CPU = bf537
#CFLAGS += $(INCLUDES) -Wall -mcpu=$(CPU) -ffunction-sections -O2 $(CPUDEFINES)
#CFLAGS += $(INCLUDES) -Wall -mcpu=$(CPU) -O2 $(CPUDEFINES) 
#LDFLAGS += $(INCLUDES) -mcpu=$(CPU) -T $(TARGET).x -Wl,--gc-sections -Xlinker
#LDLIBS += -lm
#LDRFLAGS += --initcode init.o

#all:  $(TARGET).bin $(TARGET).ldr $(TARGET)_115k.bin $(TARGET)_115k.ldr
#all:  $(TARGET).bin $(TARGET).ldr Makefile

#%.o: %.S
#	$(CC) $(ASMFLAGS) -c -o $@ $<

#$(TARGET).bin: main.o $(OBJS)
#	$(CC) $(CFLAGS) $(LDFLAGS) -Map=$(TARGET).map -o $@ $^ $(LDLIBS)

#$(TARGET)_115k.bin: $(OBJS) main_115k.o
#	$(CC) $(CFLAGS) $(LDFLAGS) -Map=$(TARGET)_115k.map -o $@ $^ $(LDLIBS)

#$(TARGET).ldr: $(TARGET).bin init.o $(TARGET).x
#	$(LDR) -f -T $(CPU) -c $(TARGET).ldr $(LDRFLAGS) $(TARGET).bin

#$(TARGET)_115k.ldr: $(TARGET)_115k.bin init.o $(TARGET).x
#	$(LDR) -f -T $(CPU) -c $(TARGET)_115k.ldr $(LDRFLAGS) $(TARGET)_115k.bin

.PHONY : clean
clean:
	rm -f *.o *.s picoc/*.o picoc/platform/*.o