/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  bfin-common-sc-rcm-bfin.ld - linker script for RCM-Bfin GCG v2.0 firmware
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
 * The common part of the default linker scripts for standalone executables
 * running on single core Blackfin processors.
 *
 * Copyright (C) 2007, 2008 Analog Devices, Inc.
 *
 * The authors hereby grant permission to use, copy, modify, distribute,
 * and license this software and its documentation for any purpose, provided
 * that existing copyright notices are retained in all copies and that this
 * notice is included verbatim in any distributions. No written agreement,
 * license, or royalty fee is required for any of the authorized uses.
 * Modifications to this software may be copyrighted by their authors
 * and need not follow the licensing terms described here, provided that
 * the new terms are clearly indicated on the first page of each file where
 * they apply.
 */

/* The default linker script, for single core blackfin standalone executables */
OUTPUT_FORMAT("elf32-bfin", "elf32-bfin",
	      "elf32-bfin")
OUTPUT_ARCH(bfin)
INCLUDE bf537.ld
MEMORY
{
	MEM_SDRAM       : ORIGIN = 0x00000000, LENGTH = 32 * 1024 * 1024
}

ENTRY(__start)

SECTIONS
{
  /* Read-only sections, merged into text segment: */
  PROVIDE (__executable_start = 0x0); . = 0x0;
  .interp         : { *(.interp) }
  .hash           : { *(.hash) }
  .dynsym         : { *(.dynsym) }
  .dynstr         : { *(.dynstr) }
  .gnu.version    : { *(.gnu.version) }
  .gnu.version_d  : { *(.gnu.version_d) }
  .gnu.version_r  : { *(.gnu.version_r) }
  .rel.init       : { *(.rel.init) }
  .rela.init      : { *(.rela.init) }
  .rel.text       : { *(.rel.text .rel.text.* .rel.gnu.linkonce.t.*) }
  .rela.text      : { *(.rela.text .rela.text.* .rela.gnu.linkonce.t.*) }
  .rel.fini       : { *(.rel.fini) }
  .rela.fini      : { *(.rela.fini) }
  .rel.rodata     : { *(.rel.rodata .rel.rodata.* .rel.gnu.linkonce.r.*) }
  .rela.rodata    : { *(.rela.rodata .rela.rodata.* .rela.gnu.linkonce.r.*) }
  .rel.data.rel.ro   : { *(.rel.data.rel.ro* .rel.gnu.linkonce.d.rel.ro.*) }
  .rela.data.rel.ro   : { *(.rela.data.rel.ro* .rela.gnu.linkonce.d.rel.ro.*) }
  .rel.data       : { *(.rel.data .rel.data.* .rel.gnu.linkonce.d.*) }
  .rela.data      : { *(.rela.data .rela.data.* .rela.gnu.linkonce.d.*) }
  .rel.tdata	  : { *(.rel.tdata .rel.tdata.* .rel.gnu.linkonce.td.*) }
  .rela.tdata	  : { *(.rela.tdata .rela.tdata.* .rela.gnu.linkonce.td.*) }
  .rel.tbss	  : { *(.rel.tbss .rel.tbss.* .rel.gnu.linkonce.tb.*) }
  .rela.tbss	  : { *(.rela.tbss .rela.tbss.* .rela.gnu.linkonce.tb.*) }
  .rel.ctors      : { *(.rel.ctors) }
  .rela.ctors     : { *(.rela.ctors) }
  .rel.dtors      : { *(.rel.dtors) }
  .rela.dtors     : { *(.rela.dtors) }
  .rel.got        : { *(.rel.got) }
  .rela.got       : { *(.rela.got) }
  .rel.sdata      : { *(.rel.sdata .rel.sdata.* .rel.gnu.linkonce.s.*) }
  .rela.sdata     : { *(.rela.sdata .rela.sdata.* .rela.gnu.linkonce.s.*) }
  .rel.sbss       : { *(.rel.sbss .rel.sbss.* .rel.gnu.linkonce.sb.*) }
  .rela.sbss      : { *(.rela.sbss .rela.sbss.* .rela.gnu.linkonce.sb.*) }
  .rel.sdata2     : { *(.rel.sdata2 .rel.sdata2.* .rel.gnu.linkonce.s2.*) }
  .rela.sdata2    : { *(.rela.sdata2 .rela.sdata2.* .rela.gnu.linkonce.s2.*) }
  .rel.sbss2      : { *(.rel.sbss2 .rel.sbss2.* .rel.gnu.linkonce.sb2.*) }
  .rela.sbss2     : { *(.rela.sbss2 .rela.sbss2.* .rela.gnu.linkonce.sb2.*) }
  .rel.bss        : { *(.rel.bss .rel.bss.* .rel.gnu.linkonce.b.*) }
  .rela.bss       : { *(.rela.bss .rela.bss.* .rela.gnu.linkonce.b.*) }
  .rel.plt        : { *(.rel.plt) }
  .rela.plt       : { *(.rela.plt) } >MEM_L1_CODE =0
  .text           :
  {
    *(.text .stub .text.* .gnu.linkonce.t.* .l1.text .l1.text.*)
    KEEP (*(.text.*personality*))
    /* .gnu.warning sections are handled specially by elf32.em.  */
    *(.gnu.warning)
  } >MEM_SDRAM =0
  .init           :
  {
    KEEP (*(.init))
  } >MEM_L1_CODE =0
  .plt            : { *(.plt) } >MEM_L1_CODE
  .fini           :
  {
    KEEP (*(.fini))
  } >MEM_L1_CODE =0
  PROVIDE (__etext = .);
  PROVIDE (_etext = .);
  PROVIDE (etext = .);
  .rodata         : { *(.rodata .rodata.* .gnu.linkonce.r.*) } >MEM_L1_CODE
  .rodata1        : { *(.rodata1) } >MEM_L1_CODE
  .sdata2         :
  {
    *(.sdata2 .sdata2.* .gnu.linkonce.s2.*)
  } >MEM_L1_CODE
  .sbss2          : { *(.sbss2 .sbss2.* .gnu.linkonce.sb2.*) } >MEM_L1_CODE
  .eh_frame_hdr : { *(.eh_frame_hdr) } >MEM_L1_CODE
  .eh_frame       : ONLY_IF_RO { KEEP (*(.eh_frame)) } >MEM_L1_CODE
  .gcc_except_table   : ONLY_IF_RO { *(.gcc_except_table .gcc_except_table.*) } >MEM_L1_CODE
  /* Adjust the address for the data segment.  We want to adjust up to
     the same address within the page on the next page up.  */
  . = ALIGN(0x1000) + (. & (0x1000 - 1));
  /* Exception handling  */
  .eh_frame       : ONLY_IF_RW { KEEP (*(.eh_frame)) } >MEM_L1_CODE
  .gcc_except_table   : ONLY_IF_RW { *(.gcc_except_table .gcc_except_table.*) } >MEM_L1_CODE
  /* Thread Local Storage sections  */
  .tdata	  : { *(.tdata .tdata.* .gnu.linkonce.td.*) } >MEM_L1_CODE
  .tbss		  : { *(.tbss .tbss.* .gnu.linkonce.tb.*) *(.tcommon) } >MEM_SDRAM
  .preinit_array     :
  {
    PROVIDE_HIDDEN (___preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (___preinit_array_end = .);
  } >MEM_SDRAM
  .init_array     :
  {
     PROVIDE_HIDDEN (___init_array_start = .);
     KEEP (*(SORT(.init_array.*)))
     KEEP (*(.init_array))
     PROVIDE_HIDDEN (___init_array_end = .);
  } >MEM_SDRAM
  .fini_array     :
  {
    PROVIDE_HIDDEN (___fini_array_start = .);
    KEEP (*(.fini_array))
    KEEP (*(SORT(.fini_array.*)))
    PROVIDE_HIDDEN (___fini_array_end = .);
  } >MEM_SDRAM
  .ctors          :
  {
    /* gcc uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin*.o(.ctors))
    /* We don't want to include the .ctor section from
       the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE (*crtend*.o ) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
  } >MEM_SDRAM
  .dtors          :
  {
    KEEP (*crtbegin*.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend*.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
  } >MEM_SDRAM
  .jcr            : { KEEP (*(.jcr)) } >MEM_SDRAM
  .data.rel.ro : { *(.data.rel.ro.local* .gnu.linkonce.d.rel.ro.local.*) *(.data.rel.ro* .gnu.linkonce.d.rel.ro.*) } >MEM_SDRAM
  .dynamic        : { *(.dynamic) } >MEM_SDRAM
  .data           :
  {
    *(.data .data.* .gnu.linkonce.d.* .l1.data .l1.data.*)
    KEEP (*(.gnu.linkonce.d.*personality*))
    SORT(CONSTRUCTORS)
  } >MEM_SDRAM
  .data1          : { *(.data1) } >MEM_SDRAM
  .got            : { *(.got.plt) *(.got) } >MEM_SDRAM
  /* We want the small data sections together, so single-instruction offsets
     can access them all, and initialized data all before uninitialized, so
     we can shorten the on-disk segment size.  */
  .sdata          :
  {
    *(.sdata .sdata.* .gnu.linkonce.s.*)
  } >MEM_SDRAM
  __edata = .; PROVIDE (_edata = .);
  .sbss           :
  {
    __bss_start = .;
    *(.dynsbss)
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.scommon)
  } >MEM_SDRAM
  .bss            :
  {
    *(.dynbss)
    *(.bss .bss.* .gnu.linkonce.b.*)
    *(COMMON)
    /* Align here to ensure that the .bss section occupies space up to
       _end.  Align after .bss to ensure correct alignment even if the
       .bss section disappears because there are no input sections.
       FIXME: Why do we need it? When there is no .bss section, we don't
       pad the .data section.  */
    . = ALIGN(. != 0 ? 32 / 8 : 1);
    __bss_end = .;
  } >MEM_SDRAM
  . = ALIGN(32 / 8);
  . = ALIGN(32 / 8);
  __end = .; PROVIDE (_end = .);
  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }

  __stack_end = ORIGIN(MEM_L1_SCRATCH) + LENGTH(MEM_L1_SCRATCH);
  PROVIDE (_supervisor_stack = .);

  /DISCARD/ : { *(.note.GNU-stack) }
}
