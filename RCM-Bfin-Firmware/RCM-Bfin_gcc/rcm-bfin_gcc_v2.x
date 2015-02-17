OUTPUT_FORMAT("elf32-bfin", "elf32-bfin", "elf32-bfin")
OUTPUT_ARCH(bfin)
ENTRY(_start)

MEMORY
{
	MEM_L1_DATA_A   : ORIGIN = 0xFF800000, LENGTH = 0x4000   /* 16K data memory bank A */
	MEM_L1_DCACHE_A : ORIGIN = 0xFF804000, LENGTH = 0x4000   /* 16K data cache, bank A */
	MEM_L1_DATA_B   : ORIGIN = 0xFF900000, LENGTH = 0x4000   /* 16K data memory bank B */
	MEM_L1_DCACHE_B : ORIGIN = 0xFF904000, LENGTH = 0x4000   /* 16K data cache, bank B */

	MEM_L1_CODE     : ORIGIN = 0xFFA00000, LENGTH = 0xC000   /* 48K instruction bank A and bank B together */
    MEM_L1_ICACHE   : ORIGIN = 0xFFA10000, LENGTH = 0x4000   /* 16K instruction cache */
	MEM_L1_SCRATCH  : ORIGIN = 0xFFB00000, LENGTH = 0x1000   /* 4K data scratchpad */

	MEM_SDRAM_CODE  : ORIGIN = 0x00000000, LENGTH = 1 * 1024 * 1024
	/* Note, our SDRAM chip actually has 32MB of space, but we only allow the linker
	access to the first 2MB of it. The rest is managed by the RCM-Bfin firmware. */
	MEM_SDRAM_DATA  : ORIGIN = 0x00100000, LENGTH = 1 * 1024 * 1024
	MEM_SDRAM_HEAP  : ORIGIN = 0x00200000, LENGTH = 1 * 1024 * 1024
}

SECTIONS
{
	.l1code         : { basiccrt.o(.text) *(.l1code) } > MEM_L1_CODE

    .sdram.text :
    {
        main.o(.text)
	} > MEM_SDRAM_CODE

	.sdram.bss          :
	{
		__sdram_bss_start = .;
		*(.bss.sdram)
		neural.o(.bss)
		. = ALIGN(32 / 8);
		__sdram_bss_end = .;

	} > MEM_SDRAM_DATA

	.text :
	{
		*(.text .text.*)
	} > MEM_SDRAM_CODE
    
	.init :
	{
		KEEP (*(.init))
	} > MEM_SDRAM_CODE
	.fini :
	{
		KEEP (*(.fini))
	} > MEM_SDRAM_CODE
    
    .rodata   : { 
        PROVIDE (__etext = .);
        PROVIDE (_etext = .);
        PROVIDE (etext = .);

        *(.rodata .rodata.*) 
    } > MEM_SDRAM_DATA

    .picoc_rodata   : { 
        __picoc_rodata_start = .;
        picoc*(.rodata .rodata.*) 
        __picoc_rodata_end = .;
    } > MEM_SDRAM_DATA

	.eh_frame : ONLY_IF_RO { KEEP (*(.eh_frame)) } > MEM_SDRAM_DATA

	/* Adjust the address for the data segment.  We want to adjust up to
		 the same address within the page on the next page up.  */
	. = ALIGN(0x1000) + (. & (0x1000 - 1));
	/* Exception handling  */
	.eh_frame       : ONLY_IF_RW { KEEP (*(.eh_frame)) } > MEM_SDRAM_DATA
	.gcc_except_table   : ONLY_IF_RW { *(.gcc_except_table .gcc_except_table.*) } > MEM_SDRAM_DATA
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
	} > MEM_SDRAM_DATA
	.dtors          :
	{
		KEEP (*crtbegin*.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend*.o ) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*(.dtors))
	} > MEM_SDRAM_DATA
	.jcr : { KEEP (*(.jcr)) } > MEM_SDRAM_DATA
    
    /* We separate out the PicoC data segment so that we can copy it around
    in order to have two PicoC programs running. */
    .picoc_data     : {
        __picoc_data_start = .;
        pico*(.data .data.* .gnu.linkonce.d.*) 
        SORT(CONSTRUCTORS)
        __picoc_data_end = .;
    } > MEM_SDRAM_DATA

	.data :
	{
		*(.data .data.*)
		SORT(CONSTRUCTORS)
	} > MEM_SDRAM_DATA
	__edata = .; PROVIDE (_edata = .);
    
    .picoc_bss :
    {
        __picoc_bss_start = .;
        pico*(.dynbss)
        pico*(.bss .bss.* .gnu.linkonce.b.*)
        pico*(COMMON)
        __picoc_bss_end = .;
    } > MEM_SDRAM_DATA

	.bss :
	{
		__bss_start = .;
		*(.bss)
		*(COMMON)
		/* Align here to ensure that the .bss section occupies space up to
		   _end.  Align after .bss to ensure correct alignment even if the
		   .bss section disappears because there are no input sections.
		   FIXME: Why do we need it? When there is no .bss section, we don't
		   pad the .data section.  */
		. = ALIGN(. != 0 ? 32 / 8 : 1);
		__bss_end = .;
	} > MEM_SDRAM_DATA
	. = ALIGN(32 / 8);
    /* . = 0x00480000; */
    
    .heap   : { 
        __end = .; /* Bare metal sbrk() */
        PROVIDE (_end = .);
    } > MEM_SDRAM_HEAP
    
   	/* The default initcode will use this symbol to initialize the stack */
	__stack_end = ORIGIN(MEM_L1_SCRATCH) + LENGTH(MEM_L1_SCRATCH);
    PROVIDE (_supervisor_stack = .);

	/DISCARD/ : { *(.note.GNU-stack) }
}
