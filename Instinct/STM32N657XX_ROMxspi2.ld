/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:    Instinct/STM32N657XX_AXISRAM2_fsbl.ld
 * Author:  NotBlackMagic
 * Brief:   Linker script for STM32N657XX Device from STM32N6 series for XIP mode, execute directly from HyperFlash
 */


/* Entry Point */
ENTRY(Reset_Handler)

/* Stack and heap sizes */
_Min_Heap_Size = 0x200;		/* required amount of heap */
_Min_Stack_Size = 0x1000;	/* required amount of stack */

/* Memories definition */
MEMORY
{
	ITCM		(rx)	: ORIGIN = 0x10000000, LENGTH = 256K	/* ITCM Baseline (0x10000000 - 0x1000FFFF); ITCM FLEXMEM Extension (0x10010000 - 0x1003FFFF) */
	DTCM		(rw)	: ORIGIN = 0x30000000, LENGTH = 256K	/* DTCM Baseline (0x30000000 - 0x3001FFFF); DTCM FELXMEM Extension (0x30020000 - 0x3003FFFF) */
	ROM			(xrw)	: ORIGIN = 0x70100400, LENGTH = 2048K	/* XSPI2 */
	AXISRAM1	(xrw)	: ORIGIN = 0x34064000, LENGTH = 624K	/* AXISRAM1 (0x34000000 - 0x340FFFFF); Remaining AXISRAM1 past the 400KB FLEXMEM + ECC carve-out */
	AXISRAM2	(xrw)	: ORIGIN = 0x34100000, LENGTH = 1024K	/* AXISRAM2 (0x34100000 - 0x341FFFFF) */
	AXISRAM3	(xrw)	: ORIGIN = 0x34200000, LENGTH = 448K	/* AXISRAM3 (0x34200000 - 0x3426FFFF) */
	AXISRAM4	(xrw)	: ORIGIN = 0x34270000, LENGTH = 448K	/* AXISRAM4 (0x34270000 - 0x342DFFFF) */
	AXISRAM5	(xrw)	: ORIGIN = 0x342E0000, LENGTH = 448K	/* AXISRAM5 (0x342E0000 - 0x3434FFFF) */
	AXISRAM6	(xrw)	: ORIGIN = 0x34350000, LENGTH = 448K	/* AXISRAM6 (0x34350000 - 0x343BFFFF) */
}

/* Highest address of the user mode stack */
_estack = ORIGIN(AXISRAM2) + LENGTH(AXISRAM2); /* end of "RAM" Ram type memory */
_sstack = _estack - _Min_Stack_Size;

/* Export DTCM bounds for initialization */
_sDTCM = ORIGIN(DTCM);
_eDTCM = ORIGIN(DTCM) + LENGTH(DTCM);

/* Export capacities for system shell / monitor */
_rom_capacity = LENGTH(ROM);
_ram_capacity = LENGTH(AXISRAM2);

/* Sections */
SECTIONS
{
	/* Mark the absolute start of ROM */
	_srom = ORIGIN(ROM);

	/* The startup code into "RAM" Ram type memory */
	.isr_vector :
	{
		. = ALIGN(4);
		KEEP(*(.isr_vector)) /* Startup code */
		. = ALIGN(4);
	} >ROM

	/* Critical code to execute from ITCM */
	_siitcm = LOADADDR(.itcm_text);
	.itcm_text :
	{
		. = ALIGN(4);
		_sitcm = .;
		*(.itcm_text)
		*(.itcm_text*)
		. = ALIGN(4);
		_eitcm = .;
	} >ITCM AT> ROM

	/* The program code and other data into "RAM" Ram type memory */
	.text :
	{
		. = ALIGN(4);
		_stext = .;
		*(.text)           /* .text sections (code) */
		*(.text*)          /* .text* sections (code) */
		*(.glue_7)         /* glue arm to thumb code */
		*(.glue_7t)        /* glue thumb to arm code */
		*(.eh_frame)
		*(.RamFunc)        /* .RamFunc sections */
		*(.RamFunc*)       /* .RamFunc* sections */

		KEEP (*(.init))
		KEEP (*(.fini))

		. = ALIGN(4);
		_etext = .;        /* define a global symbols at end of code */
	} >ROM

	/* Constant data into "RAM" Ram type memory */
	.rodata :
	{
		. = ALIGN(4);
		*(.rodata)         /* .rodata sections (constants, strings, etc.) */
		*(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
		. = ALIGN(4);
	} >ROM

	/* Secure Gateway stubs must reside in ROM adjacent to text */
	.gnu.sgstubs : ALIGN(32)
	{
		. = ALIGN(32);
		*(.gnu.sgstubs*)   /* Secure Gateway stubs */
		. = ALIGN(32);
		FILL(0x00);
	} >ROM

	.ARM.extab   : {
		. = ALIGN(4);
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		. = ALIGN(4);
	} >ROM

	.ARM : {
		. = ALIGN(4);
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
		. = ALIGN(4);
	} >ROM

	.preinit_array     :
	{
		. = ALIGN(4);
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
		. = ALIGN(4);
	} >ROM

	.init_array :
	{
		. = ALIGN(4);
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
		. = ALIGN(4);
	} >ROM

	.fini_array :
	{
		. = ALIGN(4);
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);
		. = ALIGN(4);
	} >ROM

	/* Used by the startup to initialize data */
	_sidata = LOADADDR(.data);

	/* Initialized data sections into "RAM" Ram type memory */
	.data :
	{
		. = ALIGN(4);
		_sdata = .;        /* create a global symbol at data start */
		*(.data)           /* .data sections */
		*(.data*)          /* .data* sections */
		. = ALIGN(4);
	} >AXISRAM2 AT> ROM

	/* Initialized TLS data section */
	.tdata : ALIGN(4)
	{
		*(.tdata .tdata.* .gnu.linkonce.td.*)
		. = ALIGN(4);
		_edata = .;        /* define a global symbol at data end */
		PROVIDE(__data_end = .);
		PROVIDE(__tdata_end = .);
	} >AXISRAM2 AT> ROM

	PROVIDE( __tdata_start = ADDR(.tdata) );
	PROVIDE( __tdata_size = __tdata_end - __tdata_start );
	PROVIDE( __data_start = ADDR(.data) );
	PROVIDE( __data_size = __data_end - __data_start );
	PROVIDE( __tdata_source = LOADADDR(.tdata) );
	PROVIDE( __tdata_source_end = LOADADDR(.tdata) + SIZEOF(.tdata) );
	PROVIDE( __tdata_source_size = __tdata_source_end - __tdata_source );
	PROVIDE( __data_source = LOADADDR(.data) );
	PROVIDE( __data_source_end = __tdata_source_end );
	PROVIDE( __data_source_size = __data_source_end - __data_source );

	/* The absolute end of the ROM payload is the Load Address (LMA) of .data plus its size */
	_erom = LOADADDR(.tdata) + SIZEOF(.tdata);

	.noncacheable :
	{
		. = ALIGN(8);
		__snoncacheable = .;/* create symbol for start of section */
		KEEP(*(.noncacheable))
		. = ALIGN(8);
		__enoncacheable = .;  /* create symbol for end of section */
	} > AXISRAM2

	/* Uninitialized data section into "RAM" Ram type memory */
	/* Uninitialized TLS data section */
	.tbss (NOLOAD) : ALIGN(4)
	{
		_sbss = .;         /* define a global symbol at bss start */
		__bss_start__ = _sbss;
		*(.tbss .tbss.*)
		. = ALIGN(4);
		PROVIDE( __tbss_end = . );
	} >AXISRAM2

	PROVIDE( __tbss_start = ADDR(.tbss) );
	PROVIDE( __tbss_size = __tbss_end - __tbss_start );
	PROVIDE( __tbss_offset = ADDR(.tbss) - ADDR(.tdata) );
	PROVIDE( __tls_base = __tdata_start );
	PROVIDE( __tls_end = __tbss_end );
	PROVIDE( __tls_size = __tls_end - __tls_base );
	PROVIDE( __tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)) );
	PROVIDE( __tls_size_align = (__tls_size + __tls_align - 1) & ~(__tls_align - 1) );
	PROVIDE( __arm32_tls_tcb_offset = MAX(8, __tls_align) );
	PROVIDE( __arm64_tls_tcb_offset = MAX(16, __tls_align) );

	.bss (NOLOAD) : ALIGN(4)
	{
		/* This is used by the startup in order to initialize the .bss section */
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;         /* define a global symbol at bss end */
		__bss_end__ = _ebss;
		PROVIDE( __bss_end = .);
	} >AXISRAM2

	PROVIDE( __non_tls_bss_start = ADDR(.bss) );
	PROVIDE( __bss_start = __tbss_start );
	PROVIDE( __bss_size = __bss_end - __bss_start );

	/* Fast data stored in DTCM */
	.dtcm_bss (NOLOAD) :
	{
		. = ALIGN(4);
		_sdtcm_bss = .;
		*(.dtcm_bss)
		*(.dtcm_bss*)
		. = ALIGN(4);
		_edtcm_bss = .;
	} >DTCM

	/* User_heap_stack section, used to check that there is enough "RAM" Ram  type memory left */
	._user_heap_stack (NOLOAD) :
	{
		. = ALIGN(8);
		PROVIDE ( end = . );
		PROVIDE ( _end = . );
		. = . + _Min_Heap_Size;
		. = . + _Min_Stack_Size;
		. = ALIGN(8);
	} >AXISRAM2

	/* Remove information from the compiler libraries */
	/DISCARD/ :
	{
		libc.a:* ( * )
		libm.a:* ( * )
		libgcc.a:* ( * )
	}

	.ARM.attributes 0 : { *(.ARM.attributes) }
}
