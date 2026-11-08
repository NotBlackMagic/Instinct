/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (c) 2026 NotBlackMagic (Pluma Robotics)
 *
 * File:	Instinct/STM32N657XX_AXISRAM2_fsbl.ld
 * Author:	NotBlackMagic
 * Brief:	Linker script for STM32N657XX Device from STM32N6 series for XIP mode, execute directly from HyperFlash
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
	AHBSRAM1	(xrw)	: ORIGIN = 0x38000000, LENGTH = 16K		/* AHBSRAM1 (0x38000000 - 0x38004000) */
	AHBSRAM2	(xrw)	: ORIGIN = 0x38004000, LENGTH = 16K		/* AHBSRAM2 (0x38004000 - 0x38008000) */
	VENCRAM		(xrw)	: ORIGIN = 0x34400000, LENGTH = 128K	/* VENCRAM (0x34400000 - 0x34420000) */
	BKPSRAM		(xrw)	: ORIGIN = 0x3C000000, LENGTH = 8K		/* BKPSRAM (0x3C000000 - 0x3C002000) */
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
_itcm_capacity = LENGTH(ITCM);
_dtcm_capacity = LENGTH(DTCM);
_axisram1_capacity = LENGTH(AXISRAM1);
_axisram3_capacity = LENGTH(AXISRAM3);
_axisram4_capacity = LENGTH(AXISRAM4);
_axisram5_capacity = LENGTH(AXISRAM5);
_axisram6_capacity = LENGTH(AXISRAM6);
_ahbsram1_capacity = LENGTH(AHBSRAM1);
_ahbsram2_capacity = LENGTH(AHBSRAM2);
_vencram_capacity  = LENGTH(VENCRAM);
_bkpsram_capacity  = LENGTH(BKPSRAM);

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

	/* The program code and other data into "RAM" Ram type memory */
	.text :
	{
		. = ALIGN(4);
		_stext = .;
		*(.text)		/* .text sections (code) */
		*(.text*)		/* .text* sections (code) */
		*(.glue_7)		/* glue arm to thumb code */
		*(.glue_7t)		/* glue thumb to arm code */
		*(.eh_frame)
		*(.RamFunc)		/* .RamFunc sections */
		*(.RamFunc*)	/* .RamFunc* sections */

		KEEP (*(.init))
		KEEP (*(.fini))

		. = ALIGN(4);
		_etext = .;		/* define a global symbols at end of code */
	} >ROM

	/* Constant data into "RAM" Ram type memory */
	.rodata :
	{
		. = ALIGN(4);
		*(.rodata)		/* .rodata sections (constants, strings, etc.) */
		*(.rodata*)		/* .rodata* sections (constants, strings, etc.) */
		. = ALIGN(4);
	} >ROM

	/* Secure Gateway stubs must reside in ROM adjacent to text */
	.gnu.sgstubs : ALIGN(32)
	{
		. = ALIGN(32);
		*(.gnu.sgstubs*)	/* Secure Gateway stubs */
		. = ALIGN(32);
		FILL(0x00);
	} >ROM

	.ARM.extab : {
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

	.preinit_array :
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

	/* RAM & TCM Sections with Load Addresses in ROM */

	/* ITCM critical execution code */
	.itcm_text :
	{
		. = ALIGN(4);
		_sitcm = .;
		*(.itcm_text)
		*(.itcm_text*)
		. = ALIGN(4);
		_eitcm = .;
	} >ITCM AT> ROM
	_siitcm = LOADADDR(.itcm_text);

	/* AXISRAM2 initialized data */
	.data :
	{
		. = ALIGN(4);
		_sdata = .;			/* create a global symbol at data start */
		*(.data)			/* .data sections */
		*(.data*)			/* .data* sections */
		. = ALIGN(4);
	} >AXISRAM2 AT> ROM
	/* Used by the startup to initialize data */
	_sidata = LOADADDR(.data);

	/* Initialized TLS data section */
	.tdata : ALIGN(4)
	{
		*(.tdata .tdata.* .gnu.linkonce.td.*)
		. = ALIGN(4);
		_edata = .;			/* define a global symbol at data end */
		PROVIDE(__data_end = .);
		PROVIDE(__tdata_end = .);
	} >AXISRAM2 AT> ROM

	/* DTCM initialized data & fast tables */
	.dtcm_data : ALIGN(4)
	{
		_sdtcm_data = .;
		*(.dtcm_data)
		*(.dtcm_data*)
		*(.dtcm_rodata*)
		. = ALIGN(4);
		_edtcm_data = .;
	} >DTCM AT> ROM
	_sidtcm_data = LOADADDR(.dtcm_data);

	/* The absolute end of the ROM payload is the Load Address (LMA) of .dtcm_data plus its size */
	_erom = LOADADDR(.dtcm_data) + SIZEOF(.dtcm_data);

	/* NOLOAD Sections */

	.noncacheable :
	{
		. = ALIGN(8);
		__snoncacheable = .;	/* create symbol for start of section */
		KEEP(*(.noncacheable))
		. = ALIGN(8);
		__enoncacheable = .;	/* create symbol for end of section */
	} > AXISRAM2

	/* Dedicated buffers placed in AXISRAM1 */
	.axisram1 (NOLOAD) : ALIGN(32)
	{
		. = ALIGN(32);
		_saxisram1 = .;
		*(.axisram1)
		*(.axisram1*)
		. = ALIGN(32);
		_eaxisram1 = .;
	} > AXISRAM1

	/* Dedicated buffers placed in AXISRAM3 */
	.axisram3 (NOLOAD) : ALIGN(32)
	{
		. = ALIGN(32);
		_saxisram3 = .;
		*(.axisram3)
		*(.axisram3*)
		. = ALIGN(32);
		_eaxisram3 = .;
	} > AXISRAM3

	/* Dedicated buffers placed in AXISRAM4 */
	.axisram4 (NOLOAD) : ALIGN(32)
	{
		. = ALIGN(32);
		_saxisram4 = .;
		*(.axisram4)
		*(.axisram4*)
		. = ALIGN(32);
		_eaxisram4 = .;
	} > AXISRAM4

	/* Dedicated buffers placed in AXISRAM5 */
	.axisram5 (NOLOAD) : ALIGN(32)
	{
		. = ALIGN(32);
		_saxisram5 = .;
		*(.axisram5)
		*(.axisram5*)
		. = ALIGN(32);
		_eaxisram5 = .;
	} > AXISRAM5

	/* Dedicated buffers placed in AXISRAM6 */
	.axisram6 (NOLOAD) : ALIGN(32)
	{
		. = ALIGN(32);
		_saxisram6 = .;
		*(.axisram6)
		*(.axisram6*)
		. = ALIGN(32);
		_eaxisram6 = .;
	} > AXISRAM6

	/* Peripheral AHB SRAM 1 */
	.ahbsram1 (NOLOAD) : ALIGN(4)
	{
		. = ALIGN(4);
		_sahbsram1 = .;
		*(.ahbsram1)
		*(.ahbsram1*)
		. = ALIGN(4);
		_eahbsram1 = .;
	} > AHBSRAM1

	/* Peripheral AHB SRAM 2 */
	.ahbsram2 (NOLOAD) : ALIGN(4)
	{
		. = ALIGN(4);
		_sahbsram2 = .;
		*(.ahbsram2)
		*(.ahbsram2*)
		. = ALIGN(4);
		_eahbsram2 = .;
	} > AHBSRAM2

	/* Video Encoder Hardware SRAM */
	.vencram (NOLOAD) : ALIGN(32)
	{
		. = ALIGN(32);
		_svencram = .;
		*(.vencram)
		*(.vencram*)
		. = ALIGN(32);
		_evencram = .;
	} > VENCRAM

	/* Battery Backup SRAM */
	.bkpsram (NOLOAD) : ALIGN(4)
	{
		. = ALIGN(4);
		_sbkpsram = .;
		*(.bkpsram)
		*(.bkpsram*)
		. = ALIGN(4);
		_ebkpsram = .;
	} > BKPSRAM

	/* Uninitialized data section into "RAM" Ram type memory */
	/* Uninitialized TLS data section */
	.tbss (NOLOAD) : ALIGN(4)
	{
		_sbss = .;				/* define a global symbol at bss start */
		__bss_start__ = _sbss;
		*(.tbss .tbss.*)
		. = ALIGN(4);
		PROVIDE( __tbss_end = . );
	} >AXISRAM2

	.bss (NOLOAD) : ALIGN(4)
	{
		/* This is used by the startup in order to initialize the .bss section */
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;				/* define a global symbol at bss end */
		__bss_end__ = _ebss;
		PROVIDE( __bss_end = .);
	} >AXISRAM2

	/* Fast uninitialized data stored in DTCM */
	.dtcm_bss (NOLOAD) :
	{
		. = ALIGN(4);
		_sdtcm_bss = .;
		*(.dtcm_bss)
		*(.dtcm_bss*)
		. = ALIGN(4);
		_edtcm_bss = .;
	} >DTCM

	/* User_heap_stack section, used to check that there is enough "RAM" Ram type memory left */
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
