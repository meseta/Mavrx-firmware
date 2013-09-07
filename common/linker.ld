/******************************************************************************
 * Copyright (c) 2011, Universal Air Ltd. All rights reserved.
 * Source and binaries are released under the BSD 3-Clause license
 * See readme_forebrain.txt files for the text of the license
 ******************************************************************************/

MEMORY {
  flash (rx) : ORIGIN = 0x00000000, LENGTH = 64K
  sram0 (rwx) : ORIGIN = 0x10000000, LENGTH = 8K
  usbsram (rwx) : ORIGIN = 0x20004000, LENGTH = 2K
  sram1 (rwx) : ORIGIN = 0x20000000, LENGTH = 2K
}
__top_flash = 0x0 + 0x10000;
__top_sram0 = 0x10000000 + 0x2000;
__top_usbsram = 0x20004000 + 0x800;
__top_sram1 = 0x20000000 + 0x800;

ENTRY(Reset_Handler)

SECTIONS {
	.text : ALIGN(4) {
		FILL(0xff)
		KEEP(*(.vectors))
		
		/* Global Section Table */
		. = ALIGN(4) ;
		__section_table_start = .;
		__data_section_table = .;
		LONG(LOADADDR(.data));
		LONG(    ADDR(.data));
		LONG(  SIZEOF(.data));
		LONG(LOADADDR(.data_RAM2));
		LONG(    ADDR(.data_RAM2));
		LONG(  SIZEOF(.data_RAM2));
		LONG(LOADADDR(.data_RAM3));
		LONG(    ADDR(.data_RAM3));
		LONG(  SIZEOF(.data_RAM3));
		__data_section_table_end = .;
		__bss_section_table = .;
		LONG(    ADDR(.bss));
		LONG(  SIZEOF(.bss));
		LONG(    ADDR(.bss_RAM2));
		LONG(  SIZEOF(.bss_RAM2));
		LONG(    ADDR(.bss_RAM3));
		LONG(  SIZEOF(.bss_RAM3));
		__bss_section_table_end = .;
		__section_table_end = . ;
		
		*(.after_vectors*)
        
        . = 0x000002FC;
        KEEP(*(.crp))
        
		*(.text*)
		*(.rodata .rodata.*)
		. = ALIGN(4);
		
	} > flash

	.ARM.extab : ALIGN(4) {
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > flash
	__exidx_start = .;
	
	.ARM.exidx : ALIGN(4) {
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > flash
	__exidx_end = .;
	
	_etext = .;
		
	.data_RAM2 : ALIGN(4) {
	   FILL(0xff)
		*(.data.$RAM2*)
		*(.data.$usbsram*)
	   . = ALIGN(4) ;
	} > usbsram AT>flash
	
	.data_RAM3 : ALIGN(4) {
	   FILL(0xff)
		*(.data.$RAM3*)
		*(.data.$sram1*)
	   . = ALIGN(4) ;
	} > sram1 AT>flash
	
	.uninit_RESERVED : ALIGN(4) {
		KEEP(*(.bss.$RESERVED*))
	} > sram0

	.data : ALIGN(4) {
		FILL(0xff)
		_data = .;
		*(vtable)
		*(.data*)
		. = ALIGN(4) ;
		_edata = .;
	} > sram0 AT>flash
	
	.bss_RAM2 : ALIGN(4) {
		*(.bss.$RAM2*)
		*(.bss.$usbsram*)
	   . = ALIGN(4) ;
	} > usbsram
	
	.bss_RAM3 : ALIGN(4) {
		*(.bss.$RAM3*)
		*(.bss.$sram1*)
	   . = ALIGN(4) ;
	} > sram1

	.bss : ALIGN(4) {
		_bss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4) ;
		_ebss = .;
		PROVIDE(end = .);
	} > sram0
	
	PROVIDE(_pvHeapStart = .);
	PROVIDE(__stack_top = __top_sram0 - 0);
}
