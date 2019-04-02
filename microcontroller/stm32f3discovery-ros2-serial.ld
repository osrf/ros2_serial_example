/* Debug linker script for Soho Concept Main Board (STM32F407ZG, 1024K flash,
 * 128K SRAM + 64K data SRAM) */

/* Define memory regions. */
/* Note that the CCM region is only accessible to the CPU (so it can't be used
 * for things like DMA).  Also note that the data in CCM is zeroed at the
 * beginning of the program; there is a way to change this described here:
 * http://sigalrm.blogspot.com/2013/12/using-ccm-memory-on-stm32.html
 * To put data into the CCM region, mark the data with
 *  __attribute__((section(".ccm")))
 */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 256K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 40K
	ccm (rwx) : ORIGIN = 0x10000000, LENGTH = 8K
}

EXTERN(vector_table)

/* Define the entry point of the output file. */
ENTRY(reset_handler)

/* Define sections. */
SECTIONS
{
	.text : {
                *(.vectors)
		. = ALIGN(4);
		*(.text*)	/* Program code */
		*(.rodata*)	/* Read-only data */
		. = ALIGN(4);
                _etext = .;
	} >rom

	/* C++ Static constructors/destructors, also used for __attribute__
	 * ((constructor)) and the likes */
	.preinit_array : {
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
	} >rom
	.init_array : {
		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;
	} >rom
	.fini_array : {
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} >rom

	.ARM.extab : {
		*(.ARM.extab*)
	} >rom
	.ARM.exidx : {
		__exidx_start = .;
		*(.ARM.exidx*)
		__exidx_end = .;
                _sidata = __exidx_end;
	} >rom

	.data : AT ( _sidata ) {
		. = ALIGN(4);
                _data = .;
		*(.data*)	/* Read-write initialized data */
		. = ALIGN(4);
		_edata = .;
	} >ram
        _data_loadaddr = LOADADDR(.data);

	.bss : {
                . = ALIGN(4);
                _sbss = .;
		*(.bss*)	/* Read-write zero initialized data */
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} >ram

        .stack (NOLOAD) : {
            . = ALIGN(8);
            . = . + 3000;
        } >ram

	.ccm (NOLOAD) : {
	     . = ALIGN(4);
	     _sccm = .;
	     *(.ccm)
	     . = ALIGN(4);
	     _eccm = .;
	} >ccm

	/*
	 * The .eh_frame section appears to be used for C++ exception handling.
	 * You may need to fix this if you're using C++.
	 */
	/DISCARD/ : { *(.eh_frame) }

	. = ALIGN(4);
	end = .;
}

PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
