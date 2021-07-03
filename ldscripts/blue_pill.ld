
RAM_START   = 0x20000000;
RAM_END     = 0x20005000;

_estack     = RAM_END;

/* ENTRY(main) */


SECTIONS
{
    /* This is for ability to change link address with `-Ttext=<addr>` ld option */
    .text : {}

    /* Align interrupts vectors table to 512-byte boundary */
    . = ALIGN(512);

    /* C generated vectors sections of name `.isr_vector.__vec_*` */
    INCLUDE vectors.ld

    /* ASM/C generated vectors */
    .isr_vector : { KEEP(*(.isr_vector*)) KEEP(*(.iv))  KEEP(*(.vt)) }

    /* Code and read-only data; can be aligned to (2) */
    .text ALIGN(4) : { *(.text*) *(.rodata*) }

    /* Data alignment is not stricly required */

    /* Save .text end address; .data init values retain here */
    _sidata = ALIGN(4);

    /* Move .data and .bss to ram if . isn't already there */
    . = . < RAM_START ? RAM_START : . ;

    /* Link .data always to RAM */
    .data ALIGN(4) : AT (_sidata) { _sdata = . ; *(.data*) _edata = . ; }

    /* Link .bss always to RAM after the .data */
    .bss ALIGN(4) : { _sbss = . ; *(.bss*) *(COMMON) _ebss = . ; }

    /* Remove sections that are not required */
    /DISCARD/ : { *(.ARM.attributes) *(.comment*) *(.note*) }
}
