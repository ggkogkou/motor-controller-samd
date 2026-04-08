/*
 * Source: https://github.com/wntrblm/Castor_and_Pollux/blob/main/firmware/scripts/samd21g18a.ld
 * Source: https://github.com/ARM-software/CMSIS_6/blob/main/CMSIS/Core/Template/Device_M/Config/Device_gcc.ld
 * Source: https://arm-software.github.io/CMSIS_6/v6.0.0/Core/using_pg.html
 */

/* Output Format */
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")

/* SAMD21x18A: 256KB Flash */
__ROM_BASE = 0x00000000;
__ROM_SIZE = 0x00040000;

/* SAMD21x18A: 32KB RAM */
__RAM_BASE = 0x20000000;
__RAM_SIZE = 0x00008000;

/* 8KB Stack */
__STACK_SIZE = 0x00002000;

/* 4KB Heap */
__HEAP_SIZE  = 0x00001000;

ENTRY(Reset_Handler)

MEMORY
{
    flash (rx)  : ORIGIN = __ROM_BASE, LENGTH = __ROM_SIZE
    ram   (rwx) : ORIGIN = __RAM_BASE, LENGTH = __RAM_SIZE
}

SECTIONS
{
    /*
     * FLASH SECTIONS
     */
    .text :
    {

        KEEP(*(.vectors))
        *(.text*)

        KEEP(*(.init))
        KEEP(*(.fini))

        /* Constructors -- needed for C++ */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* Destructors -- needed for C++ */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        /* Read-only data */
        *(.rodata*)

        KEEP(*(.eh_frame*))
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > flash
    __exidx_end = .;

    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;

        /* .data section copy */
        LONG (LOADADDR(.data))
        LONG (ADDR(.data))
        LONG (SIZEOF(.data) / 4)

        /* RAM functions section copy info */
        LONG (LOADADDR(.ramfunc))
        LONG (ADDR(.ramfunc))
        LONG (SIZEOF(.ramfunc) / 4)

        /* RAM LUT section copy info */
        LONG (LOADADDR(.ram_lut))
        LONG (ADDR(.ram_lut))
        LONG (SIZEOF(.ram_lut) / 4)

        __copy_table_end__ = .;
    } > flash

    .zero.table :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;

        /* .bss section zeroing */
        LONG (ADDR(.bss))
        LONG (SIZEOF(.bss) / 4)

        __zero_table_end__ = .;
    } > flash

    /*
     * RAM SECTIONS
     */
    PROVIDE(__etext = LOADADDR(.data));

    .stack (ORIGIN(ram)) (NOLOAD) :
    {
        . = ALIGN(8);
        __StackLimit = .;
        . = . + __STACK_SIZE;
        . = ALIGN(8);
        __StackTop = .;
    } > ram
    PROVIDE(__stack = __StackTop);

    .data : ALIGN(4)
    {
        __data_start__ = .;
        *(vtable)
        *(.data)
        *(.data.*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;

    } > ram AT > flash

    .ramfunc : ALIGN(4)
    {
        . = ALIGN(4);
        __ramfunc_start__ = .;
        *(.ramfunc)
        *(.ramfunc.*)
        . = ALIGN(4);
        __ramfunc_end__ = .;
    } > ram AT > flash

    .ram_lut : ALIGN(4)
    {
        . = ALIGN(4);
        __ram_lut_start__ = .;
        *(.ram_lut)
        *(.ram_lut.*)
        . = ALIGN(4);
        __ram_lut_end__ = .;
    } > ram AT > flash

    .bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > ram AT > ram

    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        __end__ = .;
        PROVIDE(end = .);
        PROVIDE(_end = __end__);
        . = . + __HEAP_SIZE;
        . = ALIGN(8);
        __HeapLimit = .;
    } > ram

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackTop <= __data_start__, "32kB RAM is not enough!!!")
}
