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
__NOINIT_SIZE = 0x00000100;

/* 8KB Stack */
__STACK_SIZE = 0x00002000;

/* 4KB Heap */
__HEAP_SIZE  = 0x00001000;

/* TMR banks; 1024 bytes each */
__TMR_BANK_SIZE = 0x400;

__STACK_TOP_ALIGNED = ALIGN(__RAM_BASE + __STACK_SIZE, __TMR_BANK_SIZE);
__TMR_BANK_1_BASE = __STACK_TOP_ALIGNED;
__TMR_BANK_2_BASE = __TMR_BANK_1_BASE + __TMR_BANK_SIZE;
__TMR_BANK_3_BASE = __TMR_BANK_2_BASE + __TMR_BANK_SIZE;

ENTRY(Reset_Handler)

MEMORY
{
    flash (rx)  : ORIGIN = __ROM_BASE, LENGTH = __ROM_SIZE
    ram   (rwx) : ORIGIN = __RAM_BASE, LENGTH = __RAM_SIZE - __NOINIT_SIZE
    noinit (rwx) : ORIGIN = __RAM_BASE + __RAM_SIZE - __NOINIT_SIZE, LENGTH = __NOINIT_SIZE
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

        /* Zero out the TMR banks contents */
        LONG (ADDR(.tmr_bank_1))
        LONG (SIZEOF(.tmr_bank_1) / 4)
        LONG (ADDR(.tmr_bank_2))
        LONG (SIZEOF(.tmr_bank_2) / 4)
        LONG (ADDR(.tmr_bank_3))
        LONG (SIZEOF(.tmr_bank_3) / 4)

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

    .tmr_bank_1 __TMR_BANK_1_BASE (NOLOAD) :
    {
        . = ALIGN(4);
        __tmr_bank_1_start__ = .;
        KEEP(*(.tmr_bank_1))
        KEEP(*(.tmr_bank_1.*))
        . = __tmr_bank_1_start__ + __TMR_BANK_SIZE;
        . = ALIGN(4);
        __tmr_bank_1_end__ = .;
    } > ram

    .tmr_bank_2 __TMR_BANK_2_BASE (NOLOAD) :
    {
        . = ALIGN(4);
        __tmr_bank_2_start__ = .;
        KEEP(*(.tmr_bank_2))
        KEEP(*(.tmr_bank_2.*))
        . = __tmr_bank_2_start__ + __TMR_BANK_SIZE;
        . = ALIGN(4);
        __tmr_bank_2_end__ = .;
    } > ram

    .tmr_bank_3 __TMR_BANK_3_BASE (NOLOAD) :
    {
        . = ALIGN(4);
        __tmr_bank_3_start__ = .;
        KEEP(*(.tmr_bank_3))
        KEEP(*(.tmr_bank_3.*))
        . = __tmr_bank_3_start__ + __TMR_BANK_SIZE;
        . = ALIGN(4);
        __tmr_bank_3_end__ = .;
    } > ram

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

    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        __noinit_start__ = .;
        KEEP(*(.noinit))
        KEEP(*(.noinit.*))
        . = ALIGN(4);
        __noinit_end__ = .;
    } > noinit

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__HeapLimit <= (ORIGIN(ram) + LENGTH(ram)), "32kB RAM is not enough!!!")
    ASSERT((__noinit_end__ - __noinit_start__) <= __NOINIT_SIZE, ".noinit does not fit in reserved RAM")

    /* Check if the TMR bank size and ordering checks */
    ASSERT((__tmr_bank_1_end__ - __tmr_bank_1_start__) == __TMR_BANK_SIZE, "TMR bank 1 wrong size")
    ASSERT((__tmr_bank_2_end__ - __tmr_bank_2_start__) == __TMR_BANK_SIZE, "TMR bank 2 wrong size")
    ASSERT((__tmr_bank_3_end__ - __tmr_bank_3_start__) == __TMR_BANK_SIZE, "TMR bank 3 wrong size")
    ASSERT(__tmr_bank_3_end__ <= __data_start__, "TMR banks overlap .data")
}
