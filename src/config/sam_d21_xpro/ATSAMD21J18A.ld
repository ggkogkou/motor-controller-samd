__ROM_BASE = 0x00000000;
__ROM_SIZE = 0x00040000;    /* SAMD21J18A: 256KB Flash */
__RAM_BASE = 0x20000000;
__RAM_SIZE = 0x00008000;    /* SAMD21J18A: 32KB RAM */
__STACK_SIZE = 0x00002000;  /* 8KB Stack (generous for M0+) */
__HEAP_SIZE  = 0x00001000;  /* 4KB Heap (adjust as needed) */

MEMORY
{
  FLASH (rx)  : ORIGIN = __ROM_BASE, LENGTH = __ROM_SIZE
  RAM   (rwx) : ORIGIN = __RAM_BASE, LENGTH = __RAM_SIZE
}

ENTRY(Reset_Handler)

SECTIONS
{
  .text :
  {
    KEEP(*(.vectors))
    *(.text*)

    KEEP(*(.init))
    KEEP(*(.fini))

    /* .ctors */
    *crtbegin.o(.ctors)
    *crtbegin?.o(.ctors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
    *(SORT(.ctors.*))
    *(.ctors)

    /* .dtors */
    *crtbegin.o(.dtors)
    *crtbegin?.o(.dtors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
    *(SORT(.dtors.*))
    *(.dtors)

    *(.rodata*)

    KEEP(*(.eh_frame*))
  } > FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  __exidx_start = .;
  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > FLASH
  __exidx_end = .;

  .copy.table :
  {
    . = ALIGN(4);
    __copy_table_start__ = .;

    /* Data section copy info */
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
  } > FLASH

  .zero.table :
  {
    . = ALIGN(4);
    __zero_table_start__ = .;
    
    /* BSS section zero info */
    LONG (ADDR(.bss))
    LONG (SIZEOF(.bss) / 4)

    __zero_table_end__ = .;
  } > FLASH

  PROVIDE(__etext = LOADADDR(.data));

  .stack (ORIGIN(RAM)) (NOLOAD) :
  {
    . = ALIGN(8);
    __StackLimit = .;
    . = . + __STACK_SIZE;
    . = ALIGN(8);
    __StackTop = .;
  } > RAM
  PROVIDE(__stack = __StackTop);
  PROVIDE(_stack = __StackTop);

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

  } > RAM AT > FLASH

  /*
   * RAM functions section - for time-critical code
   * (e.g., flash programming routines, interrupt handlers)
   */
  .ramfunc : ALIGN(4)
  {
    . = ALIGN(4);
    __ramfunc_start__ = .;
    *(.ramfunc)
    *(.ramfunc.*)
    . = ALIGN(4);
    __ramfunc_end__ = .;
  } > RAM AT > FLASH

  .ram_lut : ALIGN(4)
  {
    . = ALIGN(4);
    __ram_lut_start__ = .;
    *(.ram_lut)
    *(.ram_lut.*)
    . = ALIGN(4);
    __ram_lut_end__ = .;
  } > RAM AT > FLASH

  .bss :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > RAM AT > RAM

  .heap (COPY) :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    PROVIDE(_end = __end__);
    . = . + __HEAP_SIZE;
    . = ALIGN(8);
    __HeapLimit = .;
  } > RAM

  /* Legacy SAMD21 symbol compatibility */
  PROVIDE(_stack = __StackTop);

  /* Check if data + heap + stack exceeds RAM limit */
  ASSERT(__StackTop <= __data_start__, "region RAM overflowed with stack")
}
