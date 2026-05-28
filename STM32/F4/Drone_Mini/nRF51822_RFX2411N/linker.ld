/* linker.ld */

ENTRY(main)
_start = main;

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 16K
}

SECTIONS
{
.text :
  {
    . = ALIGN(4);
    KEEP(*(.vectors))
    KEEP(*(.isr_vector))
    *(.text*)
    *(.rodata*)
    *(.init)
    *(.fini)
    . = ALIGN(4);
    __etext = .;    
    _etext = .; 
  } > FLASH

  .data : 
  {
    . = ALIGN(4);
    __data_start__ = .;
    *(.data*)
    . = ALIGN(4);
    __data_end__ = .;
  } > RAM AT > FLASH

  .bss :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > RAM

  end = .;
  _end = .;
  __end__ = .;

  __StackTop = ORIGIN(RAM) + LENGTH(RAM);
  PROVIDE(__stack = __StackTop);

  .heap (NOLOAD) :
  {
    . = ALIGN(4);
    __end__ = .;
    end = .;
    . = . + 0x800; /* Cấp 2KB cho Heap */
    __HeapLimit = .;
  } > RAM
}