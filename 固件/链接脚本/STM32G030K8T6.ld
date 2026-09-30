ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

MEMORY
{
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 8K
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
}

/* Reserve stack space explicitly.  The worst software chain is 480 bytes;
 * Cortex-M0+ exception entry adds 32 bytes, TIM3's ISR uses 8 bytes, and
 * 56 bytes remain as explicit alignment/safety margin.  Static data must
 * stay below this boundary or the link fails closed. */
__stack_reserve = 576;
__ram_data_limit = ORIGIN(RAM) + LENGTH(RAM) - __stack_reserve;

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  ASSERT(_ebss <= __ram_data_limit,
         "static RAM exceeds reserved stack")
}
