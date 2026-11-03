ENTRY(Reset_Handler)
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  AXIRAM (xrw) : ORIGIN = 0x24000000, LENGTH = 320K
}
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
_Min_Heap_Size = 0x800;
_Min_Stack_Size = 0x800;
PHDRS
{
  flash PT_LOAD FLAGS(5);
  axi PT_LOAD FLAGS(6);
  dtcm PT_LOAD FLAGS(6);
}

SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH :flash
  .text :
  {
    . = ALIGN(4); *(.text*) *(.rodata*) *(.glue_7*)
    KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4);
  } > FLASH :flash
  .ARM.extab : { *(.ARM.extab*) } > FLASH :flash
  .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH :flash
  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH :flash
  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH :flash
  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH :flash
  _sidata = LOADADDR(.data);
  /* DMA1/2 cannot reach DTCM. Copy all initialized data, including the
     existing .AXI_SRAM DMA buffers, into DMA-accessible AXI SRAM. */
  .data :
  {
    . = ALIGN(4); _sdata = .;
    *(.data*) *(.AXI_SRAM*)
    . = ALIGN(4); _edata = .;
  } > AXIRAM AT> FLASH :axi
  .bss (NOLOAD) :
  {
    . = ALIGN(8); _sbss = .; *(.bss*) *(COMMON)
    . = ALIGN(8); _ebss = .;
  } > DTCMRAM :dtcm
  .heap_stack (NOLOAD) :
  {
    . = ALIGN(8); end = .; _end = .; __heap_start = .;
    . += _Min_Heap_Size; __heap_limit = .;
    . += _Min_Stack_Size; . = ALIGN(8);
  } > DTCMRAM :dtcm
  ASSERT(__heap_limit + _Min_Stack_Size <= _estack, "DTCM RAM overflow")
  /DISCARD/ : { *(.note.GNU-stack) }
}
