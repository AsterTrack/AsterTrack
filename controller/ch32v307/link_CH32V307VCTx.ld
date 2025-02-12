ENTRY(_start)

__stack_size = 2048;

PROVIDE(_stack_size = __stack_size);


MEMORY
{
/*	CH32V30x_D8C - CH32V307VC-CH32V307WC-CH32V307RC
	CH32V30x_D8 - CH32V303VC-CH32V303RC
	FLASH + RAM supports the following configuration
	FLASH-192K + RAM-128K
	FLASH-224K + RAM-96K
	FLASH-256K + RAM-64K  
	FLASH-288K + RAM-32K  
*/
	FLASH(rx): ORIGIN = 0x00000000, LENGTH = 192K
	RAM(xrw):  ORIGIN = 0x20000000, LENGTH = 128K
}


SECTIONS
{

	.init :
	{
		_sinit = .;
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.init)))
		. = ALIGN(4);
		_einit = .;
	} >FLASH AT>FLASH

	.vector :
	{
		*(.vector);
		. = ALIGN(64);
	} >FLASH AT>FLASH

	.text :
	{ /* Code, stays in flash */
		. = ALIGN(4);
		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} >FLASH AT>FLASH 

	.dalign :
	{ /* VMA of data (when in RAM), copy target */
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM AT>FLASH	

	.dlalign :
	{ /* LMA of data (when in flash), copy source */
		. = ALIGN(4); 
		PROVIDE(_data_lma = .);
	} >FLASH AT>FLASH

	.data :
	{ /* Initialised data to be copied from flash to ram on startup */
		*(.data .data.*)
		. = ALIGN(8);
		PROVIDE( __global_pointer$ = . + 0x800 );
		*(.sdata .sdata.*)
		*(.sdata2.*)
		. = ALIGN(8);
		*(.srodata.cst16)
		*(.srodata.cst8)
		*(.srodata.cst4)
		*(.srodata.cst2)
		*(.srodata .srodata.*)
		. = ALIGN(4);
		PROVIDE( _edata = .);
	} >RAM AT>FLASH

	.bss :
	{ /* Uninitialised data, initialised to 0 in RAM */
		. = ALIGN(4);
		PROVIDE( _sbss = .);
		*(.sbss*)
		*(.bss*)
		*(COMMON*)
		. = ALIGN(4);
		PROVIDE( _ebss = .);
	} >RAM AT>FLASH

	PROVIDE( _end = _ebss);
	PROVIDE( end = . );

	.stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size :
	{
		PROVIDE( _heap_end = . );    
		. = ALIGN(4);
		PROVIDE(_susrstack = . );
		. = . + __stack_size;
		PROVIDE( _eusrstack = .);
	} >RAM 

}



