	.text
	.global _start
_start:
	; Save registers and switch stack
	stm r6, lr, (--sp)
	mov r6, sp
	mov sp, #_fstack
	stm r6, (--sp)

	# Call main directly
	# Does not support e.g. bss or ctor/dtors
	# See vc4-toolchain for more advanced crt0/crti/crtn
	bl main

	; Restore prior stack and registers
	ldm r6, (sp++)
	mov sp, r6
	ldm r6, pc, (sp++)

