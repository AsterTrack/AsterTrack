
; Notes on the vector ISA.
;-------------------------
;
; Descriptions on https://github.com/hermanhermitage/videocoreiv/wiki/VideoCore-IV-Programmers-Manual
;
; Further examples found at https://bitbucket.org/simonj23/frappe
;

.text

.global find_active_tiles
find_active_tiles:
	; Given an input buffer and mask buffer,
	; find lines (64 bytes) with active bits after masking, 
	; then write their indices into an output buffer.
	; Requires buffer sizes to be multiple of 5*64 bytes
	; (can be manually edited to different multiple, 5 is matched to qpu_blob_tiled blocking)
	;
	; r0	src
	; r1	mask
	; r2	dest
	; r3	count

	stm			r6-r9,		(--sp)

	#define src				r7
	#define msk				r8
	#define dst				r9
	mov			src,		r0
	mov			msk,		r1
	mov			dst,		r2

	#define stride			r4
	#define numLine			r5
	#define cntLine			r6
	mov			stride,		64                                      ; stride of each line (16 32-bit tiles, so 64 bytes)
	lsr			numLine,	r3,			4                           ; number of total lines (with 16 tiles each)
	mov			cntLine,	0                                       ; counter for lines outside of current block (up to numLine)

1:
	; Load next 5 lines
	mov			r0,			5										; number of lines to fetch, and iterator to handle them all
	v32ld		H32(0++,0),	(src+=stride)			REP r0			; load 5 rows of 16 32 bit values from source
	v32ld		H32(5++,0),	(msk+=stride)			REP r0			; load 5 rows of 16 32 bit values from background mask

	#define subLine			r3
	mov			subLine,	0                                       ; counter for lines within current block (up to block size of 5)
2:
	; Check line for any non-zeros and branch to write output
	shl			r2,			subLine,	6							; Move line iterator to 6-bit position as vertical vector offset
	v32and		H32(0,0)+r2,H32(0,0)+r2,H32(5,0)+r2	SETF            ; AND source and background mask of current line
	v32add		H32(0,0)+r2,H32(0,0)+r2,#0			IFNZ IMIN r1    ; Get min element among non-zero tiles (-1 if all tiles are 0)
	bge			r1,			0,						4f				; Branch to subroutine if any non-zero exist (min index exists)
3:	; Return from subroutine

	addcmpblt	subLine,	1,			5,			2b				; Iterate over all loaded lines

	; Update addresses for next load
	add			src,		src,		5*64						; Update source address
	add			msk,		msk,		5*64						; Update mask address

	addcmpblt	cntLine,	5,			numLine,	1b				; Iterate over blocks of lines until total number of lines is reached

	; Write terminating 0xFFFF to signal end of regions
	mov			r0,			-1
	sth			r0,			(dst)

	ldm			r6-r9,		(sp++)
	rts

4:
	; Write line index to output
	add			r0,			cntLine,	subLine						; Add line counter within block to line number of first line of block
	sth			r0,			(dst)									; Write 16-bit line index to current destination address
	add			dst,		dst,		2							; Increase destination address
	b												3b				; Jump back to continue with next line in block of 5
