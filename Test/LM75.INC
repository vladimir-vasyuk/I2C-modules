	.TITLE	LM75HD -- LM75 I2C module test
	.IDENT	/X1.00/
	.ENABLE	LC
;
	.SBTTL	Definition section
;
	CSR	= 175320	; CSR
	DATA	= CSR+2		; Data-Register
;
	CR	= 4000
	HR	= 10000
	OR	= 14000
;
	M.RST	= 01		; Software reset
	M.WRT	= 03		; Write operation
	M.RDR	= 05		; Read operation
;
	IEB	= 100		; Interrupt enable
	RDYB	= 200		; Ready
	TIB	= 20000		; Transmit interrupt request
	RIB	= 40000		; Receive interrupt request
	ERRB	= 100000	; Error
;
	MODVEC	= 314		; Interrupt vector
	PRI07	= 340		; IPL = 7
;
	IEF	= 1		; Interrupt information bit
	RXF	= 4		; Receive information bit
	TXF	= 2		; Transmit information bit
	ERF	= 10		; Error information bit
