	.TITLE	EPRMHD -- EEPROM I2C module test
	.IDENT	/X1.02/
	.ENABLE	LC
;
	.SBTTL	Definition section
;
	CSR	= 175300	; CSR
	DATARW	= CSR+2		; Data-Register
	RDADDR	= CSR+4		; Read-Address-Register
	WRADDR	= CSR+6		; Write-Address-Register
	ERREG	= CSR+10	; Error-Register
	IRDREG	= CSR+12	; I2C-HOST address regs.
	IWRREG	= CSR+14	; HOST-I2C address regs.
;
	M.RST	= 01		; Software reset
	M.WRT	= 03	 	; Write operation
	M.RDR	= 05		; Read operation
;
	IEB	= 100		; Interrupt enable
	RDYB	= 200		; Ready
	TIB	= 20000		; Transmit interrupt request
	RIB	= 40000		; Receive interrupt request
	ERRB	= 100000	; Error
;
	MODVEC	= 310		; Interrupt vector
	PRI07	= 340		; IPL	= 7
;
	IEF	= 1		; Interrupt information bit
	RXF	= 4		; Receive information bit
	TXF	= 2		; Transmit information bit
	ERF	= 10		; Error information bit
