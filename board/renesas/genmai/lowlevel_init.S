/*
 * Copyright (C) 2016 Renesas America
 *
 * Based on board/renesas/rsk7264/lowlevel_init.S
 *
 * This file is released under the terms of GPL v2 and any later version.
 * See the file COPYING in the root directory of the source tree for details.
 */
#include <config.h>
#include <version.h>
#include <asm/arch/rza1-regs.h>
#include <asm/macro.h>

	.global	lowlevel_init

	.text
	.align	2

lowlevel_init:
	/* PL310 init */
	write32 0x3fffff80, 0x00000001

	/* Disable WDT */
	write16	WTCSR, WTCSR_D
	write16	WTCNT, WTCNT_D

	/* Disable Register Bank interrupts */
	/* write16 IBNR, IBNR_D */

	/* Set clocks */
	write16	FRQCR, FRQCR_D

	/* Enable all peripherals(Standby Control) */
	write8 STBCR3, STBCR3_D
	write8 STBCR4, STBCR4_D
	write8 STBCR5, STBCR5_D
	write8 STBCR6, STBCR6_D
	write8 STBCR7, STBCR7_D
	write8 STBCR8, STBCR8_D
	write8 STBCR9, STBCR9_D
	write8 STBCR10, STBCR10_D
	write8 STBCR11, STBCR11_D
	write8 STBCR12, STBCR12_D

	/* For serial booting, enable read ahead caching to speed things up */
#define DRCR_0  0x3FEFA00C
	write32 DRCR_0, 0x00010100	/* Read Burst ON, Length=2 */

	/* Enable all internal RAM */
	write8 SYSCR1, 0xFF
	write8 SYSCR2, 0xFF
	write8 SYSCR3, 0xFF

	nop
	/* back to arch calling code */
	mov	pc, lr

	.align 4
