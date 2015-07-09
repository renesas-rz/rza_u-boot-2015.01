/*
 * RZ SPI driver
 *
 * Copyright (C) 2011-2014 Renesas Solutions Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef __RZ_SPI_H__
#define __RZ_SPI_H__

#include <spi.h>

struct stRzSpi{
	struct spi_slave	slave;

	volatile void*		pRegBase;
	u32			u32MaxSpeedHz;	// max_speed_hz
	unsigned int		unMode;

	u8			data_read_dual;	// single or dual chips in data read mode
	u8			this_cmd;	// what is the current SPI command
};

/* QSPI MODE */
#define READ_MODE		(0)
#define SPI_MODE		(1)

/* QSPI registers */
#define	QSPI_CMNCR		(0x0000)
#define	QSPI_SSLDR		(0x0004)
#define	QSPI_SPBCR		(0x0008)
#define	QSPI_DRCR		(0x000c)
#define	QSPI_DRCMR		(0x0010)
#define	QSPI_DREAR		(0x0014)
#define	QSPI_DROPR		(0x0018)
#define	QSPI_DRENR		(0x001c)
#define	QSPI_SMCR		(0x0020)
#define	QSPI_SMCMR		(0x0024)
#define	QSPI_SMADR		(0x0028)
#define	QSPI_SMOPR		(0x002c)
#define	QSPI_SMENR		(0x0030)

#define	QSPI_SMRDR0		(0x0038)
#define	QSPI_SMRDR1		(0x003c)
#define	QSPI_SMWDR0		(0x0040)
#define	QSPI_SMWDR1		(0x0044)
#define	QSPI_CMNSR		(0x0048)

#define	QSPI_DRDMCR		(0x0058)
#define	QSPI_DRDRENR		(0x005c)
#define	QSPI_SMDMCR		(0x0060)
#define	QSPI_SMDRENR		(0x0064)

/* CMNCR */
#define	CMNCR_MD		(1u << 31)
#define	CMNCR_SFDE		(1u << 24)

#define	CMNCR_MOIIO3(x)		(((u32)(x) & 0x3) << 22)
#define	CMNCR_MOIIO2(x)		(((u32)(x) & 0x3) << 20)
#define	CMNCR_MOIIO1(x)		(((u32)(x) & 0x3) << 18)
#define	CMNCR_MOIIO0(x)		(((u32)(x) & 0x3) << 16)
#define	CMNCR_IO3FV(x)		(((u32)(x) & 0x3) << 14)
#define	CMNCR_IO2FV(x)		(((u32)(x) & 0x3) << 12)
#define	CMNCR_IO0FV(x)		(((u32)(x) & 0x3) << 8)

#define	CMNCR_CPHAT		(1u << 6)
#define	CMNCR_CPHAR		(1u << 5)
#define	CMNCR_SSLP		(1u << 4)
#define	CMNCR_CPOL		(1u << 3)
#define	CMNCR_BSZ(n)		(((u32)(n) & 0x3) << 0)

#define	OUT_0			(0u)
#define	OUT_1			(1u)
#define	OUT_REV			(2u)
#define	OUT_HIZ			(3u)

#define	BSZ_SINGLE		(0)
#define	BSZ_DUAL		(1)

/* SSLDR */
#define	SSLDR_SPNDL(x)		(((u32)(x) & 0x7) << 16)
#define	SSLDR_SLNDL(x)		(((u32)(x) & 0x7) << 8)
#define	SSLDR_SCKDL(x)		(((u32)(x) & 0x7) << 0)

#define	SPBCLK_1_0		(0)
#define	SPBCLK_1_5		(0)
#define	SPBCLK_2_0		(1)
#define	SPBCLK_2_5		(1)
#define	SPBCLK_3_0		(2)
#define	SPBCLK_3_5		(2)
#define	SPBCLK_4_0		(3)
#define	SPBCLK_4_5		(3)
#define	SPBCLK_5_0		(4)
#define	SPBCLK_5_5		(4)
#define	SPBCLK_6_0		(5)
#define	SPBCLK_6_5		(5)
#define	SPBCLK_7_0		(6)
#define	SPBCLK_7_5		(6)
#define	SPBCLK_8_0		(7)
#define	SPBCLK_8_5		(7)

/* SPBCR */
#define	SPBCR_SPBR(x)		(((u32)(x) & 0xff) << 8)
#define	SPBCR_BRDV(x)		(((u32)(x) & 0x3) << 0)

/* DRCR (read mode) */
#define	DRCR_SSLN		(1u << 24)
#define	DRCR_RBURST(x)		(((u32)(x) & 0xf) << 16)
#define	DRCR_RCF		(1u << 9)
#define	DRCR_RBE		(1u << 8)
#define	DRCR_SSLE		(1u << 0)

/* DRCMR (read mode) */
#define	DRCMR_CMD(c)		(((u32)(c) & 0xff) << 16)
#define	DRCMR_OCMD(c)		(((u32)(c) & 0xff) << 0)

/* DREAR (read mode) */
#define	DREAR_EAV(v)		(((u32)(v) & 0xff) << 16)
#define	DREAR_EAC(v)		(((u32)(v) & 0x7) << 0)

/* DROPR (read mode) */
#define	DROPR_OPD3(o)		(((u32)(o) & 0xff) << 24)
#define	DROPR_OPD2(o)		(((u32)(o) & 0xff) << 16)
#define	DROPR_OPD1(o)		(((u32)(o) & 0xff) << 8)
#define	DROPR_OPD0(o)		(((u32)(o) & 0xff) << 0)

/* DRENR (read mode) */
#define	DRENR_CDB(b)		(((u32)(b) & 0x3) << 30)
#define	DRENR_OCDB(b)		(((u32)(b) & 0x3) << 28)
#define	DRENR_ADB(b)		(((u32)(b) & 0x3) << 24)
#define	DRENR_OPDB(b)		(((u32)(b) & 0x3) << 20)
#define	DRENR_DRDB(b)		(((u32)(b) & 0x3) << 16)
#define	DRENR_DME		(1u << 15)
#define	DRENR_CDE		(1u << 14)
#define	DRENR_OCDE		(1u << 12)
#define	DRENR_ADE(a)		(((u32)(a) & 0xf) << 8)
#define	DRENR_OPDE(o)		(((u32)(o) & 0xf) << 4)

/* SMCR (spi mode) */
#define	SMCR_SSLKP		(1u << 8)
#define	SMCR_SPIRE		(1u << 2)
#define	SMCR_SPIWE		(1u << 1)
#define	SMCR_SPIE		(1u << 0)

/* SMCMR (spi mode) */
#define	SMCMR_CMD(c)		(((u32)(c) & 0xff) << 16)
#define	SMCMR_OCMD(o)		(((u32)(o) & 0xff) << 0)

/* SMADR (spi mode) */

/* SMOPR (spi mode) */
#define	SMOPR_OPD3(o)		(((u32)(o) & 0xff) << 24)
#define	SMOPR_OPD2(o)		(((u32)(o) & 0xff) << 16)
#define	SMOPR_OPD1(o)		(((u32)(o) & 0xff) << 8)
#define	SMOPR_OPD0(o)		(((u32)(o) & 0xff) << 0)

/* SMENR (spi mode) */
#define	SMENR_CDB(b)		(((u32)(b) & 0x3) << 30)
#define	SMENR_OCDB(b)		(((u32)(b) & 0x3) << 28)
#define	SMENR_ADB(b)		(((u32)(b) & 0x3) << 24)
#define	SMENR_OPDB(b)		(((u32)(b) & 0x3) << 20)
#define	SMENR_SPIDB(b)		(((u32)(b) & 0x3) << 16)
#define	SMENR_DME		(1u << 15)
#define	SMENR_CDE		(1u << 14)
#define	SMENR_OCDE		(1u << 12)
#define	SMENR_ADE(b)		(((u32)(b) & 0xf) << 8)
#define	SMENR_OPDE(b)		(((u32)(b) & 0xf) << 4)
#define	SMENR_SPIDE(b)		(((u32)(b) & 0xf) << 0)

#define	ADE_23_16		(0x4)
#define	ADE_23_8		(0x6)
#define	ADE_23_0		(0x7)
#define	ADE_31_0		(0xf)

#define	BITW_1BIT		(0)
#define	BITW_2BIT		(1)
#define	BITW_4BIT		(2)

#define	SPIDE_16BITS_DUAL	(0x8)
#define	SPIDE_32BITS_DUAL	(0xc)
#define	SPIDE_64BITS_DUAL	(0xf)
#define	SPIDE_8BITS	(0x8)
#define	SPIDE_16BITS	(0xc)
#define	SPIDE_32BITS	(0xf)

#define	OPDE_3			(0x8)
#define	OPDE_3_2		(0xc)
#define	OPDE_3_2_1		(0xe)
#define	OPDE_3_2_1_0		(0xf)

/* SMRDR0 (spi mode) */
/* SMRDR1 (spi mode) */
/* SMWDR0 (spi mode) */
/* SMWDR1 (spi mode) */

/* CMNSR (spi mode) */
#define	CMNSR_SSLF		(1u << 1)
#define	CMNSR_TEND		(1u << 0)

/* DRDMCR (read mode) */
#define	DRDMCR_DMDB(b)		(((u32)(b) & 0x3) << 16)
#define	DRDMCR_DMCYC(b)		(((u32)(b) & 0x7) << 0)

/* DRDRENR (read mode) */
#define	DRDRENR_ADDRE		(1u << 8)
#define	DRDRENR_OPDRE		(1u << 4)
#define	DRDRENR_DRDRE		(1u << 0)

/* SMDMCR (spi mode) */
#define	SMDMCR_DMDB(b)		(((u32)(b) & 0x3) << 16)
#define	SMDMCR_DMCYC(b)		(((u32)(b) & 0x7) << 0)

/* SMDRENR (spi mode) */
#define	SMDRENR_ADDRE		(1u << 8)
#define	SMDRENR_OPDRE		(1u << 4)
#define	SMDRENR_SPIDRE		(1u << 0)

#define	QSPI_BASE_CLK		(133333333)

/*
 *  FlashROM Chip Commands
 */

#define	CMD_READ_ID		(0x9f)	/* (REMS) Read Electronic Manufacturer Signature */
#define	CMD_PP			(0x02)	/* Page Program (3-byte address) */
#define	CMD_QPP			(0x32)	/* Quad Page Program (3-byte address) */
#define	CMD_READ		(0x03)	/* Read (3-byte address) */
#define	CMD_FAST_READ		(0x0b)	/* Fast Read (3-byte address) */
#define	CMD_DOR			(0x3b)	/* Read Dual Out (3-byte address) */
#define	CMD_QOR			(0x6b)	/* Read Quad Out (3-byte address) */
#define	CMD_SE			(0xd8)	/* Sector Erase */

#define	CMD_4READ		(0x13)	/* Read (4-byte address) */
#define	CMD_4FAST_READ		(0x0c)	/* Fast Read (4-byte address) */
#define	CMD_4PP			(0x12)	/* Page Program (4-byte address) */
#define	CMD_4SE			(0xdc)	/* Sector Erase */

static inline struct stRzSpi* to_rz_spi(struct spi_slave* sl)
{
	return container_of(sl, struct stRzSpi, slave);
}

static inline u32 qspi_read32(struct stRzSpi* stRzSpi, int nOff)
{
	return readl((u32)stRzSpi->pRegBase + nOff);
}

static inline u8 qspi_read8(struct stRzSpi* stRzSpi, int nOff)
{
	return readb((u32)stRzSpi->pRegBase + nOff);
}

static inline u16 qspi_read16(struct stRzSpi* stRzSpi, int nOff)
{
	return readw((u32)stRzSpi->pRegBase + nOff);
}

static inline void qspi_write32(struct stRzSpi* stRzSpi, u32 u32Value, int nOff)
{
	writel(u32Value, (u32)stRzSpi->pRegBase + nOff);
}

static inline void qspi_write8(struct stRzSpi* stRzSpi, u8 u8Value, int nOff)
{
	writeb(u8Value, (u32)stRzSpi->pRegBase + nOff);
}

static inline void qspi_write16(struct stRzSpi* stRzSpi, u16 u16Value, int nOff)
{
	writew(u16Value, (u32)stRzSpi->pRegBase + nOff);
}

#endif
