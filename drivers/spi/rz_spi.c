/*
 * RZ SPI driver
 * /drivers/met/spi/rz_spi.c
 *
 * Copyright (C) 2013 NEC Corp.
 *
 * Based on /drivers/met/spi/sh_spi.c
 *
 * Copyright (C) 2011-2014 Renesas Solutions Corp.
 * Copyright (C) 2015 Renesas Electronics America
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

//#define DEBUG
//#define DEBUG_DETAILED /* Print out all TX and RX data */

#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <asm/io.h>
#include "rz_spi.h"

#define	WAIT_HZ			1000	// TEND time out rate.
#define	CMD_READ_ARRAY_FAST	0x0b	// EXT-READ dummy-cmd.

static int qspi_set_config_register(struct stRzSpi*);
static u32 qspi_calc_spbcr(struct stRzSpi* pstRzSpi);
static int qspi_set_ope_mode(struct stRzSpi*,int);
static int qspi_wait_for_tend(struct stRzSpi*);

static int qspi_send_data(struct stRzSpi*,const u8*,unsigned int,unsigned int);
static int qspi_recv_data(struct stRzSpi*,u8*,unsigned int,unsigned int);

int qspi_disable_combine = 0;	// Don't combine responses from dual SPI flash
int qspi_combine_status_mode = 0; // Dual mode only, 0='OR results', 1='AND results'

/**
 * Required function for u-boot SPI subsystem
 */
struct spi_slave* spi_setup_slave(unsigned int unBus, unsigned int unCs,
	unsigned int unMaxHz, unsigned int unMode)
{
	struct stRzSpi* pstRzSpi;

	debug("call %s: bus(%d) cs(%0d) maxhz(%d) mode(%d))\n",
		__func__, unBus, unCs, unMaxHz, unMode);

	pstRzSpi = spi_alloc_slave(struct stRzSpi, unBus, unCs);
	if(pstRzSpi == NULL){
		printf("%s: malloc error.\n", __func__);
		return NULL;
	}

	pstRzSpi->slave.bus		= unBus;
	pstRzSpi->slave.cs		= unCs;
	pstRzSpi->pRegBase		= (void*)CONFIG_RZA1_BASE_QSPI0;
	pstRzSpi->u32MaxSpeedHz		= unMaxHz;
	pstRzSpi->unMode		= unMode;

	/* Save if we were usign 1 or 2 chips in data read mode
	   (so we can put it back when we're done) */
	if( qspi_read32(pstRzSpi, QSPI_CMNCR) & BSZ_DUAL )
		pstRzSpi->data_read_dual = 1;
	else
		pstRzSpi->data_read_dual = 0;

	if( pstRzSpi->slave.cs )
		printf("SF: Dual SPI mode\n");

	qspi_set_config_register(pstRzSpi);

	return &pstRzSpi->slave;
}

/**
 * Required function for u-boot SPI subsystem
 */
void spi_free_slave(struct spi_slave* pstSpiSlave)
{
	debug("call %s: bus(%d) cs(%d)\n",
	__func__, pstSpiSlave->bus, pstSpiSlave->cs);

	free(to_rz_spi(pstSpiSlave));
}

/**
 * Required function for u-boot SPI subsystem
 */
int spi_claim_bus(struct spi_slave* pstSpiSlave)
{
	debug("call %s: bus(%d) cs(%d)\n",
		__func__, pstSpiSlave->bus, pstSpiSlave->cs);
	return 0;
}

/**
 * Required function for u-boot SPI subsystem
 */
void spi_release_bus(struct spi_slave* pstSpiSlave)
{
	debug("call %s: bus(%d) cs(%d)\n",
		__func__, pstSpiSlave->bus, pstSpiSlave->cs);
}

/**
 * Required function for u-boot SPI subsystem
 */
int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	debug("call %s: bus(%d) cs(%d)\n", __func__, bus, cs);

	/* Use cs=1 to mean we want to use 2 spi flashes */
	if (!bus && (cs ==0 || cs==1) )
		return 1;
	else
		return 0;
}

/**
 * Required function for u-boot SPI subsystem
 */
void spi_cs_activate(struct spi_slave* pstSpiSlave)
{
	debug("call %s: bus(%d) cs(%d)\n",
		__func__, pstSpiSlave->bus, pstSpiSlave->cs);
}

/**
 * Required function for u-boot SPI subsystem
 */
void spi_cs_deactivate(struct spi_slave* pstSpiSlave)
{
	debug("call %s: bus(%d) cs(%d)\n",
		__func__, pstSpiSlave->bus, pstSpiSlave->cs);
}

/**
 * Required function for u-boot SPI subsystem
 */
int spi_xfer(struct spi_slave* pstSpiSlave, unsigned int bitlen,
	const void* dout, void* din, unsigned long flags)
{
	struct stRzSpi *pstRzSpi	= to_rz_spi(pstSpiSlave);
	unsigned char* pbTxData		= (unsigned char*)dout;
	unsigned char* pbRxData		= (unsigned char*)din;
	unsigned int len		= (bitlen + 7) / 8;
	int ret				= 0;
	int keep_cs_low			= flags & SPI_XFER_END ? 0 : 1;
	unsigned char dual_cmd[12];

	debug("call %s: bus(%d) cs(%d) bitlen(%d) dout=(0x%08x) din=(0x%08x), flag=(%d)\n",
		__func__, pstSpiSlave->bus, pstSpiSlave->cs,
		bitlen, (u32)dout, (u32)din, (u32)flags);

	if (qspi_wait_for_tend(pstRzSpi) < 0) {
		printf("%s: error waiting for end of last transfer\n", __func__);
		return -1;
	}

	if(flags & SPI_XFER_BEGIN){
		ret = qspi_set_ope_mode(pstRzSpi, SPI_MODE);
		if(ret){
			printf("%s: Unknown SPI mode\n", __func__);
			return 0;
		}
		if ((ret = qspi_wait_for_tend(pstRzSpi)) < 0) {
			printf("%s: error wait for poll (%d)\n", __func__, ret);
			return 0;
		}
	}

	if(pbTxData){
		if(flags & SPI_XFER_BEGIN){
			pstRzSpi->this_cmd = *pbTxData;

			/* If this is a dual SPI Flash, we need to send the same
			   command to both chips. */
			if( pstRzSpi->slave.cs )
			{
				int i,j;
				for(i=0,j=0;i<len;i++) {
					dual_cmd[j++] = pbTxData[i];
					dual_cmd[j++] = pbTxData[i];
				}
				len *= 2;
				pbTxData = dual_cmd;
			}

			ret = qspi_send_data(pstRzSpi, pbTxData, len, keep_cs_low);

			if(ret < 0){
				printf("%s: Error Send Command (%x)\n", __func__, ret);
			}else{
#ifdef DEBUG
				int nIndex;
				debug("send cmd : ");
				for(nIndex = 0; nIndex < len; nIndex++){
					debug(" %02x", *(pbTxData + nIndex));
				}
				if(flags & SPI_XFER_END) debug(" <END>");
				debug("\n");
#endif
			}
		}else{

			/* If this is a dual SPI Flash, we need to send the same
			   command data to both chips. */
			if( pstRzSpi->slave.cs )
			{
				int i,j;
				switch (pstRzSpi->this_cmd) {
					case 0x17: /* Write Bank register (CMD_BANKADDR_BRWR) */
					case 0xC5: /* Write Bank register (CMD_EXTNADDR_WREAR) */
					case 0x01: /* Write Status and configuration */
					case 0xB1: /* Write NonVolatile Configuration register (Micron) */
					case 0x81: /* Write Volatile Configuration register (Micron) */
						for(i=0,j=0;i<len;i++) {
							dual_cmd[j++] = pbTxData[i];
							dual_cmd[j++] = pbTxData[i];
						}
						len *= 2;
						pbTxData = dual_cmd;
						break;
				}
			}

			ret = qspi_send_data(pstRzSpi, pbTxData, len, keep_cs_low);
			if(ret < 0){
				printf("%s: Error Send Data (%x)\n", __func__, ret);
			}else{
#ifdef DEBUG_DETAILED
				int nIndex;
				debug("send dat : ");
				for(nIndex = 0; nIndex < len; nIndex++){
					debug(" %02x", *(pbTxData + nIndex));
					if(nIndex > 100) {
						printf("\n\tStopped after displaying 100 bytes\n");
						break;
					}

				}
				if(flags & SPI_XFER_END) debug(" <END>");
				debug("\n");
#endif
			}
		}
	}
	if(ret == 0 && pbRxData){

		ret = qspi_recv_data(pstRzSpi, pbRxData, len, keep_cs_low);
		if(ret < 0){
			printf("%s: Error Recv Data (%x)\n", __func__, ret);
		}else{
#ifdef	DEBUG_DETAILED
			int nIndex;
			debug("recv : ");
			for(nIndex = 0; nIndex < len; nIndex++){
				debug(" %02x", *(pbRxData + nIndex));
				if(nIndex > 100) {
					printf("\n\tStopped after displaying 100 bytes\n");
					break;
				}
			}
			if(flags & SPI_XFER_END) debug(" <END>");
			debug("\n");
#endif
		}
	}

	if(flags & SPI_XFER_END){
		/* Make sure CS goes back low (it might have been left high
		   from the last transfer). It's tricky because basically,
		   you have to disable RD and WR, then start a dummy transfer. */
		qspi_write32(pstRzSpi, 1 , QSPI_SMCR);

		ret = qspi_set_ope_mode(pstRzSpi, READ_MODE);
		if(ret){
			printf("%s: Unknown SPI mode\n", __func__);
		}
		if ((ret = qspi_wait_for_tend(pstRzSpi)) < 0) {
			printf("%s: error wait for poll (%d)\n", __func__, ret);
		}
	}

	return ret;
}

/* This function is called when "sf probe" is issued, meaning that
   the user wants to access the deivcce in normal single wire SPI mode.
   Since some SPI devices have to have special setups to enable QSPI mode
   or DDR QSPI mode, this function is used to reset those settings
   back to normal.

   This is a 'weak' function because it is intended that each board
   implements its own function to overide this one. */
struct spi_flash;
__attribute__((weak))
int qspi_reset_device(struct spi_flash *flash)
{
	printf("Warning: You should implement your own qspi_reset_device function\n");
	return 0;
}

/**
 * qspi_disable_auto_combine
 *
 * This function is useful only when you are in dual SPI flash mode
 * and you want to send a command but not have the results from
 * the 2 devices OR-ed together (becase you need to check each SPI
 * Flash individually).
 *
 * Just remember that you need to send a buffer size big enough to handle
 * the results from both SPI flashes.
 *
 * This only effects the very next command sent, after that it will
 * automatically reset.
 *
 * NOTE: You will need to add a prototype of this function in your
 * code to use it (it's not in any header file).
 */
void qspi_disable_auto_combine(void)
{
	qspi_disable_combine = 1;
}


/**
 *
 */
static inline int qspi_is_ssl_negated(struct stRzSpi* pstRzSpi)
{
	return !(qspi_read32(pstRzSpi, QSPI_CMNSR) & CMNSR_SSLF);
}


/**
 *
 */
static int qspi_set_config_register(struct stRzSpi* pstRzSpi)
{
	u32 value;

	debug("call %s:\n", __func__);

	/* Check if SSL still low */
	if (!qspi_is_ssl_negated(pstRzSpi)){
		/* Clear the SSL from the last data read operation */
		qspi_write32(pstRzSpi, qspi_read32(pstRzSpi,QSPI_DRCR) | DRCR_SSLN,
			QSPI_DRCR);

		/* Disable Read & Write */
		qspi_write32(pstRzSpi, 0, QSPI_SMCR);	
	}

	/* NOTES: Set swap (SFDE) so the order of bytes D0 to D7 in the SPI RX/TX FIFO are always in the
	   same order (LSB=D0, MSB=D7) regardless if the SPI did a byte,word, dwrod fetch */
	value = 
		CMNCR_MD|	       		/* spi mode */
		CMNCR_SFDE|			/* swap */
		CMNCR_MOIIO3(OUT_HIZ)|
		CMNCR_MOIIO2(OUT_HIZ)|
		CMNCR_MOIIO1(OUT_HIZ)|
		CMNCR_MOIIO0(OUT_HIZ)|
		CMNCR_IO3FV(OUT_HIZ)|
		CMNCR_IO2FV(OUT_HIZ)|
		CMNCR_IO0FV(OUT_HIZ)|
		//CMNCR_CPHAR|CMNCR_CPHAT|CMNCR_CPOL| /* spi mode3 */
		CMNCR_CPHAR|
		CMNCR_BSZ(BSZ_SINGLE);

	/* dual memory? */
	if (pstRzSpi->slave.cs)
		value |= CMNCR_BSZ(BSZ_DUAL);	/* s-flash x 2 */

	/* set common */
	qspi_write32(pstRzSpi, value, QSPI_CMNCR);

	/* flush read-cache */
	qspi_write32(pstRzSpi, qspi_read32(pstRzSpi, QSPI_DRCR) | DRCR_RCF,
		QSPI_DRCR);

	/* setup delay */
	qspi_write32(pstRzSpi,
		SSLDR_SPNDL(SPBCLK_1_0)|	/* next access delay */
		SSLDR_SLNDL(SPBCLK_1_0)|	/* SPBSSL negate delay */
		SSLDR_SCKDL(SPBCLK_1_0),	/* clock delay */
		QSPI_SSLDR);

	/* sets transfer bit rate */
	qspi_write32(pstRzSpi, qspi_calc_spbcr(pstRzSpi), QSPI_SPBCR);

	return 0;
}

/**
 *
 */
static u32 qspi_calc_spbcr(struct stRzSpi* pstRzSpi)
{
	u32 n = 1; /* FIX for clk / 2 */
	u32 N = 0;
#if 0
	u32 hz;
	/*
	 * bitrate = clk / ( 2 x n x 2 ^ N)
	 * n=SPBCR_SPBR[7:0](1 ... 255), N=SPBCR_BRDV[1:0]
	 */
	hz = QSPI_BASE_CLK / pstRzSpi->u32MaxSpeedHz;
	if (hz >= (1 << 5))  { N = 3;}
	else if (!(hz & 1))  { N = 0;}
	else if (!(hz & 3))  { N = 1;}
	else if (!(hz & 7))  { N = 2;}
	else if (!(hz & 15)) { N = 3;}
	n = hz / (1 << N);
	n = n > 255 ? 255 : n;
#endif
	return SPBCR_SPBR(n)|SPBCR_BRDV(N);
}

/**
 *
 */
static int qspi_set_ope_mode(struct stRzSpi* pstRzSpi, int mode)
{
	int ret;
	u32 cmncr = qspi_read32(pstRzSpi, QSPI_CMNCR);
	u32 drcr;

	debug("call %s: mode=%d(%s)\n", __func__, mode, mode==SPI_MODE ? "SPI": "XIP");

	if((mode == SPI_MODE) && (cmncr & CMNCR_MD)){
		debug("Already in correct mode\n");
		return 0;
	}
	if((mode != SPI_MODE) && !(cmncr & CMNCR_MD)){
		debug("Already in correct mode\n");
		return 0;
	}

	if(!qspi_is_ssl_negated(pstRzSpi)){
		/* SSL still low from last transfer */
		/* Disable Read & Write */
		qspi_write32(pstRzSpi, 0, QSPI_SMCR);	
		/* Clear the SSL from the last data read operation */
		qspi_write32(pstRzSpi, qspi_read32(pstRzSpi,QSPI_DRCR) | DRCR_SSLN,
			QSPI_DRCR);
	}

	ret = qspi_wait_for_tend(pstRzSpi);
	if(ret){
		printf("%s: hw busy\n", __func__);
		return ret;
	}

	if(mode == SPI_MODE){
		// SPI mode.
		cmncr |= CMNCR_MD;

		/* cs=0 is single chip, cs=1 is dual chip */
		if( pstRzSpi->slave.cs )
			cmncr |= BSZ_DUAL;
		else
			cmncr &= ~BSZ_DUAL;

		qspi_write32(pstRzSpi, cmncr, QSPI_CMNCR);

	}else{

		/* End the transfer by putting SSL low */
		qspi_write32(pstRzSpi, DRCR_SSLE, QSPI_DRCR);

		// EXT-READ mode.
		cmncr &= ~CMNCR_MD;

		// put back in same mode (1 or 2 chips) as was originally
		if( pstRzSpi->data_read_dual )
			cmncr |= BSZ_DUAL;
		else
			cmncr &= ~BSZ_DUAL;

		qspi_write32(pstRzSpi, cmncr, QSPI_CMNCR);

		// Flush cache
		drcr = qspi_read32(pstRzSpi, QSPI_DRCR);
		drcr |= DRCR_RCF;
		qspi_write32(pstRzSpi, drcr, QSPI_DRCR);
	}

	return 0;
}

/**
 *
 */
static int qspi_wait_for_tend(struct stRzSpi* pstRzSpi)
{
	unsigned long timebase;

	timebase = get_timer(0);
	do{
		if(qspi_read32(pstRzSpi, QSPI_CMNSR) & CMNSR_TEND){
			break;
		}
	}while(get_timer(timebase) < WAIT_HZ);

	if (!(qspi_read32(pstRzSpi, QSPI_CMNSR) & CMNSR_TEND)){
		printf("%s: wait timeout\n", __func__);
		return -1;
	}

	return 0;
}

const u8 SPIDE_for_single[5] = {0x0,
		0x8,	// 8-bit transfer (1 bytes)
		0xC,	// 16-bit transfer (2 bytes)
		0x0,	// 24-bit transfers are invalid!
		0xF};	// 32-bit transfer (3-4 bytes)
const u8 SPIDE_for_dual[9] = {0,
		0x0,	// 8-bit transfers are invalid!
		0x8,	// 16-bit transfer (1 byte)
		0x0,	// 24-bit transfers are invalid!
		0xC,	// 32-bit transfer (4 bytes)
		0x0,	// 40-bit transfers are invalid!
		0x0,	// 48-bit transfers are invalid!
		0x0,	// 56-bit transfers are invalid!
		0xF};	// 64-bit transfer (8 bytes)
/**
 *
 */
static int qspi_send_data(struct stRzSpi *pstRzSpi,
	const u8* pcnu8DataBuff, unsigned int unDataLength, unsigned int keep_cs_low)
{
	int ret;
	u32 smcr;
	u32 smenr = 0;
	u32 smwdr0;
	u32 smwdr1 = 0;
	int unit;
	int sslkp = 1;

	debug("call %s:\n", __func__);

	/* wait spi transfered */
	if ((ret = qspi_wait_for_tend(pstRzSpi)) < 0) {
		printf("%s: prev xmit timeout\n", __func__);
		return ret;
	}

	while (unDataLength > 0) {

		if( pstRzSpi->slave.cs ) {
			/* Dual memory */
			if (unDataLength >= 8)
				unit = 8;
			else
				unit = unDataLength;

			if( unit & 1 ) {
				printf("ERROR: Can't send odd number of bytes in dual memory mode\n");
				return -1;
			}

			if( unit == 6 )
				unit = 4; /* 6 byte transfers not supported */

			smenr &= ~0xF;	/* clear SPIDE bits */
			smenr |= SPIDE_for_dual[unit];
		}
		else {
			/* Single memory */
			if (unDataLength >= 4)
				unit = 4;
			else
				unit = unDataLength;
			if(unit == 3)
				unit = 2;	/* 3 byte transfers not supported */

			smenr &= ~0xF;	/* clear SPIDE bits */
			smenr |= SPIDE_for_single[unit];
		}

		if( !pstRzSpi->slave.cs ) {
			/* Single memory */

			/* set data */
			smwdr0 = (u32)*pcnu8DataBuff++;
			if (unit >= 2)
			smwdr0 |= (u32)*pcnu8DataBuff++ << 8;
			if (unit >= 3)
			smwdr0 |= (u32)*pcnu8DataBuff++ << 16;
			if (unit >= 4)
			smwdr0 |= (u32)*pcnu8DataBuff++ << 24;
		}
		else
		{
			/* Dual memory */
			if( unit == 8 ) {
				/* Note that SMWDR1 gets sent out first
				   when sending 8 bytes */
				smwdr1 = (u32)*pcnu8DataBuff++;
				smwdr1 |= (u32)*pcnu8DataBuff++ << 8;
				smwdr1 |= (u32)*pcnu8DataBuff++ << 16;
				smwdr1 |= (u32)*pcnu8DataBuff++ << 24;
			}
			/* sending 2 bytes */
			smwdr0 = (u32)*pcnu8DataBuff++;
			smwdr0 |= (u32)*pcnu8DataBuff++ << 8;

			/* sending 4 bytes */
			if( unit >= 4) {
				smwdr0 |= (u32)*pcnu8DataBuff++ << 16;
				smwdr0 |= (u32)*pcnu8DataBuff++ << 24;
			}
		}

		/* Write data to send */
		if (unit == 2){
			qspi_write16(pstRzSpi, (u16)smwdr0, QSPI_SMWDR0);
		}
		else if (unit == 1){
			qspi_write8(pstRzSpi, (u8)smwdr0, QSPI_SMWDR0);
		}
		else {
			qspi_write32(pstRzSpi, smwdr0, QSPI_SMWDR0);
		}

		if( unit == 8 ) {
			/* Dual memory only */
			qspi_write32(pstRzSpi, smwdr1, QSPI_SMWDR1);
		}

		unDataLength -= unit;
		if (unDataLength <= 0) {
			sslkp = 0;
		}

		/* set params */
		qspi_write32(pstRzSpi, 0, QSPI_SMCMR);
		qspi_write32(pstRzSpi, 0, QSPI_SMADR);
		qspi_write32(pstRzSpi, 0, QSPI_SMOPR);
		qspi_write32(pstRzSpi, smenr, QSPI_SMENR);

		/* start spi transfer */
		smcr = SMCR_SPIE|SMCR_SPIWE;
		if (sslkp || keep_cs_low)
			smcr |= SMCR_SSLKP;
		qspi_write32(pstRzSpi, smcr, QSPI_SMCR);

		/* wait spi transfered */
		if ((ret = qspi_wait_for_tend(pstRzSpi)) < 0) {
			printf("%s: data send timeout\n", __func__);
			return ret;
		}

	}
	return 0;
}

/**
 *
 */
static int qspi_recv_data(struct stRzSpi* pstRzSpi,
	u8* pu8DataBuff, unsigned int unDataLength, unsigned int keep_cs_low)
{
	int ret;
	u32 smcr;
	u32 smenr = 0;
	u32 smrdr0;
	u32 smrdr1 = 0;
	int unit;
	int sslkp = 1;
	u8 combine = 0;

	debug("call %s:\n", __func__);

	/* wait spi transfered */
	if ((ret = qspi_wait_for_tend(pstRzSpi)) < 0) {
		printf("%s: prev xmit timeout\n", __func__);
		return ret;
	}

	/* When receiving data from a command, we need to take into account
	   that there are 2 SPI devices (that each will return values) */
	if( pstRzSpi->slave.cs && !qspi_disable_combine)
	{
		switch (pstRzSpi->this_cmd) {
			case 0x9F: /* Read ID */
			case 0x05: /* Read Status register (CMD_READ_STATUS) */
			case 0x70: /* Read Status register (CMD_FLAG_STATUS) */
			case 0x35: /* Read configuration register */
			case 0x16: /* Read Bank register (CMD_BANKADDR_BRRD) */
			case 0xC8: /* Read Bank register (CMD_EXTNADDR_RDEAR) */
			case 0xB5: /* Read NONVolatile Configuration register (Micron) */
			case 0x85: /* Read Volatile Configuration register (Micron) */
				combine = 1;
				unDataLength *= 2;	// get twice as much data.
				break;
		}
	}

	/* Flash devices with this command wait for bit 7 to go high (not LOW) when
	   erase or writting is done, so we need to AND the results, not OR them,
	   when running in dual SPI flash mode */
	if( pstRzSpi->this_cmd == 0x70 )
		qspi_combine_status_mode = 1; /* AND results (WIP = 1 when ready) */
	else
		qspi_combine_status_mode = 0; /* OR resutls (WIP = 0 when ready) */

	/* Reset after each command */
	qspi_disable_combine = 0;

	while (unDataLength > 0) {
		if( pstRzSpi->slave.cs ) {
			/* Dual memory */
			if (unDataLength >= 8)
				unit = 8;
			else
				unit = unDataLength;

			if( unit & 1 ) {
				printf("ERROR: Can't read odd number of bytes in dual memory mode\n");
				return -1;
			}

			if( unit == 6 )
				unit = 4; /* 6 byte transfers not supported, do 4 then 2 */

			smenr = SPIDE_for_dual[unit];
		}
		else {
			/* Single memory */
			if (unDataLength >= 4)
				unit = 4;
			else
				unit = unDataLength;
			if(unit == 3)
				unit = 2;	/* 3 byte transfers not supported */

			smenr = SPIDE_for_single[unit];
		}

		unDataLength -= unit;
		if (unDataLength <= 0) {
			sslkp = 0;	/* Last transfer */
		}

		/* set params */
		qspi_write32(pstRzSpi, 0, QSPI_SMCMR);
		qspi_write32(pstRzSpi, 0, QSPI_SMADR);
		qspi_write32(pstRzSpi, 0, QSPI_SMOPR);
		qspi_write32(pstRzSpi, smenr, QSPI_SMENR);

		/* start spi transfer */
		smcr = SMCR_SPIE|SMCR_SPIRE|SMCR_SPIWE;
		if (sslkp | keep_cs_low )
			smcr |= SMCR_SSLKP;
		qspi_write32(pstRzSpi, smcr, QSPI_SMCR);

		/* wait spi transfered */
		if ((ret = qspi_wait_for_tend(pstRzSpi)) < 0) {
			printf("%s: data recive timeout\n", __func__);
			return ret;
		}

		/* Just read both regsiters. We'll figure out what parts
		   are valid later */
		smrdr0 = qspi_read32(pstRzSpi, QSPI_SMRDR0);
		smrdr1 = qspi_read32(pstRzSpi, QSPI_SMRDR1);

		if( !combine ) {
			if (unit == 8) {
				/* Dual Memory */
				/* SMDR1 has the begining of the RX data (but
				   only when 8 bytes are being read) */
				*pu8DataBuff++ = (u8)(smrdr1 & 0xff);
				*pu8DataBuff++ = (u8)((smrdr1 >> 8) & 0xff);
				*pu8DataBuff++ = (u8)((smrdr1 >> 16) & 0xff);
				*pu8DataBuff++ = (u8)((smrdr1 >> 24) & 0xff);
			}

			*pu8DataBuff++ = (u8)(smrdr0 & 0xff);
			if (unit >= 2) {
				*pu8DataBuff++ = (u8)((smrdr0 >> 8) & 0xff);
			}
			if (unit >= 4) {
				*pu8DataBuff++ = (u8)((smrdr0 >> 16) & 0xff);
				*pu8DataBuff++ = (u8)((smrdr0 >> 24) & 0xff);
			}

		}
		else {
			/* Dual Memory - Combine 2 streams back into 1 */
			/* OR/AND together the data coming back so the WIP bit can be
			   checked for erase/write operations */
			/* Combine results together */
			if ( unit == 8 ) {
				/* SMRDR1 always has the begining of the RX data stream */
				if( qspi_combine_status_mode) { /* AND results together */
					*pu8DataBuff++ = (u8)(smrdr1 & 0xff) & (u8)((smrdr1 >> 8) & 0xff);
					*pu8DataBuff++ = (u8)((smrdr1 >> 16) & 0xff) & (u8)((smrdr1 >> 24) & 0xff);
					*pu8DataBuff++ = (u8)(smrdr0 & 0xff) & (u8)((smrdr0 >> 8) & 0xff);
					*pu8DataBuff++ = (u8)((smrdr0 >> 16) & 0xff) & (u8)((smrdr0 >> 24) & 0xff);
				}
				else {	/* OR results together */
					*pu8DataBuff++ = (u8)(smrdr1 & 0xff) | (u8)((smrdr1 >> 8) & 0xff);
					*pu8DataBuff++ = (u8)((smrdr1 >> 16) & 0xff) | (u8)((smrdr1 >> 24) & 0xff);
					*pu8DataBuff++ = (u8)(smrdr0 & 0xff) | (u8)((smrdr0 >> 8) & 0xff);
					*pu8DataBuff++ = (u8)((smrdr0 >> 16) & 0xff) | (u8)((smrdr0 >> 24) & 0xff);
				}
			}

			if( unit == 2 ) {
				if( qspi_combine_status_mode) { /* AND results together */
					*pu8DataBuff++ = (u8)(smrdr0 & 0xff) & (u8)((smrdr0 >> 8) & 0xff);
				}
				else {	/* OR results together */
					*pu8DataBuff++ = (u8)(smrdr0 & 0xff) | (u8)((smrdr0 >> 8) & 0xff);
				}

			}
			if (unit == 4) {
				if( qspi_combine_status_mode) { /* AND results together */
					*pu8DataBuff++ = (u8)(smrdr0 & 0xff) & (u8)((smrdr0 >> 8) & 0xff);
					*pu8DataBuff++ = (u8)((smrdr0 >> 16) & 0xff) & (u8)((smrdr0 >> 24) & 0xff);
				}
				else {	/* OR results together */
					*pu8DataBuff++ = (u8)(smrdr0 & 0xff) | (u8)((smrdr0 >> 8) & 0xff);
					*pu8DataBuff++ = (u8)((smrdr0 >> 16) & 0xff) | (u8)((smrdr0 >> 24) & 0xff);
				}
			}
		}
	}

	return 0;
}

