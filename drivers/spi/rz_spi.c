/*
 * RZ SPI driver
 * /drivers/met/spi/rz_spi.c
 *
 * Copyright (C) 2013 NEC Corp.
 *
 * Based on /drivers/met/spi/sh_spi.c
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
static void qspi_set_busio(struct stRzSpi*,u8);
static int qspi_send_cmd(struct stRzSpi*,const u8*,unsigned int,int);
static int qspi_send_data(struct stRzSpi*,const u8*,unsigned int);
static int qspi_recv_data(struct stRzSpi*,u8*,unsigned int);

/**
 *
 */
struct spi_slave* spi_setup_slave(unsigned int unBus, unsigned int unCs,
	unsigned int unMaxHz, unsigned int unMode)
{
	struct stRzSpi* pstRzSpi;

	debug("call %s: bus(%d) cs(%0d) maxhz(%d) mode(%d))\n",
		__func__, unBus, unCs, unMaxHz, unMode);

	pstRzSpi = malloc(sizeof(struct stRzSpi));
	if(pstRzSpi == NULL){
		printf("%s: malloc error.\n", __func__);
		return NULL;
	}

	pstRzSpi->stSpiSlave.bus	= unBus;
	pstRzSpi->stSpiSlave.cs		= unCs;
	pstRzSpi->pRegBase		= (void*)CONFIG_RZA1_BASE_QSPI0;
	pstRzSpi->u8BitsPerWord		= 8;
	pstRzSpi->u32MaxSpeedHz		= unMaxHz;
	pstRzSpi->unMode		= unMode;
	pstRzSpi->u32DataBitw		= BITW_1BIT;
	pstRzSpi->u32DummyCycle		= 0;

	/* Save if we were usign 1 or 2 chips in data read mode
	   (so we can put it back when we're done) */
	if( qspi_read32(pstRzSpi, QSPI_CMNCR) & BSZ_DUAL )
		pstRzSpi->data_read_dual = 1;
	else
		pstRzSpi->data_read_dual = 0;

	if( pstRzSpi->stSpiSlave.cs )
		printf("SF: Dual SPI mode\n");

	if (pstRzSpi->u8BitsPerWord == 0)
		pstRzSpi->u8BitsPerWord = 8;

	qspi_set_config_register(pstRzSpi);

	return (struct spi_slave*)pstRzSpi;
}

/**
 *
 */
void spi_free_slave(struct spi_slave* pstSpiSlave)
{
	debug("call %s: bus(%d) cs(%d)\n",
	__func__, pstSpiSlave->bus, pstSpiSlave->cs);

	free(to_rz_spi(pstSpiSlave));
}

/**
 *
 */
int spi_claim_bus(struct spi_slave* pstSpiSlave)
{
	debug("call %s: bus(%d) cs(%d)\n",
		__func__, pstSpiSlave->bus, pstSpiSlave->cs);
	return 0;
}

/**
 *
 */
void spi_release_bus(struct spi_slave* pstSpiSlave)
{
	debug("call %s: bus(%d) cs(%d)\n",
		__func__, pstSpiSlave->bus, pstSpiSlave->cs);
}

/**
 *
 */
int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	debug("call %s: bus(%d) cs(%d)\n", __func__, bus, cs);

	/* This driver supports "bus = 0" and "cs = 0" only. */
//	if (!bus && !cs)

	// use cs=1 to mean we want to use 2 spi flashes
	if (!bus && (cs ==0 || cs==1) )
		return 1;
	else
		return 0;
}

/**
 *
 */
void spi_cs_activate(struct spi_slave* pstSpiSlave)
{
	debug("call %s: bus(%d) cs(%d)\n",
		__func__, pstSpiSlave->bus, pstSpiSlave->cs);
}

/**
 *
 */
void spi_cs_deactivate(struct spi_slave* pstSpiSlave)
{
	debug("call %s: bus(%d) cs(%d)\n",
		__func__, pstSpiSlave->bus, pstSpiSlave->cs);
}

/**
 *
 */
int spi_xfer(struct spi_slave* pstSpiSlave, unsigned int bitlen,
	const void* dout, void* din, unsigned long flags)
{
	struct stRzSpi *pstRzSpi	= to_rz_spi(pstSpiSlave);
	const unsigned char* pbTxData	= (const unsigned char*)dout;
	unsigned char* pbRxData		= (unsigned char*)din;
	unsigned int len		= (bitlen + 7) / 8;
	int ret				= 0;


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
			qspi_set_busio(pstRzSpi, *pbTxData);
			pstRzSpi->this_cmd = *pbTxData;
			ret = qspi_send_cmd(pstRzSpi, pbTxData, len,
				(flags & SPI_XFER_END) ? 0 : 1);
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
			ret = qspi_send_data(pstRzSpi, pbTxData, len);
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

		ret = qspi_recv_data(pstRzSpi, pbRxData, len);
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
	if (pstRzSpi->stSpiSlave.cs)
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

	debug("call %s: mode=%d\n", __func__, mode);

	if((mode == SPI_MODE) && (cmncr & CMNCR_MD)){
		return 0;
	}
	if((mode != SPI_MODE) && !(cmncr & CMNCR_MD)){
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
		if( pstRzSpi->stSpiSlave.cs )
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
#if 0

//Dont touch anything else...just leave it...
		qspi_write32(pstRzSpi, DRCR_SSLE, QSPI_DRCR);
		ret = qspi_wait_for_tend(pstRzSpi);
		if(ret){
			printf("%s: hw busy\n", __func__);
			return ret;
		}else{
			u32 u32DRENR, u32DRCMR, u32DROPR;

			u32DRENR = u32DRCMR = u32DROPR = 0;
			// Command
#if 1
			u32DRCMR |= DRCMR_CMD(CMD_4FAST_READ);
#else
			u32DRCMR |= DRCMR_CMD(CMD_READ_ARRAY_FAST);
#endif
			u32DRENR |= DRENR_CDB(BITW_1BIT);
			u32DRENR |= DRENR_CDE;
			// Option
			u32DRCMR |= DRCMR_OCMD(0);
			u32DRENR |= DRENR_OCDB(BITW_1BIT);
			// Address
			u32DRENR |= DRENR_ADB(BITW_1BIT);
			u32DRENR |= DRENR_ADE(ADE_31_0);
			// Option Data
			u32DROPR |= DROPR_OPD3(0x00);
			u32DROPR |= DROPR_OPD2(0x00);
			u32DROPR |= DROPR_OPD1(0x00);
			u32DROPR |= DROPR_OPD0(0x00);
			u32DRENR |= DRENR_OPDB(BITW_1BIT);
			u32DRENR |= DRENR_OPDE(OPDE_3);
			// Data
			u32DRENR |= DRENR_DRDB(BITW_1BIT);

			qspi_write32(pstRzSpi, u32DRCMR, QSPI_DRCMR);
			qspi_write32(pstRzSpi, u32DROPR, QSPI_DROPR);
			qspi_write32(pstRzSpi, u32DRENR, QSPI_DRENR);
		}
#endif

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

/**
 *
 */
static void qspi_set_busio(struct stRzSpi* pstRzSpi,u8 cmd)
{
	u32 data_bitw;
	u32 dmy_cycle;

	debug("call %s:\n", __func__);

	switch (cmd) {
	case CMD_FAST_READ:	/* 0x0b Fast Read (3-byte address) */
	case CMD_4FAST_READ:	/* 0x0b Fast Read (3-byte address) */
		dmy_cycle = 8;
		data_bitw = BITW_1BIT;
		break;
	case CMD_QOR:		/* 0x6b Read Quad Out (3-byte address) */
		dmy_cycle = 8;
		data_bitw = BITW_4BIT;
		break;
	case CMD_QPP:		/* 0x32 Quad Page Program (3-byte address) */
		dmy_cycle = 0;
		data_bitw = BITW_4BIT;
		break;
	default:
		data_bitw = BITW_1BIT;
		dmy_cycle = 0;
		break;
	}

	pstRzSpi->u32DataBitw	= data_bitw;
	pstRzSpi->u32DummyCycle	= dmy_cycle;

	return;
}

/**
 *
 */
static int qspi_send_cmd(struct stRzSpi* pstRzSpi, const u8* pcnu8CmdBuff,
	unsigned int unCmdLength, int sslkp)
{
	int ret;
	u32 cmd;
	u32 addr = 0;
	u32 smopr = 0;
	u32 smenr = 0;
	u32 smcr;
	u32 smdmcr = 0;

	debug("call %s:\n", __func__);

	/* wait spi transfered */
	if ((ret = qspi_wait_for_tend(pstRzSpi)) < 0) {
		printf("%s: prev xmit timeout\n", __func__);
		return ret;
	}
	/* set command */
	cmd = SMCMR_CMD(*(pcnu8CmdBuff + 0));

	/* set enable command, 1bit stream. */
	smenr = SMENR_CDE|SMENR_CDB(BITW_1BIT);

	if (unCmdLength == 3) {	/* set option data  */
		// set option data.
		smopr |= SMOPR_OPD3(*(pcnu8CmdBuff + 1));
		smopr |= SMOPR_OPD2(*(pcnu8CmdBuff + 2));
		// set OPT3 OPT2, 1bit stream.
		smenr |= SMENR_OPDE(OPDE_3_2)|SMENR_OPDB(BITW_1BIT);
	}
	if (unCmdLength == 2) {	/* set option data */
		// set option data.
		smopr |= SMOPR_OPD3(*(pcnu8CmdBuff + 1));
		// OPD3, 1bit stream.
		smenr |= SMENR_OPDE(OPDE_3)|SMENR_OPDB(BITW_1BIT);
	}
	if (unCmdLength >= 5) {	/* set address */
		// set address param.
		addr =	((u32)*(pcnu8CmdBuff + 1) << 24) |
			((u32)*(pcnu8CmdBuff + 2) << 16) |
			((u32)*(pcnu8CmdBuff + 3) << 8)  |
			(u32)*(pcnu8CmdBuff + 4);
		// 32bits address, 1bit stream
		smenr |= SMENR_ADE(ADE_31_0)|SMENR_ADB(BITW_1BIT);
		// set dummy param.
		if (pstRzSpi->u32DummyCycle > 0) {
			smenr |= SMENR_DME;
			smdmcr = SMDMCR_DMDB(BITW_1BIT)|SMDMCR_DMCYC(pstRzSpi->u32DummyCycle - 1);
		}
	}
	/* set params */
	qspi_write32(pstRzSpi, cmd,	QSPI_SMCMR);	// write command.
	qspi_write32(pstRzSpi, addr,	QSPI_SMADR);	// write address.
	qspi_write32(pstRzSpi, smopr,	QSPI_SMOPR);	// write option.
	qspi_write32(pstRzSpi, smenr,	QSPI_SMENR);	// write enable.
	qspi_write32(pstRzSpi, smdmcr,	QSPI_SMDMCR);	// write dummy cycle.

	/* Set SPI Data Transfer Enable first */
	smcr = SMCR_SPIWE;
	if (sslkp)
		smcr |= SMCR_SSLKP;	/* keep SSL low after transfer */
	qspi_write32(pstRzSpi, smcr, QSPI_SMCR);

	/* start spi transfer */
	smcr |= SMCR_SPIE;
	qspi_write32(pstRzSpi, smcr, QSPI_SMCR);	// write control byte

	return 0;
}

/**
 *
 */
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

static int qspi_send_data(struct stRzSpi *pstRzSpi,
	const u8* pcnu8DataBuff, unsigned int unDataLength)
{
	int ret;
	u32 smcr;
	u32 smenr = 0;
	u32 smwdr0;
	u32 smwdr1 = 0;
	int unit;
	int sslkp = 1;
	u8 data[10];

	debug("call %s:\n", __func__);

	/* wait spi transfered */
	if ((ret = qspi_wait_for_tend(pstRzSpi)) < 0) {
		printf("%s: prev xmit timeout\n", __func__);
		return ret;
	}
	/* Unless we are reading or writting data, we need to take
	   into account that there are 2 SPI devices */
	if( pstRzSpi->stSpiSlave.cs )
	{
		switch (pstRzSpi->this_cmd) {
			case 0x01: /* Write Status and configuration */
				/* duplicate our data to both chips */
				data[0] = pcnu8DataBuff[0];
				data[1] = pcnu8DataBuff[0];
				data[2] = pcnu8DataBuff[1];
				data[3] = pcnu8DataBuff[1];
				pcnu8DataBuff = data;
				unDataLength = 4;
				break;
		}
	}

	while (unDataLength > 0) {

		if( pstRzSpi->stSpiSlave.cs ) {
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

			smenr |= SPIDE_for_single[unit];
		}

		if( !pstRzSpi->stSpiSlave.cs ) {
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

#if 0 // is this code really needed????
		/* mask unwrite area */
		if (unit == 3) {
			smwdr0 |= 0xFF000000;
		} else if (unit == 2) {
			smwdr0 |= 0xFFFF0000;
		} else if (unit == 1) {
			smwdr0 |= 0xFFFFFF00;
		}

		if( pstRzSpi->stSpiSlave.cs ) {
			/* Dual memory */
			if (unit == 7) {
				smwdr1 |= 0xFF000000;
			} else if (unit == 6) {
				smwdr1 |= 0xFFFF0000;
			} else if (unit == 5) {
				smwdr1 |= 0xFFFFFF00;
			}
		}
#endif
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
		if (sslkp)
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
	u8* pu8DataBuff, unsigned int unDataLength)
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

	/* Unless we are reading or writting data, we need to take
	   into account that there are 2 SPI devices */
	if( pstRzSpi->stSpiSlave.cs )
	{
		switch (pstRzSpi->this_cmd) {
			case 0x9F: /* Read ID */
			case 0x05: /* Read Status register */
			case 0x35: /* Read configuration register */
				combine = 1;
				unDataLength *= 2;	// get twice as much data.
				break;
		}
	}
	while (unDataLength > 0) {
		if( pstRzSpi->stSpiSlave.cs ) {
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
		smcr = SMCR_SPIE|SMCR_SPIRE;
		if (pstRzSpi->u32DataBitw == BITW_1BIT)
			smcr |= SMCR_SPIWE;
		if (sslkp)
			smcr |= SMCR_SSLKP;
		qspi_write32(pstRzSpi, smcr, QSPI_SMCR);

		/* wait spi transfered */
		if ((ret = qspi_wait_for_tend(pstRzSpi)) < 0) {
			printf("%s: data recive timeout\n", __func__);
			return ret;
		}

	#if 0
		if (unit == 2)
			smrdr0 = qspi_read16(pstRzSpi, QSPI_SMRDR0);
		else if (unit == 1)
			smrdr0 = qspi_read8(pstRzSpi, QSPI_SMRDR0);
		else
			smrdr0 = qspi_read32(pstRzSpi, QSPI_SMRDR0);

		if( unit == 8 ) {
			/* Dual Memory */
			smrdr1 = qspi_read32(pstRzSpi, QSPI_SMRDR1);
		}
	#else
		/* Just read both regsiters. We'll figure out what parts
		   are valid later */
		smrdr0 = qspi_read32(pstRzSpi, QSPI_SMRDR0);
		smrdr1 = qspi_read32(pstRzSpi, QSPI_SMRDR1);
	#endif

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
			/* OR together the data coming back so the WIP bit can be
			   checked for erase/write operations */
			if ( unit == 8 ) {
				/* SMRDR1 always has the begining of the RX data stream */
				*pu8DataBuff++ = (u8)(smrdr1 & 0xff) | (u8)((smrdr1 >> 8) & 0xff);
				*pu8DataBuff++ = (u8)((smrdr1 >> 16) & 0xff) | (u8)((smrdr1 >> 24) & 0xff);
				*pu8DataBuff++ = (u8)(smrdr0 & 0xff) | (u8)((smrdr0 >> 8) & 0xff);
				*pu8DataBuff++ = (u8)((smrdr0 >> 16) & 0xff) | (u8)((smrdr0 >> 24) & 0xff);
			}

			if( unit == 2 ) {
				*pu8DataBuff++ = (u8)(smrdr0 & 0xff) | (u8)((smrdr0 >> 8) & 0xff);
			}
			if (unit == 4) {
				*pu8DataBuff++ = (u8)(smrdr0 & 0xff) | (u8)((smrdr0 >> 8) & 0xff);
				*pu8DataBuff++ = (u8)((smrdr0 >> 16) & 0xff) | (u8)((smrdr0 >> 24) & 0xff);
			}
		}
	}

	return 0;
}

