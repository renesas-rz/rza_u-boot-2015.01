/*
 * RIIC bus driver
 *
 * Copyright (C) 2011-2013  Renesas Solutions Corp.
 * Copyright (C) 2011 Nobuhiro Iwamatsu <nobuhiro.iwamatsu.yj@renesas.com>
 *
 * Based on i2c-sh_mobile.c
 * Portion Copyright (C) 2008 Magnus Damm
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
#include <asm/errno.h>
#include <common.h>
#include <asm/io.h>

#define RIIC_ICCR1	0x00
#define RIIC_ICCR2	0x04
#define RIIC_ICMR1	0x08
#define RIIC_ICMR2	0x0c
#define RIIC_ICMR3	0x10
#define RIIC_ICFER	0x14
#define RIIC_ICSER	0x18
#define RIIC_ICIER	0x1c
#define RIIC_ICSR1	0x20
#define RIIC_ICSR2	0x24
#define RIIC_ICBRL	0x34
#define RIIC_ICBRH	0x38
#define RIIC_ICDRT	0x3c
#define RIIC_ICDRR	0x40

/* ICCR1 */
#define ICCR1_ICE	0x80
#define ICCR1_IICRST	0x40
#define ICCR1_CLO	0x20
#define ICCR1_SOWP	0x10
#define ICCR1_SCLO	0x08
#define ICCR1_SDAO	0x04
#define ICCR1_SCLI	0x02
#define ICCR1_SDAI	0x01

/* ICCR2 */
#define ICCR2_BBSY	0x80
#define ICCR2_MST	0x40
#define ICCR2_TRS	0x20
#define ICCR2_SP	0x08
#define ICCR2_RS	0x04
#define ICCR2_ST	0x02

/* ICMR1 */
#define ICMR1_MTWP	0x80
#define ICMR1_CKS_MASK	0x70
#define ICMR1_BCWP	0x08
#define ICMR1_BC_MASK	0x07

#define ICMR1_CKS(_x)	((_x << 4) & ICMR1_CKS_MASK)
#define ICMR1_BC(_x)	((_x) & ICMR1_BC_MASK)

/* ICMR2 */
#define ICMR2_DLCS	0x80
#define ICMR2_SDDL_MASK	0x70
#define ICMR2_TMOH	0x04
#define ICMR2_TMOL	0x02
#define ICMR2_TMOS	0x01

/* ICMR3 */
#define ICMR3_SMBS	0x80
#define ICMR3_WAIT	0x40
#define ICMR3_RDRFS	0x20
#define ICMR3_ACKWP	0x10
#define ICMR3_ACKBT	0x08
#define ICMR3_ACKBR	0x04
#define ICMR3_NF_MASK	0x03

/* ICFER */
#define ICFER_FMPE	0x80
#define ICFER_SCLE	0x40
#define ICFER_NFE	0x20
#define ICFER_NACKE	0x10
#define ICFER_SALE	0x08
#define ICFER_NALE	0x04
#define ICFER_MALE	0x02
#define ICFER_TMOE	0x01

/* ICSER */
#define ICSER_HOAE	0x80
#define ICSER_DIDE	0x20
#define ICSER_GCAE	0x08
#define ICSER_SAR2E	0x04
#define ICSER_SAR1E	0x02
#define ICSER_SAR0E	0x01

/* ICIER */
#define ICIER_TIE	0x80
#define ICIER_TEIE	0x40
#define ICIER_RIE	0x20
#define ICIER_NAKIE	0x10
#define ICIER_SPIE	0x08
#define ICIER_STIE	0x04
#define ICIER_ALIE	0x02
#define ICIER_TMOIE	0x01

/* ICSR1 */
#define ICSR1_HOA	0x80
#define ICSR1_DID	0x20
#define ICSR1_GCA	0x08
#define ICSR1_AAS2	0x04
#define ICSR1_AAS1	0x02
#define ICSR1_AAS0	0x01

/* ICSR2 */
#define ICSR2_TDRE	0x80
#define ICSR2_TEND	0x40
#define ICSR2_RDRF	0x20
#define ICSR2_NACKF	0x10
#define ICSR2_STOP	0x08
#define ICSR2_START	0x04
#define ICSR2_AL	0x02
#define ICSR2_TMOF	0x01

/* ICBRH */
#define ICBRH_RESERVED	0xe0	/* The write value shoud always be 1 */
#define ICBRH_BRH_MASK	0x1f

/* ICBRL */
#define ICBRL_RESERVED	0xe0	/* The write value shoud always be 1 */
#define ICBRL_BRL_MASK	0x1f

#define RIIC_TIMEOUT	(100)	/* 100 msec */

struct riic_data {
	unsigned int reg;
};
static struct riic_data i_pd[CONFIG_SYS_MAX_I2C_BUS] =
	{ { CONFIG_SH_I2C_BASE0 },
#ifdef CONFIG_SH_I2C_BASE1
	  { CONFIG_SH_I2C_BASE1 },
#endif
#ifdef CONFIG_SH_I2C_BASE2
	  { CONFIG_SH_I2C_BASE2 },
#endif
#ifdef CONFIG_SH_I2C_BASE3
	  { CONFIG_SH_I2C_BASE3 },
#endif
#ifdef CONFIG_SH_I2C_BASE4
	  { CONFIG_SH_I2C_BASE4 },
#endif
	};
static struct riic_data *pd = &i_pd[0];

static unsigned int current_bus;

/**
 * i2c_set_bus_num - change active I2C bus
 *	@bus: bus index, zero based
 *	@returns: 0 on success, non-0 on failure
 */
int i2c_set_bus_num(unsigned int bus)
{
	if ((bus < 0) || (bus >= CONFIG_SYS_MAX_I2C_BUS)) {
		debug("%s: bad bus: %d\n", __func__, bus);
		return -1;
	}

	pd = &i_pd[bus];
	current_bus = bus;

	return 0;
}

static unsigned char riic_read(struct riic_data *pd, unsigned long addr)
{
	return inb(pd->reg + addr);
}

static void riic_write(struct riic_data *pd, unsigned char data,
		       unsigned long addr)
{
	outb(data, pd->reg + addr);
}

static void riic_set_bit(struct riic_data *pd, unsigned char val,
			 unsigned long offset)
{
	unsigned char tmp;

	tmp = riic_read(pd, offset) | val;
	riic_write(pd, tmp, offset);
}

static void riic_clear_bit(struct riic_data *pd, unsigned char val,
			   unsigned long offset)
{
	unsigned char tmp;

	tmp = riic_read(pd, offset) & ~val;
	riic_write(pd, tmp, offset);
}

static int riic_set_clock(struct riic_data *pd, int clock)
{
	switch (clock) {
	case 100000:
		riic_clear_bit(pd, ICFER_FMPE, RIIC_ICFER);
		riic_clear_bit(pd, ICMR1_CKS_MASK, RIIC_ICMR1);
		riic_set_bit(pd, ICMR1_CKS(3), RIIC_ICMR1);
		riic_write(pd, ICBRH_RESERVED | 23, RIIC_ICBRH);
		riic_write(pd, ICBRL_RESERVED | 23, RIIC_ICBRL);
		break;
	case 400000:
		riic_clear_bit(pd, ICFER_FMPE, RIIC_ICFER);
		riic_clear_bit(pd, ICMR1_CKS_MASK, RIIC_ICMR1);
		riic_set_bit(pd, ICMR1_CKS(1), RIIC_ICMR1);
		riic_write(pd, ICBRH_RESERVED | 20, RIIC_ICBRH);
		riic_write(pd, ICBRL_RESERVED | 19, RIIC_ICBRL);
		break;
	case 1000000:
		riic_set_bit(pd, ICFER_FMPE, RIIC_ICFER);
		riic_clear_bit(pd, ICMR1_CKS_MASK, RIIC_ICMR1);
		riic_set_bit(pd, ICMR1_CKS(0), RIIC_ICMR1);
		riic_write(pd, ICBRH_RESERVED | 14, RIIC_ICBRH);
		riic_write(pd, ICBRL_RESERVED | 14, RIIC_ICBRL);
		break;

	default:
		debug("%s: unsupported clock (%dkHz)\n", __func__, clock);
		return -EINVAL;
	}

	return 0;
}

static int riic_check_busy(void)
{
	/* As for I2C specification, min. bus-free-time is 
		4.7 us(Sm) and 1.3 us(Fm). */
	ulong start, timeout = (ulong)RIIC_TIMEOUT;

	start = get_timer(0);
	do {
		if (!(riic_read(pd, RIIC_ICCR2) & ICCR2_BBSY))
			return 0;
	} while (get_timer(start) < timeout);

	debug("%s: i2c bus is busy.\n", __func__);
	return -EBUSY;
}

static int riic_init_setting(struct riic_data *pd, int clock)
{
	int ret;

	riic_clear_bit(pd, ICCR1_ICE, RIIC_ICCR1);
	riic_set_bit(pd, ICCR1_IICRST, RIIC_ICCR1);
	riic_clear_bit(pd, ICCR1_IICRST, RIIC_ICCR1);

	riic_write(pd, ICSER_SAR0E, RIIC_ICSER);

	riic_write(pd, ICMR1_BC(7), RIIC_ICMR1);
	ret = riic_set_clock(pd, clock);
	if (ret < 0)
		return ret;

	riic_set_bit(pd, ICCR1_ICE, RIIC_ICCR1);	/* Enable RIIC */
	riic_set_bit(pd, ICMR3_RDRFS | ICMR3_WAIT | ICMR3_ACKWP, RIIC_ICMR3);

	return 0;
}

static int riic_wait_for_icsr2(struct riic_data *pd, unsigned short bit)
{
	unsigned char icsr2;
	ulong start, timeout = (ulong)RIIC_TIMEOUT;

	start = get_timer(0);
	do {
		icsr2 = riic_read(pd, RIIC_ICSR2);
		if (icsr2 & ICSR2_NACKF)
			return -EIO;
		if (icsr2 & bit)
			return 0;
	} while (get_timer(start) < timeout);

	debug("%s: timeout!(bit = %x icsr2 = %x, iccr2 = %x)\n", __func__,
		bit, riic_read(pd, RIIC_ICSR2), riic_read(pd, RIIC_ICCR2));

	return -ETIMEDOUT;
}

static int riic_check_nack_receive(void)
{
	if (riic_read(pd, RIIC_ICSR2) & ICSR2_NACKF) {
		/* received NACK */
		riic_clear_bit(pd, ICSR2_NACKF, RIIC_ICSR2);
		riic_set_bit(pd, ICCR2_SP, RIIC_ICCR2);
		riic_read(pd, RIIC_ICDRR);	/* dummy read */
		return -1;
	}
	return 0;
}

static void riic_set_receive_ack(struct riic_data *pd, int ack)
{
	if (ack)
		riic_clear_bit(pd, ICMR3_ACKBT, RIIC_ICMR3);
	else
		riic_set_bit(pd, ICMR3_ACKBT, RIIC_ICMR3);
}

static int riic_i2c_raw_write(u8 *buf, int len)
{
	int ret = 0;
	int index = 0;

	do {
		ret = riic_check_nack_receive();
		if (ret < 0)
			return -1;

		ret = riic_wait_for_icsr2(pd, ICSR2_TDRE);
		if (ret < 0)
			return -1;

		riic_write(pd, buf[index++], RIIC_ICDRT);
	} while (len > index);

	return ret;
}

static int riic_i2c_raw_read(u8 *buf, int len)
{
	int dummy_read = 1;
	int ret = 0;
	int index = 0;

	do {
		ret = riic_wait_for_icsr2(pd, ICSR2_RDRF);
		if (ret < 0)
			return ret;

		buf[index] = riic_read(pd, RIIC_ICDRR);
		if (dummy_read)
			dummy_read = 0;
		else
			index++;
		riic_set_receive_ack(pd, 1);
	} while (index < (len - 1));

	ret = riic_wait_for_icsr2(pd, ICSR2_RDRF);
	if (ret < 0)
		return ret;

	riic_clear_bit(pd, ICSR2_STOP, RIIC_ICSR2);
	riic_set_bit(pd, ICCR2_SP, RIIC_ICCR2);

	buf[index++] = riic_read(pd, RIIC_ICDRR);
	riic_set_receive_ack(pd, 0);

	return ret;
}

static int riic_send_mem_addr(u32 addr, int alen)
{
	int i;
	u8 b[4];

	if (alen > 4 || alen <= 0)
		return -1;
	/* change byte order and shift bit */
	for (i = alen - 1; i >= 0; i--, addr >>= 8)
		b[i] = addr & 0xff;

	return riic_i2c_raw_write(b, alen);
}

static int riic_send_start_cond(int restart)
{
	int ret;

	if (restart)
		riic_set_bit(pd, ICCR2_RS, RIIC_ICCR2);
	else
		riic_set_bit(pd, ICCR2_ST, RIIC_ICCR2);

	ret = riic_wait_for_icsr2(pd, ICSR2_START);
	if (ret < 0)
		return ret;
	riic_clear_bit(pd, ICSR2_START, RIIC_ICSR2);

	return ret;
}

static int riic_send_stop_cond(void)
{
	int ret;

	riic_clear_bit(pd, ICSR2_STOP | ICSR2_NACKF, RIIC_ICSR2);
	riic_set_bit(pd, ICCR2_SP, RIIC_ICCR2);

	ret = riic_wait_for_icsr2(pd, ICSR2_STOP);
	if (ret < 0)
		return ret;

	riic_clear_bit(pd, ICSR2_STOP | ICSR2_NACKF, RIIC_ICSR2);
	return ret;
}

static int riic_send_dev_addr(u8 chip, int read)
{
	u8 buf = ((chip << 1) | read);

	return riic_i2c_raw_write(&buf, 1);
}

int i2c_read(u8 chip, u32 addr, int alen, u8 *buffer, int len)
{
	int ret;

	ret = riic_check_busy();
	if (ret < 0)
		return ret;

	ret = riic_send_start_cond(0);
	if (ret < 0)
		goto force_exit;

	/* send addr */
	if (alen > 0) {
		ret = riic_send_dev_addr(chip, 0);
		if (ret < 0)
			goto force_exit;

		ret = riic_send_mem_addr(addr, alen);
		if (ret < 0)
			goto force_exit;

		ret = riic_wait_for_icsr2(pd, ICSR2_TEND);
		if (ret < 0)
			goto force_exit;

		/* restart */
		ret = riic_send_start_cond(1);
		if (ret < 0)
			goto force_exit;
	}
	ret = riic_send_dev_addr(chip, 1);
	if (ret < 0)
		goto force_exit;
	ret = riic_wait_for_icsr2(pd, ICSR2_RDRF);
	if (ret < 0)
		goto force_exit;

	ret = riic_check_nack_receive();
	if (ret < 0)
		goto force_exit;

	/* receive data */
	ret = riic_i2c_raw_read(buffer, len);

force_exit:
	riic_send_stop_cond();

	return ret;
}

int i2c_write(u8 chip, u32 addr, int alen, u8 *buffer, int len)
{
	int ret;

	ret = riic_check_busy();
	if (ret < 0)
		return ret;

	ret = riic_send_start_cond(0);
	if (ret < 0)
		goto force_exit;

	/* send addr */
	ret = riic_send_dev_addr(chip, 0);
	if (ret < 0)
		goto force_exit;

	if (alen > 0) {
		ret = riic_send_mem_addr(addr, alen);
		if (ret < 0)
			goto force_exit;
	}

	/* transmit data */
	ret = riic_i2c_raw_write(buffer, len);
	if (ret < 0)
		goto force_exit;
	ret = riic_wait_for_icsr2(pd, ICSR2_TEND);

force_exit:
	riic_send_stop_cond();

	return ret;
}

unsigned int i2c_get_bus_num(void)
{
	return current_bus;
}

int i2c_probe(u8 chip)
{
	u8 dummy;

	return i2c_read(chip, 0, 1, &dummy, sizeof(dummy));
}

void i2c_init(int speed, int slaveaddr)
{
	int bus = 0;

	riic_init_setting(&i_pd[0], CONFIG_SYS_I2C_SPEED);

#ifdef CONFIG_I2C_MULTI_BUS
	for (bus = 1; bus < CONFIG_SYS_MAX_I2C_BUS; bus++) {
		riic_init_setting(&i_pd[bus], CONFIG_SYS_I2C_SPEED);
	}
#endif
	return;
}
