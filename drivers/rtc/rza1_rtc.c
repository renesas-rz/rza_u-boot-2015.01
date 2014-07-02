/*
 * Copyright (C) 2008-2014 Renesas Solutions Corp.
 *
 * Based on linux/drivers/rtc/rtc-sh.c
 *
 * This file is released under the terms of GPL v2 and any later version.
 * See the file COPYING in the root directory of the source tree for details.
 */

#include <common.h>
#include <command.h>
#include <rtc.h>

/*
 * RZA1H RTC registers layout
 */
#define RTC_BASE	CONFIG_RTC_RZA1_BASE_ADDR

#define R64CNT		(RTC_BASE + 0x00)
#define RSECCNT		(RTC_BASE + 0x02)
#define RMINCNT		(RTC_BASE + 0x04)
#define RHRCNT		(RTC_BASE + 0x06)
#define RWKCNT		(RTC_BASE + 0x08)
#define RDAYCNT		(RTC_BASE + 0x0A)
#define RMONCNT		(RTC_BASE + 0x0C)
#define RYRCNT		(RTC_BASE + 0x0E)
#define RSECAR		(RTC_BASE + 0x10)
#define RMINAR		(RTC_BASE + 0x12)
#define RHRAR		(RTC_BASE + 0x14)
#define RWKAR		(RTC_BASE + 0x16)
#define RDAYAR		(RTC_BASE + 0x18)
#define RMONAR		(RTC_BASE + 0x1A)
#define RYRAR		(RTC_BASE + 0x20)
#define RCR1		(RTC_BASE + 0x1C)
#define RCR2		(RTC_BASE + 0x1E)
#define RCR3		(RTC_BASE + 0x24)
#define RCR5		(RTC_BASE + 0x26)
#define RFRH		(RTC_BASE + 0x2A)
#define RFRL		(RTC_BASE + 0x2C)

/* RCR1 Bits */
#define RCR1_CF		0x80
#define RCR1_CIE	0x10
#define RCR1_AIE	0x08

/* RCR2 Bits */
#define RCR2_RTCEN	0x08
#define RCR2_RESET	0x02
#define RCR2_START	0x01

/* RCR5 Bits */
#define RCR5_RCKSEL00	0x00	/* select RTC_X1(32.768KHz) */
#define RCR5_RCKSEL01	0x01	/* select EXTAL */
#define RCR5_RCKSEL10	0x02	/* select RTC_X3 */

static u8 rtc_read8(unsigned int addr)
{
	u8 val = *(volatile u8 *)addr;
	return val;
}

static u16 rtc_read16(unsigned int addr)
{
	u16 val = *(volatile u16 *)addr;
	return val;
}

static void rtc_write8(unsigned int addr, u8 val)
{
	*(volatile u8 *)addr = val;
}

static void rtc_write16(unsigned int addr, u16 val)
{
	*(volatile u16 *)addr = val;
}

int rtc_get(struct rtc_time *tmp)
{
	u8 cf_bit;

	do {
		u8 t;
		u16 yr, yr100;

		t = rtc_read8(RCR1);
		t &= ~RCR1_CF;	/* clear CF bit */
		rtc_write8(RCR1, t);

		tmp->tm_sec = bcd2bin(rtc_read8(RSECCNT));
		tmp->tm_min = bcd2bin(rtc_read8(RMINCNT));
		tmp->tm_hour = bcd2bin(rtc_read8(RHRCNT));
		tmp->tm_mday = bcd2bin(rtc_read8(RDAYCNT));
		tmp->tm_mon = bcd2bin(rtc_read8(RMONCNT));
		tmp->tm_wday = bcd2bin(rtc_read8(RWKCNT));

		yr = rtc_read16(RYRCNT);
		yr100 = bcd2bin(yr >> 8);
		yr &= 0xff;
		tmp->tm_year = (yr100 * 100 + bcd2bin(yr)) - 1900;

		cf_bit = rtc_read8(RCR1) & RCR1_CF;

	} while (cf_bit != 0);

	/* to unify both the u-boot and the kernel in rtc system */
	tmp->tm_year += 1900;

	return 0;
}

int rtc_set(struct rtc_time *tmp)
{
	u8 t;
	u16 year;

	/* stop and reset */
	t = rtc_read8(RCR2);
	t |= RCR2_RESET;
	t &= ~RCR2_START;
	rtc_write8(RCR2, t);

	rtc_write8(RMONCNT, bin2bcd(tmp->tm_mon));
	rtc_write8(RWKCNT, bin2bcd(tmp->tm_wday));
	rtc_write8(RDAYCNT, bin2bcd(tmp->tm_mday));
	rtc_write8(RHRCNT, bin2bcd(tmp->tm_hour));
	rtc_write8(RMINCNT, bin2bcd(tmp->tm_min));
	rtc_write8(RSECCNT, bin2bcd(tmp->tm_sec));

	year = (bin2bcd((tmp->tm_year) / 100) << 8) |
		bin2bcd((tmp->tm_year) % 100);

	rtc_write16(RYRCNT, year);

	/* start */
	t = rtc_read8(RCR2);
	t |= RCR2_RTCEN | RCR2_START;
	rtc_write8(RCR2, t);

	return 0;
}

void rtc_reset(void)
{
	u8 t;

	t = rtc_read8(RCR1);
	t &= ~(RCR1_CIE | RCR1_AIE);	/* disable interrupts */
	rtc_write8(RCR1, t);

	rtc_write8(RCR5, RCR5_RCKSEL00);	/* set source clock */

	/* start */
	t = rtc_read8(RCR2);
	t |= RCR2_RTCEN | RCR2_START;
	rtc_write8(RCR2, t);
}
