/*
 * Copyright (C) 2008-2013 Renesas Solutions Corp.
 * Copyright (C) 2007,2008 Nobobuhiro Iwamatsu <iwamatsu@nigauri.org>
 *
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/rza1-regs.h>
#include <asm/processor.h>

#define OSTM0CTL_D	0x02	/* free run mode, disable interrupt */
#define OST_MAX_COUNTER (0xFFFFFFFF)
#define OST_TIMER_RESET (0xFFFFFFFF)

/* 33.333MHz */
#define P0_CLOCK_FREQUENCY_KHZ  (33.333 * 1000)
#define MAX_CYCLE_msec          (0xFFFFFFFF / P0_CLOCK_FREQUENCY_KHZ)

static vu_long ost0_timer;

static void ost_timer_start(unsigned int timer)
{
	writeb(readb(OSTM0TS) | 0x01, OSTM0TS);
}

static void ost_timer_stop(unsigned int timer)
{
	writeb(readb(OSTM0TT) | 0x01, OSTM0TT);
}

int timer_init(void)
{
	ost0_timer = 0;

	readb(OSTM0CTL);
	writeb(OSTM0CTL_D, OSTM0CTL);

	/* User Device 0 only */
	ost_timer_stop(0);
	//writel(OST_TIMER_RESET, OSTM0CMP);
	writel(P0_CLOCK_FREQUENCY_KHZ, OSTM0CMP);
	ost_timer_start(0);

	return 0;
}

unsigned long long get_ticks(void)
{
	return ost0_timer;
}

static vu_long cmcnt = 0;
static unsigned long get_usec (void)
{
	ulong data = readl(OSTM0CNT);
	ulong diff;

	if (data >= cmcnt)
		diff = data - cmcnt;
	else
		diff = (OST_TIMER_RESET - cmcnt) + data;

	ost0_timer += diff;

	cmcnt = data;

	/* Timer source clock (P0) is 33.33 Mhz */
	return (unsigned long)(ost0_timer / 33);
}

/* return msec */
ulong get_timer(ulong base)
{
	const ulong timecnt = OST_TIMER_RESET / (33 * 1000); /*130150*/
	ulong now = (get_usec() / 1000);
	
	if (now >= base)
		return now - base;
	else
		return ((timecnt + 1) - base) + now;
}

void __mdelay(unsigned long msec)
{
	unsigned long end = (get_usec() / 1000) + msec;

	while ( (get_usec() / 1000) < end)
		continue;
}

void __udelay(unsigned long usec)
{
	unsigned long end = get_usec() + usec;

	while (get_usec() < end)
		continue;
}

unsigned long get_tbclk(void)
{
	return CONFIG_SYS_HZ;
}
