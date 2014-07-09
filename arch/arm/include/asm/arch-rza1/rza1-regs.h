/*
 * Copyright (C) 2013-2014 Renesas Solutions Corp.
*/

#ifndef _RZA1_REGS_H
#define _RZA1_REGS_H

/*
 *  Register bases.
 */
#define RZA1_WDT_BASE            (0xFCFE0000)
#define RZA1_FRQCR_BASE          (0xFCFE0010)
#define RZA1_STBCR_BASE          (0xFCFE0020)
#define RZA1_PCTR_BASE           (0xFCFE3000)
#define RZA1_OST_BASE            (0xFCFEC000)
#define RZA1_BCR_BASE            (0x3FFFC000)
#define RZA1_SDRAM_BASE          (0x3FFFC000)

/* Clock Registers */
#define FRQCR (RZA1_FRQCR_BASE + 0x00)
#define FRQCR2 (RZA1_FRQCR_BASE + 0x04)

/* Watchdog Registers */
#define WTCSR (RZA1_WDT_BASE + 0x00) /* Watchdog Timer Control Register */
#define WTCNT (RZA1_WDT_BASE + 0x02) /* Watchdog Timer Counter Register */
#define WRCSR (RZA1_WDT_BASE + 0x04) /* Watchdog Reset Control Register */

/* OSTimer Registers */
#define OSTM0CMP (RZA1_OST_BASE + 0x000)
#define OSTM0CNT (RZA1_OST_BASE + 0x004)
#define OSTM0TE  (RZA1_OST_BASE + 0x010)
#define OSTM0TS  (RZA1_OST_BASE + 0x014)
#define OSTM0TT  (RZA1_OST_BASE + 0x018)
#define OSTM0CTL (RZA1_OST_BASE + 0x020)
#define OSTM1CMP (RZA1_OST_BASE + 0x400)
#define OSTM1CNT (RZA1_OST_BASE + 0x404)
#define OSTM1TE  (RZA1_OST_BASE + 0x410)
#define OSTM1TS  (RZA1_OST_BASE + 0x414)
#define OSTM1TT  (RZA1_OST_BASE + 0x418)
#define OSTM1CTL (RZA1_OST_BASE + 0x420)

/* Standby controller registers (chapter 55) */
#define STBCR1 (RZA1_STBCR_BASE + 0x00)
#define STBCR2 (RZA1_STBCR_BASE + 0x04)
#define STBCR3 (RZA1_STBCR_BASE + 0x400)
#define STBCR4 (RZA1_STBCR_BASE + 0x404)
#define STBCR5 (RZA1_STBCR_BASE + 0x408)
#define STBCR6 (RZA1_STBCR_BASE + 0x40c)
#define STBCR7 (RZA1_STBCR_BASE + 0x410)
#define STBCR8 (RZA1_STBCR_BASE + 0x414)
#define STBCR9 (RZA1_STBCR_BASE + 0x418)
#define STBCR10 (RZA1_STBCR_BASE + 0x41c)
#define STBCR11 (RZA1_STBCR_BASE + 0x420)
#define STBCR12 (RZA1_STBCR_BASE + 0x424)
#define STBCR13 (RZA1_STBCR_BASE + 0x450)

/* Port0 Control register */
#define JPPR0   (RZA1_PCTR_BASE + 0x000)
#define JPMC0   (RZA1_PCTR_BASE + 0x040)
#define JPMCSR0 (RZA1_PCTR_BASE + 0x090)
#define JPIBC0  (RZA1_PCTR_BASE + 0x400)
#define PPR0    (RZA1_PCTR_BASE + 0x200)
#define PMC0    (RZA1_PCTR_BASE + 0x400)
#define PMCSR0  (RZA1_PCTR_BASE + 0x900)
#define PIBC0   (RZA1_PCTR_BASE + 0x4000)
/* Port1 Control register */
#define P1      (RZA1_PCTR_BASE + 0x0004)
#define PM1     (RZA1_PCTR_BASE + 0x0304)
#define PMC1    (RZA1_PCTR_BASE + 0x0404)
#define PFC1    (RZA1_PCTR_BASE + 0x0504)
#define PFCE1   (RZA1_PCTR_BASE + 0x0604)
#define PFCAE1  (RZA1_PCTR_BASE + 0x0a04)
#define PIBC1   (RZA1_PCTR_BASE + 0x4004)
#define PBDC1   (RZA1_PCTR_BASE + 0x4104)
#define PIPC1   (RZA1_PCTR_BASE + 0x4204)
/* Port2 Control register */
#define P2      (RZA1_PCTR_BASE + 0x0008)
#define PM2     (RZA1_PCTR_BASE + 0x0308)
#define PMC2    (RZA1_PCTR_BASE + 0x0408)
#define PFC2    (RZA1_PCTR_BASE + 0x0508)
#define PFCE2   (RZA1_PCTR_BASE + 0x0608)
#define PFCAE2  (RZA1_PCTR_BASE + 0x0a08)
#define PIBC2   (RZA1_PCTR_BASE + 0x4008)
#define PBDC2   (RZA1_PCTR_BASE + 0x4108)
#define PIPC2   (RZA1_PCTR_BASE + 0x4208)
/* Port3 Control register */
#define P3      (RZA1_PCTR_BASE + 0x000c)
#define PM3     (RZA1_PCTR_BASE + 0x030c)
#define PMC3    (RZA1_PCTR_BASE + 0x040c)
#define PFC3    (RZA1_PCTR_BASE + 0x050c)
#define PFCE3   (RZA1_PCTR_BASE + 0x060c)
#define PFCAE3  (RZA1_PCTR_BASE + 0x0a0c)
#define PIBC3   (RZA1_PCTR_BASE + 0x400c)
#define PBDC3   (RZA1_PCTR_BASE + 0x410c)
#define PIPC3   (RZA1_PCTR_BASE + 0x420c)
/* Port4 Control register */
#define P4      (RZA1_PCTR_BASE + 0x0010)
#define PM4     (RZA1_PCTR_BASE + 0x0310)
#define PMC4    (RZA1_PCTR_BASE + 0x0410)
#define PFC4    (RZA1_PCTR_BASE + 0x0510)
#define PFCE4   (RZA1_PCTR_BASE + 0x0610)
#define PFCAE4  (RZA1_PCTR_BASE + 0x0a10)
#define PIBC4   (RZA1_PCTR_BASE + 0x4010)
#define PBDC4   (RZA1_PCTR_BASE + 0x4110)
#define PIPC4   (RZA1_PCTR_BASE + 0x4210)
/* Port5 Control register */
#define P5      (RZA1_PCTR_BASE + 0x0014)
#define PM5     (RZA1_PCTR_BASE + 0x0314)
#define PMC5    (RZA1_PCTR_BASE + 0x0414)
#define PFC5    (RZA1_PCTR_BASE + 0x0514)
#define PFCE5   (RZA1_PCTR_BASE + 0x0614)
#define PFCAE5  (RZA1_PCTR_BASE + 0x0a14)
#define PIBC5   (RZA1_PCTR_BASE + 0x4014)
#define PBDC5   (RZA1_PCTR_BASE + 0x4114)
#define PIPC5   (RZA1_PCTR_BASE + 0x4214)
/* Port6 Control register */
#define P6      (RZA1_PCTR_BASE + 0x0018)
#define PM6     (RZA1_PCTR_BASE + 0x0318)
#define PMC6    (RZA1_PCTR_BASE + 0x0418)
#define PFC6    (RZA1_PCTR_BASE + 0x0518)
#define PFCE6   (RZA1_PCTR_BASE + 0x0618)
#define PFCAE6  (RZA1_PCTR_BASE + 0x0a18)
#define PIBC6   (RZA1_PCTR_BASE + 0x4018)
#define PBDC6   (RZA1_PCTR_BASE + 0x4118)
#define PIPC6   (RZA1_PCTR_BASE + 0x4218)
/* Port7 Control register */
#define P7      (RZA1_PCTR_BASE + 0x001c)
#define PM7     (RZA1_PCTR_BASE + 0x031c)
#define PMC7    (RZA1_PCTR_BASE + 0x041c)
#define PFC7    (RZA1_PCTR_BASE + 0x051c)
#define PFCE7   (RZA1_PCTR_BASE + 0x061c)
#define PFCAE7  (RZA1_PCTR_BASE + 0x0a1c)
#define PIBC7   (RZA1_PCTR_BASE + 0x401c)
#define PBDC7   (RZA1_PCTR_BASE + 0x411c)
#define PIPC7 (RZA1_PCTR_BASE + 0x421c)
/* Port8 Control register */
#define P8      (RZA1_PCTR_BASE + 0x0020)
#define PM8     (RZA1_PCTR_BASE + 0x0320)
#define PMC8    (RZA1_PCTR_BASE + 0x0420)
#define PFC8    (RZA1_PCTR_BASE + 0x0520)
#define PFCE8   (RZA1_PCTR_BASE + 0x0620)
#define PFCAE8  (RZA1_PCTR_BASE + 0x0a20)
#define PIBC8   (RZA1_PCTR_BASE + 0x4020)
#define PBDC8   (RZA1_PCTR_BASE + 0x4120)
#define PIPC8   (RZA1_PCTR_BASE + 0x4220)
/* Port9 Control register */
#define P9      (RZA1_PCTR_BASE + 0x0024)
#define PM9     (RZA1_PCTR_BASE + 0x0324)
#define PMC9    (RZA1_PCTR_BASE + 0x0424)
#define PFC9    (RZA1_PCTR_BASE + 0x0524)
#define PFCE9   (RZA1_PCTR_BASE + 0x0624)
#define PFCAE9  (RZA1_PCTR_BASE + 0x0a24)
#define PIBC9   (RZA1_PCTR_BASE + 0x4024)
#define PBDC9   (RZA1_PCTR_BASE + 0x4124)
#define PIPC9   (RZA1_PCTR_BASE + 0x4224)
/* Port10 Control register */
#define P10      (RZA1_PCTR_BASE + 0x0028)
#define PM10     (RZA1_PCTR_BASE + 0x0328)
#define PMC10    (RZA1_PCTR_BASE + 0x0428)
#define PFC10    (RZA1_PCTR_BASE + 0x0528)
#define PFCE10   (RZA1_PCTR_BASE + 0x0628)
#define PFCAE10  (RZA1_PCTR_BASE + 0x0a28)
#define PIBC10   (RZA1_PCTR_BASE + 0x4028)
#define PBDC10   (RZA1_PCTR_BASE + 0x4128)
#define PIPC10   (RZA1_PCTR_BASE + 0x4228)
/* Port11 Control register */
#define P11      (RZA1_PCTR_BASE + 0x002c)
#define PM11     (RZA1_PCTR_BASE + 0x032c)
#define PMC11    (RZA1_PCTR_BASE + 0x042c)
#define PFC11    (RZA1_PCTR_BASE + 0x052c)
#define PFCE11   (RZA1_PCTR_BASE + 0x062c)
#define PFCAE11  (RZA1_PCTR_BASE + 0x0a2c)
#define PIBC11   (RZA1_PCTR_BASE + 0x402c)
#define PBDC11   (RZA1_PCTR_BASE + 0x412c)
#define PIPC11   (RZA1_PCTR_BASE + 0x422c)

/* Bus State Contoller registers */
#define CMNCR  (RZA1_BCR_BASE + 0x00)
#define CS0BCR (RZA1_BCR_BASE + 0x04)
#define CS0WCR (RZA1_BCR_BASE + 0x28)
#define CS1BCR (RZA1_BCR_BASE + 0x08)
#define CS1WCR (RZA1_BCR_BASE + 0x2c)
#define CS2BCR (RZA1_BCR_BASE + 0x0c)
#define CS2WCR (RZA1_BCR_BASE + 0x30)
#define CS3BCR (RZA1_BCR_BASE + 0x10)
#define CS3WCR (RZA1_BCR_BASE + 0x34)

/* SDRAM controller registers */
#define SDCR   (RZA1_SDRAM_BASE + 0x4c)
#define RTCOR  (RZA1_SDRAM_BASE + 0x58)
#define RTCSR  (RZA1_SDRAM_BASE + 0x50)

/* Power-Down Registers (Chapter 55) */
#define SWRSTCR1 0xFCFE0460	/* Software reset control register 1 */
#define SWRSTCR2 0xFCFE0464	/* Software reset control register 2 */
#define SWRSTCR3 0xFCFE0468	/* Software reset control register 3 */
#define SYSCR1 0xFCFE0400	/* System control register 1 */
#define SYSCR2 0xFCFE0404	/* System control register 2 */
#define SYSCR3 0xFCFE0408	/* System control register 3 */
#define CPUSTS 0xFCFE0018	/* CPU status register */
#define STBREQ1 0xFCFE0030	/* Standby request register 1 */
#define STBREQ2 0xFCFE0034	/* Standby request register 2 */
#define STBACK1 0xFCFE0040	/* Standby acknowledge register 1 */
#define STBACK2 0xFCFE0044	/* Standby acknowledge register 2 */
#define RRAMKP 0xFCFF1800	/* On-chip data-retention RAM area setting register */
#define DSCTR 0xFCFF1802	/* Deep standby control register */
#define DSSSR 0xFCFF1804	/* Deep standby cancel source select register */
#define DSESR 0xFCFF1806	/* Deep standby cancel edge select register */
#define DSFR 0xFCFF1808	/* Deep standby cancel source flag register */
#define XTALCTR 0xFCFF1810	/* XTAL crystal oscillator gain control register */



#endif				/* _RZA1_REGS_H */
