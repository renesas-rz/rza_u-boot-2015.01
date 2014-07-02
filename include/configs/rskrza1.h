/*
 * Configuation settings for the Renesas RSKRZA1 board
 *
 * Copyright (C) 2013-2014 Renesas Solutions Corp.
 * Copyright (C) 2012 Renesas Electronics Europe Ltd.
 * Copyright (C) 2012 Phil Edworthy
 *
 * This file is released under the terms of GPL v2 and any later version.
 * See the file COPYING in the root directory of the source tree for details.
 */

#ifndef __RSKRZA1_H
#define __RSKRZA1_H

#undef DEBUG
#define CONFIG_ARMV7		1	/* This is an ARM V7 CPU core */
#define CONFIG_CPU_RZA1	1
#define CONFIG_BOARD_LATE_INIT	1
#define CONFIG_MACH_TYPE MACH_TYPE_RSKRZA1
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_SYS_GENERIC_BOARD

#define CONFIG_CMD_NET
#define CONFIG_CMD_MII
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_USB
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_FAT
#define CONFIG_CMD_SF
#define CONFIG_CMD_I2C
#define CONFIG_CMD_EEPROM
#define CONFIG_CMD_DATE
#define CONFIG_DOS_PARTITION
#define CONFIG_MAC_PARTITION
#define CONFIG_USB_STORAGE
#define CONFIG_CMD_SNTP
#define CONFIG_BOOTP_NTPSERVER
#define CONFIG_BOOTP_TIMEOFFSET


#define CONFIG_OF_LIBFDT
#define CONFIG_CMDLINE_EDITING
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG

#ifndef _CONFIG_CMD_DEFAULT_H
# include <config_cmd_default.h>
#endif

#define CONFIG_BAUDRATE		115200
#define CONFIG_BOOTARGS		"console=ttySC2,115200"
#define CONFIG_BOOTDELAY	3
#define CONFIG_SYS_BAUDRATE_TABLE	{ CONFIG_BAUDRATE }

#define CONFIG_SYS_LONGHELP		/* undef to save memory	*/
#define CONFIG_SYS_PROMPT	"=> "	/* Monitor Command Prompt */
#define CONFIG_SYS_CBSIZE	256	/* Boot Argument Buffer Size */
#define CONFIG_SYS_PBSIZE	256	/* Print Buffer Size */
#define CONFIG_SYS_MAXARGS	16	/* max number of command args */

#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2      "> "

#define CONFIG_SYS_ARM_CACHE_WRITETHROUGH

/* Serial */
#define CONFIG_SCIF_CONSOLE
#define CONFIG_CONS_SCIF2
#define SCIF2_BASE			0xE8008000
#define CONFIG_SH_SCIF_CLK_FREQ CONFIG_SYS_CLK_FREQ

/* Memory */
/* u-boot relocated to top 256KB of ram */
#define CONFIG_NR_DRAM_BANKS		1
#if !defined(CONFIG_BOOT_MODE0)
#define CONFIG_SYS_TEXT_BASE		0x18000000
#else
#define CONFIG_SYS_TEXT_BASE		0x00000000
#endif
#define CONFIG_SYS_SDRAM_BASE		0x08000000
#define CONFIG_SYS_SDRAM_SIZE		(32 * 1024 * 1024)

#define CONFIG_SYS_MEMTEST_START	CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_SDRAM_BASE + 0x2000000)
#define CONFIG_SYS_MALLOC_LEN		(256 * 1024)
#define CONFIG_SYS_MONITOR_LEN		(128 * 1024)
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + 4*1024*1024)
#define	CONFIG_LOADADDR			CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_INIT_SP_ADDR         (0x09F00000)

/* NOR Flash */
#define CONFIG_SYS_FLASH_BASE		0x00000000
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_MAX_FLASH_SECT	512
#define CONFIG_FLASH_CFI_DRIVER
#define CONFIG_SYS_FLASH_CFI
#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
#define CONFIG_SYS_FLASH_BANKS_LIST	{ CONFIG_SYS_FLASH_BASE }
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_FLASH_BASE

#if !defined(CONFIG_BOOT_MODE0)
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_OFFSET	0x80000
#define CONFIG_ENV_SECT_SIZE	0x40000
#else
#define CONFIG_ENV_IS_IN_FLASH
#define CONFIG_ENV_OFFSET	(512 * 1024)
#define CONFIG_ENV_SECT_SIZE	(256 * 1024)
#endif

#define CONFIG_ENV_SIZE		CONFIG_ENV_SECT_SIZE
#define CONFIG_ENV_ADDR		(CONFIG_SYS_FLASH_BASE + CONFIG_ENV_OFFSET)
#define CONFIG_ENV_OVERWRITE	1

#define __io

/* Spi-Flash configuration */
#define CONFIG_RZ_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_RZA1_BASE_QSPI0		0x3FEFA000

/* I2C configuration */
#define CONFIG_SH_RIIC
#define CONFIG_HARD_I2C
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_MAX_I2C_BUS		4
#define CONFIG_SYS_I2C_SPEED		100000 /* 100 kHz */
#define CONFIG_SYS_I2C_EEPROM_ADDR	0x50
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	1
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	4	/* 16-Byte Write Mode */
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	10	/* 10ms of delay */
#define CONFIG_SYS_I2C_MODULE		3
#define CONFIG_SH_I2C_BASE0		0xFCFEE000
#define CONFIG_SH_I2C_BASE1		0xFCFEE400
#define CONFIG_SH_I2C_BASE2		0xFCFEE800
#define CONFIG_SH_I2C_BASE3		0xFCFEEc00

/* RTC configuration */
#define CONFIG_RTC_RZA1
#define CONFIG_RTC_RZA1_BASE_ADDR	0xFCFF1000

/* Board Clock */
#define CONFIG_SYS_CLK_FREQ	66666666 /* P1 clock. */
#define CONFIG_SYS_HZ		1000

/* Network interface */
#define CONFIG_SH_ETHER
#define CONFIG_SH_ETHER_USE_PORT	0
#define CONFIG_SH_ETHER_PHY_ADDR	0
#define CONFIG_SH_ETHER_PHY_MODE PHY_INTERFACE_MODE_MII
#define CONFIG_SH_ETHER_EEPROM_ADDR	2 /* MAC address offset in EEPROM */
#define CONFIG_PHYLIB
#define CONFIG_BITBANGMII
#define CONFIG_BITBANGMII_MULTI

/* USB host controller */
#define CONFIG_USB_R8A66597_HCD
#define CONFIG_R8A66597_BASE_ADDR	0xE8207000
#define CONFIG_R8A66597_XTAL		0x0000	/* 48MHz */
#define CONFIG_R8A66597_ENDIAN		0x0000	/* little */

/*
 * Lowlevel configuration
 */
/* Disable WDT */
#define WTCSR_D		0xA518
#define WTCNT_D		0x5A00

/* Set clocks based on 13.3333MHz xtal */
#define FRQCR_D		0x1035	/* CPU= 300-400 MHz */

/* Enable all peripherals */
#define STBCR3_D	0x00000000
#define STBCR4_D	0x00000000
#define STBCR5_D	0x00000000
#define STBCR6_D	0x00000000
#define STBCR7_D	0x00000024
#define STBCR8_D	0x00000005
#define STBCR9_D	0x00000000
#define STBCR10_D	0x00000000
#define STBCR11_D	0x000000c0
#define STBCR12_D	0x000000f0

/* Port Control register */
/* Port1 Control register Reset */
#define PIBC1_D		0x0000
#define PBDC1_D		0x0000
#define PM1_D		0xffff
#define PMC1_D		0x0000
#define PIPC1_D		0x0000

/* Port1 Control register Set */
#define PBDC1_S		0x00c0
#define PFC1_S		0x4000	/* SCL3, SDA3, ET_COL, P15 */
#define PFCE1_S		0x4000
#define PFCAE1_S	0x0000
#define PIPC1_S		0x40c0
#define PMC1_S		0x40c0
#define P1_S		0x0000
#define PM1_S		0xffff
#define PIBC1_S		0xbf3f

/* Port2 Control register Reset */
#define PIBC2_D		0x0000
#define PBDC2_D		0x0000
#define PM2_D		0xffff
#define PMC2_D		0x0000
#define PIPC2_D		0x0000

/* Port2 Control register Set */
#define PBDC2_S		0xf000
#define PFC2_S		0xffff	/* ET_xxx */
#define PFCE2_S		0xf000	/* SPBIO01_0, SPBIO11_0, SPBIO21_0, SPBIO31_0 */
#define PFCAE2_S	0x0000
#define PIPC2_S		0xffff
#define PMC2_S		0xffff
#define P2_S		0x0000
#define PM2_S		0xffff
#define PIBC2_S		0x0000

/* Port3 Control register Reset */
#define PIBC3_D		0x0000
#define PBDC3_D		0x0000
#define PM3_D		0xffff
#define PMC3_D		0x0000
#define PIPC3_D		0x0000

/* Port3 Control register Set */
#define PBDC3_S		0x0008
#define PFC3_S		0x007d	/* TxD2, RxD2 */
#define PFCE3_S		0x0004	/* ET_MDIO, ET_RXCCLK, ET_RXER ET_RXDV */
#define PFCAE3_S	0x0001
#define PIPC3_S		0x007d
#define PMC3_S		0x007d
#define P3_S		0x0000
#define PM3_S		0xffff
#define PIBC3_S		0x0000

/* Port4 Control register Reset */
#define PIBC4_D		0x0000
#define PBDC4_D		0x0000
#define PM4_D		0xffff
#define PMC4_D		0x0000
#define PIPC4_D		0x0000

/* Port4 Control register Set */
#define PBDC4_S		0x0000
#define PFC4_S		0x0000
#define PFCE4_S		0x0000
#define PFCAE4_S	0x0000
#define PIPC4_S		0x0000
#define PMC4_S		0x0000
#define P4_S		0x0000
#define PM4_S		0xffff
#define PIBC4_S		0x0000

/* Port5 Control register Reset */
#define PIBC5_D		0x0000
#define PBDC5_D		0x0000
#define PM5_D		0xffff
#define PMC5_D		0x0000
#define PIPC5_D		0x0000

/* Port5 Control register Set */
#define PBDC5_S		0x0000
#define PFC5_S		0x0300	/* CS2, ET_MDC */
#define PFCE5_S		0x0000
#define PFCAE5_S	0x0100
#define PIPC5_S		0x0300
#define PMC5_S		0x0300
#define P5_S		0x0000
#define PM5_S		0xffff
#define PIBC5_S		0x0000

/* Port6 Control register Reset */
#if !defined(CONFIG_BOOT_MODE0)
#define PIBC6_D		0x0000
#define PBDC6_D		0x0000
#define PM6_D		0xffff
#define PMC6_D		0x0000
#define PIPC6_D		0x0000
#else
#define PIBC6_D		0x0000
#define PBDC6_D		0xffff
#define PM6_D		0xffff
#define PMC6_D		0xffff
#define PIPC6_D		0xffff
#endif

/* Port6 Control register Set */
#define PBDC6_S		0xffff
#define PFC6_S		0x0000	/* D1 - D15 */
#define PFCE6_S		0x0000
#define PFCAE6_S	0x0000
#define PIPC6_S		0xffff
#define PMC6_S		0xffff
#define P6_S		0x0000
#define PM6_S		0xffff
#define PIBC6_S		0x0000

/* Port7 Control register Reset */
#if !defined(CONFIG_BOOT_MODE0)
#define PIBC7_D		0x0000
#define PBDC7_D		0x0000
#define PM7_D		0xffff
#define PMC7_D		0x0000
#define PIPC7_D		0x0000
#else
#define PIBC7_D		0x0000
#define PBDC7_D		0xff01
#define PM7_D		0xffff
#define PMC7_D		0xff01
#define PIPC7_D		0xff01
#endif

/* Port7 Control register Set */
#if !defined(CONFIG_BOOT_MODE0)
#define PBDC7_S		0x0000
#else
#define PBDC7_S		0xfffd
#endif
#define PFC7_S		0x0000	/* WE0/DQMLL, RD/WR, RD, CS0 */
#define PFCE7_S		0x0000	/* CKE, CAS, RAS, WE1/DQMLL, A7-A1 */
#define PFCAE7_S	0x0000
#define PIPC7_S		0xfffd
#define PMC7_S		0xfffd
#define P7_S		0x0000
#define PM7_S		0xffff
#define PIBC7_S		0x0000

/* Port8 Control register Reset */
#if !defined(CONFIG_BOOT_MODE0)
#define PIBC8_D		0x0000
#define PBDC8_D		0x0000
#define PM8_D		0xffff
#define PMC8_D		0x0000
#define PIPC8_D		0x0000
#else
#define PIBC8_D		0x0000
#define PBDC8_D		0x1fff
#define PM8_D		0xffff
#define PMC8_D		0x1fff
#define PIPC8_D		0x1fff
#endif

/* Port8 Control register Set */
#if !defined(CONFIG_BOOT_MODE0)
#define PBDC8_S		0x0000
#else
#define PBDC8_S		0xffff
#endif
#define PFC8_S		0x0000	/* A23-A8 */
#define PFCE8_S		0x0000	/* A23-A8 NOR, A15-A8 SDRAM */
#define PFCAE8_S	0x0000
#define PIPC8_S		0xffff
#define PMC8_S		0xffff
#define P8_S		0x0000
#define PM8_S		0xffff
#define PIBC8_S		0x0000

/* Port9 Control register Reset */
#if !defined(CONFIG_BOOT_MODE0)
#define PIBC9_D		0x0000
#define PBDC9_D		0x0000
#define PM9_D		0xffff
#define PMC9_D		0x00ff
#define PIPC9_D		0x00ff
#else
#define PIBC9_D		0x0000
#define PBDC9_D		0x0000
#define PM9_D		0xffff
#define PMC9_D		0x0003
#define PIPC9_D		0x0003
#endif

/* Port9 Control register Set */
#define PBDC9_S		0x00f0
#define PFC9_S		0x00fc	/* A25,A24 NOR, P9_2-P9_7(SPBxxx) SPI Flash */
#define PFCE9_S		0x0000	/* SPBIO00_0, SPBIO10_0, SPBIO20_0, SPBIO20_0 */
#define PFCAE9_S	0x0000
#define PIPC9_S		0x00ff
#define PMC9_S		0x00ff
#define P9_S		0x0000
#define PM9_S		0xffff
#define PIBC9_S		0x0000

/* Port10 Control register Reset */
#define PIBC10_D	0x0000
#define PBDC10_D	0x0000
#define PM10_D		0xffff
#define PMC10_D		0x0000
#define PIPC10_D	0x0000

/* Port10 Control register Set */
#define PBDC10_S	0x0000
#define PFC10_S		0x0000
#define PFCE10_S	0x0000
#define PFCAE10_S	0x0000
#define PIPC10_S	0x0000
#define PMC10_S		0x0000
#define P10_S		0x0000
#define PM10_S		0xffff
#define PIBC10_S	0x0000

/* Port11 Control register Reset */
#define PIBC11_D	0x0000
#define PBDC11_D	0x0000
#define PM11_D		0xffff
#define PMC11_D		0x0000
#define PIPC11_D	0x0000

/* Port11 Control register Set */
#define PBDC11_S	0x0000
#define PFC11_S		0x0000
#define PFCE11_S	0x0000
#define PFCAE11_S	0x0000
#define PIPC11_S	0x0000
#define PMC11_S		0x0000
#define P11_S		0x0000
#define PM11_S		0xffff
#define PIBC11_S	0x0000

/* Configure NOR Flash (CS0, CS1) */
#define CS0WCR_D	0x00000b40
#define CS0BCR_D	0x10000C00
#define CS1WCR_D	0x00000b40
#define CS1BCR_D	0x10000C00

/* Configure SDRAM (CS2, CS3) */
#define CS2BCR_D	0x00004C00
#define CS2WCR_D	0x00000480
#define CS3BCR_D	0x00004C00
#define CS3WCR_D	0x00004492
#define SDCR_D		0x00110811
#define RTCOR_D		0xA55A0080
#define RTCSR_D		0xA55A0008

#endif	/* __RSKRZA1_H */
