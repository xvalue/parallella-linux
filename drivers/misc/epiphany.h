#ifndef __EPIPHANY_H__
#define __EPIPHANY_H__

#include <uapi/misc/epiphany.h>

/* Epiphany system registers */
enum e_sys_reg_id {
	E_SYS_RESET		= 0x0040,
	E_SYS_CFGTX		= 0x0044,
	E_SYS_CFGRX		= 0x0048,
	E_SYS_CFGCLK		= 0x004c,
	E_SYS_COREID		= 0x0050,
	E_SYS_VERSION		= 0x0054,
	E_SYS_GPIOIN		= 0x0058,
	E_SYS_GPIOOUT		= 0x005c
};

/* Chip registers */
enum e_chip_regs {
	E_REG_LINKCFG		= 0xf0300,
	E_REG_LINKTXCFG		= 0xf0304,
	E_REG_LINKRXCFG		= 0xf0308,
	E_REG_GPIOCFG		= 0xf030c,
	E_REG_FLAGCFG		= 0xf0318,
	E_REG_SYNC		= 0xf031c,
	E_REG_HALT		= 0xf0320,
	E_REG_RESET		= 0xf0324,
	E_REG_LINKDEBUG		= 0xf0328
};

enum e_ctrlmode {
	E_CTRLMODE_NORMAL	= 0,
	E_CTRLMODE_DMA0_LAST	= 4,
	E_CTRLMODE_DMA1_LAST	= 8,
	E_CTRLMODE_MSGMODE	= 12,
	E_CTRLMODE_MULTICAST	= 3,
	E_CTRLMODE_NORTH	= 1,
	E_CTRLMODE_EAST		= 5,
	E_CTRLMODE_SOUTH	= 9,
	E_CTRLMODE_WEST		= 13
};

enum e_core_reg {
	E_REG_BASE		= 0xf0000,
	E_REG_CONFIG		= 0xf0400,
	E_REG_MESHCONFIG	= 0xf0700
};

union e_syscfg_tx {
	u32 reg;
	struct {
		unsigned int enable:1;
		unsigned int mmu:1;
		unsigned int mode:2;      /* 0=Normal, 1=GPIO */
		unsigned int ctrlmode:4;
		unsigned int clkmode:4;   /* 0=Full speed, 1=1/2 speed */
		unsigned int resvd:20;
	};
};

union e_syscfg_rx {
	u32 reg;
	struct {
		unsigned int enable:1;
		unsigned int mmu:1;
		unsigned int path:2;    /* 0=Normal, 1=GPIO, 2=Loopback */
		unsigned int monitor:1;
		unsigned int resvd:27;
	};
};

union e_syscfg_clk {
	u32 reg;
	struct {
		unsigned int divider:4;  /* 0=off, 1=F/64 ... 7=F/1 */
		unsigned int pll:4;      /* TBD */
		unsigned int resvd:24;
	};
};

union e_syscfg_coreid {
	u32 reg;
	struct {
		unsigned int col:6;
		unsigned int row:6;
		unsigned int resvd:20;
	};
};

union e_syscfg_version {
	u32 reg;
	struct {
		unsigned int revision:8;
		unsigned int type:8;
		unsigned int platform:8;
		unsigned int generation:7;
		unsigned int debug:1;
	};
};

/* The following is for E_SYS_GPIOIN and E_SYS_GPIOOUT */
union e_syscfg_gpio {
	u32 reg;
	struct {
		unsigned int data:8;
		unsigned int frame:1;
		unsigned int wait_rd:1;
		unsigned int wait_wr:1;
		unsigned int resvd:21;
	};
};

#endif /* __EPIPHANY_H__ */
