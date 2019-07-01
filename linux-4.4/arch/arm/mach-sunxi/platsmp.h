/*
 * arch/arm/mach-sunxi/platsmp.h
 *
 * Copyright(c) 2013-2015 Allwinnertech Co., Ltd.
 *      http://www.allwinnertech.com
 *
 * Author: east_yang <yangdong@allwinnertech.com>
 *
 * sunxi smp ops header file
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __PLATSMP_H
#define __PLATSMP_H

#include <linux/platform_data/mach_sunxi.h>

#ifdef CONFIG_CPU_IDLE_SUNXI
#include <linux/sunxi-cpuidle.h>
#endif

#define CPUCFG_CPU_RST_CTRL_REG(cpu)   (((cpu) + 1) * 0x40)
#define CPUCFG_CPU_STATUS_REG(cpu)     (((cpu) + 1) * 0x40 + 0x08)
#define CPUCFG_GEN_CTRL_REG            (0x184)
#define CPUCFG_DBG_CTL1_REG            (0x1e4)
#define CPUCFG_PWROFF_GATING_REG       (0x110)
#define CPUCFG_PWR_SWITCH_REG(cpu)     (0x120 + (cpu)*0x04)
#define CPUCFG_PRIVATE0_REG            0x1a4
#define PRCM_CPU_PWROFF_REG            0x100
#define PRCM_CPU_PWR_CLAMP_REG(cpu)    (((cpu) * 4) + 0x140)

extern void __iomem *sunxi_cpucfg_base;
extern void __iomem *sunxi_cpuscfg_base;
extern void __iomem *sunxi_sysctl_base;
extern void __iomem *sunxi_rtc_base;
extern void __iomem *sunxi_soft_entry_base;

extern void sunxi_secondary_startup(void);
extern void sunxi_cpu_die(unsigned int cpu);
extern int  sunxi_cpu_kill(unsigned int cpu);
extern int  sunxi_cpu_disable(unsigned int cpu);

#ifdef CONFIG_ARCH_SUN8IW11
#include "platsmp-v1.h"
#elif defined CONFIG_ARCH_SUN8IW7
#include "platsmp-v3.h"
#else
#include "platsmp-v2.h"
#endif

#define get_nr_cores()					\
	({						\
		unsigned int __val;			\
		asm("mrc	p15, 1, %0, c9, c0, 2"	\
		    : "=r" (__val)			\
		    :					\
		    : "cc");				\
		((__val>>24) & 0x03) + 1;		\
	})

#endif /* __PLATSMP_H */
