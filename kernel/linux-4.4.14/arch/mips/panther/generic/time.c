/* 
 *  Copyright (c) 2013, 2017	Montage Inc.	All rights reserved. 
 */
#include <linux/clockchips.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>

#include <asm/time.h>
#include <asm/io.h>
#include <asm/setup.h>

unsigned long cpu_khz;

#if 1
unsigned int get_c0_compare_int(void)
{
        return CP0_LEGACY_COMPARE_IRQ;
        }
        
#else        
static int mips_cpu_timer_irq;

static void mips_timer_dispatch(void)
{
	do_IRQ(mips_cpu_timer_irq);
}


unsigned __cpuinit get_c0_compare_int(void)
{
#if 0 //def MSC01E_INT_BASE
	if (cpu_has_veic) {
		set_vi_handler(MSC01E_INT_CPUCTR, mips_timer_dispatch);
		mips_cpu_timer_irq = MSC01E_INT_BASE + MSC01E_INT_CPUCTR;

		return mips_cpu_timer_irq;
	}
#endif
	if (cpu_has_vint)
		set_vi_handler(cp0_compare_irq, mips_timer_dispatch);
	mips_cpu_timer_irq = MIPS_CPU_IRQ_BASE + cp0_compare_irq;

	return mips_cpu_timer_irq;
}
#endif

void __init plat_time_init(void)
{
	unsigned int est_freq;

	est_freq = 75000000;

	cpu_khz = est_freq / 1000;

	mips_hpt_frequency = est_freq / 2;
}

