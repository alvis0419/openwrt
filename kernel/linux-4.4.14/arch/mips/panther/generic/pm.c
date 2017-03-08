/* 
 *  Copyright (c) 2016	Montage Inc.	All rights reserved. 
 */
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/suspend.h>
#include <linux/sysfs.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/compiler.h>
#include <linux/smp.h>

#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/cpu.h>
#include <asm/processor.h>

#include <asm/io.h>
#include <asm/reboot.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/reg.h>
#include <asm/mach-cheetah/pmu.h>

#include "fastboot.h"
//#include "cache.h"

static DEFINE_SPINLOCK(system_suspend_lock);

extern void resume_panther_irq(void);
extern void resume_uart(void);
void cheetah_serial_outc(unsigned char c);

#if defined(CONFIG_MIPS_MT_SMP)
void vsmp_resume(void);
void vsmp_suspend(void);
#endif

/* 
   NOTE:
   
   1. test suspend function with following command
      The system will offline all CPUs except CPU0 and then suspend
      echo mem > /sys/power/state
   
   2. stand-alone test cpu1 offline
      echo 0 > /sys/devices/system/cpu/cpu1/online 

   3. stand-alone test cpu1 online
      echo 1 > /sys/devices/system/cpu/cpu1/online  
*/
void panther_machine_suspend(void)
{
    void (*func) (void);
    unsigned long flags;

    spin_lock_irqsave(&system_suspend_lock, flags);

    memcpy((void *)0x90000000, fastboot, sizeof(fastboot));

    local_flush_tlb_all();
    //flush_tlb_all();
    //HAL_DCACHE_WB_INVALIDATE_ALL();
    //HAL_ICACHE_INVALIDATE_ALL();
    __flush_cache_all();
    flush_icache_all();

    REG_WRITE32(PMU_WATCHDOG, 0x0);

#if defined(CONFIG_MIPS_MT_SMP)
    vsmp_suspend();
#endif

    func = (void *) 0x90000000UL;
    func();
    asm volatile ("nop;nop;nop;");

    //printk("BACK!!!!\n");

    resume_uart();
    resume_panther_irq();

#if defined(CONFIG_MIPS_MT_SMP)
    vsmp_resume();
#endif

    //cheetah_serial_outc('B'); cheetah_serial_outc('a'); cheetah_serial_outc('c'); cheetah_serial_outc('k');
    //cheetah_serial_outc('!'); cheetah_serial_outc('\r'); cheetah_serial_outc('\n');

    spin_unlock_irqrestore(&system_suspend_lock, flags);
}

static int panther_pm_enter(suspend_state_t state)
{
    printk("panther_pm_enter\n");
    panther_machine_suspend();
    return 0;
}


static int panther_pm_begin(suspend_state_t state)
{
    printk("panther_pm_begin\n");
    return 0;
}

static void panther_pm_end(void)
{
    printk("panther_pm_end\n");
}

static struct platform_suspend_ops panther_pm_ops = {
	.valid		= suspend_valid_only_mem,
	.begin		= panther_pm_begin,
	.enter		= panther_pm_enter,
	.end		= panther_pm_end,
};

static int __init pm_init(void)
{
    suspend_set_ops(&panther_pm_ops);

    //return sysfs_create_group(power_kobj, &panther_pmattr_group);
    return 0;
}

late_initcall(pm_init);
