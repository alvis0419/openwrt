/* 
 *  Copyright (c) 2016	Montage Inc.	All rights reserved. 
 */
#include <linux/pm.h>
#include <asm/io.h>
#include <asm/reboot.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/reg.h>
#include <asm/mach-cheetah/pmu.h>

static DEFINE_SPINLOCK(system_reset_lock);

static void cheetah_machine_restart(char *command);
static void cheetah_machine_halt(void);
static void cheetah_machine_power_off(void);

static void cheetah_machine_restart(char *command)
{
    unsigned long flags;

    spin_lock_irqsave(&system_reset_lock, flags);

    REG_WRITE32(PMU_WATCHDOG, 0x01000001);

    while(1)
        ;
}

static void cheetah_machine_halt(void)
{
    unsigned long flags;

    spin_lock_irqsave(&system_reset_lock, flags);

    REG_WRITE32(PMU_WATCHDOG, 0x0);

    while(1)
        ;
}

static void cheetah_machine_power_off(void)
{
    printk("System halted. Please turn off power.\n");
    cheetah_machine_halt();
}

void cheetah_reboot_setup(void)
{
	_machine_restart = cheetah_machine_restart;
	_machine_halt = cheetah_machine_halt;
	pm_power_off = cheetah_machine_power_off;
}

void pmu_reset_devices(unsigned long *device_ids)
{
    unsigned long flags;
    int i = 0;
    unsigned long curr_id;
    u32 pmu_reset_reg24_mask = 0;
    u32 pmu_reset_reg25_mask = 0;

    if(device_ids==NULL)
        return;

    while(1)
    {
        curr_id = device_ids[i];
        if((curr_id/100)==24)
            pmu_reset_reg24_mask |= (0x01 << (curr_id%100));
        else if((curr_id/100)==25)
            pmu_reset_reg25_mask |= (0x01 << (curr_id%100));
        else
            break;

        i++;
    }

    spin_lock_irqsave(&system_reset_lock, flags);

    if(pmu_reset_reg24_mask)
        REG_UPDATE32(PMU_RESET_REG24, 0x0, pmu_reset_reg24_mask);

    if(pmu_reset_reg25_mask)
        REG_UPDATE32(PMU_RESET_REG25, 0x0, pmu_reset_reg25_mask);

    if(pmu_reset_reg25_mask)
        REG_UPDATE32(PMU_RESET_REG25, 0xffffffff, pmu_reset_reg25_mask);

    if(pmu_reset_reg24_mask)
        REG_UPDATE32(PMU_RESET_REG24, 0xffffffff, pmu_reset_reg24_mask);

    spin_unlock_irqrestore(&system_reset_lock, flags);
}

void pmu_reset_devices_no_spinlock(unsigned long *device_ids)
{
    int i = 0;
    unsigned long curr_id;
    u32 pmu_reset_reg24_mask = 0;
    u32 pmu_reset_reg25_mask = 0;

    if(device_ids==NULL)
        return;

    while(1)
    {
        curr_id = device_ids[i];
        if((curr_id/100)==24)
            pmu_reset_reg24_mask |= (0x01 << (curr_id%100));
        else if((curr_id/100)==25)
            pmu_reset_reg25_mask |= (0x01 << (curr_id%100));
        else
            break;

        i++;
    }

    if(pmu_reset_reg24_mask)
        REG_UPDATE32(PMU_RESET_REG24, 0x0, pmu_reset_reg24_mask);

    if(pmu_reset_reg25_mask)
        REG_UPDATE32(PMU_RESET_REG25, 0x0, pmu_reset_reg25_mask);

    if(pmu_reset_reg25_mask)
        REG_UPDATE32(PMU_RESET_REG25, 0xffffffff, pmu_reset_reg25_mask);

    if(pmu_reset_reg24_mask)
        REG_UPDATE32(PMU_RESET_REG24, 0xffffffff, pmu_reset_reg24_mask);
}

