/* 
 *  Copyright (c) 2013	Montage Inc.	All rights reserved. 
 */
#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/pci.h>
#include <linux/screen_info.h>
#include <linux/time.h>

#include <asm/bootinfo.h>
#include <asm/mach-cheetah/generic.h>
#include <asm/mach-cheetah/prom.h>
#include <asm/dma.h>
#include <asm/traps.h>
#ifdef CONFIG_VT
#include <linux/console.h>
#endif

const char *get_system_type(void)
{
	return "Montage Soc";
}


#ifdef CONFIG_BLK_DEV_IDE
/* check this later */
#endif

extern void cheetah_reboot_setup(void);
extern void sram_init(void);
void __init plat_mem_setup(void)
{
	sram_init();

#ifdef CONFIG_DMA_COHERENT
#error CONFIG_DMA_COHERENT not applicable!
#endif

	cheetah_reboot_setup();

	/* bus error handler setup */
	board_be_init = NULL;
	board_be_handler = NULL;
}


