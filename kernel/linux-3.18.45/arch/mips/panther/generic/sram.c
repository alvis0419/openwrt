/* 
 *  Copyright (c) 2016  Montage Inc.	All rights reserved. 
 *
 *  Routines to allocate SRAM resources
 *
 */

#include <linux/string.h>
#include <linux/kernel.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/cache.h>

#if defined(CONFIG_CHEETAH_WLAN)
extern unsigned int wlan_driver_sram_size;
extern void* wlan_driver_sram_base;
#endif

// address range of 2nd 16K of SRAM
#define SRAM_ADDRESS_BASE       0xB0004000UL
#define SRAM_ADDRESS_LIMIT      0xB0008000UL

void sram_init(void)
{
    unsigned long sram_alloc_ptr;

    sram_alloc_ptr = SRAM_ADDRESS_BASE;

#if defined(CONFIG_CHEETAH_WLAN)
    if(wlan_driver_sram_size)
    {
        printk("L1 cache line size %d\n", L1_CACHE_BYTES);
        printk("SRAM: %08X-%08X (%d bytes) allocated to WLAN driver\n",
                 (unsigned) sram_alloc_ptr, ((unsigned) sram_alloc_ptr + wlan_driver_sram_size - 1), wlan_driver_sram_size);

        sram_alloc_ptr += wlan_driver_sram_size;

        // re-align to cache-line size
        if(sram_alloc_ptr % L1_CACHE_BYTES)
            sram_alloc_ptr += (L1_CACHE_BYTES - sram_alloc_ptr % L1_CACHE_BYTES);
    }
#endif

    if(sram_alloc_ptr>SRAM_ADDRESS_LIMIT)
        panic("SRAM: out of sram resource\n");
}

