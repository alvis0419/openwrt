/* 
 *  Copyright (c) 2013	Montage Inc.	All rights reserved. 
 */
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/pfn.h>
#include <linux/string.h>

#include <asm/bootinfo.h>
#include <asm/page.h>
#include <asm/sections.h>

extern  void panther_serial_outc(unsigned char c);

void __init prom_meminit(void)
{

}

void __init prom_free_prom_memory(void)
{

}

int prom_putchar(char c)
{
	panther_serial_outc(c);
    return 1;
}

