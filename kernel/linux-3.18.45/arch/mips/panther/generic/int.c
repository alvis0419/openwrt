/* 
 *  Copyright (c) 2008, 2016, 2017    Montage Inc.    All rights reserved. 
 */
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel_stat.h>
#include <linux/kernel.h>
#include <linux/random.h>

#include <asm/traps.h>
#include <asm/irq_cpu.h>
#include <asm/irq_regs.h>
#include <asm/bitops.h>
#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/mach-cheetah/irq.h>
#include <asm/mach-cheetah/cheetah.h>

static DEFINE_SPINLOCK(panther_irq_lock);

#if defined(CONFIG_MIPS_MT_SMP)

// indicate gic_present 0 to vsmp_init_secondary()
int gic_present;

static inline int clz(unsigned long x)
{
	__asm__(
	"	.set	push					\n"
	"	.set	mips32					\n"
	"	clz	%0, %1					\n"
	"	.set	pop					\n"
	: "=r" (x)
	: "r" (x));

	return x;
}

static inline unsigned int irq_ffs(unsigned int pending)
{
    return PANTHER_IRQ_END - clz(pending);
}

volatile unsigned long _int_mask[CONFIG_NR_CPUS];
#define _IC_STATUS(cpu) (((cpu)==0) ? (*((volatile unsigned long *)0xbf004900UL)) : (*((volatile unsigned long *)0xbf004910UL)))
#define _IC_MASK(cpu)   (((cpu)==0) ? (*((volatile unsigned long *)0xbf004908UL)) : (*((volatile unsigned long *)0xbf004918UL)))
#define _IC_MASK0       (*((volatile unsigned long *)0xbf004908UL))
#define _IC_MASK1       (*((volatile unsigned long *)0xbf004918UL))

asmlinkage void plat_irq_dispatch(void)
{
    int irq;
    int cpu;

    unsigned long pending = read_c0_cause() & read_c0_status();

    if(pending&CAUSEF_IP7) 
    {
        do_IRQ(7);
        return;
    }

    if(pending&CAUSEF_IP0) 
    {
        do_IRQ(0);
        return;
    }

    if(pending&CAUSEF_IP1) 
    {
        do_IRQ(1);
        return;
    }

    cpu = smp_processor_id();

    pending = _IC_STATUS(cpu) & ~_int_mask[cpu];
    if(pending)
    {
        irq = irq_ffs(pending);

        do_IRQ(irq);
    }
    else
    {
        spurious_interrupt();
    }
}

#define SMP_IRQ_ALLOCATION_CPU0 0xfffffff3

unsigned long curr_irq_allocation_cpu0 = SMP_IRQ_ALLOCATION_CPU0;
unsigned long __irqs_reassigned = 0;

void panther_irq_spin_lock(void)
{
    spin_lock(&panther_irq_lock);
}

void panther_irq_spin_unlock(void)
{
    spin_unlock(&panther_irq_lock);
}

/* shall be called with panther_irq_lock locked */
void reassign_irqs(void)
{
    _int_mask[0] = _int_mask[0] & _int_mask[1];
    _int_mask[1] = 0xffffffff;

    curr_irq_allocation_cpu0 = 0xffffffff;

    _IC_MASK0 = _int_mask[0];
    _IC_MASK1 = _int_mask[1];
    
    __irqs_reassigned = 1;
}

void restore_reassigned_irqs(void)
{
    unsigned long flags;

    spin_lock_irqsave(&panther_irq_lock, flags);

    if(__irqs_reassigned)
    {
        curr_irq_allocation_cpu0 = SMP_IRQ_ALLOCATION_CPU0;

        _int_mask[1] = _int_mask[1] & (_int_mask[0] | SMP_IRQ_ALLOCATION_CPU0);
        _int_mask[0] = _int_mask[0] | (~SMP_IRQ_ALLOCATION_CPU0);
    
        _IC_MASK0 = _int_mask[0];
        _IC_MASK1 = _int_mask[1];
        
        __irqs_reassigned = 0;
    }
    
    spin_unlock_irqrestore(&panther_irq_lock, flags);
}

void resume_panther_irq(void)
{
    _IC_MASK0 = _int_mask[0];
    _IC_MASK1 = _int_mask[1];
}

void disable_panther_irq(struct irq_data *d)
{
    //int cpu = smp_processor_id();
    unsigned int irq_nr = d->irq;
    int cpu;
    unsigned long flags;

    irq_nr -= PANTHER_IRQ_BASE;

    spin_lock_irqsave(&panther_irq_lock, flags);

    if(curr_irq_allocation_cpu0 & (0x01 << irq_nr))
        cpu = 0;
    else
        cpu = 1;

    if(cpu)
        _IC_MASK1 |= (1 << irq_nr);
    else
        _IC_MASK0 |= (1 << irq_nr);

    /* read back */
    _int_mask[cpu] = _IC_MASK(cpu);

    spin_unlock_irqrestore(&panther_irq_lock, flags);
}

void enable_panther_irq(struct irq_data *d)
{
    //int cpu = smp_processor_id();
    unsigned int irq_nr = d->irq;
    int cpu;
    unsigned long flags;

    irq_nr -= PANTHER_IRQ_BASE;

    spin_lock_irqsave(&panther_irq_lock, flags);

    if(curr_irq_allocation_cpu0 & (0x01 << irq_nr))
        cpu = 0;
    else
        cpu = 1;

    if(cpu)
        _IC_MASK1 &= ~(1 << irq_nr);
    else
        _IC_MASK0 &= ~(1 << irq_nr);

    /* read back */
    _int_mask[cpu] = _IC_MASK(cpu);

    spin_unlock_irqrestore(&panther_irq_lock, flags);
}

#define MIPS_CPU_IPI_RESCHED_IRQ    0
#define MIPS_CPU_IPI_CALL_IRQ       1
static int cpu_ipi_resched_irq, cpu_ipi_call_irq;

static irqreturn_t ipi_resched_interrupt(int irq, void *dev_id)
{
    return IRQ_HANDLED;
}

static irqreturn_t ipi_call_interrupt(int irq, void *dev_id)
{
    smp_call_function_interrupt();
    
    return IRQ_HANDLED;
}

static struct irqaction irq_resched = {
    .handler    = ipi_resched_interrupt,
    .flags      = IRQF_DISABLED|IRQF_PERCPU,
    .name       = "IPI_resched"
};

static struct irqaction irq_call = {
    .handler    = ipi_call_interrupt,
    .flags      = IRQF_DISABLED|IRQF_PERCPU,
    .name       = "IPI_call"
};

void __init arch_init_ipiirq(int irq, struct irqaction *action)
{
    setup_irq(irq, action);
    set_irq_handler(irq, handle_percpu_irq);
}

#else

#if 1

static inline int clz(unsigned long x)
{
	__asm__(
	"	.set	push					\n"
	"	.set	mips32					\n"
	"	clz	%0, %1					\n"
	"	.set	pop					\n"
	: "=r" (x)
	: "r" (x));

	return x;
}

/*
 * Version of ffs that only looks at bits 12..15.
 */
static inline unsigned int irq_ffs(unsigned int pending)
{
    return PANTHER_IRQ_END - clz(pending);
}

volatile unsigned long _int_mask = 0xFFFFFFFF;
#define _IC_STATUS  (*((volatile unsigned long *)0xbf004900UL))
#define _IC_MASK    (*((volatile unsigned long *)0xbf004908UL))

asmlinkage void plat_irq_dispatch(void)
{
    unsigned int cause = read_c0_cause();
    int irq;
    unsigned long pending;

    if (cause & CAUSEF_IP7) {	/* R4000 count / compare IRQ */
            do_IRQ(7);
#if 0 /* Temp. work-around before INTC ready */
            if((_int_mask & 0x2)==0)
                do_IRQ(9);
            if((_int_mask & (0x01 << 13))==0)
                do_IRQ(21);             
#endif
            return;
    }

    pending = _IC_STATUS & ~_int_mask;
    if(pending)
    {
        irq = irq_ffs(pending);

        do_IRQ(irq);
    }
}

#else

asmlinkage void plat_irq_dispatch(void)
{
    unsigned int pending = read_c0_cause() & read_c0_status() & ST0_IM;
    int irq;

    irq = irq_ffs(pending);

#if 0
    if (irq == MIPSCPU_INT_I8259A)
            malta_hw0_irqdispatch();
    else if (gic_present && ((1 << irq) & ipi_map[smp_processor_id()]))
            malta_ipi_irqdispatch();
#endif
    if (irq >= 0)
            do_IRQ(MIPS_CPU_IRQ_BASE + irq);
    else
            spurious_interrupt();
}
#endif


void disable_panther_irq(struct irq_data *d)
{
    unsigned int irq_nr = d->irq;
    unsigned long flags;

    irq_nr -= PANTHER_IRQ_BASE;

    spin_lock_irqsave(&panther_irq_lock, flags);    
    _IC_MASK |= (1 << irq_nr);

    /* read back */
    _int_mask = _IC_MASK;
    spin_unlock_irqrestore(&panther_irq_lock, flags);
}

void enable_panther_irq(struct irq_data *d)
{
    unsigned int irq_nr = d->irq;
    unsigned long flags;

    irq_nr -= PANTHER_IRQ_BASE;

    spin_lock_irqsave(&panther_irq_lock, flags);
    _IC_MASK &= ~(1 << irq_nr);

    /* read back */
    _int_mask = _IC_MASK;
    spin_unlock_irqrestore(&panther_irq_lock, flags);
}

void resume_panther_irq(void)
{
    _IC_MASK = _int_mask;
}

#endif // CONFIG_MIPS_MT_SMP

static struct irq_chip panther_irq_type = {
    .name = "Panther_IRQ",
    .irq_mask = disable_panther_irq,
    .irq_unmask = enable_panther_irq,
};

static struct irqaction cascade = {
    .handler	= no_action,
    .name	= "cascade",
};

void __init arch_init_irq(void)
{
    int i;

#if defined(CONFIG_MIPS_MT_SMP)
    for(i=0;i<CONFIG_NR_CPUS;i++)
    {
        _int_mask[i] = 0xffffffffUL;
    }
#endif

    mips_cpu_irq_init();

    for (i = PANTHER_IRQ_BASE; i <= PANTHER_IRQ_END; i++)
    {
        irq_set_chip_and_handler_name(i,
                                      &panther_irq_type,
                                      handle_level_irq,
                                      NULL);
    }

    setup_irq(PANTHER_CASCADE_IRQ, &cascade);

#if defined(CONFIG_MIPS_MT_SMP)
    /* set up ipi interrupts */
    cpu_ipi_resched_irq = MIPS_CPU_IRQ_BASE + MIPS_CPU_IPI_RESCHED_IRQ;
    cpu_ipi_call_irq = MIPS_CPU_IRQ_BASE + MIPS_CPU_IPI_CALL_IRQ;
    arch_init_ipiirq(cpu_ipi_resched_irq, &irq_resched);
    arch_init_ipiirq(cpu_ipi_call_irq, &irq_call);
#endif

}

