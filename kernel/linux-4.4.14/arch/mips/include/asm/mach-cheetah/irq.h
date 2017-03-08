#ifndef __ASM_CHEETAH_IRQ_H
#define __ASM_CHEETAH_IRQ_H

#define MIPS_CPU_IRQ_BASE       0
#define PANTHER_CASCADE_IRQ     (MIPS_CPU_IRQ_BASE + 2)

#define PANTHER_IRQ_BASE        8
#define PANTHER_IRQ_END         39

#define NR_IRQS                 40

#define IRQ_PDMA        (PANTHER_IRQ_BASE + 0)
#define IRQ_UART0       (PANTHER_IRQ_BASE + 1)  
#define IRQ_UART1       (PANTHER_IRQ_BASE + 2)
#define IRQ_UART2       (PANTHER_IRQ_BASE + 3)
#define IRQ_SFLASH      (PANTHER_IRQ_BASE + 4)
#define IRQ_TIMER0      (PANTHER_IRQ_BASE + 5)
#define IRQ_TIMER1      (PANTHER_IRQ_BASE + 6)
#define IRQ_TIMER2      (PANTHER_IRQ_BASE + 7)

#define IRQ_I2C         (PANTHER_IRQ_BASE + 12)
#define IRQ_GDMA        (PANTHER_IRQ_BASE + 13)
#define IRQ_WIFI        (PANTHER_IRQ_BASE + 14)
#define IRQ_CSS         (PANTHER_IRQ_BASE + 15)
#define IRQ_USB_OTG     (PANTHER_IRQ_BASE + 16)
#define IRQ_USB_HOST    (PANTHER_IRQ_BASE + 17)
#define IRQ_SDIO        (PANTHER_IRQ_BASE + 18)
#define IRQ_ETHSW       (PANTHER_IRQ_BASE + 19)
#define IRQ_HWNAT       (PANTHER_IRQ_BASE + 20)
#define IRQ_PCM         (PANTHER_IRQ_BASE + 21)

#include_next <irq.h>

#endif /* __ASM_CHEETAH_IRQ_H */

