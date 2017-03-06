/* 
 *  Copyright (c) 2013, 2016	Montage Inc.	All rights reserved. 
 *  
 *  Serial driver for Panther
 *
 */

/* force to turn on CONFIG_SERIAL_CONSOLE option */
#ifndef CONFIG_SERIAL_CONSOLE
#define CONFIG_SERIAL_CONSOLE
#endif

#define CONFIG_CHEETAH_UART_STAT

#include <linux/init.h>

#if defined(CONFIG_KGDB)
    #include <linux/kgdb.h>
#endif

#if defined(CONFIG_SERIAL_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
    #define SUPPORT_SYSRQ

/* use Ctrl-Break-h to invoke the SYSRQ help menu */
    #include <linux/sysrq.h>
#endif

#include <linux/delay.h>


#include <linux/module.h>

#include <linux/types.h>
#include <linux/circ_buf.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>

#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/console.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#ifdef CONFIG_CHEETAH_UART_STAT
    #include <linux/seq_file.h>
#endif
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/device.h>

#include <asm/setup.h>

#include <asm/irq.h>
#include <asm/io.h>

#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/pmu.h>

#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
#include <asm/mach-cheetah/idb.h>
#endif

#if defined(CONFIG_PANTHER_PDMA)
#include <asm/mach-cheetah/pdma.h>
#endif

#define NR_PORTS 3

#if defined(CONFIG_SERIAL_CONSOLE) || defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
static int cta_console_index;
#endif

#if defined(CONFIG_PANTHER_PDMA)

#define PDMA_BUF_MAX_NUM    8

struct pdma_ch_descr pdma_uart_rx_descr[NR_PORTS][PDMA_BUF_MAX_NUM] __attribute__ ((aligned(32)));
struct pdma_ch_descr pdma_uart_tx_descr[NR_PORTS][PDMA_BUF_MAX_NUM] __attribute__ ((aligned(32)));

#define PDMA_UART_BUFSIZE   4092 //PAGE_SIZE //SERIAL_XMIT_SIZE

#define pdma_enable_bitmap 0x07
//unsigned long pdma_enable_bitmap = 0x02;
static int pdma_enabled(int idx)
{
#if defined(CONFIG_SERIAL_CONSOLE) || defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
    if(cta_console_index==idx)
        return 0;
#endif

    if(pdma_enable_bitmap & (0x01 << idx))
        return 1;
    else
        return 0;
}

#endif

struct serial_state
{
    char devname[16];
    struct device *dev;
	struct tty_port	tport;
    int baud_base;
    unsigned long port;
    int irq;
    int flags;
    int type;
    int line;
    int xmit_fifo_size;
    int custom_divisor;
    int count;
    unsigned short  close_delay;
    unsigned short  closing_wait; /* time to wait before closing */
    struct async_icount icount;
    struct tasklet_struct uart_irq_tasklet;
    unsigned int uart_control;

    int curr_baudrate;

#if defined(CONFIG_PANTHER_PDMA)
    struct tasklet_struct pdma_rx_tasklet;
    int pdma_rx_buf_idx;
    int pdma_tx_buf_idx;
    unsigned char *pdma_rx_buf[PDMA_BUF_MAX_NUM];
    unsigned char *pdma_tx_buf[PDMA_BUF_MAX_NUM];
#endif

    int         read_status_mask;
    int         timeout;
    int         quot;
    int         x_char; /* xon/xoff character */
    int         MCR;    /* Modem control register */
    unsigned long       last_active;
    int         blocked_open; /* # of blocked opens */
    struct circ_buf     xmit;
};

static struct serial_state rs_table[NR_PORTS];
static DEFINE_SPINLOCK(uart_lock);

#define UARTREG(idx,reg)  (*(volatile unsigned int*)(UR_BASE+reg+(0x100 * idx)))

#define URCS_CTRL_MASK   (~((unsigned int)(1<<URCS_BRSHFT)-1)|URCS_TIE|URCS_RIE|URCS_PE|URCS_EVEN)

unsigned int urcs_cal_baud_cnt(u32 baudrate)
{   
    u32 retval;

    retval = (((PBUS_CLK * 10) / baudrate) + 5);
    retval = ((retval / 10) - 1);

    return retval;
}

void urcs_enable(int idx, unsigned int bitmap)
{
    if (idx>=NR_PORTS)
        return;

    rs_table[idx].uart_control |= bitmap;
    UARTREG(idx, URCS) = rs_table[idx].uart_control;
}

void urcs_disable(int idx, unsigned int bitmap)
{
    if (idx>=NR_PORTS)
        return;

    rs_table[idx].uart_control &= ~bitmap;
    UARTREG(idx, URCS) = rs_table[idx].uart_control;
}

void urcs_update_br(int idx, unsigned int br)
{
    if (idx>=NR_PORTS)
        return;

    rs_table[idx].uart_control = ((rs_table[idx].uart_control&0x8000FFFFUL)|(br<<URCS_BRSHFT));
    UARTREG(idx, URCS) = rs_table[idx].uart_control;
}

void resume_uart(void)
{
    int idx;
    
    for(idx=0;idx<NR_PORTS;idx++)
    {
        UARTREG(idx, URCS) = rs_table[idx].uart_control;
    }
}

#if defined(CONFIG_SERIAL_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
static unsigned long break_pressed[NR_PORTS];
#endif

static struct tty_driver *serial_driver;

/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS 256

static void uart_enable_rx(int idx)
{
    urcs_enable(idx, URCS_RXEN);
}

static void uart_disable_rx(int idx)
{
    urcs_disable(idx, URCS_RXEN);
}

static void rs_disable_tx_interrupts(int idx) 
{
    urcs_disable(idx, URCS_TIE);
}

static int transmit_chars(int idx);

#if defined(CONFIG_PANTHER_PDMA)

static void uart_tx_pdma_init(int idx)
{
    int i;
    unsigned int *p = (unsigned int *) (UR_BASE + (0x100 * idx));
    pdma_descriptor descriptor;

    rs_table[idx].pdma_tx_buf_idx = 0;
    descriptor.channel = PDMA_UART_TX_CHANNEL(idx);
    descriptor.dest_addr = PHYSICAL_ADDR((int)p + 0x10);
    descriptor.dma_total_len = 0;
    descriptor.aes_ctrl = 0;
    descriptor.intr_enable = 1;
    descriptor.src = 3;
    descriptor.dest = 0;
    descriptor.fifo_size = 3;
    descriptor.valid = 0; //PDMA_VALID;

    for (i = 0; i < PDMA_BUF_MAX_NUM; i++)
    {
        descriptor.desc_addr = UNCACHED_ADDR(&pdma_uart_tx_descr[idx][i]);
        descriptor.src_addr = PHYSICAL_ADDR(rs_table[idx].pdma_tx_buf[i]);

        if (i == (PDMA_BUF_MAX_NUM - 1))
            descriptor.next_addr = PHYSICAL_ADDR(&pdma_uart_tx_descr[idx][0]);
        else
            descriptor.next_addr = PHYSICAL_ADDR(&pdma_uart_tx_descr[idx][i + 1]);

        if (i == 0)
            pdma_loop_desc_set(&descriptor, (struct pdma_ch_descr *) UNCACHED_ADDR(&pdma_uart_tx_descr[idx][i]), DESC_TYPE_BASE);
        else
            pdma_loop_desc_set(&descriptor, (struct pdma_ch_descr *) UNCACHED_ADDR(&pdma_uart_tx_descr[idx][i]), DESC_TYPE_CHILD);
    }
}

static void uart_tx_pdma_done(u32 channel, u32 intr_status)
{    
    int idx = PDMA_CHANNEL_TO_UART_IDX(channel);

    tasklet_hi_schedule(&rs_table[idx].uart_irq_tasklet);
}

static void uart_rx_pdma_start(int idx)
{
    unsigned int *p = (unsigned int *) (UR_BASE + (0x100 * idx));
    int i;
    pdma_descriptor descriptor;    

    rs_table[idx].pdma_rx_buf_idx = 0;
    descriptor.channel = PDMA_UART_RX_CHANNEL(idx);
    descriptor.src_addr = PHYSICAL_ADDR((int)p + 0x10);
    descriptor.dma_total_len = PDMA_UART_BUFSIZE;
    descriptor.aes_ctrl = 0;
    descriptor.intr_enable = 1;
    descriptor.src = 0;
    descriptor.dest = 3;
    descriptor.fifo_size = 3;
    descriptor.valid = PDMA_VALID;
    for (i = 0; i < PDMA_BUF_MAX_NUM; i++)
    {
        descriptor.desc_addr = UNCACHED_ADDR(&pdma_uart_rx_descr[idx][i]);
        descriptor.dest_addr = PHYSICAL_ADDR(rs_table[idx].pdma_rx_buf[i]);

        if (i == (PDMA_BUF_MAX_NUM - 1))
            descriptor.next_addr = PHYSICAL_ADDR(&pdma_uart_rx_descr[idx][0]);
        else
            descriptor.next_addr = PHYSICAL_ADDR(&pdma_uart_rx_descr[idx][i + 1]);

        if (i == 0)
            pdma_loop_desc_set(&descriptor, (struct pdma_ch_descr *) UNCACHED_ADDR(&pdma_uart_rx_descr[idx][i]), DESC_TYPE_BASE);
        else
            pdma_loop_desc_set(&descriptor, (struct pdma_ch_descr *) UNCACHED_ADDR(&pdma_uart_rx_descr[idx][i]), DESC_TYPE_CHILD);
    }

    pdma_kick_channel(PDMA_UART_RX_CHANNEL(idx));
}

static void uart_pdma_stop(int idx)
{
    pdma_callback_unregister(PDMA_UART_TX_CHANNEL(idx));
    pdma_callback_unregister(PDMA_UART_RX_CHANNEL(idx));
}

static void uart_rx_pdma_done(u32 channel, u32 intr_status)
{
    int idx = PDMA_CHANNEL_TO_UART_IDX(channel);

    tasklet_hi_schedule(&rs_table[idx].pdma_rx_tasklet);
}

volatile u32 desc_config_read;
static void uart_rx_pdma_task(unsigned long idx)
{
    struct serial_state *info = rs_table[idx].info;
    struct tty_struct *tty = info->tport.tty;
    int rx_buf_idx;
    u32 len;
    unsigned char *data_buf;
    struct pdma_ch_descr *rxdescr;
    //int i;

    rxdescr = (struct pdma_ch_descr *) UNCACHED_ADDR(&pdma_uart_rx_descr[idx][0]);
    rx_buf_idx = rs_table[idx].pdma_rx_buf_idx;

#if 0
    printk("* %d\n", rs_table[idx].pdma_rx_buf_idx);
    for(i=0;i<PDMA_BUF_MAX_NUM;i++)
    {
        printk("  %d %08x\n", i, rxdescr[i].desc_config);
    }
#endif

    while(1)
    {
        //printk("- %d %08x\n", rx_buf_idx, rxdescr[rx_buf_idx].desc_config);

        if(rxdescr[rx_buf_idx].desc_config & PDMA_VALID)
            break;

        len = (rxdescr[rx_buf_idx].desc_config >> PDMA_TOTAL_LEN_SHIFT) & 0xffff;

        len++;

        #if 0
        // means PDMA got timeout status
        if ((uart_rx_data.ch_received >> (PDMA_CH_UART0_RX * 2 + 1)) == 0x1)
        {
            len++;
        }
        #endif

        //printk("len = %d\n", len);

        if (len >= PDMA_UART_BUFSIZE)
            len = PDMA_UART_BUFSIZE;
            
        if (len != 0)
        {
            data_buf = rs_table[idx].pdma_rx_buf[rx_buf_idx];
#if 1
            //tty_insert_flip_string_flags(tty, data_buf, &flag, len);
            tty_insert_flip_string(&rs_table[idx].tport, data_buf, len);

            dma_cache_inv((unsigned long) data_buf, len);
#else
            flag = TTY_NORMAL;
            for (i = 0; i < len; i++)
            {
                tty_insert_flip_char(tty, *(data_buf + i), flag);
            }
#endif
            tty_schedule_flip(tty);
            info->icount.rx += len;
        }

        rxdescr[rx_buf_idx].desc_config 
            = (rxdescr[rx_buf_idx].desc_config & 0x0000FFFF) | PDMA_VALID | (PDMA_UART_BUFSIZE << PDMA_TOTAL_LEN_SHIFT);

        desc_config_read = rxdescr[rx_buf_idx].desc_config;

        pdma_kick_channel(PDMA_UART_RX_CHANNEL(idx));

        rx_buf_idx = (rx_buf_idx+1) % PDMA_BUF_MAX_NUM;
        rs_table[idx].pdma_rx_buf_idx = rx_buf_idx;
    }
}

static void init_pdma_uart_data(void)
{
    int i, j;
    unsigned long page;

    for(i=0;i<NR_PORTS;i++)
    {
        if(pdma_enabled(i))
        {
            for(j=0;j<PDMA_BUF_MAX_NUM;j++)
            {
                page = get_zeroed_page(GFP_KERNEL);
                if(page)
                {
                    dma_cache_wback_inv(page, PDMA_UART_BUFSIZE);
                    rs_table[i].pdma_rx_buf[j] = (unsigned char *) page;
                }
                else
                {
                    panic("get_zeroed_page() failed\n");
                }
            }

            for(j=0;j<PDMA_BUF_MAX_NUM;j++)
            {
                page = get_zeroed_page(GFP_KERNEL);
                if(page)
                {
                    rs_table[i].pdma_tx_buf[j] = (unsigned char *) page;
                }
                else
                {
                    panic("get_zeroed_page() failed\n");
                }
            }
        }
    }
}
#endif


static void rs_enable_tx_interrupts(int idx)
{
#if defined(CONFIG_PANTHER_PDMA)
    if (pdma_enabled(idx))
    {
        tasklet_hi_schedule(&rs_table[idx].uart_irq_tasklet);
    }
    else
    {
        //if (transmit_chars(idx))
        urcs_enable(idx, URCS_TIE);
    }
#else
    //if (transmit_chars(idx))
    urcs_enable(idx, URCS_TIE);
#endif
}

static void rs_disable_rx_interrupts(int idx) 
{
    urcs_disable(idx, URCS_RIE);
}

static void rs_enable_rx_interrupts(int idx)
{
#if defined(CONFIG_PANTHER_PDMA)
    if (!pdma_enabled(idx))
        urcs_enable(idx, URCS_RIE);
#else
    urcs_enable(idx, URCS_RIE);
#endif
}

static void change_speed(struct tty_struct *tty, struct serial_state *info,
			 struct ktermios *old_termios)
{
    int baud;

    if (!tty)
        return;

    /* Determine divisor based on baud rate */
    baud = tty_get_baud_rate(tty);

    if (info->curr_baudrate!=baud)
    {
        if (baud > 0)
        {
            urcs_update_br(info->line, urcs_cal_baud_cnt(baud));
        }

        info->curr_baudrate = baud;
    }

    rs_table[info->line].baud_base = baud;
}

static void rs_wait_until_sent(struct tty_struct *tty, int timeout);

#include <asm/uaccess.h>

#define serial_isroot()	(capable(CAP_SYS_ADMIN))

static inline int serial_paranoia_check(struct serial_state *info,
					char *name, const char *routine)
{
#ifdef SERIAL_PARANOIA_CHECK
	static const char *badmagic =
		"Warning: bad magic number for serial struct (%s) in %s\n";
	static const char *badinfo =
		"Warning: null serial_state for (%s) in %s\n";

	if (!info) {
		printk(badinfo, name, routine);
		return 1;
	}
	if (info->magic != SERIAL_MAGIC) {
		printk(badmagic, name, routine);
		return 1;
	}
#endif
	return 0;
}

/*
 * ------------------------------------------------------------
 * rs_stop() and rs_start()
 *
 * This routines are called before setting or resetting tty->stopped.
 * They enable or disable transmitter interrupts, as necessary.
 * ------------------------------------------------------------
 */
static void rs_stop(struct tty_struct *tty)
{
    struct serial_state *info = (struct serial_state *)tty->driver_data;
    unsigned long flags;

    spin_lock_irqsave(&uart_lock, flags);

    rs_disable_tx_interrupts(info->line);

    spin_unlock_irqrestore(&uart_lock, flags);
}

static void rs_start(struct tty_struct *tty)
{
    struct serial_state *info = (struct serial_state *)tty->driver_data;
    unsigned long flags;

    spin_lock_irqsave(&uart_lock, flags);
    if (info->xmit.head != info->xmit.tail && info->xmit.buf)
    {
        rs_enable_tx_interrupts(info->line);
    }
    spin_unlock_irqrestore(&uart_lock, flags);
}

#if defined(CONFIG_KGDB)
void trigger_kgdb_breakpoint(void);
#endif

#if defined(CONFIG_SERIAL_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
static int zero_received[NR_PORTS];
#endif
//#define UART0_RX_TEST
#if defined(UART0_RX_TEST)
unsigned char next_ch;
#endif
static void receive_chars(int idx, struct serial_state *info)
{
    unsigned char ch, flag;
    volatile unsigned int *p = (unsigned int *) (UR_BASE + (0x100 * idx));
    struct  async_icount *icount;
    //int rx_loop = 0;
    int ubr;

    flag = TTY_NORMAL;
    icount = &info->icount;

    do
    {
        if (!(URBR_RDY&(ubr = p[URBR]) )) //if no input, skip
            break;

        ch = ubr >>URBR_DTSHFT;
        icount->rx++;

#if defined(CONFIG_CHEETAH_SND)
        /* XXX: possible future mux switch code here */
#endif

    if(idx==cta_console_index)
    {
#if defined(CONFIG_SERIAL_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)

        /* cheetah UART does not implement line-break detection, we use this workaround to detect line break */
        /* detect consecutive 8 zero received to hint it is a line break */
#define UART_LB_CONSECUTIVE_ZERO_THRESHOLD  8

        if (ch == 0)
        {
            zero_received[idx]++;

            if (zero_received[idx] > UART_LB_CONSECUTIVE_ZERO_THRESHOLD)
                break_pressed[idx] = jiffies;

            continue;
        }
        else
        {
            /* we have to drop first bytes received after LB */
            if ((zero_received[idx]) && (break_pressed[idx]))
            {
                zero_received[idx] = 0;
                continue;
            }
            zero_received[idx] = 0;
        }
        /* end of line-break detection routine */

#if defined(CONFIG_KGDB)
        if ((kgdb_connected) && (ch == 0x3))
        {
            break_pressed[idx] = 0;
            trigger_kgdb_breakpoint();
            continue;
        }
#endif

        if ( break_pressed[idx] )
        {
            if (time_before(jiffies, break_pressed[idx] + HZ*5))
            {
                if (ch == 0)
                    continue;
                handle_sysrq(ch);
                break_pressed[idx] = 0;
                continue;
            }
            break_pressed[idx] = 0;
        }
#endif
    }

#if defined(UART0_RX_TEST)
        if(idx==0)
        {
            //printk("%02x\n", ch);
            if(next_ch!=ch)
                printk("%02x %02x\n", next_ch, ch);
            next_ch = ch;
            next_ch++;
        }
#endif

        tty_insert_flip_char(&rs_table[idx].tport, ch, flag);
    } while (1); //rx_loop++ < 256) ;

#if 0
    // if rx some data, push it
    if (rx_loop)
        tty_flip_buffer_push(&rs_table[idx].tport);
#endif

    tty_schedule_flip(&rs_table[idx].tport);

    return;
}

static int transmit_chars(int idx)
{
    struct serial_state *info = &rs_table[idx];
    volatile int *p = (unsigned int *) (UR_BASE + (0x100 * idx));
    int enable_tx_intr = 0;

    for (;;)
    {
        if ((p[URCS>>2]&URCS_TF))
        {
            enable_tx_intr = 1;
            break;
        }
        else if (info->x_char)
        {
            p[URBR>>2] = ((unsigned int)info->x_char)<<URBR_DTSHFT;
            info->icount.tx++;
            info->x_char = 0;
            enable_tx_intr = 1;
        }
        else
        {
            spin_lock(&uart_lock);
            if (info->xmit.head == info->xmit.tail || info->tport.tty->stopped || info->tport.tty->hw_stopped)
            {
                spin_unlock(&uart_lock);
                break;
            }
            else
            {
                p[URBR>>2] = ((unsigned int)info->xmit.buf[info->xmit.tail++]) <<URBR_DTSHFT;
                info->xmit.tail = info->xmit.tail & (SERIAL_XMIT_SIZE-1);
                info->icount.tx++;
                enable_tx_intr = 1;
                if (info->xmit.head == info->xmit.tail)
                {
                    spin_unlock(&uart_lock);
                    break;
                }
            }
            spin_unlock(&uart_lock);
        }
    }

    if (CIRC_CNT(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE) < WAKEUP_CHARS)
		tty_wakeup(info->tport.tty);

    return enable_tx_intr;
}

static irqreturn_t cta_uart_interrupt(int irq, void *data)
{
    int idx = irq - IRQ_UART0;

    urcs_disable(idx, (URCS_RIE | URCS_TIE));

    tasklet_hi_schedule(&rs_table[idx].uart_irq_tasklet);

    return IRQ_HANDLED;
}

#if defined(CONFIG_PANTHER_PDMA)

static void uart_tx_pdma_task(unsigned long idx)
{
    struct serial_state *info = rs_table[idx].info;
    //struct tty_struct *tty = info->tty;
    int tx_buf_idx;
    u32 len;
    unsigned char *data_buf;
    struct pdma_ch_descr *txdescr;
    //int i;

    if (info->xmit.head == info->xmit.tail || info->tty->stopped || info->tty->hw_stopped)
        return;

    txdescr = (struct pdma_ch_descr *) UNCACHED_ADDR(&pdma_uart_tx_descr[idx][0]);
    tx_buf_idx = rs_table[idx].pdma_tx_buf_idx;

#if 0
    printk("* %d\n", rs_table[idx].pdma_tx_buf_idx);
    for(i=0;i<PDMA_BUF_MAX_NUM;i++)
    {
        printk("  %d %08x\n", i, txdescr[i].desc_config);
    }
#endif

    if(0==(txdescr[tx_buf_idx].desc_config & PDMA_VALID))
    {
        //printk("- %d %08x\n", tx_buf_idx, txdescr[tx_buf_idx].desc_config);

        data_buf = rs_table[idx].pdma_tx_buf[tx_buf_idx];

        spin_lock(&uart_lock);
#if 1
        if(info->xmit.head > info->xmit.tail)
        {
            len = info->xmit.head - info->xmit.tail;
            memcpy(data_buf, &info->xmit.buf[info->xmit.tail], len);
        }
        else
        {
            len = SERIAL_XMIT_SIZE - info->xmit.tail;
            memcpy(data_buf, &info->xmit.buf[info->xmit.tail], len);

            if(info->xmit.head)
                memcpy(&data_buf[len], &info->xmit.buf[0], info->xmit.head);

            len += info->xmit.head;
        }
        info->icount.tx += len;
        info->xmit.tail = info->xmit.head;
#else
        while(info->xmit.head != info->xmit.tail)
        {
            data_buf[len] = info->xmit.buf[info->xmit.tail];
            info->xmit.tail = (info->xmit.tail + 1) & (SERIAL_XMIT_SIZE-1);
            info->icount.tx++;
            len++;
        }
#endif
        spin_unlock(&uart_lock);

        dma_cache_wback_inv((unsigned long) data_buf, len);

        txdescr[tx_buf_idx].desc_config 
            = (txdescr[tx_buf_idx].desc_config & 0x0000FFFF) | PDMA_VALID | (len << PDMA_TOTAL_LEN_SHIFT);

        desc_config_read = txdescr[tx_buf_idx].desc_config;

        pdma_kick_channel(PDMA_UART_TX_CHANNEL(idx));
    
        tx_buf_idx = (tx_buf_idx+1) % PDMA_BUF_MAX_NUM;
        rs_table[idx].pdma_tx_buf_idx = tx_buf_idx;
    
        if (CIRC_CNT(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE) < WAKEUP_CHARS)
            tty_wakeup(info->tport.tty);
    }
#if 0
    else
    {
        printk("XXXX\n");
    }
#endif

    return;
}

#endif

static void cta_uart_deliver(unsigned long idx)
{
    volatile int *p = (unsigned int *) (UR_BASE + (0x100 * idx));
    unsigned int status;
    struct serial_state *info = &rs_table[idx];
    int tx_intr_enable = 0;

    /* XXX : Clear any interrupts we might be about to handle */

    if (!info->tport.tty)
    {
        // printk("cheetah_uart_interrupt: ignored\n");
        return ;
    }

#if defined(CONFIG_PANTHER_PDMA)
    if (pdma_enabled(idx))
        return uart_tx_pdma_task(idx);
#endif

    status = p[URCS>>2];

    /* TX holding register empty - transmit a byte */
    if (status & URCS_TE)
    {
        //URREG(URCS) = cta_uart_control|URCS_TE;
        if (transmit_chars(idx))
            tx_intr_enable = 1;
        info->last_active = jiffies;
    }
    else
    {
        tx_intr_enable = 1;
    }

    /* Byte received (which bit for RX timeout? NO) */
    if (status & URCS_RF)
    {
        /* RX Frame Error */
        if (status & URCS_FE)
            info->icount.frame++;

        /* RX Parity Error */
        if (status & URCS_PER)
            info->icount.parity++;

        receive_chars(idx, info);
        info->last_active = jiffies;
    }

    if (tx_intr_enable)
        urcs_enable(idx, (URCS_RIE | URCS_TIE));
    else
        urcs_enable(idx, URCS_RIE);

    return;
}

/*
 * -------------------------------------------------------------------
 * Here ends the serial interrupt routines.
 * -------------------------------------------------------------------
 */

/*
 * ---------------------------------------------------------------
 * Low level utility subroutines for the serial driver:  routines to
 * figure out the appropriate timeout for an interrupt chain, routines
 * to initialize and startup a serial port, and routines to shutdown a
 * serial port.  Useful stuff like that.
 * ---------------------------------------------------------------
 */
static int startup(struct tty_struct *tty, struct serial_state *info)
{
	struct tty_port *port = &info->tport;
    unsigned long flags;
    int retval=0;
    unsigned long page;

#if defined(CONFIG_PANTHER_PDMA)
    unsigned long device_ids[2];
#endif

    if (port->flags & ASYNC_INITIALIZED)
    {
        return retval;
    }

    spin_lock_irqsave(&uart_lock, flags);
    if ( !info->xmit.buf)
    {
        page = get_zeroed_page(GFP_KERNEL);
        if (!page)
        {
            retval= -ENOMEM;
            goto errout;
        }
        info->xmit.buf = (unsigned char *) page;
    }

#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("starting up ttys%d ...", info->line);
#endif

    /* Clear anything in the input buffer */
    if (serial_isroot())
    {
        if (tty)
            set_bit(TTY_IO_ERROR, &tty->flags);
        retval = 0;
    }

#if defined(CONFIG_PANTHER_PDMA)
    if (pdma_enabled(info->line))
    {
        device_ids[0] = 0;
        device_ids[1] = 0;
        if(0==info->line)
            device_ids[0] = DEVICE_ID_UART0;
        else if(1==info->line)
            device_ids[0] = DEVICE_ID_UART1;
        else if(2==info->line)
            device_ids[0] = DEVICE_ID_UART2;

        pmu_reset_devices_no_spinlock(device_ids);

        UARTREG(info->line, URCS2) |= URCS2_DMA_EN;
        
        UARTREG(info->line, URCS2) &= ~(0xf << 8);
        UARTREG(info->line, URCS2) |= (12 << 8);
        UARTREG(info->line, URCS2) &= ~(0xf << 4);
        UARTREG(info->line, URCS2) |= (8 << 4);

        pdma_callback_register(PDMA_UART_TX_CHANNEL(info->line), uart_tx_pdma_done);
        pdma_callback_register(PDMA_UART_RX_CHANNEL(info->line), uart_rx_pdma_done);

        uart_rx_pdma_start(info->line);
        uart_tx_pdma_init(info->line);
    }
#endif

    rs_enable_tx_interrupts(info->line);
    rs_enable_rx_interrupts(info->line);

    if (tty)
        clear_bit(TTY_IO_ERROR, &tty->flags);
    info->xmit.head = info->xmit.tail = 0;

    /*
     * and set the speed of the serial port
     */
    change_speed(tty, info, NULL);

    port->flags |= ASYNC_INITIALIZED;
    spin_unlock_irqrestore(&uart_lock, flags);
    return 0;

errout:
    spin_unlock_irqrestore(&uart_lock, flags);
    return retval;
}

/*
 * This routine will shutdown a serial port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 */
static void shutdown(struct tty_struct *tty, struct serial_state *info)
{
    unsigned long   flags;
	struct serial_state *state;

    if (!(info->flags & ASYNC_INITIALIZED))
        return;

	state = info;

#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("Shutting down serial port %d ....\n", info->line);
#endif

    spin_lock_irqsave(&uart_lock, flags); /* Disable interrupts */

    /*
     * clear delta_msr_wait queue to avoid mem leaks: we may free the irq
     * here so the queue might never be waken up
     */
	wake_up_interruptible(&info->tport.delta_msr_wait);

    rs_disable_tx_interrupts(info->line);
    rs_disable_rx_interrupts(info->line);

#if defined(CONFIG_PANTHER_PDMA)
    if (pdma_enabled(info->line))
    {
        UARTREG(info->line, URCS2) &= ~(URCS2_DMA_EN);
        uart_pdma_stop(info->line);
    }
#endif

    if (info->xmit.buf)
    {
        free_page((unsigned long) info->xmit.buf);
        info->xmit.buf = NULL;
    }

    if (info->tport.tty)
        set_bit(TTY_IO_ERROR, &info->tport.tty->flags);

    info->flags &= ~ASYNC_INITIALIZED;
    spin_unlock_irqrestore(&uart_lock, flags);
}


static int rs_put_char(struct tty_struct *tty, unsigned char ch)
{
    struct serial_state *info;
    unsigned long flags;

    if (!tty)
        return 0;

    info = tty->driver_data;

    if (!info->xmit.buf)
        return 0;

    spin_lock_irqsave(&uart_lock, flags);
    if (CIRC_SPACE(info->xmit.head,
                   info->xmit.tail,
                   SERIAL_XMIT_SIZE) == 0)
    {
        spin_unlock_irqrestore(&uart_lock, flags);
        return 0;
    }

    info->xmit.buf[info->xmit.head++] = ch;
    info->xmit.head &= SERIAL_XMIT_SIZE-1;
    spin_unlock_irqrestore(&uart_lock, flags);
    return 1;
}

static void rs_flush_chars(struct tty_struct *tty)
{
    struct serial_state *info = (struct serial_state *)tty->driver_data;
    unsigned long flags;

    spin_lock_irqsave(&uart_lock, flags);

    if (info->xmit.head == info->xmit.tail
        || tty->stopped
        || tty->hw_stopped
        || !info->xmit.buf)
    {
        spin_unlock_irqrestore(&uart_lock, flags);
        return;
    }

    rs_enable_tx_interrupts(info->line);

    spin_unlock_irqrestore(&uart_lock, flags);
}

static int rs_write(struct tty_struct * tty, const unsigned char *buf, int count)
{
    int c, ret = 0;
    struct serial_state *info;
    unsigned long flags;

    if (!tty)
        return 0;

    info = tty->driver_data;

    if (!info->xmit.buf)
        return 0;

    spin_lock_irqsave(&uart_lock, flags);
    while (1)
    {
        c = CIRC_SPACE_TO_END(info->xmit.head,
                              info->xmit.tail,
                              SERIAL_XMIT_SIZE);
        if (count < c)
            c = count;
        if (c <= 0)
        {
            break;
        }
        memcpy(info->xmit.buf + info->xmit.head, buf, c);
        info->xmit.head = ((info->xmit.head + c) &
                           (SERIAL_XMIT_SIZE-1));
        buf += c;
        count -= c;
        ret += c;
    }

    if (info->xmit.head != info->xmit.tail
        && !tty->stopped
        && !tty->hw_stopped
       )
    {
        rs_enable_tx_interrupts(info->line);
    }

    spin_unlock_irqrestore(&uart_lock, flags);

    return ret;
}

static int rs_write_room(struct tty_struct *tty)
{
    struct serial_state *info = (struct serial_state *)tty->driver_data;

    return CIRC_SPACE(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE);
}

static int rs_chars_in_buffer(struct tty_struct *tty)
{
    struct serial_state *info = (struct serial_state *)tty->driver_data;

    return CIRC_CNT(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE);
}

static void rs_flush_buffer(struct tty_struct *tty)
{
    struct serial_state *info = (struct serial_state *)tty->driver_data;
    unsigned long flags;

    spin_lock_irqsave(&uart_lock, flags);
    info->xmit.head = info->xmit.tail = 0;
    spin_unlock_irqrestore(&uart_lock, flags);
    tty_wakeup(tty);
}

/*
 * This function is used to send a high-priority XON/XOFF character to
 * the device
 */
static void rs_send_xchar(struct tty_struct *tty, char ch)
{
    struct serial_state *info = (struct serial_state *)tty->driver_data;
    unsigned long flags;

    spin_lock_irqsave(&uart_lock, flags);

    info->x_char = ch;
    if (ch)
    {
        rs_enable_tx_interrupts(info->line);
    }

    spin_unlock_irqrestore(&uart_lock, flags);
}

#if 0
/*
 * ------------------------------------------------------------
 * rs_throttle()
 * 
 * This routine is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 * ------------------------------------------------------------
 */
static void rs_throttle(struct tty_struct * tty)
{
    struct serial_state *info = (struct serial_state *)tty->driver_data;
    unsigned long flags;
#ifdef CHEETAH_UART_DEBUG_THROTTLE
    char    buf[64];

    printk("throttle %s: %d....\n", tty_name(tty, buf),
           tty->ldisc.chars_in_buffer(tty));
#endif

    if (I_IXOFF(tty))
        rs_send_xchar(tty, STOP_CHAR(tty));
}

static void rs_unthrottle(struct tty_struct * tty)
{
    struct serial_state *info = (struct serial_state *)tty->driver_data;
    unsigned long flags;
#ifdef CHEETAH_UART_DEBUG_THROTTLE
    char    buf[64];

    printk("unthrottle %s: %d....\n", tty_name(tty, buf),
           tty->ldisc.chars_in_buffer(tty));
#endif

    if (I_IXOFF(tty))
    {
        if (info->x_char)
            info->x_char = 0;
        else
            rs_send_xchar(tty, START_CHAR(tty));
    }
}
#endif

/*
 * ------------------------------------------------------------
 * rs_ioctl() and friends
 * ------------------------------------------------------------
 */

static int get_serial_info(struct tty_struct *tty, struct serial_state *state,
                           struct serial_struct __user * retinfo)
{
    struct serial_struct tmp;

    if (!retinfo)
        return -EFAULT;
    memset(&tmp, 0, sizeof(tmp));
	tty_lock(tty);
    tmp.line = tty->index;
    tmp.port = state->port;
//    tmp.irq = state->irq;
    tmp.flags = state->tport.flags;
    tmp.xmit_fifo_size = state->xmit_fifo_size;
    tmp.baud_base = state->baud_base;
	tmp.close_delay = state->tport.close_delay;
	tmp.closing_wait = state->tport.closing_wait;
	tmp.custom_divisor = state->custom_divisor;
	tty_unlock(tty);
    if (copy_to_user(retinfo,&tmp,sizeof(*retinfo)))
        return -EFAULT;
    return 0;
}

static int set_serial_info(struct tty_struct *tty, struct serial_state *state,
			   struct serial_struct __user * new_info)
{
	struct tty_port *port = &state->tport;
	struct serial_struct new_serial;
	bool change_spd;
	int 			retval = 0;

    if (copy_from_user(&new_serial,new_info,sizeof(new_serial)))
        return -EFAULT;

	tty_lock(tty);
    change_spd = ((new_serial.flags ^ port->flags) & ASYNC_SPD_MASK) ||
        new_serial.custom_divisor != state->custom_divisor;
    if (new_serial.irq || new_serial.port != state->port ||
            new_serial.xmit_fifo_size != state->xmit_fifo_size) {
        tty_unlock(tty);
        return -EINVAL;
    }

	if (!serial_isroot()) {
		if ((new_serial.baud_base != state->baud_base) ||
		    (new_serial.close_delay != port->close_delay) ||
		    (new_serial.xmit_fifo_size != state->xmit_fifo_size) ||
		    ((new_serial.flags & ~ASYNC_USR_MASK) !=
		     (port->flags & ~ASYNC_USR_MASK))) {
			tty_unlock(tty);
			return -EPERM;
		}
		port->flags = ((port->flags & ~ASYNC_USR_MASK) |
			       (new_serial.flags & ASYNC_USR_MASK));
		state->custom_divisor = new_serial.custom_divisor;
		goto check_and_exit;
	}

	if (new_serial.baud_base < 9600) {
		tty_unlock(tty);
		return -EINVAL;
	}

	/*
	 * OK, past this point, all the error checking has been done.
	 * At this point, we start making changes.....
	 */

	state->baud_base = new_serial.baud_base;
	port->flags = ((port->flags & ~ASYNC_FLAGS) |
			(new_serial.flags & ASYNC_FLAGS));
	state->custom_divisor = new_serial.custom_divisor;
	port->close_delay = new_serial.close_delay * HZ/100;
	port->closing_wait = new_serial.closing_wait * HZ/100;
	port->low_latency = (port->flags & ASYNC_LOW_LATENCY) ? 1 : 0;

check_and_exit:
	if (port->flags & ASYNC_INITIALIZED) {
		if (change_spd) {
			if ((port->flags & ASYNC_SPD_MASK) == ASYNC_SPD_HI)
				tty->alt_speed = 57600;
			if ((port->flags & ASYNC_SPD_MASK) == ASYNC_SPD_VHI)
				tty->alt_speed = 115200;
			if ((port->flags & ASYNC_SPD_MASK) == ASYNC_SPD_SHI)
				tty->alt_speed = 230400;
			if ((port->flags & ASYNC_SPD_MASK) == ASYNC_SPD_WARP)
				tty->alt_speed = 460800;
			change_speed(tty, state, NULL);
		}
	} else
		retval = startup(tty, state);
	tty_unlock(tty);
    return retval;
}

#if 0
/*
 * get_lsr_info - get line status register info
 *
 * Purpose: Let user call ioctl() to get info when the UART physically
 * 	    is emptied.  On bus types like RS485, the transmitter must
 * 	    release the bus after transmitting. This must be done when
 * 	    the transmit shift register is empty, not be done when the
 * 	    transmit holding register is empty.  This functionality
 * 	    allows an RS485 driver to be written in user space. 
 */
static int get_lsr_info(struct serial_state * info, unsigned int __user *value)
{
    unsigned char status;
    unsigned int result;
    unsigned long flags;

#if 0
    spin_lock_irqsave(&uart_lock, flags);
    status = custom.serdatr;
    mb();
    spin_unlock_irqrestore(&uart_lock, flags);
    result = ((status & SDR_TSRE) ? TIOCSER_TEMT : 0);
#endif
    if (copy_to_user(value, &result, sizeof(int)))
        return -EFAULT;
    return 0;
}
#endif

/*
 * rs_break() --- routine which turns the break handling on or off
 */
static int rs_break(struct tty_struct *tty, int break_state)
{
    unsigned long flags;

    spin_lock_irqsave(&uart_lock, flags);
#if 0
    if (break_state == -1)
        custom.adkcon = AC_SETCLR | AC_UARTBRK;
    else
        custom.adkcon = AC_UARTBRK;
#endif
    mb();
    spin_unlock_irqrestore(&uart_lock, flags);
    return 0;
}


static int rs_ioctl(struct tty_struct *tty,
                    unsigned int cmd, unsigned long arg)
{
    struct serial_state * info = (struct serial_state *)tty->driver_data;
	struct async_icount cprev, cnow;	/* kernel counter temps */
    struct serial_icounter_struct icount;
    void __user *argp = (void __user *)arg;
    unsigned long flags;
	DEFINE_WAIT(wait);
	int ret;

	if (serial_paranoia_check(info, tty->name, "rs_ioctl"))
		return -ENODEV;

	if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
	    (cmd != TIOCSERCONFIG) && (cmd != TIOCSERGSTRUCT) &&
	    (cmd != TIOCMIWAIT) && (cmd != TIOCGICOUNT)) {
		if (tty->flags & (1 << TTY_IO_ERROR))
		    return -EIO;
	}

    switch (cmd)
    {
		case TIOCGSERIAL:
			return get_serial_info(tty, info, argp);
		case TIOCSSERIAL:
			return set_serial_info(tty, info, argp);
		case TIOCSERCONFIG:
			return 0;
#if 0
        case TIOCSERGETLSR: /* Get line status register */
            return get_lsr_info(info, argp);
#endif
        case TIOCSERGSTRUCT:
            if (copy_to_user(argp,
                             info, sizeof(struct serial_state)))
                return -EFAULT;
            return 0;

        /*
         * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
         * - mask passed in arg for lines of interest
         *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
         * Caller should use TIOCGICOUNT to see which one it was
         */
        case TIOCMIWAIT:
			local_irq_save(flags);
			/* note the counters on entry */
			cprev = info->icount;
			local_irq_restore(flags);
			while (1) {
				prepare_to_wait(&info->tport.delta_msr_wait,
						&wait, TASK_INTERRUPTIBLE);
				local_irq_save(flags);
				cnow = info->icount; /* atomic copy */
				local_irq_restore(flags);
				if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr && 
				    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts) {
					ret = -EIO; /* no change => error */
					break;
				}
				if ( ((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
				     ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
				     ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
				     ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) {
					ret = 0;
					break;
				}
				schedule();
				/* see if a signal did it */
				if (signal_pending(current)) {
					ret = -ERESTARTSYS;
					break;
				}
				cprev = cnow;
			}
			finish_wait(&info->tport.delta_msr_wait, &wait);
			return ret;
    case TIOCGICOUNT:
        spin_lock_irqsave(&uart_lock, flags);
        cnow = info->icount;
        spin_unlock_irqrestore(&uart_lock, flags);
        icount.cts = cnow.cts;
        icount.dsr = 0;
        icount.rng = 0;
        icount.dcd = 0;
        icount.rx = cnow.rx;
        icount.tx = cnow.tx;
        icount.frame = cnow.frame;
        icount.overrun = cnow.overrun;
        icount.parity = cnow.parity;
        icount.brk = cnow.brk;
        icount.buf_overrun = cnow.buf_overrun;

        if (copy_to_user(argp, &icount, sizeof(icount)))
            return -EFAULT;
        return 0;
    case TIOCSERGWILD:
    case TIOCSERSWILD:
        /* "setserial -W" is called in Debian boot */
        printk ("TIOCSER?WILD ioctl obsolete, ignored.\n");
        return 0;

    default:
        return -ENOIOCTLCMD;
    }
    return 0;
}

static void rs_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
    struct serial_state *info = (struct serial_state *)tty->driver_data;
    unsigned int cflag = tty->termios.c_cflag;
    unsigned long flags;
    unsigned int cta_uart_control;
    int uart_idx;

    if (!info || (info->line>=NR_PORTS))
        return;

    /* check para whether changed */
    if (old_termios && (cflag == old_termios->c_cflag))
    {
        //printk("paras don't changed\n");
        return;
    }

    spin_lock_irqsave(&uart_lock, flags);

    uart_idx = info->line;
    cta_uart_control = rs_table[uart_idx].uart_control;

    /* change stop bit setting */
    if (cflag & CSTOPB)
        cta_uart_control|=URCS_SP2;
    else
        cta_uart_control&=~URCS_SP2;

    /* change parity enable/disable setting */
    if (cflag & PARENB)
        cta_uart_control|=URCS_PE;
    else
        cta_uart_control&=~URCS_PE;

    /* change parity odd/even setting */
    if (cflag & PARODD)
        cta_uart_control&=~URCS_EVEN;
    else
        cta_uart_control|=URCS_EVEN;

    /* change baud rate setting */
    if (cflag & CBAUD)
    {
        cta_uart_control&=((unsigned int)(1<<URCS_BRSHFT)-1);
        cta_uart_control|=(urcs_cal_baud_cnt(tty_termios_baud_rate(&tty->termios))<<URCS_BRSHFT);

        rs_table[uart_idx].baud_base = tty_termios_baud_rate(&tty->termios);
    }

    cta_uart_control |= URCS_RXEN;
    rs_table[uart_idx].uart_control = cta_uart_control;
    UARTREG(uart_idx, URCS) = cta_uart_control;

    if ((old_termios->c_cflag & CRTSCTS) &&
        !(tty->termios.c_cflag & CRTSCTS))
    {
        UARTREG(uart_idx, URCS2) &= (~(URCS2_RX_FLOW_CTRL_EN | URCS2_TX_FLOW_CTRL_EN));
    }
    /* turn on */
    else if (!(old_termios->c_cflag & CRTSCTS) &&
             (tty->termios.c_cflag & CRTSCTS))
    {
        UARTREG(uart_idx, URCS2) |= (URCS2_RX_FLOW_CTRL_EN | URCS2_TX_FLOW_CTRL_EN);
    }

    spin_unlock_irqrestore(&uart_lock, flags);

#if 0
    /*
     * No need to wake up processes in open wait, since they
     * sample the CLOCAL flag once, and don't recheck it.
     * XXX  It's not clear whether the current behavior is correct
     * or not.  Hence, this may change.....
     */
    if (!(old_termios->c_cflag & CLOCAL) &&
        (tty->termios.c_cflag & CLOCAL))
        wake_up_interruptible(&info->open_wait);
#endif
}

/*
 * ------------------------------------------------------------
 * rs_close()
 * 
 * This routine is called when the serial port gets closed.  First, we
 * wait for the last remaining data to be sent.  Then, we unlink its
 * async structure from the interrupt chain if necessary, and we free
 * that IRQ if nothing is left in the chain.
 * ------------------------------------------------------------
 */
static void rs_close(struct tty_struct *tty, struct file * filp)
{
	struct serial_state *state = tty->driver_data;
	struct tty_port *port = &state->tport;

	if (serial_paranoia_check(state, tty->name, "rs_close"))
		return;

	if (tty_port_close_start(port, tty, filp) == 0)
		return;

	/*
	 * At this point we stop accepting input.  To do this, we
	 * disable the receive line status interrupts, and tell the
	 * interrupt driver to stop checking the data ready bit in the
	 * line status register.
	 */
	state->read_status_mask &= ~UART_LSR_DR;
	if (port->flags & ASYNC_INITIALIZED) {
        #if defined(CONFIG_TODO)
	        /* disable receive interrupts */
	        custom.intena = IF_RBF;
		mb();
		/* clear any pending receive interrupt */
		custom.intreq = IF_RBF;
		mb();
        #endif

		/*
		 * Before we drop DTR, make sure the UART transmitter
		 * has completely drained; this is especially
		 * important if there is a transmit FIFO!
		 */
		rs_wait_until_sent(tty, state->timeout);
	}
    uart_disable_rx(state->line);
	shutdown(tty, state);
	rs_flush_buffer(tty);
		
	tty_ldisc_flush(tty);
	port->tty = NULL;

	tty_port_close_end(port, tty);
}

/*
 * rs_wait_until_sent() --- wait until the transmitter is empty
 */
static void rs_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct serial_state *info = tty->driver_data;
	unsigned long orig_jiffies, char_time;
	//int lsr;

	if (serial_paranoia_check(info, tty->name, "rs_wait_until_sent"))
		return;

	if (info->xmit_fifo_size == 0)
		return; /* Just in case.... */

	orig_jiffies = jiffies;

	/*
	 * Set the check interval to be 1/5 of the estimated time to
	 * send a single character, and make it at least 1.  The check
	 * interval should also be less than the timeout.
	 * 
	 * Note: we have to use pretty tight timings here to satisfy
	 * the NIST-PCTS.
	 */
	char_time = (info->timeout - HZ/50) / info->xmit_fifo_size;
	char_time = char_time / 5;
	if (char_time == 0)
		char_time = 1;
	if (timeout)
	  char_time = min_t(unsigned long, char_time, timeout);
	/*
	 * If the transmitter hasn't cleared in twice the approximate
	 * amount of time to send the entire FIFO, it probably won't
	 * ever clear.  This assumes the UART isn't doing flow
	 * control, which is currently the case.  Hence, if it ever
	 * takes longer than info->timeout, this is probably due to a
	 * UART bug of some kind.  So, we clamp the timeout parameter at
	 * 2*info->timeout.
	 */
	if (!timeout || timeout > 2*info->timeout)
		timeout = 2*info->timeout;
#ifdef SERIAL_DEBUG_RS_WAIT_UNTIL_SENT
	printk("In rs_wait_until_sent(%d) check=%lu...", timeout, char_time);
	printk("jiff=%lu...", jiffies);
#endif
#if defined(CONFIG_TODO)
	while(!((lsr = custom.serdatr) & SDR_TSRE)) {
#ifdef SERIAL_DEBUG_RS_WAIT_UNTIL_SENT
		printk("serdatr = %d (jiff=%lu)...", lsr, jiffies);
#endif
		msleep_interruptible(jiffies_to_msecs(char_time));
		if (signal_pending(current))
			break;
		if (timeout && time_after(jiffies, orig_jiffies + timeout))
			break;
	}
#endif
	__set_current_state(TASK_RUNNING);

#ifdef SERIAL_DEBUG_RS_WAIT_UNTIL_SENT
	printk("lsr = %d (jiff=%lu)...done\n", lsr, jiffies);
#endif
}

/*
 * rs_hangup() --- called by tty_hangup() when a hangup is signaled.
 */
static void rs_hangup(struct tty_struct *tty)
{
	struct serial_state *info = tty->driver_data;

	if (serial_paranoia_check(info, tty->name, "rs_hangup"))
		return;

	rs_flush_buffer(tty);
	shutdown(tty, info);
	info->tport.count = 0;
	info->tport.flags &= ~ASYNC_NORMAL_ACTIVE;
	info->tport.tty = NULL;
	wake_up_interruptible(&info->tport.open_wait);
}

/*
 * This routine is called whenever a serial port is opened.  It
 * enables interrupts for a serial port, linking in its async structure into
 * the IRQ chain.   It also performs the serial-specific
 * initialization for the tty structure.
 */
static int rs_open(struct tty_struct *tty, struct file * filp)
{
    struct serial_state *info;
    struct tty_port *port;
    int line, retval;

    line = tty->index;
    if ((line < 0) || (line >= NR_PORTS))
        return -ENODEV;

    info = &rs_table[line];
    info->line = line;
    port = &info->tport;

	port->count++;
	port->tty = tty;
	tty->driver_data = info;
	tty->port = port;
	if (serial_paranoia_check(info, tty->name, "rs_open"))
		return -ENODEV;

    port->low_latency = (port->flags & ASYNC_LOW_LATENCY) ? 1 : 0;

    retval = startup(tty, info);
    if (retval)
        return retval;

    retval = tty_port_block_til_ready(port, tty, filp);
    if (retval)
    {
#ifdef CHEETAH_UART_DEBUG_OPEN
        printk("rs_open returning after block_til_ready with %d\n",
               retval);
#endif
        return retval;
    }

    uart_enable_rx(info->line);

#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("rs_open %s successful...", tty->name);
#endif
    return 0;
}

#ifdef CONFIG_CHEETAH_UART_STAT
/*
 * /proc fs routines....
 */

static inline void line_info(struct seq_file *m, struct serial_state *state)
{
    volatile unsigned int *urcs2;
    char stat_buf[30];

    seq_printf(m, "%d: ",state->line);

    stat_buf[0] = 0;
    stat_buf[1] = 0;
    urcs2 = (unsigned int *) (UR_BASE + URCS2 + (0x100 * state->line));
    if (!(*urcs2 & URCS2_TX_STOP))
        strcat(stat_buf, "|RTS");
    if (!(*urcs2 & URCS2_RX_STOP))
        strcat(stat_buf, "|CTS");

    seq_printf(m, " baud:%d", state->baud_base);

    seq_printf(m, " tx:%d rx:%d", state->icount.tx, state->icount.rx);

    if (state->icount.frame)
        seq_printf(m, " fe:%d", state->icount.frame);

    if (state->icount.parity)
        seq_printf(m, " pe:%d", state->icount.parity);

    if (state->icount.brk)
        seq_printf(m, " brk:%d", state->icount.brk);

    if (state->icount.overrun)
        seq_printf(m, " oe:%d", state->icount.overrun);

    seq_printf(m, " %s\n", stat_buf+1);
}

static int rs_proc_show(struct seq_file *m, void *v)
{
    int i;

    seq_printf(m, "mt_uart:1.0 driver revision:1.0\n");

    for (i = 0; i < NR_PORTS; i++)
        line_info(m, &rs_table[i]);

    return 0;
}

static int rs_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, rs_proc_show, NULL);
}

static const struct file_operations rs_proc_fops = {
    .owner      = THIS_MODULE,
    .open       = rs_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};
#endif

#if defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
int cheetah_uart_poll_init(struct tty_driver *driver, int line, char *options)
{
    return 0;
}

int cheetah_uart_poll_get_char(struct tty_driver *driver, int line)
{
    volatile int *p = (unsigned int *) (UR_BASE + (0x100 * cta_console_index));
    int ch;

    while (1)
    {
        if (URBR_RDY & (ch = p[URBR]) ) // rx flag on, break
            break;
    }
    return ch>>URBR_DTSHFT;
}

void cheetah_uart_poll_put_char(struct tty_driver *driver, int line, char ch)
{
    int i;
    volatile int *p = (unsigned int *) (UR_BASE + (0x100 * cta_console_index));

    /* Wait for UARTA_TX register to empty */
    i = 1000000;
    while ((p[URCS>>2] & URCS_TF) && i--);
    /* Send the character */
    p[URBR>>2] = (int)ch<<URBR_DTSHFT;
}
#endif


#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)

static int uart_cmd(int argc, char *argv[])
{
    unsigned long baudrate;
    int port;

    if (argc < 1)
        goto help;

    port = argv[-1][4] - '0';

    if((port>=NR_PORTS) || (port<0))
        goto err1;

    if(sscanf(argv[0], "%d", (unsigned *) &baudrate)!=1)
        goto err1;

    if(baudrate==0)
        goto err1;

    idb_print("Change UART%d baudrate to %d bps\n", port, baudrate);

    urcs_update_br(port, urcs_cal_baud_cnt(baudrate));
    rs_table[port].baud_base = baudrate;

    return 0;

err1:
    return -1; //E_PARM;

help:
    return -3; //E_HELP;
}

struct idb_command idb_uart_cmd = 
{
    .cmdline = "uart",
    .help_msg = "uart<port> <baudrate>   Re-configure UART baudrate",
    .func = uart_cmd,
};

#endif

static const struct tty_operations serial_ops = {
    .open = rs_open,
    .close = rs_close,
    .write = rs_write,
    .put_char = rs_put_char,
    .flush_chars = rs_flush_chars,
    .write_room = rs_write_room,
    .chars_in_buffer = rs_chars_in_buffer,
    .flush_buffer = rs_flush_buffer,
    .ioctl = rs_ioctl,
    //.throttle = rs_throttle,
    //.unthrottle = rs_unthrottle,
    .set_termios = rs_set_termios,
    .stop = rs_stop,
    .start = rs_start,
    .hangup = rs_hangup,
    .break_ctl = rs_break,
    .send_xchar = rs_send_xchar,
    .wait_until_sent = rs_wait_until_sent,
#ifdef CONFIG_CHEETAH_UART_STAT
    .proc_fops = &rs_proc_fops,
#endif
#ifdef CONFIG_CONSOLE_POLL
    .poll_init  = cheetah_uart_poll_init,
    .poll_get_char  = cheetah_uart_poll_get_char,
    .poll_put_char  = cheetah_uart_poll_put_char,
#endif
};

static const struct tty_port_operations panther_uart_port_ops = {
};

static struct class *rs_class;
static int __init cheetah_rs_init(void)
{
    unsigned long flags;
    struct serial_state * state;
    char tty_name[] = "ttyS0";
    int i;

    rs_class = class_create(THIS_MODULE, "rs_tty");
    if (!rs_class)
        panic("create class rs_tty class failed");

    serial_driver = alloc_tty_driver(NR_PORTS);
    if (!serial_driver)
        return -ENOMEM;

    for (i=0;i<NR_PORTS;i++)
    {
        rs_table[i].uart_control = UARTREG(i,URCS)&URCS_CTRL_MASK;
        if (i==0)
            rs_table[i].irq = IRQ_UART0;
        else if (i==1)
            rs_table[i].irq = IRQ_UART1;
        else
            rs_table[i].irq = IRQ_UART2;
    }

    /* Initialize the tty_driver structure */

    serial_driver->owner = THIS_MODULE;
    serial_driver->driver_name = "mt_uart";
    serial_driver->name = "ttyS";
    serial_driver->major = TTY_MAJOR;
    serial_driver->minor_start = 64;
    serial_driver->type = TTY_DRIVER_TYPE_SERIAL;
    serial_driver->subtype = SERIAL_TYPE_NORMAL;
    serial_driver->init_termios = tty_std_termios;
    serial_driver->init_termios.c_cflag =
    B115200 | CS8 | CREAD | HUPCL | CLOCAL;
    serial_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
    tty_set_operations(serial_driver, &serial_ops);

    for (i=0;i<NR_PORTS;i++)
    {
        //tty_register_device(serial_driver, i, NULL);

        state = &rs_table[i];
        state->line = i;
        state->custom_divisor = 0;
        state->close_delay = 5*HZ/10;
        state->closing_wait = 30*HZ;
        state->icount.cts = state->icount.dsr = 
                            state->icount.rng = state->icount.dcd = 0;
        state->icount.rx = state->icount.tx = 0;
        state->icount.frame = state->icount.parity = 0;
        state->icount.overrun = state->icount.brk = 0;
        tty_port_init(&state->tport);
        state->tport.ops = &panther_uart_port_ops;
        tty_port_link_device(&state->tport, serial_driver, i);

        printk(KERN_INFO "ttyS%d is enabled\n",
               state->line);

        state->xmit_fifo_size = 16;

        spin_lock_irqsave(&uart_lock, flags);

#if defined(CONFIG_PANTHER_PDMA)
        if (cta_console_index==i)
        {
            memcpy(state->devname, "uart0_console", 13);
            state->devname[4] = '0' + i;
            state->devname[14] = 0;
        }
        else if (pdma_enabled(i))
        {
            memcpy(state->devname, "uart0_pdma", 10);
            state->devname[4] = '0' + i;
            state->devname[11] = 0;
        }
        else
        {
            memcpy(state->devname, "uart", 4);
            state->devname[4] = '0' + i;
            state->devname[5] = 0;
        }
#else
        memcpy(state->devname, "uart", 4);
        state->devname[4] = '0' + i;
        state->devname[5] = 0;
#endif

        if (0 > request_irq(state->irq, cta_uart_interrupt, IRQF_DISABLED, state->devname, state))
            panic("Couldn't request IRQ for UART device\n");

        tasklet_init(&state->uart_irq_tasklet, cta_uart_deliver, (unsigned long) i);
#if defined(CONFIG_PANTHER_PDMA)
        if (pdma_enabled(i))
            tasklet_init(&state->pdma_rx_tasklet, uart_rx_pdma_task, (unsigned long) i);
#endif
        spin_unlock_irqrestore(&uart_lock, flags);

        tty_name[4] = '0' + i;
        state->dev = device_create(rs_class, NULL, MKDEV(TTY_MAJOR, (64 + i)), NULL, (const char *)tty_name);
    }

    if (tty_register_driver(serial_driver))
        panic("Couldn't register serial driver\n");

#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
    register_idb_command(&idb_uart_cmd);
#endif

#if defined(CONFIG_PANTHER_PDMA)
    pdma_init();
    init_pdma_uart_data();
#endif

    return 0;
}

static __exit void cheetah_rs_exit(void) 
{
    int error;

    //printk("Unloading %s: version %s\n", serial_name, serial_version);

    if ((error = tty_unregister_driver(serial_driver)))
        printk("SERIAL: failed to unregister serial driver (%d)\n",
               error);
    put_tty_driver(serial_driver);

#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
    unregister_idb_command(&idb_uart_cmd);
#endif

}

module_init(cheetah_rs_init)
module_exit(cheetah_rs_exit)


#ifdef CONFIG_SERIAL_CONSOLE


void cheetah_serial_outc(unsigned char c)
{
    int i;
    volatile unsigned int *p = (unsigned int *) (UR_BASE + (0x100 * cta_console_index));

#if defined(CONFIG_TODO)
    /* Disable UARTA_TX interrupts */
    URCS_DISABLE(URCS_TIE);
#endif

    /* Wait for UARTA_TX register to empty */
    i = 10000;
    while ((p[URCS>>2] & URCS_TF) && i--);

    /* Send the character */
    p[URBR>>2] = (int)c<<URBR_DTSHFT;

#if defined(CONFIG_TODO)
    /* Enable UARTA_TX interrupts */
    URCS_ENABLE(URCS_TIE);
#endif
}

static __init int serial_console_setup(struct console *co, char *options)
{
    unsigned int baud = CONFIG_CHEETAH_UART_BAUDRATE;
    //int bits = 8;
    int parity = 'n';
    char *s;
    unsigned int brsr = 0;
    unsigned int cta_uart_control;
    unsigned long flags;

    if (options)
    {
        baud = simple_strtoul(options, NULL, 10);
        s = options;
        while (*s >= '0' && *s <= '9')
            s++;
        if (*s) parity = *s++;
        //if (*s) bits   = *s++ - '0';
    }

    if (baud >0)
        brsr = urcs_cal_baud_cnt(baud);
#if 0
    switch (bits)
    {
    case 7:
        break;
    default:
        break;
    }
#endif

    spin_lock_irqsave(&uart_lock, flags);

    cta_uart_control = rs_table[cta_console_index].uart_control;

    switch (parity)
    {
    case 'o':
    case 'O':
        cta_uart_control|=(URCS_PE);
        cta_uart_control&=~URCS_EVEN;
        break;
    case 'e':
    case 'E':
        cta_uart_control|=(URCS_PE|URCS_EVEN);
        break;
    default:
        break;
    }

    /* Write the control registers */
    cta_uart_control = ((cta_uart_control&0x8000FFFFUL)|(brsr<<URCS_BRSHFT));
    //cta_uart_control |= URCS_RXEN;         // enable RX

    rs_table[cta_console_index].uart_control = cta_uart_control;
    UARTREG(cta_console_index, URCS) = cta_uart_control;

    spin_unlock_irqrestore(&uart_lock, flags);

    return 0;
}

static void serial_console_write(struct console *co, const char *s,
                                 unsigned count)
{
    while (count--)
    {
        if (*s == '\n')
            cheetah_serial_outc('\r');
        cheetah_serial_outc(*s++);
    }
}

static struct tty_driver *serial_console_device(struct console *c, int *index)
{
    *index = cta_console_index;
    return serial_driver;
    //*index = cta_console_index;
    //return &serial_driver[cta_console_index];
}

static struct console cta_console = {
    .name =     "ttyS",
    .write =    serial_console_write,
    .device =   serial_console_device,
    .setup =    serial_console_setup,
    .flags =    CON_PRINTBUFFER,
    .index =    -1,
};

/*
 *	Register console.
 */
static int __init cheetah_serial_console_init(void)
{
    register_console(&cta_console);
    return 0;
}
console_initcall(cheetah_serial_console_init);

#endif

#if defined(CONFIG_SERIAL_CONSOLE) || defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)

static int __init _console_setup(char *str)
{
    volatile unsigned int *p;
    unsigned int brsr;

    if (!strncmp("ttyS", str, 4))
        cta_console_index = str[4] - '0';

    brsr = urcs_cal_baud_cnt(CONFIG_CHEETAH_UART_BAUDRATE);
    p = (unsigned int *) (UR_BASE + URCS + (0x100 * cta_console_index));
    *p =  ((*p &0x8000FFFFUL)|(brsr<<URCS_BRSHFT));

    cta_console.index = cta_console_index;

    return 0;
}
__setup("console=", _console_setup);

#endif

