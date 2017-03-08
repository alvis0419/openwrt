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
#include <linux/smp_lock.h>
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


#include <asm/setup.h>

#include <asm/system.h>

#include <asm/irq.h>

#include <asm/mach-cheetah/cheetah.h>

#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
#include <asm/mach-cheetah/idb.h>
#endif

#define NR_PORTS 3

struct serial_state
{
    char devname[6];
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
    struct async_struct *info;
    struct tasklet_struct uart_irq_tasklet;
    unsigned int uart_control;
};

struct async_struct
{
    unsigned long       port;
    int         flags;
    int         xmit_fifo_size;
    struct serial_state *state;
    struct tty_struct   *tty;
    int         read_status_mask;
    int         timeout;
    int         quot;
    int         x_char; /* xon/xoff character */
    int         close_delay;
    unsigned short      closing_wait;
    unsigned short      closing_wait2; /* obsolete */
    int         MCR;    /* Modem control register */
    unsigned long       last_active;
    int         line;
    int         curr_baudrate;
    int         blocked_open; /* # of blocked opens */
    struct circ_buf     xmit;
    struct tasklet_struct   tlet;
#ifdef DECLARE_WAITQUEUE
    wait_queue_head_t   open_wait;
    wait_queue_head_t   close_wait;
    wait_queue_head_t   delta_msr_wait;
#else	
    struct wait_queue   *open_wait;
    struct wait_queue   *close_wait;
    struct wait_queue   *delta_msr_wait;
#endif	
};

#if defined(CONFIG_SERIAL_CONSOLE) || defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
static int cta_console_index;
#endif

static struct serial_state rs_table[NR_PORTS];
static DEFINE_SPINLOCK(uart_lock);

#define UARTREG(idx,reg)  (*(volatile unsigned int*)(UR_BASE+reg+(0x100 * idx)))

#define URCS_CTRL_MASK   (~((unsigned int)(1<<URCS_BRSHFT)-1)|URCS_TIE|URCS_RIE|URCS_PE|URCS_EVEN)

u32 urcs_cal_baud_cnt(u32 baudrate)
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

static void rs_enable_tx_interrupts(int idx)
{
    if (transmit_chars(idx))
        urcs_enable(idx, URCS_TIE);
}

static void rs_disable_rx_interrupts(int idx) 
{
    urcs_disable(idx, URCS_RIE);
}

static void rs_enable_rx_interrupts(int idx)
{
    urcs_enable(idx, URCS_RIE);
}

static void change_speed(struct async_struct *info,
                         struct ktermios *old_termios)
{
    int baud;

    if (!info->tty || !info->tty->termios)
        return;

    /* Determine divisor based on baud rate */
    baud = tty_get_baud_rate(info->tty);

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
    struct async_struct *info = (struct async_struct *)tty->driver_data;
    unsigned long flags;

    spin_lock_irqsave(&uart_lock, flags);

    rs_disable_tx_interrupts(info->line);

    spin_unlock_irqrestore(&uart_lock, flags);
}

static void rs_start(struct tty_struct *tty)
{
    struct async_struct *info = (struct async_struct *)tty->driver_data;
    unsigned long flags;

    spin_lock_irqsave(&uart_lock, flags);
    if (info->xmit.head != info->xmit.tail && info->xmit.buf)
    {
        rs_enable_tx_interrupts(info->line);
    }
    spin_unlock_irqrestore(&uart_lock, flags);
}

static void rs_sched_event(struct async_struct *info)
{
    tasklet_schedule(&info->tlet);
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
static void receive_chars(int idx, struct async_struct *info)
{
    struct tty_struct *tty = info->tty;
    unsigned char ch, flag;
    volatile int *p = (int *) (UR_BASE + (0x100 * idx));
    struct  async_icount *icount;
    //int rx_loop = 0;
    int ubr;

    flag = TTY_NORMAL;
    icount = &info->state->icount;

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
                handle_sysrq(ch, tty);
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

        tty_insert_flip_char(tty, ch, flag);
    } while (1); //rx_loop++ < 256) ;

#if 0
    // if rx some data, push it
    if (rx_loop)
        tty_flip_buffer_push(tty);
#endif

    tty_schedule_flip(tty);

    return;
}

static int transmit_chars(int idx)
{
    struct async_struct *info = rs_table[idx].info;
    volatile int *p = (int *) (UR_BASE + (0x100 * idx));
    int enable_tx_intr = 0;

    for (;;)
    {
        if ((p[URCS>>2]&URCS_TF))
        {
            enable_tx_intr = 1;
            break;
        }
        else
            if (info->x_char)
        {
            p[URBR>>2] = ((unsigned int)info->x_char)<<URBR_DTSHFT;
            info->state->icount.tx++;
            info->x_char = 0;
            enable_tx_intr = 1;
        }
        else
        {
            if (info->xmit.head == info->xmit.tail || info->tty->stopped || info->tty->hw_stopped)
            {
                break;
            }
            else
            {
                p[URBR>>2] = ((unsigned int)info->xmit.buf[info->xmit.tail++]) <<URBR_DTSHFT;
                info->xmit.tail = info->xmit.tail & (SERIAL_XMIT_SIZE-1);
                info->state->icount.tx++;
                enable_tx_intr = 1;
                if (info->xmit.head == info->xmit.tail)
                    break;
            }
        }
    }

    if (CIRC_CNT(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE) < WAKEUP_CHARS)
        rs_sched_event(info);

    return enable_tx_intr;
}

static irqreturn_t cta_uart_interrupt(int irq, void *data)
{
    int idx = irq - IRQ_UART0;

    urcs_disable(idx, (URCS_RIE | URCS_TIE));

    tasklet_hi_schedule(&rs_table[idx].uart_irq_tasklet);

    return IRQ_HANDLED;
}

static void cta_uart_deliver(unsigned long idx)
{
    volatile int *p = (int *) (UR_BASE + (0x100 * idx));
    unsigned int status;
    struct async_struct * info;
    int pass_counter = 256;
    int tx_intr_enable = 0;

    /* XXX : Clear any interrupts we might be about to handle */
    info = rs_table[idx].info;
    if (!info || !info->tty)
    {
        // printk("cheetah_uart_interrupt: ignored\n");
        return ;
    }

    status = p[URCS>>2];
    do
    {
        // only handle these event
        if (!(status & (URCS_RF|URCS_TE)))
            break;

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
                info->state->icount.frame++;

            /* RX Parity Error */
            if (status & URCS_PER)
                info->state->icount.parity++;

            receive_chars(idx, info);
            info->last_active = jiffies;
        }

        status = p[URCS>>2];
    } while (--pass_counter > 0) ;

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
 * This routine is used to handle the "bottom half" processing for the
 * serial driver, known also the "software interrupt" processing.
 * This processing is done at the kernel interrupt level, after the
 * rs_interrupt() has returned, BUT WITH INTERRUPTS TURNED ON.  This
 * is where time-consuming activities which can not be done in the
 * interrupt driver proper are done; the interrupt driver schedules
 * them using rs_sched_event(), and they get done here.
 */

static void do_softint(unsigned long private_)
{
    struct async_struct *info = (struct async_struct *) private_;
    struct tty_struct   *tty;

    tty = info->tty;
    if (!tty)
        return;

    tty_wakeup(tty);
}

/*
 * ---------------------------------------------------------------
 * Low level utility subroutines for the serial driver:  routines to
 * figure out the appropriate timeout for an interrupt chain, routines
 * to initialize and startup a serial port, and routines to shutdown a
 * serial port.  Useful stuff like that.
 * ---------------------------------------------------------------
 */

static int startup(struct async_struct * info)
{
    unsigned long flags;
    int retval=0;
    unsigned long page;

    if (info->flags & ASYNC_INITIALIZED)
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
        if (info->tty)
            set_bit(TTY_IO_ERROR, &info->tty->flags);
        retval = 0;
    }

    rs_enable_tx_interrupts(info->line);
    rs_enable_rx_interrupts(info->line);

    if (info->tty)
        clear_bit(TTY_IO_ERROR, &info->tty->flags);
    info->xmit.head = info->xmit.tail = 0;

    /*
     * and set the speed of the serial port
     */
    change_speed(info, NULL);

    info->flags |= ASYNC_INITIALIZED;
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
static void shutdown(struct async_struct * info)
{
    unsigned long   flags;

    if (!(info->flags & ASYNC_INITIALIZED))
        return;

#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("Shutting down serial port %d ....\n", info->line);
#endif

    spin_lock_irqsave(&uart_lock, flags); /* Disable interrupts */

    /*
     * clear delta_msr_wait queue to avoid mem leaks: we may free the irq
     * here so the queue might never be waken up
     */
    wake_up_interruptible(&info->delta_msr_wait);

    rs_disable_tx_interrupts(info->line);
    rs_disable_rx_interrupts(info->line);

    if (info->xmit.buf)
    {
        free_page((unsigned long) info->xmit.buf);
        info->xmit.buf = NULL;
    }

    if (info->tty)
        set_bit(TTY_IO_ERROR, &info->tty->flags);

    info->flags &= ~ASYNC_INITIALIZED;
    spin_unlock_irqrestore(&uart_lock, flags);
}


static int rs_put_char(struct tty_struct *tty, unsigned char ch)
{
    struct async_struct *info;
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
    struct async_struct *info = (struct async_struct *)tty->driver_data;
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
    struct async_struct *info;
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
    struct async_struct *info = (struct async_struct *)tty->driver_data;

    return CIRC_SPACE(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE);
}

static int rs_chars_in_buffer(struct tty_struct *tty)
{
    struct async_struct *info = (struct async_struct *)tty->driver_data;

    return CIRC_CNT(info->xmit.head, info->xmit.tail, SERIAL_XMIT_SIZE);
}

static void rs_flush_buffer(struct tty_struct *tty)
{
    struct async_struct *info = (struct async_struct *)tty->driver_data;
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
    struct async_struct *info = (struct async_struct *)tty->driver_data;
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
    struct async_struct *info = (struct async_struct *)tty->driver_data;
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
    struct async_struct *info = (struct async_struct *)tty->driver_data;
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

static int get_serial_info(struct async_struct * info,
                           struct serial_struct __user * retinfo)
{
    struct serial_struct tmp;
    struct serial_state *state = info->state;

    if (!retinfo)
        return -EFAULT;
    memset(&tmp, 0, sizeof(tmp));
    lock_kernel();
    tmp.type = state->type;
    tmp.line = state->line;
    tmp.port = state->port;
    tmp.irq = state->irq;
    tmp.flags = state->flags;
    tmp.xmit_fifo_size = state->xmit_fifo_size;
    tmp.baud_base = state->baud_base;
    tmp.close_delay = state->close_delay;
    tmp.closing_wait = state->closing_wait;
    tmp.custom_divisor = state->custom_divisor;
    unlock_kernel();
    if (copy_to_user(retinfo,&tmp,sizeof(*retinfo)))
        return -EFAULT;
    return 0;
}

static int set_serial_info(struct async_struct * info,
                           struct serial_struct __user * new_info)
{
    struct serial_struct new_serial;
    struct serial_state *state;
    unsigned int        change_irq,change_port;
    int             retval = 0;

    if (copy_from_user(&new_serial,new_info,sizeof(new_serial)))
        return -EFAULT;

    lock_kernel();
    state = info->state;

    change_irq = new_serial.irq != state->irq;
    change_port = (new_serial.port != state->port);
    if (change_irq || change_port || (new_serial.xmit_fifo_size != state->xmit_fifo_size))
    {
        unlock_kernel();
        return -EINVAL;
    }

    if (!serial_isroot())
    {
        if ((new_serial.baud_base != state->baud_base) ||
            (new_serial.close_delay != state->close_delay) ||
            (new_serial.xmit_fifo_size != state->xmit_fifo_size) ||
            ((new_serial.flags & ~ASYNC_USR_MASK) !=
             (state->flags & ~ASYNC_USR_MASK)))
            return -EPERM;
        state->flags = ((state->flags & ~ASYNC_USR_MASK) |
                        (new_serial.flags & ASYNC_USR_MASK));
        info->flags = ((info->flags & ~ASYNC_USR_MASK) |
                       (new_serial.flags & ASYNC_USR_MASK));
        state->custom_divisor = new_serial.custom_divisor;
        goto check_and_exit;
    }

    if (new_serial.baud_base < 9600)
    {
        unlock_kernel();
        return -EINVAL;
    }

    /*
     * OK, past this point, all the error checking has been done.
     * At this point, we start making changes.....
     */

    state->baud_base = new_serial.baud_base;
    state->flags = ((state->flags & ~ASYNC_FLAGS) |
                    (new_serial.flags & ASYNC_FLAGS));
    info->flags = ((state->flags & ~ASYNC_INTERNAL_FLAGS) |
                   (info->flags & ASYNC_INTERNAL_FLAGS));
    state->custom_divisor = new_serial.custom_divisor;
    state->close_delay = new_serial.close_delay * HZ/100;
    state->closing_wait = new_serial.closing_wait * HZ/100;
    info->tty->low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;

    check_and_exit:
    unlock_kernel();

    if (!(info->flags & ASYNC_INITIALIZED))
        retval = startup(info);

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
static int get_lsr_info(struct async_struct * info, unsigned int __user *value)
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


static int rs_ioctl(struct tty_struct *tty, struct file * file,
                    unsigned int cmd, unsigned long arg)
{
    struct async_struct * info = (struct async_struct *)tty->driver_data;
    struct async_icount cprev, cnow;    /* kernel counter temps */
    struct serial_icounter_struct icount;
    void __user *argp = (void __user *)arg;
    unsigned long flags;

    if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
        (cmd != TIOCSERCONFIG) && (cmd != TIOCSERGSTRUCT) &&
        (cmd != TIOCMIWAIT) && (cmd != TIOCGICOUNT))
    {
        if (tty->flags & (1 << TTY_IO_ERROR))
            return -EIO;
    }

    switch (cmd)
    {
    case TIOCGSERIAL:
        return get_serial_info(info, argp);
    case TIOCSSERIAL:
        return set_serial_info(info, argp);
    case TIOCSERCONFIG:
        return 0;
#if 0
    case TIOCSERGETLSR: /* Get line status register */
        return get_lsr_info(info, argp);
#endif
    case TIOCSERGSTRUCT:
        if (copy_to_user(argp,
                         info, sizeof(struct async_struct)))
            return -EFAULT;
        return 0;

        /*
         * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
         * - mask passed in arg for lines of interest
         *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
         * Caller should use TIOCGICOUNT to see which one it was
         */
    case TIOCMIWAIT:
        spin_lock_irqsave(&uart_lock, flags);
        /* note the counters on entry */
        cprev = info->state->icount;
        spin_unlock_irqrestore(&uart_lock, flags);
        while (1)
        {
            interruptible_sleep_on(&info->delta_msr_wait);
            /* see if a signal did it */
            if (signal_pending(current))
                return -ERESTARTSYS;
            spin_lock_irqsave(&uart_lock, flags);
            cnow = info->state->icount; /* atomic copy */
            spin_unlock_irqrestore(&uart_lock, flags);
            if (cnow.cts == cprev.cts)
                return -EIO; /* no change => error */
            if ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts))
            {
                return 0;
            }
            cprev = cnow;
        }
        /* NOTREACHED */

        /*
         * Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
         * Return: write counters to the user passed counter struct
         * NB: both 1->0 and 0->1 transitions are counted except for
         *     RI where only 0->1 is counted.
         */
    case TIOCGICOUNT:
        spin_lock_irqsave(&uart_lock, flags);
        cnow = info->state->icount;
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
    struct async_struct *info = (struct async_struct *)tty->driver_data;
    unsigned int cflag = tty->termios->c_cflag;
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
        cta_uart_control|=(urcs_cal_baud_cnt(tty_termios_baud_rate(tty->termios))<<URCS_BRSHFT);

        rs_table[uart_idx].baud_base = tty_termios_baud_rate(tty->termios);
    }

    cta_uart_control |= URCS_RXEN;
    rs_table[uart_idx].uart_control = cta_uart_control;
    UARTREG(uart_idx, URCS) = cta_uart_control;

    if ((old_termios->c_cflag & CRTSCTS) &&
        !(tty->termios->c_cflag & CRTSCTS))
    {
        UARTREG(uart_idx, URCS2) &= (~(URCS2_RX_FLOW_CTRL_EN | URCS2_TX_FLOW_CTRL_EN));
    }
    /* turn on */
    else if (!(old_termios->c_cflag & CRTSCTS) &&
             (tty->termios->c_cflag & CRTSCTS))
    {
        UARTREG(uart_idx, URCS2) |= (URCS2_RX_FLOW_CTRL_EN | URCS2_TX_FLOW_CTRL_EN);
    }

    spin_unlock_irqrestore(&uart_lock, flags);

#if 1
    /*
     * No need to wake up processes in open wait, since they
     * sample the CLOCAL flag once, and don't recheck it.
     * XXX  It's not clear whether the current behavior is correct
     * or not.  Hence, this may change.....
     */
    if (!(old_termios->c_cflag & CLOCAL) &&
        (tty->termios->c_cflag & CLOCAL))
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
    struct async_struct * info = (struct async_struct *)tty->driver_data;
    struct serial_state *state;
    unsigned long flags;

    if (!info)
        return;

    state = info->state;

    spin_lock_irqsave(&uart_lock, flags);

    if (tty_hung_up_p(filp))
    {
        spin_unlock_irqrestore(&uart_lock, flags);
        return;
    }

#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("rs_close ttys%d, count = %d\n", info->line, state->count);
#endif
    if ((tty->count == 1) && (state->count != 1))
    {
        /*
         * Uh, oh.  tty->count is 1, which means that the tty
         * structure will be freed.  state->count should always
         * be one in these conditions.  If it's greater than
         * one, we've got real problems, since it means the
         * serial port won't be shutdown.
         */
#ifdef CHEETAH_UART_DEBUG_OPEN
        printk("rs_close: bad serial port count; tty->count is 1, "
               "state->count is %d\n", state->count);
#endif
        state->count = 1;
    }
    if (--state->count < 0)
    {
#ifdef CHEETAH_UART_DEBUG_OPEN
        printk("rs_close: bad serial port count for ttys%d: %d\n",
               info->line, state->count);
#endif
        state->count = 0;
    }
    if (state->count)
    {
        spin_unlock_irqrestore(&uart_lock, flags);
        return;
    }
    info->flags |= ASYNC_CLOSING;
    /*
     * Now we wait for the transmit buffer to clear; and we notify 
     * the line discipline to only process XON/XOFF characters.
     */
    tty->closing = 1;

    spin_unlock_irqrestore(&uart_lock, flags);

    if (info->closing_wait != ASYNC_CLOSING_WAIT_NONE)
        tty_wait_until_sent(tty, info->closing_wait);
    if (info->flags & ASYNC_INITIALIZED)
    {
        /*
         * Before we drop DTR, make sure the UART transmitter
         * has completely drained; this is especially
         * important if there is a transmit FIFO!
         */
        rs_wait_until_sent(tty, info->timeout);
    }
    shutdown(info);
    rs_flush_buffer(tty);

    uart_disable_rx(info->line);

    tty_ldisc_flush(tty);
    tty->closing = 0;
    info->tty = NULL;
    if (info->blocked_open)
    {
        if (info->close_delay)
        {
            msleep_interruptible(jiffies_to_msecs(info->close_delay));
        }
        wake_up_interruptible(&info->open_wait);
    }
    info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CLOSING);
    wake_up_interruptible(&info->close_wait);
}

/*
 * rs_wait_until_sent() --- wait until the transmitter is empty
 */
static void rs_wait_until_sent(struct tty_struct *tty, int timeout)
{
#if 1
    return;
#else
    struct async_struct * info = (struct async_struct *)tty->driver_data;
    unsigned long orig_jiffies, char_time;
    int lsr;

    if (info->xmit_fifo_size == 0)
        return; /* Just in case.... */

    orig_jiffies = jiffies;

    lock_kernel();
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
#ifdef CHEETAH_UART_DEBUG_RS_WAIT_UNTIL_SENT
    printk("In rs_wait_until_sent(%d) check=%lu...", timeout, char_time);
    printk("jiff=%lu...", jiffies);
#endif
#if 0
    while (!((lsr = custom.serdatr) & SDR_TSRE))
    {
#ifdef CHEETAH_UART_DEBUG_RS_WAIT_UNTIL_SENT
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
    unlock_kernel();
#ifdef CHEETAH_UART_DEBUG_RS_WAIT_UNTIL_SENT
    printk("lsr = %d (jiff=%lu)...done\n", lsr, jiffies);
#endif
#endif
}

/*
 * rs_hangup() --- called by tty_hangup() when a hangup is signaled.
 */
static void rs_hangup(struct tty_struct *tty)
{
    struct async_struct * info = (struct async_struct *)tty->driver_data;
    struct serial_state *state = info->state;

    state = info->state;

    rs_flush_buffer(tty);
    shutdown(info);
    state->count = 0;
    info->flags &= ~ASYNC_NORMAL_ACTIVE;
    info->tty = NULL;
    wake_up_interruptible(&info->open_wait);
}

/*
 * ------------------------------------------------------------
 * rs_open() and friends
 * ------------------------------------------------------------
 */
static int block_til_ready(struct tty_struct *tty, struct file * filp,
                           struct async_struct *info)
{
#ifdef DECLARE_WAITQUEUE
    DECLARE_WAITQUEUE(wait, current);
#else
    struct wait_queue wait = { current, NULL};
#endif
    struct serial_state *state = info->state;
    int     retval;
    int     extra_count = 0; //do_clocal = 0,
    unsigned long   flags;

    /*
     * If the device is in the middle of being closed, then block
     * until it's done, and then try again.
     */
    if (tty_hung_up_p(filp) ||
        (info->flags & ASYNC_CLOSING))
    {
        if (info->flags & ASYNC_CLOSING)
            interruptible_sleep_on(&info->close_wait);
#ifdef SERIAL_DO_RESTART
        return((info->flags & ASYNC_HUP_NOTIFY) ?
               -EAGAIN : -ERESTARTSYS);
#else
        return -EAGAIN;
#endif
    }

    /*
     * If non-blocking mode is set, or the port is not enabled,
     * then make the check up front and then exit.
     */
    if ((filp->f_flags & O_NONBLOCK) ||
        (tty->flags & (1 << TTY_IO_ERROR)))
    {
        info->flags |= ASYNC_NORMAL_ACTIVE;
        return 0;
    }

#if 0
    if (tty->termios->c_cflag & CLOCAL)
        do_clocal = 1;
#endif

    /*
     * Block waiting for the carrier detect and the line to become
     * free (i.e., not in use by the callout).  While we are in
     * this loop, state->count is dropped by one, so that
     * rs_close() knows when to free things.  We restore it upon
     * exit, either normal or abnormal.
     */
    retval = 0;
    add_wait_queue(&info->open_wait, &wait);
#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("block_til_ready before block: ttys%d, count = %d\n",
           state->line, state->count);
#endif
    spin_lock_irqsave(&uart_lock, flags);
    if (!tty_hung_up_p(filp))
    {
        extra_count = 1;
        state->count--;
    }
    spin_unlock_irqrestore(&uart_lock, flags);
    info->blocked_open++;
    while (1)
    {
        if (1)
            break;
#if 0
        spin_lock_irqsave(&uart_lock, flags);
        if (tty->termios->c_cflag & CBAUD)
            rtsdtr_ctrl(SER_DTR|SER_RTS);
        spin_unlock_irqrestore(&uart_lock, flags);
#endif
        set_current_state(TASK_INTERRUPTIBLE);
        if (tty_hung_up_p(filp) ||
            !(info->flags & ASYNC_INITIALIZED))
        {
#ifdef SERIAL_DO_RESTART
            if (info->flags & ASYNC_HUP_NOTIFY)
                retval = -EAGAIN;
            else
                retval = -ERESTARTSYS;
#else
            retval = -EAGAIN;
#endif
            break;
        }
#if 0
        if (!(info->flags & ASYNC_CLOSING) &&
            (do_clocal || (!(ciab.pra & SER_DCD)) ))
            break;
#endif
        if (signal_pending(current))
        {
            retval = -ERESTARTSYS;
            break;
        }
#ifdef CHEETAH_UART_DEBUG_OPEN
        printk("block_til_ready blocking: ttys%d, count = %d\n",
               info->line, state->count);
#endif
        schedule();
    }
    __set_current_state(TASK_RUNNING);
    remove_wait_queue(&info->open_wait, &wait);
    if (extra_count)
        state->count++;
    info->blocked_open--;
#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("block_til_ready after blocking: ttys%d, count = %d\n",
           info->line, state->count);
#endif
    if (retval)
        return retval;
    info->flags |= ASYNC_NORMAL_ACTIVE;
    return 0;
}

static int get_async_struct(int line, struct async_struct **ret_info)
{
    struct async_struct *info;
    struct serial_state *sstate;

    sstate = rs_table + line;
    sstate->count++;
    if (sstate->info)
    {
        *ret_info = sstate->info;
        return 0;
    }
    info = kzalloc(sizeof(struct async_struct), GFP_KERNEL);
    if (!info)
    {
        sstate->count--;
        return -ENOMEM;
    }
#ifdef DECLARE_WAITQUEUE
    init_waitqueue_head(&info->open_wait);
    init_waitqueue_head(&info->close_wait);
    init_waitqueue_head(&info->delta_msr_wait);
#endif
    info->port = sstate->port;
    info->flags = sstate->flags;
    info->xmit_fifo_size = sstate->xmit_fifo_size;
    info->line = line;
    tasklet_init(&info->tlet, do_softint, (unsigned long)info);
    info->state = sstate;
    if (sstate->info)
    {
        kfree(info);
        *ret_info = sstate->info;
        return 0;
    }
    *ret_info = sstate->info = info;
    return 0;
}

/*
 * This routine is called whenever a serial port is opened.  It
 * enables interrupts for a serial port, linking in its async structure into
 * the IRQ chain.   It also performs the serial-specific
 * initialization for the tty structure.
 */
static int rs_open(struct tty_struct *tty, struct file * filp)
{
    struct async_struct *info;
    int             retval, line;

    line = tty->index;
    if ((line < 0) || (line >= NR_PORTS))
    {
        return -ENODEV;
    }
    retval = get_async_struct(line, &info);
    if (retval)
    {
        return retval;
    }
    tty->driver_data = info;
    info->tty = tty;
#ifdef CHEETAH_UART_DEBUG_OPEN
    printk("rs_open %s, count = %d\n", tty->name, info->state->count);
#endif
    info->tty->low_latency = (info->flags & ASYNC_LOW_LATENCY) ? 1 : 0;

    /*
     * If the port is the middle of closing, bail out now
     */
    if (tty_hung_up_p(filp) ||
        (info->flags & ASYNC_CLOSING))
    {
        if (info->flags & ASYNC_CLOSING)
            interruptible_sleep_on(&info->close_wait);
#ifdef SERIAL_DO_RESTART
        return((info->flags & ASYNC_HUP_NOTIFY) ?
               -EAGAIN : -ERESTARTSYS);
#else
        return -EAGAIN;
#endif
    }

    /*
     * Start up serial port
     */
    retval = startup(info);
    if (retval)
    {
        return retval;
    }

    retval = block_til_ready(tty, filp, info);
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
    volatile int *urcs2;
    char stat_buf[30];

    seq_printf(m, "%d: ",state->line);

    stat_buf[0] = 0;
    stat_buf[1] = 0;
    urcs2 = (int *) (UR_BASE + URCS2 + (0x100 * state->line));
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
    volatile int *p = (int *) (UR_BASE + (0x100 * cta_console_index));
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
    volatile int *p = (int *) (UR_BASE + (0x100 * cta_console_index));

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

static int __init cheetah_rs_init(void)
{
    unsigned long flags;
    struct serial_state * state;
    int i;

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
#ifdef CONFIG_CHEETAH_GVCOM_CONSOLE
    /* assume only 1 GVCOM */
    serial_driver->name_base = 1;
#endif
    serial_driver->major = TTY_MAJOR;
    serial_driver->minor_start = 64;
    serial_driver->type = TTY_DRIVER_TYPE_SERIAL;
    serial_driver->subtype = SERIAL_TYPE_NORMAL;
    serial_driver->init_termios = tty_std_termios;
    serial_driver->init_termios.c_cflag =
    B115200 | CS8 | CREAD | HUPCL | CLOCAL;
    serial_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
    tty_set_operations(serial_driver, &serial_ops);

    if (tty_register_driver(serial_driver))
        panic("Couldn't register serial driver\n");

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

        printk(KERN_INFO "ttyS%d is enabled\n",
               state->line);

        state->xmit_fifo_size = 16;

        spin_lock_irqsave(&uart_lock, flags);

        memcpy(state->devname, "uart", 4);
        state->devname[4] = '0' + i;
        state->devname[5] = 0;

        if (0 > request_irq(state->irq, cta_uart_interrupt, IRQF_DISABLED, state->devname, state))
            panic("Couldn't request IRQ for UART device\n");

        tasklet_init(&state->uart_irq_tasklet, cta_uart_deliver, (unsigned long) i);
        spin_unlock_irqrestore(&uart_lock, flags);
    }

#if defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)
    register_idb_command(&idb_uart_cmd);
#endif

    return 0;
}

static __exit void cheetah_rs_exit(void) 
{
    int error;
    int i;
    struct async_struct *info = rs_table[0].info;

    //printk("Unloading %s: version %s\n", serial_name, serial_version);

    for (i=0;i<NR_PORTS;i++)
    {
        info = rs_table[i].info;
        if (info)
            tasklet_kill(&info->tlet);
    }

    if ((error = tty_unregister_driver(serial_driver)))
        printk("SERIAL: failed to unregister serial driver (%d)\n",
               error);
    put_tty_driver(serial_driver);

    for (i=0;i<NR_PORTS;i++)
    {
        info = rs_table[i].info;
        if (info)
        {
            rs_table[0].info = NULL;
            kfree(info);
        }
    }

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
    volatile int *p = (int *) (UR_BASE + (0x100 * cta_console_index));

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
    int baud = CONFIG_CHEETAH_UART_BAUDRATE;
    //int bits = 8;
    int parity = 'n';
    char *s;
    int brsr = 0;
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

    cta_uart_control = rs_table[0].uart_control;

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

    rs_table[0].uart_control = cta_uart_control;
    UARTREG(0, URCS) = cta_uart_control;

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
#ifdef CONFIG_CHEETAH_GVCOM_CONSOLE
    (void)cta_console;
#else
    register_console(&cta_console);
#endif
    return 0;
}
console_initcall(cheetah_serial_console_init);

#endif

#if defined(CONFIG_SERIAL_CONSOLE) || defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_CHEETAH_INTERNAL_DEBUGGER)

static int __init _console_setup(char *str)
{
    volatile int *p;
    int brsr;

    if (!strncmp("ttyS", str, 4))
        cta_console_index = str[4] - '0';

    brsr = urcs_cal_baud_cnt(CONFIG_CHEETAH_UART_BAUDRATE);
    p = (int *) (UR_BASE + URCS + (0x100 * cta_console_index));
    *p =  ((*p &0x8000FFFFUL)|(brsr<<URCS_BRSHFT));

    cta_console.index = cta_console_index;

    return 0;
}
__setup("console=", _console_setup);

#endif

