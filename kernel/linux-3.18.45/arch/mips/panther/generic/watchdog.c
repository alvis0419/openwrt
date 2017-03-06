/* 
 *  Copyright (c) 2008, 2009, 2016    Montage Inc.    All rights reserved. 
 *
 *  Watchdog driver for Montage SoC
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>

#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/reg.h>
#include <asm/mach-cheetah/pmu.h>

static DEFINE_SPINLOCK(wdt_lock);

static int nowayout = WATCHDOG_NOWAYOUT;
static int heartbeat = 60;  /* (secs) Default is 1 minute */
static unsigned long wdt_status;
static unsigned long boot_status;

#define	WDT_IN_USE          0
#define	WDT_OK_TO_CLOSE     1

void cheetah_wdt_enable(void)
{
    unsigned long flags;
    u32 period;

    spin_lock_irqsave(&wdt_lock, flags);

    // 32KHz watchdog counter
    period = (0x0ffffff & (heartbeat * 32000));
    REG_WRITE32(PMU_WATCHDOG, (0x01000000 | period) );

    spin_unlock_irqrestore(&wdt_lock, flags);
}

void cheetah_wdt_disable(void)
{
    unsigned long flags;

    spin_lock_irqsave(&wdt_lock, flags);

    REG_WRITE32(PMU_WATCHDOG, 0x0);

    spin_unlock_irqrestore(&wdt_lock, flags);
}

static int cheetah_wdt_open(struct inode *inode, struct file *file)
{
    if (test_and_set_bit(WDT_IN_USE, &wdt_status))
        return -EBUSY;

    clear_bit(WDT_OK_TO_CLOSE, &wdt_status);
    cheetah_wdt_enable();
    return nonseekable_open(inode, file);
}

static ssize_t
cheetah_wdt_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
    if (len)
    {
        if (!nowayout)
        {
            size_t i;

            clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

            for (i = 0; i != len; i++)
            {
                char c;

                if (get_user(c, data + i))
                    return -EFAULT;
                if (c == 'V')
                    set_bit(WDT_OK_TO_CLOSE, &wdt_status);
            }
        }
        cheetah_wdt_enable();
    }
    return len;
}

static struct watchdog_info ident = {
    .options    = WDIOF_CARDRESET | WDIOF_MAGICCLOSE |
                  WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
    .identity   = "Camelot Watchdog",
};


static long cheetah_wdt_ioctl(struct file *file, unsigned int cmd,
                              unsigned long arg)
{
    int ret = -ENOTTY;
    int time;

    switch (cmd)
    {
    case WDIOC_GETSUPPORT:
        ret = copy_to_user((struct watchdog_info *)arg, &ident,
                           sizeof(ident)) ? -EFAULT : 0;
        break;

    case WDIOC_GETSTATUS:
        ret = put_user(0, (int *)arg);
        break;

    case WDIOC_GETBOOTSTATUS:
        ret = put_user(boot_status, (int *)arg);
        break;

    case WDIOC_KEEPALIVE:
        cheetah_wdt_enable();
        ret = 0;
        break;

    case WDIOC_SETTIMEOUT:
        ret = get_user(time, (int *)arg);
        if (ret)
            break;

        if (time <= 0 || time > 60)
        {
            ret = -EINVAL;
            break;
        }

        heartbeat = time;
        cheetah_wdt_enable();
        /* Fall through */

    case WDIOC_GETTIMEOUT:
        ret = put_user(heartbeat, (int *)arg);
        break;
    }
    return ret;
}

static int cheetah_wdt_release(struct inode *inode, struct file *file)
{
    if (test_bit(WDT_OK_TO_CLOSE, &wdt_status))
        cheetah_wdt_disable();
    else
        printk(KERN_CRIT "WATCHDOG: Device closed unexpectedly - "
               "timer will not stop\n");
    clear_bit(WDT_IN_USE, &wdt_status);
    clear_bit(WDT_OK_TO_CLOSE, &wdt_status);

    return 0;
}


static const struct file_operations cheetah_wdt_fops = {
    .owner          = THIS_MODULE,
    .llseek         = no_llseek,
    .write          = cheetah_wdt_write,
    .unlocked_ioctl = cheetah_wdt_ioctl,
    .open           = cheetah_wdt_open,
    .release        = cheetah_wdt_release,
};

static struct miscdevice cheetah_wdt_miscdev = {
    .minor      = WATCHDOG_MINOR,
    .name       = "watchdog",
    .fops       = &cheetah_wdt_fops,
};

static int __init cheetah_wdt_init(void)
{
    int ret;

    spin_lock_init(&wdt_lock);

    /* boot status support in HW? */
    boot_status = 0;

    ret = misc_register(&cheetah_wdt_miscdev);
    if (ret == 0)
        printk(KERN_INFO "Camelot Watchdog Timer: heartbeat %d sec\n",
               heartbeat);
    return ret;
}

static void __exit cheetah_wdt_exit(void)
{
    misc_deregister(&cheetah_wdt_miscdev);
}


module_init(cheetah_wdt_init);
module_exit(cheetah_wdt_exit);

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeat in seconds (default 60s)");

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started");

MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);

