/*
 *  Copyright (c) 2016	Montage Inc.	All rights reserved.
 */
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <asm/irq.h>
#include <asm/mach-cheetah/cheetah.h>

#define I2C_TARGET_CLOCK    (400 * 1000)

#define DEFAULT_I2C_TIMEOUT     (HZ/10)

//#define I2CMT_DEBUG
#ifdef I2CMT_DEBUG
#define i2c_debug(fmt, ...) \
    printk(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define i2c_debug(fmt, ...)
#endif


struct i2c_algo_mt_data
{
    wait_queue_head_t waitq;
    spinlock_t lock;
    u32 SR_enabled, SR_received;
};

#define I2C_PRERLO      (0x00)
#define I2C_PRERHI      (0x04)
#define I2C_CTR         (0x08)
#define     I2C_CTR_CORE_EN     0x80
#define     I2C_CTR_INTR_EN     0x40
#define     I2C_CTR_SLAVE_MODE  0x01
#define I2C_TX          (0x0c)
#define I2C_RX          (0x0c)
#define I2C_CMD         (0x10)
#define     I2C_CMD_START       0x80
#define     I2C_CMD_STOP        0x40
#define     I2C_CMD_READ        0x20
#define     I2C_CMD_WRITE       0x10
#define     I2C_CMD_ACK         0x08
#define     I2C_CMD_IACK        0x01     /* interrupt acknowledge */
#define I2C_STA         (0x10)
#define     I2C_STA_RX_NACK     0x80     /* 1: NACK,  0: ACK */
#define     I2C_STA_BUSY        0x40     /* I2C bus busy '1' after START signal detected, '0' after STOP signal detected */
#define     I2C_STA_ARB_LOST    0x20     /* arbitration lost */
#define     I2C_TIP             0x02     /* Transfer in progress. '1' when transferring data, '0' when transfer complete */
#define     I2C_IF              0x01     /* Interrupt Flag. This bit is set when an interrupt is pending. The Interrupt Flag is set when:  one byte transfer has been completed or arbitration is lost */
#define I2C_CMD_STA     (0x18)

#define I2CREG_READ32(x)    (*(volatile u32 *) (I2C_BASE + (x)))
#define I2CREG_WRITE32(x,val)	do { \
    i2c_debug("[I2CREG_WRITE32] %x, %x", x, (u32)val); \
    (*(volatile u32 *)(I2C_BASE + (x)) = (u32)(val)); \
} while(0)

#define I2CREG_UPDATE32(x,val,mask) do {           \
    u32 newval = (*(volatile u32 *) (I2C_BASE + (x)));     \
    newval = (( newval & ~(mask) ) | ( (val) & (mask) ));            \
    *(volatile u32 *)(I2C_BASE + (x)) = newval;            \
} while(0)

static inline unsigned char 
iic_cook_addr(struct i2c_msg *msg) 
{
    unsigned char addr;

    addr = (msg->addr << 1);

    if (msg->flags & I2C_M_RD)
        addr |= 1;

    /*
     * Read or Write?
     */
    if (msg->flags & I2C_M_REV_DIR_ADDR)
        addr ^= 1;

    i2c_debug("[%s:%d] addr %x\n", __func__, __LINE__, addr);
    return addr;   
}

static void 
mt_i2c_reset(struct i2c_algo_mt_data *mt_adap)
{

} 

static void 
mt_i2c_enable(struct i2c_algo_mt_data *mt_adap)
{
    u32 prescale = (PBUS_CLK/(5*I2C_TARGET_CLOCK)) - 1;
    i2c_debug("[%s:%d] enable i2c prescale %d\n", __func__, __LINE__, prescale);
    
    mt_adap->SR_enabled = I2C_IF | I2C_STA_ARB_LOST | I2C_STA_RX_NACK | I2C_STA_BUSY | I2C_TIP;

    I2CREG_WRITE32(I2C_CTR, 0);
    I2CREG_WRITE32(I2C_CMD, I2C_CMD_IACK);
    I2CREG_WRITE32(I2C_PRERLO, (prescale & 0x000000FFUL));
    I2CREG_WRITE32(I2C_PRERHI, ((prescale & 0x0000FF00UL) >> 8));

    I2CREG_WRITE32(I2C_CTR, I2C_CTR_CORE_EN | I2C_CTR_INTR_EN);
}

static void 
mt_i2c_transaction_cleanup(struct i2c_algo_mt_data *mt_adap)
{

}

/* 
 * NB: the handler has to clear the source of the interrupt! 
 * Then it passes the SR flags of interest to BH via adap data
 */
static irqreturn_t 
mt_i2c_irq_handler(int this_irq, void *dev_id) 
{
    struct i2c_algo_mt_data *mt_adap = dev_id;
    u32 sr;

    sr = I2CREG_READ32(I2C_STA);
    I2CREG_WRITE32(I2C_CMD, (I2CREG_READ32(I2C_CMD_STA) | I2C_CMD_IACK));
    i2c_debug("[%s:%d] sr %x\n", __func__, __LINE__, sr);

    if ((sr &= mt_adap->SR_enabled))
    {
        mt_adap->SR_received |= sr;
        i2c_debug("[%s:%d] SR_received %x\n", __func__, __LINE__, mt_adap->SR_received);
        wake_up_interruptible(&mt_adap->waitq);
    }

    return IRQ_HANDLED;
}

/* check all error conditions, clear them , report most important */
static int 
mt_i2c_error(u32 sr, u32 flags)
{
    int rc = 0;

    if ((flags & I2C_STA_RX_NACK) && (sr & I2C_STA_RX_NACK))
    {
        if ( !rc ) rc = -ENODEV;
    }
    if ((flags & I2C_STA_ARB_LOST) && (sr & I2C_STA_ARB_LOST))
    {
        if ( !rc ) rc = -EAGAIN;
    }

    return rc;  
}

static inline u32 
mt_i2c_get_srstat(struct i2c_algo_mt_data *mt_adap)
{
    unsigned long flags;
    u32 sr;

    spin_lock_irqsave(&mt_adap->lock, flags);
    sr = mt_adap->SR_received;
    mt_adap->SR_received = 0;
    spin_unlock_irqrestore(&mt_adap->lock, flags);

    i2c_debug("[%s:%d] get from WaitEvent sr %x\n", __func__, __LINE__, sr);
    return sr;
}

/*
 * sleep until interrupted, then recover and analyse the SR
 * saved by handler
 */
typedef int (* compare_func)(unsigned test, unsigned mask);
/* returns 1 on correct comparison */

static int 
mt_i2c_wait_event(struct i2c_algo_mt_data *mt_adap, 
                  unsigned flags, unsigned* status,
                  compare_func compare)
{
    unsigned sr = 0;
    int interrupted;
    int done;
    int rc = 0;

    do
    {
        interrupted = wait_event_interruptible_timeout (
                                                       mt_adap->waitq,
                                                       (done = compare( sr = mt_i2c_get_srstat(mt_adap) ,flags )),
                                                       DEFAULT_I2C_TIMEOUT
                                                       );
        if ((rc = mt_i2c_error(sr, flags)) < 0)
        {
            i2c_debug("[%s:%d] mt_i2c_error %x\n", __func__, __LINE__, sr);
            *status = sr;
            return rc;
        }
        else if (!interrupted)
        {
            i2c_debug("[%s:%d] !interrupted %x\n", __func__, __LINE__, sr);
            *status = sr;
            return -ETIMEDOUT;
        }
    } while (!done);

    *status = sr;

    return 0;
}

/*
 * Concrete compare_funcs 
 */
static int 
all_bits_clear(unsigned test, unsigned mask)
{
    return(test & mask) == 0;
}

static int 
any_bits_set(unsigned test, unsigned mask)
{
    i2c_debug("[%s:%d] test value: %x, mask : %d\n", __func__, __LINE__, test, mask);
    return(test & mask) != 0;
}

static int 
mt_i2c_wait_ack(struct i2c_algo_mt_data *mt_adap, int *status)
{
    return mt_i2c_wait_event(
                            mt_adap, 
                            I2C_IF | I2C_STA_ARB_LOST | I2C_STA_RX_NACK,
                            status, any_bits_set);
}

static int 
mt_i2c_wait_cmd_done(struct i2c_algo_mt_data *mt_adap, int *status)
{
    return mt_i2c_wait_event(
                            mt_adap, 
                            I2C_IF | I2C_STA_ARB_LOST,
                            status, any_bits_set);
}

static int 
mt_i2c_wait_idle(struct i2c_algo_mt_data *mt_adap, int *status)
{
    if (0==(I2CREG_READ32(I2C_STA) & (I2C_STA_BUSY | I2C_TIP)))
        return 0;

    return mt_i2c_wait_event(
                            mt_adap, I2C_STA_BUSY | I2C_TIP, status, all_bits_clear);
}

static int 
mt_i2c_send_target_addr(struct i2c_algo_mt_data *mt_adap, 
                        struct i2c_msg* msg)
{
    int rc;
    int status;

    i2c_debug("[%s:%d]\n", __func__, __LINE__);

    I2CREG_WRITE32(I2C_TX, iic_cook_addr(msg));

    I2CREG_WRITE32(I2C_CMD, I2C_CMD_START | I2C_CMD_WRITE);

    rc = mt_i2c_wait_ack(mt_adap, &status);

    return rc;
}

static int 
mt_i2c_write_byte(struct i2c_algo_mt_data *mt_adap, char byte, 
                  int stop)
{
    int rc;
    int status;

    I2CREG_WRITE32(I2C_TX, byte);

    if (stop)
        I2CREG_WRITE32(I2C_CMD, I2C_CMD_STOP | I2C_CMD_WRITE);
    else
        I2CREG_WRITE32(I2C_CMD, I2C_CMD_WRITE);

    rc = mt_i2c_wait_ack(mt_adap, &status);

    return rc;
} 

static int 
mt_i2c_read_byte(struct i2c_algo_mt_data *mt_adap, char* byte, 
                 int stop)
{
    int rc = 0;
    int status;

    if (stop)
    {
        I2CREG_WRITE32(I2C_CMD, I2C_CMD_STOP | I2C_CMD_READ | I2C_CMD_ACK);
        mt_i2c_wait_cmd_done(mt_adap, &status);
    }
    else
    {    
        I2CREG_WRITE32(I2C_CMD, I2C_CMD_READ);
        rc = mt_i2c_wait_ack(mt_adap, &status);
        if (rc != 0)
            return rc;
    }

    *byte = I2CREG_READ32(I2C_RX);
    
    return rc;
}

static int 
mt_i2c_writebytes(struct i2c_adapter *i2c_adap, const char *buf, int count)
{
    struct i2c_algo_mt_data *mt_adap = i2c_adap->algo_data;
    int ii;
    int rc = 0;
    i2c_debug("[%s:%d]\n", __func__, __LINE__);
    
    for (ii = 0; rc == 0 && ii != count; ++ii)
        rc = mt_i2c_write_byte(mt_adap, buf[ii], ii==count-1);
    return rc;
}

static int 
mt_i2c_readbytes(struct i2c_adapter *i2c_adap, char *buf, int count)
{
    struct i2c_algo_mt_data *mt_adap = i2c_adap->algo_data;
    int ii;
    int rc = 0;
    i2c_debug("[%s:%d]\n", __func__, __LINE__);

    for (ii = 0; rc == 0 && ii != count; ++ii)
        rc = mt_i2c_read_byte(mt_adap, &buf[ii], ii==count-1);

    return rc;
}

/*
 * Description:  This function implements combined transactions.  Combined
 * transactions consist of combinations of reading and writing blocks of data.
 * FROM THE SAME ADDRESS
 * Each transfer (i.e. a read or a write) is separated by a repeated start
 * condition.
 */
static int 
mt_i2c_handle_msg(struct i2c_adapter *i2c_adap, struct i2c_msg* pmsg) 
{
    struct i2c_algo_mt_data *mt_adap = i2c_adap->algo_data;
    int rc;

    rc = mt_i2c_send_target_addr(mt_adap, pmsg);
    if (rc < 0)
    {
        return rc;
    }

    if ((pmsg->flags&I2C_M_RD))
    {
        return mt_i2c_readbytes(i2c_adap, pmsg->buf, pmsg->len);
    }
    else
    {
        return mt_i2c_writebytes(i2c_adap, pmsg->buf, pmsg->len);
    }
}

/*
 * master_xfer() - main read/write entry
 */
static int 
mt_i2c_master_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg *msgs, 
                   int num)
{
    struct i2c_algo_mt_data *mt_adap = i2c_adap->algo_data;
    int im = 0;
    int ret = 0;
    int status;
    i2c_debug("[%s:%d]\n", __func__, __LINE__);

    ret = mt_i2c_wait_idle(mt_adap, &status);

    if (ret)
        return ret;

    //mt_i2c_reset(mt_adap);
    //mt_i2c_enable(mt_adap);

    for (im = 0; ret == 0 && im != num; im++)
    {
        ret = mt_i2c_handle_msg(i2c_adap, &msgs[im]);
    }

    mt_i2c_transaction_cleanup(mt_adap);

    if (ret)
        return ret;

    return im;   
}

static u32 
mt_i2c_func(struct i2c_adapter *adap)
{
    //return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
    //return I2C_FUNC_I2C | I2C_FUNC_PROTOCOL_MANGLING;
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_QUICK | 
           I2C_FUNC_SMBUS_READ_BYTE_DATA	| I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
           I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE;
}

static const struct i2c_algorithm mt_i2c_algo = {
    .master_xfer    = mt_i2c_master_xfer,
    .functionality  = mt_i2c_func,
};

static int 
mt_i2c_remove(struct platform_device *pdev)
{
    struct i2c_adapter *padapter = platform_get_drvdata(pdev);
    struct i2c_algo_mt_data *adapter_data = 
    (struct i2c_algo_mt_data *)padapter->algo_data;

    I2CREG_WRITE32(I2C_CTR, 0);
    I2CREG_WRITE32(I2C_CMD, I2C_CMD_IACK);

    kfree(adapter_data);
    kfree(padapter);

    platform_set_drvdata(pdev, NULL);

    return 0;
}

static int 
mt_i2c_probe(struct platform_device *pdev)
{
    int ret;
    struct i2c_adapter *new_adapter;
    struct i2c_algo_mt_data *adapter_data;
    i2c_debug("[%s:%d] probe\n", __func__, __LINE__);

    new_adapter = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
    if (!new_adapter)
    {
        ret = -ENOMEM;
        goto out;
    }

    adapter_data = kzalloc(sizeof(struct i2c_algo_mt_data), GFP_KERNEL);
    if (!adapter_data)
    {
        ret = -ENOMEM;
        goto free_adapter;
    }

    ret = request_irq(IRQ_I2C, mt_i2c_irq_handler, 0,
                      pdev->name, adapter_data);

    if (ret)
    {
        ret = -EIO;
        goto free_both;
    }

    memcpy(new_adapter->name, pdev->name, strlen(pdev->name));
    new_adapter->owner = THIS_MODULE;
    //new_adapter->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
    new_adapter->class = I2C_CLASS_HWMON;
    new_adapter->dev.parent = &pdev->dev;
    new_adapter->nr = (pdev->id != -1) ? pdev->id : 0;

    /*
     * Default values...should these come in from board code?
     */
    new_adapter->timeout = DEFAULT_I2C_TIMEOUT;
    new_adapter->algo = &mt_i2c_algo;

    init_waitqueue_head(&adapter_data->waitq);
    spin_lock_init(&adapter_data->lock);

    mt_i2c_reset(adapter_data);
    mt_i2c_enable(adapter_data);

    platform_set_drvdata(pdev, new_adapter);
    new_adapter->algo_data = adapter_data;

    i2c_add_adapter(new_adapter);
    i2c_debug("[%s:%d] probe success\n", __func__, __LINE__);

    return 0;

    free_both:
    kfree(adapter_data);

    free_adapter:
    kfree(new_adapter);

    out:
    i2c_debug("[%s:%d] probe failed\n", __func__, __LINE__);
    return ret;
}


static struct platform_driver mt_i2c_driver = {
    .probe      = mt_i2c_probe,
    .remove     = mt_i2c_remove,
    .driver     = {
        .owner  = THIS_MODULE,
        .name   = "mt-i2c",
    },
};

static int __init 
i2c_mt_init (void)
{
    int ret;

    ret = platform_driver_register(&mt_i2c_driver);
    if (ret)
        printk(KERN_ERR "i2c-mt: probe failed: %d\n", ret);

    return ret;
}

static void __exit 
i2c_mt_exit (void)
{
    platform_driver_unregister(&mt_i2c_driver);
    return;
}

module_init (i2c_mt_init);
module_exit (i2c_mt_exit);

