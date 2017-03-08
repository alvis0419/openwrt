/* 
 *  Copyright (c) 2013	Montage Inc.	All rights reserved. 
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/system.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/gpio.h>

#ifdef CONFIG_GPIOLIB
struct camelot_gpio {
	struct gpio_chip gpio_chip;
};
#endif

enum {
	GPIO_NONE = 0,
	GPIO_IN,
	GPIO_OUT,
	GPIO_IO,
};
enum {
	GPIO_PKG_NONE = 0, // unknown
	GPIO_PKG_SDR,      // QFP128
	GPIO_PKG_DDR,      // QFN88
};

struct gpio_info cta_gpio_sdr_tb[] = {
    {  0, GPIO_IO,   0, 0, 0,                (EN_ETH0_FNC | EN_SDIO_FNC)                        },    // GPIO0
    {  1, GPIO_IO,   0, 0, 0,                (EN_ETH0_FNC | EN_SDIO_FNC)                        },    // GPIO1
    {  2, GPIO_IO,   0, 0, 0,                (EN_ETH0_FNC | EN_SDIO_FNC)                        },    // GPIO2
    {  3, GPIO_IO,   0, 0, 0,                (EN_ETH0_FNC | EN_SDIO_FNC)                        },    // GPIO3
    {  4, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_ADC_OUT_FNC)                     },    // GPIO4
    {  5, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_ADC_OUT_FNC)                     },    // GPIO5
    {  6, GPIO_IO,   0, 0, 0,                (EN_ADC_OUT_FNC)                                   },    // GPIO6
    {  7, GPIO_OUT,  1, 0, EN_PWM_AUX_FNC,   0                                                  },    // GPIO7
    {  8, GPIO_IN,   1, 0, EN_JTAG_AUX_FNC,  0                                                  },    // GPIO8
    {  9, GPIO_OUT,  1, 0, EN_JTAG_AUX_FNC,  0                                                  },    // GPIO9
    { 10, GPIO_IN,   1, 0, EN_JTAG_AUX_FNC,  0                                                  },    // GPIO10
    { 11, GPIO_IN,   1, 0, EN_JTAG_AUX_FNC,  0                                                  },    // GPIO11
    { 12, GPIO_IO,   0, 0, 0,                (EN_ROM1_FNC | EN_ANA_MON_FNC | EN_ADC_OUT_FNC)    },    // GPIO12
    { 13, GPIO_IO,   0, 0, 0,                (EN_LED_AUX_FNC | EN_ANA_MON_FNC | EN_ADC_OUT_FNC) },    // GPIO13
    { 14, GPIO_IO,   0, 0, 0,                (EN_PCM0_FNC | EN_ETH1_FNC)                        },    // GPIO14
    { 15, GPIO_IO,   0, 0, 0,                (EN_PCM0_FNC | EN_ETH1_FNC)                        },    // GPIO15
    { 16, GPIO_IO,   0, 0, 0,                (EN_PCM0_FNC | EN_ETH1_FNC)                        },    // GPIO16
    { 17, GPIO_IO,   0, 0, 0,                (EN_PCM0_FNC | EN_ETH1_FNC)                        },    // GPIO17
    { 18, GPIO_IO,   0, 0, 0,                (EN_PCM0_FNC | EN_ETH1_FNC)                        },    // GPIO18
    { 19, GPIO_IO,   0, 0, 0,                (EN_PCM1_FNC | EN_ETH1_FNC)                        },    // GPIO19
    { 20, GPIO_IO,   0, 0, 0,                (EN_PCM1_FNC | EN_ETH1_FNC)                        },    // GPIO20
    { 21, GPIO_IO,   0, 0, 0,                (EN_PCM1_FNC | EN_ADC_OUT_FNC)                     },    // GPIO21
    { 22, GPIO_IO,   0, 0, 0,                (EN_PCM1_FNC | EN_MDIO_AUX_FNC)                    },    // GPIO22
    { 23, GPIO_IO,   0, 0, 0,                (EN_PCM1_FNC | EN_MDIO_AUX_FNC)                    },    // GPIO23
    { 24, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_ADC_OUT_FNC)                     },    // GPIO24
    { 25, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_MDIO_FNC)                        },    // GPIO25
    { 26, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_MDIO_FNC)                        },    // GPIO26
    { 27, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_ETH0_FNC)                        },    // GPIO27
    { 28, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_ETH0_FNC)                        },    // GPIO28
    { 29, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_ETH0_FNC)                        },    // GPIO29
    { 30, GPIO_NONE, 0, 0, 0,                0                                                  },    // GPIO30
    { 31, GPIO_NONE, 0, 0, 0,                0                                                  },    // GPIO31
    { 32, GPIO_IO,   0, 0, EN_SWI2C_AUX_FNC, (EN_UART_AUX_FNC | EN_I2C_AUX_FNC)                 },    // GPIO32
    { 33, GPIO_IO,   0, 0, EN_SWI2C_AUX_FNC, (EN_UART_AUX_FNC | EN_I2C_AUX_FNC)                 },    // GPIO33
};
struct gpio_info cta_gpio_ddr_tb[] = {
    {  0, GPIO_IO,   1, 0, EN_JTAG_FNC,      0                                                  },    // GPIO0
    {  1, GPIO_IO,   1, 0, EN_JTAG_FNC,      0                                                  },    // GPIO1
    {  2, GPIO_IO,   0, 0, 0,                (EN_ETH0_FNC | EN_SDIO_FNC)                        },    // GPIO2
    {  3, GPIO_IO,   0, 0, 0,                (EN_ETH0_FNC | EN_SDIO_FNC)                        },    // GPIO3
    {  4, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_ADC_OUT_FNC)                     },    // GPIO4
    {  5, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_ADC_OUT_FNC)                     },    // GPIO5
    {  6, GPIO_IO,   0, 0, 0,                (EN_ADC_OUT_FNC)                                   },    // GPIO6
    {  7, GPIO_IO,   0, 0, 0,                (EN_LED_FNC | EN_ADC_OUT_FNC)                      },    // GPIO7
    {  8, GPIO_IO,   0, 0, 0,                (EN_LED_FNC | EN_ADC_OUT_FNC)                      },    // GPIO8
    {  9, GPIO_IO,   0, 0, 0,                (EN_LED_FNC | EN_ADC_OUT_FNC)                      },    // GPIO9
    { 10, GPIO_IO,   0, 0, 0,                (EN_PWM_FNC | EN_ADC_OUT_FNC)                      },    // GPIO10
    { 11, GPIO_IO,   0, 0, 0,                (EN_PWM_FNC)                                       },    // GPIO11
    { 12, GPIO_IO,   0, 0, 0,                (EN_ROM1_FNC | EN_ANA_MON_FNC | EN_ADC_OUT_FNC)    },    // GPIO12
    { 13, GPIO_IO,   0, 0, 0,                (EN_LED_AUX_FNC | EN_ANA_MON_FNC | EN_ADC_OUT_FNC) },    // GPIO13
    { 14, GPIO_IO,   0, 0, 0,                (EN_PCM0_FNC | EN_ETH1_FNC)                        },    // GPIO14
    { 15, GPIO_IO,   0, 0, 0,                (EN_PCM0_FNC | EN_ETH1_FNC)                        },    // GPIO15
    { 16, GPIO_IO,   0, 0, 0,                (EN_PCM0_FNC | EN_ETH1_FNC)                        },    // GPIO16
    { 17, GPIO_IO,   0, 0, 0,                (EN_PCM0_FNC | EN_ETH1_FNC)                        },    // GPIO17
    { 18, GPIO_IO,   0, 0, 0,                (EN_PCM0_FNC | EN_ETH1_FNC)                        },    // GPIO18
    { 19, GPIO_IO,   0, 0, 0,                (EN_PCM1_FNC | EN_ETH1_FNC)                        },    // GPIO19
    { 20, GPIO_IO,   0, 0, 0,                (EN_PCM1_FNC | EN_ETH1_FNC)                        },    // GPIO20
    { 21, GPIO_IO,   0, 0, 0,                (EN_PCM1_FNC | EN_ADC_OUT_FNC)                     },    // GPIO21
    { 22, GPIO_IO,   0, 0, 0,                (EN_PCM1_FNC | EN_MDIO_AUX_FNC)                    },    // GPIO22
    { 23, GPIO_IO,   0, 0, 0,                (EN_PCM1_FNC | EN_MDIO_AUX_FNC)                    },    // GPIO23
    { 24, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_ADC_OUT_FNC)                     },    // GPIO24
    { 25, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_MDIO_FNC)                        },    // GPIO25
    { 26, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_MDIO_FNC)                        },    // GPIO26
    { 27, GPIO_IO,   0, 0, 0,                (EN_SDIO_FNC | EN_ETH0_FNC)                        },    // GPIO27
    { 28, GPIO_IO,   1, 0, EN_JTAG_FNC,      0                                                  },    // GPIO28
    { 29, GPIO_IO,   1, 0, EN_JTAG_FNC,      0                                                  },    // GPIO29
    { 30, GPIO_IO,   0, 0, EN_SWI2C_FNC,     (EN_UART_FNC | EN_I2C_AUX_FNC)                     },    // GPIO30
    { 31, GPIO_IO,   0, 0, EN_SWI2C_FNC,     (EN_UART_FNC | EN_I2C_AUX_FNC)                     },    // GPIO31
    { 32, GPIO_NONE, 0, 0, 0,                0                                                  },    // GPIO32
    { 33, GPIO_NONE, 0, 0, 0,                0                                                  },    // GPIO33
};

#define GPIO_SDR_TBL_NUMS       (sizeof(cta_gpio_sdr_tb) / sizeof(struct gpio_info))
#define GPIO_DDR_TBL_NUMS       (sizeof(cta_gpio_ddr_tb) / sizeof(struct gpio_info))

#if defined(CONFIG_TODO)
static u32 chip_pkg = GPIO_PKG_NONE;
#endif

#if defined(CONFIG_TODO)
/* internal function: change the shared pin to gpio mode */
static struct gpio_info *__camelot_gpio_pin_enable(unsigned gpio)
{
    struct gpio_info *info = NULL;

    int sel = 0;

    if (chip_pkg == GPIO_PKG_NONE) {
        if (!(GPREG(PINMUX) & EN_SIP_FNC)) {
            // SDR mode
            chip_pkg = GPIO_PKG_SDR;
        }
        else {
            // DDR mode
            chip_pkg = GPIO_PKG_DDR;
        }
    }
    if (chip_pkg == GPIO_PKG_SDR) {
        if (gpio < GPIO_SDR_TBL_NUMS) {
            GPREG(PINMUX) &= ~cta_gpio_sdr_tb[gpio].disable;
            GPREG(PINMUX) |= cta_gpio_sdr_tb[gpio].enable;
            info = &cta_gpio_sdr_tb[gpio];
            sel = cta_gpio_sdr_tb[gpio].sel;
        }
    }
    if (chip_pkg == GPIO_PKG_DDR) {
        if (gpio < GPIO_DDR_TBL_NUMS) {
            GPREG(PINMUX) &= ~cta_gpio_ddr_tb[gpio].disable;
            GPREG(PINMUX) |= cta_gpio_ddr_tb[gpio].enable;
            info = &cta_gpio_ddr_tb[gpio];
            sel = cta_gpio_ddr_tb[gpio].sel;
        }
    }
    if (sel)
    	GPREG((gpio / 32) ? GP2SEL : GPSEL) |= ( 1 << (gpio % 32) );
    else
    	GPREG((gpio / 32) ? GP2SEL : GPSEL) &= ~( 1 << (gpio % 32) );

    return info;
}
#endif

/* only GPIO0 support interrupt mode. It might not enabled, though. Check it on first usage! */
int camelot_gpio_to_irq(unsigned gpio)
{
#if defined(CONFIG_TODO)
	if(gpio==0)
	{
		printk("camelot_gpio_to_irq: gpio interrupt might not be enabled.\n");
		return IRQ_GPIO0;
	}
	else
#endif	
		return -EINVAL;
}
EXPORT_SYMBOL(camelot_gpio_to_irq);

/* The values are boolean, zero for low, nonzero for high */
int camelot_gpio_get_value(unsigned gpio)
{
#if !defined(CONFIG_TODO)
    return 0;
#else
	unsigned long flags;
	int value;

	if(gpio < CHEETAH_GPIO_LINES)
	{
		local_irq_save(flags);

		__camelot_gpio_pin_enable(gpio);

		value = (GPREG((gpio / 32) ? GP2VAL : GPVAL) & (1 << (gpio % 32)));

		local_irq_restore(flags);
		return value;
	} 
	else
		return 0;
#endif
}
EXPORT_SYMBOL(camelot_gpio_get_value);

void camelot_gpio_set_value(unsigned gpio, int value)
{
#if defined(CONFIG_TODO)
    unsigned long flags;

	if(gpio < CHEETAH_GPIO_LINES)
	{
		local_irq_save(flags);

		__camelot_gpio_pin_enable(gpio);

		if (value)
			GPREG((gpio / 32) ? GP2SET : GPSET) = 1 << (gpio % 32) ;
		else
			GPREG((gpio / 32) ? GP2CLR : GPCLR) = 1 << (gpio % 32) ;

		local_irq_restore(flags);
	}
#endif
}
EXPORT_SYMBOL(camelot_gpio_set_value);

int camelot_gpio_direction_input(unsigned gpio)
{
#if !defined(CONFIG_TODO)
    return -EINVAL;
#else
    struct gpio_info *info;
    unsigned long flags;
    int err = -EINVAL;

	if(gpio < CHEETAH_GPIO_LINES)
	{
		local_irq_save(flags);

		info = __camelot_gpio_pin_enable(gpio);
        if (info && (info->dir & GPIO_IN)) {
    		GPREG((gpio / 32) ? GP2DIR : GPDIR) &= ~(1 << (gpio % 32));		/* set the corresponding bit to 0 for input */
		    err = 0;
        }

		local_irq_restore(flags);
	}

	return err;
#endif
}
EXPORT_SYMBOL(camelot_gpio_direction_input);

int camelot_gpio_direction_output(unsigned gpio, int value)
{
#if !defined(CONFIG_TODO)
    return -EINVAL;
#else
    struct gpio_info *info;
    unsigned long flags;
    int err = -EINVAL;

	if(gpio < CHEETAH_GPIO_LINES)
	{
		local_irq_save(flags);
		info = __camelot_gpio_pin_enable(gpio);
        if (info && (info->dir & GPIO_OUT)) {
        	camelot_gpio_set_value(gpio, value);
		    GPREG((gpio / 32) ? GP2DIR : GPDIR) |= (1 << (gpio % 32));		/* set the corresponding bit to 1 for output */
            err = 0;
        }

		local_irq_restore(flags);
	}

	return err;
#endif
}
EXPORT_SYMBOL(camelot_gpio_direction_output);
#define camelot_gpio_dbg_show NULL

#ifdef CONFIG_GPIOLIB
static int cta_gpio_request(struct gpio_chip *chip, unsigned offset)
{
    return 0;
}
static void cta_gpio_free(struct gpio_chip *chip, unsigned offset)
{
/*
 * Don't change gpio status, even it's free
 * For example:
 *     For CONFIG_GPIO_AUDIO_AMP_POWER, pcm device is closed, but GPIO_MUTE_PCM is still work
 */
#if 0
    struct gpio_info *info = __camelot_gpio_pin_enable(offset);
    unsigned gpio = offset;
    if (info) {
        /* can't be sure that other gpio still is used
        if (info->enable)
            GPREG(PINMUX) &= ~info->enable;
        */
        if (info->sel)
        	GPREG((gpio / 32) ? GP2SEL : GPSEL) &= ~( 1 << (gpio % 32) );
    }
#endif
}
static int cta_gpio_direction_out(struct gpio_chip *chip,
				     unsigned offset, int value)
{
    return camelot_gpio_direction_output(offset, value);
}
static int cta_gpio_direction_in(struct gpio_chip *chip, unsigned offset)
{
    return camelot_gpio_direction_input(offset);
}
static int cta_gpio_get(struct gpio_chip *chip, unsigned offset)
{
   return camelot_gpio_get_value(offset);
}
static void cta_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
   return camelot_gpio_set_value(offset, value);
}
static struct gpio_chip template_chip = {
    .label            = "cheetah",
    .owner            = THIS_MODULE,
    .request          = cta_gpio_request,
    .free             = cta_gpio_free,
    .direction_input  = cta_gpio_direction_in,
    .get              = cta_gpio_get,
    .direction_output = cta_gpio_direction_out,
    .set              = cta_gpio_set,
    .dbg_show         = camelot_gpio_dbg_show,
    .can_sleep        = 1,
};

static int __devinit camelot_gpio_probe(struct platform_device *pdev)
{
	struct camelot_gpio *camelot_gpio;
	int ret;

	camelot_gpio = kzalloc(sizeof(*camelot_gpio), GFP_KERNEL);
	if (camelot_gpio == NULL)
		return -ENOMEM;
    camelot_gpio->gpio_chip = template_chip;
	camelot_gpio->gpio_chip.ngpio = CHEETAH_GPIO_LINES;
	camelot_gpio->gpio_chip.dev = &pdev->dev;
    camelot_gpio->gpio_chip.base = 0;

	ret = gpiochip_add(&camelot_gpio->gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n",
			ret);
		goto err;
	}

	platform_set_drvdata(pdev, camelot_gpio);

	return ret;

err:
	kfree(camelot_gpio);
	return ret;
}

static int __devexit camelot_gpio_remove(struct platform_device *pdev)
{
	struct camelot_gpio *camelot_gpio = platform_get_drvdata(pdev);
	int ret;

	ret = gpiochip_remove(&camelot_gpio->gpio_chip);
	if (ret == 0)
		kfree(camelot_gpio);

	return ret;
}

static struct platform_driver camelot_gpio_driver = {
	.driver.name	= "cta-gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= camelot_gpio_probe,
	.remove		= __devexit_p(camelot_gpio_remove),
};
#endif

/*
 * SD/sd
 * LED/led
 * PWM/pwm
 * PCM0/pcm0
 * PCM1/pcm1
 */
struct pinmux_cmd {
    char name[5];            // enabled name
    char dname[5];           // diabled name
    unsigned int sdr_reg;
    unsigned int ddr_reg;
};
struct pinmux_cmd pinmux_cmd_tb[] = {
    { "SD",   "sd",   EN_SDIO_FNC, EN_SDIO_FNC },
    { "LED",  "led",  EN_LED_AUX_FNC, EN_LED_FNC },
    { "PWM",  "pwm",  EN_PWM_AUX_FNC, EN_PWM_FNC },
    { "PCM0", "pcm0", EN_PCM0_FNC, EN_PCM0_FNC },
    { "PCM1", "pcm1", EN_PCM1_FNC, EN_PCM1_FNC },
};
static int proc_pinmux_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
#if !defined(CONFIG_TODO)
    return -EIO;
#else
    char *p = page;
    int len;
    unsigned int reg = GPREG(PINMUX);
    struct gpio_info *info;
    int info_count;
    int en;
    int dir;
    int i;

    p += sprintf(p, "Current PinMux Status:\n");

    for (i=0;i<sizeof(pinmux_cmd_tb)/sizeof(struct pinmux_cmd);i++) {
        if (!(reg & EN_SIP_FNC)) {
            // SDR mode
            p += sprintf(p, "%s\n", (reg & pinmux_cmd_tb[i].sdr_reg) ? pinmux_cmd_tb[i].name : pinmux_cmd_tb[i].dname);
        }
        else {
            // DDR mode
            p += sprintf(p, "%s\n", (reg & pinmux_cmd_tb[i].ddr_reg) ? pinmux_cmd_tb[i].name : pinmux_cmd_tb[i].dname);
        }
    }

    p += sprintf(p, "Current GPIO Status:\n");

    if (!(reg & EN_SIP_FNC)) {
        // SDR mode
        info = cta_gpio_sdr_tb;
        info_count = GPIO_SDR_TBL_NUMS;
    }
    else {
        // DDR mode
        info = cta_gpio_ddr_tb;
        info_count = GPIO_DDR_TBL_NUMS;
    }

    for (i=0;i<info_count;i++) {
        en = 0;
        dir = info[i].dir;
        if (!(reg & info[i].disable)) {
            if (info[i].sel) {
                if ((reg & info[i].enable) && (GPREG((i / 32) ? GP2SEL : GPSEL) & ( 1 << (i % 32) )))
                    en = 1;
            }
            else {
                en = 1;
            }
        }

        switch(dir) {
            case GPIO_IN:
                p += sprintf(p, "%s%d\t", (en) ? "GPI" : "gpi", info[i].idx);
                break;
            case GPIO_OUT:
                p += sprintf(p, "%s%d\t", (en) ? "GPO" : "gpo", info[i].idx);
                break;
            case GPIO_IO:
                p += sprintf(p, "%s%d\t", (en) ? "GPIO" : "gpio", info[i].idx);
                break;
            default:
                break;
        }

        if ((i % 4) == 3)
            p += sprintf(p, "\n");
    }

    len = (p - page) - off;
    if (len < 0)
        len = 0;

    *eof = (len <= count) ? 1 : 0;
    *start = page + off;

    return len;
#endif
}

static int proc_pinmux_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
#if !defined(CONFIG_TODO)
    return -EIO;
#else
    char buf[300];
    unsigned int reg = GPREG(PINMUX);
    struct gpio_info *info;
    int info_count;
    int i, j;

    if (count > 0 && count < 299) {
        if (copy_from_user(buf, buffer, count))
            return -EFAULT;
        buf[count-1] = '\0';

        if (!(reg & EN_SIP_FNC)) {
            // SDR mode
            info = cta_gpio_sdr_tb;
            info_count = GPIO_SDR_TBL_NUMS;
        }
        else {
            // DDR mode
            info = cta_gpio_ddr_tb;
            info_count = GPIO_DDR_TBL_NUMS;
        }

        for (i=0;i<sizeof(pinmux_cmd_tb)/sizeof(struct pinmux_cmd);i++) {
            if (!strncasecmp(buf, pinmux_cmd_tb[i].name, strlen(pinmux_cmd_tb[i].name))) {
                if (!(GPREG(PINMUX) & EN_SIP_FNC)) {
                    // SDR mode
                    reg = pinmux_cmd_tb[i].sdr_reg;
                }
                else {
                    // DDR mode
                    reg = pinmux_cmd_tb[i].ddr_reg;
                }
                GPREG(PINMUX) |= reg;
                for (j=0;j<info_count;j++) {
                    if (info[j].sel && (info[j].enable & reg))
                        GPREG((j / 32) ? GP2SEL : GPSEL) &= ~( 1 << (j % 32) );
                }
                printk("switch back to %s\n", pinmux_cmd_tb[i].name);
                break;
            }
        }
    }
    return count;
#endif
}

static int __init camelot_gpio_init(void)
{
    struct proc_dir_entry *res;

    res = create_proc_entry("pinmux", S_IWUSR | S_IRUGO, NULL);
    if (!res)
        return -ENOMEM;

    res->read_proc = proc_pinmux_read;
    res->write_proc = proc_pinmux_write;

#ifdef CONFIG_GPIOLIB
    return platform_driver_register(&camelot_gpio_driver);
#else
    return 0;
#endif
}
arch_initcall(camelot_gpio_init);

static void __exit camelot_gpio_exit(void)
{
    remove_proc_entry("pinmux", NULL);

#ifdef CONFIG_GPIOLIB
    platform_driver_unregister(&camelot_gpio_driver);
#endif
}
module_exit(camelot_gpio_exit);

