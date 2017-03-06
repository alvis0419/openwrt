#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/i2c-gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <asm/mach-cheetah/cheetah.h>
#include <asm/mach-cheetah/gpio.h>
#include <asm/mach-cheetah/cta_keypad.h>

#if !defined(CONFIG_CHEETAH_MTD_SIZE)
#define CONFIG_CHEETAH_MTD_SIZE 0
#endif

/* the default MTD partition layout is designed to use with OpenWRT */
#define MTDPART_BOOT_OFS           0x00000000
#define MTDPART_BOOT_SIZE          0x00010000
#define MTDPART_CDB_OFS            (MTDPART_BOOT_OFS + MTDPART_BOOT_SIZE)
#define MTDPART_CDB_SIZE           0x00010000
#if defined(CONFIG_CHEETAH_BACKUP_IMAGE_BUILD)
    #define MTDPART_LINUX_OFS          (CONFIG_CHEETAH_MTD_SIZE - CONFIG_CHEETAH_BACKUP_SIZE)
    #define MTDPART_LINUX_SIZE         0x00200000
    #define MTDPART_ROOTFS_OFS         (MTDPART_LINUX_OFS + MTDPART_LINUX_SIZE)
    #define MTDPART_ROOTFS_SIZE        (CONFIG_CHEETAH_BACKUP_SIZE - MTDPART_LINUX_SIZE)
    #define MTDPART_FIRMWARE_OFS       (MTDPART_CDB_OFS + MTDPART_CDB_SIZE)
    #define MTDPART_FIRMWARE_SIZE      (CONFIG_CHEETAH_MTD_SIZE - CONFIG_CHEETAH_BACKUP_SIZE - MTDPART_FIRMWARE_OFS - MTDPART_RWFS_SIZE)
#else
    #define MTDPART_LINUX_OFS          (MTDPART_CDB_OFS + MTDPART_CDB_SIZE)
    #define MTDPART_LINUX_SIZE         0x00200000
    #define MTDPART_ROOTFS_OFS         (MTDPART_LINUX_OFS + MTDPART_LINUX_SIZE)
    #define MTDPART_ROOTFS_SIZE        (CONFIG_CHEETAH_MTD_SIZE - CONFIG_CHEETAH_BACKUP_SIZE - MTDPART_ROOTFS_OFS - MTDPART_RWFS_SIZE)
    #define MTDPART_FIRMWARE_OFS       MTDPART_LINUX_OFS
    #define MTDPART_FIRMWARE_SIZE      (MTDPART_LINUX_SIZE + MTDPART_ROOTFS_SIZE)
#endif
#if defined(CONFIG_MTD_RWPART_EXTRA_PARTITION)
    #define MTDPART_RWFS_OFS           CONFIG_MTD_RWPART_EXTRA_PARTITION_OFFSET
    #define MTDPART_RWFS_SIZE          (CONFIG_CHEETAH_MTD_SIZE - CONFIG_MTD_RWPART_EXTRA_PARTITION_OFFSET - CONFIG_CHEETAH_BACKUP_SIZE)
#else
    #define MTDPART_RWFS_SIZE          0
#endif
#ifndef CONFIG_CHEETAH_BACKUP_IMAGE
    #define CONFIG_CHEETAH_BACKUP_SIZE 0
#endif
#if (MTDPART_ROOTFS_SIZE <= 0)
//#error "No Space for rootfs!!"
#endif

#if (CHEETAH_VERSION_CODE < CHEETAH_VERSION(2,0))
#error "Don't Support Cheetah Version 1.0"
#endif
struct mtd_partition cheetah_partitions[] =
{
    {
        .name       = "boot",
        .size       = MTDPART_BOOT_SIZE,
        .offset     = MTDPART_BOOT_OFS,
    },
    {
        .name       = "cdb",
        .size       = MTDPART_CDB_SIZE,
        .offset     = MTDPART_CDB_OFS,
    },
    {
        .name       = "firmware",
        .size       = MTDPART_FIRMWARE_SIZE,
        .offset     = MTDPART_FIRMWARE_OFS,     //MTDPART_OFS_APPEND
    },
    {
        .name       = "linux",
        .size       = MTDPART_LINUX_SIZE,
        .offset     = MTDPART_LINUX_OFS,        //MTDPART_OFS_APPEND
    },
    {
        .name       = "rootfs",
        .size       = MTDPART_ROOTFS_SIZE,      //MTDPART_SIZ_FULL
        .offset     = MTDPART_ROOTFS_OFS,       //MTDPART_OFS_APPEND
    },
#if defined(CONFIG_MTD_RWPART_EXTRA_PARTITION)
    {
        .name       = "rwfs",
        .size       = MTDPART_RWFS_SIZE,
        .offset     = MTDPART_RWFS_OFS,
    },
#endif
    /* ALWAYS keep this part at the end of the structure to for indicating the partition numbers */
    {
        .size       = 0,
        .offset     = 0,
    },
};


#define USB_EHCI_LEN		0x100

#if defined(CONFIG_CHEETAH_USB1_HOST_ROLE)

static u64 usb1_ehci_dmamask = DMA_BIT_MASK(32);

static struct resource cheetah_usb1_ehci_resources[] = {
	[0] = {
		.start		= USB_OTG_BASE,
		.end		= USB_OTG_BASE + USB_EHCI_LEN - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_USB_OTG,
		.end		= IRQ_USB_OTG,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device cheetah_usb1_ehci_device = {
	.name		= "mt-ehci",
	.id		= 1,
	.dev = {
		.dma_mask		= &usb1_ehci_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(cheetah_usb1_ehci_resources),
	.resource	= cheetah_usb1_ehci_resources,
};

#endif

#if defined(CONFIG_CHEETAH_USB_HOST_ROLE) || defined(CONFIG_CHEETAH_USB_DUAL_ROLE)

static u64 ehci_dmamask = DMA_BIT_MASK(32);

/* Montage EHCI (USB high speed host controller) */
static struct resource cheetah_usb_ehci_resources[] = {
	[0] = {
		.start		= USB_BASE,
		.end		= USB_BASE + USB_EHCI_LEN - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_USB_HOST,
		.end		= IRQ_USB_HOST,
		.flags		= IORESOURCE_IRQ,
	},
};

#ifdef CONFIG_CHEETAH_USB_DUAL_ROLE
struct platform_device cheetah_usb_ehci_device = {
#else
static struct platform_device cheetah_usb_ehci_device = {
#endif
	.name		= "mt-ehci",
	.id		= 0,
	.dev = {
		.dma_mask		= &ehci_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(cheetah_usb_ehci_resources),
	.resource	= cheetah_usb_ehci_resources,
};

#endif

#define USB_UDC_LEN		0x100

#if defined(CONFIG_CHEETAH_USB1_DEVICE_ROLE)

static u64 usb1_udc_dmamask = DMA_BIT_MASK(32);

/* Montage UDC (USB gadget controller) */
static struct resource cheetah_usb1_gdt_resources[] = {
	[0] = {
		.start		= USB_OTG_BASE,
		.end		= USB_OTG_BASE + USB_UDC_LEN - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_USB_OTG,
		.end		= IRQ_USB_OTG,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device cheetah_usb1_gdt_device = {
	.name		= "mt-udc",
	.id		= 1,
	.dev = {
		.dma_mask		= &usb1_udc_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(cheetah_usb1_gdt_resources),
	.resource	= cheetah_usb1_gdt_resources,
};
#endif

#if defined(CONFIG_CHEETAH_USB_DEVICE_ROLE) || defined(CONFIG_CHEETAH_USB_DUAL_ROLE)

static u64 udc_dmamask = DMA_BIT_MASK(32);

/* Montage UDC (USB gadget controller) */
static struct resource cheetah_usb_gdt_resources[] = {
	[0] = {
		.start		= USB_BASE,
		.end		= USB_BASE + USB_UDC_LEN - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_USB_HOST,
		.end		= IRQ_USB_HOST,
		.flags		= IORESOURCE_IRQ,
	},
};

#ifdef CONFIG_CHEETAH_USB_DUAL_ROLE
struct platform_device cheetah_usb_gdt_device = {
#else
static struct platform_device cheetah_usb_gdt_device = {
#endif
	.name		= "mt-udc",
	.id		= 0,
	.dev = {
		.dma_mask		= &udc_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(cheetah_usb_gdt_resources),
	.resource	= cheetah_usb_gdt_resources,
};
#endif

#if defined(CONFIG_CHEETAH_USB_DUAL_ROLE)

static u64 vbus_dmamask = DMA_BIT_MASK(32);

static struct platform_device cheetah_usb_vbus_device = {
	.name		= "mt-vbus",
	.id		= 0,
	.dev = {
		.dma_mask		= &vbus_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	//.num_resources	= ARRAY_SIZE(cheetah_usb_otg_resources),
	//.resource	= cheetah_usb_otg_resources,
};

#endif

#ifdef CONFIG_CHEETAH_SDHCI

#define CHEETAH_SZ_HSMMC	(0x100)

static struct resource cheetah_hsmmc_resource[] = {
	[0] = {
		.start = SDIO_BASE,
		.end   = SDIO_BASE + CHEETAH_SZ_HSMMC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDIO,
		.end   = IRQ_SDIO,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 cheetah_device_hsmmc_dmamask = 0xffffffffUL;

static struct platform_device cheetah_device_hsmmc0 = {
	.name		= "cheetah-sdhci",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(cheetah_hsmmc_resource),
	.resource	= cheetah_hsmmc_resource,
	.dev		= {
		.dma_mask		= &cheetah_device_hsmmc_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
	},
};
#endif

#ifdef CONFIG_CHEETAH_MMCSPI
struct spi_gpio_platform_data cheetah_spi_gpio_platform_data = {
    .sck = CONFIG_CHEETAH_SPI_SCK_GPIO_NUM,
    .mosi = CONFIG_CHEETAH_SPI_MOSI_GPIO_NUM,
    .miso = CONFIG_CHEETAH_SPI_MISO_GPIO_NUM,
    .num_chipselect = 1,
};
static struct platform_device cheetah_spi_gpio_device = {
    .name   = "spi_gpio",
#ifdef CONFIG_CHEETAH_GSPI
    .id     = 2,
#else
    .id     = 1,
#endif
    .dev    = {
        .platform_data  = &cheetah_spi_gpio_platform_data,
    },
};

static int mmc_spi_get_cd(struct device *dev)
{
#ifdef CONFIG_CHEETAH_SDHCI
extern int sdhci_mmc_spi_get_cd(void);
    if (!gpio_get_value(CONFIG_CHEETAH_SPI_CD_GPIO_NUM))
        return sdhci_mmc_spi_get_cd();
    else
        return 0;
#else
    return !gpio_get_value(CONFIG_CHEETAH_SPI_CD_GPIO_NUM);
#endif
}
static int mmc_spi_get_irq(struct device *dev)
{
    return !gpio_get_value(CONFIG_CHEETAH_SPI_IRQ_GPIO_NUM);
}
typedef irqreturn_t (*my_detect_int)(int, void *);
static my_detect_int mmc_spi_int = NULL;
static struct task_struct *mmc_spi_task = NULL;
static int mmc_spi_kthread(void *data)
{
    struct mmc_host *mmc = (struct mmc_host *)data;
    unsigned long t = 100;
    unsigned long count = 1000;
    int cd = 1;
    int irq = 1;
    while (!kthread_should_stop()) {
        msleep_interruptible(t);
#ifdef CONFIG_CHEETAH_SDHCI
        if (GPREG(PINMUX) & EN_SDIO_FNC)
            continue;
#endif
        if (mmc_spi_int) {
            if (mmc_spi_get_cd(mmc->parent) == cd) {
                mmc_spi_int(0, mmc);
                cd ^= 1;
            }
        }
        if (mmc->caps & MMC_CAP_SDIO_IRQ) {
            struct mmc_card *card = mmc->card;
            if (card && (mmc_spi_get_irq(mmc->parent) == irq)) {
                if (irq) {
                    struct sdio_func *func = card->sdio_single_irq;
                    mmc->sdio_irq_pending = true;
                    if (func && mmc->sdio_irq_pending) {
                        func->irq_handler(func);
                    }
                    mmc->sdio_irq_pending = false;
                }
                irq ^= 1;
            }
        }
        else {
            if (count++ > 600) {
                count = 0;
                printk("SPI Host is Polling Mode!\n");
            }
        }
    }
    return 0;
}
static int mmc_spi_init(struct device *dev,
	irqreturn_t (*detect_int)(int, void *), void *data)
{
    struct mmc_host *mmc = (struct mmc_host *)data;
    if (mmc->caps & MMC_CAP_SDIO_IRQ) {
    	gpio_direction_input(CONFIG_CHEETAH_SPI_IRQ_GPIO_NUM);
    }
    if (detect_int && !mmc_spi_task) {
        mmc_spi_int = detect_int;
        mmc_spi_task = kthread_run(mmc_spi_kthread, data, "mmc_spi_kthread");
    }
    return 0;
}

static struct mmc_spi_platform_data cheetah_mmc_spi_info = {
    .init   = mmc_spi_init,
    .get_cd = mmc_spi_get_cd,
/*
 * MMC_CAP_NEEDS_POLL has some shortcomings
 *    1. SDIO uses CMD7(select card) to polling, but it can't be used in SPI mode
 *    2. mmc_rescan() always polling Card if mmc_spi_host->caps has MMC_CAP_NEEDS_POLL
 *       it makes we can't switch the function between SDHC and SPI mode dynamically
 *
 *    use kernel thread:mmc_spi_kthread to replace polling
 */ 
//    .caps = MMC_CAP_NEEDS_POLL,
/*
 * MMC_CAP_SDIO_IRQ
 * in sdio_irq_thread(), 
 * We want to allow for SDIO cards to work even on non SDIO
 * aware hosts.  One thing that non SDIO host cannot do is
 * asynchronous notification of pending SDIO card interrupts
 * hence we poll for them in that case.
 *
 * if we are SDIO host, we must support pending SDIO card interrupts
 * so we should add MMC_CAP_SDIO_IRQ
 * if we are not, (SPI is non SDIO aware host)
 * we depend on sdio_irq_thread to polling pending SDIO card interrupt
 * so we shouldn't add MMC_CAP_SDIO_IRQ
 *
 * But for verification, polling causes the additional command transfer, not easy to debug
 * so we pretend that we have SDIO IRQ, don't use polling to detect
 *
 * by nady
 */
    .caps = MMC_CAP_SDIO_IRQ,
    .ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.3V only */
};
static struct spi_board_info cheetah_spi_board_info[] = {
    {
        .modalias        = "mmc_spi",
        .platform_data   = &cheetah_mmc_spi_info,
        .max_speed_hz    = 2 * 1000 * 1000,
        .mode            = SPI_MODE_0,
        .controller_data = (void *)CONFIG_CHEETAH_SPI_CS_GPIO_NUM,
        .chip_select     = 0,
#ifdef CONFIG_CHEETAH_GSPI
        .bus_num         = 2,
#else
        .bus_num         = 1,
#endif
    },
};
#endif

#if defined(CONFIG_MONTAGE_I2C)
static struct platform_device mt_i2c_device = {
	.name = "mt-i2c",
	.id   = 0,
	.dev  = {
	}
};
#endif

#if defined(CONFIG_CHEETAH_SND) || defined(CONFIG_CHEETAH_SND_MODULE)
#if 1
static struct i2c_gpio_platform_data gpio_i2c_bus_data = {
	.sda_pin = CONFIG_SND_I2C_SDA,
	.scl_pin = CONFIG_SND_I2C_SCL,
	.udelay  = 20,
	.timeout = 100,
	.scl_is_output_only = 1,
};

static struct platform_device cheetah_device_gpio_i2c = {
	.name = "i2c-gpio",
	.id   = 0,
	.dev  = {
		.platform_data = &gpio_i2c_bus_data,
	}
};
#endif

static struct resource cheetah_i2s_resource_0[] = {
	{
//#ifdef CONFIG_CHEETAH_SND_PCM0
		.start	= AI0_BASE,
		.end	= AI0_BASE + 0x52,
		.flags  = IORESOURCE_MEM,
	}, 
	{
		.start  = IRQ_PCM,
		.end    = IRQ_PCM,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource cheetah_i2s_resource_1[] = {
	{
		.start	= AI1_BASE,
		.end	= AI1_BASE + 0x52,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_PCM,
		.end	= IRQ_PCM,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device cheetah_device_i2s_0 = {
	.name		= "cta-i2s",
	.id		= 0,
	.resource	= cheetah_i2s_resource_0,
	.num_resources 	= ARRAY_SIZE(cheetah_i2s_resource_0),
};

struct platform_device cheetah_device_i2s_1 = {
	.name 		= "cta-i2s",
	.id		= 1,
	.resource	= cheetah_i2s_resource_1,
	.num_resources	= ARRAY_SIZE(cheetah_i2s_resource_1),
};

struct platform_device cheetah_device_asoc = {
	.name       = "cta-pcm-audio",
	.id         = -1,
};
#endif

#ifdef CONFIG_CHEETAH_GI2C
#if defined(CONFIG_CHEETAH_SND) || defined(CONFIG_CHEETAH_SND_MODULE)
    #define CHEETAH_GI2C_FIRST_ID 1
#else
    #define CHEETAH_GI2C_FIRST_ID 0
#endif
static struct i2c_gpio_platform_data gi2c_bus_data = {
	.sda_pin = CONFIG_CHEETAH_GI2C_SDA_GPIO_NUM,
	.scl_pin = CONFIG_CHEETAH_GI2C_SCL_GPIO_NUM,
	.udelay  = 20,
	.timeout = 100,
	.scl_is_output_only = 1,
};
struct platform_device cheetah_device_gi2c = {
	.name = "i2c-gpio",
	.id   = CHEETAH_GI2C_FIRST_ID,
	.dev  = {
		.platform_data = &gi2c_bus_data,
	}
};
#endif

struct platform_device cheetah_device_gpio = {
	.name       = "cta-gpio",
	.id         = -1,
};

static const uint32_t cta_keymap[] = {
	/*row,col,keynum*/
	KEY(2, 0, 1),
	KEY(2, 1, 2),
	KEY(2, 2, 3),
	KEY(1, 0, 4),
	KEY(1, 1, 5),
	KEY(1, 2, 6),
	KEY(0, 0, 7),
	KEY(0, 1, 8),
	KEY(0, 2, 9),
};

static struct cta_keymap_data keymap_data = {
	.keymap			= cta_keymap,
	.keymap_size	= ARRAY_SIZE(cta_keymap),
};

static const int cta_row_gpios[] =
		{13, 23, 22};
static const int cta_col_gpios[] =
		{19, 20, 21};

static struct cta_keypad_platform_data cta_pdata = {
	.keymap_data		= &keymap_data,
	.row_gpios			= cta_row_gpios,
	.col_gpios			= cta_col_gpios,
	.num_row_gpios		= ARRAY_SIZE(cta_row_gpios),
	.num_col_gpios		= ARRAY_SIZE(cta_col_gpios),
	.col_scan_delay_us	= 10,
	.active_low			= 1,
	.wakeup				= 1,
};

struct platform_device cheetah_matrix_keypad = {
	.name       = "cta-gpio-keypad",
	.id         = -1,
	.dev  = {
		.platform_data = &cta_pdata,
	}
};

static const uint32_t cta_ana_keymap[] = {
	/*key, no-used, adc*/
	KEY(1, 0, 3),
	KEY(2, 0, 6),
	KEY(3, 0, 9),
	KEY(4, 0, 12),
	KEY(5, 0, 18),
	KEY(6, 0, 20),
	KEY(6, 0, 21),
	KEY(7, 0, 24),
	KEY(8, 0, 28),
	//KEY(8, 0, 21),
	//KEY(9, 0, 24),
	//KEY(10, 0, 25),
	//KEY(11, 0, 27), 
}; 
 
static struct cta_keymap_data keymap_ana_data = {
	.keymap			= cta_ana_keymap,
	.keymap_size	= ARRAY_SIZE(cta_ana_keymap),
};

static struct cta_ana_keypad_platform_data cta_ana_pdata = {
	.keymap_data		= &keymap_ana_data,
	.wakeup				= 1,
};

struct platform_device cheetah_ana_keypad = {
	.name       = "cta-ana-keypad",
	.id         = -1,
	.dev  = {
		.platform_data = &cta_ana_pdata,
	}
};

struct gpio_keys_button my_all_buttons[] = {
	{
		.gpio = -1,
		.active_low = 1,
		.desc = "",
		.type = EV_KEY,
		.code = BTN_0,
	},
	{
		.gpio = -1,
		.active_low = 1,
		.desc = "",
		.type = EV_KEY,
		.code = BTN_1,
	},
	{
		.gpio = -1,
		.active_low = 1,
		.desc = "",
		.type = EV_KEY,
		.code = BTN_2,
	},
	{
		.gpio = -1,
		.active_low = 1,
		.desc = "",
		.type = EV_KEY,
		.code = BTN_3,
	},
	{
		.gpio = -1,
		.active_low = 1,
		.desc = "",
		.type = EV_KEY,
		.code = BTN_4,
	},
	{
		.gpio = -1,
		.active_low = 1,
		.desc = "",
		.type = EV_KEY,
		.code = BTN_5,
	},
};

static struct gpio_keys_platform_data cheetah_device_gpio_buttons_data = {
	.buttons = my_all_buttons,
	.nbuttons = sizeof(my_all_buttons)/sizeof(my_all_buttons[0]),
	.poll_interval = 10,
};

struct platform_device cheetah_device_gpio_buttons = {
	.name       = "cta-gpio-buttons",
	.id         = -1,
	.dev  = {
		.platform_data = &cheetah_device_gpio_buttons_data,
	}
};

#ifdef CONFIG_LEDS_GPIO
#define CHEETAH_LED_GPIO(n) \
{ \
	.name           = "cheetah_gpio"#n, \
	.gpio           = n, \
	.active_low     = 1, \
},
	
static struct gpio_led cheetah_led_gpio_pins[] = {
#ifdef CONFIG_CHEETAH_LEDGPIO_00
CHEETAH_LED_GPIO(0)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_01
CHEETAH_LED_GPIO(1)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_02
CHEETAH_LED_GPIO(2)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_03
CHEETAH_LED_GPIO(3)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_04
CHEETAH_LED_GPIO(4)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_05
CHEETAH_LED_GPIO(5)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_06
CHEETAH_LED_GPIO(6)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_07
CHEETAH_LED_GPIO(7)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_08
CHEETAH_LED_GPIO(8)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_09
CHEETAH_LED_GPIO(9)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_10
CHEETAH_LED_GPIO(10)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_11
CHEETAH_LED_GPIO(11)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_12
CHEETAH_LED_GPIO(12)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_13
CHEETAH_LED_GPIO(13)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_14
CHEETAH_LED_GPIO(14)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_15
CHEETAH_LED_GPIO(15)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_16
CHEETAH_LED_GPIO(16)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_17
CHEETAH_LED_GPIO(17)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_18
CHEETAH_LED_GPIO(18)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_19
CHEETAH_LED_GPIO(19)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_20
CHEETAH_LED_GPIO(20)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_21
CHEETAH_LED_GPIO(21)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_22
CHEETAH_LED_GPIO(22)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_23
CHEETAH_LED_GPIO(23)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_24
CHEETAH_LED_GPIO(24)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_25
CHEETAH_LED_GPIO(25)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_26
CHEETAH_LED_GPIO(26)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_27
CHEETAH_LED_GPIO(27)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_28
CHEETAH_LED_GPIO(28)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_29
CHEETAH_LED_GPIO(29)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_30
CHEETAH_LED_GPIO(30)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_31
CHEETAH_LED_GPIO(31)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_32
CHEETAH_LED_GPIO(32)
#endif
#ifdef CONFIG_CHEETAH_LEDGPIO_33
CHEETAH_LED_GPIO(33)
#endif
};

static struct gpio_led_platform_data cheetah_led_gpio_data = {
	.num_leds = ARRAY_SIZE(cheetah_led_gpio_pins),
	.leds     = cheetah_led_gpio_pins,
};

static struct platform_device cheetah_led_gpio_device = {
	.name               = "leds-gpio",
	.id                 = -1,
	.dev.platform_data  = &cheetah_led_gpio_data,
};
#endif

#ifdef CONFIG_CHEETAH_GPIO_ROTARY_ENCODER
#include <linux/rotary_encoder.h>

static struct rotary_encoder_platform_data my_rotary_encoder_info = {
	.steps		= 24,
	.axis		= REL_HWHEEL,
	.relative_axis	= true,
	.rollover	= false,
	.gpio_a		= CONFIG_CHEETAH_ROTARY_ENCODER_GPIO_A,
	.gpio_b		= CONFIG_CHEETAH_ROTARY_ENCODER_GPIO_B,
	.inverted_a	= 0,
	.inverted_b	= 0,
};

static struct platform_device cheetah_rotary_encoder_device = {
	.name		= "cheetah-rotary-encoder",
	.id		= 0,
	.dev		= {
		.platform_data = &my_rotary_encoder_info,
	}
};
#endif

static struct platform_device *cheetah_platform_devices[] __initdata = {

#if defined(CONFIG_MONTAGE_I2C)
	&mt_i2c_device,
#endif

#if defined(CONFIG_CHEETAH_USB_HOST_ROLE)
	&cheetah_usb_ehci_device,
#endif

#if defined(CONFIG_CHEETAH_USB_DEVICE_ROLE)
	&cheetah_usb_gdt_device,
#endif

#if defined(CONFIG_CHEETAH_USB_DUAL_ROLE)
	&cheetah_usb_vbus_device,
#endif

#if defined(CONFIG_CHEETAH_USB1_HOST_ROLE)
	&cheetah_usb1_ehci_device,
#endif

#if defined(CONFIG_CHEETAH_USB1_DEVICE_ROLE)
	&cheetah_usb1_gdt_device,
#endif

#ifdef CONFIG_CHEETAH_SDHCI
	&cheetah_device_hsmmc0,
#endif

#ifdef CONFIG_CHEETAH_MMCSPI
	&cheetah_spi_gpio_device,
#endif

#if defined(CONFIG_CHEETAH_SND) || defined(CONFIG_CHEETAH_SND_MODULE)
	&cheetah_device_gpio_i2c,
#if defined(CONFIG_CHEETAH_SND_PCM0)
	&cheetah_device_i2s_0,
#endif
#if defined(CONFIG_CHEETAH_SND_PCM1)
	&cheetah_device_i2s_1,
#endif
	&cheetah_device_asoc,
#endif
#ifdef CONFIG_CHEETAH_GI2C
	&cheetah_device_gi2c,
#endif
	&cheetah_device_gpio,
	&cheetah_device_gpio_buttons,
#if defined(CONFIG_CHEETAH_KEYPAD)
	&cheetah_matrix_keypad,
#endif
#if defined(CONFIG_CHEETAH_R2R_KEYPAD)
	&cheetah_ana_keypad,
#endif
#ifdef CONFIG_LEDS_GPIO
	&cheetah_led_gpio_device,
#endif
#ifdef CONFIG_CHEETAH_GPIO_ROTARY_ENCODER
	&cheetah_rotary_encoder_device,
#endif
};
static int __init cheetah_platform_init(void)
{
#ifdef CONFIG_CHEETAH_MMCSPI
    /* disable PINMUX mdio function */
    GPREG(PINMUX) &= ~(EN_MDIO_FNC);
    /* disable PINMUX eth0 function */
    GPREG(PINMUX) &= ~(EN_ETH0_FNC);
    /* disable PINMUX sdio function */
    GPREG(PINMUX) &= ~(EN_SDIO_FNC);
    spi_register_board_info(cheetah_spi_board_info,
                ARRAY_SIZE(cheetah_spi_board_info));
#endif
	return platform_add_devices(cheetah_platform_devices,
				    ARRAY_SIZE(cheetah_platform_devices));
}

arch_initcall(cheetah_platform_init);
