/* 
 *  Copyright (c) 2008, 2009    Acrospeed Inc.    All rights reserved.
 */
#ifndef __ASM_CHEETAH_GPIO_H
#define __ASM_CHEETAH_GPIO_H

#include <linux/kernel.h>

#define CHEETAH_GPIO_NUMS       30      /* GPIO[0~29] */
#define CHEETAH_GPIO_LINES		64		/* GPIO[0~63] */

#if defined(CONFIG_CHEETAH_SND) || defined(CONFIG_CHEETAH_SND_MODULE)
#ifdef CONFIG_CHEETAH_SND_DEDICATED_I2C
#if CONFIG_CHEETAH_SND_SCL_GPIO_NUM == CONFIG_CHEETAH_SND_SDA_GPIO_NUM
#error "SND SCL/SDA GPIO NUM are the same!!"
#endif
#if CONFIG_CHEETAH_SND_SCL_GPIO_NUM < 0 || CONFIG_CHEETAH_SND_SCL_GPIO_NUM > CHEETAH_GPIO_NUMS - 1
#error "SND SCL GPIO NUM is out of range!!"
#endif
#if CONFIG_CHEETAH_SND_SDA_GPIO_NUM < 0 || CONFIG_CHEETAH_SND_SDA_GPIO_NUM > CHEETAH_GPIO_NUMS - 1
#error "GI2C SDA GPIO NUM is out of range!!"
#endif

#define CONFIG_SND_I2C_SDA CONFIG_CHEETAH_SND_SDA_GPIO_NUM
#define CONFIG_SND_I2C_SCL CONFIG_CHEETAH_SND_SCL_GPIO_NUM
#else
#define CONFIG_SND_I2C_SDA (CHEETAH_GPIO_LINES-1)
#define CONFIG_SND_I2C_SCL (CHEETAH_GPIO_LINES-2)
#endif

#define CONFIG_SND_I2C_SDA_SDR 33
#define CONFIG_SND_I2C_SCL_SDR 32
#define CONFIG_SND_I2C_SDA_DDR 31
#define CONFIG_SND_I2C_SCL_DDR 30

struct i2c_core {
    u32 sda;
    u32 scl;
};
#endif

struct gpio_info {
    u8  idx;
    u8  dir;
    u8  sel;
    u8  res;
    u32 enable;
    u32 disable;
};

#ifdef CONFIG_GPIOLIB
extern int camelot_gpio_to_irq(unsigned gpio);

static inline int gpio_to_irq(unsigned gpio)
{
    return camelot_gpio_to_irq(gpio);
}
#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep
#else
extern int camelot_gpio_to_irq(unsigned gpio);
extern int camelot_gpio_get_value(unsigned gpio);
extern void camelot_gpio_set_value(unsigned gpio, int value);
extern int camelot_gpio_direction_input(unsigned gpio);
extern int camelot_gpio_direction_output(unsigned gpio, int value);

static inline int gpio_request(unsigned gpio, const char *label)
{
    return 0;
}
static inline void gpio_free(unsigned gpio)
{
}
static inline int gpio_to_irq(unsigned gpio)
{
	return camelot_gpio_to_irq(gpio);
}
static inline int gpio_get_value(unsigned gpio)
{
	return camelot_gpio_get_value(gpio);
}
static inline void gpio_set_value(unsigned gpio, int value)
{
	camelot_gpio_set_value(gpio, value);
}
static inline int gpio_direction_input(unsigned gpio)
{
	return camelot_gpio_direction_input(gpio);
}

static inline int gpio_direction_output(unsigned gpio, int value)
{
	return camelot_gpio_direction_output(gpio, value);
}
#endif
/* cansleep wrappers */
#include <asm-generic/gpio.h>

#endif /* __ASM_CHEETAH_GPIO_H */
