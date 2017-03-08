#
#	Copyright (c) 2013, 2017	Montage Inc.	All rights reserved. 
#
#EXTRA_CFLAGS += -Werror
#NOSTDINC_FLAGS += -I$(src)/../wla/include/
#obj-$(CONFIG_CHEETAH) += init.o setup.o reset.o prom.o time.o int.o serial.o platform.o register.o gpio.o sram.o
#obj-$(CONFIG_CHEETAH) += init.o setup.o reset.o prom.o time.o int.o uart_pdma.o platform.o register.o gpio.o sram.o otp.o
obj-$(CONFIG_CHEETAH) += init.o setup.o reset.o prom.o time.o int.o uart_pdma.o platform.o
#pwr.o cta_gpio_hotplug.o clk.o mactbl.o

obj-$(CONFIG_PM) += pm.o

#obj-$(CONFIG_CHEETAH_INTERNAL_DEBUGGER)	+= idb.o

obj-$(CONFIG_CHEETAH_GSPI) += gspi_dev.o cta_gspi.o gspi.o 

obj-$(CONFIG_CHEETAH_BOOTLOADER_CDB) += boot_cdb.o

obj-$(CONFIG_PANTHER_PDMA) += pdma.o

obj-$(CONFIG_CHEETAH_GDMA) += gdma.o
#obj-$(CONFIG_CHEETAH_GDMA) += gdma.o gdma_linux.o

obj-$(CONFIG_CHEETAH_GVCOM) += gpiovcom.o

obj-$(CONFIG_CHEETAH_GI2C) += gi2c.o

obj-$(CONFIG_CHEETAH_KEYPAD) += cta_keypad.o

obj-$(CONFIG_CHEETAH_ANA_KEYPAD) += cta_ana_keypad.o

obj-$(CONFIG_CHEETAH_ST7565P_LCD) += st7565p_lcd.o

obj-$(CONFIG_CHEETAH_ST7588T_LCD) += st7588t_lcd.o

obj-$(CONFIG_CHEETAH_MADC) += cta_madc.o

obj-$(CONFIG_CHEETAH_PWM) += cta_pwm.o

obj-$(CONFIG_CHEETAH_GPIO_ROTARY_ENCODER) += rotary_encoder.o

#obj-$(CONFIG_MTD_CHEETAH) += cta_flash.o

obj-$(CONFIG_LEDS_TRIGGER_CTAINTF) += ledtrig-ctaintf.o

#obj-$(CONFIG_MONTAGE_I2C) += i2c-mt.o

#obj-$(CONFIG_MONTAGE_WATCHDOG) += watchdog.o

#obj-$(CONFIG_EARLY_PRINTK)	+=

#EXTRA_CFLAGS += -Werror
#
# 	Uncomment to use Switch Port0 as WAN port
#
#EXTRA_CFLAGS += -DCML_WAN_P0

