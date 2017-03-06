/* 
 *	Cheetah
 *
 */

#ifndef __CHEETAH_H__
#define __CHEETAH_H__

#include <asm/addrspace.h>

#ifdef LANGUAGE_C
#define vchar		volatile unsigned char
#define vshort		volatile unsigned short
#define vlong		volatile unsigned long
#define pvlong      *(vlong *)
#define pvchar      *(vchar *)
#else
#define pvlong
#define pvchar
#endif 

#define CHEETAH_VERSION(a,b)    (((a) << 4) + b)
#define CHEETAH_VERSION_CODE    CONFIG_CHEETAH_VERSION_CODE

#define PANTHER_REG_BASE    (0xBF000000UL)
#define CHEETAH_REG_BASE	(0xaf000000)

#define MI_BASE     CHEETAH_REG_BASE
#define	TM_BASE		(PANTHER_REG_BASE + 0x01000)
#define SMI_BASE	(PANTHER_REG_BASE + 0x02200)
#define UR_BASE		(PANTHER_REG_BASE + 0x02900)
#define UMAC_REG_BASE	(MI_BASE+0x3000)
#define	I2C_BASE        (PANTHER_REG_BASE + 0x02400)
#define GPIO_BASE   (PANTHER_REG_BASE + 0x02500)
#define PMU_BASE    (PANTHER_REG_BASE + 0x04800)
#define ANA_BASE    (MI_BASE+0x5800)
#define	AI0_BASE	(0x1F000000UL+0x7000)
#define AI1_BASE	(0x1F000000UL+0x7100)
#define MAC_BASE	(PANTHER_REG_BASE + 0x08800)
#define SDIO_BASE	(0x1F000000UL + 0x0B000)
#define USB_BASE	(PANTHER_REG_BASE + 0x0C000)
#define USB_OTG_BASE (PANTHER_REG_BASE + 0x0D000)

#define SROMC1_BASE     (MI_BASE+0x0020)
#define SROMC2_BASE     (MI_BASE+0x0030)

#define	MR1C		0x00
#define	MR2C		0x04
/* SROMEN register is only applicable for IP3280 */
#define SROMEN      0x08
#define	MSDC		0x10
/* MSR1X & MSR2X registers are only applicable for IP3280 */
#define	MSR1C		0x20
#define	MSR1A		0x24
#define	MSR1D		0x28
#define	MSR2C		0x30
#define	MSR2A		0x34
#define	MSR2D		0x38

#define	T0CR		(0x00/4)	/* current value */
#define	T0LR		(0x04/4)	/* reload value */
#define	T0PR		(0x08/4)	/* prescaler */
#define	T0CN		(0x0c/4)	/* control */
#define T0IS		(0x10/4)	/* interrupt status */
#define	T1CR		(0x14/4)	/* current value */
#define	T1LR		(0x18/4)	/* reload value */
#define	T1PR		(0x1c/4)	/* prescaler */
#define	T1CN		(0x20/4)	/* control */
#define T1IS		(0x24/4)	/* interrupt status */
#define	T2CR		(0x28/4)	/* current value */
#define	T2LR		(0x2c/4)	/* reload value */
#define	T2PR		(0x30/4)	/* prescaler */
#define	T2CN		(0x34/4)	/* control */
#define T2IS		(0x38/4)	/* interrupt status */


#define	ISTS		(0x0>>2)
#define	IMSK		(0x4>>2)
#define	ISR1		(0x8>>2)
#define	ISR2		(0xc>>2)
#define	IGPIOXSEL	(0x10>>2)

#define URBR		0x0		/* tx/rx buffer register */
#define  URBR_DTSHFT      24
#define  URBR_RDY         (1<<23)
#define URCS		0x4		/* control/status */
#define  URCS_RXEN      (1<<31)
#define  URCS_BRSHFT      16
#define  URCS_TIE	(1<<15)
#define  URCS_RIE	(1<<14)
#define  URCS_PE	(1<<13)
#define  URCS_EVEN	(1<<12)
#define  URCS_SP2	(1<<11)
#define  URCS_LB	(1<<10)
#define  URCS_P		(1<<9)
#define  URCS_PER	(1<<8)
#define  URCS_FE	(1<<7)
#define  URCS_RTH_S	(4)		/* rx threshold */
#define  URCS_TF	(1<<3)		/* tx fifo full */
#define  URCS_TE	(1<<2)		/* tx empty */
#define  URCS_RF	(1<<1)		/* rx full */
#define  URCS_TB	(1<<0)		/* tx busy */
#define URCS2		0x8
#define  URCS2_RX_STOP          (1<<13)
#define  URCS2_TX_STOP          (1<<12)
#define  URCS2_RX_FLOW_CTRL_EN  (1<<2)
#define  URCS2_TX_FLOW_CTRL_EN  (1<<1)
#define  URCS2_DMA_EN           (1<<0)

#define GPVAL       (0x00/4)
#define GPSET       (0x04/4)
#define GPCLR       (0x08/4)
#define GPDIR       (0x0c/4)
#define GPSEL       (0x10/4)
#define MISC1       (0x14/4)
#define IO_T        (0x1c/4)
#define DRIVER1     (0x20/4)
#define  DRV_SDIO         (0x0000000CUL)
#define DRIVER2     (0x24/4)
#define TMIICLK     (0x3c/4)
#define GP2VAL      (0x40/4)
#define GP2SET      (0x44/4)
#define GP2CLR      (0x48/4)
#define GP2DIR      (0x4c/4)
#define GP2SEL      (0x50/4)
#define CS3SEL      (0x84/4)
#define GDEBUG      (0x90/4)
#define PINMUX      (0x94/4)
#define  EN_FUNCMODE      (3 << 26)
#define  EN_ADC_OUT_FNC   (1 << 25)
#define  EN_ANA_MON_FNC   (1 << 24)
#define  EN_MDIO_AUX_FNC  (1 << 23)
#define  EN_SWI2C_AUX_FNC (1 << 22)
#define  EN_SWI2C_FNC     (1 << 21)
#define  EN_RF_LEGA_FNC   (1 << 20)
#define  EN_JTAG_AUX_FNC  (1 << 19)
#define  EN_LED_AUX_FNC   (1 << 18)
#define  EN_UART_AUX_FNC  (1 << 17)
#define  EN_I2C_AUX_FNC   (1 << 16)
#define  EN_PWM_AUX_FNC   (1 << 15)
#define  EN_ROM1_FNC      (1 << 14)
#define  EN_JTAG_FNC      (1 << 13)
#define  EN_AGC_FNC       (1 << 12)
#define  EN_PWM_FNC       (1 << 11)
#define  EN_LED_FNC       (1 << 10)
#define  EN_UART_FNC      (1 << 9)
#define  EN_MDIO_FNC      (1 << 8)
#define  EN_PCM1_FNC      (1 << 7)
#define  EN_PCM0_FNC      (1 << 6)
#define  EN_ETH1_FNC      (1 << 5)
#define  EN_ETH0_FNC      (1 << 4)
#define  EN_SDIO_FNC      (1 << 3)
#define  EN_W6_FNC        (1 << 2)
#define  EN_SIP_FNC       (1 << 1)
#define  EN_EXT_SDR_FNC   (1 << 0)
#define SWRST       (0xa0/4)
#define  PAUSE_SDIO       (1 << 13)
#define  SWRST_SDIO       (1 << 5)
#define SDIO        (0xa4/4)
#define  SDIO_ENDIAN_PIN  (4)
#define SRM_LBND    (0xc8/4)
#define SRM_UBND    (0xcc/4)
#define USBMOD      (0xf8/4)
#define MADCSAMP    (0x430/4)
#define PWM         (0x480/4)


/* analog bank register */
#define CLKEN       (0x1C/4)
#define  PCM_CLK_EN  (1<<7)
#define  I2S_CLK_EN  (1<<6)
#define  SD_CLK_EN   (1<<2)
#define CLKDIV      (0x20/4)
#define  CLKDIV_CPUFFST  16
#define  CLKDIV_CPUSHFT  8
#define  CLKDIV_POSTDIV_NUM 8
#define  CPU_CLK_UPDATE  (1<<31)
#define  CPU_CLK_NOGATE  (1<<30)
#define  CPU_CLK_PREDIV  (0x00FF0000UL)
#define  CPU_CLK_POSTDIV (0x07000000UL)
#define  SYS_CLK_UPDATE  (1<<15)
#define  SYS_CLK_NOGATE  (1<<14)
#define  SYS_CLK_PREDIV  (0x000000FFUL)
#define  SYS_CLK_POSTDIV (0x00000700UL)
#define  CLKDIV_POSTDIV_NUM 8
#define CLKDIV2     (0x3C/4)
#define I2S_FREQUENCY_SETTING  0xAF00583CUL
#define  I2S_CLK_UPDATE  (1<<31)
#define  I2S_CLK_NOGATE  (1<<30)
#define  I2S_CLK_PREDIV  (0x000FFF00UL)
#define  I2S_CLK_POSTDIV (0x0000007FUL)
#define CLKDIV3     (0x40/4)
#define PCM_FREQUENCY_SETTING  0xAF005840UL
#define  PCM_CLK_UPDATE  (1<<31)
#define  PCM_CLK_NOGATE  (1<<30)
#define  PCM_CLK_PREDIV  (0x0000FFF0UL)
#define  PCM_CLK_POSTDIV (0x00000007UL)
#define RMIIADDR    (0x50/4)
#define VOLCTRL     (0x60/4)
#define DDR_V1P2_CTR_L (0x78/4)
#define DDR_CTR_L   (0x7c/4)
#define DDR_V1P2_CTR_R (0x88/4)
#define DDR_CTR_R   (0x8c/4)
#define VOLCTRL     (0x60/4)
#define MADCCTL     (0x28/4)

/* USB bank register */
#define OTGSC          (0x0a4/4)
#define  AVV              (1<<9)
#define  AVVIS            (1<<17)
#define  AVVIE            (1<<25)
#define PHY_DIG_CTRL   (0x308/4)
#define  VBUS_DETECTION   (1<<12)
#define  RECOVERY_CLK_INV (1<<4)
#define USB_REG0       (0x320/4)
#define  SQ_DELAY         (1<<31)
#define USB_PHY_PLL    (0x328/4)
#define USB_A0_REG     (0x32c/4)
#define  USB_DOWN         (1<<0)

#define URREG(offset)   (*(volatile unsigned int*)(UR_BASE+offset))
#define GPREG(reg)      __check_gpio_reg_access__() //((volatile unsigned int*)(GPIO_BASE))[reg]
#define ANAREG(reg)	    __check_ana_reg_access__() //((volatile unsigned int*)(ANA_BASE))[reg]
#define USBREG(reg)	    ((volatile unsigned int*)(USB_BASE))[reg]

#define FUNC_MODE_EN    1 << 31
#define EXT_SW_HNAT     1 << 30
#define EXT_SW_WIFI     1 << 29
#define EXT_BB          1 << 28
#define DEBUG_ASIC      1 << 4
#define DEBUG_FPGA      0 << 4

#define DEBUG_WMAC      0
#define DEBUG_HNAT      1
#define DEBUG_BB        2
#define DEBUG_USB       3
#define DEBUG_MI        4

#define SMI_CTRL0       0xb4
#define SMI_CTRL1       0xb8


#define EN_ADC_OUT_FNC   (1 << 25)
#define EN_ANA_MON_FNC   (1 << 24)
#define EN_MDIO_AUX_FNC  (1 << 23)
#define EN_SWI2C_AUX_FNC (1 << 22)
#define EN_SWI2C_FNC     (1 << 21)
#define EN_RF_LEGA_FNC   (1 << 20)
#define EN_JTAG_AUX_FNC  (1 << 19)
#define EN_LED_AUX_FNC   (1 << 18)
#define EN_UART_AUX_FNC  (1 << 17)
#define EN_I2C_AUX_FNC   (1 << 16)
#define EN_PWM_AUX_FNC   (1 << 15)
#define EN_ROM1_FNC      (1 << 14)
#define EN_JTAG_FNC      (1 << 13)
#define EN_AGC_FNC       (1 << 12)
#define EN_PWM_FNC       (1 << 11)
#define EN_LED_FNC       (1 << 10)
#define EN_UART_FNC      (1 << 9)
#define EN_MDIO_FNC      (1 << 8)
#define EN_PCM1_FNC      (1 << 7)
#define EN_PCM0_FNC      (1 << 6)
#define EN_ETH1_FNC      (1 << 5)
#define EN_ETH0_FNC      (1 << 4)
#define EN_SDIO_FNC      (1 << 3)
#define EN_W6_FNC        (1 << 2)
#define EN_SIP_FNC       (1 << 1)
#define EN_EXT_SDR_FNC   (1 << 0)


#ifdef CONFIG_CHEETAH_FPGA
#define	DEFAULT_CPU_CLK		75000000
#define DEFAULT_SYS_CLK     50000000
#define PBUS_CLK            40000000
#else
#define	DEFAULT_CPU_CLK		150000000
#define DEFAULT_SYS_CLK     150000000
#define PBUS_CLK            120000000
#endif

extern unsigned int CPU_CLK;
extern unsigned int SYS_CLK;


#define ANAREG_UPDATE(x, val, mask) do {                            \
    ANAREG((x)) = (( ANAREG((x)) & ~(mask) ) | ( (val) & (mask) )); \
} while(0)


/*---------------------------------------------
    for debugging
 *--------------------------------------------*/
#ifdef LANGUAGE_C
#define     IO_DEBUG_OUT(a)    *((volatile unsigned char *)(0xbfc00000|a)) = 0 
//#define     IO_DEBUG_OUT(a)
#endif //LANGUAGE_C

#ifdef CONFIG_CHEETAH_SDHCI_MODULE
	#ifndef CONFIG_CHEETAH_SDHCI
		#define CONFIG_CHEETAH_SDHCI 1
	#endif
#endif

#ifdef CONFIG_CHEETAH_SND_MODULE
	#ifndef CONFIG_CHEETAH_SND
		#define CONFIG_CHEETAH_SND 1
	#endif
#endif

#endif
