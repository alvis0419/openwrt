#ifndef __MT_PMU_H__
#define __MT_PMU_H__


#define PMU_WATCHDOG             (PMU_BASE + 0x0020)

#define PMU_RESET_REG24          (PMU_BASE + 0x0260)
#define PMU_RESET_REG25          (PMU_BASE + 0x0264)

#define DEVICE_ID_GPIO          2431
#define DEVICE_ID_PWM           2430
#define DEVICE_ID_OTP           2429
#define DEVICE_ID_PDMA          2428
#define DEVICE_ID_UART2         2427
#define DEVICE_ID_UART1         2426
#define DEVICE_ID_UART0         2425
#define DEVICE_ID_SMI           2424
#define DEVICE_ID_TIMER         2423
#define DEVICE_ID_DISPLAY       2422
#define DEVICE_ID_GRAPHIC       2421
#define DEVICE_ID_DDR           2420
#define DEVICE_ID_USBOTG        2419
#define DEVICE_ID_USB           2418
#define DEVICE_ID_SDIO          2417
#define DEVICE_ID_AES           2416
#define DEVICE_ID_TSI           2415
#define DEVICE_ID_GSPI          2414
#define DEVICE_ID_HNAT          2413
#define DEVICE_ID_SWP2          2412
#define DEVICE_ID_SWP1          2411
#define DEVICE_ID_SWP0          2410
#define DEVICE_ID_SPDIF         2409
#define DEVICE_ID_PCM           2408
#define DEVICE_ID_RTC           2407
#define DEVICE_ID_BBP           2406
#define DEVICE_ID_GDMA          2405
#define DEVICE_ID_SRAM_CTRL     2404
#define DEVICE_ID_PBUS          2403
#define DEVICE_ID_RBUS          2402
#define DEVICE_ID_DBUS          2401
#define DEVICE_ID_CPU           2400
#define DEVICE_ID_WIFIMAC       2531
#define DEVICE_ID_SWP1_PORT     2530
#define DEVICE_ID_SWP0_PORT     2529

void pmu_reset_devices(unsigned long *device_ids);
void pmu_reset_devices_no_spinlock(unsigned long *device_ids);

#endif

