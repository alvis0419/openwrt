/*=============================================================================+
|                                                                              |
| Copyright 2016                                                               |
| Montage Technology, Inc. All rights reserved.                                |
|                                                                              |
+=============================================================================*/
/*!
 *   \file pdma_driver.h
 *   \brief Peripheral DMA API
 *   \author Montage
 */
 
#ifndef __PDMA_H__
#define __PDMA_H__

/*=============================================================================+
| Included Files                                                               |
+=============================================================================*/
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/sched.h>	/* for completion */
#include <linux/mutex.h>

/*=============================================================================+
| Define                                                                       |
+=============================================================================*/
#define UNCACHED_ADDR(x) (((u32) (x) & 0x1fffffffUL) | 0xa0000000UL)    // mapping physical memory
#define PHYSICAL_ADDR(x) ((u32) (x) & 0x1fffffffUL)     // mapping physical memory

#define PDMA_INTERRUPT

#define PDMA_BASE_ADDR      0xBF002C00
#define PDMAREG(offset)     *((volatile u32 *)(PDMA_BASE_ADDR + offset))

#define PDMA_DESCR_ALIGN_SIZE   0x20

#define PDMA_POOLING_COUNT 1000

#define PDMA_AHB_BUS   (0 << 1)
#define PDMA_AXI_BUS   (1 << 1)
#define PDMA_KEEP_ADDR (0 << 0)
#define PDMA_INC_ADDR  (1 << 0)

#define PDMA_TOTAL_LEN_SHIFT  (16)
#define PDMA_AES_CTRL_SHIFT   (12)
#define PDMA_ENDIAN_SHIFT     (11)
#define PDMA_INTR_SHIFT       (10)
#define PDMA_SRC_SHFT         (8)
#define PDMA_DEST_SHFT        (6)
#define PDMA_DEVICE_SIZE_SHFT (1)
#define PDMA_VALID     1

#define PDMA_CH_UART0_TX 0
#define PDMA_CH_UART0_RX 1
#define PDMA_CH_UART1_TX 2
#define PDMA_CH_UART1_RX 3
#define PDMA_CH_UART2_TX 4
#define PDMA_CH_UART2_RX 5
#define PDMA_CH_SPI_TX   6
#define PDMA_CH_SPI_RX   7

#define PDMA_UART_TX_CHANNEL(uart_idx)  ( 2 * (uart_idx) )
#define PDMA_UART_RX_CHANNEL(uart_idx)  ( 1 + ( 2 * (uart_idx) ))
#define PDMA_CHANNEL_TO_UART_IDX(channel)   ( (channel) / 2 )

#define MAX_CHANNEL_NUM 10

// aboue ch_enable
#define CH0_UART0_ENABLE  0x00000003 // for UART0 write
#define CH1_UART0_ENABLE  0x0000000c // for UART0 read
#define CH2_UART1_ENABLE  0x00000030 // for UART1 write
#define CH3_UART1_ENABLE  0x000000c0 // for UART1 read
#define CH4_UART2_ENABLE  0x00000300 // for UART2 write
#define CH5_UART2_ENABLE  0x00000c00 // for UART2 read
#define CH6_PDMA_ENABLE   0x00003000 // for PDMA write
#define CH7_PDMA_ENABLE   0x0000c000 // for PDMA read
#define CH8_EMPTY_ENABLE  0x00030000 // no use now
#define CH9_EMPTY_ENABLE  0x000c0000 // no use now

#define PDMA_ENABLE            0x00
#define PDMA_CTRL              0x04
#define PDMA_INTR_ENABLE       0x08
#define PDMA_INTR_STATUS       0x0c
#define PDMA_CHANNEL_PRIORITY  0x10

#define LDMA_CH0_DESCR_BADDR   0x14
#define LDMA_CH0_CURRENT_ADDR  0x18
#define LDMA_CH0_STATUS        0x1c
                                  
#define LDMA_CH1_DESCR_BADDR   0x20
#define LDMA_CH1_CURRENT_ADDR  0x24
#define LDMA_CH1_STATUS        0x28
                                  
#define LDMA_CH2_DESCR_BADDR   0x2c
#define LDMA_CH2_CURRENT_ADDR  0x30
#define LDMA_CH2_STATUS        0x34
                                  
#define LDMA_CH3_DESCR_BADDR   0x38
#define LDMA_CH3_CURRENT_ADDR  0x3c
#define LDMA_CH3_STATUS        0x40
                                  
#define LDMA_CH4_DESCR_BADDR   0x44
#define LDMA_CH4_CURRENT_ADDR  0x48
#define LDMA_CH4_STATUS        0x4c
                                                          
#define LDMA_CH5_DESCR_BADDR   0x50
#define LDMA_CH5_CURRENT_ADDR  0x54
#define LDMA_CH5_STATUS        0x58
                                                          
#define LDMA_CH6_DESCR_BADDR   0x5c
#define LDMA_CH6_CURRENT_ADDR  0x60
#define LDMA_CH6_STATUS        0x64
                                                          
#define LDMA_CH7_DESCR_BADDR   0x68
#define LDMA_CH7_CURRENT_ADDR  0x6c
#define LDMA_CH7_STATUS        0x70
                                  
#define LDMA_CH8_DESCR_BADDR   0x74
#define LDMA_CH8_CURRENT_ADDR  0x78
#define LDMA_CH8_STATUS        0x7c
                                                          
#define LDMA_CH9_DESCR_BADDR   0x80
#define LDMA_CH9_CURRENT_ADDR  0x84
#define LDMA_CH9_STATUS        0x88

#define PDMA_DEBUG0            0x8c
#define PDMA_DEBUG1            0x90
#define LDMA_DEBUG_OUT_LOW     0x94
#define LDMA_DEBUG_OUT_HIGH    0x98

// about Pin-strapping setting
#define PIN_STRAP_REG_ADDR  0xbf004828UL

#define CH6_SLV_ADDR  0xbf002000   // SF Write
#define CH7_SLV_ADDR  0xbf002000   // SF Read
#define PDMA_AES_CTRL          0x9C
    #define PDMA_AES_REVERSE   0x0001
    #define PDMA_AES_MODE      0x001E
        #define PDMA_AES_MODE_ECB    0x0000
        #define PDMA_AES_MODE_CBC    0x0002
    #define PDMA_AES_KEYLEN    0x0060
        #define PDMA_AES_KEYLEN_128  0x0000
        #define PDMA_AES_KEYLEN_192  0x0020
        #define PDMA_AES_KEYLEN_256  0x0040

#define PDMA_AES_CTRL2         0xA0
    #define PDMA_AES_OTPKEY_ENC_DISABLE 0x00000001

#define PDMA_AES_KEY           0xA4    // 8 x 32-bits registers

#define AES_ENABLE      (0x01 << 12)
#define AES_OP_ENCRYPT  0x00
#define AES_OP_DECRYPT  (0x01 << 13)
#define AES_KEYSEL_OTP  0x00
#define AES_KEYSEL_REG  (0x01 << 14)

#define DESC_TYPE_BASE 0
#define DESC_TYPE_CHILD 1

// about secure boot
#define DISABLE_SECURE 0
#define ENABLE_SECURE_REG  1
#define ENABLE_SECURE_OTP  2
#define ACTION_WRITE 0
#define ACTION_READ  1

/*
 * sleep until interrupted, then recover and analyse the SR saved by handler
 */
typedef int (* compare_func)(unsigned test, unsigned mask);
/* returns 1 on correct comparison */

typedef void (*pdma_callback) (u32 channel, u32 intr_status);
typedef struct
{
    u32 channel;
    u32 desc_addr;
    u32 next_addr;
    u32 src_addr;
    u32 dest_addr;
    u32 dma_total_len;
    u32 aes_ctrl;
    u32 endian;
    u32 intr_enable;
    u32 src;
    u32 dest;
    u32 fifo_size;
    u32 valid;
} pdma_descriptor;

struct pdma_intr_data
{
    wait_queue_head_t waitq;
    spinlock_t lock;
    u32 ch_received;
};

struct pdma_ch_descr
{
    volatile u32 desc_config;
    u32 dest_addr;
    u32 src_addr;
    u32 next_addr;
};

/*=============================================================================+
| Function Prototypes                                                          |
+=============================================================================*/
void pdma_pooling_wait(void);
    
int pdma_init(void);

void pdma_kick_channel(u32 channel);

void pdma_desc_set(pdma_descriptor *descriptor);

void pdma_callback_register(u32 channel, pdma_callback callback);

void pdma_callback_unregister(u32 channel);

void pdma_program_aes_key(unsigned char *key, int length);

int translate_pdma_status(u32 channel, u32 intr_status);

int any_bits_set(unsigned test, unsigned mask);

u32 get_channel_data_len(u32 channel);

void pdma_loop_desc_set(pdma_descriptor *p_desc, struct pdma_ch_descr *p_ch_desc, int is_base_desc);

u32 get_desc_data_len(struct pdma_ch_descr *p_desc);

void restart_loop_desc(struct pdma_ch_descr *p_desc);

#endif             // __PDMA_DRIVER_H__
