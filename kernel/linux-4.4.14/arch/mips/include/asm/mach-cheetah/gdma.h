#ifndef __ASM_CHEETAH_GDMA_H__
#define __ASM_CHEETAH_GDMA_H__

#include <asm/types.h>
#include <cheetah.h>

#include <asm/irq.h>

#include <linux/spinlock.h>
#include <linux/interrupt.h>


#define UNCACHED_ADDR(x) (((u32) (x) & 0x1fffffffUL) | 0xa0000000UL)
#define PHYSICAL_ADDR(x) ((u32) (x) & 0x1fffffffUL)

#define GDMAREG_READ32(addr)       (*(volatile u32*)((addr)))
#define GDMAREG_WRITE32(addr, val) (*(volatile u32*)((addr)) = (u32)(val))

#define DMADBG(args...)     printk(args)

#define DMAPANIC(args...)   do { panic(args);  while(1); } while(0);

/* enable this define to allow retry if the descriptors allocated are not consecutive */
//#define DO_RETRY_FOR_CONSECUTIVE_ALLOCATION

#define GDMA_DESCR_COUNT    512

/* caution: shall always less than GDMA_DESCR_COUNT; keep this settings unless you know does this threshold means */
#define INSERT_SYNC_DESCR_THRESHOLD     (GDMA_DESCR_COUNT / 2)        

#define DMA_REGBASE         0xBF005500UL

#define DMA_ENABLE	        (DMA_REGBASE	+	0x00)
#define DMA_KICK	        (DMA_REGBASE	+	0x04)

#define DMA_INTR_STATUS	    (DMA_REGBASE	+	0x08)
    #define DMA_INTR_STATUS_COUNT   0x00000001UL
    #define DMA_INTR_STATUS_TIME    0x00000002UL

#define DMA_INTR_MASK	    (DMA_REGBASE + 0x0c)
    #define DMA_INTR_MASK_COUNT	    0x00000001UL
    #define DMA_INTR_MASK_TIME	    0x00000002UL

#define DMA_INTR_THRESHOLD	(DMA_REGBASE	+	0x10)
    #define DMA_INTR_THRESHOLD_TIME	    0xFFFF0000UL
    #define DMA_INTR_THRESHOLD_COUNT	0x0000FFFFUL

#define DMA_DESCR_BADDR	    (DMA_REGBASE + 0x14)

#define DMA_READ_DESCR_ADDR (DMA_REGBASE + 0x18)

typedef struct
{
    union
    {
        struct 
        {
            volatile u16 ctrl:2;
            u16 eor:1;
            volatile u16 sw_inuse:1;	// software only controlled bit (indicate the descr. is used/hold by someone)
            u16     :9;
            u16 cksum_init:1;
            u16 operation:2;
        
            volatile u16 cksum_result;
        
            u32 dest_addr;
            u32 src_addr;
        
            u16 cksum_offset;
            u16 cksum_length;
        
            u16 operation_length;
            u16 cksum_initval;
        
            u32 sw_callback;
            u32 sw_priv;
            u32 sw_priv2;
        };

        struct
        {
            volatile u16 ctrl_word0;

            volatile u16 cksum_result;
        
            u32 dest_addr;
            u32 src_addr;
        
            u16 cksum_offset;
            u16 cksum_length;
        
            u16 operation_length;
            u16 cksum_initval;
        
            u32 sw_callback;
            u32 sw_priv;
            u32 sw_priv2;
        };
    };
} __attribute__((aligned(0x20),__packed__)) dma_descriptor;

#define GDMA_CTRL_GO	        0x3
#define GDMA_CTRL_GO_POLL	    0x2
#define GDMA_CTRL_DONE_SKIP	    0x1
#define GDMA_CTRL_DONE_STOP	    0x0

#define GDMA_OP_COPY	    0x0
#define GDMA_OP_COPY_CSUM	0x1
#define GDMA_OP_CSUM	    0x2

int gdma_get_free_descr(short *descr_idx, int count);
    
#define GDMA_POLL_CW0(descr_index, operation, cksum_init) \
    ((GDMA_CTRL_GO_POLL << 14) | ((cksum_init) << 2) | (operation) \
     | (((descr_index)==(GDMA_DESCR_COUNT-1)) ? (0x01 << 13) : 0x0) | (0x01 << 12))

#define GDMA_INTR_CW0(descr_index, operation, cksum_init) \
    ((GDMA_CTRL_GO << 14) | ((cksum_init) << 2) | (operation) \
     | (((descr_index)==(GDMA_DESCR_COUNT-1)) ? (0x01 << 13) : 0x0) | (0x01 << 12))

static inline void gdma_kick(void)
{
    GDMAREG_WRITE32(DMA_KICK, 1);
}

extern dma_descriptor *pdma_descr;
extern int dma_descr_count;

typedef void (*dma_callback)(void *priv);

#define USE_GDMA_CW_MACRO

#endif // __ASM_CHEETAH_GDMA_H__

