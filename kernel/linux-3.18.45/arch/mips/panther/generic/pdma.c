/*=============================================================================+
|                                                                              |
| Copyright 2016                                                               |
| Montage Technology, Inc. All rights reserved.                                |
|                                                                              |
+=============================================================================*/
/*!
 *   \file pdma_driver.c
 *   \brief Peripheral DMA API
 *   \author Montage
 */

/*=============================================================================+
| Define                                                                       |
+=============================================================================*/   
#define PDMA_DEBUG

#ifdef PDMA_DEBUG
#define sf_log(fmt, args...) printk(fmt, ##args)
#else
#define sf_log(fmt, args...)
#endif

/*=============================================================================+
| Included Files                                                               |
+=============================================================================*/
#include <asm/mach-cheetah/pdma.h>

#ifdef PDMA_INTERRUPT
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#endif

/*=============================================================================+
| Variables                                                                    |
+=============================================================================*/
static u32 is_initialized = 0;

struct pdma_intr_data pdma_data;

struct pdma_ch_descr ch_descr[MAX_CHANNEL_NUM] __attribute__ ((aligned(32)));

u32 little_endian = 0;

pdma_callback intr_callback_table[MAX_CHANNEL_NUM];

u32 ch_enable_table[MAX_CHANNEL_NUM] = {
    CH0_UART0_ENABLE,
    CH1_UART0_ENABLE,
    CH2_UART1_ENABLE,
    CH3_UART1_ENABLE,
    CH4_UART2_ENABLE,
    CH5_UART2_ENABLE,
    CH6_PDMA_ENABLE,
    CH7_PDMA_ENABLE,
    CH8_EMPTY_ENABLE,
    CH9_EMPTY_ENABLE,
};

/*=============================================================================+
| Functions                                                                    |
+=============================================================================*/
void pdma_program_aes_key(unsigned char *key, int length)
{
    // length shall be 16 (128bits), 24 (192bits) or 32 (256bits)
    u32 keyval;
    unsigned char *k = key;
    int i;

    for (i = 0; i < length/4; i++)
    {
        keyval = (k[3] << 24) | (k[2] << 16) | (k[1] << 8) | k[0];

        PDMAREG((PDMA_AES_KEY + i*4)) = keyval;
        k += 4;
    }
}

/* check all error conditions */
int translate_pdma_status(u32 channel, u32 intr_status)
{
    int rc = 0;
    
    if (intr_status == 0)
    {
        // should not happen
        sf_log("ERROR: Get PDMA INTR status is 0 !!!\n");
    }
    else
    {
        // too many message, set default to comment
        // sf_log("Get PDMA INTR status: %x\n", ahb_read_data);
    }

    if ((intr_status >> (channel * 2 + 1)) == 0x1)
    {
        rc = -ETIMEDOUT;
        sf_log("PDMA channel got timeout !!, channel index = %d\n", channel);
    }

    return rc;
}

/*
 * Concrete compare_funcs 
 */
int any_bits_set(unsigned test, unsigned mask)
{
//	    sf_log("[%s:%d] test value: %x, mask : %d\n", __func__, __LINE__, test, mask);
    return(test & mask) != 0;
}


void pdma_pooling_wait(void)
{
    volatile u32 ahb_read_data;
    volatile u8 pdma_rx_status = 0x00;
    volatile u8 pdma_tx_status = 0x00;
    int count = 0;
    
    while ((pdma_rx_status == 0x00) && (pdma_tx_status == 0x00))
    {
        ahb_read_data = PDMAREG(PDMA_INTR_STATUS);
        pdma_rx_status = (ahb_read_data >> PDMA_CH_SPI_RX * 2) & 0x03;
        pdma_tx_status = (ahb_read_data >> PDMA_CH_SPI_TX * 2) & 0x03;

        count++;
        if (count >= PDMA_POOLING_COUNT) {
            sf_log("ahb_read_data = %x\n", ahb_read_data);
            sf_log("pdma_rx_status = %x\n", pdma_rx_status);
            sf_log("pdma_tx_status = %x\n", pdma_tx_status);
//	            sf_log("CH6_DESC_BASE = %x\n", (*(u32 *)ch6_descr));
//	            sf_log("CH7_DESC_BASE = %x\n", (*(u32 *)ch7_descr));
            break;
        }
    }

//	    translate_pdma_status(ahb_read_data, pdma_rx_status, pdma_tx_status);
}

irqreturn_t pdma_intr_handler(int this_irq, void *dev_id)
{
//	    sf_log("PDMA interrupt coming !!!\n");
    int i;
    u32 intr_status;

    intr_status = PDMAREG(PDMA_INTR_STATUS);
    // clear INTR status
    PDMAREG(PDMA_INTR_STATUS) = intr_status;
//	    sf_log("[%s:%d] intr_status %x\n", __func__, __LINE__, intr_status);

    for (i = 0; i < MAX_CHANNEL_NUM; i++)
    {
        if ((intr_status & ch_enable_table[i]))
        {
            if (intr_callback_table[i] != NULL)
            {
                intr_callback_table[i](i, intr_status);
            }
            else
            {
                sf_log("Not have maching callback, disable interrupt at this channel\n");
                pdma_callback_unregister(i);
            }
        }
    }
    
    return IRQ_HANDLED;
}

int pdma_init(void)
{
    int ret;

    if (is_initialized)
    {
        return 0;
    }
    is_initialized = 1;
    
    sf_log("pdma_init()\n");

    if (0 == (*((volatile unsigned long *)PIN_STRAP_REG_ADDR) & 0x01))
    {
        little_endian = 1;
    }
    sf_log("little_endian = 0x%x\n", little_endian);

    // TODO will add hardware reset for PDMA here

    // clear PDMA register
    PDMAREG(PDMA_INTR_ENABLE) = 0;  //ahb_read_data;
    PDMAREG(PDMA_ENABLE) = 0;       //ahb_read_data;
    
    // Set channel descriptors
    PDMAREG(LDMA_CH0_DESCR_BADDR) = PHYSICAL_ADDR(&ch_descr[0]);
    PDMAREG(LDMA_CH1_DESCR_BADDR) = PHYSICAL_ADDR(&ch_descr[1]);
    PDMAREG(LDMA_CH6_DESCR_BADDR) = PHYSICAL_ADDR(&ch_descr[6]);
    PDMAREG(LDMA_CH7_DESCR_BADDR) = PHYSICAL_ADDR(&ch_descr[7]);
	
#ifdef PDMA_INTERRUPT
    ret = request_irq(IRQ_PDMA, pdma_intr_handler, 0, "pdma", (void *) IRQ_PDMA);
#endif

    return ret;
}

void pdma_kick_channel(u32 channel)
{
    PDMAREG(PDMA_CTRL) = (1 << channel);
}

u32 pdma_desc_rdata;
void pdma_desc_set(pdma_descriptor *descriptor)
{
    descriptor->desc_addr = UNCACHED_ADDR(&ch_descr[descriptor->channel]);

    // Set Destination Addr
    (*(u32 *)(descriptor->desc_addr + 4)) = descriptor->dest_addr;

    // Set Source Addr
    (*(u32 *)(descriptor->desc_addr + 8)) = descriptor->src_addr;

    // Set Next descriptor Addr
    (*(u32 *)(descriptor->desc_addr + 12)) = descriptor->next_addr;

    // Set length, size, owner
    (*(u32 *)descriptor->desc_addr) = (little_endian << PDMA_ENDIAN_SHIFT) |
                                      (descriptor->dma_total_len << PDMA_TOTAL_LEN_SHIFT) |
                                      (descriptor->intr_enable << PDMA_INTR_SHIFT) |
                                      (descriptor->src << PDMA_SRC_SHFT) | 
                                      (descriptor->dest << PDMA_DEST_SHFT) |
                                      (descriptor->aes_ctrl) |
                                      (descriptor->fifo_size << PDMA_DEVICE_SIZE_SHFT) |
                                      (descriptor->valid);
    
    pdma_desc_rdata = *((volatile u32 *)descriptor->desc_addr);
    pdma_kick_channel(descriptor->channel);
}

void reset_desc_base_addr(u32 channel, u32 base_addr)
{
    PDMAREG(PDMA_ENABLE) &= ~(1 << channel);

    switch (channel)
    {
        case PDMA_CH_UART0_TX:
            PDMAREG(LDMA_CH0_DESCR_BADDR) = PHYSICAL_ADDR(base_addr);
            break;
        case PDMA_CH_UART0_RX:
            PDMAREG(LDMA_CH1_DESCR_BADDR) = PHYSICAL_ADDR(base_addr); 
            break;
        case PDMA_CH_UART1_TX:
            PDMAREG(LDMA_CH2_DESCR_BADDR) = PHYSICAL_ADDR(base_addr);
            break;
        case PDMA_CH_UART1_RX:
            PDMAREG(LDMA_CH3_DESCR_BADDR) = PHYSICAL_ADDR(base_addr);
            break;
        case PDMA_CH_UART2_TX:
            PDMAREG(LDMA_CH4_DESCR_BADDR) = PHYSICAL_ADDR(base_addr);
            break;
        case PDMA_CH_UART2_RX:
            PDMAREG(LDMA_CH5_DESCR_BADDR) = PHYSICAL_ADDR(base_addr);
            break;
        case PDMA_CH_SPI_TX:
            PDMAREG(LDMA_CH6_DESCR_BADDR) = PHYSICAL_ADDR(base_addr);
            break;
        case PDMA_CH_SPI_RX:
            PDMAREG(LDMA_CH7_DESCR_BADDR) = PHYSICAL_ADDR(base_addr);
            break;
        default:
            break;
    }

    // reset channel to enable new base address
    PDMAREG(PDMA_ENABLE) |= (1 << channel);
}

void pdma_loop_desc_set(pdma_descriptor *p_desc, struct pdma_ch_descr *p_ch_desc, int is_base_desc)
{    
    if (is_base_desc == DESC_TYPE_BASE)
    {
        reset_desc_base_addr(p_desc->channel, p_desc->desc_addr);
    }
    
    // Set Destination Addr
    (*(u32 *)(p_desc->desc_addr + 4)) = p_desc->dest_addr;

    // Set Source Addr
    (*(u32 *)(p_desc->desc_addr + 8)) = p_desc->src_addr;

    // Set Next descriptor Addr
    (*(u32 *)(p_desc->desc_addr + 12)) = p_desc->next_addr;

    // Set length, size, owner
    (*(u32 *)p_desc->desc_addr) = (little_endian << PDMA_ENDIAN_SHIFT) |
                                  (p_desc->dma_total_len << PDMA_TOTAL_LEN_SHIFT) |
                                  (p_desc->intr_enable << PDMA_INTR_SHIFT) |
                                  (p_desc->src << PDMA_SRC_SHFT) | 
                                  (p_desc->dest << PDMA_DEST_SHFT) |
                                  (p_desc->aes_ctrl) |
                                  (p_desc->fifo_size << PDMA_DEVICE_SIZE_SHFT) |
                                  (p_desc->valid);

//	    printk("==================================: %d\n", i);
//	    printk("desc.channel = 0x%x\n", desc.channel);
//	    printk("desc.desc_addr = 0x%x\n", desc.desc_addr);
//	    printk("desc.dest_addr = 0x%x\n", desc.dest_addr);
//	    printk("desc.src_addr = 0x%x\n", desc.src_addr);
//	    printk("desc.dma_total_len = 0x%x\n", desc.dma_total_len);
//	    printk("desc.intr_enable = 0x%x\n", desc.intr_enable);
//	    printk("desc.next_desc_addr = 0x%x\n", (*(u32 *)(desc.desc_addr + 12)));
    
    pdma_desc_rdata = *((volatile u32 *)p_desc->desc_addr);
}


void pdma_callback_register(u32 channel, pdma_callback callback)
{
    // enable channel
    PDMAREG(PDMA_ENABLE) |= (1 << channel);
    // enable channel interrupt
    PDMAREG(PDMA_INTR_ENABLE) |= (1 << channel);
    intr_callback_table[channel] = callback;
}

void pdma_callback_unregister(u32 channel)
{
    // disable channel
    PDMAREG(PDMA_ENABLE) &= ~(1 << channel);
    // disable channel interrupt
    PDMAREG(PDMA_INTR_ENABLE) &= ~(1 << channel);
    intr_callback_table[channel] = NULL;
}

u32 get_channel_data_len(u32 channel)
{
    u32 len;
    struct pdma_ch_descr *ch_desc = (struct pdma_ch_descr *)UNCACHED_ADDR(&ch_descr[channel]);

    len = (ch_desc->desc_config >> PDMA_TOTAL_LEN_SHIFT) & 0xffff;

    return len;
}

u32 get_desc_data_len(struct pdma_ch_descr *p_desc)
{
    u32 len;
    struct pdma_ch_descr *ch_desc;

    ch_desc = (struct pdma_ch_descr *)UNCACHED_ADDR(p_desc);
    len = (ch_desc->desc_config >> PDMA_TOTAL_LEN_SHIFT) & 0xffff;

    return len;
}

void restart_loop_desc(struct pdma_ch_descr *p_desc)
{
    struct pdma_ch_descr *ch_desc;

    ch_desc = (struct pdma_ch_descr *)UNCACHED_ADDR(p_desc);
    ch_desc->desc_config |= PDMA_VALID;
    ch_desc->desc_config |= (250 << PDMA_TOTAL_LEN_SHIFT);

//	    printk("desc_config = 0x%x\n", ch_desc->desc_config);
//	    printk("dest_addr = 0x%x\n", ch_desc->dest_addr);
//	    printk("src_addr = 0x%x\n", ch_desc->src_addr);
//	    printk("next_addr = 0x%x\n", ch_desc->next_addr);
//	    printk("cur_addr = 0x%x\n", (u32)ch_desc);
}

module_init(pdma_init)
