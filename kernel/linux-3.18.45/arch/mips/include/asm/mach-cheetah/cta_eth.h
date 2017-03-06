/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*!
*   \file cta_eth.h
*   \brief  Registers of Cheetah ethernet
*   \author Montage
*/

#ifndef CTA_ETH_H
#define CTA_ETH_H

#include <linux/if_vlan.h>

#ifndef __BIT
#define __BIT(_a)				(0x1 << (_a))
#endif

/*!
 * \brief The number of receive buffer descrcriptors
*/
#ifdef CTA_ETH_RX_DESC_SZ
#define NUM_RX_DESC				CTA_ETH_RX_DESC_SZ
#else
#define NUM_RX_DESC				48	//32
#endif
/*!
 * \brief The number of transmit buffer descrcriptors
*/
#ifdef CTA_ETH_TX_DESC_SZ
#define NUM_TX_DESC				CTA_ETH_TX_DESC_SZ
#else
#define NUM_TX_DESC				96	//32
#endif
/*!
 * \brief The number of hardware buffer descrcriptors
*/
#define NUM_HW_DESC				4

#define ETH_ADDR_LEN			6
#define ETH_TYPE_LEN			2
#define ETH_HEAD_LEN			(2*ETH_ADDR_LEN + ETH_TYPE_LEN)
#define ETH_CRC_LEN				4
#define ETH_MIN_FRAMELEN		60
//#define TX_LEN_PADDING		1

#define MAX_IF_NUM				5

typedef struct if_cta_eth_s {
#ifdef _ECOS
	struct eth_drv_sc *sc;
#else
	struct net_device *sc;
#endif
	struct vlan_group *vg;
	struct net_device *if_swap;
	struct net_device *br_dev;

	unsigned short unit;
	unsigned short link_status;
	
	unsigned short flags;
	unsigned int to_vlan;
	unsigned char macaddr[ETH_ADDR_LEN];
	unsigned short phy_addr;
} if_cta_eth_t;


#define PRIVIFFG_ACTIVE			0x2
#define PRIVIFFG_MAC_SET		0x1
#define CTA_ETH_NBUF_LEN		1536

/*  buf descriptor for tx/rx control  */
typedef struct bufdesc_s {
#ifdef __ECOS
	struct eth_drv_sc *sc;
#else
	struct net_device *sc;
#endif
	void *buf;
} bufdesc_t;


/*  RX descriptor  */
typedef struct rxdesc_s {
	unsigned int w0;
	unsigned int w1;
} rxdesc_t;


/*  TX descriptor  */
typedef struct txdesc_s {
	unsigned int w0;
	unsigned int w1;
} txdesc_t;

#define MPHY_TR					(1<<31)
#define MPHY_WR					(1<<30)
#define MPHY_RD					(0<<30)
#define MPHY_EXT				(1<<5)
#define MPHY_PA_S				24
#define MPHY_RA_S				16

#define MCR_LOOPBACK			(1<<0)
#define MCR_RXMAC_EN			(1<<1)
#define MCR_FLOWCTRL			(1<<2)
#define MCR_LINKUP				(1<<3)
#define MCR_RXDMA_EN			(1<<4)
#define MCR_HNAT_EN				(1<<5)
#define MCR_TOCPU				(1<<6)
#define MCR_BYBASS_UPDATE		(1<<7)
#define MCR_TXMAC_EN			(1<<8)
#define MCR_TX_FLOWCTRL			(1<<9)
#define MCR_BYBASS_LOOKUP		(1<<10)
#define MCR_ROUTE_LOOPBACK		(1<<11)
#define MCR_TXDMA_EN			(1<<12)
#define MCR_TXDMA_RETRY_EN		(1<<13)
#define MCR_NATBUCKET_SIZE		(1<<14)
#define MCR_PREFETCH_DIS		(1<<15)
#define MCR_REVERSEIDX_GEN		(1<<16)
#define MCR_HMU_EN				(1<<17)
#define MCR_CACHE_EN			(1<<18)
#define MCR_HASH_FUNC			(1<<19)
#define MCR_BWCTRL_DROP			(1<<20)
#define MCR_BWCTRL_TRIGGER		(1<<21)
#define MCR_HMU_TESTMODE		(1<<22)
#define MCR_HWDS_EXT_RAM		(1<<23)
#define MCR_SMI_S				24

#define DS0_RX_OWNBIT			(1<<31)
#define DS0_TX_OWNBIT			(1<<30)
#define DS0_END_OF_RING			(1<<29)
#define DS0_FREE_OF_PACKET		(1<<28)
#define DS0_DROP_OF_PACKET		(1<<27)
#define DS0_LEN_S				16
#define DS0_VL					(1<<11)		//vlan valid
#define DS0_VID_S				8			//vlan index

//packet info
#define DS0_PT_S				13			//rx packet type
#define DS0_PT_B				3			//rx packet type, 3bits
#define DS0_IPF					(1<<12)		//ip fragment
#define DS0_INB					(1<<7)		//inbond direction
#define DS0_TRSF				(1<<6)		//tcp RST/SYN/FIN flag
#define DS0_NATH				(1<<5)		//nat table lookup hit
#define DS0_ALG_S				(1<<0)		//ALG 5bits
//if DES_D (drop is set)
#define DS0_FOR					(1<<15)		//fifo overrun
#define DS0_L3LE				(1<<14)		//layer 3 length error(runt packet/over length/length unmatch)
#define DS0_PPPOEUM				(1<<13)		//pppoe unmatch
#define DS0_DAUM				(1<<12)		//destination unmatch
#define DS0_INB					(1<<7)		//inbond direction
#define DS0_L2LE				(1<<6)		//layer 2 length error(runt packet/over length)
#define DS0_NATH				(1<<5)		//nat table lookup hit
#define DS0_ETTL				(1<<4)		//ip's ttl <2
#define DS0_ECRC				(1<<3)		//crc error
#define DS0_EL4CS				(1<<2)		//tcp checksum error
#define DS0_EIPCS				(1<<1)		//ip checksum error
#define DS0_ETSYC				(1<<0)		//tcp sync over
/* DS1 */
#define DS1_POE					(1<<31)
#define DS1_PSI_S				28
#define DS1_BUF_M				((1<<28)-1)
/* for tx only */
#define DS0_DF					(1<<27)		//tx raw packet
#define DS0_TST_S				13			//tx status
#define DS0_TST_B				3			//tx status, 3bits
#define DS0_TST_M 				(((1 << DS0_TST_B)-1 )<< DS0_TST_S)
#define DS0_M					(1<<12)		//more fragment in current packet
#define DS0_DMI_S				0			//dest mac index

#define MPHY					0x00
#define MCR						0x04
#define MAC_RXCR				0x14

#define MIE_HMU					(1<<0)
#define MIE_TX					(1<<1)
#define MIE_SQE					(1<<2)
#define MIE_HQE					(1<<3)
#define MIE_BE					(1<<4)
#define MIE_RX					(1<<5)

#define MIS_HMU					(1<<0)
#define MIS_TX					(1<<1)
#define MIS_SQE					(1<<2)
#define MIS_HQE					(1<<3)
#define MIS_BE					(1<<4)
#define MIS_RX					(1<<5)

#define MIE_ENABLE				(0x3f)
#define MIE						0x84
#define MIM						0x84
#define MIC						0x88
#define MIS						0x8C

#define MSWDB					0x08
#define MHWDB					0x0c
#define MBUFB					0x10
#define MRXCR					0x14
#define MPOET					0x18
#define MVL01					0x1c
#define MVL23					0x20
#define MVL45					0x24
#define MVL67					0x28
#define MTSR					0x2c
#define MDROPS					0x30
#define MPKTSZ					0x34
#define MFP						0x38
#define MSA00					0x3c
#define MSA01					0x40
#define MSA10					0x44
#define MSA11					0x48
#define MAUTO					0x4c
#define MFC0					0x50
#define MFC1					0x54
#define MDBUG					0x58
#define MSRST					0x5c
#define MMEMB					0x60
#define MTXDB					0x64

#define MPROT					0x80
#define ML3						0x90
#define MTMII					0x98

#define MTT						0xc0
#define MCT						0xc4
#define MTTH					0xc8
#define MTW						0xcc
#define MARPB					0xD0
#define MLBUF					0xD4
#define MSTAG					0xe8

#define MHTOBB					0x180
#define MHTIBB					0x184
#define MTB						0x188
#define MSIP					0x194
#define MECF					0x198	//NAT entry counter full status
#define MNATEA					0x19C
#define MCRCIV					0x1a0	//CRC10 initial value

#define MHIP					0x1d8
#define MHPORT					0x1dc
#define MHPROT					0x1e0
#define MHKEY					0x1e4
#define MRXCT					0x200
	#define RXTOHW_CNT			0xFFFF0000UL
	#define RXTOSW_CNT			0x0000FFFFUL
	#define RXTOHW_CNT_SHIFT		16
#define MTXCT					0x204
	#define TXFROMHW_CNT			0xFFFF0000UL
	#define TXFROMSW_CNT			0x0000FFFFUL
	#define TXFROMHW_CNT_SHIFT		16
#define NHM						0x208	//HMU hash mode b0: inbond idx using sip xor dip
#define MSMS					0x20c
#define HWDESC					0x1C0

//#define HW_RESET				0x20A0

#define RX_VIDX(_status)		(((_status) & 0x00000700) >> 8)
#define RX_FRAMELEN(_status)	(((_status) & 0x07ff0000) >> 16)
#define RX_ERROR(_status)		((_status) & 0x0000ffff)
#define RX_PROTYPE(_status)		(((_status) & 0x0000e000) >> 13)

/* line status */
#define GEN_STATUS_RESTART		0x80	// restart phy setting
#define GEN_STATUS_AN			0x10	// doing the Auto Nego
#define GEN_STATUS_FORCE		0x08	// force the link status, no AN
#define GEN_STATUS_FDX			0x04	// 1 = full duplex, 0 = half
#define GEN_STATUS_100MBPS		0x02	// 1 = 100 Mbps, 0 = 10 Mbps
#define GEN_STATUS_LINK			0x01	// 1 = link up, 0 = link down

/*  Receive machine  */
typedef struct rxctrl_s {
	rxdesc_t *prxd;
	unsigned short num_rxd;
	unsigned short num_freerxd;
	unsigned short rx_head;
	unsigned short rx_tail;

	unsigned int rx_pktcnt;
	unsigned int rx_bytecnt;
	unsigned int rx_mcpkt;
	unsigned int rx_bcpkt;
	unsigned int rx_err;
	unsigned int rx_errcode[16];
} rxctrl_t;

/*  Transmit machine  */
typedef struct txctrl_s {
	txdesc_t *ptxd;
	unsigned short num_txd;
	volatile unsigned short num_freetxd;
	volatile unsigned short tx_head;
	volatile unsigned short tx_tail;

	bufdesc_t *ptxbuf;
	unsigned int tx_pktcnt;
	unsigned int tx_ok;
	unsigned int tx_ubuf;
	unsigned int tx_sglist;
	unsigned int tx_drop;
	unsigned int tx_err;
	unsigned int tx_bytecnt;
} txctrl_t;

typedef struct EPHY {
	unsigned short mdio_fail;
	unsigned short res;

	unsigned int anc_fail;
#if 0
	unsigned int dsp_fail;
	/* 20131211 */
	unsigned int dsp_p20x17_fail;
	unsigned int dsp_p20x15_fail;
	/* 20131213 */
	unsigned int dsp_p40x14_fail;
	unsigned int rst_unlock;
#endif
} EPHY;

typedef struct cta_eth_s {
	unsigned int vector;		// interrupt vector
#ifdef __ECOS
	unsigned int int_hdl;		// handle for int.handler
	cyg_interrupt int_obj;		// interrupt object
	struct eth_drv_sc *sc_main;
#else
	int (*ndo_set_mac_address)(struct net_device *dev, void *addr);
	struct net_device *sc_main;
#endif //_ECOS
	unsigned short flags;
	unsigned short if_cnt;
	unsigned short act_ifmask;
	unsigned int int_mask;
	unsigned char flowctrl_mode;
	unsigned char flowctrl_state;

	txctrl_t tx;
	rxctrl_t rx;
	rxctrl_t hw;

	rxctrl_t *act_rx;
	if_cta_eth_t *if_array[MAX_IF_NUM];

	EPHY ephy;

	struct workqueue_struct *wq;
	struct work_struct work;
	
	struct napi_struct cta_napi;
	struct timer_list timer;
} cta_eth_t;

#define MREG(ofs)				((volatile unsigned int*)MAC_BASE)[ofs>>2]
#define ETH_REG32(ofs)			((volatile unsigned int*)MAC_BASE)[ofs>>2]
#define TX_MAX_BUFCNT			3	//6
#define REG_READ32(reg)			(*(volatile unsigned int*)(reg))
#define REG_WRITE32(reg, val)		(*(volatile unsigned int*)(reg) = (unsigned int)(val))
#define MREG_READ32(reg)		REG_READ32((unsigned int)&(((unsigned long *)MAC_BASE)[(reg)>>2]))
#define MREG_WRITE32(reg, val)		REG_WRITE32((unsigned int)&(((unsigned long *)MAC_BASE)[(reg)>>2]), (val))
#define SMIREG_READ32(reg)		REG_READ32((unsigned int)&(((unsigned long *)SMI_BASE)[(reg)>>2]))
#define SMIREG_WRITE32(reg, val)	REG_WRITE32((unsigned int)&(((unsigned long *)SMI_BASE)[(reg)>>2]), (val))

#define virtophy(va) 			(((unsigned int)va)&0x1fffffff)
#define phytovirt(pa)			(((unsigned int)pa)|0xa0000000)
#define nonca_addr(va)			(((unsigned int)va)|0xa0000000)
#define ca_addr(va)				(((unsigned int)va)|0x80000000)


#define BUF_IPOFF 				0x40
#define BUF_IPOMSK 				(~(BUF_IPOFF-1))
#define BUF_MSK					(~((1<<11)-1))
#define BUF_MSB_OFS				0x0		//to save MSB16 of buffer pointer
#define BUF_HW_OFS				0x4		//where hw use it as the first byte
#define BUF_SW_OFS				0x0		//where sw use it as the software pointer

#define VLAN_VIDX_SKB_CB(__skb) ((struct vlan_skb_vlaninfo *)&((__skb)->cb[20]))
#define vlan_tx_vidx_get(__skb) (VLAN_VIDX_SKB_CB(__skb)->vidx)

#define HAL_DCACHE_FLUSH(a,l)	pci_map_single(0,(void *)a,l,PCI_DMA_FROMDEVICE)
#define HAL_DCACHE_STORE(a,l)	pci_map_single(0,(void *)a,l,PCI_DMA_TODEVICE)

struct vlan_skb_vlaninfo
{
	unsigned char used:1;
	unsigned char resv:3;
	unsigned char vidx:4;
};

/* phy id */
#define PHYID_ICPLUS_IP101G		0x02430c54
#define PHYID_MONT_EPHY			0x00177c01

extern void cta_eth_mdio_wr(short phy,short reg,unsigned short val);
extern unsigned short cta_eth_mdio_rd(short phy,short reg);
extern int an_cmd(short phyno, u16 val, u16 cap);
extern short phy_status(short phyno);
extern unsigned int cta_eth_phy_init(unsigned char p);
#endif	/*  CTA_ETH_H  */
