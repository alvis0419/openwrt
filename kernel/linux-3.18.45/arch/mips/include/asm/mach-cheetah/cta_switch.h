/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*!
*   \file cta_switch.h
*   \brief  Registers of Cheetah switch
*   \author Montage
*/

#ifndef _CTA_SWITCH_H_
#define _CTA_SWITCH_H_

#ifdef __ECOS
#include <mt_types.h>
#include <common.h>
#else
#if defined(CONFIG_CHEETAH_FPGA)
#define CONFIG_FPGA
#endif
#include <asm/types.h>
#endif

#define UNCACHE_BASE		(0xa0000000)
#define CACHE_BASE			(0x80000000)
#define PHYSICAL_ADDR(va) 	(((u32)va)&0x1fffffff)
#define VIRTUAL_ADDR(pa)	(((u32)pa)|UNCACHE_BASE)
#define CACHED_ADDR(va)		((PHYSICAL_ADDR(va))|CACHE_BASE)
#define NONCACHED_ADDR(va)	(((u32)va)|UNCACHE_BASE)

#define SW_BASE				0xbf008000UL
#if !defined(UMAC_REG_BASE)
#define UMAC_REG_BASE		0xaf003000UL
#endif

#ifdef __ECOS
#if !defined(ANA_BASE)
#define ANA_BASE			0xaf005800UL
#endif
#define ANAREG(reg) ((volatile unsigned int*)(ANA_BASE))[reg]

#undef PINMUX
#undef CLKDIV

/* GPIO bank register */
#define PINMUX				(0x94/4)
	#define EN_MDIO_AUX_FNC		(1 << 23)
	#define EN_MDIO_FNC			(1 << 8)
	#define EN_ETH1_FNC			(1 << 5)
	#define EN_ETH0_FNC			(1 << 4)

/* analog bank register */
#define CLKDIV				(0x20/4)
	#define CLKDIV_CPUFFST		16
	#define CLKDIV_CPUSHFT		8
	#define CLKDIV_POSTDIV_NUM	8
	#define CPU_CLK_UPDATE		(1<<31)
	#define CPU_CLK_NOGATE		(1<<30)
	#define CPU_CLK_PREDIV		(0x00FF0000UL)
	#define CPU_CLK_POSTDIV		(0x07000000UL)
	#define SYS_CLK_UPDATE		(1<<15)
	#define SYS_CLK_NOGATE		(1<<14)
	#define SYS_CLK_PREDIV		(0x000000FFUL)
	#define SYS_CLK_POSTDIV		(0x00000700UL)
#endif

#define MACREG_READ32(x)  (*(volatile u32*)(UMAC_REG_BASE+(x)))
#define MACREG_WRITE32(x,val) (*(volatile u32*)(UMAC_REG_BASE+(x)) = (u32)(val))
#define MACREG_UPDATE32(x,val,mask) do {           \
    u32 newval;                                        \
    newval = *(volatile u32*) (UMAC_REG_BASE+(x));     \
    newval = (( newval & ~(mask) ) | ( (val) & (mask) ));\
    *(volatile u32*)(UMAC_REG_BASE+(x)) = newval;      \
} while(0)

#define SWREG_READ32(x)  (*(volatile u32*)(SW_BASE+(x)))
#define SWREG_WRITE32(x,val) (*(volatile u32*)(SW_BASE+(x)) = (u32)(val))
#define SWREG_UPDATE32(x,val,mask) do {           \
    u32 newval;                                        \
    newval = *(volatile u32*) (SW_BASE+(x));     \
    newval = (( newval & ~(mask) ) | ( (val) & (mask) ));\
    *(volatile u32*)(SW_BASE+(x)) = newval;      \
} while(0)

#ifdef CONFIG_MPEGTS
#define SWITCH_BUF_HEADER_COUNT				(0)
#define SWITCH_BUF_SIZE						(0)
#else
#define SWITCH_BUF_HEADER_COUNT				(62)
#define SWITCH_BUF_SIZE						(256)
#endif

#define P0_OFS		0x000
#define P1_OFS		0x100
#define P2_OFS		0x200

#ifdef IP_OFFSET
#undef IP_OFFSET
#endif

#define IP_OFFSET				64
#define INTERFRAME_GAP			22
#define MAX_FORWARD_LEN			1536
#define SW_DESC_NUMBER			204
#define PREAMBLE_BYTE_SIZE		7
#define MAX_PHY_NUM				2

//#define FUNC_MODE				0x2090
//    #define FMODE_EN                0x80000000UL
//    #define FMODE_EXT_SW_HNAT       0x40000000UL
//    #define FMODE_EXT_SW_WIFI       0x20000000UL
//    #define FMODE_EXT_BB            0x10000000UL
//    #define FMODE_BB_ONLY           0x08000000UL
//    #define FMODE_AFE_ONLY          0x04000000UL
//    #define FMODE_SW_TEST_MODE      0x02000000UL
//    #define FMODE_EFUSE_MODE        0x01000000UL
//    #define FMODE_DEBUG_EN          0x00000010UL
//    #define FMODE_DEBUG_SELECT      0x0000000FUL

//#define HW_RESET				0x20A0
//    #define HW_RESET_WMAC           0x00000001UL
//    #define HW_RESET_HNAT           0x00000002UL
//    #define HW_RESET_USB            0x00000004UL
//    #define HW_RESET_SWITCH         0x00000008UL
//    #define HW_RESET_BB             0x00000010UL
//    #define HW_RESET_SW_P0          0x00000040UL
//    #define HW_RESET_SW_P1          0x00000080UL
//    #define HW_RESET_WIFI_PAUSE     0x00000100UL
//    #define HW_RESET_HNAT_PAUSE     0x00000200UL
//    #define HW_RESET_USB_PAUSE      0x00000400UL
//    #define HW_RESET_SWITCH_PAUSE   0x00000800UL
//    #define HW_RESET_BB_PAUSE       0x00001000UL
//    #define HW_RESET_RMII_P0        0x00010000UL
//    #define HW_RESET_RMII_P1        0x00020000UL
//    #define HW_RESET_DCM_BB2MAC     0x80000000UL    /* FPGA only */

//#define RMII_MII				0x203C
#define P2_PHYMODE_CLK			0x30
	#define P2_PHYMODE_MII_100M		0x00000000UL
	#define P2_PHYMODE_HALF_SYS		0x00000010UL
	#define P2_PHYMODE_RMII			0x00000030UL
#define P0_ADAPTER_MODE			0x40
	#define P0_ADAPTER_MII			0x00000000UL
	#define P0_ADAPTER_RMII			0x00000040UL
#define P1_ADAPTER_MODE			0x80
	#define P1_ADAPTER_MII			0x00000000UL
	#define P1_ADAPTER_RMII			0x00000080UL
#define P0_PHYMODE_CLK			0x300
	#define P0_PHYMODE_MII_10M		0x00000000UL
	#define P0_PHYMODE_MII_100M		0x00000100UL
	#define P0_PHYMODE_RMII			0x00000300UL
#define P1_PHYMODE_CLK			0xC00
	#define P1_PHYMODE_MII_10M		0x00000000UL
	#define P1_PHYMODE_MII_100M		0x00000400UL
	#define P1_PHYMODE_RMII			0x00000C00UL
#define P0_RMII_OUT_REFCLK		0x1000
#define P1_RMII_OUT_REFCLK		0x2000
#define P0_MII_MODE				0x4000
	#define P0_MII_MAC				0x00000000UL
	#define P0_MII_PHY				0x00004000UL
#define P1_MII_MODE				0x8000
	#define P1_MII_MAC				0x00000000UL
	#define P1_MII_PHY				0x00008000UL

#define EMAC_CFG				0x00
	#define ETH_FLOWCTRL			0x40000000UL
	#define CRS_SCHEME				0x02000000UL
	#define BITRATE_SEL				0x00C00000UL
		#define BITRATE_10M				0x00000000UL
		#define BITRATE_100M			0x00400000UL
	#define HALF_DUPLEX				0x00200000UL
	#define BUF_OFS					0x0001F000UL
	#define TXCLK_INV				0x00000008UL
	#define RXCLK_INV				0x00000004UL
	#define TXEN					0x00000002UL
	#define RXEN					0x00000001UL

#define EMAC_RX_CFG				0x14
	#define RX_USE_LU_VLAN			0x00002000UL
	#define TIDMAP_SRC				0x00000060UL
		#define TIDMAP_TOS				0x00000000UL
		#define TIDMAP_VLAN				0x00000020UL
		#define TIDMAP_HDESC			0x00000040UL
	#define SECOND_VLAN				0x00000010UL
	#define RM_VLAN					0x00000004UL

#define EMAC_TX_CFG				0x18
	#define MIN_IFG					0x003F0000UL
	#define PREAMBLE_SIZE			0x00007000UL
	#define TX_DEBUG				0x00000800UL
	#define PAYLOAD_FLOWCTRL		0x00000080UL
	#define USE_DESC_VID			0x00000040UL
	#define INS_VLAN				0x00000008UL
	#define AUTO_RETRY				0x00000002UL
	#define COLIISION_DROP16		0x00000001UL

#define EMAC_STATE				0x1C
	#define E2C_DESC_CNT			0xFF000000UL
	#define E2W_DESC_CNT			0x00FF0000UL
	#define BUF_RUNOUT				0x00000020UL
	#define NO_E2W_HWDESC			0x00000010UL
	#define NO_E2W_DWDESC			0x00000008UL
	#define LAN_BUSY				0x00000004UL
	#define RXRDY					0x00000002UL
	#define TXRDY					0x00000001UL
	
#define EMAC_TID_TRANS_TBL_0_3	0x20
#define EMAC_TID_TRANS_TBL_4_7	0x24

#define EMAC_VLAN_ID_TBL_0_1	0x28
	#define VLAN_ID_TBL_0			0x00000FFFUL
	#define VLAN_ID_TBL_1			0x0FFF0000UL
#define EMAC_VLAN_ID_TBL_2_3	0x2C
	#define VLAN_ID_TBL_2			0x00000FFFUL
	#define VLAN_ID_TBL_3			0x0FFF0000UL
#define EMAC_VLAN_ID_TBL_4_5	0x30
	#define VLAN_ID_TBL_4			0x00000FFFUL
	#define VLAN_ID_TBL_5			0x0FFF0000UL
#define EMAC_VLAN_ID_TBL_6_7	0x34
	#define VLAN_ID_TBL_6			0x00000FFFUL
	#define VLAN_ID_TBL_7			0x0FFF0000UL

#define EMAC_TXRX_MISC			0x38

#define EMAC_PAUSE_MACADDR_H	0x40
	#define PAUSE_CLR				0x80000000UL
	#define PAUSE_CNT				0x7FFF0000UL

#define EMAC_PAUSE_MACADDR_L	0x44

/* Interrupt register */
#define EMAC_INT_MASK_REG		0x64
#define EMAC_INT_CLR			0x68
#define EMAC_INT_STATUS			0x6C
	#define EMAC_INT_SW_BUF_FULL	0x00000001UL		/* software buffer full */
	#define EMAC_INT_HW_BUF_FULL	0x00000002UL		/* fastpath buffer full */
	#define EMAC_INT_BUF_FULL		0x00000004UL		/* switch internal buffer full */
	#define EMAC_INT_BUF_AL_FULL	0x00000008UL		/* switch internal buffer almost full */
	#define EMAC_INT_DESC_FULL		0x00000010UL		/* switch internal descriptor full */

#define EMAC_STATISTIC_0		0x70
	#define ACBUSY_MCAST_CLR		0x80000000UL
	#define ACBUSY_MCAST_CNT		0x7FFF0000UL
	#define ACBUSY_UCAST_CLR		0x00008000UL
	#define ACBUSY_UCAST_CNT		0x00007FFFUL

#define EMAC_STATISTIC_1		0x74
	#define UNSIZE_CLR				0x80000000UL
	#define UNSIZE_CNT				0x7FFF0000UL
	#define OVERSIZE_CLR			0x00008000UL
	#define OVERSIZE_CNT			0x00007FFFUL

#define EMAC_STATISTIC_2		0x78
	#define OVERRUN_CLR				0x80000000UL
	#define OVERRUN_CNT				0x7FFF0000UL
	#define CRC_ERR_CLR				0x00008000UL
	#define CRC_ERR_CNT				0x00007FFFUL

#define EMAC_STATISTIC_3		0x7C
	#define RX_ENQ_CLR				0x80000000UL
	#define RX_ENQ_CNT				0x3FFFFFFFUL

#define EMAC_STATISTIC_4		0x80
	#define UNRUN_PKT_CLR			0x80000000UL
	#define UNRUN_PKT				0x7FFF0000UL
	#define UNRUN_CNT_CLR			0x00008000UL
	#define UNRUN_CNT				0x00007FFFUL

#define EMAC_STATISTIC_5		0x84
	#define JAM_CLR					0x80000000UL
	#define JAM_CNT					0x7FFF0000UL
	#define COLIISION_DROP16_CLR	0x00008000UL
	#define COLIISION_DROP16_CNT	0x00007FFFUL

#define EMAC_STATISTIC_6		0x88
	#define TX_DEQ_CLR				0x80000000UL
	#define TX_DEQ_CNT				0x3FFFFFFFUL

#define EMAC_STATISTIC_7		0x8C
	#define RXDV_CLR				0x80000000UL
	#define RXDV_CNT				0x3FFFFFFFUL

#define EMAC_STATISTIC_8		0x90
	#define CRSDV_CLR				0x80000000UL
	#define CRSDV_CNT				0x3FFFFFFFUL

#define EMAC_STATISTIC_9		0x94
	#define TX_EN_CLR				0x80000000UL
	#define TX_EN_CNT				0x3FFFFFFFUL

#define EMAC_LED				0x98
	#define LED_DARK_TH				0x7FF80000UL
	#define LED_LIGHT_TH			0x0007FF80UL
	#define LED_POLARITY			0x00000040UL
	#define LED_COUNTUNIT			0x00000020UL
	#define LED_AUTOMODE			0x00000010UL
	#define LED_AUTOFREQ			0x0000000CUL
	#define LED_MODE				0x00000003UL

#define EMAC_FSM				0x9C
	#define TX_FSM					0xFF000000UL
	#define TXDMA_FSM				0x00FF0000UL
	#define RX_FSM					0x0000FF00UL
	#define RXDMA_FSM				0x000000FFUL

#define EMAC_IFG				0xA0
	#define IFG_UPPER_BOUND			0xFF000000UL
	#define IFG_100M_CRS			0x00FF0000UL
	#define IFG_10M_CRS				0x0000FF00UL
	#define IFG_TXEN				0x000000FFUL


#define SW_PORT_CFG				0x300
	#define PORT_EN					0x000000FFUL
	#define TX_INS_VLAN				0x0000FF00UL

#define SW_CPU_PORT				0x304

#define SW_VLAN_GROUP0			0x308
#define SW_VLAN_GROUP1			0x30C
#define SW_VLAN_GROUP2			0x310
#define SW_VLAN_GROUP3			0x314
#define SW_VLAN_GROUP4			0x318
#define SW_VLAN_GROUP5			0x31C
#define SW_VLAN_GROUP6			0x320
#define SW_VLAN_GROUP7			0x324

#define SW_BSSID2VLAN0_1		0x328
#define SW_BSSID2VLAN2_3		0x32C
#define SW_BSSID2VLAN4_5		0x330
#define SW_BSSID2VLAN6_7		0x334

#define SW_PORT_VID0_1			0x338
#define SW_PORT_VID2_3			0x33C

#define SW_DEBUG_MODE			0x340
#define SW_DEBUG_STATUS			0x344
	#define SW_DEBUG_STATUS_MASK	0x0000FFFFUL

#define SW_DESC_INIT_DONE		0x380
#define SW_DESC_BADDR			0x384
#define SW_ADD_DESC_REQ			0x388
#define SW_ADD_DESC_HT			0x38C
#define SW_ADD_DESC_AMOUNT		0x390

#define SW_DESC_CUR_HT			0x394
#define SW_DESC_INFO0			0x398
#define SW_DESC_INFO1			0x39C

#define SW_HWBUF_HDR_BADDR		0x3A0
#define SW_HWBUF_HDR_HT			0x3A4
#define SW_HWBUF_SIZE			0x3A8
#define SW_HWBUF_POOL_CTRL		0x3AC
	#define SW_HWBUF_INIT_DONE		0x00000001UL
	#define SW_P3DMA_ENABLE			0x00000002UL

#define SW_HWBUF_FC1			0x3B0
#define SW_HWBUF_FC2			0x3B4
#define SW_TIMER				0x3BC

#define SW_FC_CTRL				0x3C0
	#define SW_FC_ENABLE			0x00000001UL
	#define SW_FC_TOGGLE_RANGE		0xFFFF0000UL
#define SW_FC_PUBLIC			0x3C4
	#define SW_FC_THD_REJ			0x0000FFFFUL
	#define SW_FC_THD_DROP			0xFFFF0000UL
#define SW_FC_PRIVATE0_1		0x3C8
#define SW_FC_PRIVATE2_3		0x3CC
#define SW_FC_COUNT0			0x3D0
#define SW_FC_COUNT1			0x3D4
#define SW_FC_COUNT2			0x3D8
#define SW_FC_COUNT3			0x3DC
#define SW_FC_ENQ0				0x3E0
#define SW_FC_ENQ1				0x3E4

typedef struct {
#if 1
#if defined(__BIG_ENDIAN)
	u32 offset:8;
	u32 next_index:11;
	u32 len:13;

	u32 ep:1;
	u32  :2;
	u32 dptr:29;
#else
	u32 len:13;
	u32 next_index:11;
	u32 offset:8;

	u32 dptr:29;
	u32  :2;
	u32 ep:1;
#endif
#else
	u32 next_index:16;
	u32 ep:1;
	u32 offset_h:2;				/* offset: high bits */
	u32 len:13;
	u32 offset:6;				/* offset: low bits */
	u32 dptr:26;
#endif
} __attribute__((__packed__)) buf_header;

typedef struct {
	u32 phyid[2];
	buf_header *switch_buf_headers;
	u16 switch_buf_headers_count;
	u16 switch_buf_size;
	u8 *switch_buf;

	u8 bitmap;
	u8 func;

	u16 mode:4;
	u16 tclk:4;
	u16 rclk:4;
	u16 boost:4;

	u32 sysclk:4;
	u32 res:28;

	u32 reg_core[20];
	u16 local[MAX_PHY_NUM];
	u16 partner[MAX_PHY_NUM];
	u16 status[MAX_PHY_NUM];
	u8 addr[MAX_PHY_NUM][6];	/* ethernet hardware address */
	u32 mon_hit;
	u32 mon_cnt;
	u32 mon_bufempty_hit;
	u32 mon_bufempty_cnt;
	u32 mon_p1rxzombie_hit;
	u32 mon_p1rxzombie_cnt;
	u32 mon_p2rxzombie_hit;
	u32 mon_p2rxzombie_cnt;
	u32 mon_p2bufempty_hit;
	u32 mon_p2bufempty_cnt;
	u32 mon_crc;
	u32 mon_phy_p20x17;
	u32 mon_phy_p20x15;
	u32 mon_phy_p40x14;
	u32 mon_phy_anerr;
} SWITCH_INFO;

#define P0_PHY_ADDR 	1
#define P1_PHY_ADDR 	0
enum {
#if defined(CONFIG_P0_AS_LAN)
	PORT_ETH1_IDX = 0,
	PORT_ETH0_IDX,
#else
	PORT_ETH0_IDX = 0,
	PORT_ETH1_IDX,
#endif
	PORT_HNAT_IDX,
	PORT_WLAN_IDX,
};

#if defined(CONFIG_P0_AS_LAN)
#define PORT_ETH1				0x00000001UL
#define PORT_ETH0				0x00000002UL
#else
#define PORT_ETH0				0x00000001UL
#define PORT_ETH1				0x00000002UL
#endif
#define PORT_HNAT				0x00000004UL
#define PORT_WLAN				0x00000008UL

#define FLG_8021Q_OUTB			0x00008000UL

#define MASK_PORT_BMAP			0x0000000FUL
#define MASK_8021Q_IDX			0x00000700UL
#define MASK_8021Q_FLG			0x0000F000UL
#define MASK_8021Q_VID			0x0FFF0000UL
#define MASK_BSSID_IDX			MASK_8021Q_IDX
#define MASK_PVID_IDX			MASK_8021Q_IDX

#define CAP_ACTIVE				0x00000001UL
#define CAP_BRIDGE				0x00000002UL
#define CAP_BRIDGE_STATE		0x00000004UL
#define CAP_TXRETRY				0x00000008UL
#define CAP_MONITOR				0x00000010UL
#define CAP_RESERVED3			0x00000020UL
#define CAP_RESERVED4			0x00000040UL
#define CAP_POLLPHY				0x00000080UL

enum {
	SL0 = 0,
	SL8,
	SL16,
	SR0,
	SR8,
	SR16,
	S_NONE,
};

enum
{
	PHY_CAP_AN = (1<<14),
	PHY_CAP_PAUSE = (1<<10),
	PHY_CAP_100F = (1<<8),
	PHY_CAP_100H = (1<<7),
	PHY_CAP_10F = (1<<6),
	PHY_CAP_10H = (1<<5),
	
	PHY_AN = (1<<6),
	PHY_ANC = (1<<5),
	PHY_LINK = (1<<4),
	PHY_100M = (1<<1),
	PHY_FDX = (1<<0),
	
	PHYR_ST_ANC = (1<<5),
	PHYR_ST_AN = (1<<3),
	PHYR_ST_LINK = (1<<2),
	
	
	PHYR_CTL_AN = (1<<12),
	PHYR_CTL_DUPLEX = (1<<8),
	PHYR_CTL_SPEED = (1<<13),
};

enum
{
	MODE_ETH_DBG = 0,
	MODE_AP = 1,
	MODE_WIFI_RT,
	MODE_WISP,
	MODE_REPEATER,
	MODE_BRIDGE,
	MODE_SMART,
	MODE_3G,
#ifdef CONFIG_WLA_P2P
	MODE_P2P,
	MODE_SW_MAX=MODE_P2P
#else
	MODE_SW_MAX=MODE_3G
#endif
};

void cta_switch_port_attach(u8 idx, u8 vlan, u8 calibr);
void cta_switch_port_detach(u8 idx);
int cta_switch_port_ctrl(u8 phy, int link);
void cta_switch_txrx_ctrl(u8 idx, u8 tclk, u8 rclk);
void cta_switch_sclk_ctrl(void);
int cta_switch_mac_ctrl(int idx, u8 *mac);
int cta_switch_8021q_ctrl(u32 val);
void cta_switch_ethernet_forward_to_wifi(u32 permit);
int cta_switch_bssid_ctrl(u32 val);
int cta_switch_pvid_ctrl(u32 val);
int cta_switch_state_ctrl(u32 val);
void cta_switch_brg_write_state(u8 val);
int cta_switch_brg_read_state(void);
int cta_switch_brg_state(void);
int cta_switch_pp_state(void);
int cta_switch_work_mode(int mode);
int cta_switch_port_idx(int unit);
u32 cta_switch_phyid(int idx);
u16 cta_switch_phybcap(int idx);
u16 cta_switch_phyctrl(int idx);
s8 cta_switch_init(int advert);
s8 cta_switch_buf_init(void);
s8 cta_switch_reg_init(void);
void cta_switch_led_init(int idx);
void cta_switch_led_action(int idx, u16 link);
u8 cta_switch_monitor(void);
u8 cta_switch_monitor_status(void);
u8 cta_switch_crc_monitor(int idx);
u32 cta_switch_inc_phy_p20x17(void);
u32 cta_switch_inc_phy_p20x15(void);
u32 cta_switch_inc_phy_p40x14(void);
u32 cta_switch_inc_phy_anerr(void);
void cta_an(void);

#endif //#define _CTA_SWITCH_H_

