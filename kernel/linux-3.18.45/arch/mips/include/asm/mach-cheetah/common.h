/*=============================================================================+
|                                                                              |
| Copyright 2013                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*! 
*   \file common.h
*   \brief  
*   \author Montage
*/

#ifndef _COMMON_H_
#define _COMMON_H_

#ifdef __ECOS
#include <mt_types.h>

#define UMAC_REG_BASE    0xaf003000UL
#else
#include <asm/types.h>
#include <cheetah.h>

#define STA_FORWARD_MASK	0x00FF
#define STA_BSSDESC_MASK	0xFF00

#define STA_FORWARD_SHIFT	0
#define STA_BSSDESC_SHIFT	8

enum
{
	CLI_OK = 0,
	CLI_SHOW_USAGE = 1,
	CLI_ERROR = -1,
	CLI_ERR_PARM = -2,
	CLI_ERR_ADDR = -3,
	CLI_ERR_UNALIGN = -4,
};
#endif

enum {
	SOFTWARE_FORWARD = 1,
	HARDWARE_FORWARD,
};

#define ARGV_NO_FLOOD_MC			0x0001
#define ARGV_FLOOD_UNKNOW_UC		0x0002

#define IFF_ETH_DEV			0x1000	/* Ether netdevice */
#define IFF_WLA_DEV			0x2000	/* Wireless netdevice */
#define IFF_USB_DEV			0x4000	/* USB netdevice */
#define IFF_BSS_MASK		0xc000	/* BSS desc mask */

#define IFF_BSS_SHIFT		14		/* BSS desc shift */

#define MACADDR_TABLE_MAX		4

#define PHYSICAL_MACADDR		0x1
#define AP_MACADDR				0x2
#define STA_MACADDR				0x4
#define P2P_MACADDR				0x8
#define IBSS_MACADDR			0x10
#define WIF_USED				0x80

#define CREG_READ32(x)  (*(volatile u32*)(UMAC_REG_BASE+(x)))
#define CREG_WRITE32(x,val) (*(volatile u32*)(UMAC_REG_BASE+(x)) = (u32)(val))
#define CREG_UPDATE32(x,val,mask) do {           \
    u32 newval;                                        \
    newval = *(volatile u32*) (UMAC_REG_BASE+(x));     \
    newval = (( newval & ~(mask) ) | ( (val) & (mask) ));\
    *(volatile u32*)(UMAC_REG_BASE+(x)) = newval;      \
} while(0)

#define STA_DS_TABLE_CFG    0x87C
    #define STA_DS_TABLE_CFG_DONE       0x80000000UL
	#define STA_TABLE_MAX_SEARCH		0x000001FFUL
	#define DS_TABLE_MAX_SEACH			0x00003E00UL
	#define STA_TABLE_HASH_MODE			0x00008000UL
    #define STA_TABLE_HASH_MASK         0x00FF0000UL
	#define STA_DS_TABLE_CACHE_CFG		0x1F000000UL
	#define STA_TABLE_CLR				0x20000000UL
	#define DS_TABLE_CLR				0x40000000UL

#define MAC_BSSID0      0x8a8       /* BSSID bitmap & BSSID[47:32] */
    #define MAC_MBSSID_BITMAP       0x00FF0000UL
#define MAC_BSSID1      0x8ac       /* BSSID[31:0] */
	#define MSSID_MODE				(1 << 24) /* force TX only use first BSSID, 
											It is used for single mac address but multiple SSID  */ 		

#define MAC_BSSID_FUNC_MAP  0x8b0

#define MAC_BSSID_LOW_1 0x8f4
#define MAC_BSSID_LOW_2 0x8f8
#define MAC_BSSID_LOW_3 0x8fc
#define MAC_BSSID_LOW_4 0x900
#define MAC_BSSID_LOW_5 0x904
#define MAC_BSSID_LOW_6 0x908
#define MAC_BSSID_LOW_7 0x90c

struct macaddr_pool {
	char count;
	struct {
		char type;
		unsigned char addr[6];
	}m[MACADDR_TABLE_MAX];
};

void program_macaddr_table(struct macaddr_pool *mp);
s16 register_macaddr(u8 type, u8 *macaddr);
void clean_macaddr(char *addr, char type);
void clean_all_macaddr(void);
void lookup_table_done(void);

extern struct macaddr_pool maddr_tables;

extern u16 (*br_forward_lookup_hook)(char *addr);
extern u8 (*monitor_wifi_status_hook)(void);
extern void (*set_wifi_recover_hook)(u8 val);
#endif //#define _COMMON_H_

