/*=============================================================================+
|                                                                              |
| Copyright 2012                                                               |
| Montage Inc. All right reserved.                                             |
|                                                                              |
+=============================================================================*/
/*!
*   \file cta_hnat.h
*   \brief  Registers of Cheetah hnat
*   \author Montage
*/

#ifndef CTA_HNAT_H
#define CTA_HNAT_H

#ifdef __ECOS
#define HNAT_LOCK_INIT
#define HNAT_LOCK						cyg_scheduler_lock()
#define HNAT_UNLOCK						cyg_scheduler_unlock()
#else
#define HNAT_LOCK						local_irq_save(hnat_flags)
#define HNAT_UNLOCK						local_irq_restore(hnat_flags)
#endif

#define CTA_HNAT_NOHNAT					0
#define CTA_HNAT_FULLHNAT				1
#define CTA_HNAT_SEMIHNAT				2

#define CTA_HNAT_PRIVATE_IF				0
#define CTA_HNAT_PUBLIC_IF				1

#define HNRC_LEARNMAC					2
#define HNRC_TX							1
#define HNRC_OK							0
#define HNRC_ERR						-1
#define HNRC_SAME						-2

#define HNAT_DBG_NATDS					(1<<1)
#define HNAT_DBG_HASHO					(1<<2)
#define HNAT_DBG_HASHI					(1<<3)

#define SIP2LSB(v)						((unsigned short)((v << 16) >>16))
#define SIP2MSB(v)						((unsigned short)(v >> 16))
#define LSB2SIP(v, m)					((unsigned int)(v | (m << 16)))

#define CTA_HNAT_PUBLIC_IP_MAX_NUM		16
#define CTA_HNAT_PUBLIC_PPPOE_MAX_NUM	8

#define IP_PACKET_TYPE					0x0800
#define PPPOE_PACKET_TYPE				0x8864
#define VLAN_PACKET_TYPE				0x8100

enum{
	HNAT_ACC_NONE = 0,
	HNAT_ACC_LEARNMAC,
	HNAT_ACC_ATTACHED,
};

typedef struct
{
#if defined(__BIG_ENDIAN)
	/* half-word 0 */
	unsigned short	mipidx:4;			/* global IP address index */
	unsigned short	counter:12;			/* HW will update this counter */
#define ml_next		counter
	/* half-word 1 */
	unsigned short	mp;					/* global port */
	/* half-word 2 */
	unsigned char	prot;				/* protocol */
	unsigned char	hw_resv:1;
	unsigned char	full:1;
	unsigned char	tocpu:1;
	unsigned char	drop:1;
	unsigned char	alg:4;
	/* half-word 3 */
	unsigned short	siplsb;				/* Least significant two bytes of local IP address */
	/* half-word 4,5 */
	unsigned int	dip;				/* remote IP address */
	/* half-word 6 */
	unsigned short	sp;					/* local port */
	/* half-word 7 */
	unsigned short	dp;					/* remote port */
	/* half-word 8 */
	unsigned short	sw_resv1:4;
	unsigned short	hnat_acc:2;			/* HW accelerate the seesion */
	unsigned short	sw_resv4:2;
	unsigned short	dmac:8;
	/* half-word 9 */
	unsigned short	sw_resv2:8;
	unsigned short	smac:8;

	/* half-word 10-15 */
	unsigned short	sipmsb;				/* Most significant two bytes of local IP address */
	unsigned short	used:1;				/* indicate the descriptor has used */
	unsigned short	sw_resv3:2;
	unsigned short	ihnext:13;
#ifndef CTA_HNAT_BUCKET_SIZE_24
	unsigned short	sw_resv_word[4];
#endif
#else
	/* half-word 0 */
	unsigned short	counter:12;			/* HW will update this counter */
#define ml_next		counter
	unsigned short	mipidx:4;			/* global IP address index */
	/* half-word 1 */
	unsigned short	mp;					/* global port */
	/* half-word 2 */
	unsigned char	alg:4;
	unsigned char	drop:1;
	unsigned char	tocpu:1;
	unsigned char	full:1;
	unsigned char	hw_resv:1;
	unsigned char	prot;				/* protocol */
	/* half-word 3 */
	unsigned short	siplsb;				/* Least significant two bytes of local IP address */
	/* half-word 4,5 */
	unsigned int	dip;				/* remote IP address */
	/* half-word 6 */
	unsigned short	sp;					/* local port */
	/* half-word 7 */
	unsigned short	dp;					/* remote port */
	/* half-word 8 */
	unsigned short	dmac:8;
	unsigned short	sw_resv4:2;
	unsigned short	hnat_acc:2;			/* HW accelerate the seesion */
	unsigned short	sw_resv1:4;
	/* half-word 9 */
	unsigned short	smac:8;
	unsigned short	sw_resv2:8;

	/* half-word 10-15 */
	unsigned short	sipmsb;				/* Most significant two bytes of local IP address */
	unsigned short	ihnext:13;
	unsigned short	sw_resv3:2;
	unsigned short	used:1;				/* indicate the descriptor has used */
#ifndef CTA_HNAT_BUCKET_SIZE_24
	unsigned short	sw_resv_word[4];
#endif
#endif
}natdes;

#ifdef __ECOS
typedef struct cb{
#if defined(__BIG_ENDIAN)
	unsigned int dir:1;
	unsigned int poe:4;
	unsigned int force:1;
	unsigned int pkt_len:16;
	unsigned int mac_index:8;
	unsigned int res:2;
	unsigned int pkt;
	union {
		struct
		{
			unsigned int res1:8;
			unsigned int stag:8;
			unsigned int vlan:16;
		} CB_qos;
		unsigned int dmac;
	} CB_dat;
#else
	unsigned int res:2;
	unsigned int mac_index:8;
	unsigned int pkt_len:16;
	unsigned int force:1;
	unsigned int poe:4;
	unsigned int dir:1;

	unsigned int pkt;
	union {
		struct
		{
			unsigned int vlan:16;
			unsigned int stag:8;
			unsigned int res1:8;
		} CB_qos;
		unsigned int dmac;
	} CB_dat;
#endif
}cb_t;

#define mtocb(m)			((cb_t *)(m->m_pkthdr.cb))		/* control block on mbuf structure */
#define cb_stag				CB_dat.CB_qos.stag
#define cb_vlan				CB_dat.CB_qos.vlan
#define cb_dmac				CB_dat.dmac
#else
#define M_HWMOD				0x1000							/* HW NAT has modified header */
#define SKB2SEMICB(skb)		((semi_cb_t *)&skb->cb[0])		/* semi hnat control block on skb structure */
#define IS_HWMOD(semi_cb)	(((semi_cb)->flags & M_HWMOD) && ((semi_cb)->magic == SEMI_CB_MAGIC))

/* semi hnat control block */
typedef struct semi_cb_s {
	unsigned long long magic;
	unsigned short flags;
	unsigned short reserved;
	void *ds;
	unsigned int w0;
	unsigned int w1;
} semi_cb_t;
#define SEMI_CB_MAGIC  0x5F53656d495F6342ULL  // '_' 'S' 'e' 'm' 'I' '_' 'c' 'B'
#endif

short find_mipidx(unsigned long ip);
int cta_hnat_init(void);
void cta_hnat_reset(void);
void cta_hnat_unload(void);
void cta_hnat_cfgmode(int mode);
int cta_hnat_cfgip(int which, int ip, int is_pppoe);

natdes *cta_hnat_alloc(void);
void cta_hnat_free(natdes *);

int cta_hnat_attach(natdes *);
void cta_hnat_detach(natdes *);
int cta_hnat_find_mac(char *mac);

void cta_hnat_learn_mac(char *hwadrp, char dir, char vid) __attribute__ ((section(".iram")));
unsigned int cta_hnat_get_counter(short s);

unsigned long hmu_keygen(unsigned short ip, unsigned short port, unsigned short prot);
short sess_to_idx(natdes *);
natdes *idx_to_sess(short s);
unsigned int idx_to_mip(int mipidx);
int cta_hnat_get_statistic(void);
int cta_hnat_rx_check(char *buf, char *pkt, unsigned int w0, unsigned int w1);
void cta_hnat_manual_cfgmac(natdes *ds, char *mac, char tag, int dir);
void sess_dc_inval(natdes *ds);
void sess_dc_store(natdes *ds);
void cta_hnat_dump(short cmd, void *parm);
#ifdef CONFIG_SPECIAL_TAG
unsigned short cta_hnat_get_tpid(natdes *ds, cb_t *cb);
#endif

#ifdef __ECOS
void cta_hnat_copy_cbinfo(char *iphdr, int len, natdes *ds, cb_t *cb);
#else
void cta_hnat_fast_send(char *iphdr, int len, unsigned long key, semi_cb_t *semi_cb);
void cta_hnat_set_semicb(unsigned long buf_cb, void *hw_nat, unsigned int w0, unsigned int w1);

extern int cta_eth_debug_lvl;
int cta_vlan_dev_hwaccel_hard_start_xmit(struct sk_buff *skb, struct net_device *dev);
void cta_hnat_initboard(short wport);

struct cta_vlancmd_table
{
	unsigned short  vid;
	unsigned char   member;
	unsigned char   tag;
	unsigned char   untag;
	unsigned char   port;
	unsigned char   flag;
	unsigned char   pri;
	unsigned char   used;
};

extern int vlan_dev_hwaccel_hard_start_xmit(struct sk_buff *skb, struct net_device *dev);
extern int hnat_fast_send(unsigned long key);
#endif
extern unsigned short hnat_max_num;
extern int hnat_mode_g;
#endif /* CTA_HNAT_H */
