/* 
 *  Copyright (c) 2008, 2009    Acrospeed Inc.    All rights reserved.
 *
 *  Camelot platform CPU feature overrides
 */
#ifndef __ASM_CHEETAH_CPU_FEATURE_OVERRIDES_H
#define __ASM_CHEETAH_CPU_FEATURE_OVERRIDES_H

#define cpu_has_tlb			1
#define cpu_has_4kex			1
#define cpu_has_4k_cache		1
//#define cpu_has_fpu			0
//#define cpu_has_32fpr			0
#define cpu_has_counter			1
//#define cpu_has_watch			0
#define cpu_has_divec			1
#define cpu_has_vce			0
//#define cpu_has_cache_cdex_p	0
//#define cpu_has_cache_cdex_s	0
//#define cpu_has_prefetch		1
#define cpu_has_mcheck			0
#define cpu_has_ejtag			1
#define cpu_has_llsc			1
#define cpu_has_veic          		0
#define cpu_has_vint          		0
//#define cpu_has_mips16		1

#endif /* __ASM_CHEETAH_CPU_FEATURE_OVERRIDES_H */
