cmd_/workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/linux/tc_act/.install := bash scripts/headers_install.sh /workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/linux/tc_act ./include/uapi/linux/tc_act tc_csum.h tc_defact.h tc_gact.h tc_ipt.h tc_mirred.h tc_nat.h tc_pedit.h tc_skbedit.h; bash scripts/headers_install.sh /workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/linux/tc_act ./include/linux/tc_act ; bash scripts/headers_install.sh /workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/linux/tc_act ./include/generated/uapi/linux/tc_act ; for F in ; do echo "\#include <asm-generic/$$F>" > /workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/linux/tc_act/$$F; done; touch /workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/linux/tc_act/.install
