cmd_/workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/video/.install := bash scripts/headers_install.sh /workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/video ./include/uapi/video edid.h sisfb.h uvesafb.h; bash scripts/headers_install.sh /workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/video ./include/video ; bash scripts/headers_install.sh /workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/video ./include/generated/uapi/video ; for F in ; do echo "\#include <asm-generic/$$F>" > /workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/video/$$F; done; touch /workplace/openwrt/build_dir/toolchain-mipsel_mips32_gcc-4.8-linaro_uClibc-0.9.33.2/linux-dev//include/video/.install
