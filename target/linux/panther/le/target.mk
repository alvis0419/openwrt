ARCH:=mipsel
ARCH_PACKAGES:=panther_mipsel
SUBTARGET:=le
BOARDNAME:=Little Endian
FEATURES:=ramdisk

define Target/Description
	Build LE firmware images for MIPS Malta CoreLV board running in
	little-endian mode
endef
