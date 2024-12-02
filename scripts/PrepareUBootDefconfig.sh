#!/bin/bash

# This script prepares temporary UBoot defconfig for specified board and
# feature set for Windows IoT platform

if [ $# -ne 3 ]; then
	echo "Usage:"
	echo "  $0 <BOARD> <UUU> <SECURE> [DEBUG]"
	echo "BOARD:  imx8M|imx8Mm|imx8Mn|imx8Mnddr4|imx8Mp|imx8X|imx93|imx95"
	echo "UUU:    1|0"
	echo "SECURE: 1|0"
	echo "DEBUG: 1|0"
	exit 1
fi

BOARD=${1,,}
UUU=$2
SECURE=$3

# -- Validate input parameters
if [ "$UUU" != "1" ] && [ "$UUU" != "0" ]; then
	echo "Unknown UUU option: $UUU"
	exit 1
fi

if [ "$SECURE" != "1" ] && [ "$SECURE" != "0" ]; then
	echo "Unknown SECURE option: $SECURE"
	exit 1
fi
# -- END Validate input parameters

# SED Expression to enable HABv4
# First parameter: Input/output filename
sed_habv4 () {
	sed -i -n -E \
        -e '/(^CONFIG_IMX_HAB=)|(^CONFIG_SPL_FIT_SIGNATURE=)|(^CONFIG_ENV_IS_)|(^CONFIG_ENV_OFFSET=)|(^CONFIG_ENV_SIZE=)|(^$)/!p' \
        -e '$aCONFIG_IMX_HAB=y\nCONFIG_IMX_SPL_FIT_FDT_SIGNATURE=y' \
        -e '$aCONFIG_ENV_IS_NOWHERE=y' \
        "$1"
}

# SED Expression to enable AHAB
# First parameter: Input/output filename
sed_ahab () {
    sed -i -n -E \
        -e '/(^CONFIG_AHAB_BOOT=)|(^CONFIG_ENV_IS_)|(^CONFIG_ENV_OFFSET=)|(^CONFIG_ENV_SIZE=)|(^$)/!p' \
        -e '$aCONFIG_AHAB_BOOT=y' \
        -e '$aCONFIG_ENV_IS_NOWHERE=y' \
        "$1"
}

# SED Expression to disable UUU support
# First parameter: Input/output filename
sed_no_uuu () {
    sed -i -n -E \
        -e '/(^CONFIG_FASTBOOT)|(^CONFIG_UDP_FUNCTION_FASTBOOT=)|(^CONFIG_USB_FUNCTION_FASTBOOT=)|(^CONFIG_CMD_FASTBOOT=)|(^CONFIG_CMD_DFU=)/!p' \
        -e '$aCONFIG_DFU=n' \
        -e '$aCONFIG_FASTBOOT=n' \
        -e '$aCONFIG_FASTBOOT_UUU_SUPPORT=n' \
        "$1"
}

UBOOT_BASE="uboot-imx/configs"
if [ ! -d  "$UBOOT_BASE" ]; then
	echo "Cannot find UBoot directory! Please run this script from BSP root."
	exit 1
fi

case $BOARD in
	imx8m)  DEFCONF_IN="$UBOOT_BASE/imx8mq_evk_nt_defconfig" ;;
	imx8mm)	DEFCONF_IN="$UBOOT_BASE/imx8mm_evk_nt_defconfig" ;;
	imx8mn)	DEFCONF_IN="$UBOOT_BASE/imx8mn_evk_nt_defconfig" ;;
	imx8mnddr4)	DEFCONF_IN="$UBOOT_BASE/imx8mn_ddr4_evk_nt_defconfig" ;;
	imx8mp)	DEFCONF_IN="$UBOOT_BASE/imx8mp_evk_nt_defconfig" ;;
	imx8x)	DEFCONF_IN="$UBOOT_BASE/imx8qxp_mek_nt_defconfig" ;;
	imx93)	DEFCONF_IN="$UBOOT_BASE/imx93_11x11_evk_nt_defconfig" ;;
	imx95)	DEFCONF_IN="$UBOOT_BASE/imx95_19x19_evk_nt_defconfig" ;;
	*)		echo "Unknown board! ($BOARD)"; exit 1 ;;
esac

DEFCONF_OUT="${DEFCONF_IN%defconfig}TEMP_defconfig"
echo -ne \
	"# Generated defconfig!\n# UUU=$UUU, SECURE=$SECURE\n# DATE=$(date)\n" > $DEFCONF_OUT
cat $DEFCONF_IN >> $DEFCONF_OUT
if [ "$SECURE" == "1" ]; then
	if [ "$BOARD" == "imx8x" ] || [ "$BOARD" == "imx93" ]  || [ "$BOARD" == "imx95" ]; then
		sed_ahab $DEFCONF_OUT
	else
		sed_habv4 $DEFCONF_OUT
	fi
fi

if [ "$UUU" == "0" ]; then
	sed_no_uuu $DEFCONF_OUT
fi

if [ "$DEBUG" == "1" ]; then
	echo "---------------------------------------------" 1>&2
	echo "Original defconfig: $DEFCONF_IN" 1>&2
	echo "Modified defconfig: $DEFCONF_OUT" 1>&2
	echo "---------------------------------------------" 1>&2
	echo "DIFF original --> temporary:" 1>&2
	diff --color $DEFCONF_IN $DEFCONF_OUT 1>&2
	echo "---------------------------------------------" 1>&2
fi

echo $(basename "$DEFCONF_OUT")