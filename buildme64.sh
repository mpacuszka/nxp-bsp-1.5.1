#!/bin/bash
# 
# Copyright 2022 NXP 
#  
# Redistribution and use in source and binary forms, with or without modification, 
# are permitted provided that the following conditions are met: 
#
# Redistributions of source code must retain the above copyright notice, this list 
# of conditions and the following disclaimer. 
#
# Redistributions in binary form must reproduce the above copyright notice, this 
# list of conditions and the following disclaimer in the documentation and/or 
# other materials provided with the distribution. 
#
# Neither the name of the copyright holder nor the names of its 
# contributors may be used to endorse or promote products derived from this 
# software without specific prior written permission. 
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
# 


# File contents:
#
# 1. help string
# 2. enabled features list
# 3. argument check
# 4. arguments to feature list processing
# 5. high assurance boot helper functions
# 6. build board
# 7. main
#   1. echo selected configuration
#   2. for each enabled board:
#       - set environment vars for board selected
#       - call build_board()

# 1. help string
usage() { echo "Usage: $0 [-b BOARD_1] [-b BOARD_2] [-t TARGET_1] .. [-c]"        1>&2
          echo ""                                                                 1>&2
          echo "Description:          "                                           1>&2
          echo "          [-b|-board]"                                            1>&2
          echo "            { all,"                                               1>&2
          echo "              8M, MX8M_EVK,"                                      1>&2
          echo "              8Mm, MX8M_MINI_EVK"                                 1>&2
          echo "              8Mn, MX8M_NANO_EVK "                                1>&2
          echo "              8Mp, MX8M_PLUS_EVK "                                1>&2
          echo "              8X, MX8QXP_MEK"                                     1>&2
          echo "              93, MX93_11X11_EVK }"                               1>&2
          echo ""                                                                 1>&2
          echo "          [-t|-target_app] "                                      1>&2
          echo "            { all, u|uboot, atf, optee, apps|tee_apps,"           1>&2
          echo "              uimg|uboot_image, tools|uefi_tools, uefi,"          1>&2
          echo "              profile_dev, profile_secure, profile_frontpage,"    1>&2
          echo "              secured_efi|secured_uefi}"                          1>&2
          echo ""                                                                 1>&2
          echo "          [-fw|-fw_bin] - Build firmware_uuu.bin"                 1>&2
          echo "                          Not needed with -t all"                 1>&2
          echo "          [-cap|-capsule]"                                        1>&2
          echo "          [-c|-clean]"                                            1>&2
          echo "          [-nu|-no_uuu]"                                          1>&2
          echo "          [-h|-help]"                                             1>&2
          echo "          [-bc|build_configuration]"                              1>&2
          echo "            { release|RELEASE - for Release version of uefi,"     1>&2
          echo "              debug|DEBUG     - for Debug version of uefi }"      1>&2
          echo ""                                                                 1>&2
          echo "Advanced options for experienced users"                           1>&2
          echo "          [-ao|-advance_option]"                                  1>&2
          echo "            { rpmb_reset_fat, rpmb_write_key, no_rpmb_test_key"   1>&2
          echo "              optee_core_v, optee_ta_v}"                          1>&2
          echo ""                                                                 1>&2
          echo "Environment variables:          "                                 1>&2
          echo "          [TARGET_WINDOWS_BSP_DIR] - Path to imx-windows-iot,"    1>&2
          echo "               in which the firmware shall be updated."           1>&2
          echo "          [KEY_ROOT] - Path to custom PKI root"                   1>&2
          echo "------------------------------------------------------------"     1>&2 ;
          exit 1; }


# 2. enabled features list
# features are enabled later by script parameters

script_name=$0
build_8m=0
build_8m_mini=0
build_8m_nano=0
build_8mn_ddr4=0
build_8m_plus=0
build_8qxp=0
build_93=0

clean=0
build_uboot=0
build_atf=0
build_optee=0
build_tee_apps=0
build_uboot_image=0
build_uefi_tools=0
build_uefi=0
build_capsule=0
build_secure=0
rpmb_reset_fat="CFG_RPMB_RESET_FAT=n"
rpmb_write_key="CFG_RPMB_WRITE_KEY=n"
optee_core_v="CFG_TEE_CORE_DEBUG=n TRACE_LEVEL=0 CFG_TEE_CORE_LOG_LEVEL=0"
optee_ta_v="CFG_TA_DEBUG=n CFG_TEE_CORE_TA_TRACE=0 CFG_TEE_TA_LOG_LEVEL=0"
# (OpTEE) Use test key by default
rpmb_key="CFG_RPMB_TESTKEY=y"
optee_cfg_tzdram_start=""

# uboot is by default built with UUU support, the name of output firmware for this build type:
firmware_out_file_name="firmware_uuu"

build_all_selected=0
build_configuration=RELEASE
build_uboot_with_uuu_support=1
build_uefi_profile=DEV

# 3. argument check
# prints help if no arguments

if [ $# -eq 0 ]; then
    echo No arguments specified.
    usage
    exit 0
fi

# 4. arguments to feature list processing
# process arguments one by one in switch-case using shift
#
# variables:
#   - #$
#       - number of arguments left
#   - $1
#       - first argument:   name  (ie. -t    in '-t uboot')
#   - $2
#       - second argument:  value (eg. uboot in '-t uboot')

while test $# -gt 0
do
    case "$1" in

    -c|-clean)
        clean=1
        ;;
    -h|-help)
        usage
        ;;
    -nu|-no_uuu)
        build_uboot_with_uuu_support=0
        # uboot is without UUU support, the name of output firmware will be:
        firmware_out_file_name="firmware"
        ;;
    -cap|-capsule)
        build_capsule=1
        ;;
    -fw)
        build_uboot_image=1
        ;;
    -t|-target_app)
        case "${2}" in
            u|uboot)
                build_uboot=1
                ;;
            atf)
                build_atf=1
                ;;
            tee|optee)
                build_optee=1
                ;;
            apps|tee_apps)
                build_optee=1
                build_tee_apps=1
                ;;
            uimg|uboot_image)
                build_atf=1
                build_uboot=1
                build_optee=1
                build_uboot_image=1
                ;;
            tools|uefi_tools)
                build_uefi_tools=1
                ;;
            efi|uefi)
                build_uefi=1
                ;;
            secured_efi|secured_uefi)
                build_uefi_profile=SECURE
                build_secure=1
                ;;
            profile_secure)
                build_uefi_profile=SECURE
                build_secure=1
                ;;
            profile_dev)
                build_uefi_profile=DEV
                ;;
            profile_frontpage)
                build_uefi_profile=FRONTPAGE
                ;;
            all)
                build_all_selected=1
                build_uboot=1
                build_atf=1
                build_optee=1
                build_tee_apps=1
                build_uboot_image=1
                build_uefi_tools=1
                build_uefi=1
                ;;
            *)
                echo "Unknown option ${2} specified for -b --build" 
                usage
                ;;
        esac
        shift
        ;;
    -b|-board)
        case "${2}" in
            8M|MX8M_EVK)
                build_8m=1
                ;;
            8Mm|MX8M_MINI_EVK)
                build_8m_mini=1
                ;;
            8Mn|MX8M_NANO_EVK)
                build_8m_nano=1
                ;;
            8Mn4|MX8MN_DDR4_EVK)
                build_8mn_ddr4=1
                ;;
            8Mp|MX8M_PLUS_EVK)
                build_8m_plus=1
                ;;
            8X|MX8QXP_MEK)
                build_8qxp=1
                ;;
            93|MX93_11X11_EVK)
                build_93=1
                ;;
            all)
                build_8m=1
                build_8m_mini=1
                build_8m_nano=1
                build_8m_plus=1
                build_8qxp=1
                build_93=1
                ;;
            *)
                echo "Unknown option ${2} specified for -b --build" 
                usage
                ;;
        esac
        shift
        ;;
    -bc|build_configuration)
        case "${2}" in
            release|RELEASE)
                build_configuration=RELEASE
                ;;
            debug|DEBUG)
                build_configuration=DEBUG
                ;;
            *)
            echo "Unknown option ${2} specified for -v"
            usage
            ;;
        esac
        shift
        ;;

    -ao|-advance_option)
        case "${2}" in
            rpmb_reset_fat)
                rpmb_reset_fat="CFG_RPMB_RESET_FAT=y"
                ;;
            rpmb_write_key)
                rpmb_write_key="CFG_RPMB_WRITE_KEY=y"
                ;;
            no_rpmb_test_key)
                rpmb_key="CFG_RPMB_TESTKEY=n"
                ;;
            optee_core_v)
                optee_core_v="CFG_TEE_CORE_DEBUG=y TRACE_LEVEL=2 CFG_TEE_CORE_LOG_LEVEL=2"
                ;;
            optee_ta_v)
                optee_ta_v="CFG_TA_DEBUG=y CFG_TEE_CORE_TA_TRACE=1 CFG_TEE_TA_LOG_LEVEL=2"
                ;;
            optee_core_vv)
                optee_core_v="CFG_TEE_CORE_DEBUG=y TRACE_LEVEL=4 CFG_TEE_CORE_LOG_LEVEL=4"
                ;;
            optee_ta_vv)
                optee_ta_v="CFG_TA_DEBUG=y CFG_TEE_CORE_TA_TRACE=1 CFG_TEE_TA_LOG_LEVEL=4"
                ;;
            *)
                echo "Unknown option ${2} specified for -b --build"
                usage
                ;;
        esac
        shift
        ;;
    *)
        echo "Unknown option ${1} specified"
        usage
        ;;
    esac
    shift
done

# 5. high assurance boot helper functions

export IMX8_REPO_ROOT=$(pwd)
export CST_ROOT=$IMX8_REPO_ROOT/cst

if [ -z "$KEY_ROOT" ]; then
    export KEY_ROOT=$IMX8_REPO_ROOT/imx-windows-iot/build/firmware/test_keys_no_security
    echo "$(tput setaf 1)KEY_ROOT not specified! When using secure build, the resulting binaries will be signed by test keys !!!$(tput sgr 0)"
    KEY_ROOT_AHAB="$KEY_ROOT/ahab"
else
    echo "Using KEY_ROOT at $KEY_ROOT"
    KEY_ROOT_AHAB="$KEY_ROOT"
fi

if [ $build_uboot_image -eq 1 ]; then
    FILE=firmware-imx
    if ! [ -e "$FILE" ]; then
        echo "ERROR: $FILE directory does not exist. Download and extract the iMX firmware from NXP’s website and place it to firmware-imx. See i.MX_Win10_User's_Guide.pdf for instruction how to install required SW."
        exit
    fi
fi

if [ $build_secure -eq 1 ]; then
    FILE=cst
    if ! [ -d "$FILE" ] && ! [ -L "$FILE" ]; then
        echo "ERROR: $FILE directory does not exist. Download and extract the Code Signing Tools (CST) from NXP’s website. See i.MX_Win10_User's_Guide.pdf for instruction how to install required SW."
        exit
    fi
fi

# Requires paths to spl_uboot.log and firmware binary as parameters.
generate_spl_csf () {
    awk -v binary_path="$2" -v indentation=4 '/spl hab block/ { printf "%*sBlocks = %s %s %s \"%s\"\n", indentation, "", $4, $5, $6, binary_path }' $1 \
        | cat $IMX8_REPO_ROOT/imx-windows-iot/build/firmware/csf_templates/mx8/csf_spl.txt.template - \
        | sed -e "s#{KEY_ROOT}#$KEY_ROOT#g" > $MKIMAGE_WORK_DIR/csf_spl.txt
    $CST_ROOT/linux64/bin/cst --o $MKIMAGE_WORK_DIR/csf_spl.bin --i $MKIMAGE_WORK_DIR/csf_spl.txt
}

# Requires paths to spl_uboot.log and firmware binary as parameters.
generate_fit_fdt_csf () {
    awk -v binary_path="$2" -v indentation=4 '/fit-fdt hab block/ { printf "%*sBlocks = %s %s %s \"%s\"\n", indentation, "", $4, $5, $6, binary_path }' $1 \
        | cat $IMX8_REPO_ROOT/imx-windows-iot/build/firmware/csf_templates/mx8/csf_fit_fdt.txt.template - \
        | sed -e "s#{KEY_ROOT}#$KEY_ROOT#g" > $MKIMAGE_WORK_DIR/csf_fit_fdt.txt
    $CST_ROOT/linux64/bin/cst --o $MKIMAGE_WORK_DIR/csf_fit_fdt.bin --i $MKIMAGE_WORK_DIR/csf_fit_fdt.txt
}


# Requires paths to spl_uboot.log, fit_hab.log, and firmware binary as parameters.
generate_fit_csf () {
    { \
        awk -v binary_path="$3" -v indentation=4 \
                        '/sld hab block/ { printf "%*sBlocks = 0x%08x 0x%06x 0x%05x \"%s\"", indentation, "", strtonum($4), strtonum($5), strtonum($6), binary_path }' $1 \
        && awk -v binary_path="$3" -v indentation=12 'BEGIN { ORS="" } \
                        /print_fit_hab/ { x=NR+5;next}  (NR<=x) { printf ", \\\n%*s 0x%08x 0x%06x 0x%05x \"%s\"", indentation, "", strtonum($1), strtonum($2), strtonum($3), binary_path }' $2 \
        && echo ;
    } \
        | cat $IMX8_REPO_ROOT/imx-windows-iot/build/firmware/csf_templates/mx8/csf_fit.txt.template - \
        | sed -e "s#{KEY_ROOT}#$KEY_ROOT#g" > $MKIMAGE_WORK_DIR/csf_fit.txt
    $CST_ROOT/linux64/bin/cst --o $MKIMAGE_WORK_DIR/csf_fit.bin --i $MKIMAGE_WORK_DIR/csf_fit.txt
}

# Requires paths to UEFI firmware binary as parameter.
generate_uefi_csf () {
    cat $IMX8_REPO_ROOT/imx-windows-iot/build/firmware/csf_templates/mx8/csf_uefi.txt.template \
        | sed -e "s#{KEY_ROOT}#$KEY_ROOT#g" > csf_uefi.txt
    UEFI_SIZE=$(stat --printf=%s $1) ;\
    INDENTATION=4 ;\
    printf "%*sBlocks = 0x40480000 0x00000 0x%x \"$1\"\n" $INDENTATION "" $UEFI_SIZE >> csf_uefi.txt
}

# Requires paths to spl_uboot.log and u-boot firmware binary as parameters.
sign_uboot_binary () {
    SIGNED_UBOOT_BIN=$(dirname $2)/signed_$(basename $2) ;\
    cp $2 $SIGNED_UBOOT_BIN ;\
    SPL_CSF_OFF=$(awk '/^ csf_off/ { print $2 }' $1) ;\
    FIT_FDT_CSF_OFF=$(awk '/fit-fdt csf_off/ { print $3 }' $1) ;\
    FIT_CSF_OFF=$(awk '/ sld_csf_off/ { print $2 }' $1) ;\
    dd if=$MKIMAGE_WORK_DIR/csf_spl.bin of=$SIGNED_UBOOT_BIN seek=$(printf '%d' $SPL_CSF_OFF) bs=1 conv=notrunc ;\
    dd if=$MKIMAGE_WORK_DIR/csf_fit.bin of=$SIGNED_UBOOT_BIN seek=$(printf '%d' $FIT_CSF_OFF) bs=1 conv=notrunc
    # Needs to be done after FIT signiture is entered in flash.bin as it is in FIT FDT sidned data
    # more details in https://github.com/nxp-imx/uboot-imx/commit/25fdc42caa30faa586a277162ae5373d3e2bc2be
    # and https://jira.sw.nxp.com/browse/LFU-573
    generate_fit_fdt_csf $1 $SIGNED_UBOOT_BIN || exit $?
    dd if=$MKIMAGE_WORK_DIR/csf_fit_fdt.bin of=$SIGNED_UBOOT_BIN seek=$(printf '%d' $FIT_FDT_CSF_OFF) bs=1 conv=notrunc ;\
}

# Requires UEFI size as parameters.
generate_ivt_for_uefi () {
    cat $IMX8_REPO_ROOT/imx-windows-iot/build/firmware/csf_templates/mx8/genIVT-template.pl | sed -e "s#{UEFI_PADDED_SIZE}#$1#g" > genIVT.pl
    perl genIVT.pl
}

# Requires paths to UEFI firmware binary as parameter. Calculation of the IVT position/offset is derived from uefi binary length read out from uefi.fit.
sign_uefi_binary () {
    SIGNED_UEFI_BIN=$(dirname $1)/signed_$(basename $1) ;\
    cp $1 $SIGNED_UEFI_BIN ;\
    UEFI_SIZE=$(printf "0x%s\n" $(od uefi.fit --endian=big --skip-bytes 12 --read-bytes 4 --width=4 --format=x4 --address-radix=none)) ;\
    UEFI_PADDED_SIZE=$(printf "0x%x" $(( ( ${UEFI_SIZE} + 0x40 + 0xfff ) & ~0xfff ))) ;\
    truncate -s $((${UEFI_PADDED_SIZE})) $SIGNED_UEFI_BIN;\
    generate_ivt_for_uefi $UEFI_PADDED_SIZE ;\
    cat ivt.bin >> $SIGNED_UEFI_BIN ;\
    generate_uefi_csf $SIGNED_UEFI_BIN ;\
    $CST_ROOT/linux64/bin/cst -i csf_uefi.txt -o csf_uefi.bin ;\
    cat csf_uefi.bin >> $SIGNED_UEFI_BIN;\
    echo "UEFI SIZE: $UEFI_SIZE padded: $UEFI_PADDED_SIZE"
}

# Returns version string from iMX8CommonDsc.inc in hex format: 2022-01-03=>0x07E60103
get_firmware_version()
{
    local DSCFILE=mu_platform_nxp/Silicon/ARM/NXP/iMX8Pkg/iMX8CommonDsc.inc
    local tag=$1
    local value=`cat ${DSCFILE} | grep "^ *$tag *=" | cut -d'=' -f2`

    # Convert version string to hex number: 2022-01-03=>0x07E60103
    local numbers=(${value//-/ }) #Convert to array 2022,01,03
    local result=0
    for i in "${numbers[@]}"; do
        result=$((result<<8))
        result=$((result + 10#$i))  #10# ... converts bash string to integer in decadic, otherwise 0x is considered octal
    done
    local result_string=$(printf "0x%08x" $result)
    echo $result_string
}

# 6. definition of build board function

build_board () {
    if [ -z "$AARCH64_TOOLCHAIN_PATH" ] ; then
        echo "AARCH64_TOOLCHAIN_PATH is not set."
        echo "Example: export AARCH64_TOOLCHAIN_PATH=~/gcc-linaro-7.4.1-2019.02-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-"
        exit
    fi

    # Prepare env vars
    export CROSS_COMPILE=$AARCH64_TOOLCHAIN_PATH
    export CROSS_COMPILE64=$AARCH64_TOOLCHAIN_PATH
    export GCC5_AARCH64_PREFIX=$AARCH64_TOOLCHAIN_PATH
    export ARCH=arm64

    if [ -d "$TARGET_WINDOWS_BSP_DIR/components/Arm64BootFirmware" ]; then
        TARGET_WINDOWS_BSP_PREFIX=$TARGET_WINDOWS_BSP_DIR
    else
        TARGET_WINDOWS_BSP_PREFIX=imx-windows-iot
    fi
    TARGET_FIRMWARE_COMPONENTS_DIR="imx-windows-iot/components/Arm64BootFirmware/$bsp_folder" # eg. imx-windows-iot/components/Arm64BootFirmware/MX8M_EVK
    TARGET_FIRMWARE_DIR="${TARGET_WINDOWS_BSP_PREFIX}/BSP/firmware/$bsp_folder" # eg. imx-windows-iot/BSP/firmware/MX8M_EVK

    # Build U-Boot
    if [ $build_uboot -eq 1 ]; then
        if [ $build_uboot_with_uuu_support -eq 1 ]; then
            echo "$(tput setaf 1) Building uBoot image with uuu support! Defconfig: $uboot_defconfig $(tput sgr 0)"
        fi
        pushd uboot-imx/ || exit $?
        if [ $clean -eq 1 ]; then
            make distclean
            make clean
        fi
        make $uboot_defconfig || exit $?
        make -s -j12 spl/u-boot-spl.bin u-boot.bin || exit $?
        popd
    fi

    if [ $build_atf -eq 1 ]; then
        pushd imx-atf || exit $?
        if [ $clean -eq 1 ]; then
            make PLAT=$atf_plat clean || exit $?
        fi
        make -s PLAT=$atf_plat SPD=opteed WINDOWS_BSP=y bl31 || exit $?
        popd
    fi

    if [ $build_optee -eq 1 ]; then
        export -n CROSS_COMPILE
        export -n ARCH
        pushd imx-optee-os || exit $?
        if [ $clean -eq 1 ]; then
            make clean PLATFORM=imx PLATFORM_FLAVOR=$optee_plat || exit $?
            rm -r ./out
        fi
        echo "make -s -j12 PLATFORM=imx PLATFORM_FLAVOR=$optee_plat WINDOWS_BSP=y $optee_cfg_tzdram_start $rpmb_write_key $rpmb_reset_fat $optee_core_v $rpmb_key CFG_RPMB_FS=y CFG_REE_FS=n CFG_IMXCRYPT=y CFG_CORE_HEAP_SIZE=131072 CFG_RPMB_FS_RD_ENTRIES=30 CFG_RPMB_FS_CACHE_ENTRIES=60"
        make -s -j12 PLATFORM=imx \
            PLATFORM_FLAVOR=$optee_plat \
            WINDOWS_BSP=y \
            $optee_cfg_tzdram_start \
            $rpmb_write_key \
            $rpmb_reset_fat \
            $optee_core_v \
            $rpmb_key \
            CFG_RPMB_FS=y \
            CFG_REE_FS=n \
            CFG_IMXCRYPT=y \
            CFG_CORE_HEAP_SIZE=131072 \
            CFG_RPMB_FS_RD_ENTRIES=30 \
            CFG_RPMB_FS_CACHE_ENTRIES=60 || exit $?

        popd
        export CROSS_COMPILE=$AARCH64_TOOLCHAIN_PATH
        export ARCH=arm64
    fi

    if [ $build_tee_apps -eq 1 ]; then
        export TA_CROSS_COMPILE=$AARCH64_TOOLCHAIN_PATH
        export TA_DEV_KIT_DIR=$IMX8_REPO_ROOT/imx-optee-os/out/arm-plat-imx/export-ta_arm64
        export TA_CPU=cortex-a53

        # Apply patches on wolfssl subrepo in order to fix build error caused by fall_through attribute decleration
        pushd MSRSec/external/wolfssl || exit $?
        git apply $IMX8_REPO_ROOT/patches/fall_through_issue.patch
        popd
        pushd MSRSec/external/ms-tpm-20-ref/external/wolfssl || exit $?
        git apply $IMX8_REPO_ROOT/patches/fall_through_issue.patch
        popd

        pushd MSRSec/TAs/optee_ta || exit $?
        if [ $clean -eq 1 ]; then
            rm -r ./out
        fi
        echo "CFG_TA_OPTEE_CORE_API_COMPAT_1_1=y CFG_ARM64_ta_arm64=y $optee_ta_v CFG_FTPM_USE_WOLF=y CFG_AUTHVARS_USE_WOLF=y"
        make CFG_TA_OPTEE_CORE_API_COMPAT_1_1=y CFG_ARM64_ta_arm64=y $optee_ta_v CFG_FTPM_USE_WOLF=y CFG_AUTHVARS_USE_WOLF=y || exit $?
        popd

        mkdir -p mu_platform_nxp/Microsoft/OpteeClientPkg/Bin/AuthvarsTa/Arm64/Test || exit $?
        mkdir -p mu_platform_nxp/Microsoft/OpteeClientPkg/Bin/fTpmTa/Arm64/Test || exit $?
        cp MSRSec/TAs/optee_ta/out/AuthVars/2d57c0f7-bddf-48ea-832f-d84a1a219301.ta  mu_platform_nxp/Microsoft/OpteeClientPkg/Bin/AuthvarsTa/Arm64/Test/ && \
        cp MSRSec/TAs/optee_ta/out/AuthVars/2d57c0f7-bddf-48ea-832f-d84a1a219301.elf mu_platform_nxp/Microsoft/OpteeClientPkg/Bin/AuthvarsTa/Arm64/Test/ && \
        cp MSRSec/TAs/optee_ta/out/fTPM/bc50d971-d4c9-42c4-82cb-343fb7f37896.ta mu_platform_nxp/Microsoft/OpteeClientPkg/Bin/fTpmTa/Arm64/Test/     && \
        cp MSRSec/TAs/optee_ta/out/fTPM/bc50d971-d4c9-42c4-82cb-343fb7f37896.elf mu_platform_nxp/Microsoft/OpteeClientPkg/Bin/fTpmTa/Arm64/Test/     || exit $?

        export -n TA_CROSS_COMPILE
        export -n TA_DEV_KIT_DIR
        export -n TA_CPU
    fi

    if [ $build_uefi_tools -eq 1 ]; then
        pushd mu_platform_nxp && \
        cd MU_BASECORE        || exit $?
        if [ $clean -eq 1 ]; then
            cd BaseTools      && \
            make clean        || exit $?
            cd ..
        fi
        make -s -C BaseTools  || exit $?
        cd ..
        popd
    fi

    if [ $build_uefi -eq 1 ]; then
        pushd mu_platform_nxp || exit $?
        if [ $clean -eq 1 ]; then
            rm -rf Build
            rm -rf Conf
        else
            # Incremental build fix: rm -f Build/"${uefi_folder}"/${build_configuration}_GCC5/FV/Ffs/4E09D4EF-743F-4589-B5EF-8AC493F9DBE2ArmPlatformPrePiUniCore/4E09D4EF-743F-4589-B5EF-8AC493F9DBE2.efi
            rm -f Build/"${uefi_folder}"/${build_configuration}_GCC5/FV/Ffs/*ArmPlatformPrePiUniCore/*.efi
        fi
        export GCC5_AARCH64_PREFIX=$AARCH64_TOOLCHAIN_PATH

        python3 NXP/"${uefi_folder}"/PlatformBuild.py --update || exit $?
        python3 NXP/"${uefi_folder}"/PlatformBuild.py -v TARGET=$build_configuration \
        PROFILE=${build_uefi_profile} MAX_CONCURRENT_THREAD_NUMBER=10 TOOL_CHAIN_TAG=GCC5 BUILDREPORTING=TRUE BUILDREPORT_TYPES="PCD" || exit $?

        pushd Build/"${uefi_folder}"/${build_configuration}_GCC5/FV || exit $?
        # compress the uefi binary, keep the original file (-k)
        rm -f IMXBOARD_EFI.fd.gz
        gzip -k IMXBOARD_EFI.fd
        # make the FIT image
        if [ $atf_plat == "imx93" ]; then
            cp $IMX8_REPO_ROOT/imx-windows-iot/build/firmware/its/uefi_imx93_unsigned.its .                   && \
            $IMX8_REPO_ROOT/uboot-imx/tools/mkimage -f uefi_imx93_unsigned.its -r uefi.fit || exit $?
        elif [ $atf_plat == "imx8qx" ]; then
            cp $IMX8_REPO_ROOT/imx-windows-iot/build/firmware/its/uefi_imx8x_unsigned.its .                   && \
            $IMX8_REPO_ROOT/uboot-imx/tools/mkimage -f uefi_imx8x_unsigned.its -r uefi.fit || exit $?
        else
            cp $IMX8_REPO_ROOT/imx-windows-iot/build/firmware/its/uefi_imx8m_unsigned.its .                   && \
            $IMX8_REPO_ROOT/uboot-imx/tools/mkimage -f uefi_imx8m_unsigned.its -r uefi.fit || exit $?
        fi

        # copy UEFI fit into mkimage directory (+ add signature if required)
        # iMX8QX UEFI is signed at container level
        if [ $build_secure -eq 1 ] && [ $atf_plat != "imx8qx" ]; then
            sign_uefi_binary uefi.fit || exit $?
            cp -f signed_uefi.fit ${MKIMAGE_WORK_DIR}/uefi.fit || exit $?
        else
            cp -f uefi.fit ${MKIMAGE_WORK_DIR} || exit $?
        fi
        popd
        popd
        cp mu_platform_nxp/Build/"${uefi_folder}"/${build_configuration}_GCC5/FV/uefi.fit ${TARGET_FIRMWARE_COMPONENTS_DIR}/uefi.fit || exit $?
    fi

    if [ $build_uboot_image -eq 1 ]; then
        if [ $mkimage_SOC == "iMX8QX" ]; then
            cp -f ${TARGET_FIRMWARE_COMPONENTS_DIR}/mx8qx*-ahab-container.img $MKIMAGE_WORK_DIR && \
            cp -f uboot-imx/u-boot.bin  $MKIMAGE_WORK_DIR                                       && \
            cp -f ${TARGET_FIRMWARE_COMPONENTS_DIR}/scfw_tcm.bin $MKIMAGE_WORK_DIR            || exit $?
        elif [ $mkimage_SOC == "iMX9" ]; then
            cp -f ${TARGET_FIRMWARE_COMPONENTS_DIR}/mx93a*-ahab-container.img $MKIMAGE_WORK_DIR && \
            cp -f uboot-imx/u-boot.bin  $MKIMAGE_WORK_DIR                                       && \
            cp -f firmware-imx/firmware/ddr/synopsys/lpddr4_imem_*.bin $MKIMAGE_WORK_DIR        && \
            cp -f firmware-imx/firmware/ddr/synopsys/lpddr4_dmem_*.bin $MKIMAGE_WORK_DIR      || exit $?
        else
            cp -f firmware-imx/firmware/ddr/synopsys/lpddr4_pmu_train_*.bin $MKIMAGE_WORK_DIR && \
            cp -f firmware-imx/firmware/ddr/synopsys/ddr4_*.bin $MKIMAGE_WORK_DIR             && \
            cp -f firmware-imx/firmware/hdmi/cadence/signed_hdmi_imx8m.bin $MKIMAGE_WORK_DIR  && \
            cp -f uboot-imx/u-boot-nodtb.bin  $MKIMAGE_WORK_DIR                    && \
            cp -f uboot-imx/arch/arm/dts/"$uboot_dtb" $MKIMAGE_WORK_DIR            || exit $?
        fi
        cp -f imx-optee-os/out/arm-plat-imx/core/tee-raw.bin $MKIMAGE_WORK_DIR/tee.bin    && \
        cp -f imx-atf/build/"$atf_plat"/release/bl31.bin $MKIMAGE_WORK_DIR                && \
        cp -f uboot-imx/spl/u-boot-spl.bin $MKIMAGE_WORK_DIR                   && \
        cp -f uboot-imx/tools/mkimage $MKIMAGE_WORK_DIR/mkimage_uboot          || exit $?

        pushd imx-mkimage || exit $?
        if [ $clean -eq 1 ]; then
            make SOC=$mkimage_SOC clean
        fi
        if [ $build_8mn_ddr4 -eq 1 ]; then
            make SOC=$mkimage_SOC flash_ddr4_evk 2>&1 | tee $MKIMAGE_WORK_DIR/spl_uboot.log || exit $?
        elif [ $atf_plat == "imx93" ]; then
            echo "Building container 3"
            make SOC=$mkimage_SOC REV=A1 u-boot-atf-optee-uefi-container.img 2>&1 > $MKIMAGE_WORK_DIR/container3.log || exit $?
            if [ $build_secure -eq 1 ]; then
                pushd $MKIMAGE_WORK_DIR
                # Parse mkimage log, generate CSF and sign container 3
                CONTAINER_PATH=u-boot-atf-optee-uefi-container.img \
                SRK_TABLE_PATH="$KEY_ROOT_AHAB/crts/SRKtable.bin" \
                CERT_PEM_PATH="$KEY_ROOT_AHAB/crts/SRK1_sha384_secp384r1_v3_usr_crt.pem" \
                LOGFILE_PATH=container3.log \
                ./gen_csf.sh > csf_container3.txt
                $CST_ROOT/linux64/bin/cst -i csf_container3.txt -o container3.bin
                mv -f container3.bin u-boot-atf-optee-uefi-container.img
                popd
            fi
            echo "Building container 1+2"
            make SOC=$mkimage_SOC REV=A1 flash_singleboot_uefi 2>&1 | tee $MKIMAGE_WORK_DIR/container12.log || exit $?
            if [ $build_secure -eq 1 ]; then
                pushd $MKIMAGE_WORK_DIR
                # Parse mkimage log, generate CSF and sign container 1+2
                CONTAINER_PATH=flash.bin \
                SRK_TABLE_PATH="$KEY_ROOT_AHAB/crts/SRKtable.bin" \
                CERT_PEM_PATH="$KEY_ROOT_AHAB/crts/SRK1_sha384_secp384r1_v3_usr_crt.pem" \
                LOGFILE_PATH=container12.log \
                ./gen_csf.sh > csf_container12.txt
                $CST_ROOT/linux64/bin/cst -i csf_container12.txt -o signed_flash.bin
                popd
            fi
        elif [ $atf_plat == "imx8qx" ]; then
            # Flash image consists of 3 containers - [SECO] + [SCFW + SPL] + [ATF + OP-TEE + UBoot + UEFI]
            # 1. container is provided and signed by NXP
            # Because of how mkimage works, it is easier first to separately create and sign the 3. container, then
            # bundle 1. + 2. + 3. container, and then sign the 2. container within the bundle
            echo "Building container 3"
            make SOC=$mkimage_SOC REV=C0 u-boot-atf-optee-uefi-container.img 2>&1 > $MKIMAGE_WORK_DIR/container3.log || exit $?
            if [ $build_secure -eq 1 ]; then
                pushd $MKIMAGE_WORK_DIR
                # Parse mkimage log, generate CSF and sign container 3
                CONTAINER_PATH=u-boot-atf-optee-uefi-container.img \
                SRK_TABLE_PATH="$KEY_ROOT_AHAB/crts/SRKtable.bin" \
                CERT_PEM_PATH="$KEY_ROOT_AHAB/crts/SRK1_sha384_secp384r1_v3_usr_crt.pem" \
                LOGFILE_PATH=container3.log \
                ./gen_qxp_csf.sh > csf_container3.txt
                $CST_ROOT/linux64/bin/cst -i csf_container3.txt -o container3.bin
                mv -f container3.bin u-boot-atf-optee-uefi-container.img
                popd
            fi
            echo "Building container 1+2"
            make SOC=$mkimage_SOC REV=C0 flash_spl_uefi 2>&1 > $MKIMAGE_WORK_DIR/container12.log || exit $?
            if [ $build_secure -eq 1 ]; then
                pushd $MKIMAGE_WORK_DIR
                # Parse mkimage log, generate CSF and sign container 1+2
                CONTAINER_PATH=flash.bin \
                SRK_TABLE_PATH="$KEY_ROOT_AHAB/crts/SRKtable.bin" \
                CERT_PEM_PATH="$KEY_ROOT_AHAB/crts/SRK1_sha384_secp384r1_v3_usr_crt.pem" \
                LOGFILE_PATH=container12.log \
                ./gen_qxp_csf.sh > csf_container12.txt
                $CST_ROOT/linux64/bin/cst -i csf_container12.txt -o signed_flash.bin
                popd
            fi
        else
            make SOC=$mkimage_SOC flash_hdmi_spl_uboot 2>&1 | tee $MKIMAGE_WORK_DIR/spl_uboot.log || exit $?
        fi
        popd
        cp $MKIMAGE_WORK_DIR/flash.bin ${TARGET_FIRMWARE_COMPONENTS_DIR}/flash.bin || exit $?
        echo "flash.bin copied to:" ${TARGET_FIRMWARE_COMPONENTS_DIR}/flash.bin || exit $?
    fi

    if [ $build_secure -eq 1 ] && [ $atf_plat != "imx8qx" ] && [ $atf_plat != "imx93" ]; then
        pushd imx-mkimage || exit $?
        rm -f print_fit_hab
        if [ $build_8mn_ddr4 -eq 1 ]; then
            make SOC=$mkimage_SOC print_fit_hab_ddr4 > $MKIMAGE_WORK_DIR/fit_hab.log || exit $?
        else
            make SOC=$mkimage_SOC print_fit_hab > $MKIMAGE_WORK_DIR/fit_hab.log || exit $?
        fi
        generate_spl_csf $MKIMAGE_WORK_DIR/spl_uboot.log $MKIMAGE_WORK_DIR/flash.bin || exit $?
        generate_fit_csf $MKIMAGE_WORK_DIR/spl_uboot.log $MKIMAGE_WORK_DIR/fit_hab.log $MKIMAGE_WORK_DIR/flash.bin || exit $?
        sign_uboot_binary $MKIMAGE_WORK_DIR/spl_uboot.log $MKIMAGE_WORK_DIR/flash.bin || exit $?
        popd
    fi
    if [ $build_secure -eq 1 ]; then
        cp ${MKIMAGE_WORK_DIR}/signed_flash.bin ${TARGET_FIRMWARE_COMPONENTS_DIR}/signed_flash.bin || exit $?
        echo "signed_flash.bin copied to:" ${TARGET_FIRMWARE_COMPONENTS_DIR}/signed_flash.bin || exit $?
    fi

    echo "-----------------------------------------------------------------"

    if [ $build_secure -ne 1 ]; then    
        echo "${firmware_out_file_name}.bin created as:" ${TARGET_FIRMWARE_DIR}/${firmware_out_file_name}.bin || exit $?
        dd if=${TARGET_FIRMWARE_COMPONENTS_DIR}/flash.bin of=${TARGET_FIRMWARE_DIR}/${firmware_out_file_name}.bin bs=512 || exit $?
    else
        echo "signed_firmware.bin created as:" ${TARGET_FIRMWARE_DIR}/signed_${firmware_out_file_name}.bin || exit $?
        dd if=${TARGET_FIRMWARE_COMPONENTS_DIR}/signed_flash.bin of=${TARGET_FIRMWARE_DIR}/signed_${firmware_out_file_name}.bin bs=512 || exit $?
    fi

    if [ $build_capsule -eq 1 ]; then
        echo "building capsule from ${firmware_out_file_name}.bin" || exit $?
        PYTHON_TOOLS=mu_platform_nxp/MU_BASECORE/BaseTools/Source/Python
        export PYTHONPATH=$PYTHONPATH:$PYTHON_TOOLS
        capsule_guid='62AF20A3-7016-424A-9BF8-9CCC86584090'
        fw_ver=$(get_firmware_version FIRMWARE_VER)
        lsv=$(get_firmware_version FIRMWARE_VER_LOWEST_SUPPORTED)

        python3 $PYTHON_TOOLS/Capsule/GenerateCapsule.py -v --encode ${TARGET_FIRMWARE_DIR}/${firmware_out_file_name}.bin --fw-version ${fw_ver} --lsv ${lsv} \
        --guid $capsule_guid --monotonic-count 2 --signer-private-cert $PYTHON_TOOLS/Pkcs7Sign/TestCert.pem \
        --other-public-cert $PYTHON_TOOLS/Pkcs7Sign/TestSub.pub.pem --trusted-public-cert $PYTHON_TOOLS/Pkcs7Sign/TestRoot.pub.pem -o ${TARGET_FIRMWARE_DIR}/FirmwareCapsuleIMX.cap || exit $?
    fi
    echo "Done"
}

# 7. main
# the main function of this script
#
#   7.1. echo selected configuration

echo building bootloader image for Windows 10 IOT Enterprise
echo "Build Configuration: " $build_configuration
echo "UEFI build profile:  " $build_uefi_profile
echo -----------------
echo 
echo Selected configuration:
echo "clean              $clean"
echo "build_uboot        $build_uboot"
echo "build_atf          $build_atf"
echo "build_optee        $build_optee"
echo "build_tee_apps     $build_tee_apps"
echo "build_uboot_image  $build_uboot_image"
echo "build_uefi_tools   $build_uefi_tools"
echo "build_uefi         $build_uefi"
echo "build_secure       $build_secure"
echo -----------------
if [ $build_all_selected -eq 0 ]; then
    echo 
    echo Warning \"-t all\" not set! Build of reduced feature set should be used for development purposes only and requires \"$script_name -t all\" to be run first.
fi
echo
echo Building..
echo

#  7.2. for each board
#       - set environment vars for board selected
#       - call build_board()
#
#  > Note: '$?' contains return value

# available configs bellow:
#   1) i.MX 8MQ EVK config
#   2) i.MX 8M Mini EVK config
#   3) i.MX 8M Nano EVK config
#   4) i.MX 8M Plus EVK config
#   5) i.MX 8X Plus CPU config
#   6) i.MX 93 EVK config

# i.MX 8MQ EVK config
if [ $build_8m -eq 1 ]; then
    echo "Board type IMX8M EVK"
    bsp_folder="MX8M_EVK"
    uefi_folder="MX8M_EVK"
    uboot_defconfig=`scripts/PrepareUBootDefconfig.sh imx8M "$build_uboot_with_uuu_support" "$build_secure"`
    atf_plat="imx8mq"
    optee_plat="mx8mqevk"
    uboot_dtb="imx8mq-evk.dtb"
    mkimage_SOC="iMX8M"
    export MKIMAGE_WORK_DIR=$IMX8_REPO_ROOT/imx-mkimage/iMX8M
    uefi_offset_kb=1940
    optee_cfg_tzdram_start="CFG_TZDRAM_START=0xFE000000"

    build_board
    CMD_RET=$?
    echo "$mkimage_SOC Build has returned $CMD_RET !"
    BUILD_RV=$((BUILD_RV + CMD_RET))
fi

# i.MX 8M Mini EVK config
if [ $build_8m_mini -eq 1 ]; then
    echo "Board type IMX8MM EVK"
    bsp_folder="MX8M_MINI_EVK"
    uefi_folder="MX8M_MINI_EVK"
    uboot_defconfig=`scripts/PrepareUBootDefconfig.sh imx8Mm "$build_uboot_with_uuu_support" "$build_secure"`
    atf_plat="imx8mm"
    optee_plat="mx8mmevk"
    uboot_dtb="imx8mm-evk.dtb"
    mkimage_SOC="iMX8MM"
    export MKIMAGE_WORK_DIR=$IMX8_REPO_ROOT/imx-mkimage/iMX8M
    uefi_offset_kb=1940
    optee_cfg_tzdram_start="CFG_TZDRAM_START=0xBE000000"

    build_board
    CMD_RET=$?
    echo "$mkimage_SOC Build has returned $CMD_RET !"
    BUILD_RV=$((BUILD_RV + CMD_RET))
fi

# i.MX 8M Nano EVK config
if [ $build_8m_nano -eq 1 ]; then
    echo "Board type IMX8MN EVK"
    bsp_folder="MX8M_NANO_EVK"
    uefi_folder=${bsp_folder}
    uboot_defconfig=`scripts/PrepareUBootDefconfig.sh imx8Mn "$build_uboot_with_uuu_support" "$build_secure"`
    atf_plat="imx8mn"
    optee_plat="mx8mnevk"
    uboot_dtb="imx8mn-evk.dtb"
    mkimage_SOC="iMX8MN"
    export MKIMAGE_WORK_DIR=$IMX8_REPO_ROOT/imx-mkimage/iMX8M
    uefi_offset_kb=1940
    optee_cfg_tzdram_start="CFG_TZDRAM_START=0xBE000000"

    build_board
    CMD_RET=$?
    echo "$mkimage_SOC Build has returned $CMD_RET !"
        BUILD_RV=$((BUILD_RV + CMD_RET))
fi

# i.MX 8M Nano DDR4 EVK config
if [ $build_8mn_ddr4 -eq 1 ]; then
    echo "Board type IMX8MN DDR4 EVK"
    bsp_folder="MX8M_NANO_EVK"
    uefi_folder=${bsp_folder}
    uboot_defconfig=`scripts/PrepareUBootDefconfig.sh imx8Mnddr4 "$build_uboot_with_uuu_support" "$build_secure"`
    atf_plat="imx8mn"
    optee_plat="mx8mnevk"
    uboot_dtb="imx8mn-ddr4-evk.dtb"
    mkimage_SOC="iMX8MN"
    export MKIMAGE_WORK_DIR=$IMX8_REPO_ROOT/imx-mkimage/iMX8M
    uefi_offset_kb=1940
    optee_cfg_tzdram_start="CFG_TZDRAM_START=0xBE000000"

    build_board
    CMD_RET=$?
    echo "$mkimage_SOC Build has returned $CMD_RET !"
    BUILD_RV=$((BUILD_RV + CMD_RET))
fi

# i.MX 8M Plus EVK config
if [ $build_8m_plus -eq 1 ]; then
    echo "Board type IMX8MP EVK"
    bsp_folder="MX8M_PLUS_EVK"
    uefi_folder=${bsp_folder}
    uboot_defconfig=`scripts/PrepareUBootDefconfig.sh imx8Mp "$build_uboot_with_uuu_support" "$build_secure"`
    atf_plat="imx8mp"
    optee_plat="mx8mpevk"
    uboot_dtb="imx8mp-evk.dtb"
    mkimage_SOC="iMX8MP"
    export MKIMAGE_WORK_DIR=$IMX8_REPO_ROOT/imx-mkimage/iMX8M
    uefi_offset_kb=1940
    optee_cfg_tzdram_start="CFG_TZDRAM_START=0xFE000000"

    build_board
    CMD_RET=$?
    echo "$mkimage_SOC Build has returned $CMD_RET !"
    BUILD_RV=$((BUILD_RV + CMD_RET))
fi

# i.MX8X Plus CPU config
if [ $build_8qxp -eq 1 ]; then
echo "Board type IMX8QXP MEK"
    bsp_folder="MX8QXP_MEK"
    uefi_folder=${bsp_folder}
    uboot_defconfig=`scripts/PrepareUBootDefconfig.sh imx8X "$build_uboot_with_uuu_support" "$build_secure"`
    atf_plat="imx8qx"
    optee_plat="mx8qxpmek"
    uboot_dtb="fsl-imx8qxp-mek.dtb"
    mkimage_SOC="iMX8QX"
    export MKIMAGE_WORK_DIR=$IMX8_REPO_ROOT/imx-mkimage/${mkimage_SOC}
    uefi_offset_kb=1940
    optee_cfg_tzdram_start="CFG_TZDRAM_START=0xFE000000"

    build_board
    CMD_RET=$?
    echo "$mkimage_SOC Build has returned $CMD_RET !"
    BUILD_RV=$((BUILD_RV + CMD_RET))
fi

# i.MX 93 11x11 EVK config
if [ $build_93 -eq 1 ]; then
echo "Board type IMX93 11x11 EVK"
    bsp_folder="MX93_11X11_EVK"
    uefi_folder=${bsp_folder}
    uboot_defconfig=`scripts/PrepareUBootDefconfig.sh imx93 "$build_uboot_with_uuu_support" "$build_secure"`
    atf_plat="imx93"
    optee_plat="mx93evk"
    uboot_dtb="imx93-11x11-evk.dts"
    mkimage_SOC="iMX9"
    export MKIMAGE_WORK_DIR=$IMX8_REPO_ROOT/imx-mkimage/${mkimage_SOC}
    uefi_offset_kb=1940
    optee_cfg_tzdram_start="CFG_TZDRAM_START=0x80200000"

    build_board
    CMD_RET=$?
    echo "$mkimage_SOC Build has returned $CMD_RET !"
    BUILD_RV=$((BUILD_RV + CMD_RET))
fi

echo "Total failures: $BUILD_RV"
exit $BUILD_RV
