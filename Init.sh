#!/bin/bash
# 
# Copyright 2022 NXP 
# All rights reserved. 
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

GitUserName="NXP Build Server"
GitUserEmail="XXX@nxp.com"

echo "Populating source files from git ..."
git reset --hard

if [ -d "$SUBMODULE_CACHE/modules" ]; then
    echo "Copying existing $SUBMODULE_CACHE/modules to .git/modules."
    cp -r $SUBMODULE_CACHE/modules .git
fi

echo "Populating source files in submodules ..."
git submodule update --init --force --recursive --depth 1


echo "Patching external submodules ..."
patch_dir=$(pwd)/patches;

declare -A patches=( 
    ["imx-atf"]="imx-atf"
    ["imx-mkimage"]="imx-mkimage"
    ["imx-optee-os"]="imx-optee-os"
    ["MU_BASECORE-USB_eHCI"]="mu_platform_nxp/MU_BASECORE"
    ["uboot-imx"]="uboot-imx"
)

for patch_name in "${!patches[@]}";
do
    module_path="${patches[$patch_name]}";
    pushd $module_path;
        git apply $patch_dir/${patch_name}.patch;
        git add -A -f .
        git -c user.name="$GitUserName" -c user.email="$GitUserEmail" commit -m "NXP i.MX BSP support patch" 2>/dev/null || echo "Failed to commit patch to your local copy of $module_path. Fix errors and commit patch to $module_path submodule manually. Shell command: pushd $module_path; git commit -m \"NXP i.MX BSP support patch\"; popd;"
    popd;
done
