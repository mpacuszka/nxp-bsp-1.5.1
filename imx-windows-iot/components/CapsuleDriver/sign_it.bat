:: Copyright 2023 NXP 
::  
:: Redistribution and use in source and binary forms, with or without modification, 
:: are permitted provided that the following conditions are met: 
::
:: Redistributions of source code must retain the above copyright notice, this list 
:: of conditions and the following disclaimer. 
::
:: Redistributions in binary form must reproduce the above copyright notice, this 
:: list of conditions and the following disclaimer in the documentation and/or 
:: other materials provided with the distribution. 
::
:: Neither the name of the copyright holder nor the names of its 
:: contributors may be used to endorse or promote products derived from this 
:: software without specific prior written permission. 
::
:: THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
:: ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
:: WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
:: DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
:: ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
:: (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
:: LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
:: ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
:: (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
:: SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
:: 

@echo off
REM Copy ./imx-windows-iot/BSP/firmware/NXPEVK_*/FirmwareCapsuleIMX.cap to ./firmware.bin
REM Created based on https://docs.microsoft.com/en-us/windows-hardware/drivers/bringup/authoring-an-update-driver-package

@echo RUNNING: makecert.exe -r -pe -a sha256 -eku 1.3.6.1.5.5.7.3.3 -n CN=NXP -sv fwu.pvk fwu.cer
"c:\Program Files (x86)\Windows Kits\10\bin\10.0.19041.0\x86\makecert.exe" -r -pe -a sha256 -eku 1.3.6.1.5.5.7.3.3 -n CN=NXP -sv fwu.pvk fwu.cer

@echo RUNNING: pvk2pfx.exe -pvk fwu.pvk -spc fwu.cer -pi imx8 -spc fwu.cer -pfx fwu.pfx
"c:\Program Files (x86)\Windows Kits\10\bin\10.0.19041.0\x86\pvk2pfx.exe" -pvk fwu.pvk -spc fwu.cer -pi imx8 -spc fwu.cer -pfx fwu.pfx

@echo RUNNING: inf2cat.exe /driver:. /os:10_VB_ARM64 /verbose
"c:\Program Files (x86)\Windows Kits\10\bin\10.0.19041.0\x86\inf2cat.exe" /driver:. /os:10_VB_ARM64 /verbose

@echo RUNNING: signtool.exe sign /fd sha256 /f fwu.pfx /p imx8 catalog.cat
"c:\Program Files (x86)\Windows Kits\10\bin\10.0.19041.0\x86\signtool.exe" sign /fd sha256 /f fwu.pfx /p imx8 catalog.cat

REM @echo on
REM "c:\Program Files (x86)\Windows Kits\10\bin\10.0.19041.0\x86\inf2cat.exe" /driver:. /os:10_VB_ARM64 /verbose
REM "c:\Program Files (x86)\Windows Kits\10\bin\10.0.19041.0\x86\signtool.exe" sign /debug /ph /fd sha256 /sha1 749E4CC65EE7B3E1CFF5E51B9EEC5560225A44D7 catalog.cat
