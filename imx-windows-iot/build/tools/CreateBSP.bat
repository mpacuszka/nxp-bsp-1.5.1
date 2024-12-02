:: Copyright (c) Microsoft Corporation. All rights reserved.
:: Copyright 2022-2023 NXP
:: Licensed under the MIT License.

@echo off
setlocal enableextensions disabledelayedexpansion

pushd %~dp0\..\..

if "%1"=="" (
    echo "Missing SOC argument to CreateBSP.bat"
    echo "Usage: CreateBSP.bat <SOC>"
    echo "eg: CreateBSP.bat iMX8"
    exit /b 1
)

if "%Configuration%"=="" (
    echo configuration environment variable is not set
    exit /b 1
)

if "%PLATFORM%"=="" (
    echo platform environment variable is not set
    exit /b 1
)

set SOC=%1
set REPO_ROOT=%cd%
set BSP_ROOT=%cd%\BSP\IoTEntOnNXP
set COMP_ROOT=%cd%\components
set DRVR_ROOT=%BSP_ROOT%\drivers
set KBPTCH_ROOT=%BSP_ROOT%\kbpatch
set BUILD_ROOT=%REPO_ROOT%\build\solution\iMXPlatform\Build\%PLATFORM%\%Configuration%
set FAILURE=

echo BSP_ROOT is "%BSP_ROOT%"
echo BUILD_ROOT is "%BUILD_ROOT%"
echo DRVR_ROOT is "%DRVR_ROOT%"
echo KBPTCH_ROOT is "%KBPTCH_ROOT%"
echo REPO_ROOT is "%REPO_ROOT%"

if "%SOC%"=="iMX8" (
    goto ARM64
)

echo Unrecognized SOC type: %SOC%
exit /b 1

:ARM64

:: Copy Driver Packages
echo Copying Drivers to %DRVR_ROOT%

mkdir "%DRVR_ROOT%\Audio" >NUL 2>NUL
copy "%BUILD_ROOT%\imxaud\*" "%DRVR_ROOT%\Audio\"
if errorlevel 1 (set FAILURE=imxaud & goto ERROR)
copy "%REPO_ROOT%\driver\audio\controller\imxaud\imxaud.wm.xml" "%DRVR_ROOT%\Audio\" >NUL

mkdir "%DRVR_ROOT%\Audio" >NUL 2>NUL
copy "%BUILD_ROOT%\imxaudsc\*" "%DRVR_ROOT%\Audio\" 
if errorlevel 1 (set FAILURE=imxaudsc & goto ERROR)
copy "%REPO_ROOT%\driver\audio\controller\imxaudsc\imxaudsc.wm.xml" "%DRVR_ROOT%\Audio\" >NUL

echo Copying Drivers to "%DRVR_ROOT%"
mkdir "%DRVR_ROOT%\Audio" >NUL 2>NUL
copy "%BUILD_ROOT%\imxaud2\*" "%DRVR_ROOT%\Audio\" >NUL
if errorlevel 1 (set FAILURE=imxaud2 & goto ERROR)
copy "%REPO_ROOT%\driver\audio\controller\imxaud2\imxaud2.wm.xml" "%DRVR_ROOT%\Audio\" >NUL

mkdir "%DRVR_ROOT%\Audio" >NUL 2>NUL
copy "%BUILD_ROOT%\imxaud_hdmi\*" "%DRVR_ROOT%\Audio\" >NUL
if errorlevel 1 (set FAILURE=imxaud_hdmi & goto ERROR)
copy "%REPO_ROOT%\driver\audio\controller\imxaud_hdmi\imxaud_hdmi.wm.xml" "%DRVR_ROOT%\Audio\" >NUL

:: Copy ALS
mkdir "%DRVR_ROOT%\ALS" >NUL 2>NUL
copy "%BUILD_ROOT%\imxals\*" "%DRVR_ROOT%\ALS\" >NUL
if errorlevel 1 (set FAILURE=imxals & goto ERROR)
copy "%REPO_ROOT%\driver\als\imxals\imxals.wm.xml" "%DRVR_ROOT%\ALS\" >NUL

mkdir "%DRVR_ROOT%\CAN_MC" >NUL 2>NUL
copy "%BUILD_ROOT%\imxcan_mc\*" "%DRVR_ROOT%\CAN_MC\" >NUL
if errorlevel 1 (set FAILURE=imxcan_mc & goto ERROR)

mkdir "%DRVR_ROOT%\CAN_NMC" >NUL 2>NUL
copy "%BUILD_ROOT%\imxcan_nmc\*" "%DRVR_ROOT%\CAN_NMC\" >NUL
if errorlevel 1 (set FAILURE=imxcan_nmc & goto ERROR)

mkdir "%DRVR_ROOT%\ECOMPASS" >NUL 2>NUL
copy "%BUILD_ROOT%\imxecompass\*" "%DRVR_ROOT%\ECOMPASS\" >NUL
if errorlevel 1 (set FAILURE=imxecompass & goto ERROR)

mkdir "%DRVR_ROOT%\IMU" >NUL 2>NUL
copy "%BUILD_ROOT%\imximu\*" "%DRVR_ROOT%\IMU\" >NUL
if errorlevel 1 (set FAILURE=imximu & goto ERROR)

mkdir "%DRVR_ROOT%\GPIO" >NUL 2>NUL
copy "%BUILD_ROOT%\imxgpio\*" "%DRVR_ROOT%\GPIO\" >NUL
if errorlevel 1 (set FAILURE=imxgpio & goto ERROR)
copy "%REPO_ROOT%\driver\gpio\imxgpio\imxgpio.wm.xml" "%DRVR_ROOT%\GPIO\" >NUL

mkdir "%DRVR_ROOT%\GPIO_8X" >NUL 2>NUL
copy "%BUILD_ROOT%\imxgpio_8x\*" "%DRVR_ROOT%\GPIO_8X\" >NUL
if errorlevel 1 (set FAILURE=imxgpio_8x & goto ERROR)

mkdir "%DRVR_ROOT%\RGPIO" >NUL 2>NUL
copy "%BUILD_ROOT%\imxrgpio\*" "%DRVR_ROOT%\RGPIO\" >NUL
if errorlevel 1 (set FAILURE=imxrgpio & goto ERROR)

mkdir "%DRVR_ROOT%\I2C" >NUL 2>NUL
copy "%BUILD_ROOT%\imxi2c\*" "%DRVR_ROOT%\I2C\" >NUL
if errorlevel 1 (set FAILURE=imxi2c & goto ERROR)
copy "%REPO_ROOT%\driver\i2c\imxi2c\imxi2c.wm.xml" "%DRVR_ROOT%\I2C\" >NUL

mkdir "%DRVR_ROOT%\LPI2C" >NUL 2>NUL
copy "%BUILD_ROOT%\imxlpi2c\*" "%DRVR_ROOT%\LPI2C\" >NUL
if errorlevel 1 (set FAILURE=imxlpi2c & goto ERROR)
copy "%REPO_ROOT%\driver\i2c\imxlpi2c\imxlpi2c.wm.xml" "%DRVR_ROOT%\LPI2C\" >NUL

mkdir "%DRVR_ROOT%\SCFW" >NUL 2>NUL
copy "%BUILD_ROOT%\imxscfw\*" "%DRVR_ROOT%\SCFW\" >NUL
if errorlevel 1 (set FAILURE=imxscfw & goto ERROR)
copy "%REPO_ROOT%\driver\i2c\imxscfw\imxscfw.wm.xml" "%DRVR_ROOT%\SCFW\" >NUL

mkdir "%DRVR_ROOT%\Net" >NUL 2>NUL
copy "%BUILD_ROOT%\imxnetmini\*" "%DRVR_ROOT%\Net\" >NUL
if errorlevel 1 (set FAILURE=imxnetmini & goto ERROR)
copy "%REPO_ROOT%\driver\net\ndis\imxnetmini\imxnetmini.wm.xml" "%DRVR_ROOT%\Net\" >NUL

mkdir "%DRVR_ROOT%\Net" >NUL 2>NUL
copy "%BUILD_ROOT%\imxqosmini\*" "%DRVR_ROOT%\Net\" >NUL
if errorlevel 1 (set FAILURE=imxqosmini & goto ERROR)
copy "%REPO_ROOT%\driver\net\ndis\imxqosmini\imxqosmini.wm.xml" "%DRVR_ROOT%\Net\" >NUL

mkdir "%DRVR_ROOT%\UART" >NUL 2>NUL
copy "%BUILD_ROOT%\imxuart\*" "%DRVR_ROOT%\UART\" >NUL
if errorlevel 1 (set FAILURE=imxuart & goto ERROR)
copy "%REPO_ROOT%\driver\serial\imxuart\imxuart.wm.xml" "%DRVR_ROOT%\UART\" >NUL

mkdir "%DRVR_ROOT%\LPUART" >NUL 2>NUL
copy "%BUILD_ROOT%\imxlpuart\*" "%DRVR_ROOT%\LPUART\" >NUL
if errorlevel 1 (set FAILURE=imxlpuart & goto ERROR)
copy "%REPO_ROOT%\driver\serial\imxlpuart\imxlpuart.wm.xml" "%DRVR_ROOT%\LPUART\" >NUL

mkdir "%DRVR_ROOT%\SPI" >NUL 2>NUL
copy "%BUILD_ROOT%\imxecspi\*" "%DRVR_ROOT%\SPI\" >NUL
if errorlevel 1 (set FAILURE=imxecspi & goto ERROR)
copy "%REPO_ROOT%\driver\spi\imxecspi\imxecspi.wm.xml" "%DRVR_ROOT%\SPI\" >NUL

mkdir "%DRVR_ROOT%\SPI" >NUL 2>NUL
copy "%BUILD_ROOT%\imxlpspi\*" "%DRVR_ROOT%\SPI\" >NUL
if errorlevel 1 (set FAILURE=imxlpspi & goto ERROR)
copy "%REPO_ROOT%\driver\spi\imxlpspi\imxlpspi.wm.xml" "%DRVR_ROOT%\SPI\" >NUL

mkdir "%DRVR_ROOT%\PWM" >NUL 2>NUL
copy "%BUILD_ROOT%\imxpwm\*" "%DRVR_ROOT%\PWM\" >NUL
if errorlevel 1 (set FAILURE=imxpwm & goto ERROR)
copy "%REPO_ROOT%\driver\pwm\imxpwm\imxpwm.wm.xml" "%DRVR_ROOT%\PWM\" >NUL

mkdir "%DRVR_ROOT%\PWM_TPM" >NUL 2>NUL
copy "%BUILD_ROOT%\imxpwm_tpm\*" "%DRVR_ROOT%\PWM_TPM\" >NUL
if errorlevel 1 (set FAILURE=imxpwm_tpm & goto ERROR)
copy "%REPO_ROOT%\driver\pwm_tpm\imxpwm_tpm\imxpwm_tpm.wm.xml" "%DRVR_ROOT%\PWM_TPM\" >NUL

mkdir "%DRVR_ROOT%\ThermalSensor" >NUL 2>NUL
copy "%BUILD_ROOT%\imxtmu\*" "%DRVR_ROOT%\ThermalSensor" >NUL
if errorlevel 1 (set FAILURE=imxtmu & goto ERROR)
copy "%REPO_ROOT%\driver\ThermalSensor\imxtmu.wm.xml" "%DRVR_ROOT%\ThermalSensor" >NUL

mkdir "%DRVR_ROOT%\ThermalClient" >NUL 2>NUL
copy "%BUILD_ROOT%\imxthermalclient\*" "%DRVR_ROOT%\ThermalClient" >NUL
if errorlevel 1 (set FAILURE=imxthermalclient & goto ERROR)
copy "%REPO_ROOT%\driver\ThermalClient\imxthermalclient.wm.xml" "%DRVR_ROOT%\ThermalClient" >NUL

mkdir "%DRVR_ROOT%\USDHC" >NUL 2>NUL
copy "%BUILD_ROOT%\imxusdhc\*" "%DRVR_ROOT%\USDHC\" >NUL
if errorlevel 1 (set FAILURE=imxusdhc & goto ERROR)
copy "%REPO_ROOT%\driver\sd\imxusdhc\imxusdhc.wm.xml" "%DRVR_ROOT%\USDHC\" >NUL

mkdir "%DRVR_ROOT%\OPTEE" >NUL 2>NUL
copy "%BUILD_ROOT%\OpteeTrEE\*" "%DRVR_ROOT%\OPTEE\" >NUL
if errorlevel 1 (set FAILURE=OpteeTrEE & goto ERROR)
copy "%REPO_ROOT%\driver\TrEE\TrEE\OpteeTrEE.wm.xml" "%DRVR_ROOT%\OPTEE\" >NUL

mkdir "%DRVR_ROOT%\Gyro" >NUL 2>NUL
copy "%BUILD_ROOT%\imxgyro_fxas21002\*" "%DRVR_ROOT%\Gyro\" >NUL
if errorlevel 1 (set FAILURE=Gyro & goto ERROR)

:: Copy Hantro VPU stuff
mkdir "%DRVR_ROOT%\VPU" >NUL 2>NUL
copy "%BUILD_ROOT%\imxvpukm\*" "%DRVR_ROOT%\VPU\" >NUL
if errorlevel 1 (set FAILURE=imxvpukm & goto ERROR)
copy "%REPO_ROOT%\driver\video\imxvpukm\imxvpukm.wm.xml" "%DRVR_ROOT%\VPU\" >NUL

copy "%BUILD_ROOT%\imxvpumft\imxvpumft.dll" "%DRVR_ROOT%\VPU\" >NUL
if errorlevel 1 (set FAILURE=imxvpumft & goto ERROR)

copy "%BUILD_ROOT%\imxomxg1\imxomxg1.dll" "%DRVR_ROOT%\VPU\" >NUL
if errorlevel 1 (set FAILURE=imxomxg1 & goto ERROR)

copy "%BUILD_ROOT%\imxomxg2\imxomxg2.dll" "%DRVR_ROOT%\VPU\" >NUL
if errorlevel 1 (set FAILURE=imxomxg2 & goto ERROR)

copy "%BUILD_ROOT%\imx-vpu-dwl\imx-vpu-dwl.dll" "%DRVR_ROOT%\VPU\" >NUL
if errorlevel 1 (set FAILURE=imx-vpu-dwl & goto ERROR)

copy "%COMP_ROOT%\video\*" "%DRVR_ROOT%\VPU\" >NUL
if errorlevel 1 (set FAILURE=vcruntime140.dll & goto ERROR)

:: Copy Malone VPU stuff
mkdir "%DRVR_ROOT%\VPUMalone" >NUL 2>NUL
copy "%BUILD_ROOT%\malonekm\*" "%DRVR_ROOT%\VPUMalone\" >NUL
if errorlevel 1 (set FAILURE=malonekm & goto ERROR)
copy "%BUILD_ROOT%\malonemft\malonemft.dll" "%DRVR_ROOT%\VPUMalone\" >NUL
if errorlevel 1 (set FAILURE=malonemft & goto ERROR)

copy "%COMP_ROOT%\video\*" "%DRVR_ROOT%\VPUMalone\" >NUL
if errorlevel 1 (set FAILURE=vcruntime140.dll & goto ERROR)

:: Copy USB
mkdir "%DRVR_ROOT%\USB" >NUL 2>NUL
copy "%BUILD_ROOT%\imxUcmTcpciCxClient\*" "%DRVR_ROOT%\USB\" >NUL
if errorlevel 1 (set FAILURE=imxUcmTcpciCxClient & goto ERROR)
copy "%REPO_ROOT%\driver\USB\imxUcmTcpciCxClient\imxUcmTcpciCxClient.wm.xml" "%DRVR_ROOT%\USB\" >NUL


mkdir "%DRVR_ROOT%\Wm8960codec" >NUL 2>NUL
copy "%BUILD_ROOT%\Wm8960codec\*" "%DRVR_ROOT%\Wm8960codec\" >NUL
if errorlevel 1 (set FAILURE=Wm8960codec & goto ERROR)

mkdir "%DRVR_ROOT%\Wm8962codec" >NUL 2>NUL
copy "%BUILD_ROOT%\Wm8962codec\*" "%DRVR_ROOT%\Wm8962codec\" >NUL
if errorlevel 1 (set FAILURE=Wm8962codec & goto ERROR)

:: Copy Camera
mkdir "%DRVR_ROOT%\ImxCamera" >NUL 2>NUL
copy "%BUILD_ROOT%\imxcamera_package\*.sys" "%DRVR_ROOT%\ImxCamera" >NUL
copy "%BUILD_ROOT%\imxcamera_package\*.cat" "%DRVR_ROOT%\ImxCamera" >NUL
copy "%BUILD_ROOT%\imxcamera_package\*.fw" "%DRVR_ROOT%\ImxCamera" >NUL
copy "%BUILD_ROOT%\imxcamera_package\imxcamera_package.inf" "%DRVR_ROOT%\ImxCamera" >NUL
if errorlevel 1 (set FAILURE=Avstream & goto ERROR)

:: Copy HAL Extension Packages
echo Copying HAL Extension Packages to "%DRVR_ROOT%"
mkdir "%DRVR_ROOT%\HalExtDma" >NUL 2>NUL
copy "%BUILD_ROOT%\HalExtiMXDma\*" "%DRVR_ROOT%\HalExtDma\" >NUL
if errorlevel 1 (set FAILURE=HalExtiMXDma & goto ERROR)
copy "%REPO_ROOT%\hals\halext\HalExtiMXDma\HalExtiMXDma.wm.xml" "%DRVR_ROOT%\HalExtDma\" >NUL

:: Copy Galcore GPU
echo Copying Galcore GPU driver to "%DRVR_ROOT%"
mkdir "%DRVR_ROOT%\Galcore" >NUL 2>NUL
copy "%BUILD_ROOT%\galcore\galcore\*" "%DRVR_ROOT%\Galcore\" >NUL
:: if errorlevel 1 (set FAILURE=Galcore & goto ERROR)

:: Copy PEP
mkdir "%DRVR_ROOT%\PEP" >NUL 2>NUL
copy "%BUILD_ROOT%\imxpep\*" "%DRVR_ROOT%\PEP\" >NUL
if errorlevel 1 (set FAILURE=imxpep & goto ERROR)
copy "%REPO_ROOT%\driver\pep\sys\imxpep.wm.xml" "%DRVR_ROOT%\PEP\" >NUL

:: Copy image package files
echo Copying Packages to "%KBPTCH_ROOT%"
mkdir "%KBPTCH_ROOT%" >NUL 2>NUL
copy "%COMP_ROOT%\kbpatch\*" "%KBPTCH_ROOT%\" >NUL
if errorlevel 2 (set FAILURE=KBPatch & goto ERROR)

:: Copy Wifi mrvlpcie8897 
echo Copying Wifi mrvlpcie8897 to "%DRVR_ROOT%"
mkdir "%DRVR_ROOT%\Wifi_mrvlpcie8897" >NUL 2>NUL
copy "%COMP_ROOT%\Wifi\mrvlpcie8897\*" "%DRVR_ROOT%\Wifi_mrvlpcie8897\" >NUL
if errorlevel 1 (set FAILURE=mrvlpcie8897 & goto ERROR)

:: Copy Wifi mrvlpcie8997 
echo Copying Wifi mrvlpcie8997 to "%DRVR_ROOT%"
mkdir "%DRVR_ROOT%\Wifi_mrvlpcie8997" >NUL 2>NUL
copy "%COMP_ROOT%\Wifi\mrvlpcie8997\*" "%DRVR_ROOT%\Wifi_mrvlpcie8997\" >NUL
if errorlevel 1 (set FAILURE=mrvlpcie8997 & goto ERROR)

:: Copy usbehci fix
echo Copying USB EHCI fix to "%DRVR_ROOT%"
mkdir "%DRVR_ROOT%\usbehci" >NUL 2>NUL
copy "%COMP_ROOT%\usbehci\*" "%DRVR_ROOT%\usbehci\" >NUL
if errorlevel 1 (set FAILURE=usbehci & goto ERROR)

exit /b 0

:ERROR
echo CreateBSP.bat failed to copy %FAILURE% binaries to the BSP directory! Please ensure that the drivers have built in Visual Studio.
exit /b 1
