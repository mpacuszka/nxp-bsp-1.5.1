The iMXNorFlashDxe is a Boot-time and RunTime UEFI driver interfacing with Nor Flash.
External interfaces:

-------------               ----------------              --------------              ---------------
|   UEFI    |  FvbProtocol  |              |  FlexSPI if  |     SoC    |  SPI wires   |             |
| Variable  | <==========>  | iMXNorFlash  | <==========> |   FlexSPI  | <==========> |  NOR Flash  |
| Service   |  FtwProtocol  |    Dxe       |              |      HW    |              |     Chip    |
|           |               |              |              |            |              |             |
-------------               ----------------              --------------              ---------------

The iMXNorFlashDxe was integrated and tested  with iMX8MP and iMX8MN EVK boards.

The layers inside iMXNorFlashDxe are as follows (from Top to Bottom)

iMXNorFlashDxe.c    .. the UEFI DriverEntry and high level functions
iMXNorFlashFvb.c    .. Fvb = FirmwareVolumeBlock protocol - the UEFI interface required by VariableService
iMXNorFlashHw.h/c   .. the interface towards the lower layers taken over from Uboot Serial Flash driver
iMXNorDriver.h/c    .. the driver methods, ported from Uboot spi-nor-core.c and spi-mem.c
iMXNorFspi.h/c      .. the FlexSPI driver  ported from Uboot nxp_fspi.c
iMXNorSpiCommon.h   .. common definitions needed by the ported Uboot code


HOW TO ENABLE THE UEFI NON-VOLATILE VARIABLES:
==============================================
1) Set PcdEmuVariableNvModeEnable to FALSE in iMX8CommonDsc.inc (2 occurences, search for "Emu")
2) Add "sf probe" to CONFIG_BOOTCOMMAND in uboot defconfig. This is needed because the UEFI driver
currently does not setup the Clock needed for the FSPI module on the SoC:
uboot/configs/imx8mn_evk_nt_uuu_defconfig
-CONFIG_BOOTCOMMAND="usb start; bootm ${loadaddr};"
+CONFIG_BOOTCOMMAND="usb start; sf probe; bootm ${loadaddr};"


TESTING NON-VOLATILE UEFI VARIABLES:
====================================
1) In Uboot prompt, issue "sf probe" and "sf erase 0 5000" ... this will make sure the Flash is erased
2) Boot to Windows, enter elevated PowerShell and issue these commands:
Set-ExecutionPolicy Unrestricted
Install-Module UEFI
Note: With Get-UEFIVariable/Set-UEFIVariable sometimes there are exceptions from Windows before the call enters UEFI, to be clarified with MS
Set-UEFIVariable -VariableName Blah1 -Value Hello1
Get-UEFIVariable -VariableName Blah1
After restart or power off/on the variable Blah1 shall still be present

Another example:
Get-UEFIVariable -VariableName BootOrder -AsByteArray
[byte[]] $array1 = 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00
Set-UEFIVariable -VariableName BootOrder -ByteArray $array1

3) Another option to see UEFI Variables is to enter UEFI Shell and use commands "setvar" or "dmpstore"
