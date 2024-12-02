/*
* iMX8M Mini Differentiated System Description Table Fields (DSDT)
*
*  Copyright (c) 2018, Microsoft Corporation. All rights reserved.
*  Copyright 2019-2020,2022-2023 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
*/

#include "Dsdt-Common.h"
#include "iMX8Platform.h"
#include <IndustryStandard/Acpi.h>
#include <Library/AcpiLib.h>
#include <Library/PcdLib.h>

DefinitionBlock("DsdtTable.aml", "DSDT", 5, "MSFT", "EDK2", 1) {
  Scope(_SB) {
    include("Dsdt-Platform.asl")
    /* Dependencies/Resources */
    include("Dsdt-Gpr.asl") /* Shared general purpose registers */
    include("Dsdt-Gpio.asl")
    include("Dsdt-I2c.asl")
    include("Dsdt-Spi.asl")
    include("Dsdt-Uart.asl")
    /* Others */
#if FixedPcdGet32(PcdCsi1CameraOv5640)
    include("Dsdt-Camera_Ov5640_mipi_csi1.asl")
#elif FixedPcdGet32(PcdCsi1CameraOv10635)
    include("Dsdt-Camera_Ov10635_mipi_csi1.asl")
#endif
    include("Dsdt-Sdhc.asl")
    include("Dsdt-Usb.asl")
    include("Dsdt-Rhp.asl")
    include("Dsdt-Enet.asl")
#if FixedPcdGetBool(PcdSecureBootEnable)
    include("Dsdt-TrEE.asl")
#endif
    include("Dsdt-PCIe.asl")
    include("Dsdt-Audio.asl")
    include("Dsdt-Pwm.asl")
    include("Dsdt-HantroVpuLF.asl") /* Limited Feature */
    include("Dsdt-TCPC.asl")
    include("Dsdt-Tad.asl")
  }
}
