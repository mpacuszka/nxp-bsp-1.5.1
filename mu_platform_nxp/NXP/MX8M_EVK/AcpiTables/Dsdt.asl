/*
* iMX8 MQ Differentiated System Description Table Fields (DSDT)
*
*  Copyright (c) 2018, Microsoft Corporation. All rights reserved.
*  Copyright 2020,2022-2023 NXP
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
     /* Shared general purpose registers */
    include("Dsdt-Gpr.asl")
    /* Dependencies */
    include("Dsdt-Gpio.asl")
    include("Dsdt-I2c.asl")
    include("Dsdt-Spi.asl")
    include("Dsdt-Uart.asl")
    /* Others */
    include("Dsdt-Audio.asl")
#if FixedPcdGet32(PcdCsi1CameraOv5640)
    include("Dsdt-Camera_Ov5640_mipi_csi1.asl")
#elif FixedPcdGet32(PcdCsi1CameraOv10635)
    include("Dsdt-Camera_Ov10635_mipi_csi1.asl")
#endif
#if FixedPcdGet32(PcdCsi2CameraOv5640)
    include("Dsdt-Camera_Ov5640_mipi_csi2.asl")
#elif FixedPcdGet32(PcdCsi2CameraOv10635)
    include("Dsdt-Camera_Ov10635_mipi_csi2.asl")
#endif
    include("Dsdt-Enet.asl")
    include("Dsdt-Gfx.asl")
    include("Dsdt-PCIe.asl")
    include("Dsdt-Pwm.asl")
    include("Dsdt-Rhp.asl")
    include("Dsdt-Sdhc.asl")
    include("Dsdt-TCPC.asl")
    include("Dsdt-Tmu.asl")
#if FixedPcdGetBool(PcdSecureBootEnable)
    include("Dsdt-TrEE.asl")
#endif
    include("Dsdt-Usb.asl")
    include("Dsdt-HantroVpuFF.asl") /* Full Feature */
    include("Dsdt-Tad.asl")
  }
}
