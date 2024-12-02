/*
* Copyright 2023-2024 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef _DC_SOC_H_
#define _DC_SOC_H_

EFI_STATUS DcSocInit(imxDisplayInterfaceType displayInterface, UINT32 PixelClk);
EFI_STATUS DcVideoInit(imxDisplayInterfaceType displayInterface, IMX_DISPLAY_TIMING *Timing,
                   EFI_PHYSICAL_ADDRESS  FrameBaseAddress, BOOLEAN WaitFrmCntrMove);
EFI_STATUS DcPixelLinkStop(imxDisplayInterfaceType displayInterface);
EFI_STATUS DcPixelLinkEnDi(imxDisplayInterfaceType displayInterface, BOOLEAN enable);
EFI_STATUS DcVideoStop(imxDisplayInterfaceType displayInterface);
EFI_STATUS DcClockStop(imxDisplayInterfaceType displayInterface);

#endif  /* _DC_SOC_H_ */
