/****************************************************************************
* Copyright (c) Microsoft Corporation.
*
*    Licensed under the MIT License.
*    Licensed under the GPL License.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/

#pragma once

#include "d3dkmddi.h"
namespace MQ
{
#include "MQ\Gc7LCmdCompBB.h"
#include "MQ\Gc7LCmdCompCF.h"
#include "MQ\Gc7LCmdCompAB.h"
#include "MQ\Gc7LCmdCompSB.h"
#include "MQ\Gc7LCmdCompTB.h"
#include "MQ\Gc7LCmdCompCTB.h"
};
namespace MP
{
#include "MP\Gc7LCmdCompBB.h"
#include "MP\Gc7LCmdCompCF.h"
#include "MP\Gc7LCmdCompAB.h"
#include "MP\Gc7LCmdCompSB.h"
#include "MP\Gc7LCmdCompTB.h"
#include "MP\Gc7LCmdCompCTB.h"
};
#include "Gc7LCmdGdiOp.h"

//
// The index is DXGK_GDIOP_BITBLT
//

const unsigned int GDI_BITBLT_ROPS = DXGK_GDIROP_ROP3 + 1;
const unsigned int GDI_ALPHABLEND_ROPS  = 5;
const unsigned int GDI_TRANSPARENTBLT_ROPS  = 2;

unsigned int GC_7L_GDI_OP_STARTS[] =
{
    0,
    DXGK_GDIOP_BITBLT,
    DXGK_GDIOP_COLORFILL      + GDI_BITBLT_ROPS,
    DXGK_GDIOP_ALPHABLEND     + GDI_BITBLT_ROPS + DXGK_GDIROPCF_ROP3,
    DXGK_GDIOP_STRETCHBLT     + GDI_BITBLT_ROPS + DXGK_GDIROPCF_ROP3 + GDI_ALPHABLEND_ROPS,
    DXGK_GDIOP_ESCAPE         + GDI_BITBLT_ROPS + DXGK_GDIROPCF_ROP3 + GDI_ALPHABLEND_ROPS,
    DXGK_GDIOP_TRANSPARENTBLT + GDI_BITBLT_ROPS + DXGK_GDIROPCF_ROP3 + GDI_ALPHABLEND_ROPS,
    DXGK_GDIOP_CLEARTYPEBLEND + GDI_BITBLT_ROPS + DXGK_GDIROPCF_ROP3 + GDI_ALPHABLEND_ROPS + GDI_TRANSPARENTBLT_ROPS
};

//
// The index is OP_STARTS[OP] + ROP for BitBlt or ColorFill
//

namespace MQ
{
GC_7L_GDI_OP GC_7L_GDI_OP_TABLE[] =
{
    {
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_INVALID
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_SRCCOPY
        GC_7L_BITBLT_SHADER_SRCCOPY,
        sizeof(GC_7L_BITBLT_SHADER_SRCCOPY),
        0,
        GC_7L_CMD_COMPUTE_BITBLT_1,
        sizeof(GC_7L_CMD_COMPUTE_BITBLT_1),
        GC_7L_CMD_BITBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_BITBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_SRCINVERT
        GC_7L_BITBLT_SHADER_SRCINVERT,
        sizeof(GC_7L_BITBLT_SHADER_SRCINVERT),
        0,
        GC_7L_CMD_COMPUTE_BITBLT_2_3_4,
        sizeof(GC_7L_CMD_COMPUTE_BITBLT_2_3_4),
        GC_7L_CMD_BITBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_BITBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_SRCAND
        GC_7L_BITBLT_SHADER_SRCAND,
        sizeof(GC_7L_BITBLT_SHADER_SRCAND),
        0,
        GC_7L_CMD_COMPUTE_BITBLT_2_3_4,
        sizeof(GC_7L_CMD_COMPUTE_BITBLT_2_3_4),
        GC_7L_CMD_BITBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_BITBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_SRCOR
        GC_7L_BITBLT_SHADER_SRCOR,
        sizeof(GC_7L_BITBLT_SHADER_SRCOR),
        0,
        GC_7L_CMD_COMPUTE_BITBLT_2_3_4,
        sizeof(GC_7L_CMD_COMPUTE_BITBLT_2_3_4),
        GC_7L_CMD_BITBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_BITBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_ROP3
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_BITBLT + SRCCOPY of A8
        GC_7L_BITBLT_SHADER_SRCCOPY_A8,
        sizeof(GC_7L_BITBLT_SHADER_SRCCOPY_A8),
        0,
        GC_7L_CMD_COMPUTE_BITBLT_1_A8,
        sizeof(GC_7L_CMD_COMPUTE_BITBLT_1_A8),
        GC_7L_CMD_BITBLT_PATCH_TABLE_A8,
        sizeof(GC_7L_CMD_BITBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_INVALID
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_PATCOPY
        GC_7L_COLORFILL_SHADER_PATCOPY,
        sizeof(GC_7L_COLORFILL_SHADER_PATCOPY),
        0,
        GC_7L_CMD_COMPUTE_COLORFILL_1,
        sizeof(GC_7L_CMD_COMPUTE_COLORFILL_1),
        GC_7L_CMD_COLORFILL_PATCH_TABLE,
        sizeof(GC_7L_CMD_COLORFILL_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_PATINVERT
        GC_7L_COLORFILL_SHADER_PATINVERT,
        sizeof(GC_7L_COLORFILL_SHADER_PATINVERT),
        0,
        GC_7L_CMD_COMPUTE_COLORFILL_2_5_6,
        sizeof(GC_7L_CMD_COMPUTE_COLORFILL_2_5_6),
        GC_7L_CMD_COLORFILL_PATCH_TABLE,
        sizeof(GC_7L_CMD_COLORFILL_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_PDXN
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_DSTINVERT
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_PATAND
        GC_7L_COLORFILL_SHADER_PATAND,
        sizeof(GC_7L_COLORFILL_SHADER_PATAND),
        0,
        GC_7L_CMD_COMPUTE_COLORFILL_2_5_6,
        sizeof(GC_7L_CMD_COMPUTE_COLORFILL_2_5_6),
        GC_7L_CMD_COLORFILL_PATCH_TABLE,
        sizeof(GC_7L_CMD_COLORFILL_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_PATOR
        GC_7L_COLORFILL_SHADER_PATOR,
        sizeof(GC_7L_COLORFILL_SHADER_PATOR),
        0,
        GC_7L_CMD_COMPUTE_COLORFILL_2_5_6,
        sizeof(GC_7L_CMD_COMPUTE_COLORFILL_2_5_6),
        GC_7L_CMD_COLORFILL_PATCH_TABLE,
        sizeof(GC_7L_CMD_COLORFILL_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_ROP3
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Generic
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Generic Per Pixel
        GC_7L_ALPHABLEND_SHADER_CONSTALPHA,
        sizeof(GC_7L_ALPHABLEND_SHADER_CONSTALPHA),
        0,
        GC_7L_CMD_COMPUTE_ALPHABLEND_1_4,
        sizeof(GC_7L_CMD_COMPUTE_ALPHABLEND_1_4),
        GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4,
        sizeof(GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Constant Alpha of 0XFF
        GC_7L_ALPHABLEND_SHADER_PERPIXEL,
        sizeof(GC_7L_ALPHABLEND_SHADER_PERPIXEL),
        0,
        GC_7L_CMD_COMPUTE_ALPHABLEND_2,
        sizeof(GC_7L_CMD_COMPUTE_ALPHABLEND_2),
        GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4,
        sizeof(GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Constant Alpha of 0xFF and 1:1
        GC_7L_ALPHABLEND_SHADER_PERPIXEL_11,
        sizeof(GC_7L_ALPHABLEND_SHADER_PERPIXEL_11),
        0,
        GC_7L_CMD_COMPUTE_ALPHABLEND_3,
        sizeof(GC_7L_CMD_COMPUTE_ALPHABLEND_3),
        GC_7L_CMD_ALPHABLEND_PATCH_TABLE_3_5,
        sizeof(GC_7L_CMD_ALPHABLEND_PATCH_TABLE_3_5)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Non Per Pixel Alpha
        GC_7L_ALPHABLEND_SHADER_NPP,
        sizeof(GC_7L_ALPHABLEND_SHADER_NPP),
        0,
        GC_7L_CMD_COMPUTE_ALPHABLEND_1_4,
        sizeof(GC_7L_CMD_COMPUTE_ALPHABLEND_1_4),
        GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4,
        sizeof(GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Non Per Pixel Alpha and 1:1
        GC_7L_ALPHABLEND_SHADER_NPP_11,
        sizeof(GC_7L_ALPHABLEND_SHADER_NPP_11),
        0,
        GC_7L_CMD_COMPUTE_ALPHABLEND_5,
        sizeof(GC_7L_CMD_COMPUTE_ALPHABLEND_5),
        GC_7L_CMD_ALPHABLEND_PATCH_TABLE_3_5,
        sizeof(GC_7L_CMD_ALPHABLEND_PATCH_TABLE_3_5)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_STRETCHBLT
        GC_7L_STRETCHBLT_SHADER,
        sizeof(GC_7L_STRETCHBLT_SHADER),
        0,
        GC_7L_CMD_COMPUTE_STRETCHBLT,
        sizeof(GC_7L_CMD_COMPUTE_STRETCHBLT),
        GC_7L_CMD_STRETCHBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_STRETCHBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_ESCAPE
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_TRANSPARENTBLT
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_TRANSPARENTBLT : HonorAlpha of 1
        GC_7L_TRANSPARENTBLT_SHADER_ALPHA,
        sizeof(GC_7L_TRANSPARENTBLT_SHADER_ALPHA),
        0,
        GC_7L_CMD_COMPUTE_TRANSPARENTBLT_1_2,
        sizeof(GC_7L_CMD_COMPUTE_TRANSPARENTBLT_1_2),
        GC_7L_CMD_TRANSPARENTBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_TRANSPARENTBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_TRANSPARENTBLT : HonorAlpha of 0
        GC_7L_TRANSPARENTBLT_SHADER_NO_ALPHA,
        sizeof(GC_7L_TRANSPARENTBLT_SHADER_NO_ALPHA),
        0,
        GC_7L_CMD_COMPUTE_TRANSPARENTBLT_1_2,
        sizeof(GC_7L_CMD_COMPUTE_TRANSPARENTBLT_1_2),
        GC_7L_CMD_TRANSPARENTBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_TRANSPARENTBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_CLEARTYPEBLEND
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_CLEARTYPEBLEND : CTB
        GC_7L_CLEARTYPEBLEND_SHADER,
        sizeof(GC_7L_CLEARTYPEBLEND_SHADER),
        0,
        GC_7L_CMD_COMPUTE_CLEARTYPEBLEND_1,
        sizeof(GC_7L_CMD_COMPUTE_CLEARTYPEBLEND_1),
        GC_7L_CMD_CLEARTYPEBLEND_PATCH_TABLE_1,
        sizeof(GC_7L_CMD_CLEARTYPEBLEND_PATCH_TABLE_1)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_CLEARTYPEBLEND : CTB Gamma of -1
        GC_7L_CLEARTYPEBLEND_SHADER_NO_GAMMA,
        sizeof(GC_7L_CLEARTYPEBLEND_SHADER_NO_GAMMA),
        0,
        GC_7L_CMD_COMPUTE_CLEARTYPEBLEND_2,
        sizeof(GC_7L_CMD_COMPUTE_CLEARTYPEBLEND_2),
        GC_7L_CMD_CLEARTYPEBLEND_PATCH_TABLE_2,
        sizeof(GC_7L_CMD_CLEARTYPEBLEND_PATCH_TABLE_2)/sizeof(GC_7L_PATCH_ENTRY)
    },
};
};

namespace MP
{
GC_7L_GDI_OP GC_7L_GDI_OP_TABLE[] =
{
    {
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_INVALID
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_SRCCOPY
        GC_7L_BITBLT_SHADER_SRCCOPY,
        sizeof(GC_7L_BITBLT_SHADER_SRCCOPY),
        0,
        GC_7L_CMD_COMPUTE_BITBLT_1,
        sizeof(GC_7L_CMD_COMPUTE_BITBLT_1),
        GC_7L_CMD_BITBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_BITBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_SRCINVERT
        GC_7L_BITBLT_SHADER_SRCINVERT,
        sizeof(GC_7L_BITBLT_SHADER_SRCINVERT),
        0,
        GC_7L_CMD_COMPUTE_BITBLT_2_3_4,
        sizeof(GC_7L_CMD_COMPUTE_BITBLT_2_3_4),
        GC_7L_CMD_BITBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_BITBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_SRCAND
        GC_7L_BITBLT_SHADER_SRCAND,
        sizeof(GC_7L_BITBLT_SHADER_SRCAND),
        0,
        GC_7L_CMD_COMPUTE_BITBLT_2_3_4,
        sizeof(GC_7L_CMD_COMPUTE_BITBLT_2_3_4),
        GC_7L_CMD_BITBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_BITBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_SRCOR
        GC_7L_BITBLT_SHADER_SRCOR,
        sizeof(GC_7L_BITBLT_SHADER_SRCOR),
        0,
        GC_7L_CMD_COMPUTE_BITBLT_2_3_4,
        sizeof(GC_7L_CMD_COMPUTE_BITBLT_2_3_4),
        GC_7L_CMD_BITBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_BITBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_BITBLT + DXGK_GDIROP_ROP3
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_BITBLT + SRCCOPY of A8
        GC_7L_BITBLT_SHADER_SRCCOPY_A8,
        sizeof(GC_7L_BITBLT_SHADER_SRCCOPY_A8),
        0,
        GC_7L_CMD_COMPUTE_BITBLT_1_A8,
        sizeof(GC_7L_CMD_COMPUTE_BITBLT_1_A8),
        GC_7L_CMD_BITBLT_PATCH_TABLE_A8,
        sizeof(GC_7L_CMD_BITBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_INVALID
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_PATCOPY
        GC_7L_COLORFILL_SHADER_PATCOPY,
        sizeof(GC_7L_COLORFILL_SHADER_PATCOPY),
        0,
        GC_7L_CMD_COMPUTE_COLORFILL_1,
        sizeof(GC_7L_CMD_COMPUTE_COLORFILL_1),
        GC_7L_CMD_COLORFILL_PATCH_TABLE,
        sizeof(GC_7L_CMD_COLORFILL_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_PATINVERT
        GC_7L_COLORFILL_SHADER_PATINVERT,
        sizeof(GC_7L_COLORFILL_SHADER_PATINVERT),
        0,
        GC_7L_CMD_COMPUTE_COLORFILL_2_5_6,
        sizeof(GC_7L_CMD_COMPUTE_COLORFILL_2_5_6),
        GC_7L_CMD_COLORFILL_PATCH_TABLE,
        sizeof(GC_7L_CMD_COLORFILL_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_PDXN
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_DSTINVERT
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_PATAND
        GC_7L_COLORFILL_SHADER_PATAND,
        sizeof(GC_7L_COLORFILL_SHADER_PATAND),
        0,
        GC_7L_CMD_COMPUTE_COLORFILL_2_5_6,
        sizeof(GC_7L_CMD_COMPUTE_COLORFILL_2_5_6),
        GC_7L_CMD_COLORFILL_PATCH_TABLE,
        sizeof(GC_7L_CMD_COLORFILL_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_PATOR
        GC_7L_COLORFILL_SHADER_PATOR,
        sizeof(GC_7L_COLORFILL_SHADER_PATOR),
        0,
        GC_7L_CMD_COMPUTE_COLORFILL_2_5_6,
        sizeof(GC_7L_CMD_COMPUTE_COLORFILL_2_5_6),
        GC_7L_CMD_COLORFILL_PATCH_TABLE,
        sizeof(GC_7L_CMD_COLORFILL_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_COLORFILL + DXGK_GDIROPCF_ROP3
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Generic
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Generic Per Pixel
        GC_7L_ALPHABLEND_SHADER_CONSTALPHA,
        sizeof(GC_7L_ALPHABLEND_SHADER_CONSTALPHA),
        0,
        GC_7L_CMD_COMPUTE_ALPHABLEND_1_4,
        sizeof(GC_7L_CMD_COMPUTE_ALPHABLEND_1_4),
        GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4,
        sizeof(GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Constant Alpha of 0XFF
        GC_7L_ALPHABLEND_SHADER_PERPIXEL,
        sizeof(GC_7L_ALPHABLEND_SHADER_PERPIXEL),
        0,
        GC_7L_CMD_COMPUTE_ALPHABLEND_2,
        sizeof(GC_7L_CMD_COMPUTE_ALPHABLEND_2),
        GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4,
        sizeof(GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Constant Alpha of 0xFF and 1:1
        GC_7L_ALPHABLEND_SHADER_PERPIXEL_11,
        sizeof(GC_7L_ALPHABLEND_SHADER_PERPIXEL_11),
        0,
        GC_7L_CMD_COMPUTE_ALPHABLEND_3,
        sizeof(GC_7L_CMD_COMPUTE_ALPHABLEND_3),
        GC_7L_CMD_ALPHABLEND_PATCH_TABLE_3_5,
        sizeof(GC_7L_CMD_ALPHABLEND_PATCH_TABLE_3_5)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Non Per Pixel Alpha
        GC_7L_ALPHABLEND_SHADER_NPP,
        sizeof(GC_7L_ALPHABLEND_SHADER_NPP),
        0,
        GC_7L_CMD_COMPUTE_ALPHABLEND_1_4,
        sizeof(GC_7L_CMD_COMPUTE_ALPHABLEND_1_4),
        GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4,
        sizeof(GC_7L_CMD_ALPHABLEND_PATCH_TABLE_1_2_4)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_ALPHABLEND : Non Per Pixel Alpha and 1:1
        GC_7L_ALPHABLEND_SHADER_NPP_11,
        sizeof(GC_7L_ALPHABLEND_SHADER_NPP_11),
        0,
        GC_7L_CMD_COMPUTE_ALPHABLEND_5,
        sizeof(GC_7L_CMD_COMPUTE_ALPHABLEND_5),
        GC_7L_CMD_ALPHABLEND_PATCH_TABLE_3_5,
        sizeof(GC_7L_CMD_ALPHABLEND_PATCH_TABLE_3_5)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_STRETCHBLT
        GC_7L_STRETCHBLT_SHADER,
        sizeof(GC_7L_STRETCHBLT_SHADER),
        0,
        GC_7L_CMD_COMPUTE_STRETCHBLT,
        sizeof(GC_7L_CMD_COMPUTE_STRETCHBLT),
        GC_7L_CMD_STRETCHBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_STRETCHBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_ESCAPE
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_TRANSPARENTBLT
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_TRANSPARENTBLT : HonorAlpha of 1
        GC_7L_TRANSPARENTBLT_SHADER_ALPHA,
        sizeof(GC_7L_TRANSPARENTBLT_SHADER_ALPHA),
        0,
        GC_7L_CMD_COMPUTE_TRANSPARENTBLT_1_2,
        sizeof(GC_7L_CMD_COMPUTE_TRANSPARENTBLT_1_2),
        GC_7L_CMD_TRANSPARENTBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_TRANSPARENTBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_TRANSPARENTBLT : HonorAlpha of 0
        GC_7L_TRANSPARENTBLT_SHADER_NO_ALPHA,
        sizeof(GC_7L_TRANSPARENTBLT_SHADER_NO_ALPHA),
        0,
        GC_7L_CMD_COMPUTE_TRANSPARENTBLT_1_2,
        sizeof(GC_7L_CMD_COMPUTE_TRANSPARENTBLT_1_2),
        GC_7L_CMD_TRANSPARENTBLT_PATCH_TABLE,
        sizeof(GC_7L_CMD_TRANSPARENTBLT_PATCH_TABLE)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_CLEARTYPEBLEND
        0, 0, 0, 0, 0, 0, 0
    },
    {                       // DXGK_GDIOP_CLEARTYPEBLEND : CTB
        GC_7L_CLEARTYPEBLEND_SHADER,
        sizeof(GC_7L_CLEARTYPEBLEND_SHADER),
        0,
        GC_7L_CMD_COMPUTE_CLEARTYPEBLEND_1,
        sizeof(GC_7L_CMD_COMPUTE_CLEARTYPEBLEND_1),
        GC_7L_CMD_CLEARTYPEBLEND_PATCH_TABLE_1,
        sizeof(GC_7L_CMD_CLEARTYPEBLEND_PATCH_TABLE_1)/sizeof(GC_7L_PATCH_ENTRY)
    },
    {                       // DXGK_GDIOP_CLEARTYPEBLEND : CTB Gamma of -1
        GC_7L_CLEARTYPEBLEND_SHADER_NO_GAMMA,
        sizeof(GC_7L_CLEARTYPEBLEND_SHADER_NO_GAMMA),
        0,
        GC_7L_CMD_COMPUTE_CLEARTYPEBLEND_2,
        sizeof(GC_7L_CMD_COMPUTE_CLEARTYPEBLEND_2),
        GC_7L_CMD_CLEARTYPEBLEND_PATCH_TABLE_2,
        sizeof(GC_7L_CMD_CLEARTYPEBLEND_PATCH_TABLE_2)/sizeof(GC_7L_PATCH_ENTRY)
    },
};
};


