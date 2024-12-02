/*
 * Copyright 2023 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


extern "C" {
#include "Trace.h"
#if (MALONE_TRACE == MALONE_TRACE_WPP)
#include "imxblit.tmh"
#endif
#include "imx8q_driver.h"
#include "Device.h"
#include <svc/scfw.h>
#include <svc/ipc.h>
#include <svc/pm/pm_api.h>
#include <svc/misc/misc_api.h>
}
#define uint32_t UINT32

#include "imxdpuv1_registers.h"
#include "dpr_prg_registers.h"
#include "imxblit_public.h"

#define REG_SET_FIELD(field, value) (( (value) << (field ## _SHIFT))&(field ## _MASK))
#define REG_GET_FIELD(field, reg) (((reg)&(field ## _MASK)) >> (field  ## _SHIFT))
#define IMXDPUV1_STORE9_SEQCOMPLETE_IRQ	2U
#define IMXDPUV1_STORE9_SEQCOMPLETE_IRQ_MASK	(0x1U << IMXDPUV1_STORE9_SEQCOMPLETE_IRQ)
#define IMXDPUV1_IRQSTEER_CHANnCTL_OFFSET            0x0U
#define IMXDPUV1_IRQSTEER_IRQSTEER_CH0MASK1_OFFSET   0x8U

#define ALIGN(x, y)  ((((x) + ((y)-1))/(y))*(y))

/* Index of PRG to handle LUMA */
#define LUMA_PRG_INDEX 1

struct blitCfg_t {
	UINT8 *source_paddr_y;
	UINT8 *source_paddr_uv;

	UINT32 source_width;
	UINT32 source_stride;
	UINT32 source_height;

	UINT8 *destination_paddr;

	UINT32 destination_width;
	UINT32 destination_stride;
	UINT32 destination_height;

	BOOL source_interlaced;
};

VOID WriteDCReg32(BLITTER_DEVICE_CONTEXT *blitterContextPtr, const UINT32 RegAddr, const UINT32 Val)
{
	volatile UINT32* regPtr = blitterContextPtr->DcRegPtr;

	regPtr += RegAddr / 4;
	*regPtr = Val;
}

VOID ReadDCReg32(BLITTER_DEVICE_CONTEXT *blitterContextPtr, const UINT32 RegAddr, UINT32& Val)
{
	volatile UINT32* regPtr = blitterContextPtr->DcRegPtr;

	regPtr += RegAddr / 4;
	Val = *regPtr;
}

VOID WriteDPRReg32(BLITTER_DEVICE_CONTEXT *blitterContextPtr, const UINT8 DprIndex, const UINT32 RegAddr, const UINT32 Val)
{
	volatile UINT32* regPtr;

	if (DprIndex == 0) {
		regPtr = blitterContextPtr->DprCh0RegPtr;
	}
	else {
		regPtr = blitterContextPtr->DprCh1RegPtr;
	}
	regPtr += RegAddr / 4;
	*regPtr = Val;
}

VOID ReadDPRReg32(BLITTER_DEVICE_CONTEXT *blitterContextPtr, const UINT8 DprIndex, const UINT32 RegAddr, UINT32& Val)
{
	volatile UINT32* regPtr;

	if (DprIndex == 0) {
		regPtr = blitterContextPtr->DprCh0RegPtr;
	}
	else {
		regPtr = blitterContextPtr->DprCh1RegPtr;
	}
	regPtr += RegAddr / 4;
	Val = *regPtr;
}

VOID WritePRGReg32(BLITTER_DEVICE_CONTEXT *blitterContextPtr, const UINT8 PrgIndex, const UINT32 RegAddr, const UINT32 Val)
{
	volatile UINT32* regPtr;
	if (PrgIndex == 0) {
		regPtr = blitterContextPtr->Prg0RegPtr;
	}
	else {
		regPtr = blitterContextPtr->Prg1RegPtr;
	}
	regPtr += RegAddr / 4;
	*regPtr = Val;
}

VOID ReadPRGReg32(BLITTER_DEVICE_CONTEXT *blitterContextPtr, const UINT8 PrgIndex, const UINT32 RegAddr, UINT32& Val)
{
	volatile UINT32* regPtr;

	if (PrgIndex == 0) {
		regPtr = blitterContextPtr->Prg0RegPtr;
	}
	else {
		regPtr = blitterContextPtr->Prg1RegPtr;
	}
	regPtr += RegAddr / 4;
	Val = *regPtr;
}

uint32_t get_burst_size(UINT_PTR src_paddr)
{
	unsigned long index;
	uint32_t burst_size;

	_BitScanForward(&index, (unsigned long)src_paddr);
	burst_size = 1 << (index - 1);
	burst_size = ALIGN(burst_size, 8);
	burst_size = min(burst_size, 128);

	return burst_size;
}

VOID Update_PRG(BLITTER_DEVICE_CONTEXT *blitterContextPtr, UINT8 prg, blitCfg_t *BlitCfg)
{
	/*
* dpu one burst get 8 x burst-length data; max burst-length is 16
* max-burst = 128 (8 x 16)
*/
	UINT32 src_paddr;

	if (prg == LUMA_PRG_INDEX) {
		src_paddr = (UINT32)(UINT_PTR)BlitCfg->source_paddr_y;
	}
	else {
		src_paddr = (UINT32)(UINT_PTR)BlitCfg->source_paddr_uv;
	}

	UINT32 burst_size = get_burst_size(src_paddr);
	UINT32 stride = ALIGN(BlitCfg->source_width + ALIGN(src_paddr % 8, 8), burst_size);
	UINT32 height = BlitCfg->source_height;

	if (prg != LUMA_PRG_INDEX && (stride <= burst_size)) {
		height /= 2;
	}
	
	WritePRGReg32(blitterContextPtr, prg, IMX_PRG_STRIDE, REG_SET_FIELD(IMX_PRG_STRIDE_STRIDE, stride-1));
	WritePRGReg32(blitterContextPtr, prg, IMX_PRG_WIDTH, REG_SET_FIELD(IMX_PRG_WIDTH_WIDTH, BlitCfg->source_width-1));
	WritePRGReg32(blitterContextPtr, prg, IMX_PRG_HEIGHT, REG_SET_FIELD(IMX_PRG_HEIGHT_HEIGHT, BlitCfg->source_interlaced? (height/2) - 1 : height-1));
	WritePRGReg32(blitterContextPtr, prg, IMX_PRG_OFFSET, 0);
	WritePRGReg32(blitterContextPtr, prg, IMX_PRG_BADDR, src_paddr);

	UINT32 Value = REG_SET_FIELD(IMX_PRG_CTRL_SC_DATA_TYPE, 0) |    /* 8 bit per component */
		REG_SET_FIELD(IMX_PRG_CTRL_HANDSHAKE_MODE, 1) |             /* 8 lines hansshake mode for VPU */
		REG_SET_FIELD(IMX_PRG_CTRL_SHADOW_LOAD_MODE, 1) |           /* The PRG Shadow Register Load by external CTX_ENABLE */
		REG_SET_FIELD(IMX_PRG_CTRL_DES_DATA_TYPE, 3);               /* 8bpp per pixel */
	
	if (prg != LUMA_PRG_INDEX && stride > burst_size) {
		Value |= REG_SET_FIELD(IMX_PRG_CTRL_UV_EN, 1);
	}

	WritePRGReg32(blitterContextPtr, prg, IMX_PRG_CTRL, Value);
}

VOID Update_DPR_PRG(BLITTER_DEVICE_CONTEXT *blitterContextPtr, blitCfg_t *BlitCfg)
{

	UINT32 plane_width = ALIGN(BlitCfg->source_width, 8);
	UINT32 plane1_height = ALIGN(BlitCfg->source_height, 8);
	//UINT32 plane2_height = plane1_height/2;

	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_IRQ_MASK, 0xFF);	/* Disable all interrupts */

	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_FRAME_CTRL0, REG_SET_FIELD(IMX_DPR_FRAME_CTRL0_PITCH, (BlitCfg->source_stride)));	/* Set stride */

	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_FRAME_2P_CTRL0, 0x0);	/* Maximum number of 64 bytes to prefetch per request */
	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_FRAME_2P_BASE_ADDR_CTRL0, (UINT32)(UINT_PTR)BlitCfg->source_paddr_uv);

	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_FRAME_1P_CTRL0, 0x0);	/* Maximum number of 64 bytes to prefetch per request */
	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_FRAME_1P_PIX_X_CTRL, plane_width);
	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_FRAME_1P_PIX_Y_CTRL, BlitCfg->source_interlaced? plane1_height/2: plane1_height);
	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_FRAME_1P_BASE_ADDR_CTRL0, (UINT32)(UINT_PTR)BlitCfg->source_paddr_y);

	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_FRAME_PIX_X_ULC_CTRL, 0);  /* No X offset */
	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_FRAME_PIX_Y_ULC_CTRL, 0);  /* No Y offset */

	//WriteDPRReg32(1, IMX_DPR_RTRAM_CTRL0, 0x0000003E); /* Configure RTRAM Treshold */
	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_RTRAM_CTRL0,
		REG_SET_FIELD(IMX_DPR_RTRAM_CTRL0_THRES_HIGH, 7) |
		REG_SET_FIELD(IMX_DPR_RTRAM_CTRL0_THRES_LOW, 3));
	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_MODE_CTRL0,
		REG_SET_FIELD(IMX_DPR_MODE_CTRL0_RTR_3BUF_EN, 0) |       /* Process 2 RTRAM buffers */
		REG_SET_FIELD(IMX_DPR_MODE_CTRL0_RTR_4LINE_BUF_EN, 0) |  /* 8 RTRAM lines per buffer/VPU tiles */
		REG_SET_FIELD(IMX_DPR_MODE_CTRL0_TILE_TYPE, 3) |		 /* VPU tile type */
		REG_SET_FIELD(IMX_DPR_MODE_CTRL0_YUV_EN, 1) |            /* 2-Plane YUV420 */
		REG_SET_FIELD(IMX_DPR_MODE_CTRL0_COMP_2PLANE_EN, 1) |    /* 2-Plane operation */
		REG_SET_FIELD(IMX_DPR_MODE_CTRL0_PIX_SIZE, 0)            /* 8 bits per pixel */
	);

	WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_SYSTEM_CTRL0,
		REG_SET_FIELD(IMX_DPR_SYSTEM_CTRL0_SW_SHADOW_LOAD_SEL, 1) |
		REG_SET_FIELD(IMX_DPR_SYSTEM_CTRL0_RUN_EN, 1) |
		REG_SET_FIELD(IMX_DPR_SYSTEM_CTRL0_SHADOW_LOAD_EN, 1)
	);

	/* Setup PRG */
	Update_PRG(blitterContextPtr, 0, BlitCfg);
	Update_PRG(blitterContextPtr, 1, BlitCfg);

	/* Trigger update of PRG registers */
	WritePRGReg32(blitterContextPtr, 0, IMX_PRG_REG_UPDATE, 1);
	WritePRGReg32(blitterContextPtr, 1, IMX_PRG_REG_UPDATE, 1);

}

VOID Update_DPU(BLITTER_DEVICE_CONTEXT *blitterContextPtr, blitCfg_t *BlitCfg)
{
	UINT32 source_height = BlitCfg->source_height;

	if (BlitCfg->source_interlaced)
	{
		/* In case of interlaced videos, the top half contains even lines and bottom half contains even lines. Will use only the top half and stretch.*/
		source_height /= 2;
	}

	/* Setup FetchDecode*/
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_CONTROL,
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_CONTROL_YUV422UPSAMPLINGMODE, IMXDPUV1_FETCHDECODE9_CONTROL_YUV422UPSAMPLINGMODE__INTERPOLATE) |
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_CONTROL_INPUTSELECT, IMXDPUV1_FETCHDECODE9_CONTROL_INPUTSELECT__COMPPACK)
	);

	uint32_t burst_size = get_burst_size((UINT_PTR)BlitCfg->source_paddr_y);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_BURSTBUFFERMANAGEMENT,
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_BURSTBUFFERMANAGEMENT_LINEMODE, IMXDPUV1_FETCHDECODE9_BURSTBUFFERMANAGEMENT_LINEMODE__BLIT) |
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_BURSTBUFFERMANAGEMENT_SETNUMBUFFERS, 16) |
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_BURSTBUFFERMANAGEMENT_SETBURSTLENGTH, burst_size/8)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_BASEADDRESS0, (UINT32) (UINT_PTR)BlitCfg->source_paddr_y);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_SOURCEBUFFERATTRIBUTES0,
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_SOURCEBUFFERATTRIBUTES0_BITSPERPIXEL0, 8) | 
#ifdef LINEAR
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_SOURCEBUFFERATTRIBUTES0_STRIDE0, ALIGN(BlitCfg->source_stride, burst_size) - 1)
#else
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_SOURCEBUFFERATTRIBUTES0_STRIDE0, ALIGN(BlitCfg->source_width, burst_size)-1)
#endif

	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_SOURCEBUFFERDIMENSION0,
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_SOURCEBUFFERDIMENSION0_LINECOUNT0, source_height - 1) |
#ifdef LINEAR
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_SOURCEBUFFERDIMENSION0_LINEWIDTH0, BlitCfg->source_stride - 1)
#else
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_SOURCEBUFFERDIMENSION0_LINEWIDTH0, BlitCfg->source_width - 1)
#endif
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_COLORCOMPONENTBITS0,
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_COLORCOMPONENTBITS0_COMPONENTBITSRED0, 8)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_COLORCOMPONENTSHIFT0, 0);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_LAYEROFFSET0, 0);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_CLIPWINDOWOFFSET0, 0);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_CLIPWINDOWDIMENSIONS0,
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_CLIPWINDOWDIMENSIONS0_CLIPWINDOWHEIGHT0, source_height - 1) |
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_CLIPWINDOWDIMENSIONS0_CLIPWINDOWWIDTH0, BlitCfg->source_width - 1)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_CONSTANTCOLOR0, 0xFF);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_LAYERPROPERTY0,
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_LAYERPROPERTY0_SOURCEBUFFERENABLE0, 1) |
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_LAYERPROPERTY0_CLIPWINDOWENABLE0, 1) |
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_LAYERPROPERTY0_YUVCONVERSIONMODE0, IMXDPUV1_FETCHDECODE9_LAYERPROPERTY0_YUVCONVERSIONMODE0__ITU601_FR) |
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_LAYERPROPERTY0_ALPHASRCENABLE0, 1)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_FRAMEDIMENSIONS,
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_FRAMEDIMENSIONS_FRAMEHEIGHT, source_height - 1) |
		REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_FRAMEDIMENSIONS_FRAMEWIDTH, BlitCfg->source_width - 1)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_FRAMERESAMPLING, 0x00104000);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_FRAMEPROPERTIES0, 0x00);

	/* Setup FetchWarp*/
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_CONTROL, 0x0);

	burst_size = get_burst_size((UINT_PTR)BlitCfg->source_paddr_uv);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_BURSTBUFFERMANAGEMENT,
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_BURSTBUFFERMANAGEMENT_LINEMODE, IMXDPUV1_FETCHWARP9_BURSTBUFFERMANAGEMENT_LINEMODE__BLIT) |
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_BURSTBUFFERMANAGEMENT_SETNUMBUFFERS, 16) |
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_BURSTBUFFERMANAGEMENT_SETBURSTLENGTH, burst_size / 8)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_BASEADDRESS0, (UINT32) (UINT_PTR)BlitCfg->source_paddr_uv);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_SOURCEBUFFERATTRIBUTES0,
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_SOURCEBUFFERATTRIBUTES0_BITSPERPIXEL0, 16) |
#ifdef LINEAR
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_SOURCEBUFFERATTRIBUTES0_STRIDE0, ALIGN(BlitCfg->source_stride, burst_size) - 1)
#else
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_SOURCEBUFFERATTRIBUTES0_STRIDE0, ALIGN(BlitCfg->source_width, burst_size)-1)
#endif

	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_SOURCEBUFFERDIMENSION0,
#ifdef LINEAR
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_SOURCEBUFFERDIMENSION0_LINECOUNT0, (BlitCfg->source_height / 2) - 1) |
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_SOURCEBUFFERDIMENSION0_LINEWIDTH0, (BlitCfg->source_stride / 2) - 1)
#else
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_SOURCEBUFFERDIMENSION0_LINECOUNT0, source_height - 1) |
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_SOURCEBUFFERDIMENSION0_LINEWIDTH0, BlitCfg->source_width - 1)
#endif
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_COLORCOMPONENTBITS0,
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_COLORCOMPONENTBITS0_COMPONENTBITSGREEN0, 8) | 
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_COLORCOMPONENTBITS0_COMPONENTBITSBLUE0, 8)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_COLORCOMPONENTSHIFT0,
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_COLORCOMPONENTSHIFT0_COMPONENTSHIFTGREEN0, 0) |
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_COLORCOMPONENTSHIFT0_COMPONENTSHIFTBLUE0, 8));
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_LAYEROFFSET0, 0);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_CLIPWINDOWOFFSET0, 0);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_CLIPWINDOWDIMENSIONS0,
#ifdef LINEAR
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_CLIPWINDOWDIMENSIONS0_CLIPWINDOWHEIGHT0, (BlitCfg->source_height / 2) - 1) |
#else
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_CLIPWINDOWDIMENSIONS0_CLIPWINDOWHEIGHT0, source_height - 1) |
#endif
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_CLIPWINDOWDIMENSIONS0_CLIPWINDOWWIDTH0, (BlitCfg->source_width / 2) - 1)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_CONSTANTCOLOR0, 0xFF);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_LAYERPROPERTY0,
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_LAYERPROPERTY0_SOURCEBUFFERENABLE0, 1) |
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_LAYERPROPERTY0_CLIPWINDOWENABLE0, 1) |
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_LAYERPROPERTY0_ALPHASRCENABLE0, 1)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_FRAMEDIMENSIONS,
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_FRAMEDIMENSIONS_FRAMEHEIGHT, source_height - 1) |
		REG_SET_FIELD(IMXDPUV1_FETCHWARP9_FRAMEDIMENSIONS_FRAMEWIDTH, BlitCfg->source_width - 1)
	);
#ifdef LINEAR
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_FRAMERESAMPLING, 0x00082000);
#else
	WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_FRAMERESAMPLING, 0x00102000);
#endif


	/* Setup Store*/
	WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_CONTROL, 0x0);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_BURSTBUFFERMANAGEMENT,
		REG_SET_FIELD(IMXDPUV1_STORE9_BURSTBUFFERMANAGEMENT_SETBURSTLENGTH, 4)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_BASEADDRESS, (UINT32)(UINT_PTR)BlitCfg->destination_paddr);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_DESTINATIONBUFFERATTRIBUTES,
		REG_SET_FIELD(IMXDPUV1_STORE9_DESTINATIONBUFFERATTRIBUTES_BITSPERPIXEL, 32) |
		REG_SET_FIELD(IMXDPUV1_STORE9_DESTINATIONBUFFERATTRIBUTES_STRIDE, BlitCfg->destination_stride - 1)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_DESTINATIONBUFFERDIMENSION,
		REG_SET_FIELD(IMXDPUV1_STORE9_DESTINATIONBUFFERDIMENSION_LINECOUNT, BlitCfg->destination_height - 1) |
		REG_SET_FIELD(IMXDPUV1_STORE9_DESTINATIONBUFFERDIMENSION_LINEWIDTH, BlitCfg->destination_width - 1)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_FRAMEOFFSET, 0x0);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_COLORCOMPONENTBITS,
		REG_SET_FIELD(IMXDPUV1_STORE9_COLORCOMPONENTBITS_COMPONENTBITSRED, 8) |
		REG_SET_FIELD(IMXDPUV1_STORE9_COLORCOMPONENTBITS_COMPONENTBITSGREEN, 8) |
		REG_SET_FIELD(IMXDPUV1_STORE9_COLORCOMPONENTBITS_COMPONENTBITSBLUE, 8) |
		REG_SET_FIELD(IMXDPUV1_STORE9_COLORCOMPONENTBITS_COMPONENTBITSALPHA, 8)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_COLORCOMPONENTSHIFT,
		REG_SET_FIELD(IMXDPUV1_STORE9_COLORCOMPONENTSHIFT_COMPONENTSHIFTRED, 0x10) |
		REG_SET_FIELD(IMXDPUV1_STORE9_COLORCOMPONENTSHIFT_COMPONENTSHIFTGREEN, 0x8) |
		REG_SET_FIELD(IMXDPUV1_STORE9_COLORCOMPONENTSHIFT_COMPONENTSHIFTBLUE, 0) |
		REG_SET_FIELD(IMXDPUV1_STORE9_COLORCOMPONENTSHIFT_COMPONENTSHIFTALPHA, 0x18)
	);

	if (BlitCfg->source_interlaced)
	{
		/* Set scaler and blit path for stretching interlaced video */
		WriteDCReg32(blitterContextPtr, IMXDPUV1_MATRIX9_CONTROL, 0x0);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_MATRIX9_RED0, 0x0);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_MATRIX9_RED1, 0x0);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_MATRIX9_GREEN0, 0x0);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_MATRIX9_GREEN1, 0x0);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_MATRIX9_BLUE0, 0x0);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_MATRIX9_BLUE1, 0x0);

		WriteDCReg32(blitterContextPtr, IMXDPUV1_HSCALER9_CONTROL,
			REG_SET_FIELD(IMXDPUV1_HSCALER9_CONTROL_OUTPUT_SIZE, BlitCfg->destination_width - 1) |
			REG_SET_FIELD(IMXDPUV1_HSCALER9_CONTROL_FILTER_MODE, IMXDPUV1_HSCALER9_CONTROL_FILTER_MODE__LINEAR) |
			REG_SET_FIELD(IMXDPUV1_HSCALER9_CONTROL_SCALE_MODE, IMXDPUV1_HSCALER9_CONTROL_SCALE_MODE__UPSCALE) |
			REG_SET_FIELD(IMXDPUV1_HSCALER9_CONTROL_MODE, IMXDPUV1_HSCALER9_CONTROL_MODE__ACTIVE)
		);

		WriteDCReg32(blitterContextPtr, IMXDPUV1_HSCALER9_SETUP1, 0x00080000);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_HSCALER9_SETUP2, 0x00000000);

		WriteDCReg32(blitterContextPtr, IMXDPUV1_VSCALER9_CONTROL,
			REG_SET_FIELD(IMXDPUV1_VSCALER9_CONTROL_OUTPUT_SIZE, BlitCfg->destination_height - 1) |
			REG_SET_FIELD(IMXDPUV1_VSCALER9_CONTROL_FILTER_MODE, IMXDPUV1_VSCALER9_CONTROL_FILTER_MODE__LINEAR) |
			REG_SET_FIELD(IMXDPUV1_VSCALER9_CONTROL_SCALE_MODE, IMXDPUV1_VSCALER9_CONTROL_SCALE_MODE__UPSCALE) |
			REG_SET_FIELD(IMXDPUV1_VSCALER9_CONTROL_MODE, IMXDPUV1_VSCALER9_CONTROL_MODE__ACTIVE)
		);

		WriteDCReg32(blitterContextPtr, IMXDPUV1_VSCALER9_SETUP1, 0x00040000);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_VSCALER9_SETUP2, 0x00000000);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_VSCALER9_SETUP3, 0x00000000);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_VSCALER9_SETUP4, 0x00000000);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_VSCALER9_SETUP5, 0x00000000);

		/* Setup blit path */
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_FETCHWARP9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_FETCHWARP9_DYNAMIC_FETCHWARP9_SRC_SEL, IMXDPUV1_PIXENGCFG_FETCHWARP9_DYNAMIC_FETCHWARP9_SRC_SEL__DISABLE));
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_FETCHDECODE9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_FETCHDECODE9_DYNAMIC_FETCHDECODE9_SRC_SEL, IMXDPUV1_PIXENGCFG_FETCHDECODE9_DYNAMIC_FETCHDECODE9_SRC_SEL__FETCHPERSP9)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_CLKEN, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_CLKEN__AUTOMATIC) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_PRIM_SEL, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_PRIM_SEL__FETCHDECODE9)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC_MATRIX9_CLKEN, IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC_MATRIX9_CLKEN__AUTOMATIC) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC_MATRIX9_SRC_SEL, IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC_MATRIX9_SRC_SEL__ROP9)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC_HSCALER9_CLKEN, IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC_HSCALER9_CLKEN__AUTOMATIC) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC_HSCALER9_SRC_SEL, IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC_HSCALER9_SRC_SEL__VSCALER9)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC_VSCALER9_CLKEN, IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC_VSCALER9_CLKEN__AUTOMATIC) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC_VSCALER9_SRC_SEL, IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC_VSCALER9_SRC_SEL__MATRIX9)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_PRIM_SEL, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_PRIM_SEL__DISABLE) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_SEC_SEL, IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_SEC_SEL__DISABLE)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_STORE9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_STORE9_DYNAMIC_STORE9_SRC_SEL, IMXDPUV1_PIXENGCFG_STORE9_DYNAMIC_STORE9_SRC_SEL__HSCALER9)
		);
	}
	else
	{
		/* Setup blit path */
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_FETCHWARP9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_FETCHWARP9_DYNAMIC_FETCHWARP9_SRC_SEL, IMXDPUV1_PIXENGCFG_FETCHWARP9_DYNAMIC_FETCHWARP9_SRC_SEL__DISABLE));
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_FETCHDECODE9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_FETCHDECODE9_DYNAMIC_FETCHDECODE9_SRC_SEL, IMXDPUV1_PIXENGCFG_FETCHDECODE9_DYNAMIC_FETCHDECODE9_SRC_SEL__FETCHPERSP9)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_PRIM_SEL, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_PRIM_SEL__DISABLE)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC_MATRIX9_SRC_SEL, IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC_MATRIX9_SRC_SEL__DISABLE)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC_HSCALER9_SRC_SEL, IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC_HSCALER9_SRC_SEL__DISABLE)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC_VSCALER9_SRC_SEL, IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC_VSCALER9_SRC_SEL__DISABLE)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_PRIM_SEL, IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_PRIM_SEL__DISABLE) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_SEC_SEL, IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_SEC_SEL__DISABLE)
		);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_STORE9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_STORE9_DYNAMIC_STORE9_SRC_SEL, IMXDPUV1_PIXENGCFG_STORE9_DYNAMIC_STORE9_SRC_SEL__FETCHDECODE9)
		);
	}
	/* Trigger reconfiguration */
	WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_STORE9_TRIGGER,
		REG_SET_FIELD(IMXDPUV1_PIXENGCFG_STORE9_TRIGGER_STORE9_SYNC_TRIGGER, 1)
	);
	WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_START,
		REG_SET_FIELD(IMXDPUV1_STORE9_START_START, 1)
	);
}

BOOLEAN IsOpComplete(BLITTER_DEVICE_CONTEXT *blitterContextPtr)
{
	ASSERT(blitterContextPtr->blitter_status == BLIT_BUSY);
	UINT32 Value;
	BOOLEAN completed = FALSE;

	ReadDCReg32(blitterContextPtr, IMXDPUV1_COMCTRL_INTERRUPTSTATUS0, Value);

	if (Value & IMXDPUV1_STORE9_SEQCOMPLETE_IRQ_MASK) {
		WriteDCReg32(blitterContextPtr, IMXDPUV1_COMCTRL_INTERRUPTCLEAR0, IMXDPUV1_STORE9_SEQCOMPLETE_IRQ_MASK);
		/* In case we have detected the interrupt, break the loop */
		completed = TRUE;
		blitterContextPtr->blitter_status = BLIT_IDLE;
	}

	return completed;
}

BOOLEAN CheckForInterrupt(BLITTER_DEVICE_CONTEXT *blitterContextPtr)
{
	UINT32 Value;

	// for interrupt, use IMXDPUV1_COMCTRL_USERINTERRUPTCLEAR0, IMXDPUV1_COMCTRL_USERINTERRUPTSTATUS0

	ReadDCReg32(blitterContextPtr, IMXDPUV1_COMCTRL_INTERRUPTSTATUS0, Value);

	if (Value & IMXDPUV1_STORE9_SEQCOMPLETE_IRQ_MASK) {
		WriteDCReg32(blitterContextPtr, IMXDPUV1_COMCTRL_INTERRUPTCLEAR0, IMXDPUV1_STORE9_SEQCOMPLETE_IRQ_MASK);
		/* In case we have detected the interrupt, break the loop */
		return 1;
	}

	return 0;
}

#if 0
VOID WaitComplete(BLITTER_DEVICE_CONTEXT *blitterContextPtr)
{
	
	WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_STORE9_TRIGGER,
		REG_SET_FIELD(IMXDPUV1_PIXENGCFG_STORE9_TRIGGER_STORE9_TRIGGER_SEQUENCE_COMPLETE, 1)
	);

	//uint64_t deadline = deadline_init(MSEC_TO_NS(100));
	do {
		/* Test the interrupt flag */
		// UINT32 Value;

		{ // 2 ms delay.
			LARGE_INTEGER Interval;
			// Interval.QuadPart = -2 * 1000 * 10; // Interval is in 100 ns units.
			Interval.QuadPart = -1 * 1000 * 10; // Interval is in 100 ns units.
			KeDelayExecutionThread(KernelMode, FALSE, &Interval);
		}


		if (CheckForInterrupt(blitterContextPtr))
		{
			break;
		}
		//usleep(1000);
	//} while (!deadline_is_expired(deadline));
	} while (1);
	{ // 2 ms delay.
		LARGE_INTEGER Interval;
		// Interval.QuadPart = -2 * 1000 * 10; // Interval is in 100 ns units.
		Interval.QuadPart = -4 * 1000 * 10; // Interval is in 100 ns units.
		KeDelayExecutionThread(KernelMode, FALSE, &Interval);
	}
}
#endif

BOOLEAN BlitEvtInterruptIsr(_In_ WDFINTERRUPT WdfInterrupt, _In_ ULONG MessageID)
/*!
 * EvtInterruptIsr callback handles CSI errors and schedules DPC for finished frames.
 *
 * @param WdfInterrupt handle to WDF interrupt object.
 * @param MessageID device is not using MSI, always zero.
 *
 * @returns TRUE if interrupt has been serviced.
 */
{
	UNREFERENCED_PARAMETER(MessageID);
	BOOLEAN servicedIrq = false;
	MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr = DeviceGetContext(WdfInterruptGetDevice(WdfInterrupt));

	ASSERT(deviceContextPtr);
	BLITTER_DEVICE_CONTEXT *blitterCtxPtr = &deviceContextPtr->BlitterCtx;

	if (CheckForInterrupt(blitterCtxPtr)) {
		servicedIrq = true;
		//
		KeQuerySystemTimePrecise(&blitterCtxPtr->m_BlitDoneIsrTime);
		blitterCtxPtr->m_BlitDurationToIsrMs = ((blitterCtxPtr->m_BlitDoneIsrTime.QuadPart - blitterCtxPtr->m_BlitStartTime.QuadPart) / 10000);
		DBG_PERF_PRINT_INFO("Blitter duration from START to ISR in miliseconds: %llu\n", blitterCtxPtr->m_BlitDurationToIsrMs);
		//DBG_EVENTS_PRINT_INFO("G2D->KM ISR() Blit done");
		WdfDpcEnqueue(blitterCtxPtr->Dpc); // Release VFB from Driver
	}
	// Consider barrier after IRQ disable.

	return servicedIrq;
}

VOID BlitEvtDpc(_In_ WDFDPC Dpc)
{
	MALONE_VPU_DEVICE_CONTEXT *deviceContextPtr = DeviceGetContext(WdfDpcGetParentObject(Dpc));
	ASSERT(deviceContextPtr);
	BLITTER_DEVICE_CONTEXT *blitterCtxPtr = &deviceContextPtr->BlitterCtx;
	//
	KeQuerySystemTimePrecise(&blitterCtxPtr->m_BlitDoneDpcTime);
	blitterCtxPtr->m_BlitDurationToIsrMs = ((blitterCtxPtr->m_BlitDoneDpcTime.QuadPart - blitterCtxPtr->m_BlitStartTime.QuadPart) / 10000);
	t2l_frame_done(blitterCtxPtr->vdec_stream_ctx, blitterCtxPtr->vfb, blitterCtxPtr->ofb);
	DBG_PERF_PRINT_INFO("Blitter duration from START to DPC in miliseconds: %llu\n", blitterCtxPtr->m_BlitDurationToIsrMs);
	DBG_EVENTS_PRINT_INFO("G2D->KM DPC() Blit done, state: BUSY -> IDLE, ofb index=%d, vfb index=%d", blitterCtxPtr->ofb->index, blitterCtxPtr->vfb->index);
	blitterCtxPtr->blitter_status = BLIT_IDLE;
	t2l_frame(blitterCtxPtr->vdec_stream_ctx);

}

NTSTATUS BlitPrepareDpuHw(BLITTER_DEVICE_CONTEXT *blitterContextPtr)
/*!
 * Passes Initializes CSI2RX peripheral module defaults.
 *
 * @param MipiRes containing resources and information from ACPI.
 *
 * @returns STATUS_SUCCESS or error code.
 */
{
	NTSTATUS status = STATUS_SUCCESS;

	{
		sc_ipc_struct_t mu_ipcHandle = { 0 };

		status = sc_ipc_open(&mu_ipcHandle, &blitterContextPtr->DeviceContextPtr->scfw_ipc_id); // FIXME This opening and closing MU might interleave and thus fail.

		/* Setup PRG for NV12 Amphion tiled blit */
		sc_err_t scError = sc_misc_set_control(&mu_ipcHandle, SC_R_DC_0_BLIT0, SC_C_SEL0, 1);
		if (scError != SC_ERR_NONE) {
			status = STATUS_UNSUCCESSFUL;
			//DoTrace(TRACE_LEVEL_ERROR, "sc_misc_set_control failed 0x%x", scError);
			//
		}

		scError = sc_misc_set_control(&mu_ipcHandle, SC_R_DC_0, SC_C_KACHUNK_CNT, 64);
		if (scError != SC_ERR_NONE) {
			status = STATUS_UNSUCCESSFUL;
			//DoTrace(TRACE_LEVEL_ERROR, "sc_misc_set_control failed 0x%x", scError);
			//
		}
		sc_ipc_close(&mu_ipcHandle);
	}
	if (NT_SUCCESS(status)) {
		/* Initialize DPU blit engine */
		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_STORE9_STATIC, IMXDPUV1_PIXENGCFG_STORE9_STATIC_RESET_VALUE);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_STATICCONTROL, IMXDPUV1_FETCHDECODE9_STATICCONTROL_RESET_VALUE);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_STATICCONTROL, IMXDPUV1_FETCHWARP9_STATICCONTROL_RESET_VALUE);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHECO9_STATICCONTROL, IMXDPUV1_FETCHECO9_STATICCONTROL_RESET_VALUE);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_MATRIX9_STATICCONTROL, IMXDPUV1_MATRIX9_STATICCONTROL_RESET_VALUE);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_ROP9_STATICCONTROL, IMXDPUV1_ROP9_STATICCONTROL_RESET_VALUE);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_HSCALER9_STATICCONTROL, IMXDPUV1_HSCALER9_STATICCONTROL_RESET_VALUE);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_VSCALER9_STATICCONTROL, IMXDPUV1_VSCALER9_STATICCONTROL_RESET_VALUE);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_BLITBLEND9_STATICCONTROL, IMXDPUV1_BLITBLEND9_STATICCONTROL_RESET_VALUE);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_STATICCONTROL, IMXDPUV1_STORE9_STATICCONTROL_RESET_VALUE);

		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_STORE9_STATIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_STORE9_STATIC_STORE9_SHDEN, 1) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_STORE9_STATIC_STORE9_POWERDOWN, 0) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_STORE9_STATIC_STORE9_SYNC_MODE, IMXDPUV1_PIXENGCFG_STORE9_STATIC_STORE9_SYNC_MODE__SINGLE) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_STORE9_STATIC_STORE9_DIV, 0x80));
		WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHDECODE9_STATICCONTROL,
			REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_STATICCONTROL_SHDEN, 1) |
			REG_SET_FIELD(IMXDPUV1_FETCHDECODE9_STATICCONTROL_BASEADDRESSAUTOUPDATE, 0));
		WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHWARP9_STATICCONTROL,
			REG_SET_FIELD(IMXDPUV1_FETCHWARP9_STATICCONTROL_SHDEN, 1) |
			REG_SET_FIELD(IMXDPUV1_FETCHWARP9_STATICCONTROL_BASEADDRESSAUTOUPDATE, 0) |
			IMXDPUV1_FETCHWARP9_STATICCONTROL_RESET_VALUE);
		WriteDCReg32(blitterContextPtr, IMXDPUV1_FETCHECO9_STATICCONTROL,
			REG_SET_FIELD(IMXDPUV1_FETCHECO9_STATICCONTROL_SHDEN, 1) |
			REG_SET_FIELD(IMXDPUV1_FETCHECO9_STATICCONTROL_BASEADDRESSAUTOUPDATE, 0));
		WriteDCReg32(blitterContextPtr, IMXDPUV1_ROP9_STATICCONTROL,
			REG_SET_FIELD(IMXDPUV1_ROP9_STATICCONTROL_SHDEN, 1));
		WriteDCReg32(blitterContextPtr, IMXDPUV1_MATRIX9_STATICCONTROL,
			REG_SET_FIELD(IMXDPUV1_MATRIX9_STATICCONTROL_SHDEN, 1));
		WriteDCReg32(blitterContextPtr, IMXDPUV1_HSCALER9_STATICCONTROL,
			REG_SET_FIELD(IMXDPUV1_HSCALER9_STATICCONTROL_SHDEN, 1));
		WriteDCReg32(blitterContextPtr, IMXDPUV1_VSCALER9_STATICCONTROL,
			REG_SET_FIELD(IMXDPUV1_VSCALER9_STATICCONTROL_SHDEN, 1));
		WriteDCReg32(blitterContextPtr, IMXDPUV1_BLITBLEND9_STATICCONTROL,
			REG_SET_FIELD(IMXDPUV1_BLITBLEND9_STATICCONTROL_SHDEN, 1));
		WriteDCReg32(blitterContextPtr, IMXDPUV1_STORE9_STATICCONTROL,
			REG_SET_FIELD(IMXDPUV1_STORE9_STATICCONTROL_SHDEN, 1) |
			REG_SET_FIELD(IMXDPUV1_STORE9_STATICCONTROL_BASEADDRESSAUTOUPDATE, 0));

		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_PRIM_SEL, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_PRIM_SEL__FETCHPERSP9) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_SEC_SEL, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_SEC_SEL__DISABLE) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_TERT_SEL, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_TERT_SEL__DISABLE) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_CLKEN, IMXDPUV1_PIXENGCFG_ROP9_DYNAMIC_ROP9_CLKEN__AUTOMATIC));

		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC_MATRIX9_SRC_SEL, IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC_MATRIX9_SRC_SEL__ROP9) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC_MATRIX9_CLKEN, IMXDPUV1_PIXENGCFG_MATRIX9_DYNAMIC_MATRIX9_CLKEN__AUTOMATIC));

		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC_HSCALER9_SRC_SEL, IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC_HSCALER9_SRC_SEL__MATRIX9) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC_HSCALER9_CLKEN, IMXDPUV1_PIXENGCFG_HSCALER9_DYNAMIC_HSCALER9_CLKEN__AUTOMATIC));

		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC_VSCALER9_SRC_SEL, IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC_VSCALER9_SRC_SEL__HSCALER9) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC_VSCALER9_CLKEN, IMXDPUV1_PIXENGCFG_VSCALER9_DYNAMIC_VSCALER9_CLKEN__AUTOMATIC));

		WriteDCReg32(blitterContextPtr, IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_PRIM_SEL, IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_PRIM_SEL__VSCALER9) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_SEC_SEL, IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_SEC_SEL__DISABLE) |
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_CLKEN, IMXDPUV1_PIXENGCFG_BLITBLEND9_DYNAMIC_BLITBLEND9_CLKEN__AUTOMATIC));

		/* Reset DPR*/
		WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_SYSTEM_CTRL0 + IMX_DPR_PRG_SET_OFFSET, IMX_DPR_SYSTEM_CTRL0_SOFT_RESET_MASK);
		{
			// 100us delay.
			LARGE_INTEGER Interval;
			Interval.QuadPart = -100 * 100; // Interval is number of msecs in 100 ns units.
			KeDelayExecutionThread(KernelMode, FALSE, &Interval);
		}
		WriteDPRReg32(blitterContextPtr, 1, IMX_DPR_SYSTEM_CTRL0 + IMX_DPR_PRG_CLR_OFFSET, IMX_DPR_SYSTEM_CTRL0_SOFT_RESET_MASK);

		/* Reset PRG */
		WritePRGReg32(blitterContextPtr, 0, IMX_PRG_CTRL + IMX_DPR_PRG_SET_OFFSET, IMX_PRG_CTRL_SOFTRST_MASK);
		WritePRGReg32(blitterContextPtr, 1, IMX_PRG_CTRL + IMX_DPR_PRG_SET_OFFSET, IMX_PRG_CTRL_SOFTRST_MASK);

		{
			// 100us delay.
			LARGE_INTEGER Interval;
			Interval.QuadPart = -100 * 100; // Interval is number of msecs in 100 ns units.
			KeDelayExecutionThread(KernelMode, FALSE, &Interval);
		}

		WritePRGReg32(blitterContextPtr, 0, IMX_PRG_CTRL + IMX_DPR_PRG_CLR_OFFSET, IMX_PRG_CTRL_SOFTRST_MASK);
		WritePRGReg32(blitterContextPtr, 1, IMX_PRG_CTRL + IMX_DPR_PRG_CLR_OFFSET, IMX_PRG_CTRL_SOFTRST_MASK);

		/* Enable Sequence complete Interrupt */
		InterlockedOr((volatile long *)blitterContextPtr->DcRegPtr + (IMXDPUV1_COMCTRL_USERINTERRUPTENABLE0 / 4), IMXDPUV1_STORE9_SEQCOMPLETE_IRQ_MASK);

		/* Enable Sequence complete Interrupt in IRQ Steer */
		InterlockedOr((volatile long *)(blitterContextPtr->IrqSteerRegPtr + IMXDPUV1_IRQSTEER_CHANnCTL_OFFSET / 4), 1);
		InterlockedOr((volatile long *)(blitterContextPtr->IrqSteerRegPtr + IMXDPUV1_IRQSTEER_IRQSTEER_CH0MASK1_OFFSET / 4), IMXDPUV1_STORE9_SEQCOMPLETE_IRQ_MASK);
	}

    return status;
}

// void BlitStartBlit(BLITTER_DEVICE_CONTEXT *blitterContextPtr, vdec_ctx_t *Ctx /* buffers, resolution */)
NTSTATUS BlitStartBlit(vdec_ctx_t *VpuCtx, ofb_t *ofb, vfb_t *vfb)
/*!
 * Try queue non-blocking background Blit operation.
 *
 * @param dev ..
 */
{
	NTSTATUS status = STATUS_SUCCESS;
	UINT32 IPIdentifier;
	BLITTER_DEVICE_CONTEXT *blitterCtxPtr = &VpuCtx->dev->BlitterCtx;
	blitCfg_t blitCfg;

	ASSERT(blitterCtxPtr->blitter_status == BLIT_IDLE);

	blitterCtxPtr->vdec_stream_ctx = VpuCtx;
	// TODO: We could still fail! Remember to zero following lines back in such case.
	blitterCtxPtr->blitter_status = BLIT_BUSY;
	blitterCtxPtr->ofb = ofb;
	blitterCtxPtr->vfb = vfb;

	/* Source parameters */
	blitCfg.source_paddr_y = (UINT8*)vfb->luma.physAddrAligned; // FIXME types 
	blitCfg.source_paddr_uv = (UINT8 *)vfb->chroma.physAddrAligned;

	blitCfg.source_width = VpuCtx->width;
	blitCfg.source_stride = blitCfg.source_width;
	blitCfg.source_height = VpuCtx->height;
	blitCfg.source_interlaced = !VpuCtx->pSeqinfo->uProgressive;

	/* Destination parameters */
	blitCfg.destination_paddr = (UINT8*)ofb->mem.physAddrAligned;

	blitCfg.destination_width = ofb->width; // TODO It's actually ofb->owidth, shall we rework way this resolution is passed aroud Blitter and user space?
	blitCfg.destination_stride = ofb->width; // ofb->ostride; // FIXME (ofb->ostride + (BYTES_PER_PIXEL_RGBA - 1)) / BYTES_PER_PIXEL_RGBA;
	blitCfg.destination_stride = 4* blitCfg.destination_stride; // Value represented in Bytes thus times four.
	blitCfg.destination_height = ofb->height; // ofb->oheight;

	if (NT_SUCCESS(status)) {
		ReadDCReg32(blitterCtxPtr, IMXDPUV1_COMCTRL_IPIDENTIFIER, IPIdentifier);

		if (IPIdentifier != 0) {
#ifndef LINEAR
			Update_DPR_PRG(blitterCtxPtr, &blitCfg);
#endif
			Update_DPU(blitterCtxPtr, &blitCfg);
#if 0
			{ // 50 ms delay.
				LARGE_INTEGER Interval;
				Interval.QuadPart = -50 * 1000 * 10; // Interval is number of msecs in 100 ns units.
				KeDelayExecutionThread(KernelMode, FALSE, &Interval);
			}

			UINT32 Value;
			ReadDCReg32(IMXDPUV1_STORE9_WRITEADDRESS, Value);
			//if (Value != 0) {
	//			//
			//}
			KdPrint(("STORE9_WRITEADDRESS = (0x%x)\r\n", Value));
			ReadPRGReg32(0, IMX_PRG_STATUS, Value);
			KdPrint(("PRG0 STATUS = (0x%x)\r\n", Value));
			ReadPRGReg32(1, IMX_PRG_STATUS, Value);
			KdPrint(("PRG1 STATUS = (0x%x)\r\n", Value));
			ReadDPRReg32(1, IMX_DPR_IRQ_NONMASK_STATUS, Value);
			KdPrint(("DPR1 IRQ_NONMASK_STATU = (0x%x)\r\n", Value));

			ReadDCReg32(IMXDPUV1_FETCHDECODE9_STATUS, Value);
			KdPrint(("FETCHDECODE9_STATUS = (0x%x)\r\n", Value));
			ReadDCReg32(IMXDPUV1_FETCHDECODE9_HIDDENSTATUS, Value);
			KdPrint(("FETCHDECODE9_HIDDENSTATUS = (0x%x)\r\n", Value));
			ReadDCReg32(IMXDPUV1_FETCHWARP9_STATUS, Value);
			KdPrint(("FETCHWARP9_STATUS = (0x%x)\r\n", Value));
			ReadDCReg32(IMXDPUV1_FETCHWARP9_HIDDENSTATUS, Value);
			KdPrint(("FETCHWARP9_HIDDENSTATUS = (0x%x)\r\n", Value));
			ReadDCReg32(IMXDPUV1_STORE9_STATUS, Value);
			KdPrint(("STORE9_STATUS = (0x%x)\r\n", Value));
#endif
		KeQuerySystemTimePrecise(&blitterCtxPtr->m_BlitStartTime);

		}
		// FIXME: Peter, is this call ok? I was not getting interupts without it.
		WriteDCReg32(blitterCtxPtr, IMXDPUV1_PIXENGCFG_STORE9_TRIGGER,
			REG_SET_FIELD(IMXDPUV1_PIXENGCFG_STORE9_TRIGGER_STORE9_TRIGGER_SEQUENCE_COMPLETE, 1)
		);
		DBG_EVENTS_PRINT_INFO("KM->G2D Blit start, state: IDLE -> BUSY, ofb index=%d, vfb index=%d", ofb->index, vfb->index);
	}

	return status;
}
