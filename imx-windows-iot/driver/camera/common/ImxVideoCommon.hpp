/*
 * Copyright 2017, 2020-2022 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Pixel format FOURCC. */
#define FSL_VIDEO_FOURCC(a, b, c, d) \
    ((UINT32)(a) | ((UINT32)(b) << 8U) | ((UINT32)(c) << 16U) | ((UINT32)(d) << 24U))

/*! @brief Macro to define resolution. */
#define FSL_VIDEO_RESOLUTION(width, height) ((UINT32)(width) | ((UINT32)(height) << 16U))

#define FSL_VIDEO_EXTRACT_WIDTH(resolution)  ((UINT16)((resolution)&0xFFFFU))
#define FSL_VIDEO_EXTRACT_HEIGHT(resolution) ((UINT16)((resolution) >> 16U))

#define DEVICE_ENDPOINT_NAME_MAX_LEN 256

/*! @brief Pixel format definition. */
typedef enum _video_pixel_format : unsigned long
{
    kVIDEO_PixelFormatNone = 0x0,
    /* RGB */
    kVIDEO_PixelFormatRGB888 = 0xe436eb7d, /*!< 24-bit RGB888. */
    kVIDEO_PixelFormatXRGB8888 = 0xe436eb7e, /*!< 32-bit XRGB8888. */
    kVIDEO_PixelFormatARGB8888 = 0x773c9ac0, /*!< 32-bit ARGB8888. */

    kVIDEO_PixelFormatRGB565 = 0xe436eb7b, /*!< 16-bit RGB565. */

    /* YUV */
    kVIDEO_PixelFormatYUYV = 0x32595559, /*!< YUV422, Y-U-Y-V. */
    kVIDEO_PixelFormatUYVY = 0x59565955, /*!< YUV422, U-Y-V-Y. */
    kVIDEO_PixelFormatNV12 = 0x3231564E, /*!< YUV422, U-Y-V-Y. */
} video_pixel_format_t;

/*! @brief Resolution definition. */
typedef enum _video_resolution
{
    kVIDEO_ResolutionVGA   = FSL_VIDEO_RESOLUTION(640, 480),   /*!< VGA, 640 * 480 */
    kVIDEO_ResolutionQVGA  = FSL_VIDEO_RESOLUTION(320, 240),   /*!< QVGA, 320 * 240 */
    kVIDEO_ResolutionQQVGA = FSL_VIDEO_RESOLUTION(160, 120),   /*!< QQVGA, 160 * 120 */
    kVIDEO_ResolutionCIF   = FSL_VIDEO_RESOLUTION(352, 288),   /*!< CIF, 352 * 288 */
    kVIDEO_ResolutionQCIF  = FSL_VIDEO_RESOLUTION(176, 144),   /*!< QCIF, 176 * 144 */
    kVIDEO_ResolutionQQCIF = FSL_VIDEO_RESOLUTION(88, 72),     /*!< QQCIF, 88 * 72 */
    kVIDEO_Resolution720P  = FSL_VIDEO_RESOLUTION(1280, 720),  /*!< 720P, 1280 * 720 */
    kVIDEO_Resolution1080P = FSL_VIDEO_RESOLUTION(1920, 1080), /*!< 1080P, 1920 * 1280*/
    kVIDEO_ResolutionWXGA  = FSL_VIDEO_RESOLUTION(1280, 800),  /*!< WXGA, 1280 * 800 */
} video_resolution_t;

#pragma pack( push )
#pragma pack( 4 )

struct FrameInfo_t
{
    PVOID m_RequestCtxPtr;
    PVOID m_Fb; // Should be same as Virtual
    LONG m_Stride;
    ULONG m_ByteCount;
    size_t m_FbSize; // Should be same as ByteCount
    PUCHAR m_Virtual;
    BOOLEAN m_StrideValid;
    FrameInfo_t(PUCHAR VirtualAddress, ULONG ByteCount, LONG Stride, BOOLEAN StrideIsValid) : m_Virtual(VirtualAddress), m_ByteCount(ByteCount), m_Stride(Stride), m_StrideValid(StrideIsValid) { ; }
    FrameInfo_t() : m_Virtual(0), m_ByteCount(0), m_Stride(0), m_StrideValid(0) { ; }
};

struct camera_config_t // : csi2rx_config_t
{
    UINT32 resolution;                  /*!< Resolution, see @ref video_resolution_t and @ref FSL_VIDEO_RESOLUTION. */
    video_pixel_format_t cameraPixelFormat;   /*!< Pixel format. */
    video_pixel_format_t resultPixelFormat;   /*!< Pixel format. */
    UINT8 framePerSec;                 /*!< Frame per second. */
    UINT8 mipiChannel;                 /*!< MIPI virtual channel. */
    UINT8 csiLanes;                    /*!< MIPI CSI data lanes count. */
    UINT32 DisableLaneMask;
    UINT32 mipiLaneClk;

    bool ContClkMode;

    camera_config_t(
        UINT32 Resolution,
        video_pixel_format_t CameraPixelFormat,
        video_pixel_format_t ResultPixelFormat,
        UINT8 FramePerSec,
        UINT8 MipiChannel,
        UINT8 CsiLanes
    ) :
        resolution(Resolution),
        cameraPixelFormat(CameraPixelFormat),
        resultPixelFormat(ResultPixelFormat),
        framePerSec(FramePerSec),
        mipiChannel(MipiChannel),
        csiLanes(CsiLanes)
    {};

    UINT8 GetPixelSizeBits(const video_pixel_format_t &pixelFormat) const
    {
        UINT8 ret;

        switch (pixelFormat) {
        case kVIDEO_PixelFormatRGB888:
            ret = 24;
            break;

        case kVIDEO_PixelFormatRGB565:
        case kVIDEO_PixelFormatYUYV:
        case kVIDEO_PixelFormatUYVY:
            ret = 16;
            break;

        case kVIDEO_PixelFormatNV12:
            ret = 12;
            break;

        default:
            ret = 0;
            break;
        }
        return ret;
    }
};
#pragma pack( pop )
