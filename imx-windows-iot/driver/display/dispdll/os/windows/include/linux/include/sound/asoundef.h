/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __SOUND_ASOUNDEF_H
#define __SOUND_ASOUNDEF_H

/*
 *  Advanced Linux Sound Architecture - ALSA - Driver
 *  Copyright (c) 1994-2000 by Jaroslav Kysela <perex@perex.cz>
 *  Copyright 2023 NXP
 */

/****************************************************************************
 *                                                                          *
 *        Digital audio interface					    *
 *                                                                          *
 ****************************************************************************/

/* AES/IEC958 channel status bits */
#define IEC958_AES0_PROFESSIONAL	(1<<0)	/* 0 = consumer, 1 = professional */
#define IEC958_AES0_NONAUDIO		(1<<1)	/* 0 = audio, 1 = non-audio */
#define IEC958_AES0_PRO_EMPHASIS	(7<<2)	/* mask - emphasis */
#define IEC958_AES0_PRO_EMPHASIS_NOTID	(0<<2)	/* emphasis not indicated */
#define IEC958_AES0_PRO_EMPHASIS_NONE	(1<<2)	/* none emphasis */
#define IEC958_AES0_PRO_EMPHASIS_5015	(3<<2)	/* 50/15us emphasis */
#define IEC958_AES0_PRO_EMPHASIS_CCITT	(7<<2)	/* CCITT J.17 emphasis */
#define IEC958_AES0_PRO_FREQ_UNLOCKED	(1<<5)	/* source sample frequency: 0 = locked, 1 = unlocked */
#define IEC958_AES0_PRO_FS		(3<<6)	/* mask - sample frequency */
#define IEC958_AES0_PRO_FS_NOTID	(0<<6)	/* fs not indicated */
#define IEC958_AES0_PRO_FS_44100	(1<<6)	/* 44.1kHz */
#define IEC958_AES0_PRO_FS_48000	(2<<6)	/* 48kHz */
#define IEC958_AES0_PRO_FS_32000	(3<<6)	/* 32kHz */
#define IEC958_AES0_CON_NOT_COPYRIGHT	(1<<2)	/* 0 = copyright, 1 = not copyright */
#define IEC958_AES0_CON_EMPHASIS	(7<<3)	/* mask - emphasis */
#define IEC958_AES0_CON_EMPHASIS_NONE	(0<<3)	/* none emphasis */
#define IEC958_AES0_CON_EMPHASIS_5015	(1<<3)	/* 50/15us emphasis */
#define IEC958_AES0_CON_MODE		(3<<6)	/* mask - mode */
#define IEC958_AES1_PRO_MODE		(15<<0)	/* mask - channel mode */
#define IEC958_AES1_PRO_MODE_NOTID	(0<<0)	/* not indicated */
#define IEC958_AES1_PRO_MODE_STEREOPHONIC (2<<0) /* stereophonic - ch A is left */
#define IEC958_AES1_PRO_MODE_SINGLE	(4<<0)	/* single channel */
#define IEC958_AES1_PRO_MODE_TWO	(8<<0)	/* two channels */
#define IEC958_AES1_PRO_MODE_PRIMARY	(12<<0)	/* primary/secondary */
#define IEC958_AES1_PRO_MODE_BYTE3	(15<<0)	/* vector to byte 3 */
#define IEC958_AES1_PRO_USERBITS	(15<<4)	/* mask - user bits */
#define IEC958_AES1_PRO_USERBITS_NOTID	(0<<4)	/* not indicated */
#define IEC958_AES1_PRO_USERBITS_192	(8<<4)	/* 192-bit structure */
#define IEC958_AES1_PRO_USERBITS_UDEF	(12<<4)	/* user defined application */
#define IEC958_AES1_CON_CATEGORY	0x7f
#define IEC958_AES1_CON_GENERAL		0x00
#define IEC958_AES1_CON_LASEROPT_MASK	0x07
#define IEC958_AES1_CON_LASEROPT_ID	0x01
#define IEC958_AES1_CON_IEC908_CD	(IEC958_AES1_CON_LASEROPT_ID|0x00)
#define IEC958_AES1_CON_NON_IEC908_CD	(IEC958_AES1_CON_LASEROPT_ID|0x08)
#define IEC958_AES1_CON_MINI_DISC	(IEC958_AES1_CON_LASEROPT_ID|0x48)
#define IEC958_AES1_CON_DVD		(IEC958_AES1_CON_LASEROPT_ID|0x18)
#define IEC958_AES1_CON_LASTEROPT_OTHER	(IEC958_AES1_CON_LASEROPT_ID|0x78)
#define IEC958_AES1_CON_DIGDIGCONV_MASK 0x07
#define IEC958_AES1_CON_DIGDIGCONV_ID	0x02
#define IEC958_AES1_CON_PCM_CODER	(IEC958_AES1_CON_DIGDIGCONV_ID|0x00)
#define IEC958_AES1_CON_MIXER		(IEC958_AES1_CON_DIGDIGCONV_ID|0x10)
#define IEC958_AES1_CON_RATE_CONVERTER	(IEC958_AES1_CON_DIGDIGCONV_ID|0x18)
#define IEC958_AES1_CON_SAMPLER		(IEC958_AES1_CON_DIGDIGCONV_ID|0x20)
#define IEC958_AES1_CON_DSP		(IEC958_AES1_CON_DIGDIGCONV_ID|0x28)
#define IEC958_AES1_CON_DIGDIGCONV_OTHER (IEC958_AES1_CON_DIGDIGCONV_ID|0x78)
#define IEC958_AES1_CON_MAGNETIC_MASK	0x07
#define IEC958_AES1_CON_MAGNETIC_ID	0x03
#define IEC958_AES1_CON_DAT		(IEC958_AES1_CON_MAGNETIC_ID|0x00)
#define IEC958_AES1_CON_VCR		(IEC958_AES1_CON_MAGNETIC_ID|0x08)
#define IEC958_AES1_CON_DCC		(IEC958_AES1_CON_MAGNETIC_ID|0x40)
#define IEC958_AES1_CON_MAGNETIC_DISC	(IEC958_AES1_CON_MAGNETIC_ID|0x18)
#define IEC958_AES1_CON_MAGNETIC_OTHER	(IEC958_AES1_CON_MAGNETIC_ID|0x78)
#define IEC958_AES1_CON_BROADCAST1_MASK 0x07
#define IEC958_AES1_CON_BROADCAST1_ID	0x04
#define IEC958_AES1_CON_DAB_JAPAN	(IEC958_AES1_CON_BROADCAST1_ID|0x00)
#define IEC958_AES1_CON_DAB_EUROPE	(IEC958_AES1_CON_BROADCAST1_ID|0x08)
#define IEC958_AES1_CON_DAB_USA		(IEC958_AES1_CON_BROADCAST1_ID|0x60)
#define IEC958_AES1_CON_SOFTWARE	(IEC958_AES1_CON_BROADCAST1_ID|0x40)
#define IEC958_AES1_CON_IEC62105	(IEC958_AES1_CON_BROADCAST1_ID|0x20)
#define IEC958_AES1_CON_BROADCAST1_OTHER (IEC958_AES1_CON_BROADCAST1_ID|0x78)
#define IEC958_AES1_CON_BROADCAST2_MASK 0x0f
#define IEC958_AES1_CON_BROADCAST2_ID	0x0e
#define IEC958_AES1_CON_MUSICAL_MASK	0x07
#define IEC958_AES1_CON_MUSICAL_ID	0x05
#define IEC958_AES1_CON_SYNTHESIZER	(IEC958_AES1_CON_MUSICAL_ID|0x00)
#define IEC958_AES1_CON_MICROPHONE	(IEC958_AES1_CON_MUSICAL_ID|0x08)
#define IEC958_AES1_CON_MUSICAL_OTHER	(IEC958_AES1_CON_MUSICAL_ID|0x78)
#define IEC958_AES1_CON_ADC_MASK	0x1f
#define IEC958_AES1_CON_ADC_ID		0x06
#define IEC958_AES1_CON_ADC		(IEC958_AES1_CON_ADC_ID|0x00)
#define IEC958_AES1_CON_ADC_OTHER	(IEC958_AES1_CON_ADC_ID|0x60)
#define IEC958_AES1_CON_ADC_COPYRIGHT_MASK 0x1f
#define IEC958_AES1_CON_ADC_COPYRIGHT_ID 0x16
#define IEC958_AES1_CON_ADC_COPYRIGHT	(IEC958_AES1_CON_ADC_COPYRIGHT_ID|0x00)
#define IEC958_AES1_CON_ADC_COPYRIGHT_OTHER (IEC958_AES1_CON_ADC_COPYRIGHT_ID|0x60)
#define IEC958_AES1_CON_SOLIDMEM_MASK	0x0f
#define IEC958_AES1_CON_SOLIDMEM_ID	0x08
#define IEC958_AES1_CON_SOLIDMEM_DIGITAL_RECORDER_PLAYER (IEC958_AES1_CON_SOLIDMEM_ID|0x00)
#define IEC958_AES1_CON_SOLIDMEM_OTHER	(IEC958_AES1_CON_SOLIDMEM_ID|0x70)
#define IEC958_AES1_CON_EXPERIMENTAL	0x40
#define IEC958_AES1_CON_ORIGINAL	(1<<7)	/* this bits depends on the category code */
#define IEC958_AES2_PRO_SBITS		(7<<0)	/* mask - sample bits */
#define IEC958_AES2_PRO_SBITS_20	(2<<0)	/* 20-bit - coordination */
#define IEC958_AES2_PRO_SBITS_24	(4<<0)	/* 24-bit - main audio */
#define IEC958_AES2_PRO_SBITS_UDEF	(6<<0)	/* user defined application */
#define IEC958_AES2_PRO_WORDLEN		(7<<3)	/* mask - source word length */
#define IEC958_AES2_PRO_WORDLEN_NOTID	(0<<3)	/* not indicated */
#define IEC958_AES2_PRO_WORDLEN_22_18	(2<<3)	/* 22-bit or 18-bit */
#define IEC958_AES2_PRO_WORDLEN_23_19	(4<<3)	/* 23-bit or 19-bit */
#define IEC958_AES2_PRO_WORDLEN_24_20	(5<<3)	/* 24-bit or 20-bit */
#define IEC958_AES2_PRO_WORDLEN_20_16	(6<<3)	/* 20-bit or 16-bit */
#define IEC958_AES2_CON_SOURCE		(15<<0)	/* mask - source number */
#define IEC958_AES2_CON_SOURCE_UNSPEC	(0<<0)	/* unspecified */
#define IEC958_AES2_CON_CHANNEL		(15<<4)	/* mask - channel number */
#define IEC958_AES2_CON_CHANNEL_UNSPEC	(0<<4)	/* unspecified */
#define IEC958_AES3_CON_FS		(15<<0)	/* mask - sample frequency */
#define IEC958_AES3_CON_FS_44100	(0<<0)	/* 44.1kHz */
#define IEC958_AES3_CON_FS_NOTID	(1<<0)	/* non indicated */
#define IEC958_AES3_CON_FS_48000	(2<<0)	/* 48kHz */
#define IEC958_AES3_CON_FS_32000	(3<<0)	/* 32kHz */
#define IEC958_AES3_CON_FS_22050	(4<<0)	/* 22.05kHz */
#define IEC958_AES3_CON_FS_24000	(6<<0)	/* 24kHz */
#define IEC958_AES3_CON_FS_88200	(8<<0)	/* 88.2kHz */
#define IEC958_AES3_CON_FS_768000	(9<<0)	/* 768kHz */
#define IEC958_AES3_CON_FS_96000	(10<<0)	/* 96kHz */
#define IEC958_AES3_CON_FS_176400	(12<<0)	/* 176.4kHz */
#define IEC958_AES3_CON_FS_192000	(14<<0)	/* 192kHz */
#define IEC958_AES3_CON_CLOCK		(3<<4)	/* mask - clock accuracy */
#define IEC958_AES3_CON_CLOCK_1000PPM	(0<<4)	/* 1000 ppm */
#define IEC958_AES3_CON_CLOCK_50PPM	(1<<4)	/* 50 ppm */
#define IEC958_AES3_CON_CLOCK_VARIABLE	(2<<4)	/* variable pitch */
#define IEC958_AES4_CON_MAX_WORDLEN_24	(1<<0)	/* 0 = 20-bit, 1 = 24-bit */
#define IEC958_AES4_CON_WORDLEN		(7<<1)	/* mask - sample word length */
#define IEC958_AES4_CON_WORDLEN_NOTID	(0<<1)	/* not indicated */
#define IEC958_AES4_CON_WORDLEN_20_16	(1<<1)	/* 20-bit or 16-bit */
#define IEC958_AES4_CON_WORDLEN_22_18	(2<<1)	/* 22-bit or 18-bit */
#define IEC958_AES4_CON_WORDLEN_23_19	(4<<1)	/* 23-bit or 19-bit */
#define IEC958_AES4_CON_WORDLEN_24_20	(5<<1)	/* 24-bit or 20-bit */
#define IEC958_AES4_CON_WORDLEN_21_17	(6<<1)	/* 21-bit or 17-bit */
#define IEC958_AES4_CON_ORIGFS		(15<<4)	/* mask - original sample frequency */
#define IEC958_AES4_CON_ORIGFS_NOTID	(0<<4)	/* not indicated */
#define IEC958_AES4_CON_ORIGFS_192000	(1<<4)	/* 192kHz */
#define IEC958_AES4_CON_ORIGFS_12000	(2<<4)	/* 12kHz */
#define IEC958_AES4_CON_ORIGFS_176400	(3<<4)	/* 176.4kHz */
#define IEC958_AES4_CON_ORIGFS_96000	(5<<4)	/* 96kHz */
#define IEC958_AES4_CON_ORIGFS_8000	(6<<4)	/* 8kHz */
#define IEC958_AES4_CON_ORIGFS_88200	(7<<4)	/* 88.2kHz */
#define IEC958_AES4_CON_ORIGFS_16000	(8<<4)	/* 16kHz */
#define IEC958_AES4_CON_ORIGFS_24000	(9<<4)	/* 24kHz */
#define IEC958_AES4_CON_ORIGFS_11025	(10<<4)	/* 11.025kHz */
#define IEC958_AES4_CON_ORIGFS_22050	(11<<4)	/* 22.05kHz */
#define IEC958_AES4_CON_ORIGFS_32000	(12<<4)	/* 32kHz */
#define IEC958_AES4_CON_ORIGFS_48000	(13<<4)	/* 48kHz */
#define IEC958_AES4_CON_ORIGFS_44100	(15<<4)	/* 44.1kHz */
#define IEC958_AES5_CON_CGMSA		(3<<0)	/* mask - CGMS-A */
#define IEC958_AES5_CON_CGMSA_COPYFREELY (0<<0)	/* copying is permitted without restriction */
#define IEC958_AES5_CON_CGMSA_COPYONCE	(1<<0)	/* one generation of copies may be made */
#define IEC958_AES5_CON_CGMSA_COPYNOMORE (2<<0)	/* condition not be used */
#define IEC958_AES5_CON_CGMSA_COPYNEVER	(3<<0)	/* no copying is permitted */

/****************************************************************************
 *                                                                          *
 *        CEA-861 Audio InfoFrame. Used in HDMI and DisplayPort		    *
 *                                                                          *
 ****************************************************************************/
#define CEA861_AUDIO_INFOFRAME_DB1CC		(7<<0) /* mask - channel count */
#define CEA861_AUDIO_INFOFRAME_DB1CT		(0xf<<4) /* mask - coding type */
#define CEA861_AUDIO_INFOFRAME_DB1CT_FROM_STREAM (0<<4) /* refer to stream */
#define CEA861_AUDIO_INFOFRAME_DB1CT_IEC60958	(1<<4) /* IEC-60958 L-PCM */
#define CEA861_AUDIO_INFOFRAME_DB1CT_AC3	(2<<4) /* AC-3 */
#define CEA861_AUDIO_INFOFRAME_DB1CT_MPEG1	(3<<4) /* MPEG1 Layers 1 & 2 */
#define CEA861_AUDIO_INFOFRAME_DB1CT_MP3	(4<<4) /* MPEG1 Layer 3 */
#define CEA861_AUDIO_INFOFRAME_DB1CT_MPEG2_MULTICH (5<<4) /* MPEG2 Multichannel */
#define CEA861_AUDIO_INFOFRAME_DB1CT_AAC	(6<<4) /* AAC */
#define CEA861_AUDIO_INFOFRAME_DB1CT_DTS	(7<<4) /* DTS */
#define CEA861_AUDIO_INFOFRAME_DB1CT_ATRAC	(8<<4) /* ATRAC */
#define CEA861_AUDIO_INFOFRAME_DB1CT_ONEBIT	(9<<4) /* One Bit Audio */
#define CEA861_AUDIO_INFOFRAME_DB1CT_DOLBY_DIG_PLUS (10<<4) /* Dolby Digital + */
#define CEA861_AUDIO_INFOFRAME_DB1CT_DTS_HD	(11<<4) /* DTS-HD */
#define CEA861_AUDIO_INFOFRAME_DB1CT_MAT	(12<<4) /* MAT (MLP) */
#define CEA861_AUDIO_INFOFRAME_DB1CT_DST	(13<<4) /* DST */
#define CEA861_AUDIO_INFOFRAME_DB1CT_WMA_PRO	(14<<4) /* WMA Pro */
#define CEA861_AUDIO_INFOFRAME_DB2SF		(7<<2) /* mask - sample frequency */
#define CEA861_AUDIO_INFOFRAME_DB2SF_FROM_STREAM (0<<2) /* refer to stream */
#define CEA861_AUDIO_INFOFRAME_DB2SF_32000	(1<<2) /* 32kHz */
#define CEA861_AUDIO_INFOFRAME_DB2SF_44100	(2<<2) /* 44.1kHz */
#define CEA861_AUDIO_INFOFRAME_DB2SF_48000	(3<<2) /* 48kHz */
#define CEA861_AUDIO_INFOFRAME_DB2SF_88200	(4<<2) /* 88.2kHz */
#define CEA861_AUDIO_INFOFRAME_DB2SF_96000	(5<<2) /* 96kHz */
#define CEA861_AUDIO_INFOFRAME_DB2SF_176400	(6<<2) /* 176.4kHz */
#define CEA861_AUDIO_INFOFRAME_DB2SF_192000	(7<<2) /* 192kHz */
#define CEA861_AUDIO_INFOFRAME_DB2SS		(3<<0) /* mask - sample size */
#define CEA861_AUDIO_INFOFRAME_DB2SS_FROM_STREAM (0<<0) /* refer to stream */
#define CEA861_AUDIO_INFOFRAME_DB2SS_16BIT	(1<<0) /* 16 bits */
#define CEA861_AUDIO_INFOFRAME_DB2SS_20BIT	(2<<0) /* 20 bits */
#define CEA861_AUDIO_INFOFRAME_DB2SS_24BIT	(3<<0) /* 24 bits */
#define CEA861_AUDIO_INFOFRAME_DB5_DM_INH	(1<<7) /* mask - inhibit downmixing */
#define CEA861_AUDIO_INFOFRAME_DB5_DM_INH_PERMITTED (0<<7) /* stereo downmix permitted */
#define CEA861_AUDIO_INFOFRAME_DB5_DM_INH_PROHIBITED (1<<7) /* stereo downmis prohibited */
#define CEA861_AUDIO_INFOFRAME_DB5_LSV		(0xf<<3) /* mask - level-shift values */

#endif /* __SOUND_ASOUNDEF_H */
