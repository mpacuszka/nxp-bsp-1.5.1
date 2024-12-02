/*
* Copyright 2018,2022 NXP
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the disclaimer
* below) provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* * Neither the name of NXP nor the names of its contributors may be used to
* endorse or promote products derived from this software without specific prior
* written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef _ENET_QOS_IOMAP_H
#define _ENET_QOS_IOMAP_H

#define BIT(bitnum)     (1UL << bitnum)
#define GENMASK(high, low)   (((UINT32(1) << ((high)-(low)+1)) - 1) << (low))

/**< IMX_ENET_QOS_MAC_CONFIGURATION - MAC Configuration Register */
typedef union IMX_ENET_QOS_MAC_CONFIGURATION_union_t {
    UINT32 R;
    struct {
        UINT32 RE : 1;                 /**< Receiver Enable */
        UINT32 TE : 1;                 /**< Transmitter Enable */
        UINT32 PRELEN : 2;                 /**< Preamble Length for Transmit packets */
        UINT32 DC : 1;                 /**< Deferral Check */
        UINT32 BL : 2;                 /**< Back-Off Limit */
        UINT32 _unused7 : 1;                 /**< Reserved. */
        UINT32 DR : 1;                 /**< Disable Retry */
        UINT32 DCRS : 1;                 /**< Disable Carrier Sense During Transmission */
        UINT32 DO : 1;                 /**< Disable Receive Own */
        UINT32 ECRSFD : 1;                 /**< Enable Carrier Sense Before Transmission in Full-Duplex Mode */
        UINT32 LM : 1;                 /**< Loopback Mode */
        UINT32 DM : 1;                 /**< Duplex Mode */
        UINT32 FES : 1;                 /**< Speed */
        UINT32 PS : 1;                 /**< Port Select */
        UINT32 JE : 1;                 /**< Jumbo Packet Enable When this bit is set, the MAC allows jumbo packets of 9,018 bytes (9,022 bytes for VLAN tagged packets) without reporting a giant packet error in the Rx packet status. */
        UINT32 JD : 1;                 /**< Jabber Disable */
        UINT32 BE : 1;                 /**< Packet Burst Enable When this bit is set, the MAC allows packet bursting during transmission in the GMII half-duplex mode. */
        UINT32 WD : 1;                 /**< Watchdog Disable */
        UINT32 ACS : 1;                 /**< Automatic Pad or CRC Stripping When this bit is set, the MAC strips the Pad or FCS field on the incoming packets only if the value of the length field is less than 1,536 bytes. */
        UINT32 CST : 1;                 /**< CRC stripping for Type packets When this bit is set, the last four bytes (FCS) of all packets of Ether type (type field greater than 1,536) are stripped and dropped before forwarding the packet to the application. */
        UINT32 S2KP : 1;                 /**< IEEE 802. */
        UINT32 GPSLCE : 1;                 /**< Giant Packet Size Limit Control Enable */
        UINT32 IPG : 3;                 /**< Inter-Packet Gap These bits control the minimum IPG between packets during transmission. */
        UINT32 IPC : 1;                 /**< Checksum Offload */
        UINT32 SARC : 3;                 /**< Source Address Insertion or Replacement Control */
        UINT32 _unused31 : 1;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_CONFIGURATION_t;

/**< Register MAC_CONFIGURATION - MAC Configuration Register */
#define IMX_ENET_QOS_MAC_CONFIGURATION           0
/**< Field RE - Receiver Enable */
#define IMX_ENET_QOS_MAC_CONFIGURATION_RE_SHIFT  0
#define IMX_ENET_QOS_MAC_CONFIGURATION_RE_MASK   0x1
#define IMX_ENET_QOS_MAC_CONFIGURATION_RE(v)     (((v) << 0) & 0x1)
/**< Field TE - Transmitter Enable */
#define IMX_ENET_QOS_MAC_CONFIGURATION_TE_SHIFT  1
#define IMX_ENET_QOS_MAC_CONFIGURATION_TE_MASK   0x2
#define IMX_ENET_QOS_MAC_CONFIGURATION_TE(v)     (((v) << 1) & 0x2)
/**< Field PRELEN - Preamble Length for Transmit packets */
#define IMX_ENET_QOS_MAC_CONFIGURATION_PRELEN_SHIFT 2
#define IMX_ENET_QOS_MAC_CONFIGURATION_PRELEN_MASK 0xC
#define IMX_ENET_QOS_MAC_CONFIGURATION_PRELEN(v) (((v) << 2) & 0xC)
/**< Field DC - Deferral Check */
#define IMX_ENET_QOS_MAC_CONFIGURATION_DC_SHIFT  4
#define IMX_ENET_QOS_MAC_CONFIGURATION_DC_MASK   0x10
#define IMX_ENET_QOS_MAC_CONFIGURATION_DC(v)     (((v) << 4) & 0x10)
/**< Field BL - Back-Off Limit */
#define IMX_ENET_QOS_MAC_CONFIGURATION_BL_SHIFT  5
#define IMX_ENET_QOS_MAC_CONFIGURATION_BL_MASK   0x60
#define IMX_ENET_QOS_MAC_CONFIGURATION_BL(v)     (((v) << 5) & 0x60)
/**< Field DR - Disable Retry */
#define IMX_ENET_QOS_MAC_CONFIGURATION_DR_SHIFT  8
#define IMX_ENET_QOS_MAC_CONFIGURATION_DR_MASK   0x100
#define IMX_ENET_QOS_MAC_CONFIGURATION_DR(v)     (((v) << 8) & 0x100)
/**< Field DCRS - Disable Carrier Sense During Transmission */
#define IMX_ENET_QOS_MAC_CONFIGURATION_DCRS_SHIFT 9
#define IMX_ENET_QOS_MAC_CONFIGURATION_DCRS_MASK 0x200
#define IMX_ENET_QOS_MAC_CONFIGURATION_DCRS(v)   (((v) << 9) & 0x200)
/**< Field DO - Disable Receive Own */
#define IMX_ENET_QOS_MAC_CONFIGURATION_DO_SHIFT  10
#define IMX_ENET_QOS_MAC_CONFIGURATION_DO_MASK   0x400
#define IMX_ENET_QOS_MAC_CONFIGURATION_DO(v)     (((v) << 10) & 0x400)
/**< Field ECRSFD - Enable Carrier Sense Before Transmission in Full-Duplex Mode */
#define IMX_ENET_QOS_MAC_CONFIGURATION_ECRSFD_SHIFT 11
#define IMX_ENET_QOS_MAC_CONFIGURATION_ECRSFD_MASK 0x800
#define IMX_ENET_QOS_MAC_CONFIGURATION_ECRSFD(v) (((v) << 11) & 0x800)
/**< Field LM - Loopback Mode */
#define IMX_ENET_QOS_MAC_CONFIGURATION_LM_SHIFT  12
#define IMX_ENET_QOS_MAC_CONFIGURATION_LM_MASK   0x1000
#define IMX_ENET_QOS_MAC_CONFIGURATION_LM(v)     (((v) << 12) & 0x1000)
/**< Field DM - Duplex Mode */
#define IMX_ENET_QOS_MAC_CONFIGURATION_DM_SHIFT  13
#define IMX_ENET_QOS_MAC_CONFIGURATION_DM_MASK   0x2000
#define IMX_ENET_QOS_MAC_CONFIGURATION_DM(v)     (((v) << 13) & 0x2000)
/**< Field FES - Speed */
#define IMX_ENET_QOS_MAC_CONFIGURATION_FES_SHIFT 14
#define IMX_ENET_QOS_MAC_CONFIGURATION_FES_MASK  0x4000
#define IMX_ENET_QOS_MAC_CONFIGURATION_FES(v)    (((v) << 14) & 0x4000)
/**< Field PS - Port Select */
#define IMX_ENET_QOS_MAC_CONFIGURATION_PS_SHIFT  15
#define IMX_ENET_QOS_MAC_CONFIGURATION_PS_MASK   0x8000
#define IMX_ENET_QOS_MAC_CONFIGURATION_PS(v)     (((v) << 15) & 0x8000)
/**< Field JE - Jumbo Packet Enable When this bit is set, the MAC allows jumbo packets of 9,018 bytes (9,022 bytes for VLAN tagged packets) without reporting a giant packet error in the Rx packet status. */
#define IMX_ENET_QOS_MAC_CONFIGURATION_JE_SHIFT  16
#define IMX_ENET_QOS_MAC_CONFIGURATION_JE_MASK   0x10000
#define IMX_ENET_QOS_MAC_CONFIGURATION_JE(v)     (((v) << 16) & 0x10000)
/**< Field JD - Jabber Disable */
#define IMX_ENET_QOS_MAC_CONFIGURATION_JD_SHIFT  17
#define IMX_ENET_QOS_MAC_CONFIGURATION_JD_MASK   0x20000
#define IMX_ENET_QOS_MAC_CONFIGURATION_JD(v)     (((v) << 17) & 0x20000)
/**< Field BE - Packet Burst Enable When this bit is set, the MAC allows packet bursting during transmission in the GMII half-duplex mode. */
#define IMX_ENET_QOS_MAC_CONFIGURATION_BE_SHIFT  18
#define IMX_ENET_QOS_MAC_CONFIGURATION_BE_MASK   0x40000
#define IMX_ENET_QOS_MAC_CONFIGURATION_BE(v)     (((v) << 18) & 0x40000)
/**< Field WD - Watchdog Disable */
#define IMX_ENET_QOS_MAC_CONFIGURATION_WD_SHIFT  19
#define IMX_ENET_QOS_MAC_CONFIGURATION_WD_MASK   0x80000
#define IMX_ENET_QOS_MAC_CONFIGURATION_WD(v)     (((v) << 19) & 0x80000)
/**< Field ACS - Automatic Pad or CRC Stripping When this bit is set, the MAC strips the Pad or FCS field on the incoming packets only if the value of the length field is less than 1,536 bytes. */
#define IMX_ENET_QOS_MAC_CONFIGURATION_ACS_SHIFT 20
#define IMX_ENET_QOS_MAC_CONFIGURATION_ACS_MASK  0x100000
#define IMX_ENET_QOS_MAC_CONFIGURATION_ACS(v)    (((v) << 20) & 0x100000)
/**< Field CST - CRC stripping for Type packets When this bit is set, the last four bytes (FCS) of all packets of Ether type (type field greater than 1,536) are stripped and dropped before forwarding the packet to the application. */
#define IMX_ENET_QOS_MAC_CONFIGURATION_CST_SHIFT 21
#define IMX_ENET_QOS_MAC_CONFIGURATION_CST_MASK  0x200000
#define IMX_ENET_QOS_MAC_CONFIGURATION_CST(v)    (((v) << 21) & 0x200000)
/**< Field S2KP - IEEE 802. */
#define IMX_ENET_QOS_MAC_CONFIGURATION_S2KP_SHIFT 22
#define IMX_ENET_QOS_MAC_CONFIGURATION_S2KP_MASK 0x400000
#define IMX_ENET_QOS_MAC_CONFIGURATION_S2KP(v)   (((v) << 22) & 0x400000)
/**< Field GPSLCE - Giant Packet Size Limit Control Enable */
#define IMX_ENET_QOS_MAC_CONFIGURATION_GPSLCE_SHIFT 23
#define IMX_ENET_QOS_MAC_CONFIGURATION_GPSLCE_MASK 0x800000
#define IMX_ENET_QOS_MAC_CONFIGURATION_GPSLCE(v) (((v) << 23) & 0x800000)
/**< Field IPG - Inter-Packet Gap These bits control the minimum IPG between packets during transmission. */
#define IMX_ENET_QOS_MAC_CONFIGURATION_IPG_SHIFT 24
#define IMX_ENET_QOS_MAC_CONFIGURATION_IPG_MASK  0x7000000
#define IMX_ENET_QOS_MAC_CONFIGURATION_IPG(v)    (((v) << 24) & 0x7000000)
/**< Field IPC - Checksum Offload */
#define IMX_ENET_QOS_MAC_CONFIGURATION_IPC_SHIFT 27
#define IMX_ENET_QOS_MAC_CONFIGURATION_IPC_MASK  0x8000000
#define IMX_ENET_QOS_MAC_CONFIGURATION_IPC(v)    (((v) << 27) & 0x8000000)
/**< Field SARC - Source Address Insertion or Replacement Control */
#define IMX_ENET_QOS_MAC_CONFIGURATION_SARC_SHIFT 28
#define IMX_ENET_QOS_MAC_CONFIGURATION_SARC_MASK 0x70000000
#define IMX_ENET_QOS_MAC_CONFIGURATION_SARC(v)   (((v) << 28) & 0x70000000)

/**< IMX_ENET_QOS_MAC_EXT_CONFIGURATION - MAC Extended Configuration Register */
typedef union IMX_ENET_QOS_MAC_EXT_CONFIGURATION_union_t {
    UINT32 R;
    struct {
        UINT32 GPSL : 14;                /**< Giant Packet Size Limit */
        UINT32 _unused14 : 2;                 /**< Reserved. */
        UINT32 DCRCC : 1;                 /**< Disable CRC Checking for Received Packets */
        UINT32 SPEN : 1;                 /**< Slow Protocol Detection Enable */
        UINT32 USP : 1;                 /**< Unicast Slow Protocol Packet Detect */
        UINT32 PDC : 1;                 /**< Packet Duplication Control */
        UINT32 _unused20 : 3;                 /**< Reserved. */
        UINT32 _unused23 : 1;                 /**< Reserved. */
        UINT32 EIPGEN : 1;                 /**< Extended Inter-Packet Gap Enable */
        UINT32 EIPG : 5;                 /**< Extended Inter-Packet Gap */
        UINT32 _unused30 : 1;                 /**< Reserved. */
        UINT32 _unused31 : 1;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_EXT_CONFIGURATION_t;

/**< Register MAC_EXT_CONFIGURATION - MAC Extended Configuration Register */
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION       0x4
/**< Field GPSL - Giant Packet Size Limit */
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_GPSL_SHIFT 0
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_GPSL_MASK 0x3FFF
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_GPSL(v) (((v) << 0) & 0x3FFF)
/**< Field DCRCC - Disable CRC Checking for Received Packets */
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_DCRCC_SHIFT 16
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_DCRCC_MASK 0x10000
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_DCRCC(v) (((v) << 16) & 0x10000)
/**< Field SPEN - Slow Protocol Detection Enable */
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_SPEN_SHIFT 17
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_SPEN_MASK 0x20000
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_SPEN(v) (((v) << 17) & 0x20000)
/**< Field USP - Unicast Slow Protocol Packet Detect */
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_USP_SHIFT 18
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_USP_MASK 0x40000
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_USP(v) (((v) << 18) & 0x40000)
/**< Field PDC - Packet Duplication Control */
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_PDC_SHIFT 19
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_PDC_MASK 0x80000
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_PDC(v) (((v) << 19) & 0x80000)
/**< Field EIPGEN - Extended Inter-Packet Gap Enable */
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_EIPGEN_SHIFT 24
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_EIPGEN_MASK 0x1000000
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_EIPGEN(v) (((v) << 24) & 0x1000000)
/**< Field EIPG - Extended Inter-Packet Gap */
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_EIPG_SHIFT 25
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_EIPG_MASK 0x3E000000
#define IMX_ENET_QOS_MAC_EXT_CONFIGURATION_EIPG(v) (((v) << 25) & 0x3E000000)

/**< IMX_ENET_QOS_MAC_PACKET_FILTER - MAC Packet Filter */
typedef union IMX_ENET_QOS_MAC_PACKET_FILTER_union_t {
    UINT32 R;
    struct {
        UINT32 PR : 1;                 /**< Promiscuous Mode */
        UINT32 HUC : 1;                 /**< Hash Unicast */
        UINT32 HMC : 1;                 /**< Hash Multicast */
        UINT32 DAIF : 1;                 /**< DA Inverse Filtering */
        UINT32 PM : 1;                 /**< Pass All Multicast */
        UINT32 DBF : 1;                 /**< Disable Broadcast Packets */
        UINT32 PCF : 2;                 /**< Pass Control Packets These bits control the forwarding of all control packets (including unicast and multicast Pause packets). */
        UINT32 SAIF : 1;                 /**< SA Inverse Filtering */
        UINT32 SAF : 1;                 /**< Source Address Filter Enable */
        UINT32 HPF : 1;                 /**< Hash or Perfect Filter */
        UINT32 _unused11 : 5;                 /**< Reserved. */
        UINT32 VTFE : 1;                 /**< VLAN Tag Filter Enable */
        UINT32 _unused17 : 3;                 /**< Reserved. */
        UINT32 IPFE : 1;                 /**< Layer 3 and Layer 4 Filter Enable */
        UINT32 DNTU : 1;                 /**< Drop Non-TCP/UDP over IP Packets */
        UINT32 _unused22 : 9;                 /**< Reserved. */
        UINT32 RA : 1;                 /**< Receive All */
    } B;
} IMX_ENET_QOS_MAC_PACKET_FILTER_t;

/**< Register MAC_PACKET_FILTER - MAC Packet Filter */
#define IMX_ENET_QOS_MAC_PACKET_FILTER           0x8
/**< Field PR - Promiscuous Mode */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_PR_SHIFT  0
#define IMX_ENET_QOS_MAC_PACKET_FILTER_PR_MASK   0x1
#define IMX_ENET_QOS_MAC_PACKET_FILTER_PR(v)     (((v) << 0) & 0x1)
/**< Field HUC - Hash Unicast */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_HUC_SHIFT 1
#define IMX_ENET_QOS_MAC_PACKET_FILTER_HUC_MASK  0x2
#define IMX_ENET_QOS_MAC_PACKET_FILTER_HUC(v)    (((v) << 1) & 0x2)
/**< Field HMC - Hash Multicast */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_HMC_SHIFT 2
#define IMX_ENET_QOS_MAC_PACKET_FILTER_HMC_MASK  0x4
#define IMX_ENET_QOS_MAC_PACKET_FILTER_HMC(v)    (((v) << 2) & 0x4)
/**< Field DAIF - DA Inverse Filtering */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_DAIF_SHIFT 3
#define IMX_ENET_QOS_MAC_PACKET_FILTER_DAIF_MASK 0x8
#define IMX_ENET_QOS_MAC_PACKET_FILTER_DAIF(v)   (((v) << 3) & 0x8)
/**< Field PM - Pass All Multicast */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_PM_SHIFT  4
#define IMX_ENET_QOS_MAC_PACKET_FILTER_PM_MASK   0x10
#define IMX_ENET_QOS_MAC_PACKET_FILTER_PM(v)     (((v) << 4) & 0x10)
/**< Field DBF - Disable Broadcast Packets */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_DBF_SHIFT 5
#define IMX_ENET_QOS_MAC_PACKET_FILTER_DBF_MASK  0x20
#define IMX_ENET_QOS_MAC_PACKET_FILTER_DBF(v)    (((v) << 5) & 0x20)
/**< Field PCF - Pass Control Packets These bits control the forwarding of all control packets (including unicast and multicast Pause packets). */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_PCF_SHIFT 6
#define IMX_ENET_QOS_MAC_PACKET_FILTER_PCF_MASK  0xC0
#define IMX_ENET_QOS_MAC_PACKET_FILTER_PCF(v)    (((v) << 6) & 0xC0)
/**< Field SAIF - SA Inverse Filtering */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_SAIF_SHIFT 8
#define IMX_ENET_QOS_MAC_PACKET_FILTER_SAIF_MASK 0x100
#define IMX_ENET_QOS_MAC_PACKET_FILTER_SAIF(v)   (((v) << 8) & 0x100)
/**< Field SAF - Source Address Filter Enable */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_SAF_SHIFT 9
#define IMX_ENET_QOS_MAC_PACKET_FILTER_SAF_MASK  0x200
#define IMX_ENET_QOS_MAC_PACKET_FILTER_SAF(v)    (((v) << 9) & 0x200)
/**< Field HPF - Hash or Perfect Filter */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_HPF_SHIFT 10
#define IMX_ENET_QOS_MAC_PACKET_FILTER_HPF_MASK  0x400
#define IMX_ENET_QOS_MAC_PACKET_FILTER_HPF(v)    (((v) << 10) & 0x400)
/**< Field VTFE - VLAN Tag Filter Enable */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_VTFE_SHIFT 16
#define IMX_ENET_QOS_MAC_PACKET_FILTER_VTFE_MASK 0x10000
#define IMX_ENET_QOS_MAC_PACKET_FILTER_VTFE(v)   (((v) << 16) & 0x10000)
/**< Field IPFE - Layer 3 and Layer 4 Filter Enable */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_IPFE_SHIFT 20
#define IMX_ENET_QOS_MAC_PACKET_FILTER_IPFE_MASK 0x100000
#define IMX_ENET_QOS_MAC_PACKET_FILTER_IPFE(v)   (((v) << 20) & 0x100000)
/**< Field DNTU - Drop Non-TCP/UDP over IP Packets */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_DNTU_SHIFT 21
#define IMX_ENET_QOS_MAC_PACKET_FILTER_DNTU_MASK 0x200000
#define IMX_ENET_QOS_MAC_PACKET_FILTER_DNTU(v)   (((v) << 21) & 0x200000)
/**< Field RA - Receive All */
#define IMX_ENET_QOS_MAC_PACKET_FILTER_RA_SHIFT  31
#define IMX_ENET_QOS_MAC_PACKET_FILTER_RA_MASK   0x80000000
#define IMX_ENET_QOS_MAC_PACKET_FILTER_RA(v)     (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT - Watchdog Timeout */
typedef union IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT_union_t {
    UINT32 R;
    struct {
        UINT32 WTO : 4;                 /**< Watchdog Timeout */
        UINT32 _unused4 : 4;                 /**< Reserved. */
        UINT32 PWE : 1;                 /**< Programmable Watchdog Enable */
        UINT32 _unused9 : 23;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT_t;

/**< Register MAC_WATCHDOG_TIMEOUT - Watchdog Timeout */
#define IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT        0xC
/**< Field WTO - Watchdog Timeout */
#define IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT_WTO_SHIFT 0
#define IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT_WTO_MASK 0xF
#define IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT_WTO(v) (((v) << 0) & 0xF)
/**< Field PWE - Programmable Watchdog Enable */
#define IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT_PWE_SHIFT 8
#define IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT_PWE_MASK 0x100
#define IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT_PWE(v) (((v) << 8) & 0x100)

/**< IMX_ENET_QOS_MAC_HASH_TABLE_REG0 - MAC Hash Table Register 0 */
typedef union IMX_ENET_QOS_MAC_HASH_TABLE_REG0_union_t {
    UINT32 R;
    struct {
        UINT32 HT31T0 : 32;                /**< MAC Hash Table First 32 Bits This field contains the first 32 Bits [31:0] of the Hash table. */
    } B;
} IMX_ENET_QOS_MAC_HASH_TABLE_REG0_t;

/**< Register MAC_HASH_TABLE_REG0 - MAC Hash Table Register 0 */
#define IMX_ENET_QOS_MAC_HASH_TABLE_REG0         0x10
/**< Field HT31T0 - MAC Hash Table First 32 Bits This field contains the first 32 Bits [31:0] of the Hash table. */
#define IMX_ENET_QOS_MAC_HASH_TABLE_REG0_HT31T0_SHIFT 0
#define IMX_ENET_QOS_MAC_HASH_TABLE_REG0_HT31T0_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_HASH_TABLE_REG0_HT31T0(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_HASH_TABLE_REG1 - MAC Hash Table Register 1 */
typedef union IMX_ENET_QOS_MAC_HASH_TABLE_REG1_union_t {
    UINT32 R;
    struct {
        UINT32 HT63T32 : 32;                /**< MAC Hash Table Second 32 Bits This field contains the second 32 Bits [63:32] of the Hash table. */
    } B;
} IMX_ENET_QOS_MAC_HASH_TABLE_REG1_t;

/**< Register MAC_HASH_TABLE_REG1 - MAC Hash Table Register 1 */
#define IMX_ENET_QOS_MAC_HASH_TABLE_REG1         0x14
/**< Field HT63T32 - MAC Hash Table Second 32 Bits This field contains the second 32 Bits [63:32] of the Hash table. */
#define IMX_ENET_QOS_MAC_HASH_TABLE_REG1_HT63T32_SHIFT 0
#define IMX_ENET_QOS_MAC_HASH_TABLE_REG1_HT63T32_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_HASH_TABLE_REG1_HT63T32(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_VLAN_TAG_CTRL - MAC VLAN Tag Control */
typedef union IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 OB : 1;                 /**< Operation Busy */
        UINT32 CT : 1;                 /**< Command Type */
        UINT32 OFS : 5;                 /**< Offset */
        UINT32 _unused7 : 10;                /**< Reserved. */
        UINT32 VTIM : 1;                 /**< VLAN Tag Inverse Match Enable */
        UINT32 ESVL : 1;                 /**< Enable S-VLAN When this bit is set, the MAC transmitter and receiver consider the S-VLAN packets (Type = 0x88A8) as valid VLAN tagged packets. */
        UINT32 _unused19 : 2;                 /**< Reserved. */
        UINT32 EVLS : 2;                 /**< Enable VLAN Tag Stripping on Receive This field indicates the stripping operation on the outer VLAN Tag in received packet. */
        UINT32 _unused23 : 1;                 /**< Reserved. */
        UINT32 EVLRXS : 1;                 /**< Enable VLAN Tag in Rx status */
        UINT32 VTHM : 1;                 /**< VLAN Tag Hash Table Match Enable */
        UINT32 EDVLP : 1;                 /**< Enable Double VLAN Processing */
        UINT32 ERIVLT : 1;                 /**< ERIVLT */
        UINT32 EIVLS : 2;                 /**< Enable Inner VLAN Tag Stripping on Receive This field indicates the stripping operation on inner VLAN Tag in received packet. */
        UINT32 _unused30 : 1;                 /**< Reserved. */
        UINT32 EIVLRXS : 1;                 /**< Enable Inner VLAN Tag in Rx Status */
    } B;
} IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_t;

/**< Register MAC_VLAN_TAG_CTRL - MAC VLAN Tag Control */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL           0x50
/**< Field OB - Operation Busy */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_OB_SHIFT  0
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_OB_MASK   0x1
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_OB(v)     (((v) << 0) & 0x1)
/**< Field CT - Command Type */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_CT_SHIFT  1
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_CT_MASK   0x2
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_CT(v)     (((v) << 1) & 0x2)
/**< Field OFS - Offset */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_OFS_SHIFT 2
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_OFS_MASK  0x7C
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_OFS(v)    (((v) << 2) & 0x7C)
/**< Field VTIM - VLAN Tag Inverse Match Enable */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_VTIM_SHIFT 17
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_VTIM_MASK 0x20000
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_VTIM(v)   (((v) << 17) & 0x20000)
/**< Field ESVL - Enable S-VLAN When this bit is set, the MAC transmitter and receiver consider the S-VLAN packets (Type = 0x88A8) as valid VLAN tagged packets. */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_ESVL_SHIFT 18
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_ESVL_MASK 0x40000
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_ESVL(v)   (((v) << 18) & 0x40000)
/**< Field EVLS - Enable VLAN Tag Stripping on Receive This field indicates the stripping operation on the outer VLAN Tag in received packet. */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EVLS_SHIFT 21
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EVLS_MASK 0x600000
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EVLS(v)   (((v) << 21) & 0x600000)
/**< Field EVLRXS - Enable VLAN Tag in Rx status */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EVLRXS_SHIFT 24
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EVLRXS_MASK 0x1000000
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EVLRXS(v) (((v) << 24) & 0x1000000)
/**< Field VTHM - VLAN Tag Hash Table Match Enable */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_VTHM_SHIFT 25
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_VTHM_MASK 0x2000000
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_VTHM(v)   (((v) << 25) & 0x2000000)
/**< Field EDVLP - Enable Double VLAN Processing */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EDVLP_SHIFT 26
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EDVLP_MASK 0x4000000
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EDVLP(v)  (((v) << 26) & 0x4000000)
/**< Field ERIVLT - ERIVLT */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_ERIVLT_SHIFT 27
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_ERIVLT_MASK 0x8000000
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_ERIVLT(v) (((v) << 27) & 0x8000000)
/**< Field EIVLS - Enable Inner VLAN Tag Stripping on Receive This field indicates the stripping operation on inner VLAN Tag in received packet. */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EIVLS_SHIFT 28
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EIVLS_MASK 0x30000000
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EIVLS(v)  (((v) << 28) & 0x30000000)
/**< Field EIVLRXS - Enable Inner VLAN Tag in Rx Status */
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EIVLRXS_SHIFT 31
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EIVLRXS_MASK 0x80000000
#define IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_EIVLRXS(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_VLAN_TAG_DATA - MAC VLAN Tag Data */
typedef union IMX_ENET_QOS_MAC_VLAN_TAG_DATA_union_t {
    UINT32 R;
    struct {
        UINT32 VID : 16;                /**< VLAN Tag ID */
        UINT32 VEN : 1;                 /**< VLAN Tag Enable */
        UINT32 ETV : 1;                 /**< 12bits or 16bits VLAN comparison */
        UINT32 DOVLTC : 1;                 /**< Disable VLAN Type Comparison */
        UINT32 ERSVLM : 1;                 /**< Enable S-VLAN Match for received Frames */
        UINT32 ERIVLT : 1;                 /**< Enable Inner VLAN Tag Comparison */
        UINT32 _unused21 : 3;                 /**< Reserved. */
        UINT32 DMACHEN : 1;                 /**< DMA Channel Number Enable */
        UINT32 DMACHN : 3;                 /**< DMA Channel Number */
        UINT32 _unused28 : 4;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_VLAN_TAG_DATA_t;

/**< Register MAC_VLAN_TAG_DATA - MAC VLAN Tag Data */
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA           0x54
/**< Field VID - VLAN Tag ID */
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_VID_SHIFT 0
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_VID_MASK  0xFFFF
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_VID(v)    (((v) << 0) & 0xFFFF)
/**< Field VEN - VLAN Tag Enable */
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_VEN_SHIFT 16
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_VEN_MASK  0x10000
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_VEN(v)    (((v) << 16) & 0x10000)
/**< Field ETV - 12bits or 16bits VLAN comparison */
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_ETV_SHIFT 17
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_ETV_MASK  0x20000
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_ETV(v)    (((v) << 17) & 0x20000)
/**< Field DOVLTC - Disable VLAN Type Comparison */
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_DOVLTC_SHIFT 18
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_DOVLTC_MASK 0x40000
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_DOVLTC(v) (((v) << 18) & 0x40000)
/**< Field ERSVLM - Enable S-VLAN Match for received Frames */
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_ERSVLM_SHIFT 19
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_ERSVLM_MASK 0x80000
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_ERSVLM(v) (((v) << 19) & 0x80000)
/**< Field ERIVLT - Enable Inner VLAN Tag Comparison */
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_ERIVLT_SHIFT 20
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_ERIVLT_MASK 0x100000
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_ERIVLT(v) (((v) << 20) & 0x100000)
/**< Field DMACHEN - DMA Channel Number Enable */
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_DMACHEN_SHIFT 24
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_DMACHEN_MASK 0x1000000
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_DMACHEN(v) (((v) << 24) & 0x1000000)
/**< Field DMACHN - DMA Channel Number */
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_DMACHN_SHIFT 25
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_DMACHN_MASK 0xE000000
#define IMX_ENET_QOS_MAC_VLAN_TAG_DATA_DMACHN(v) (((v) << 25) & 0xE000000)

/**< IMX_ENET_QOS_MAC_VLAN_HASH_TABLE - MAC VLAN Hash Table */
typedef union IMX_ENET_QOS_MAC_VLAN_HASH_TABLE_union_t {
    UINT32 R;
    struct {
        UINT32 VLHT : 16;                /**< VLAN Hash Table This field contains the 16-bit VLAN Hash Table. */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_VLAN_HASH_TABLE_t;

/**< Register MAC_VLAN_HASH_TABLE - MAC VLAN Hash Table */
#define IMX_ENET_QOS_MAC_VLAN_HASH_TABLE         0x58
/**< Field VLHT - VLAN Hash Table This field contains the 16-bit VLAN Hash Table. */
#define IMX_ENET_QOS_MAC_VLAN_HASH_TABLE_VLHT_SHIFT 0
#define IMX_ENET_QOS_MAC_VLAN_HASH_TABLE_VLHT_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_VLAN_HASH_TABLE_VLHT(v) (((v) << 0) & 0xFFFF)

/**< IMX_ENET_QOS_MAC_VLAN_INCL - VLAN Tag Inclusion or Replacement */
typedef union IMX_ENET_QOS_MAC_VLAN_INCL_union_t {
    UINT32 R;
    struct {
        UINT32 VLT : 16;                /**< VLAN Tag for Transmit Packets */
        UINT32 VLC : 2;                 /**< VLAN Tag Control in Transmit Packets - 2'b00: No VLAN tag deletion, insertion, or replacement - 2'b01: VLAN tag deletion The MAC removes the VLAN type (bytes 13 and 14) and VLAN tag (bytes 15 and 16) of all transmitted packets with VLAN tags. */
        UINT32 VLP : 1;                 /**< VLAN Priority Control */
        UINT32 CSVL : 1;                 /**< C-VLAN or S-VLAN */
        UINT32 VLTI : 1;                 /**< VLAN Tag Input When this bit is set, it indicates that the VLAN tag to be inserted or replaced in Tx packet should be taken from: - The Tx descriptor */
        UINT32 CBTI : 1;                 /**< Channel based tag insertion */
        UINT32 _unused22 : 2;                 /**< Reserved. */
        UINT32 ADDR : 3;                 /**< Address */
        UINT32 _unused27 : 3;                 /**< Reserved. */
        UINT32 RDWR : 1;                 /**< Read write control */
        UINT32 BUSY : 1;                 /**< Busy */
    } B;
} IMX_ENET_QOS_MAC_VLAN_INCL_t;

/**< Register MAC_VLAN_INCL - VLAN Tag Inclusion or Replacement */
#define IMX_ENET_QOS_MAC_VLAN_INCL               0x60
/**< Field VLT - VLAN Tag for Transmit Packets */
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLT_SHIFT     0
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLT_MASK      0xFFFF
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLT(v)        (((v) << 0) & 0xFFFF)
/**< Field VLC - VLAN Tag Control in Transmit Packets - 2'b00: No VLAN tag deletion, insertion, or replacement - 2'b01: VLAN tag deletion The MAC removes the VLAN type (bytes 13 and 14) and VLAN tag (bytes 15 and 16) of all transmitted packets with VLAN tags. */
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLC_SHIFT     16
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLC_MASK      0x30000
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLC(v)        (((v) << 16) & 0x30000)
/**< Field VLP - VLAN Priority Control */
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLP_SHIFT     18
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLP_MASK      0x40000
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLP(v)        (((v) << 18) & 0x40000)
/**< Field CSVL - C-VLAN or S-VLAN */
#define IMX_ENET_QOS_MAC_VLAN_INCL_CSVL_SHIFT    19
#define IMX_ENET_QOS_MAC_VLAN_INCL_CSVL_MASK     0x80000
#define IMX_ENET_QOS_MAC_VLAN_INCL_CSVL(v)       (((v) << 19) & 0x80000)
/**< Field VLTI - VLAN Tag Input When this bit is set, it indicates that the VLAN tag to be inserted or replaced in Tx packet should be taken from: - The Tx descriptor */
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLTI_SHIFT    20
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLTI_MASK     0x100000
#define IMX_ENET_QOS_MAC_VLAN_INCL_VLTI(v)       (((v) << 20) & 0x100000)
/**< Field CBTI - Channel based tag insertion */
#define IMX_ENET_QOS_MAC_VLAN_INCL_CBTI_SHIFT    21
#define IMX_ENET_QOS_MAC_VLAN_INCL_CBTI_MASK     0x200000
#define IMX_ENET_QOS_MAC_VLAN_INCL_CBTI(v)       (((v) << 21) & 0x200000)
/**< Field ADDR - Address */
#define IMX_ENET_QOS_MAC_VLAN_INCL_ADDR_SHIFT    24
#define IMX_ENET_QOS_MAC_VLAN_INCL_ADDR_MASK     0x7000000
#define IMX_ENET_QOS_MAC_VLAN_INCL_ADDR(v)       (((v) << 24) & 0x7000000)
/**< Field RDWR - Read write control */
#define IMX_ENET_QOS_MAC_VLAN_INCL_RDWR_SHIFT    30
#define IMX_ENET_QOS_MAC_VLAN_INCL_RDWR_MASK     0x40000000
#define IMX_ENET_QOS_MAC_VLAN_INCL_RDWR(v)       (((v) << 30) & 0x40000000)
/**< Field BUSY - Busy */
#define IMX_ENET_QOS_MAC_VLAN_INCL_BUSY_SHIFT    31
#define IMX_ENET_QOS_MAC_VLAN_INCL_BUSY_MASK     0x80000000
#define IMX_ENET_QOS_MAC_VLAN_INCL_BUSY(v)       (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_INNER_VLAN_INCL - MAC Inner VLAN Tag Inclusion or Replacement */
typedef union IMX_ENET_QOS_MAC_INNER_VLAN_INCL_union_t {
    UINT32 R;
    struct {
        UINT32 VLT : 16;                /**< VLAN Tag for Transmit Packets */
        UINT32 VLC : 2;                 /**< VLAN Tag Control in Transmit Packets */
        UINT32 VLP : 1;                 /**< VLAN Priority Control */
        UINT32 CSVL : 1;                 /**< C-VLAN or S-VLAN */
        UINT32 VLTI : 1;                 /**< VLAN Tag Input When this bit is set, it indicates that the VLAN tag to be inserted or replaced in Tx packet should be taken from: - The Tx descriptor */
        UINT32 _unused21 : 11;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_INNER_VLAN_INCL_t;

/**< Register MAC_INNER_VLAN_INCL - MAC Inner VLAN Tag Inclusion or Replacement */
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL         0x64
/**< Field VLT - VLAN Tag for Transmit Packets */
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLT_SHIFT 0
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLT_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLT(v)  (((v) << 0) & 0xFFFF)
/**< Field VLC - VLAN Tag Control in Transmit Packets */
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLC_SHIFT 16
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLC_MASK 0x30000
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLC(v)  (((v) << 16) & 0x30000)
/**< Field VLP - VLAN Priority Control */
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLP_SHIFT 18
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLP_MASK 0x40000
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLP(v)  (((v) << 18) & 0x40000)
/**< Field CSVL - C-VLAN or S-VLAN */
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_CSVL_SHIFT 19
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_CSVL_MASK 0x80000
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_CSVL(v) (((v) << 19) & 0x80000)
/**< Field VLTI - VLAN Tag Input When this bit is set, it indicates that the VLAN tag to be inserted or replaced in Tx packet should be taken from: - The Tx descriptor */
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLTI_SHIFT 20
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLTI_MASK 0x100000
#define IMX_ENET_QOS_MAC_INNER_VLAN_INCL_VLTI(v) (((v) << 20) & 0x100000)

/**< IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q - MAC Q0 Tx Flow Control */
typedef union IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_union_t {
    UINT32 R;
    struct {
        UINT32 FCB_BPA : 1;                 /**< Flow Control Busy or Backpressure Activate */
        UINT32 TFE : 1;                 /**< Transmit Flow Control Enable */
        UINT32 _unused2 : 2;                 /**< Reserved. */
        UINT32 PLT : 3;                 /**< Pause Low Threshold */
        UINT32 DZPQ : 1;                 /**< Disable Zero-Quanta Pause */
        UINT32 _unused8 : 8;                 /**< Reserved. */
        UINT32 PT : 16;                /**< Pause Time */
    } B;
} IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_t;

/**< Register MAC_TX_FLOW_CTRL_Q - MAC Q0 Tx Flow Control */
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q          0x70
/**< Field FCB_BPA - Flow Control Busy or Backpressure Activate */
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_FCB_BPA_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_FCB_BPA_MASK 0x1
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_FCB_BPA(v) (((v) << 0) & 0x1)
/**< Field TFE - Transmit Flow Control Enable */
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_TFE_SHIFT 1
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_TFE_MASK 0x2
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_TFE(v)   (((v) << 1) & 0x2)
/**< Field PLT - Pause Low Threshold */
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_PLT_SHIFT 4
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_PLT_MASK 0x70
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_PLT(v)   (((v) << 4) & 0x70)
/**< Field DZPQ - Disable Zero-Quanta Pause */
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_DZPQ_SHIFT 7
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_DZPQ_MASK 0x80
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_DZPQ(v)  (((v) << 7) & 0x80)
/**< Field PT - Pause Time */
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_PT_SHIFT 16
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_PT_MASK  0xFFFF0000
#define IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_PT(v)    (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MAC_RX_FLOW_CTRL - MAC Rx Flow Control */
typedef union IMX_ENET_QOS_MAC_RX_FLOW_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 RFE : 1;                 /**< Receive Flow Control Enable */
        UINT32 UP : 1;                 /**< Unicast Pause Packet Detect */
        UINT32 _unused2 : 6;                 /**< Reserved. */
        UINT32 PFCE : 1;                 /**< Priority Based Flow Control Enable */
        UINT32 _unused9 : 23;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_RX_FLOW_CTRL_t;

/**< Register MAC_RX_FLOW_CTRL - MAC Rx Flow Control */
#define IMX_ENET_QOS_MAC_RX_FLOW_CTRL            0x90
/**< Field RFE - Receive Flow Control Enable */
#define IMX_ENET_QOS_MAC_RX_FLOW_CTRL_RFE_SHIFT  0
#define IMX_ENET_QOS_MAC_RX_FLOW_CTRL_RFE_MASK   0x1
#define IMX_ENET_QOS_MAC_RX_FLOW_CTRL_RFE(v)     (((v) << 0) & 0x1)
/**< Field UP - Unicast Pause Packet Detect */
#define IMX_ENET_QOS_MAC_RX_FLOW_CTRL_UP_SHIFT   1
#define IMX_ENET_QOS_MAC_RX_FLOW_CTRL_UP_MASK    0x2
#define IMX_ENET_QOS_MAC_RX_FLOW_CTRL_UP(v)      (((v) << 1) & 0x2)
/**< Field PFCE - Priority Based Flow Control Enable */
#define IMX_ENET_QOS_MAC_RX_FLOW_CTRL_PFCE_SHIFT 8
#define IMX_ENET_QOS_MAC_RX_FLOW_CTRL_PFCE_MASK  0x100
#define IMX_ENET_QOS_MAC_RX_FLOW_CTRL_PFCE(v)    (((v) << 8) & 0x100)

/**< IMX_ENET_QOS_MAC_RXQ_CTRL4 - Receive Queue Control 4 */
typedef union IMX_ENET_QOS_MAC_RXQ_CTRL4_union_t {
    UINT32 R;
    struct {
        UINT32 UFFQE : 1;                 /**< Unicast Address Filter Fail Packets Queuing Enable. */
        UINT32 UFFQ : 3;                 /**< Unicast Address Filter Fail Packets Queue. */
        UINT32 _unused4 : 4;                 /**< Reserved. */
        UINT32 MFFQE : 1;                 /**< Multicast Address Filter Fail Packets Queuing Enable. */
        UINT32 MFFQ : 3;                 /**< Multicast Address Filter Fail Packets Queue. */
        UINT32 _unused12 : 4;                 /**< Reserved. */
        UINT32 VFFQE : 1;                 /**< VLAN Tag Filter Fail Packets Queuing Enable */
        UINT32 VFFQ : 3;                 /**< VLAN Tag Filter Fail Packets Queue */
        UINT32 _unused20 : 12;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_RXQ_CTRL4_t;

/**< Register MAC_RXQ_CTRL4 - Receive Queue Control 4 */
#define IMX_ENET_QOS_MAC_RXQ_CTRL4               0x94
/**< Field UFFQE - Unicast Address Filter Fail Packets Queuing Enable. */
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_UFFQE_SHIFT   0
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_UFFQE_MASK    0x1
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_UFFQE(v)      (((v) << 0) & 0x1)
/**< Field UFFQ - Unicast Address Filter Fail Packets Queue. */
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_UFFQ_SHIFT    1
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_UFFQ_MASK     0xE
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_UFFQ(v)       (((v) << 1) & 0xE)
/**< Field MFFQE - Multicast Address Filter Fail Packets Queuing Enable. */
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_MFFQE_SHIFT   8
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_MFFQE_MASK    0x100
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_MFFQE(v)      (((v) << 8) & 0x100)
/**< Field MFFQ - Multicast Address Filter Fail Packets Queue. */
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_MFFQ_SHIFT    9
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_MFFQ_MASK     0xE00
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_MFFQ(v)       (((v) << 9) & 0xE00)
/**< Field VFFQE - VLAN Tag Filter Fail Packets Queuing Enable */
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_VFFQE_SHIFT   16
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_VFFQE_MASK    0x10000
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_VFFQE(v)      (((v) << 16) & 0x10000)
/**< Field VFFQ - VLAN Tag Filter Fail Packets Queue */
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_VFFQ_SHIFT    17
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_VFFQ_MASK     0xE0000
#define IMX_ENET_QOS_MAC_RXQ_CTRL4_VFFQ(v)       (((v) << 17) & 0xE0000)

/**< IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0 - Transmit Queue Priority Mapping 0 */
typedef union IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_union_t {
    UINT32 R;
    struct {
        UINT32 PSTQ0 : 8;                 /**< Priorities Selected in Transmit Queue 0 */
        UINT32 PSTQ1 : 8;                 /**< Priorities Selected in Transmit Queue 1 This bit is similar to the PSTQ0 bit. */
        UINT32 PSTQ2 : 8;                 /**< Priorities Selected in Transmit Queue 2 This bit is similar to the PSTQ0 bit. */
        UINT32 PSTQ3 : 8;                 /**< Priorities Selected in Transmit Queue 3 This bit is similar to the PSTQ0 bit. */
    } B;
} IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_t;

/**< Register MAC_TXQ_PRTY_MAP0 - Transmit Queue Priority Mapping 0 */
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0           0x98
/**< Field PSTQ0 - Priorities Selected in Transmit Queue 0 */
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ0_SHIFT 0
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ0_MASK 0xFF
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ0(v)  (((v) << 0) & 0xFF)
/**< Field PSTQ1 - Priorities Selected in Transmit Queue 1 This bit is similar to the PSTQ0 bit. */
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ1_SHIFT 8
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ1_MASK 0xFF00
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ1(v)  (((v) << 8) & 0xFF00)
/**< Field PSTQ2 - Priorities Selected in Transmit Queue 2 This bit is similar to the PSTQ0 bit. */
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ2_SHIFT 16
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ2_MASK 0xFF0000
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ2(v)  (((v) << 16) & 0xFF0000)
/**< Field PSTQ3 - Priorities Selected in Transmit Queue 3 This bit is similar to the PSTQ0 bit. */
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ3_SHIFT 24
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ3_MASK 0xFF000000
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_PSTQ3(v)  (((v) << 24) & 0xFF000000)

/**< IMX_ENET_QOS_MAC_TXQ_PRTY_MAP1 - Transmit Queue Priority Mapping 1 */
typedef union IMX_ENET_QOS_MAC_TXQ_PRTY_MAP1_union_t {
    UINT32 R;
    struct {
        UINT32 PSTQ4 : 8;                 /**< Priorities Selected in Transmit Queue 4 */
        UINT32 _unused8 : 8;                 /**< Reserved. */
        UINT32 _unused16 : 8;                 /**< Reserved. */
        UINT32 _unused24 : 8;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_TXQ_PRTY_MAP1_t;

/**< Register MAC_TXQ_PRTY_MAP1 - Transmit Queue Priority Mapping 1 */
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP1           0x9C
/**< Field PSTQ4 - Priorities Selected in Transmit Queue 4 */
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP1_PSTQ4_SHIFT 0
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP1_PSTQ4_MASK 0xFF
#define IMX_ENET_QOS_MAC_TXQ_PRTY_MAP1_PSTQ4(v)  (((v) << 0) & 0xFF)

/**< IMX_ENET_QOS_MAC_RXQ_CTRL - Receive Queue Control 0 */
typedef union IMX_ENET_QOS_MAC_RXQ_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 RXQ0EN : 2;                 /**< Receive Queue 0 Enable This field indicates whether Rx Queue 0 is enabled for AV or DCB. */
        UINT32 RXQ1EN : 2;                 /**< Receive Queue 1 Enable This field is similar to the RXQ0EN field. */
        UINT32 RXQ2EN : 2;                 /**< Receive Queue 2 Enable This field is similar to the RXQ0EN field. */
        UINT32 RXQ3EN : 2;                 /**< Receive Queue 3 Enable This field is similar to the RXQ0EN field. */
        UINT32 RXQ4EN : 2;                 /**< Receive Queue 4 Enable This field is similar to the RXQ0EN field. */
        UINT32 _unused10 : 2;                 /**< Reserved. */
        UINT32 _unused12 : 2;                 /**< Reserved. */
        UINT32 _unused14 : 2;                 /**< Reserved. */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_RXQ_CTRL_t;

/**< Register MAC_RXQ_CTRL - Receive Queue Control 0 */
#define IMX_ENET_QOS_MAC_RXQ_CTRL                0xA0
/**< Field RXQ0EN - Receive Queue 0 Enable This field indicates whether Rx Queue 0 is enabled for AV or DCB. */
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ0EN_SHIFT   0
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ0EN_MASK    0x3
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ0EN(v)      (((v) << 0) & 0x3)
/**< Field RXQ1EN - Receive Queue 1 Enable This field is similar to the RXQ0EN field. */
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ1EN_SHIFT   2
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ1EN_MASK    0xC
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ1EN(v)      (((v) << 2) & 0xC)
/**< Field RXQ2EN - Receive Queue 2 Enable This field is similar to the RXQ0EN field. */
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ2EN_SHIFT   4
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ2EN_MASK    0x30
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ2EN(v)      (((v) << 4) & 0x30)
/**< Field RXQ3EN - Receive Queue 3 Enable This field is similar to the RXQ0EN field. */
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ3EN_SHIFT   6
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ3EN_MASK    0xC0
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ3EN(v)      (((v) << 6) & 0xC0)
/**< Field RXQ4EN - Receive Queue 4 Enable This field is similar to the RXQ0EN field. */
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ4EN_SHIFT   8
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ4EN_MASK    0x300
#define IMX_ENET_QOS_MAC_RXQ_CTRL_RXQ4EN(v)      (((v) << 8) & 0x300)

/**< IMX_ENET_QOS_MAC_INTERRUPT_STATUS - Interrupt Status */
typedef union IMX_ENET_QOS_MAC_INTERRUPT_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 RGSMIIIS : 1;                 /**< RGMII or SMII Interrupt Status */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 _unused2 : 1;                 /**< Reserved. */
        UINT32 PHYIS : 1;                 /**< PHY Interrupt */
        UINT32 PMTIS : 1;                 /**< PMT Interrupt Status */
        UINT32 LPIIS : 1;                 /**< LPI Interrupt Status */
        UINT32 _unused6 : 2;                 /**< Reserved. */
        UINT32 MMCIS : 1;                 /**< MMC Interrupt Status */
        UINT32 MMCRXIS : 1;                 /**< MMC Receive Interrupt Status */
        UINT32 MMCTXIS : 1;                 /**< MMC Transmit Interrupt Status */
        UINT32 MMCRXIPIS : 1;                 /**< MMC Receive Checksum Offload Interrupt Status */
        UINT32 TSIS : 1;                 /**< Timestamp Interrupt Status */
        UINT32 TXSTSIS : 1;                 /**< Transmit Status Interrupt */
        UINT32 RXSTSIS : 1;                 /**< Receive Status Interrupt */
        UINT32 _unused15 : 1;                 /**< Reserved. */
        UINT32 _unused16 : 1;                 /**< Reserved. */
        UINT32 FPEIS : 1;                 /**< Frame Preemption Interrupt Status */
        UINT32 MDIOIS : 1;                 /**< MDIO Interrupt Status */
        UINT32 MFTIS : 1;                 /**< MMC FPE Transmit Interrupt Status */
        UINT32 MFRIS : 1;                 /**< MMC FPE Receive Interrupt Status */
        UINT32 _unused21 : 11;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_INTERRUPT_STATUS_t;

/**< Register MAC_INTERRUPT_STATUS - Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS        0xB0
/**< Field RGSMIIIS - RGMII or SMII Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_RGSMIIIS_SHIFT 0
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_RGSMIIIS_MASK 0x1
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_RGSMIIIS(v) (((v) << 0) & 0x1)
/**< Field PHYIS - PHY Interrupt */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_PHYIS_SHIFT 3
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_PHYIS_MASK 0x8
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_PHYIS(v) (((v) << 3) & 0x8)
/**< Field PMTIS - PMT Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_PMTIS_SHIFT 4
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_PMTIS_MASK 0x10
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_PMTIS(v) (((v) << 4) & 0x10)
/**< Field LPIIS - LPI Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_LPIIS_SHIFT 5
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_LPIIS_MASK 0x20
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_LPIIS(v) (((v) << 5) & 0x20)
/**< Field MMCIS - MMC Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCIS_SHIFT 8
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCIS_MASK 0x100
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCIS(v) (((v) << 8) & 0x100)
/**< Field MMCRXIS - MMC Receive Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCRXIS_SHIFT 9
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCRXIS_MASK 0x200
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCRXIS(v) (((v) << 9) & 0x200)
/**< Field MMCTXIS - MMC Transmit Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCTXIS_SHIFT 10
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCTXIS_MASK 0x400
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCTXIS(v) (((v) << 10) & 0x400)
/**< Field MMCRXIPIS - MMC Receive Checksum Offload Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCRXIPIS_SHIFT 11
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCRXIPIS_MASK 0x800
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MMCRXIPIS(v) (((v) << 11) & 0x800)
/**< Field TSIS - Timestamp Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_TSIS_SHIFT 12
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_TSIS_MASK 0x1000
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_TSIS(v) (((v) << 12) & 0x1000)
/**< Field TXSTSIS - Transmit Status Interrupt */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_TXSTSIS_SHIFT 13
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_TXSTSIS_MASK 0x2000
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_TXSTSIS(v) (((v) << 13) & 0x2000)
/**< Field RXSTSIS - Receive Status Interrupt */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_RXSTSIS_SHIFT 14
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_RXSTSIS_MASK 0x4000
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_RXSTSIS(v) (((v) << 14) & 0x4000)
/**< Field FPEIS - Frame Preemption Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_FPEIS_SHIFT 17
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_FPEIS_MASK 0x20000
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_FPEIS(v) (((v) << 17) & 0x20000)
/**< Field MDIOIS - MDIO Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MDIOIS_SHIFT 18
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MDIOIS_MASK 0x40000
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MDIOIS(v) (((v) << 18) & 0x40000)
/**< Field MFTIS - MMC FPE Transmit Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MFTIS_SHIFT 19
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MFTIS_MASK 0x80000
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MFTIS(v) (((v) << 19) & 0x80000)
/**< Field MFRIS - MMC FPE Receive Interrupt Status */
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MFRIS_SHIFT 20
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MFRIS_MASK 0x100000
#define IMX_ENET_QOS_MAC_INTERRUPT_STATUS_MFRIS(v) (((v) << 20) & 0x100000)

/**< IMX_ENET_QOS_MAC_INTERRUPT_ENABLE - Interrupt Enable */
typedef union IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_union_t {
    UINT32 R;
    struct {
        UINT32 RGSMIIIE : 1;                 /**< RGMII or SMII Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of RGSMIIIS bit in MAC_INTERRUPT_STATUS register. */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 _unused2 : 1;                 /**< Reserved. */
        UINT32 PHYIE : 1;                 /**< PHY Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[PHYIS]. */
        UINT32 PMTIE : 1;                 /**< PMT Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[PMTIS]. */
        UINT32 LPIIE : 1;                 /**< LPI Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[LPIIS]. */
        UINT32 _unused6 : 6;                 /**< Reserved. */
        UINT32 TSIE : 1;                 /**< Timestamp Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[TSIS]. */
        UINT32 TXSTSIE : 1;                 /**< Transmit Status Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[TXSTSIS]. */
        UINT32 RXSTSIE : 1;                 /**< Receive Status Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[RXSTSIS]. */
        UINT32 _unused15 : 1;                 /**< Reserved. */
        UINT32 _unused16 : 1;                 /**< Reserved. */
        UINT32 FPEIE : 1;                 /**< Frame Preemption Interrupt Enable When this bit is set, it enables the assertion of the interrupt when FPEIS field is set in the MAC_INTERRUPT_STATUS. */
        UINT32 MDIOIE : 1;                 /**< MDIO Interrupt Enable When this bit is set, it enables the assertion of the interrupt when MDIOIS field is set in the MAC_INTERRUPT_STATUS register. */
        UINT32 _unused19 : 13;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_t;

/**< Register MAC_INTERRUPT_ENABLE - Interrupt Enable */
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE        0xB4
/**< Field RGSMIIIE - RGMII or SMII Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of RGSMIIIS bit in MAC_INTERRUPT_STATUS register. */
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_RGSMIIIE_SHIFT 0
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_RGSMIIIE_MASK 0x1
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_RGSMIIIE(v) (((v) << 0) & 0x1)
/**< Field PHYIE - PHY Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[PHYIS]. */
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_PHYIE_SHIFT 3
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_PHYIE_MASK 0x8
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_PHYIE(v) (((v) << 3) & 0x8)
/**< Field PMTIE - PMT Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[PMTIS]. */
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_PMTIE_SHIFT 4
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_PMTIE_MASK 0x10
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_PMTIE(v) (((v) << 4) & 0x10)
/**< Field LPIIE - LPI Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[LPIIS]. */
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_LPIIE_SHIFT 5
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_LPIIE_MASK 0x20
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_LPIIE(v) (((v) << 5) & 0x20)
/**< Field TSIE - Timestamp Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[TSIS]. */
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_TSIE_SHIFT 12
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_TSIE_MASK 0x1000
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_TSIE(v) (((v) << 12) & 0x1000)
/**< Field TXSTSIE - Transmit Status Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[TXSTSIS]. */
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_TXSTSIE_SHIFT 13
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_TXSTSIE_MASK 0x2000
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_TXSTSIE(v) (((v) << 13) & 0x2000)
/**< Field RXSTSIE - Receive Status Interrupt Enable When this bit is set, it enables the assertion of the interrupt signal because of the setting of MAC_INTERRUPT_STATUS[RXSTSIS]. */
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_RXSTSIE_SHIFT 14
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_RXSTSIE_MASK 0x4000
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_RXSTSIE(v) (((v) << 14) & 0x4000)
/**< Field FPEIE - Frame Preemption Interrupt Enable When this bit is set, it enables the assertion of the interrupt when FPEIS field is set in the MAC_INTERRUPT_STATUS. */
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_FPEIE_SHIFT 17
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_FPEIE_MASK 0x20000
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_FPEIE(v) (((v) << 17) & 0x20000)
/**< Field MDIOIE - MDIO Interrupt Enable When this bit is set, it enables the assertion of the interrupt when MDIOIS field is set in the MAC_INTERRUPT_STATUS register. */
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_MDIOIE_SHIFT 18
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_MDIOIE_MASK 0x40000
#define IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_MDIOIE(v) (((v) << 18) & 0x40000)

/**< IMX_ENET_QOS_MAC_RX_TX_STATUS - Receive Transmit Status */
typedef union IMX_ENET_QOS_MAC_RX_TX_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 TJT : 1;                 /**< Transmit Jabber Timeout This bit indicates that the Transmit Jabber Timer expired which happens when the packet size exceeds 2,048 bytes (10,240 bytes when the Jumbo packet is enabled) and JD bit is reset in the MAC_CONFIGURATION register. */
        UINT32 NCARR : 1;                 /**< No Carrier When the DTXSTS bit is set in the MAC_OPERATION_MODE register, this bit indicates that the carrier signal from the PHY is not present at the end of preamble transmission. */
        UINT32 LCARR : 1;                 /**< Loss of Carrier When the DTXSTS bit is set in the MAC_OPERATION_MODE register, this bit indicates that the loss of carrier occurred during packet transmission, that is, the phy_crs_i signal was inactive for one or more transmission clock periods during packet transmission. */
        UINT32 EXDEF : 1;                 /**< Excessive Deferral When the DTXSTS bit is set in the MAC_OPERATION_MODE register and the DC bit is set in the MAC_CONFIGURATION register, this bit indicates that the transmission ended because of excessive deferral of over 24,288 bit times (155,680 in 1000/2500 Mbps mode or when Jumbo packet is enabled). */
        UINT32 LCOL : 1;                 /**< Late Collision When the DTXSTS bit is set in the MAC_OPERATION_MODE register, this bit indicates that the packet transmission aborted because a collision occurred after the collision window (64 bytes including Preamble in MII mode; 512 bytes including Preamble and Carrier Extension in GMII mode). */
        UINT32 EXCOL : 1;                 /**< Excessive Collisions When the DTXSTS bit is set in the MAC_OPERATION_MODE register, this bit indicates that the transmission aborted after 16 successive collisions while attempting to transmit the current packet. */
        UINT32 _unused6 : 2;                 /**< Reserved. */
        UINT32 RWT : 1;                 /**< Receive Watchdog Timeout This bit is set when a packet with length greater than 2,048 bytes is received (10, 240 bytes when Jumbo Packet mode is enabled) and the WD bit is reset in the MAC_CONFIGURATION register. */
        UINT32 _unused9 : 23;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_RX_TX_STATUS_t;

/**< Register MAC_RX_TX_STATUS - Receive Transmit Status */
#define IMX_ENET_QOS_MAC_RX_TX_STATUS            0xB8
/**< Field TJT - Transmit Jabber Timeout This bit indicates that the Transmit Jabber Timer expired which happens when the packet size exceeds 2,048 bytes (10,240 bytes when the Jumbo packet is enabled) and JD bit is reset in the MAC_CONFIGURATION register. */
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_TJT_SHIFT  0
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_TJT_MASK   0x1
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_TJT(v)     (((v) << 0) & 0x1)
/**< Field NCARR - No Carrier When the DTXSTS bit is set in the MAC_OPERATION_MODE register, this bit indicates that the carrier signal from the PHY is not present at the end of preamble transmission. */
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_NCARR_SHIFT 1
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_NCARR_MASK 0x2
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_NCARR(v)   (((v) << 1) & 0x2)
/**< Field LCARR - Loss of Carrier When the DTXSTS bit is set in the MAC_OPERATION_MODE register, this bit indicates that the loss of carrier occurred during packet transmission, that is, the phy_crs_i signal was inactive for one or more transmission clock periods during packet transmission. */
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_LCARR_SHIFT 2
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_LCARR_MASK 0x4
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_LCARR(v)   (((v) << 2) & 0x4)
/**< Field EXDEF - Excessive Deferral When the DTXSTS bit is set in the MAC_OPERATION_MODE register and the DC bit is set in the MAC_CONFIGURATION register, this bit indicates that the transmission ended because of excessive deferral of over 24,288 bit times (155,680 in 1000/2500 Mbps mode or when Jumbo packet is enabled). */
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_EXDEF_SHIFT 3
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_EXDEF_MASK 0x8
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_EXDEF(v)   (((v) << 3) & 0x8)
/**< Field LCOL - Late Collision When the DTXSTS bit is set in the MAC_OPERATION_MODE register, this bit indicates that the packet transmission aborted because a collision occurred after the collision window (64 bytes including Preamble in MII mode; 512 bytes including Preamble and Carrier Extension in GMII mode). */
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_LCOL_SHIFT 4
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_LCOL_MASK  0x10
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_LCOL(v)    (((v) << 4) & 0x10)
/**< Field EXCOL - Excessive Collisions When the DTXSTS bit is set in the MAC_OPERATION_MODE register, this bit indicates that the transmission aborted after 16 successive collisions while attempting to transmit the current packet. */
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_EXCOL_SHIFT 5
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_EXCOL_MASK 0x20
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_EXCOL(v)   (((v) << 5) & 0x20)
/**< Field RWT - Receive Watchdog Timeout This bit is set when a packet with length greater than 2,048 bytes is received (10, 240 bytes when Jumbo Packet mode is enabled) and the WD bit is reset in the MAC_CONFIGURATION register. */
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_RWT_SHIFT  8
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_RWT_MASK   0x100
#define IMX_ENET_QOS_MAC_RX_TX_STATUS_RWT(v)     (((v) << 8) & 0x100)

/**< IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS - PMT Control and Status */
typedef union IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 PWRDWN : 1;                 /**< Power Down When this bit is set, the MAC receiver drops all received packets until it receives the expected magic packet or remote wake-up packet. */
        UINT32 MGKPKTEN : 1;                 /**< Magic Packet Enable When this bit is set, a power management event is generated when the MAC receives a magic packet. */
        UINT32 RWKPKTEN : 1;                 /**< Remote Wake-Up Packet Enable When this bit is set, a power management event is generated when the MAC receives a remote wake-up packet. */
        UINT32 _unused3 : 2;                 /**< Reserved. */
        UINT32 MGKPRCVD : 1;                 /**< Magic Packet Received When this bit is set, it indicates that the power management event is generated because of the reception of a magic packet. */
        UINT32 RWKPRCVD : 1;                 /**< Remote Wake-Up Packet Received When this bit is set, it indicates that the power management event is generated because of the reception of a remote wake-up packet. */
        UINT32 _unused7 : 2;                 /**< Reserved. */
        UINT32 GLBLUCAST : 1;                 /**< Global Unicast When this bit set, any unicast packet filtered by the MAC (DAF) address recognition is detected as a remote wake-up packet. */
        UINT32 RWKPFE : 1;                 /**< Remote Wake-up Packet Forwarding Enable When this bit is set along with RWKPKTEN, the MAC receiver drops all received frames until it receives the expected Wake-up frame. */
        UINT32 _unused11 : 13;                /**< Reserved. */
        UINT32 RWKPTR : 5;                 /**< Remote Wake-up FIFO Pointer This field gives the current value (0 to 7, 15, or 31 when 4, 8, or 16 Remote Wake-up Packet Filters are selected) of the Remote Wake-up Packet Filter register pointer. */
        UINT32 _unused29 : 2;                 /**< Reserved. */
        UINT32 RWKFILTRST : 1;                 /**< Remote Wake-Up Packet Filter Register Pointer Reset When this bit is set, the remote wake-up packet filter register pointer is reset to 3'b000. */
    } B;
} IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_t;

/**< Register MAC_PMT_CONTROL_STATUS - PMT Control and Status */
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS      0xC0
/**< Field PWRDWN - Power Down When this bit is set, the MAC receiver drops all received packets until it receives the expected magic packet or remote wake-up packet. */
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_PWRDWN_SHIFT 0
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_PWRDWN_MASK 0x1
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_PWRDWN(v) (((v) << 0) & 0x1)
/**< Field MGKPKTEN - Magic Packet Enable When this bit is set, a power management event is generated when the MAC receives a magic packet. */
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_MGKPKTEN_SHIFT 1
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_MGKPKTEN_MASK 0x2
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_MGKPKTEN(v) (((v) << 1) & 0x2)
/**< Field RWKPKTEN - Remote Wake-Up Packet Enable When this bit is set, a power management event is generated when the MAC receives a remote wake-up packet. */
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPKTEN_SHIFT 2
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPKTEN_MASK 0x4
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPKTEN(v) (((v) << 2) & 0x4)
/**< Field MGKPRCVD - Magic Packet Received When this bit is set, it indicates that the power management event is generated because of the reception of a magic packet. */
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_MGKPRCVD_SHIFT 5
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_MGKPRCVD_MASK 0x20
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_MGKPRCVD(v) (((v) << 5) & 0x20)
/**< Field RWKPRCVD - Remote Wake-Up Packet Received When this bit is set, it indicates that the power management event is generated because of the reception of a remote wake-up packet. */
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPRCVD_SHIFT 6
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPRCVD_MASK 0x40
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPRCVD(v) (((v) << 6) & 0x40)
/**< Field GLBLUCAST - Global Unicast When this bit set, any unicast packet filtered by the MAC (DAF) address recognition is detected as a remote wake-up packet. */
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_GLBLUCAST_SHIFT 9
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_GLBLUCAST_MASK 0x200
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_GLBLUCAST(v) (((v) << 9) & 0x200)
/**< Field RWKPFE - Remote Wake-up Packet Forwarding Enable When this bit is set along with RWKPKTEN, the MAC receiver drops all received frames until it receives the expected Wake-up frame. */
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPFE_SHIFT 10
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPFE_MASK 0x400
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPFE(v) (((v) << 10) & 0x400)
/**< Field RWKPTR - Remote Wake-up FIFO Pointer This field gives the current value (0 to 7, 15, or 31 when 4, 8, or 16 Remote Wake-up Packet Filters are selected) of the Remote Wake-up Packet Filter register pointer. */
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPTR_SHIFT 24
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPTR_MASK 0x1F000000
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKPTR(v) (((v) << 24) & 0x1F000000)
/**< Field RWKFILTRST - Remote Wake-Up Packet Filter Register Pointer Reset When this bit is set, the remote wake-up packet filter register pointer is reset to 3'b000. */
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKFILTRST_SHIFT 31
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKFILTRST_MASK 0x80000000
#define IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_RWKFILTRST(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_RWK_PACKET_FILTER - Remote Wakeup Filter */
typedef union IMX_ENET_QOS_MAC_RWK_PACKET_FILTER_union_t {
    UINT32 R;
    struct {
        UINT32 WKUPFRMFTR : 32;                /**< RWK Packet Filter This field contains the various controls of RWK Packet filter. */
    } B;
} IMX_ENET_QOS_MAC_RWK_PACKET_FILTER_t;

/**< Register MAC_RWK_PACKET_FILTER - Remote Wakeup Filter */
#define IMX_ENET_QOS_MAC_RWK_PACKET_FILTER       0xC4
/**< Field WKUPFRMFTR - RWK Packet Filter This field contains the various controls of RWK Packet filter. */
#define IMX_ENET_QOS_MAC_RWK_PACKET_FILTER_WKUPFRMFTR_SHIFT 0
#define IMX_ENET_QOS_MAC_RWK_PACKET_FILTER_WKUPFRMFTR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RWK_PACKET_FILTER_WKUPFRMFTR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS - LPI Control and Status */
typedef union IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 TLPIEN : 1;                 /**< Transmit LPI Entry When this bit is set, it indicates that the MAC Transmitter has entered the LPI state because of the setting of the LPIEN bit. */
        UINT32 TLPIEX : 1;                 /**< Transmit LPI Exit When this bit is set, it indicates that the MAC transmitter exited the LPI state after the application cleared the LPIEN bit and the LPI TW Timer has expired. */
        UINT32 RLPIEN : 1;                 /**< Receive LPI Entry When this bit is set, it indicates that the MAC Receiver has received an LPI pattern and entered the LPI state. */
        UINT32 RLPIEX : 1;                 /**< Receive LPI Exit When this bit is set, it indicates that the MAC Receiver has stopped receiving the LPI pattern on the GMII or MII interface, exited the LPI state, and resumed the normal reception. */
        UINT32 _unused4 : 4;                 /**< Reserved. */
        UINT32 TLPIST : 1;                 /**< Transmit LPI State When this bit is set, it indicates that the MAC is transmitting the LPI pattern on the GMII or MII interface. */
        UINT32 RLPIST : 1;                 /**< Receive LPI State When this bit is set, it indicates that the MAC is receiving the LPI pattern on the GMII or MII interface. */
        UINT32 _unused10 : 6;                 /**< Reserved. */
        UINT32 LPIEN : 1;                 /**< LPI Enable When this bit is set, it instructs the MAC Transmitter to enter the LPI state. */
        UINT32 PLS : 1;                 /**< PHY Link Status This bit indicates the link status of the PHY. */
        UINT32 PLSEN : 1;                 /**< PHY Link Status Enable This bit enables the link status received on the RGMII, SGMII, or SMII Receive paths to be used for activating the LPI LS TIMER. */
        UINT32 LPITXA : 1;                 /**< LPI Tx Automate This bit controls the behavior of the MAC when it is entering or coming out of the LPI mode on the Transmit side. */
        UINT32 LPIATE : 1;                 /**< LPI Timer Enable This bit controls the automatic entry of the MAC Transmitter into and exit out of the LPI state. */
        UINT32 LPITCSE : 1;                 /**< LPI Tx Clock Stop Enable When this bit is set, the MAC asserts sbd_tx_clk_gating_ctrl_o signal high after it enters Tx LPI mode to indicate that the Tx clock to MAC can be stopped. */
        UINT32 _unused22 : 10;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_t;

/**< Register MAC_LPI_CONTROL_STATUS - LPI Control and Status */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS      0xD0
/**< Field TLPIEN - Transmit LPI Entry When this bit is set, it indicates that the MAC Transmitter has entered the LPI state because of the setting of the LPIEN bit. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_TLPIEN_SHIFT 0
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_TLPIEN_MASK 0x1
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_TLPIEN(v) (((v) << 0) & 0x1)
/**< Field TLPIEX - Transmit LPI Exit When this bit is set, it indicates that the MAC transmitter exited the LPI state after the application cleared the LPIEN bit and the LPI TW Timer has expired. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_TLPIEX_SHIFT 1
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_TLPIEX_MASK 0x2
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_TLPIEX(v) (((v) << 1) & 0x2)
/**< Field RLPIEN - Receive LPI Entry When this bit is set, it indicates that the MAC Receiver has received an LPI pattern and entered the LPI state. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_RLPIEN_SHIFT 2
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_RLPIEN_MASK 0x4
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_RLPIEN(v) (((v) << 2) & 0x4)
/**< Field RLPIEX - Receive LPI Exit When this bit is set, it indicates that the MAC Receiver has stopped receiving the LPI pattern on the GMII or MII interface, exited the LPI state, and resumed the normal reception. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_RLPIEX_SHIFT 3
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_RLPIEX_MASK 0x8
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_RLPIEX(v) (((v) << 3) & 0x8)
/**< Field TLPIST - Transmit LPI State When this bit is set, it indicates that the MAC is transmitting the LPI pattern on the GMII or MII interface. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_TLPIST_SHIFT 8
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_TLPIST_MASK 0x100
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_TLPIST(v) (((v) << 8) & 0x100)
/**< Field RLPIST - Receive LPI State When this bit is set, it indicates that the MAC is receiving the LPI pattern on the GMII or MII interface. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_RLPIST_SHIFT 9
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_RLPIST_MASK 0x200
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_RLPIST(v) (((v) << 9) & 0x200)
/**< Field LPIEN - LPI Enable When this bit is set, it instructs the MAC Transmitter to enter the LPI state. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPIEN_SHIFT 16
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPIEN_MASK 0x10000
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPIEN(v) (((v) << 16) & 0x10000)
/**< Field PLS - PHY Link Status This bit indicates the link status of the PHY. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_PLS_SHIFT 17
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_PLS_MASK 0x20000
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_PLS(v) (((v) << 17) & 0x20000)
/**< Field PLSEN - PHY Link Status Enable This bit enables the link status received on the RGMII, SGMII, or SMII Receive paths to be used for activating the LPI LS TIMER. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_PLSEN_SHIFT 18
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_PLSEN_MASK 0x40000
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_PLSEN(v) (((v) << 18) & 0x40000)
/**< Field LPITXA - LPI Tx Automate This bit controls the behavior of the MAC when it is entering or coming out of the LPI mode on the Transmit side. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPITXA_SHIFT 19
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPITXA_MASK 0x80000
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPITXA(v) (((v) << 19) & 0x80000)
/**< Field LPIATE - LPI Timer Enable This bit controls the automatic entry of the MAC Transmitter into and exit out of the LPI state. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPIATE_SHIFT 20
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPIATE_MASK 0x100000
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPIATE(v) (((v) << 20) & 0x100000)
/**< Field LPITCSE - LPI Tx Clock Stop Enable When this bit is set, the MAC asserts sbd_tx_clk_gating_ctrl_o signal high after it enters Tx LPI mode to indicate that the Tx clock to MAC can be stopped. */
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPITCSE_SHIFT 21
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPITCSE_MASK 0x200000
#define IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_LPITCSE(v) (((v) << 21) & 0x200000)

/**< IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL - LPI Timers Control */
typedef union IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL_union_t {
    UINT32 R;
    struct {
        UINT32 TWT : 16;                /**< LPI TW Timer This field specifies the minimum time (in microseconds) for which the MAC waits after it stops transmitting the LPI pattern to the PHY and before it resumes the normal transmission. */
        UINT32 LST : 10;                /**< LPI LS Timer This field specifies the minimum time (in milliseconds) for which the link status from the PHY should be up (OKAY) before the LPI pattern can be transmitted to the PHY. */
        UINT32 _unused26 : 6;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL_t;

/**< Register MAC_LPI_TIMERS_CONTROL - LPI Timers Control */
#define IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL      0xD4
/**< Field TWT - LPI TW Timer This field specifies the minimum time (in microseconds) for which the MAC waits after it stops transmitting the LPI pattern to the PHY and before it resumes the normal transmission. */
#define IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL_TWT_SHIFT 0
#define IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL_TWT_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL_TWT(v) (((v) << 0) & 0xFFFF)
/**< Field LST - LPI LS Timer This field specifies the minimum time (in milliseconds) for which the link status from the PHY should be up (OKAY) before the LPI pattern can be transmitted to the PHY. */
#define IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL_LST_SHIFT 16
#define IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL_LST_MASK 0x3FF0000
#define IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL_LST(v) (((v) << 16) & 0x3FF0000)

/**< IMX_ENET_QOS_MAC_LPI_ENTRY_TIMER - Tx LPI Entry Timer Control */
typedef union IMX_ENET_QOS_MAC_LPI_ENTRY_TIMER_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 3;                 /**< Reserved. */
        UINT32 LPIET : 17;                /**< LPI Entry Timer This field specifies the time in microseconds the MAC waits to enter LPI mode, after it has transmitted all the frames. */
        UINT32 _unused20 : 12;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_LPI_ENTRY_TIMER_t;

/**< Register MAC_LPI_ENTRY_TIMER - Tx LPI Entry Timer Control */
#define IMX_ENET_QOS_MAC_LPI_ENTRY_TIMER         0xD8
/**< Field LPIET - LPI Entry Timer This field specifies the time in microseconds the MAC waits to enter LPI mode, after it has transmitted all the frames. */
#define IMX_ENET_QOS_MAC_LPI_ENTRY_TIMER_LPIET_SHIFT 3
#define IMX_ENET_QOS_MAC_LPI_ENTRY_TIMER_LPIET_MASK 0xFFFF8
#define IMX_ENET_QOS_MAC_LPI_ENTRY_TIMER_LPIET(v) (((v) << 3) & 0xFFFF8)

/**< IMX_ENET_QOS_MAC_ONEUS_TIC_COUNTER - One-microsecond Reference Timer */
typedef union IMX_ENET_QOS_MAC_ONEUS_TIC_COUNTER_union_t {
    UINT32 R;
    struct {
        UINT32 TIC_1US_CNTR : 12;               /**< 1US TIC Counter The application must program this counter so that the number of clock cycles of CSR clock is 1us. */
        UINT32 _unused12 : 20;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_ONEUS_TIC_COUNTER_t;

/**< Register MAC_ONEUS_TIC_COUNTER - One-microsecond Reference Timer */
#define IMX_ENET_QOS_MAC_ONEUS_TIC_COUNTER       0xDC
/**< Field TIC_1US_CNTR - 1US TIC Counter The application must program this counter so that the number of clock cycles of CSR clock is 1us. */
#define IMX_ENET_QOS_MAC_ONEUS_TIC_COUNTER_TIC_1US_CNTR_SHIFT 0
#define IMX_ENET_QOS_MAC_ONEUS_TIC_COUNTER_TIC_1US_CNTR_MASK 0xFFF
#define IMX_ENET_QOS_MAC_ONEUS_TIC_COUNTER_TIC_1US_CNTR(v) (((v) << 0) & 0xFFF)

/**< IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS - PHY Interface Control and Status */
typedef union IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 TC : 1;                 /**< Transmit Configuration in RGMII, SGMII, or SMII When set, this bit enables the transmission of duplex mode, link speed, and link up or down information to the PHY in the RGMII, SMII, or SGMII port. */
        UINT32 LUD : 1;                 /**< Link Up or Down This bit indicates whether the link is up or down during transmission of configuration in the RGMII, SGMII, or SMII interface. */
        UINT32 _unused2 : 1;                 /**< Reserved. */
        UINT32 _unused3 : 1;                 /**< Reserved. */
        UINT32 _unused4 : 1;                 /**< Reserved. */
        UINT32 _unused5 : 11;                /**< Reserved. */
        UINT32 LNKMOD : 1;                 /**< Link Mode This bit indicates the current mode of operation of the link. */
        UINT32 LNKSPEED : 2;                 /**< Link Speed This bit indicates the current speed of the link. */
        UINT32 LNKSTS : 1;                 /**< Link Status This bit indicates whether the link is up (1'b1) or down (1'b0). */
        UINT32 _unused20 : 1;                 /**< Reserved. */
        UINT32 _unused21 : 1;                 /**< Reserved. */
        UINT32 _unused22 : 10;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_t;

/**< Register MAC_PHYIF_CONTROL_STATUS - PHY Interface Control and Status */
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS    0xF8
/**< Field TC - Transmit Configuration in RGMII, SGMII, or SMII When set, this bit enables the transmission of duplex mode, link speed, and link up or down information to the PHY in the RGMII, SMII, or SGMII port. */
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_TC_SHIFT 0
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_TC_MASK 0x1
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_TC(v) (((v) << 0) & 0x1)
/**< Field LUD - Link Up or Down This bit indicates whether the link is up or down during transmission of configuration in the RGMII, SGMII, or SMII interface. */
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LUD_SHIFT 1
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LUD_MASK 0x2
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LUD(v) (((v) << 1) & 0x2)
/**< Field LNKMOD - Link Mode This bit indicates the current mode of operation of the link. */
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LNKMOD_SHIFT 16
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LNKMOD_MASK 0x10000
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LNKMOD(v) (((v) << 16) & 0x10000)
/**< Field LNKSPEED - Link Speed This bit indicates the current speed of the link. */
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LNKSPEED_SHIFT 17
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LNKSPEED_MASK 0x60000
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LNKSPEED(v) (((v) << 17) & 0x60000)
/**< Field LNKSTS - Link Status This bit indicates whether the link is up (1'b1) or down (1'b0). */
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LNKSTS_SHIFT 19
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LNKSTS_MASK 0x80000
#define IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_LNKSTS(v) (((v) << 19) & 0x80000)

/**< IMX_ENET_QOS_MAC_VERSION - MAC Version */
typedef union IMX_ENET_QOS_MAC_VERSION_union_t {
    UINT32 R;
    struct {
        UINT32 SNPSVER : 8;                 /**< Synopsys-defined Version */
        UINT32 USERVER : 8;                 /**< User-defined Version (8'h10) */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_VERSION_t;

/**< Register MAC_VERSION - MAC Version */
#define IMX_ENET_QOS_MAC_VERSION                 0x110
/**< Field SNPSVER - Synopsys-defined Version */
#define IMX_ENET_QOS_MAC_VERSION_SNPSVER_SHIFT   0
#define IMX_ENET_QOS_MAC_VERSION_SNPSVER_MASK    0xFF
#define IMX_ENET_QOS_MAC_VERSION_SNPSVER(v)      (((v) << 0) & 0xFF)
/**< Field USERVER - User-defined Version (8'h10) */
#define IMX_ENET_QOS_MAC_VERSION_USERVER_SHIFT   8
#define IMX_ENET_QOS_MAC_VERSION_USERVER_MASK    0xFF00
#define IMX_ENET_QOS_MAC_VERSION_USERVER(v)      (((v) << 8) & 0xFF00)

/**< IMX_ENET_QOS_MAC_DEBUG - MAC Debug */
typedef union IMX_ENET_QOS_MAC_DEBUG_union_t {
    UINT32 R;
    struct {
        UINT32 RPESTS : 1;                 /**< MAC GMII or MII Receive Protocol Engine Status When this bit is set, it indicates that the MAC GMII or MII receive protocol engine is actively receiving data, and it is not in the Idle state. */
        UINT32 RFCFCSTS : 2;                 /**< MAC Receive Packet Controller FIFO Status When this bit is set, this field indicates the active state of the small FIFO Read and Write controllers of the MAC Receive Packet Controller module. */
        UINT32 _unused3 : 13;                /**< Reserved. */
        UINT32 TPESTS : 1;                 /**< MAC GMII or MII Transmit Protocol Engine Status When this bit is set, it indicates that the MAC GMII or MII transmit protocol engine is actively transmitting data, and it is not in the Idle state. */
        UINT32 TFCSTS : 2;                 /**< MAC Transmit Packet Controller Status This field indicates the state of the MAC Transmit Packet Controller module. */
        UINT32 _unused19 : 13;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_DEBUG_t;

/**< Register MAC_DEBUG - MAC Debug */
#define IMX_ENET_QOS_MAC_DEBUG                   0x114
/**< Field RPESTS - MAC GMII or MII Receive Protocol Engine Status When this bit is set, it indicates that the MAC GMII or MII receive protocol engine is actively receiving data, and it is not in the Idle state. */
#define IMX_ENET_QOS_MAC_DEBUG_RPESTS_SHIFT      0
#define IMX_ENET_QOS_MAC_DEBUG_RPESTS_MASK       0x1
#define IMX_ENET_QOS_MAC_DEBUG_RPESTS(v)         (((v) << 0) & 0x1)
/**< Field RFCFCSTS - MAC Receive Packet Controller FIFO Status When this bit is set, this field indicates the active state of the small FIFO Read and Write controllers of the MAC Receive Packet Controller module. */
#define IMX_ENET_QOS_MAC_DEBUG_RFCFCSTS_SHIFT    1
#define IMX_ENET_QOS_MAC_DEBUG_RFCFCSTS_MASK     0x6
#define IMX_ENET_QOS_MAC_DEBUG_RFCFCSTS(v)       (((v) << 1) & 0x6)
/**< Field TPESTS - MAC GMII or MII Transmit Protocol Engine Status When this bit is set, it indicates that the MAC GMII or MII transmit protocol engine is actively transmitting data, and it is not in the Idle state. */
#define IMX_ENET_QOS_MAC_DEBUG_TPESTS_SHIFT      16
#define IMX_ENET_QOS_MAC_DEBUG_TPESTS_MASK       0x10000
#define IMX_ENET_QOS_MAC_DEBUG_TPESTS(v)         (((v) << 16) & 0x10000)
/**< Field TFCSTS - MAC Transmit Packet Controller Status This field indicates the state of the MAC Transmit Packet Controller module. */
#define IMX_ENET_QOS_MAC_DEBUG_TFCSTS_SHIFT      17
#define IMX_ENET_QOS_MAC_DEBUG_TFCSTS_MASK       0x60000
#define IMX_ENET_QOS_MAC_DEBUG_TFCSTS(v)         (((v) << 17) & 0x60000)

/**< IMX_ENET_QOS_MAC_HW_FEAT - Optional Features or Functions 0 */
typedef union IMX_ENET_QOS_MAC_HW_FEAT_union_t {
    UINT32 R;
    struct {
        UINT32 MIISEL : 1;                 /**< 10 or 100 Mbps Support This bit is set to 1 when 10/100 Mbps is selected as the Mode of Operation */
        UINT32 GMIISEL : 1;                 /**< 1000 Mbps Support This bit is set to 1 when 1000 Mbps is selected as the Mode of Operation */
        UINT32 HDSEL : 1;                 /**< Half-duplex Support This bit is set to 1 when the half-duplex mode is selected */
        UINT32 PCSSEL : 1;                 /**< PCS Registers (TBI, SGMII, or RTBI PHY interface) This bit is set to 1 when the TBI, SGMII, or RTBI PHY interface option is selected */
        UINT32 VLHASH : 1;                 /**< VLAN Hash Filter Selected This bit is set to 1 when the Enable VLAN Hash Table Based Filtering option is selected */
        UINT32 SMASEL : 1;                 /**< SMA (MDIO) Interface This bit is set to 1 when the Enable Station Management (MDIO Interface) option is selected */
        UINT32 RWKSEL : 1;                 /**< PMT Remote Wake-up Packet Enable This bit is set to 1 when the Enable Remote Wake-Up Packet Detection option is selected */
        UINT32 MGKSEL : 1;                 /**< PMT Magic Packet Enable This bit is set to 1 when the Enable Magic Packet Detection option is selected */
        UINT32 MMCSEL : 1;                 /**< RMON Module Enable This bit is set to 1 when the Enable MAC Management Counters (MMC) option is selected */
        UINT32 ARPOFFSEL : 1;                 /**< ARP Offload Enabled This bit is set to 1 when the Enable IPv4 ARP Offload option is selected */
        UINT32 _unused10 : 2;                 /**< Reserved. */
        UINT32 TSSEL : 1;                 /**< IEEE 1588-2008 Timestamp Enabled This bit is set to 1 when the Enable IEEE 1588 Timestamp Support option is selected */
        UINT32 EEESEL : 1;                 /**< Energy Efficient Ethernet Enabled This bit is set to 1 when the Enable Energy Efficient Ethernet (EEE) option is selected */
        UINT32 TXCOESEL : 1;                 /**< Transmit Checksum Offload Enabled This bit is set to 1 when the Enable Transmit TCP/IP Checksum Insertion option is selected */
        UINT32 _unused15 : 1;                 /**< Reserved. */
        UINT32 RXCOESEL : 1;                 /**< Receive Checksum Offload Enabled This bit is set to 1 when the Enable Receive TCP/IP Checksum Check option is selected */
        UINT32 _unused17 : 1;                 /**< Reserved. */
        UINT32 ADDMACADRSEL : 5;                /**< MAC Addresses 1-31 Selected This bit is set to 1 when the non-zero value is selected for Enable Additional 1-31 MAC Address Registers option */
        UINT32 MACADR32SEL : 1;                 /**< MAC Addresses 32-63 Selected This bit is set to 1 when the Enable Additional 32 MAC Address Registers (32-63) option is selected */
        UINT32 MACADR64SEL : 1;                 /**< MAC Addresses 64-127 Selected This bit is set to 1 when the Enable Additional 64 MAC Address Registers (64-127) option is selected */
        UINT32 TSSTSSEL : 2;                 /**< Timestamp System Time Source This bit indicates the source of the Timestamp system time: This bit is set to 1 when the Enable IEEE 1588 Timestamp Support option is selected */
        UINT32 SAVLANINS : 1;                 /**< Source Address or VLAN Insertion Enable This bit is set to 1 when the Enable SA and VLAN Insertion on Tx option is selected */
        UINT32 ACTPHYSEL : 3;                 /**< Active PHY Selected When you have multiple PHY interfaces in your configuration, this field indicates the sampled value of phy_intf_sel_i during reset de-assertion. */
        UINT32 _unused31 : 1;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_HW_FEAT_t;

/**< Register MAC_HW_FEAT - Optional Features or Functions 0 */
#define IMX_ENET_QOS_MAC_HW_FEAT                 0x11C
/**< Field MIISEL - 10 or 100 Mbps Support This bit is set to 1 when 10/100 Mbps is selected as the Mode of Operation */
#define IMX_ENET_QOS_MAC_HW_FEAT_MIISEL_SHIFT    0
#define IMX_ENET_QOS_MAC_HW_FEAT_MIISEL_MASK     0x1
#define IMX_ENET_QOS_MAC_HW_FEAT_MIISEL(v)       (((v) << 0) & 0x1)
/**< Field GMIISEL - 1000 Mbps Support This bit is set to 1 when 1000 Mbps is selected as the Mode of Operation */
#define IMX_ENET_QOS_MAC_HW_FEAT_GMIISEL_SHIFT   1
#define IMX_ENET_QOS_MAC_HW_FEAT_GMIISEL_MASK    0x2
#define IMX_ENET_QOS_MAC_HW_FEAT_GMIISEL(v)      (((v) << 1) & 0x2)
/**< Field HDSEL - Half-duplex Support This bit is set to 1 when the half-duplex mode is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_HDSEL_SHIFT     2
#define IMX_ENET_QOS_MAC_HW_FEAT_HDSEL_MASK      0x4
#define IMX_ENET_QOS_MAC_HW_FEAT_HDSEL(v)        (((v) << 2) & 0x4)
/**< Field PCSSEL - PCS Registers (TBI, SGMII, or RTBI PHY interface) This bit is set to 1 when the TBI, SGMII, or RTBI PHY interface option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_PCSSEL_SHIFT    3
#define IMX_ENET_QOS_MAC_HW_FEAT_PCSSEL_MASK     0x8
#define IMX_ENET_QOS_MAC_HW_FEAT_PCSSEL(v)       (((v) << 3) & 0x8)
/**< Field VLHASH - VLAN Hash Filter Selected This bit is set to 1 when the Enable VLAN Hash Table Based Filtering option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_VLHASH_SHIFT    4
#define IMX_ENET_QOS_MAC_HW_FEAT_VLHASH_MASK     0x10
#define IMX_ENET_QOS_MAC_HW_FEAT_VLHASH(v)       (((v) << 4) & 0x10)
/**< Field SMASEL - SMA (MDIO) Interface This bit is set to 1 when the Enable Station Management (MDIO Interface) option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_SMASEL_SHIFT    5
#define IMX_ENET_QOS_MAC_HW_FEAT_SMASEL_MASK     0x20
#define IMX_ENET_QOS_MAC_HW_FEAT_SMASEL(v)       (((v) << 5) & 0x20)
/**< Field RWKSEL - PMT Remote Wake-up Packet Enable This bit is set to 1 when the Enable Remote Wake-Up Packet Detection option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_RWKSEL_SHIFT    6
#define IMX_ENET_QOS_MAC_HW_FEAT_RWKSEL_MASK     0x40
#define IMX_ENET_QOS_MAC_HW_FEAT_RWKSEL(v)       (((v) << 6) & 0x40)
/**< Field MGKSEL - PMT Magic Packet Enable This bit is set to 1 when the Enable Magic Packet Detection option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_MGKSEL_SHIFT    7
#define IMX_ENET_QOS_MAC_HW_FEAT_MGKSEL_MASK     0x80
#define IMX_ENET_QOS_MAC_HW_FEAT_MGKSEL(v)       (((v) << 7) & 0x80)
/**< Field MMCSEL - RMON Module Enable This bit is set to 1 when the Enable MAC Management Counters (MMC) option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_MMCSEL_SHIFT    8
#define IMX_ENET_QOS_MAC_HW_FEAT_MMCSEL_MASK     0x100
#define IMX_ENET_QOS_MAC_HW_FEAT_MMCSEL(v)       (((v) << 8) & 0x100)
/**< Field ARPOFFSEL - ARP Offload Enabled This bit is set to 1 when the Enable IPv4 ARP Offload option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_ARPOFFSEL_SHIFT 9
#define IMX_ENET_QOS_MAC_HW_FEAT_ARPOFFSEL_MASK  0x200
#define IMX_ENET_QOS_MAC_HW_FEAT_ARPOFFSEL(v)    (((v) << 9) & 0x200)
/**< Field TSSEL - IEEE 1588-2008 Timestamp Enabled This bit is set to 1 when the Enable IEEE 1588 Timestamp Support option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_TSSEL_SHIFT     12
#define IMX_ENET_QOS_MAC_HW_FEAT_TSSEL_MASK      0x1000
#define IMX_ENET_QOS_MAC_HW_FEAT_TSSEL(v)        (((v) << 12) & 0x1000)
/**< Field EEESEL - Energy Efficient Ethernet Enabled This bit is set to 1 when the Enable Energy Efficient Ethernet (EEE) option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_EEESEL_SHIFT    13
#define IMX_ENET_QOS_MAC_HW_FEAT_EEESEL_MASK     0x2000
#define IMX_ENET_QOS_MAC_HW_FEAT_EEESEL(v)       (((v) << 13) & 0x2000)
/**< Field TXCOESEL - Transmit Checksum Offload Enabled This bit is set to 1 when the Enable Transmit TCP/IP Checksum Insertion option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_TXCOESEL_SHIFT  14
#define IMX_ENET_QOS_MAC_HW_FEAT_TXCOESEL_MASK   0x4000
#define IMX_ENET_QOS_MAC_HW_FEAT_TXCOESEL(v)     (((v) << 14) & 0x4000)
/**< Field RXCOESEL - Receive Checksum Offload Enabled This bit is set to 1 when the Enable Receive TCP/IP Checksum Check option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_RXCOESEL_SHIFT  16
#define IMX_ENET_QOS_MAC_HW_FEAT_RXCOESEL_MASK   0x10000
#define IMX_ENET_QOS_MAC_HW_FEAT_RXCOESEL(v)     (((v) << 16) & 0x10000)
/**< Field ADDMACADRSEL - MAC Addresses 1-31 Selected This bit is set to 1 when the non-zero value is selected for Enable Additional 1-31 MAC Address Registers option */
#define IMX_ENET_QOS_MAC_HW_FEAT_ADDMACADRSEL_SHIFT 18
#define IMX_ENET_QOS_MAC_HW_FEAT_ADDMACADRSEL_MASK 0x7C0000
#define IMX_ENET_QOS_MAC_HW_FEAT_ADDMACADRSEL(v) (((v) << 18) & 0x7C0000)
/**< Field MACADR32SEL - MAC Addresses 32-63 Selected This bit is set to 1 when the Enable Additional 32 MAC Address Registers (32-63) option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_MACADR32SEL_SHIFT 23
#define IMX_ENET_QOS_MAC_HW_FEAT_MACADR32SEL_MASK 0x800000
#define IMX_ENET_QOS_MAC_HW_FEAT_MACADR32SEL(v)  (((v) << 23) & 0x800000)
/**< Field MACADR64SEL - MAC Addresses 64-127 Selected This bit is set to 1 when the Enable Additional 64 MAC Address Registers (64-127) option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_MACADR64SEL_SHIFT 24
#define IMX_ENET_QOS_MAC_HW_FEAT_MACADR64SEL_MASK 0x1000000
#define IMX_ENET_QOS_MAC_HW_FEAT_MACADR64SEL(v)  (((v) << 24) & 0x1000000)
/**< Field TSSTSSEL - Timestamp System Time Source This bit indicates the source of the Timestamp system time: This bit is set to 1 when the Enable IEEE 1588 Timestamp Support option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_TSSTSSEL_SHIFT  25
#define IMX_ENET_QOS_MAC_HW_FEAT_TSSTSSEL_MASK   0x6000000
#define IMX_ENET_QOS_MAC_HW_FEAT_TSSTSSEL(v)     (((v) << 25) & 0x6000000)
/**< Field SAVLANINS - Source Address or VLAN Insertion Enable This bit is set to 1 when the Enable SA and VLAN Insertion on Tx option is selected */
#define IMX_ENET_QOS_MAC_HW_FEAT_SAVLANINS_SHIFT 27
#define IMX_ENET_QOS_MAC_HW_FEAT_SAVLANINS_MASK  0x8000000
#define IMX_ENET_QOS_MAC_HW_FEAT_SAVLANINS(v)    (((v) << 27) & 0x8000000)
/**< Field ACTPHYSEL - Active PHY Selected When you have multiple PHY interfaces in your configuration, this field indicates the sampled value of phy_intf_sel_i during reset de-assertion. */
#define IMX_ENET_QOS_MAC_HW_FEAT_ACTPHYSEL_SHIFT 28
#define IMX_ENET_QOS_MAC_HW_FEAT_ACTPHYSEL_MASK  0x70000000
#define IMX_ENET_QOS_MAC_HW_FEAT_ACTPHYSEL(v)    (((v) << 28) & 0x70000000)

#define IMX_ENET_QOS_MAC_HW_FEAT_RXFIFOSIZE_MASK     (0x1FU)
#define IMX_ENET_QOS_MAC_HW_FEAT_RXFIFOSIZE_SHIFT    (0U)
/*! RXFIFOSIZE - MTL Receive FIFO Size This field contains the configured value of MTL Rx FIFO in
 *    bytes expressed as Log to base 2 minus 7, that is, Log2(RXFIFO_SIZE) -7:
 *  0b00011..1024 bytes
 *  0b00000..128 bytes
 *  0b01010..128 KB
 *  0b00111..16384 bytes
 *  0b00100..2048 bytes
 *  0b00001..256 bytes
 *  0b01011..256 KB
 *  0b01000..32 KB
 *  0b00101..4096 bytes
 *  0b00010..512 bytes
 *  0b01001..64 KB
 *  0b00110..8192 bytes
 *  0b01100..Reserved
 */
#define IMX_ENET_QOS_MAC_HW_FEAT_RXFIFOSIZE(x)       (((uint32_t)(((uint32_t)(x)) << ENET_QOS_MAC_HW_FEAT_RXFIFOSIZE_SHIFT)) & ENET_QOS_MAC_HW_FEAT_RXFIFOSIZE_MASK)

#define IMX_ENET_QOS_MAC_HW_FEAT_TXFIFOSIZE_MASK     (0x7C0U)
#define IMX_ENET_QOS_MAC_HW_FEAT_TXFIFOSIZE_SHIFT    (6U)
 /*! TXFIFOSIZE - MTL Transmit FIFO Size This field contains the configured value of MTL Tx FIFO in
  *    bytes expressed as Log to base 2 minus 7, that is, Log2(TXFIFO_SIZE) -7:
  *  0b00011..1024 bytes
  *  0b00000..128 bytes
  *  0b01010..128 KB
  *  0b00111..16384 bytes
  *  0b00100..2048 bytes
  *  0b00001..256 bytes
  *  0b01000..32 KB
  *  0b00101..4096 bytes
  *  0b00010..512 bytes
  *  0b01001..64 KB
  *  0b00110..8192 bytes
  *  0b01011..Reserved
  */
#define IMX_ENET_QOS_MAC_HW_FEAT_TXFIFOSIZE(x)       (((uint32_t)(((uint32_t)(x)) << ENET_QOS_MAC_HW_FEAT_TXFIFOSIZE_SHIFT)) & ENET_QOS_MAC_HW_FEAT_TXFIFOSIZE_MASK)

/**< IMX_ENET_QOS_MAC_MDIO_ADDRESS - MDIO Address */
typedef union IMX_ENET_QOS_MAC_MDIO_ADDRESS_union_t {
    UINT32 R;
    struct {
        UINT32 GB : 1;                 /**< GMII Busy The application sets this bit to instruct the SMA to initiate a Read or Write access to the MDIO slave. */
        UINT32 C45E : 1;                 /**< Clause 45 PHY Enable When this bit is set, Clause 45 capable PHY is connected to MDIO. */
        UINT32 GOC_0 : 1;                 /**< GMII Operation Command 0 This is the lower bit of the operation command to the PHY or RevMII. */
        UINT32 GOC_1 : 1;                 /**< GMII Operation Command 1 This bit is higher bit of the operation command to the PHY or RevMII, GOC_1 and GOC_O is encoded as follows: - 00: Reserved - 01: Write - 10: Post Read Increment Address for Clause 45 PHY - 11: Read When Clause 22 PHY or RevMII is enabled, only Write and Read commands are valid. */
        UINT32 SKAP : 1;                 /**< Skip Address Packet When this bit is set, the SMA does not send the address packets before read, write, or post-read increment address packets. */
        UINT32 _unused5 : 3;                 /**< Reserved. */
        UINT32 CR : 4;                 /**< CSR Clock Range The CSR Clock Range selection determines the frequency of the MDC clock according to the CSR clock frequency used in your design: - 0000: CSR clock = 60-100 MHz; MDC clock = CSR clock/42 - 0001: CSR clock = 100-150 MHz; MDC clock = CSR clock/62 - 0010: CSR clock = 20-35 MHz; MDC clock = CSR clock/16 - 0011: CSR clock = 35-60 MHz; MDC clock = CSR clock/26 - 0100: CSR clock = 150-250 MHz; MDC clock = CSR clock/102 - 0101: CSR clock = 250-300 MHz; MDC clock = CSR clock/124 - 0110: CSR clock = 300-500 MHz; MDC clock = CSR clock/204 - 0111: CSR clock = 500-800 MHz; MDC clock = CSR clock/324 The suggested range of CSR clock frequency applicable for each value (when Bit 11 = 0) ensures that the MDC clock is approximately between 1. */
        UINT32 NTC : 3;                 /**< Number of Trailing Clocks This field controls the number of trailing clock cycles generated on gmii_mdc_o (MDC) after the end of transmission of MDIO frame. */
        UINT32 _unused15 : 1;                 /**< Reserved. */
        UINT32 RDA : 5;                 /**< Register/Device Address These bits select the PHY register in selected Clause 22 PHY device. */
        UINT32 PA : 5;                 /**< Physical Layer Address This field indicates which Clause 22 PHY devices (out of 32 devices) the MAC is accessing. */
        UINT32 BTB : 1;                 /**< Back to Back transactions When this bit is set and the NTC has value greater than 0, then the MAC informs the completion of a read or write command at the end of frame transfer (before the trailing clocks are transmitted). */
        UINT32 PSE : 1;                 /**< Preamble Suppression Enable When this bit is set, the SMA suppresses the 32-bit preamble and transmits MDIO frames with only 1 preamble bit. */
        UINT32 _unused28 : 4;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MDIO_ADDRESS_t;

/**< Register MAC_MDIO_ADDRESS - MDIO Address */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS            0x200
/**< Field GB - GMII Busy The application sets this bit to instruct the SMA to initiate a Read or Write access to the MDIO slave. */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_GB_SHIFT   0
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_GB_MASK    0x1
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_GB(v)      (((v) << 0) & 0x1)
/**< Field C45E - Clause 45 PHY Enable When this bit is set, Clause 45 capable PHY is connected to MDIO. */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_C45E_SHIFT 1
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_C45E_MASK  0x2
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_C45E(v)    (((v) << 1) & 0x2)
/**< Field GOC_0 - GMII Operation Command 0 This is the lower bit of the operation command to the PHY or RevMII. */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_GOC_0_SHIFT 2
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_GOC_0_MASK 0x4
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_GOC_0(v)   (((v) << 2) & 0x4)
/**< Field GOC_1 - GMII Operation Command 1 This bit is higher bit of the operation command to the PHY or RevMII, GOC_1 and GOC_O is encoded as follows: - 00: Reserved - 01: Write - 10: Post Read Increment Address for Clause 45 PHY - 11: Read When Clause 22 PHY or RevMII is enabled, only Write and Read commands are valid. */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_GOC_1_SHIFT 3
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_GOC_1_MASK 0x8
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_GOC_1(v)   (((v) << 3) & 0x8)
/**< Field SKAP - Skip Address Packet When this bit is set, the SMA does not send the address packets before read, write, or post-read increment address packets. */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_SKAP_SHIFT 4
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_SKAP_MASK  0x10
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_SKAP(v)    (((v) << 4) & 0x10)
/**< Field CR - CSR Clock Range The CSR Clock Range selection determines the frequency of the MDC clock according to the CSR clock frequency used in your design: - 0000: CSR clock = 60-100 MHz; MDC clock = CSR clock/42 - 0001: CSR clock = 100-150 MHz; MDC clock = CSR clock/62 - 0010: CSR clock = 20-35 MHz; MDC clock = CSR clock/16 - 0011: CSR clock = 35-60 MHz; MDC clock = CSR clock/26 - 0100: CSR clock = 150-250 MHz; MDC clock = CSR clock/102 - 0101: CSR clock = 250-300 MHz; MDC clock = CSR clock/124 - 0110: CSR clock = 300-500 MHz; MDC clock = CSR clock/204 - 0111: CSR clock = 500-800 MHz; MDC clock = CSR clock/324 The suggested range of CSR clock frequency applicable for each value (when Bit 11 = 0) ensures that the MDC clock is approximately between 1. */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_CR_SHIFT   8
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_CR_MASK    0xF00
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_CR(v)      (((v) << 8) & 0xF00)
/**< Field NTC - Number of Trailing Clocks This field controls the number of trailing clock cycles generated on gmii_mdc_o (MDC) after the end of transmission of MDIO frame. */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_NTC_SHIFT  12
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_NTC_MASK   0x7000
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_NTC(v)     (((v) << 12) & 0x7000)
/**< Field RDA - Register/Device Address These bits select the PHY register in selected Clause 22 PHY device. */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_RDA_SHIFT  16
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_RDA_MASK   0x1F0000
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_RDA(v)     (((v) << 16) & 0x1F0000)
/**< Field PA - Physical Layer Address This field indicates which Clause 22 PHY devices (out of 32 devices) the MAC is accessing. */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_PA_SHIFT   21
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_PA_MASK    0x3E00000
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_PA(v)      (((v) << 21) & 0x3E00000)
/**< Field BTB - Back to Back transactions When this bit is set and the NTC has value greater than 0, then the MAC informs the completion of a read or write command at the end of frame transfer (before the trailing clocks are transmitted). */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_BTB_SHIFT  26
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_BTB_MASK   0x4000000
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_BTB(v)     (((v) << 26) & 0x4000000)
/**< Field PSE - Preamble Suppression Enable When this bit is set, the SMA suppresses the 32-bit preamble and transmits MDIO frames with only 1 preamble bit. */
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_PSE_SHIFT  27
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_PSE_MASK   0x8000000
#define IMX_ENET_QOS_MAC_MDIO_ADDRESS_PSE(v)     (((v) << 27) & 0x8000000)

/**< IMX_ENET_QOS_MAC_MDIO_DATA - MAC MDIO Data */
typedef union IMX_ENET_QOS_MAC_MDIO_DATA_union_t {
    UINT32 R;
    struct {
        UINT32 GD : 16;                /**< GMII Data This field contains the 16-bit data value read from the PHY or RevMII after a Management Read operation or the 16-bit data value to be written to the PHY or RevMII before a Management Write operation. */
        UINT32 RA : 16;                /**< Register Address This field is valid only when C45E is set. */
    } B;
} IMX_ENET_QOS_MAC_MDIO_DATA_t;

/**< Register MAC_MDIO_DATA - MAC MDIO Data */
#define IMX_ENET_QOS_MAC_MDIO_DATA               0x204
/**< Field GD - GMII Data This field contains the 16-bit data value read from the PHY or RevMII after a Management Read operation or the 16-bit data value to be written to the PHY or RevMII before a Management Write operation. */
#define IMX_ENET_QOS_MAC_MDIO_DATA_GD_SHIFT      0
#define IMX_ENET_QOS_MAC_MDIO_DATA_GD_MASK       0xFFFF
#define IMX_ENET_QOS_MAC_MDIO_DATA_GD(v)         (((v) << 0) & 0xFFFF)
/**< Field RA - Register Address This field is valid only when C45E is set. */
#define IMX_ENET_QOS_MAC_MDIO_DATA_RA_SHIFT      16
#define IMX_ENET_QOS_MAC_MDIO_DATA_RA_MASK       0xFFFF0000
#define IMX_ENET_QOS_MAC_MDIO_DATA_RA(v)         (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MAC_CSR_SW_CTRL - CSR Software Control */
typedef union IMX_ENET_QOS_MAC_CSR_SW_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 RCWE : 1;                 /**< Register Clear on Write 1 Enable When this bit is set, the access mode of some register fields changes to Clear on Write 1, the application needs to set that respective bit to 1 to clear it. */
        UINT32 _unused1 : 7;                 /**< Reserved. */
        UINT32 _unused8 : 1;                 /**< Reserved. */
        UINT32 _unused9 : 23;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_CSR_SW_CTRL_t;

/**< Register MAC_CSR_SW_CTRL - CSR Software Control */
#define IMX_ENET_QOS_MAC_CSR_SW_CTRL             0x230
/**< Field RCWE - Register Clear on Write 1 Enable When this bit is set, the access mode of some register fields changes to Clear on Write 1, the application needs to set that respective bit to 1 to clear it. */
#define IMX_ENET_QOS_MAC_CSR_SW_CTRL_RCWE_SHIFT  0
#define IMX_ENET_QOS_MAC_CSR_SW_CTRL_RCWE_MASK   0x1
#define IMX_ENET_QOS_MAC_CSR_SW_CTRL_RCWE(v)     (((v) << 0) & 0x1)

/**< IMX_ENET_QOS_MAC_FPE_CTRL_STS - Frame Preemption Control */
typedef union IMX_ENET_QOS_MAC_FPE_CTRL_STS_union_t {
    UINT32 R;
    struct {
        UINT32 EFPE : 1;                 /**< Enable Tx Frame Preemption When set Frame Preemption Tx functionality is enabled. */
        UINT32 SVER : 1;                 /**< Send Verify mPacket When set indicates hardware to send a verify mPacket. */
        UINT32 SRSP : 1;                 /**< Send Respond mPacket When set indicates hardware to send a Respond mPacket. */
        UINT32 S1_SET_0 : 1;                 /**< Synopsys Reserved, Must be set to "0". */
        UINT32 _unused4 : 12;                /**< Reserved. */
        UINT32 RVER : 1;                 /**< Received Verify Frame Set when a Verify mPacket is received. */
        UINT32 RRSP : 1;                 /**< Received Respond Frame Set when a Respond mPacket is received. */
        UINT32 TVER : 1;                 /**< Transmitted Verify Frame Set when a Verify mPacket is transmitted (triggered by setting SVER field). */
        UINT32 TRSP : 1;                 /**< Transmitted Respond Frame Set when a Respond mPacket is transmitted (triggered by setting SRSP field). */
        UINT32 _unused20 : 12;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_FPE_CTRL_STS_t;

/**< Register MAC_FPE_CTRL_STS - Frame Preemption Control */
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS            0x234
/**< Field EFPE - Enable Tx Frame Preemption When set Frame Preemption Tx functionality is enabled. */
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_EFPE_SHIFT 0
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_EFPE_MASK  0x1
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_EFPE(v)    (((v) << 0) & 0x1)
/**< Field SVER - Send Verify mPacket When set indicates hardware to send a verify mPacket. */
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_SVER_SHIFT 1
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_SVER_MASK  0x2
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_SVER(v)    (((v) << 1) & 0x2)
/**< Field SRSP - Send Respond mPacket When set indicates hardware to send a Respond mPacket. */
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_SRSP_SHIFT 2
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_SRSP_MASK  0x4
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_SRSP(v)    (((v) << 2) & 0x4)
/**< Field S1_SET_0 - Synopsys Reserved, Must be set to "0". */
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_S1_SET_0_SHIFT 3
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_S1_SET_0_MASK 0x8
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_S1_SET_0(v) (((v) << 3) & 0x8)
/**< Field RVER - Received Verify Frame Set when a Verify mPacket is received. */
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_RVER_SHIFT 16
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_RVER_MASK  0x10000
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_RVER(v)    (((v) << 16) & 0x10000)
/**< Field RRSP - Received Respond Frame Set when a Respond mPacket is received. */
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_RRSP_SHIFT 17
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_RRSP_MASK  0x20000
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_RRSP(v)    (((v) << 17) & 0x20000)
/**< Field TVER - Transmitted Verify Frame Set when a Verify mPacket is transmitted (triggered by setting SVER field). */
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_TVER_SHIFT 18
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_TVER_MASK  0x40000
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_TVER(v)    (((v) << 18) & 0x40000)
/**< Field TRSP - Transmitted Respond Frame Set when a Respond mPacket is transmitted (triggered by setting SRSP field). */
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_TRSP_SHIFT 19
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_TRSP_MASK  0x80000
#define IMX_ENET_QOS_MAC_FPE_CTRL_STS_TRSP(v)    (((v) << 19) & 0x80000)

/**< IMX_ENET_QOS_MAC_PRESN_TIME_NS - 32-bit Binary Rollover Equivalent Time */
typedef union IMX_ENET_QOS_MAC_PRESN_TIME_NS_union_t {
    UINT32 R;
    struct {
        UINT32 MPTN : 32;                /**< MAC 1722 Presentation Time in ns These bits indicate the value of the 32-bit binary rollover equivalent time of the PTP System Time in ns */
    } B;
} IMX_ENET_QOS_MAC_PRESN_TIME_NS_t;

/**< Register MAC_PRESN_TIME_NS - 32-bit Binary Rollover Equivalent Time */
#define IMX_ENET_QOS_MAC_PRESN_TIME_NS           0x240
/**< Field MPTN - MAC 1722 Presentation Time in ns These bits indicate the value of the 32-bit binary rollover equivalent time of the PTP System Time in ns */
#define IMX_ENET_QOS_MAC_PRESN_TIME_NS_MPTN_SHIFT 0
#define IMX_ENET_QOS_MAC_PRESN_TIME_NS_MPTN_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PRESN_TIME_NS_MPTN(v)   (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PRESN_TIME_UPDT - MAC 1722 Presentation Time */
typedef union IMX_ENET_QOS_MAC_PRESN_TIME_UPDT_union_t {
    UINT32 R;
    struct {
        UINT32 MPTU : 32;                /**< MAC 1722 Presentation Time Update This field holds the init value or the update value for the presentation time. */
    } B;
} IMX_ENET_QOS_MAC_PRESN_TIME_UPDT_t;

/**< Register MAC_PRESN_TIME_UPDT - MAC 1722 Presentation Time */
#define IMX_ENET_QOS_MAC_PRESN_TIME_UPDT         0x244
/**< Field MPTU - MAC 1722 Presentation Time Update This field holds the init value or the update value for the presentation time. */
#define IMX_ENET_QOS_MAC_PRESN_TIME_UPDT_MPTU_SHIFT 0
#define IMX_ENET_QOS_MAC_PRESN_TIME_UPDT_MPTU_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PRESN_TIME_UPDT_MPTU(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_HIGH - MAC Address0 High */
typedef union IMX_ENET_QOS_HIGH_union_t {
    UINT32 R;
    struct {
        UINT32 ADDRHI : 16;                /**< MAC Address0[47:32] This field contains the upper 16 bits [47:32] of the first 6-byte MAC address. */
        UINT32 DCS : 5;                 /**< DMA Channel Select If the PDC bit of MAC_EXT_CONFIGURATION register is not set: This field contains the binary representation of the DMA Channel number to which an Rx packet whose DA matches the MAC Address0 content is routed. */
        UINT32 _unused21 : 10;                /**< Reserved. */
        UINT32 AE : 1;                 /**< Address Enable This bit is always set to 1. */
    } B;
} IMX_ENET_QOS_MAC_ADDRESS_HIGH_t;

/**< Register HIGH - MAC Address0 High */
#define IMX_ENET_QOS_HIGH                        0x300
/**< Field ADDRHI - MAC Address0[47:32] This field contains the upper 16 bits [47:32] of the first 6-byte MAC address. */
#define IMX_ENET_QOS_HIGH_ADDRHI_SHIFT           0
#define IMX_ENET_QOS_HIGH_ADDRHI_MASK            0xFFFF
#define IMX_ENET_QOS_HIGH_ADDRHI(v)              (((v) << 0) & 0xFFFF)
/**< Field DCS - DMA Channel Select If the PDC bit of MAC_EXT_CONFIGURATION register is not set: This field contains the binary representation of the DMA Channel number to which an Rx packet whose DA matches the MAC Address0 content is routed. */
#define IMX_ENET_QOS_HIGH_DCS_SHIFT              16
#define IMX_ENET_QOS_HIGH_DCS_MASK               0x1F0000
#define IMX_ENET_QOS_HIGH_DCS(v)                 (((v) << 16) & 0x1F0000)
/**< Field AE - Address Enable This bit is always set to 1. */
#define IMX_ENET_QOS_HIGH_AE_SHIFT               31
#define IMX_ENET_QOS_HIGH_AE_MASK                0x80000000
#define IMX_ENET_QOS_HIGH_AE(v)                  (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_LOW - MAC Address0 Low */
typedef union IMX_ENET_QOS_LOW_union_t {
    UINT32 R;
    struct {
        UINT32 ADDRLO : 32;                /**< MAC Address0[31:0] This field contains the lower 32 bits of the first 6-byte MAC address. */
    } B;
} IMX_ENET_QOS_MAC_ADDRESS_LOW_t;

/**< Register LOW - MAC Address0 Low */
#define IMX_ENET_QOS_LOW                         0x304
/**< Field ADDRLO - MAC Address0[31:0] This field contains the lower 32 bits of the first 6-byte MAC address. */
#define IMX_ENET_QOS_LOW_ADDRLO_SHIFT            0
#define IMX_ENET_QOS_LOW_ADDRLO_MASK             0xFFFFFFFF
#define IMX_ENET_QOS_LOW_ADDRLO(v)               (((v) << 0) & 0xFFFFFFFF)

typedef struct IMX_ENET_QOS_MAC_ADDRESS_struct_t {
    volatile IMX_ENET_QOS_MAC_ADDRESS_HIGH_t HIGH;          /**< MAC Address0 High */
    volatile IMX_ENET_QOS_MAC_ADDRESS_LOW_t LOW;            /**< MAC Address0 Low */
} IMX_ENET_QOS_MAC_ADDRESS_t;

/**< IMX_ENET_QOS_MAC_MMC_CONTROL - MMC Control */
typedef union IMX_ENET_QOS_MAC_MMC_CONTROL_union_t {
    UINT32 R;
    struct {
        UINT32 CNTRST : 1;                 /**< Counters Reset When this bit is set, all counters are reset. */
        UINT32 CNTSTOPRO : 1;                 /**< Counter Stop Rollover When this bit is set, the counter does not roll over to zero after reaching the maximum value. */
        UINT32 RSTONRD : 1;                 /**< Reset on Read When this bit is set, the MMC counters are reset to zero after Read (self-clearing after reset). */
        UINT32 CNTFREEZ : 1;                 /**< MMC Counter Freeze When this bit is set, it freezes all MMC counters to their current value. */
        UINT32 CNTPRST : 1;                 /**< Counters Preset When this bit is set, all counters are initialized or preset to almost full or almost half according to the CNTPRSTLVL bit. */
        UINT32 CNTPRSTLVL : 1;                 /**< Full-Half Preset When this bit is low and the CNTPRST bit is set, all MMC counters get preset to almost-half value. */
        UINT32 _unused6 : 2;                 /**< Reserved. */
        UINT32 UCDBC : 1;                 /**< Update MMC Counters for Dropped Broadcast Packets Note: The CNTRST bit has a higher priority than the CNTPRST bit. */
        UINT32 _unused9 : 23;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_CONTROL_t;

/**< Register MAC_MMC_CONTROL - MMC Control */
#define IMX_ENET_QOS_MAC_MMC_CONTROL             0x700
/**< Field CNTRST - Counters Reset When this bit is set, all counters are reset. */
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTRST_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTRST_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTRST(v)   (((v) << 0) & 0x1)
/**< Field CNTSTOPRO - Counter Stop Rollover When this bit is set, the counter does not roll over to zero after reaching the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTSTOPRO_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTSTOPRO_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTSTOPRO(v) (((v) << 1) & 0x2)
/**< Field RSTONRD - Reset on Read When this bit is set, the MMC counters are reset to zero after Read (self-clearing after reset). */
#define IMX_ENET_QOS_MAC_MMC_CONTROL_RSTONRD_SHIFT 2
#define IMX_ENET_QOS_MAC_MMC_CONTROL_RSTONRD_MASK 0x4
#define IMX_ENET_QOS_MAC_MMC_CONTROL_RSTONRD(v)  (((v) << 2) & 0x4)
/**< Field CNTFREEZ - MMC Counter Freeze When this bit is set, it freezes all MMC counters to their current value. */
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTFREEZ_SHIFT 3
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTFREEZ_MASK 0x8
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTFREEZ(v) (((v) << 3) & 0x8)
/**< Field CNTPRST - Counters Preset When this bit is set, all counters are initialized or preset to almost full or almost half according to the CNTPRSTLVL bit. */
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTPRST_SHIFT 4
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTPRST_MASK 0x10
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTPRST(v)  (((v) << 4) & 0x10)
/**< Field CNTPRSTLVL - Full-Half Preset When this bit is low and the CNTPRST bit is set, all MMC counters get preset to almost-half value. */
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTPRSTLVL_SHIFT 5
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTPRSTLVL_MASK 0x20
#define IMX_ENET_QOS_MAC_MMC_CONTROL_CNTPRSTLVL(v) (((v) << 5) & 0x20)
/**< Field UCDBC - Update MMC Counters for Dropped Broadcast Packets Note: The CNTRST bit has a higher priority than the CNTPRST bit. */
#define IMX_ENET_QOS_MAC_MMC_CONTROL_UCDBC_SHIFT 8
#define IMX_ENET_QOS_MAC_MMC_CONTROL_UCDBC_MASK  0x100
#define IMX_ENET_QOS_MAC_MMC_CONTROL_UCDBC(v)    (((v) << 8) & 0x100)

/**< IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT - MMC Rx Interrupt */
typedef union IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_union_t {
    UINT32 R;
    struct {
        UINT32 RXGBPKTIS : 1;                 /**< MMC Receive Good Bad Packet Counter Interrupt Status This bit is set when the rxpacketcount_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RXGBOCTIS : 1;                 /**< MMC Receive Good Bad Octet Counter Interrupt Status This bit is set when the rxoctetcount_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RXGOCTIS : 1;                 /**< MMC Receive Good Octet Counter Interrupt Status This bit is set when the rxoctetcount_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXBCGPIS : 1;                 /**< MMC Receive Broadcast Good Packet Counter Interrupt Status This bit is set when the rxbroadcastpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXMCGPIS : 1;                 /**< MMC Receive Multicast Good Packet Counter Interrupt Status This bit is set when the rxmulticastpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXCRCERPIS : 1;                 /**< MMC Receive CRC Error Packet Counter Interrupt Status This bit is set when the rxcrcerror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXALGNERPIS : 1;                 /**< MMC Receive Alignment Error Packet Counter Interrupt Status This bit is set when the rxalignmenterror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXRUNTPIS : 1;                 /**< MMC Receive Runt Packet Counter Interrupt Status This bit is set when the rxrunterror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXJABERPIS : 1;                 /**< MMC Receive Jabber Error Packet Counter Interrupt Status This bit is set when the rxjabbererror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUSIZEGPIS : 1;                 /**< MMC Receive Undersize Good Packet Counter Interrupt Status This bit is set when the rxundersize_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXOSIZEGPIS : 1;                 /**< MMC Receive Oversize Good Packet Counter Interrupt Status This bit is set when the rxoversize_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RX64OCTGBPIS : 1;                /**< MMC Receive 64 Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx64octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RX65T127OCTGBPIS : 1;            /**< MMC Receive 65 to 127 Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx65to127octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RX128T255OCTGBPIS : 1;           /**< MMC Receive 128 to 255 Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx128to255octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RX256T511OCTGBPIS : 1;           /**< MMC Receive 256 to 511 Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx256to511octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RX512T1023OCTGBPIS : 1;          /**< MMC Receive 512 to 1023 Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx512to1023octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RX1024TMAXOCTGBPIS : 1;          /**< MMC Receive 1024 to Maximum Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx1024tomaxoctets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUCGPIS : 1;                 /**< MMC Receive Unicast Good Packet Counter Interrupt Status This bit is set when the rxunicastpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXLENERPIS : 1;                 /**< MMC Receive Length Error Packet Counter Interrupt Status This bit is set when the rxlengtherror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXORANGEPIS : 1;                 /**< MMC Receive Out Of Range Error Packet Counter Interrupt Status. */
        UINT32 RXPAUSPIS : 1;                 /**< MMC Receive Pause Packet Counter Interrupt Status This bit is set when the rxpausepackets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXFOVPIS : 1;                 /**< MMC Receive FIFO Overflow Packet Counter Interrupt Status This bit is set when the rxfifooverflow counter reaches half of the maximum value or the maximum value. */
        UINT32 RXVLANGBPIS : 1;                 /**< MMC Receive VLAN Good Bad Packet Counter Interrupt Status This bit is set when the rxvlanpackets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RXWDOGPIS : 1;                 /**< MMC Receive Watchdog Error Packet Counter Interrupt Status This bit is set when the rxwatchdog error counter reaches half of the maximum value or the maximum value. */
        UINT32 RXRCVERRPIS : 1;                 /**< MMC Receive Error Packet Counter Interrupt Status This bit is set when the rxrcverror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXCTRLPIS : 1;                 /**< MMC Receive Control Packet Counter Interrupt Status This bit is set when the rxctrlpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXLPIUSCIS : 1;                 /**< MMC Receive LPI microsecond counter interrupt status This bit is set when the Rx_LPI_USEC_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 RXLPITRCIS : 1;                 /**< MMC Receive LPI transition counter interrupt status This bit is set when the Rx_LPI_Tran_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused28 : 4;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_t;

/**< Register MAC_MMC_RX_INTERRUPT - MMC Rx Interrupt */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT        0x704
/**< Field RXGBPKTIS - MMC Receive Good Bad Packet Counter Interrupt Status This bit is set when the rxpacketcount_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXGBPKTIS_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXGBPKTIS_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXGBPKTIS(v) (((v) << 0) & 0x1)
/**< Field RXGBOCTIS - MMC Receive Good Bad Octet Counter Interrupt Status This bit is set when the rxoctetcount_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXGBOCTIS_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXGBOCTIS_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXGBOCTIS(v) (((v) << 1) & 0x2)
/**< Field RXGOCTIS - MMC Receive Good Octet Counter Interrupt Status This bit is set when the rxoctetcount_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXGOCTIS_SHIFT 2
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXGOCTIS_MASK 0x4
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXGOCTIS(v) (((v) << 2) & 0x4)
/**< Field RXBCGPIS - MMC Receive Broadcast Good Packet Counter Interrupt Status This bit is set when the rxbroadcastpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXBCGPIS_SHIFT 3
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXBCGPIS_MASK 0x8
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXBCGPIS(v) (((v) << 3) & 0x8)
/**< Field RXMCGPIS - MMC Receive Multicast Good Packet Counter Interrupt Status This bit is set when the rxmulticastpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXMCGPIS_SHIFT 4
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXMCGPIS_MASK 0x10
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXMCGPIS(v) (((v) << 4) & 0x10)
/**< Field RXCRCERPIS - MMC Receive CRC Error Packet Counter Interrupt Status This bit is set when the rxcrcerror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXCRCERPIS_SHIFT 5
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXCRCERPIS_MASK 0x20
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXCRCERPIS(v) (((v) << 5) & 0x20)
/**< Field RXALGNERPIS - MMC Receive Alignment Error Packet Counter Interrupt Status This bit is set when the rxalignmenterror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXALGNERPIS_SHIFT 6
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXALGNERPIS_MASK 0x40
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXALGNERPIS(v) (((v) << 6) & 0x40)
/**< Field RXRUNTPIS - MMC Receive Runt Packet Counter Interrupt Status This bit is set when the rxrunterror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXRUNTPIS_SHIFT 7
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXRUNTPIS_MASK 0x80
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXRUNTPIS(v) (((v) << 7) & 0x80)
/**< Field RXJABERPIS - MMC Receive Jabber Error Packet Counter Interrupt Status This bit is set when the rxjabbererror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXJABERPIS_SHIFT 8
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXJABERPIS_MASK 0x100
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXJABERPIS(v) (((v) << 8) & 0x100)
/**< Field RXUSIZEGPIS - MMC Receive Undersize Good Packet Counter Interrupt Status This bit is set when the rxundersize_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXUSIZEGPIS_SHIFT 9
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXUSIZEGPIS_MASK 0x200
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXUSIZEGPIS(v) (((v) << 9) & 0x200)
/**< Field RXOSIZEGPIS - MMC Receive Oversize Good Packet Counter Interrupt Status This bit is set when the rxoversize_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXOSIZEGPIS_SHIFT 10
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXOSIZEGPIS_MASK 0x400
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXOSIZEGPIS(v) (((v) << 10) & 0x400)
/**< Field RX64OCTGBPIS - MMC Receive 64 Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx64octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX64OCTGBPIS_SHIFT 11
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX64OCTGBPIS_MASK 0x800
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX64OCTGBPIS(v) (((v) << 11) & 0x800)
/**< Field RX65T127OCTGBPIS - MMC Receive 65 to 127 Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx65to127octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX65T127OCTGBPIS_SHIFT 12
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX65T127OCTGBPIS_MASK 0x1000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX65T127OCTGBPIS(v) (((v) << 12) & 0x1000)
/**< Field RX128T255OCTGBPIS - MMC Receive 128 to 255 Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx128to255octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX128T255OCTGBPIS_SHIFT 13
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX128T255OCTGBPIS_MASK 0x2000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX128T255OCTGBPIS(v) (((v) << 13) & 0x2000)
/**< Field RX256T511OCTGBPIS - MMC Receive 256 to 511 Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx256to511octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX256T511OCTGBPIS_SHIFT 14
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX256T511OCTGBPIS_MASK 0x4000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX256T511OCTGBPIS(v) (((v) << 14) & 0x4000)
/**< Field RX512T1023OCTGBPIS - MMC Receive 512 to 1023 Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx512to1023octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX512T1023OCTGBPIS_SHIFT 15
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX512T1023OCTGBPIS_MASK 0x8000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX512T1023OCTGBPIS(v) (((v) << 15) & 0x8000)
/**< Field RX1024TMAXOCTGBPIS - MMC Receive 1024 to Maximum Octet Good Bad Packet Counter Interrupt Status This bit is set when the rx1024tomaxoctets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX1024TMAXOCTGBPIS_SHIFT 16
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX1024TMAXOCTGBPIS_MASK 0x10000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RX1024TMAXOCTGBPIS(v) (((v) << 16) & 0x10000)
/**< Field RXUCGPIS - MMC Receive Unicast Good Packet Counter Interrupt Status This bit is set when the rxunicastpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXUCGPIS_SHIFT 17
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXUCGPIS_MASK 0x20000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXUCGPIS(v) (((v) << 17) & 0x20000)
/**< Field RXLENERPIS - MMC Receive Length Error Packet Counter Interrupt Status This bit is set when the rxlengtherror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXLENERPIS_SHIFT 18
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXLENERPIS_MASK 0x40000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXLENERPIS(v) (((v) << 18) & 0x40000)
/**< Field RXORANGEPIS - MMC Receive Out Of Range Error Packet Counter Interrupt Status. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXORANGEPIS_SHIFT 19
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXORANGEPIS_MASK 0x80000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXORANGEPIS(v) (((v) << 19) & 0x80000)
/**< Field RXPAUSPIS - MMC Receive Pause Packet Counter Interrupt Status This bit is set when the rxpausepackets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXPAUSPIS_SHIFT 20
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXPAUSPIS_MASK 0x100000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXPAUSPIS(v) (((v) << 20) & 0x100000)
/**< Field RXFOVPIS - MMC Receive FIFO Overflow Packet Counter Interrupt Status This bit is set when the rxfifooverflow counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXFOVPIS_SHIFT 21
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXFOVPIS_MASK 0x200000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXFOVPIS(v) (((v) << 21) & 0x200000)
/**< Field RXVLANGBPIS - MMC Receive VLAN Good Bad Packet Counter Interrupt Status This bit is set when the rxvlanpackets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXVLANGBPIS_SHIFT 22
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXVLANGBPIS_MASK 0x400000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXVLANGBPIS(v) (((v) << 22) & 0x400000)
/**< Field RXWDOGPIS - MMC Receive Watchdog Error Packet Counter Interrupt Status This bit is set when the rxwatchdog error counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXWDOGPIS_SHIFT 23
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXWDOGPIS_MASK 0x800000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXWDOGPIS(v) (((v) << 23) & 0x800000)
/**< Field RXRCVERRPIS - MMC Receive Error Packet Counter Interrupt Status This bit is set when the rxrcverror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXRCVERRPIS_SHIFT 24
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXRCVERRPIS_MASK 0x1000000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXRCVERRPIS(v) (((v) << 24) & 0x1000000)
/**< Field RXCTRLPIS - MMC Receive Control Packet Counter Interrupt Status This bit is set when the rxctrlpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXCTRLPIS_SHIFT 25
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXCTRLPIS_MASK 0x2000000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXCTRLPIS(v) (((v) << 25) & 0x2000000)
/**< Field RXLPIUSCIS - MMC Receive LPI microsecond counter interrupt status This bit is set when the Rx_LPI_USEC_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXLPIUSCIS_SHIFT 26
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXLPIUSCIS_MASK 0x4000000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXLPIUSCIS(v) (((v) << 26) & 0x4000000)
/**< Field RXLPITRCIS - MMC Receive LPI transition counter interrupt status This bit is set when the Rx_LPI_Tran_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXLPITRCIS_SHIFT 27
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXLPITRCIS_MASK 0x8000000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_RXLPITRCIS(v) (((v) << 27) & 0x8000000)

/**< IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT - MMC Tx Interrupt */
typedef union IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_union_t {
    UINT32 R;
    struct {
        UINT32 TXGBOCTIS : 1;                 /**< MMC Transmit Good Bad Octet Counter Interrupt Status This bit is set when the txoctetcount_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXGBPKTIS : 1;                 /**< MMC Transmit Good Bad Packet Counter Interrupt Status This bit is set when the txpacketcount_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXBCGPIS : 1;                 /**< MMC Transmit Broadcast Good Packet Counter Interrupt Status This bit is set when the txbroadcastpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXMCGPIS : 1;                 /**< MMC Transmit Multicast Good Packet Counter Interrupt Status This bit is set when the txmulticastpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TX64OCTGBPIS : 1;                /**< MMC Transmit 64 Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx64octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TX65T127OCTGBPIS : 1;            /**< MMC Transmit 65 to 127 Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx65to127octets_gb counter reaches half the maximum value, and also when it reaches the maximum value. */
        UINT32 TX128T255OCTGBPIS : 1;           /**< MMC Transmit 128 to 255 Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx128to255octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TX256T511OCTGBPIS : 1;           /**< MMC Transmit 256 to 511 Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx256to511octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TX512T1023OCTGBPIS : 1;          /**< MMC Transmit 512 to 1023 Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx512to1023octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TX1024TMAXOCTGBPIS : 1;          /**< MMC Transmit 1024 to Maximum Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx1024tomaxoctets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXUCGBPIS : 1;                 /**< MMC Transmit Unicast Good Bad Packet Counter Interrupt Status This bit is set when the txunicastpackets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXMCGBPIS : 1;                 /**< MMC Transmit Multicast Good Bad Packet Counter Interrupt Status The bit is set when the txmulticastpackets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXBCGBPIS : 1;                 /**< MMC Transmit Broadcast Good Bad Packet Counter Interrupt Status This bit is set when the txbroadcastpackets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXUFLOWERPIS : 1;                /**< MMC Transmit Underflow Error Packet Counter Interrupt Status This bit is set when the txunderflowerror counter reaches half of the maximum value or the maximum value. */
        UINT32 TXSCOLGPIS : 1;                 /**< MMC Transmit Single Collision Good Packet Counter Interrupt Status This bit is set when the txsinglecol_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXMCOLGPIS : 1;                 /**< MMC Transmit Multiple Collision Good Packet Counter Interrupt Status This bit is set when the txmulticol_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXDEFPIS : 1;                 /**< MMC Transmit Deferred Packet Counter Interrupt Status This bit is set when the txdeferred counter reaches half of the maximum value or the maximum value. */
        UINT32 TXLATCOLPIS : 1;                 /**< MMC Transmit Late Collision Packet Counter Interrupt Status This bit is set when the txlatecol counter reaches half of the maximum value or the maximum value. */
        UINT32 TXEXCOLPIS : 1;                 /**< MMC Transmit Excessive Collision Packet Counter Interrupt Status This bit is set when the txexesscol counter reaches half of the maximum value or the maximum value. */
        UINT32 TXCARERPIS : 1;                 /**< MMC Transmit Carrier Error Packet Counter Interrupt Status This bit is set when the txcarriererror counter reaches half of the maximum value or the maximum value. */
        UINT32 TXGOCTIS : 1;                 /**< MMC Transmit Good Octet Counter Interrupt Status This bit is set when the txoctetcount_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXGPKTIS : 1;                 /**< MMC Transmit Good Packet Counter Interrupt Status This bit is set when the txpacketcount_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXEXDEFPIS : 1;                 /**< MMC Transmit Excessive Deferral Packet Counter Interrupt Status This bit is set when the txexcessdef counter reaches half of the maximum value or the maximum value. */
        UINT32 TXPAUSPIS : 1;                 /**< MMC Transmit Pause Packet Counter Interrupt Status This bit is set when the txpausepacketserror counter reaches half of the maximum value or the maximum value. */
        UINT32 TXVLANGPIS : 1;                 /**< MMC Transmit VLAN Good Packet Counter Interrupt Status This bit is set when the txvlanpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXOSIZEGPIS : 1;                 /**< MMC Transmit Oversize Good Packet Counter Interrupt Status This bit is set when the txoversize_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXLPIUSCIS : 1;                 /**< MMC Transmit LPI microsecond counter interrupt status This bit is set when the Tx_LPI_USEC_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 TXLPITRCIS : 1;                 /**< MMC Transmit LPI transition counter interrupt status This bit is set when the Tx_LPI_Tran_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused28 : 4;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_t;

/**< Register MAC_MMC_TX_INTERRUPT - MMC Tx Interrupt */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT        0x708
/**< Field TXGBOCTIS - MMC Transmit Good Bad Octet Counter Interrupt Status This bit is set when the txoctetcount_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGBOCTIS_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGBOCTIS_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGBOCTIS(v) (((v) << 0) & 0x1)
/**< Field TXGBPKTIS - MMC Transmit Good Bad Packet Counter Interrupt Status This bit is set when the txpacketcount_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGBPKTIS_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGBPKTIS_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGBPKTIS(v) (((v) << 1) & 0x2)
/**< Field TXBCGPIS - MMC Transmit Broadcast Good Packet Counter Interrupt Status This bit is set when the txbroadcastpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXBCGPIS_SHIFT 2
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXBCGPIS_MASK 0x4
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXBCGPIS(v) (((v) << 2) & 0x4)
/**< Field TXMCGPIS - MMC Transmit Multicast Good Packet Counter Interrupt Status This bit is set when the txmulticastpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXMCGPIS_SHIFT 3
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXMCGPIS_MASK 0x8
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXMCGPIS(v) (((v) << 3) & 0x8)
/**< Field TX64OCTGBPIS - MMC Transmit 64 Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx64octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX64OCTGBPIS_SHIFT 4
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX64OCTGBPIS_MASK 0x10
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX64OCTGBPIS(v) (((v) << 4) & 0x10)
/**< Field TX65T127OCTGBPIS - MMC Transmit 65 to 127 Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx65to127octets_gb counter reaches half the maximum value, and also when it reaches the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX65T127OCTGBPIS_SHIFT 5
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX65T127OCTGBPIS_MASK 0x20
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX65T127OCTGBPIS(v) (((v) << 5) & 0x20)
/**< Field TX128T255OCTGBPIS - MMC Transmit 128 to 255 Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx128to255octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX128T255OCTGBPIS_SHIFT 6
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX128T255OCTGBPIS_MASK 0x40
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX128T255OCTGBPIS(v) (((v) << 6) & 0x40)
/**< Field TX256T511OCTGBPIS - MMC Transmit 256 to 511 Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx256to511octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX256T511OCTGBPIS_SHIFT 7
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX256T511OCTGBPIS_MASK 0x80
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX256T511OCTGBPIS(v) (((v) << 7) & 0x80)
/**< Field TX512T1023OCTGBPIS - MMC Transmit 512 to 1023 Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx512to1023octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX512T1023OCTGBPIS_SHIFT 8
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX512T1023OCTGBPIS_MASK 0x100
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX512T1023OCTGBPIS(v) (((v) << 8) & 0x100)
/**< Field TX1024TMAXOCTGBPIS - MMC Transmit 1024 to Maximum Octet Good Bad Packet Counter Interrupt Status This bit is set when the tx1024tomaxoctets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX1024TMAXOCTGBPIS_SHIFT 9
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX1024TMAXOCTGBPIS_MASK 0x200
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TX1024TMAXOCTGBPIS(v) (((v) << 9) & 0x200)
/**< Field TXUCGBPIS - MMC Transmit Unicast Good Bad Packet Counter Interrupt Status This bit is set when the txunicastpackets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXUCGBPIS_SHIFT 10
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXUCGBPIS_MASK 0x400
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXUCGBPIS(v) (((v) << 10) & 0x400)
/**< Field TXMCGBPIS - MMC Transmit Multicast Good Bad Packet Counter Interrupt Status The bit is set when the txmulticastpackets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXMCGBPIS_SHIFT 11
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXMCGBPIS_MASK 0x800
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXMCGBPIS(v) (((v) << 11) & 0x800)
/**< Field TXBCGBPIS - MMC Transmit Broadcast Good Bad Packet Counter Interrupt Status This bit is set when the txbroadcastpackets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXBCGBPIS_SHIFT 12
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXBCGBPIS_MASK 0x1000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXBCGBPIS(v) (((v) << 12) & 0x1000)
/**< Field TXUFLOWERPIS - MMC Transmit Underflow Error Packet Counter Interrupt Status This bit is set when the txunderflowerror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXUFLOWERPIS_SHIFT 13
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXUFLOWERPIS_MASK 0x2000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXUFLOWERPIS(v) (((v) << 13) & 0x2000)
/**< Field TXSCOLGPIS - MMC Transmit Single Collision Good Packet Counter Interrupt Status This bit is set when the txsinglecol_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXSCOLGPIS_SHIFT 14
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXSCOLGPIS_MASK 0x4000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXSCOLGPIS(v) (((v) << 14) & 0x4000)
/**< Field TXMCOLGPIS - MMC Transmit Multiple Collision Good Packet Counter Interrupt Status This bit is set when the txmulticol_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXMCOLGPIS_SHIFT 15
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXMCOLGPIS_MASK 0x8000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXMCOLGPIS(v) (((v) << 15) & 0x8000)
/**< Field TXDEFPIS - MMC Transmit Deferred Packet Counter Interrupt Status This bit is set when the txdeferred counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXDEFPIS_SHIFT 16
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXDEFPIS_MASK 0x10000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXDEFPIS(v) (((v) << 16) & 0x10000)
/**< Field TXLATCOLPIS - MMC Transmit Late Collision Packet Counter Interrupt Status This bit is set when the txlatecol counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXLATCOLPIS_SHIFT 17
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXLATCOLPIS_MASK 0x20000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXLATCOLPIS(v) (((v) << 17) & 0x20000)
/**< Field TXEXCOLPIS - MMC Transmit Excessive Collision Packet Counter Interrupt Status This bit is set when the txexesscol counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXEXCOLPIS_SHIFT 18
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXEXCOLPIS_MASK 0x40000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXEXCOLPIS(v) (((v) << 18) & 0x40000)
/**< Field TXCARERPIS - MMC Transmit Carrier Error Packet Counter Interrupt Status This bit is set when the txcarriererror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXCARERPIS_SHIFT 19
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXCARERPIS_MASK 0x80000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXCARERPIS(v) (((v) << 19) & 0x80000)
/**< Field TXGOCTIS - MMC Transmit Good Octet Counter Interrupt Status This bit is set when the txoctetcount_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGOCTIS_SHIFT 20
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGOCTIS_MASK 0x100000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGOCTIS(v) (((v) << 20) & 0x100000)
/**< Field TXGPKTIS - MMC Transmit Good Packet Counter Interrupt Status This bit is set when the txpacketcount_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGPKTIS_SHIFT 21
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGPKTIS_MASK 0x200000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXGPKTIS(v) (((v) << 21) & 0x200000)
/**< Field TXEXDEFPIS - MMC Transmit Excessive Deferral Packet Counter Interrupt Status This bit is set when the txexcessdef counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXEXDEFPIS_SHIFT 22
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXEXDEFPIS_MASK 0x400000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXEXDEFPIS(v) (((v) << 22) & 0x400000)
/**< Field TXPAUSPIS - MMC Transmit Pause Packet Counter Interrupt Status This bit is set when the txpausepacketserror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXPAUSPIS_SHIFT 23
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXPAUSPIS_MASK 0x800000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXPAUSPIS(v) (((v) << 23) & 0x800000)
/**< Field TXVLANGPIS - MMC Transmit VLAN Good Packet Counter Interrupt Status This bit is set when the txvlanpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXVLANGPIS_SHIFT 24
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXVLANGPIS_MASK 0x1000000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXVLANGPIS(v) (((v) << 24) & 0x1000000)
/**< Field TXOSIZEGPIS - MMC Transmit Oversize Good Packet Counter Interrupt Status This bit is set when the txoversize_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXOSIZEGPIS_SHIFT 25
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXOSIZEGPIS_MASK 0x2000000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXOSIZEGPIS(v) (((v) << 25) & 0x2000000)
/**< Field TXLPIUSCIS - MMC Transmit LPI microsecond counter interrupt status This bit is set when the Tx_LPI_USEC_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXLPIUSCIS_SHIFT 26
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXLPIUSCIS_MASK 0x4000000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXLPIUSCIS(v) (((v) << 26) & 0x4000000)
/**< Field TXLPITRCIS - MMC Transmit LPI transition counter interrupt status This bit is set when the Tx_LPI_Tran_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXLPITRCIS_SHIFT 27
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXLPITRCIS_MASK 0x8000000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_TXLPITRCIS(v) (((v) << 27) & 0x8000000)

/**< IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK - MMC Rx Interrupt Mask */
typedef union IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_union_t {
    UINT32 R;
    struct {
        UINT32 RXGBPKTIM : 1;                 /**< MMC Receive Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxpacketcount_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RXGBOCTIM : 1;                 /**< MMC Receive Good Bad Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxoctetcount_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RXGOCTIM : 1;                 /**< MMC Receive Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxoctetcount_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXBCGPIM : 1;                 /**< MMC Receive Broadcast Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxbroadcastpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXMCGPIM : 1;                 /**< MMC Receive Multicast Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxmulticastpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXCRCERPIM : 1;                 /**< MMC Receive CRC Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxcrcerror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXALGNERPIM : 1;                 /**< MMC Receive Alignment Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxalignmenterror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXRUNTPIM : 1;                 /**< MMC Receive Runt Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxrunterror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXJABERPIM : 1;                 /**< MMC Receive Jabber Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxjabbererror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUSIZEGPIM : 1;                 /**< MMC Receive Undersize Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxundersize_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXOSIZEGPIM : 1;                 /**< MMC Receive Oversize Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxoversize_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RX64OCTGBPIM : 1;                /**< MMC Receive 64 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rx64octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RX65T127OCTGBPIM : 1;            /**< MMC Receive 65 to 127 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rx65to127octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RX128T255OCTGBPIM : 1;           /**< MMC Receive 128 to 255 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rx128to255octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RX256T511OCTGBPIM : 1;           /**< MMC Receive 256 to 511 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rx256to511octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RX512T1023OCTGBPIM : 1;          /**< MMC Receive 512 to 1023 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rx512to1023octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RX1024TMAXOCTGBPIM : 1;          /**< MMC Receive 1024 to Maximum Octet Good Bad Packet Counter Interrupt Mask. */
        UINT32 RXUCGPIM : 1;                 /**< MMC Receive Unicast Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxunicastpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXLENERPIM : 1;                 /**< MMC Receive Length Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxlengtherror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXORANGEPIM : 1;                 /**< MMC Receive Out Of Range Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxoutofrangetype counter reaches half of the maximum value or the maximum value. */
        UINT32 RXPAUSPIM : 1;                 /**< MMC Receive Pause Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxpausepackets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXFOVPIM : 1;                 /**< MMC Receive FIFO Overflow Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxfifooverflow counter reaches half of the maximum value or the maximum value. */
        UINT32 RXVLANGBPIM : 1;                 /**< MMC Receive VLAN Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxvlanpackets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 RXWDOGPIM : 1;                 /**< MMC Receive Watchdog Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxwatchdog counter reaches half of the maximum value or the maximum value. */
        UINT32 RXRCVERRPIM : 1;                 /**< MMC Receive Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxrcverror counter reaches half of the maximum value or the maximum value. */
        UINT32 RXCTRLPIM : 1;                 /**< MMC Receive Control Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxctrlpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 RXLPIUSCIM : 1;                 /**< MMC Receive LPI microsecond counter interrupt Mask Setting this bit masks the interrupt when the Rx_LPI_USEC_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 RXLPITRCIM : 1;                 /**< MMC Receive LPI transition counter interrupt Mask Setting this bit masks the interrupt when the Rx_LPI_Tran_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused28 : 4;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_t;

/**< Register MAC_MMC_RX_INTERRUPT_MASK - MMC Rx Interrupt Mask */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK   0x70C
/**< Field RXGBPKTIM - MMC Receive Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxpacketcount_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXGBPKTIM_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXGBPKTIM_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXGBPKTIM(v) (((v) << 0) & 0x1)
/**< Field RXGBOCTIM - MMC Receive Good Bad Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxoctetcount_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXGBOCTIM_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXGBOCTIM_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXGBOCTIM(v) (((v) << 1) & 0x2)
/**< Field RXGOCTIM - MMC Receive Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxoctetcount_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXGOCTIM_SHIFT 2
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXGOCTIM_MASK 0x4
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXGOCTIM(v) (((v) << 2) & 0x4)
/**< Field RXBCGPIM - MMC Receive Broadcast Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxbroadcastpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXBCGPIM_SHIFT 3
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXBCGPIM_MASK 0x8
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXBCGPIM(v) (((v) << 3) & 0x8)
/**< Field RXMCGPIM - MMC Receive Multicast Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxmulticastpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXMCGPIM_SHIFT 4
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXMCGPIM_MASK 0x10
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXMCGPIM(v) (((v) << 4) & 0x10)
/**< Field RXCRCERPIM - MMC Receive CRC Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxcrcerror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXCRCERPIM_SHIFT 5
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXCRCERPIM_MASK 0x20
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXCRCERPIM(v) (((v) << 5) & 0x20)
/**< Field RXALGNERPIM - MMC Receive Alignment Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxalignmenterror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXALGNERPIM_SHIFT 6
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXALGNERPIM_MASK 0x40
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXALGNERPIM(v) (((v) << 6) & 0x40)
/**< Field RXRUNTPIM - MMC Receive Runt Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxrunterror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXRUNTPIM_SHIFT 7
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXRUNTPIM_MASK 0x80
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXRUNTPIM(v) (((v) << 7) & 0x80)
/**< Field RXJABERPIM - MMC Receive Jabber Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxjabbererror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXJABERPIM_SHIFT 8
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXJABERPIM_MASK 0x100
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXJABERPIM(v) (((v) << 8) & 0x100)
/**< Field RXUSIZEGPIM - MMC Receive Undersize Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxundersize_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXUSIZEGPIM_SHIFT 9
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXUSIZEGPIM_MASK 0x200
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXUSIZEGPIM(v) (((v) << 9) & 0x200)
/**< Field RXOSIZEGPIM - MMC Receive Oversize Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxoversize_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXOSIZEGPIM_SHIFT 10
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXOSIZEGPIM_MASK 0x400
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXOSIZEGPIM(v) (((v) << 10) & 0x400)
/**< Field RX64OCTGBPIM - MMC Receive 64 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rx64octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX64OCTGBPIM_SHIFT 11
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX64OCTGBPIM_MASK 0x800
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX64OCTGBPIM(v) (((v) << 11) & 0x800)
/**< Field RX65T127OCTGBPIM - MMC Receive 65 to 127 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rx65to127octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX65T127OCTGBPIM_SHIFT 12
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX65T127OCTGBPIM_MASK 0x1000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX65T127OCTGBPIM(v) (((v) << 12) & 0x1000)
/**< Field RX128T255OCTGBPIM - MMC Receive 128 to 255 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rx128to255octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX128T255OCTGBPIM_SHIFT 13
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX128T255OCTGBPIM_MASK 0x2000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX128T255OCTGBPIM(v) (((v) << 13) & 0x2000)
/**< Field RX256T511OCTGBPIM - MMC Receive 256 to 511 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rx256to511octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX256T511OCTGBPIM_SHIFT 14
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX256T511OCTGBPIM_MASK 0x4000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX256T511OCTGBPIM(v) (((v) << 14) & 0x4000)
/**< Field RX512T1023OCTGBPIM - MMC Receive 512 to 1023 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rx512to1023octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX512T1023OCTGBPIM_SHIFT 15
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX512T1023OCTGBPIM_MASK 0x8000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX512T1023OCTGBPIM(v) (((v) << 15) & 0x8000)
/**< Field RX1024TMAXOCTGBPIM - MMC Receive 1024 to Maximum Octet Good Bad Packet Counter Interrupt Mask. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX1024TMAXOCTGBPIM_SHIFT 16
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX1024TMAXOCTGBPIM_MASK 0x10000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RX1024TMAXOCTGBPIM(v) (((v) << 16) & 0x10000)
/**< Field RXUCGPIM - MMC Receive Unicast Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxunicastpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXUCGPIM_SHIFT 17
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXUCGPIM_MASK 0x20000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXUCGPIM(v) (((v) << 17) & 0x20000)
/**< Field RXLENERPIM - MMC Receive Length Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxlengtherror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXLENERPIM_SHIFT 18
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXLENERPIM_MASK 0x40000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXLENERPIM(v) (((v) << 18) & 0x40000)
/**< Field RXORANGEPIM - MMC Receive Out Of Range Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxoutofrangetype counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXORANGEPIM_SHIFT 19
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXORANGEPIM_MASK 0x80000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXORANGEPIM(v) (((v) << 19) & 0x80000)
/**< Field RXPAUSPIM - MMC Receive Pause Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxpausepackets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXPAUSPIM_SHIFT 20
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXPAUSPIM_MASK 0x100000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXPAUSPIM(v) (((v) << 20) & 0x100000)
/**< Field RXFOVPIM - MMC Receive FIFO Overflow Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxfifooverflow counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXFOVPIM_SHIFT 21
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXFOVPIM_MASK 0x200000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXFOVPIM(v) (((v) << 21) & 0x200000)
/**< Field RXVLANGBPIM - MMC Receive VLAN Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxvlanpackets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXVLANGBPIM_SHIFT 22
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXVLANGBPIM_MASK 0x400000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXVLANGBPIM(v) (((v) << 22) & 0x400000)
/**< Field RXWDOGPIM - MMC Receive Watchdog Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxwatchdog counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXWDOGPIM_SHIFT 23
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXWDOGPIM_MASK 0x800000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXWDOGPIM(v) (((v) << 23) & 0x800000)
/**< Field RXRCVERRPIM - MMC Receive Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxrcverror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXRCVERRPIM_SHIFT 24
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXRCVERRPIM_MASK 0x1000000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXRCVERRPIM(v) (((v) << 24) & 0x1000000)
/**< Field RXCTRLPIM - MMC Receive Control Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxctrlpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXCTRLPIM_SHIFT 25
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXCTRLPIM_MASK 0x2000000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXCTRLPIM(v) (((v) << 25) & 0x2000000)
/**< Field RXLPIUSCIM - MMC Receive LPI microsecond counter interrupt Mask Setting this bit masks the interrupt when the Rx_LPI_USEC_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXLPIUSCIM_SHIFT 26
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXLPIUSCIM_MASK 0x4000000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXLPIUSCIM(v) (((v) << 26) & 0x4000000)
/**< Field RXLPITRCIM - MMC Receive LPI transition counter interrupt Mask Setting this bit masks the interrupt when the Rx_LPI_Tran_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXLPITRCIM_SHIFT 27
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXLPITRCIM_MASK 0x8000000
#define IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_RXLPITRCIM(v) (((v) << 27) & 0x8000000)

/**< IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK - MMC Tx Interrupt Mask */
typedef union IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_union_t {
    UINT32 R;
    struct {
        UINT32 TXGBOCTIM : 1;                 /**< MMC Transmit Good Bad Octet Counter Interrupt Mask Setting this bit masks the interrupt when the txoctetcount_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXGBPKTIM : 1;                 /**< MMC Transmit Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txpacketcount_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXBCGPIM : 1;                 /**< MMC Transmit Broadcast Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txbroadcastpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXMCGPIM : 1;                 /**< MMC Transmit Multicast Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txmulticastpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TX64OCTGBPIM : 1;                /**< MMC Transmit 64 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx64octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TX65T127OCTGBPIM : 1;            /**< MMC Transmit 65 to 127 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx65to127octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TX128T255OCTGBPIM : 1;           /**< MMC Transmit 128 to 255 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx128to255octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TX256T511OCTGBPIM : 1;           /**< MMC Transmit 256 to 511 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx256to511octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TX512T1023OCTGBPIM : 1;          /**< MMC Transmit 512 to 1023 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx512to1023octets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TX1024TMAXOCTGBPIM : 1;          /**< MMC Transmit 1024 to Maximum Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx1024tomaxoctets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXUCGBPIM : 1;                 /**< MMC Transmit Unicast Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txunicastpackets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXMCGBPIM : 1;                 /**< MMC Transmit Multicast Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txmulticastpackets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXBCGBPIM : 1;                 /**< MMC Transmit Broadcast Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txbroadcastpackets_gb counter reaches half of the maximum value or the maximum value. */
        UINT32 TXUFLOWERPIM : 1;                /**< MMC Transmit Underflow Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txunderflowerror counter reaches half of the maximum value or the maximum value. */
        UINT32 TXSCOLGPIM : 1;                 /**< MMC Transmit Single Collision Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txsinglecol_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXMCOLGPIM : 1;                 /**< MMC Transmit Multiple Collision Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txmulticol_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXDEFPIM : 1;                 /**< MMC Transmit Deferred Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txdeferred counter reaches half of the maximum value or the maximum value. */
        UINT32 TXLATCOLPIM : 1;                 /**< MMC Transmit Late Collision Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txlatecol counter reaches half of the maximum value or the maximum value. */
        UINT32 TXEXCOLPIM : 1;                 /**< MMC Transmit Excessive Collision Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txexcesscol counter reaches half of the maximum value or the maximum value. */
        UINT32 TXCARERPIM : 1;                 /**< MMC Transmit Carrier Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txcarriererror counter reaches half of the maximum value or the maximum value. */
        UINT32 TXGOCTIM : 1;                 /**< MMC Transmit Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the txoctetcount_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXGPKTIM : 1;                 /**< MMC Transmit Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txpacketcount_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXEXDEFPIM : 1;                 /**< MMC Transmit Excessive Deferral Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txexcessdef counter reaches half of the maximum value or the maximum value. */
        UINT32 TXPAUSPIM : 1;                 /**< MMC Transmit Pause Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txpausepackets counter reaches half of the maximum value or the maximum value. */
        UINT32 TXVLANGPIM : 1;                 /**< MMC Transmit VLAN Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txvlanpackets_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXOSIZEGPIM : 1;                 /**< MMC Transmit Oversize Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txoversize_g counter reaches half of the maximum value or the maximum value. */
        UINT32 TXLPIUSCIM : 1;                 /**< MMC Transmit LPI microsecond counter interrupt Mask Setting this bit masks the interrupt when the Tx_LPI_USEC_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 TXLPITRCIM : 1;                 /**< MMC Transmit LPI transition counter interrupt Mask Setting this bit masks the interrupt when the Tx_LPI_Tran_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused28 : 4;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_t;

/**< Register MAC_MMC_TX_INTERRUPT_MASK - MMC Tx Interrupt Mask */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK   0x710
/**< Field TXGBOCTIM - MMC Transmit Good Bad Octet Counter Interrupt Mask Setting this bit masks the interrupt when the txoctetcount_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGBOCTIM_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGBOCTIM_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGBOCTIM(v) (((v) << 0) & 0x1)
/**< Field TXGBPKTIM - MMC Transmit Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txpacketcount_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGBPKTIM_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGBPKTIM_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGBPKTIM(v) (((v) << 1) & 0x2)
/**< Field TXBCGPIM - MMC Transmit Broadcast Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txbroadcastpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXBCGPIM_SHIFT 2
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXBCGPIM_MASK 0x4
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXBCGPIM(v) (((v) << 2) & 0x4)
/**< Field TXMCGPIM - MMC Transmit Multicast Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txmulticastpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXMCGPIM_SHIFT 3
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXMCGPIM_MASK 0x8
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXMCGPIM(v) (((v) << 3) & 0x8)
/**< Field TX64OCTGBPIM - MMC Transmit 64 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx64octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX64OCTGBPIM_SHIFT 4
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX64OCTGBPIM_MASK 0x10
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX64OCTGBPIM(v) (((v) << 4) & 0x10)
/**< Field TX65T127OCTGBPIM - MMC Transmit 65 to 127 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx65to127octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX65T127OCTGBPIM_SHIFT 5
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX65T127OCTGBPIM_MASK 0x20
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX65T127OCTGBPIM(v) (((v) << 5) & 0x20)
/**< Field TX128T255OCTGBPIM - MMC Transmit 128 to 255 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx128to255octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX128T255OCTGBPIM_SHIFT 6
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX128T255OCTGBPIM_MASK 0x40
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX128T255OCTGBPIM(v) (((v) << 6) & 0x40)
/**< Field TX256T511OCTGBPIM - MMC Transmit 256 to 511 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx256to511octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX256T511OCTGBPIM_SHIFT 7
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX256T511OCTGBPIM_MASK 0x80
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX256T511OCTGBPIM(v) (((v) << 7) & 0x80)
/**< Field TX512T1023OCTGBPIM - MMC Transmit 512 to 1023 Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx512to1023octets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX512T1023OCTGBPIM_SHIFT 8
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX512T1023OCTGBPIM_MASK 0x100
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX512T1023OCTGBPIM(v) (((v) << 8) & 0x100)
/**< Field TX1024TMAXOCTGBPIM - MMC Transmit 1024 to Maximum Octet Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the tx1024tomaxoctets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX1024TMAXOCTGBPIM_SHIFT 9
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX1024TMAXOCTGBPIM_MASK 0x200
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TX1024TMAXOCTGBPIM(v) (((v) << 9) & 0x200)
/**< Field TXUCGBPIM - MMC Transmit Unicast Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txunicastpackets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXUCGBPIM_SHIFT 10
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXUCGBPIM_MASK 0x400
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXUCGBPIM(v) (((v) << 10) & 0x400)
/**< Field TXMCGBPIM - MMC Transmit Multicast Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txmulticastpackets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXMCGBPIM_SHIFT 11
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXMCGBPIM_MASK 0x800
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXMCGBPIM(v) (((v) << 11) & 0x800)
/**< Field TXBCGBPIM - MMC Transmit Broadcast Good Bad Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txbroadcastpackets_gb counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXBCGBPIM_SHIFT 12
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXBCGBPIM_MASK 0x1000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXBCGBPIM(v) (((v) << 12) & 0x1000)
/**< Field TXUFLOWERPIM - MMC Transmit Underflow Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txunderflowerror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXUFLOWERPIM_SHIFT 13
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXUFLOWERPIM_MASK 0x2000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXUFLOWERPIM(v) (((v) << 13) & 0x2000)
/**< Field TXSCOLGPIM - MMC Transmit Single Collision Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txsinglecol_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXSCOLGPIM_SHIFT 14
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXSCOLGPIM_MASK 0x4000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXSCOLGPIM(v) (((v) << 14) & 0x4000)
/**< Field TXMCOLGPIM - MMC Transmit Multiple Collision Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txmulticol_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXMCOLGPIM_SHIFT 15
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXMCOLGPIM_MASK 0x8000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXMCOLGPIM(v) (((v) << 15) & 0x8000)
/**< Field TXDEFPIM - MMC Transmit Deferred Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txdeferred counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXDEFPIM_SHIFT 16
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXDEFPIM_MASK 0x10000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXDEFPIM(v) (((v) << 16) & 0x10000)
/**< Field TXLATCOLPIM - MMC Transmit Late Collision Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txlatecol counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXLATCOLPIM_SHIFT 17
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXLATCOLPIM_MASK 0x20000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXLATCOLPIM(v) (((v) << 17) & 0x20000)
/**< Field TXEXCOLPIM - MMC Transmit Excessive Collision Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txexcesscol counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXEXCOLPIM_SHIFT 18
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXEXCOLPIM_MASK 0x40000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXEXCOLPIM(v) (((v) << 18) & 0x40000)
/**< Field TXCARERPIM - MMC Transmit Carrier Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txcarriererror counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXCARERPIM_SHIFT 19
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXCARERPIM_MASK 0x80000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXCARERPIM(v) (((v) << 19) & 0x80000)
/**< Field TXGOCTIM - MMC Transmit Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the txoctetcount_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGOCTIM_SHIFT 20
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGOCTIM_MASK 0x100000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGOCTIM(v) (((v) << 20) & 0x100000)
/**< Field TXGPKTIM - MMC Transmit Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txpacketcount_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGPKTIM_SHIFT 21
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGPKTIM_MASK 0x200000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXGPKTIM(v) (((v) << 21) & 0x200000)
/**< Field TXEXDEFPIM - MMC Transmit Excessive Deferral Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txexcessdef counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXEXDEFPIM_SHIFT 22
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXEXDEFPIM_MASK 0x400000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXEXDEFPIM(v) (((v) << 22) & 0x400000)
/**< Field TXPAUSPIM - MMC Transmit Pause Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txpausepackets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXPAUSPIM_SHIFT 23
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXPAUSPIM_MASK 0x800000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXPAUSPIM(v) (((v) << 23) & 0x800000)
/**< Field TXVLANGPIM - MMC Transmit VLAN Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txvlanpackets_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXVLANGPIM_SHIFT 24
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXVLANGPIM_MASK 0x1000000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXVLANGPIM(v) (((v) << 24) & 0x1000000)
/**< Field TXOSIZEGPIM - MMC Transmit Oversize Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the txoversize_g counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXOSIZEGPIM_SHIFT 25
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXOSIZEGPIM_MASK 0x2000000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXOSIZEGPIM(v) (((v) << 25) & 0x2000000)
/**< Field TXLPIUSCIM - MMC Transmit LPI microsecond counter interrupt Mask Setting this bit masks the interrupt when the Tx_LPI_USEC_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXLPIUSCIM_SHIFT 26
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXLPIUSCIM_MASK 0x4000000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXLPIUSCIM(v) (((v) << 26) & 0x4000000)
/**< Field TXLPITRCIM - MMC Transmit LPI transition counter interrupt Mask Setting this bit masks the interrupt when the Tx_LPI_Tran_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXLPITRCIM_SHIFT 27
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXLPITRCIM_MASK 0x8000000
#define IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_TXLPITRCIM(v) (((v) << 27) & 0x8000000)

/**< IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_BAD - Tx Octet Count Good and Bad */
typedef union IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TXOCTGB : 32;                /**< Tx Octet Count Good Bad This field indicates the number of bytes transmitted, exclusive of preamble and retried bytes, in good and bad packets. */
    } B;
} IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_BAD_t;

/**< Register MAC_TX_OCTET_COUNT_GOOD_BAD - Tx Octet Count Good and Bad */
#define IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_BAD 0x714
/**< Field TXOCTGB - Tx Octet Count Good Bad This field indicates the number of bytes transmitted, exclusive of preamble and retried bytes, in good and bad packets. */
#define IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_BAD_TXOCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_BAD_TXOCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_BAD_TXOCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_BAD - Tx Packet Count Good and Bad */
typedef union IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TXPKTGB : 32;                /**< Tx Packet Count Good Bad This field indicates the number of good and bad packets transmitted, exclusive of retried packets. */
    } B;
} IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_BAD_t;

/**< Register MAC_TX_PACKET_COUNT_GOOD_BAD - Tx Packet Count Good and Bad */
#define IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_BAD 0x718
/**< Field TXPKTGB - Tx Packet Count Good Bad This field indicates the number of good and bad packets transmitted, exclusive of retried packets. */
#define IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_BAD_TXPKTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_BAD_TXPKTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_BAD_TXPKTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD - Tx Broadcast Packets Good */
typedef union IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 TXBCASTG : 32;                /**< Tx Broadcast Packets Good This field indicates the number of good broadcast packets transmitted. */
    } B;
} IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_t;

/**< Register MAC_TX_BROADCAST_PACKETS_GOOD - Tx Broadcast Packets Good */
#define IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD 0x71C
/**< Field TXBCASTG - Tx Broadcast Packets Good This field indicates the number of good broadcast packets transmitted. */
#define IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_TXBCASTG_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_TXBCASTG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_TXBCASTG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD - Tx Multicast Packets Good */
typedef union IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 TXMCASTG : 32;                /**< Tx Multicast Packets Good This field indicates the number of good multicast packets transmitted. */
    } B;
} IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_t;

/**< Register MAC_TX_MULTICAST_PACKETS_GOOD - Tx Multicast Packets Good */
#define IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD 0x720
/**< Field TXMCASTG - Tx Multicast Packets Good This field indicates the number of good multicast packets transmitted. */
#define IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_TXMCASTG_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_TXMCASTG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_TXMCASTG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_64OCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 64-Byte Packets */
typedef union IMX_ENET_QOS_MAC_TX_64OCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TX64OCTGB : 32;                /**< Tx 64Octets Packets Good_Bad This field indicates the number of good and bad packets transmitted with length 64 bytes, exclusive of preamble and retried packets. */
    } B;
} IMX_ENET_QOS_MAC_TX_64OCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_TX_64OCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 64-Byte Packets */
#define IMX_ENET_QOS_MAC_TX_64OCTETS_PACKETS_GOOD_BAD 0x724
/**< Field TX64OCTGB - Tx 64Octets Packets Good_Bad This field indicates the number of good and bad packets transmitted with length 64 bytes, exclusive of preamble and retried packets. */
#define IMX_ENET_QOS_MAC_TX_64OCTETS_PACKETS_GOOD_BAD_TX64OCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_64OCTETS_PACKETS_GOOD_BAD_TX64OCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_64OCTETS_PACKETS_GOOD_BAD_TX64OCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_65TO127OCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 65 to 127-Byte Packets */
typedef union IMX_ENET_QOS_MAC_TX_65TO127OCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TX65_127OCTGB : 32;              /**< Tx 65To127Octets Packets Good Bad This field indicates the number of good and bad packets transmitted with length between 65 and 127 (inclusive) bytes, exclusive of preamble and retried packets. */
    } B;
} IMX_ENET_QOS_MAC_TX_65TO127OCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_TX_65TO127OCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 65 to 127-Byte Packets */
#define IMX_ENET_QOS_MAC_TX_65TO127OCTETS_PACKETS_GOOD_BAD 0x728
/**< Field TX65_127OCTGB - Tx 65To127Octets Packets Good Bad This field indicates the number of good and bad packets transmitted with length between 65 and 127 (inclusive) bytes, exclusive of preamble and retried packets. */
#define IMX_ENET_QOS_MAC_TX_65TO127OCTETS_PACKETS_GOOD_BAD_TX65_127OCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_65TO127OCTETS_PACKETS_GOOD_BAD_TX65_127OCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_65TO127OCTETS_PACKETS_GOOD_BAD_TX65_127OCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_128TO255OCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 128 to 255-Byte Packets */
typedef union IMX_ENET_QOS_MAC_TX_128TO255OCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TX128_255OCTGB : 32;             /**< Tx 128To255Octets Packets Good Bad This field indicates the number of good and bad packets transmitted with length between 128 and 255 (inclusive) bytes, exclusive of preamble and retried packets. */
    } B;
} IMX_ENET_QOS_MAC_TX_128TO255OCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_TX_128TO255OCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 128 to 255-Byte Packets */
#define IMX_ENET_QOS_MAC_TX_128TO255OCTETS_PACKETS_GOOD_BAD 0x72C
/**< Field TX128_255OCTGB - Tx 128To255Octets Packets Good Bad This field indicates the number of good and bad packets transmitted with length between 128 and 255 (inclusive) bytes, exclusive of preamble and retried packets. */
#define IMX_ENET_QOS_MAC_TX_128TO255OCTETS_PACKETS_GOOD_BAD_TX128_255OCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_128TO255OCTETS_PACKETS_GOOD_BAD_TX128_255OCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_128TO255OCTETS_PACKETS_GOOD_BAD_TX128_255OCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_256TO511OCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 256 to 511-Byte Packets */
typedef union IMX_ENET_QOS_MAC_TX_256TO511OCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TX256_511OCTGB : 32;             /**< Tx 256To511Octets Packets Good Bad This field indicates the number of good and bad packets transmitted with length between 256 and 511 (inclusive) bytes, exclusive of preamble and retried packets. */
    } B;
} IMX_ENET_QOS_MAC_TX_256TO511OCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_TX_256TO511OCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 256 to 511-Byte Packets */
#define IMX_ENET_QOS_MAC_TX_256TO511OCTETS_PACKETS_GOOD_BAD 0x730
/**< Field TX256_511OCTGB - Tx 256To511Octets Packets Good Bad This field indicates the number of good and bad packets transmitted with length between 256 and 511 (inclusive) bytes, exclusive of preamble and retried packets. */
#define IMX_ENET_QOS_MAC_TX_256TO511OCTETS_PACKETS_GOOD_BAD_TX256_511OCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_256TO511OCTETS_PACKETS_GOOD_BAD_TX256_511OCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_256TO511OCTETS_PACKETS_GOOD_BAD_TX256_511OCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_512TO1023OCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 512 to 1023-Byte Packets */
typedef union IMX_ENET_QOS_MAC_TX_512TO1023OCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TX512_1023OCTGB : 32;            /**< Tx 512To1023Octets Packets Good Bad This field indicates the number of good and bad packets transmitted with length between 512 and 1023 (inclusive) bytes, exclusive of preamble and retried packets. */
    } B;
} IMX_ENET_QOS_MAC_TX_512TO1023OCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_TX_512TO1023OCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 512 to 1023-Byte Packets */
#define IMX_ENET_QOS_MAC_TX_512TO1023OCTETS_PACKETS_GOOD_BAD 0x734
/**< Field TX512_1023OCTGB - Tx 512To1023Octets Packets Good Bad This field indicates the number of good and bad packets transmitted with length between 512 and 1023 (inclusive) bytes, exclusive of preamble and retried packets. */
#define IMX_ENET_QOS_MAC_TX_512TO1023OCTETS_PACKETS_GOOD_BAD_TX512_1023OCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_512TO1023OCTETS_PACKETS_GOOD_BAD_TX512_1023OCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_512TO1023OCTETS_PACKETS_GOOD_BAD_TX512_1023OCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 1024 to Max-Byte Packets */
typedef union IMX_ENET_QOS_MAC_TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TX1024_MAXOCTGB : 32;            /**< Tx 1024ToMaxOctets Packets Good Bad This field indicates the number of good and bad packets transmitted with length between 1024 and maxsize (inclusive) bytes, exclusive of preamble and retried packets. */
    } B;
} IMX_ENET_QOS_MAC_TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD - Tx Good and Bad 1024 to Max-Byte Packets */
#define IMX_ENET_QOS_MAC_TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD 0x738
/**< Field TX1024_MAXOCTGB - Tx 1024ToMaxOctets Packets Good Bad This field indicates the number of good and bad packets transmitted with length between 1024 and maxsize (inclusive) bytes, exclusive of preamble and retried packets. */
#define IMX_ENET_QOS_MAC_TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_TX1024_MAXOCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_TX1024_MAXOCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_TX1024_MAXOCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_UNICAST_PACKETS_GOOD_BAD - Good and Bad Unicast Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_UNICAST_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TXUCASTGB : 32;                /**< Tx Unicast Packets Good Bad This field indicates the number of good and bad unicast packets transmitted. */
    } B;
} IMX_ENET_QOS_MAC_TX_UNICAST_PACKETS_GOOD_BAD_t;

/**< Register MAC_TX_UNICAST_PACKETS_GOOD_BAD - Good and Bad Unicast Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_UNICAST_PACKETS_GOOD_BAD 0x73C
/**< Field TXUCASTGB - Tx Unicast Packets Good Bad This field indicates the number of good and bad unicast packets transmitted. */
#define IMX_ENET_QOS_MAC_TX_UNICAST_PACKETS_GOOD_BAD_TXUCASTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_UNICAST_PACKETS_GOOD_BAD_TXUCASTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_UNICAST_PACKETS_GOOD_BAD_TXUCASTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_BAD - Good and Bad Multicast Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TXMCASTGB : 32;                /**< Tx Multicast Packets Good Bad This field indicates the number of good and bad multicast packets transmitted. */
    } B;
} IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_BAD_t;

/**< Register MAC_TX_MULTICAST_PACKETS_GOOD_BAD - Good and Bad Multicast Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_BAD 0x740
/**< Field TXMCASTGB - Tx Multicast Packets Good Bad This field indicates the number of good and bad multicast packets transmitted. */
#define IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_BAD_TXMCASTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_BAD_TXMCASTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_BAD_TXMCASTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_BAD - Good and Bad Broadcast Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 TXBCASTGB : 32;                /**< Tx Broadcast Packets Good Bad This field indicates the number of good and bad broadcast packets transmitted. */
    } B;
} IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_BAD_t;

/**< Register MAC_TX_BROADCAST_PACKETS_GOOD_BAD - Good and Bad Broadcast Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_BAD 0x744
/**< Field TXBCASTGB - Tx Broadcast Packets Good Bad This field indicates the number of good and bad broadcast packets transmitted. */
#define IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_BAD_TXBCASTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_BAD_TXBCASTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_BAD_TXBCASTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_UNDERFLOW_ERROR_PACKETS - Tx Packets Aborted By Underflow Error */
typedef union IMX_ENET_QOS_MAC_TX_UNDERFLOW_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 TXUNDRFLW : 32;                /**< Tx Underflow Error Packets This field indicates the number of packets aborted because of packets underflow error. */
    } B;
} IMX_ENET_QOS_MAC_TX_UNDERFLOW_ERROR_PACKETS_t;

/**< Register MAC_TX_UNDERFLOW_ERROR_PACKETS - Tx Packets Aborted By Underflow Error */
#define IMX_ENET_QOS_MAC_TX_UNDERFLOW_ERROR_PACKETS 0x748
/**< Field TXUNDRFLW - Tx Underflow Error Packets This field indicates the number of packets aborted because of packets underflow error. */
#define IMX_ENET_QOS_MAC_TX_UNDERFLOW_ERROR_PACKETS_TXUNDRFLW_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_UNDERFLOW_ERROR_PACKETS_TXUNDRFLW_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_UNDERFLOW_ERROR_PACKETS_TXUNDRFLW(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_SINGLE_COLLISION_GOOD_PACKETS - Single Collision Good Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_SINGLE_COLLISION_GOOD_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 TXSNGLCOLG : 32;                /**< Tx Single Collision Good Packets This field indicates the number of successfully transmitted packets after a single collision in the half-duplex mode. */
    } B;
} IMX_ENET_QOS_MAC_TX_SINGLE_COLLISION_GOOD_PACKETS_t;

/**< Register MAC_TX_SINGLE_COLLISION_GOOD_PACKETS - Single Collision Good Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_SINGLE_COLLISION_GOOD_PACKETS 0x74C
/**< Field TXSNGLCOLG - Tx Single Collision Good Packets This field indicates the number of successfully transmitted packets after a single collision in the half-duplex mode. */
#define IMX_ENET_QOS_MAC_TX_SINGLE_COLLISION_GOOD_PACKETS_TXSNGLCOLG_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_SINGLE_COLLISION_GOOD_PACKETS_TXSNGLCOLG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_SINGLE_COLLISION_GOOD_PACKETS_TXSNGLCOLG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS - Multiple Collision Good Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 TXMULTCOLG : 32;                /**< Tx Multiple Collision Good Packets This field indicates the number of successfully transmitted packets after multiple collisions in the half-duplex mode. */
    } B;
} IMX_ENET_QOS_MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS_t;

/**< Register MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS - Multiple Collision Good Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS 0x750
/**< Field TXMULTCOLG - Tx Multiple Collision Good Packets This field indicates the number of successfully transmitted packets after multiple collisions in the half-duplex mode. */
#define IMX_ENET_QOS_MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS_TXMULTCOLG_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS_TXMULTCOLG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS_TXMULTCOLG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_DEFERRED_PACKETS - Deferred Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_DEFERRED_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 TXDEFRD : 32;                /**< Tx Deferred Packets This field indicates the number of successfully transmitted after a deferral in the half-duplex mode. */
    } B;
} IMX_ENET_QOS_MAC_TX_DEFERRED_PACKETS_t;

/**< Register MAC_TX_DEFERRED_PACKETS - Deferred Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_DEFERRED_PACKETS     0x754
/**< Field TXDEFRD - Tx Deferred Packets This field indicates the number of successfully transmitted after a deferral in the half-duplex mode. */
#define IMX_ENET_QOS_MAC_TX_DEFERRED_PACKETS_TXDEFRD_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_DEFERRED_PACKETS_TXDEFRD_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_DEFERRED_PACKETS_TXDEFRD(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_LATE_COLLISION_PACKETS - Late Collision Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_LATE_COLLISION_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 TXLATECOL : 32;                /**< Tx Late Collision Packets This field indicates the number of packets aborted because of late collision error. */
    } B;
} IMX_ENET_QOS_MAC_TX_LATE_COLLISION_PACKETS_t;

/**< Register MAC_TX_LATE_COLLISION_PACKETS - Late Collision Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_LATE_COLLISION_PACKETS 0x758
/**< Field TXLATECOL - Tx Late Collision Packets This field indicates the number of packets aborted because of late collision error. */
#define IMX_ENET_QOS_MAC_TX_LATE_COLLISION_PACKETS_TXLATECOL_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_LATE_COLLISION_PACKETS_TXLATECOL_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_LATE_COLLISION_PACKETS_TXLATECOL(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_EXCESSIVE_COLLISION_PACKETS - Excessive Collision Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_EXCESSIVE_COLLISION_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 TXEXSCOL : 32;                /**< Tx Excessive Collision Packets This field indicates the number of packets aborted because of excessive (16) collision errors. */
    } B;
} IMX_ENET_QOS_MAC_TX_EXCESSIVE_COLLISION_PACKETS_t;

/**< Register MAC_TX_EXCESSIVE_COLLISION_PACKETS - Excessive Collision Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_EXCESSIVE_COLLISION_PACKETS 0x75C
/**< Field TXEXSCOL - Tx Excessive Collision Packets This field indicates the number of packets aborted because of excessive (16) collision errors. */
#define IMX_ENET_QOS_MAC_TX_EXCESSIVE_COLLISION_PACKETS_TXEXSCOL_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_EXCESSIVE_COLLISION_PACKETS_TXEXSCOL_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_EXCESSIVE_COLLISION_PACKETS_TXEXSCOL(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_CARRIER_ERROR_PACKETS - Carrier Error Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_CARRIER_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 TXCARR : 32;                /**< Tx Carrier Error Packets This field indicates the number of packets aborted because of carrier sense error (no carrier or loss of carrier). */
    } B;
} IMX_ENET_QOS_MAC_TX_CARRIER_ERROR_PACKETS_t;

/**< Register MAC_TX_CARRIER_ERROR_PACKETS - Carrier Error Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_CARRIER_ERROR_PACKETS 0x760
/**< Field TXCARR - Tx Carrier Error Packets This field indicates the number of packets aborted because of carrier sense error (no carrier or loss of carrier). */
#define IMX_ENET_QOS_MAC_TX_CARRIER_ERROR_PACKETS_TXCARR_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_CARRIER_ERROR_PACKETS_TXCARR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_CARRIER_ERROR_PACKETS_TXCARR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD - Bytes Transmitted in Good Packets */
typedef union IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 TXOCTG : 32;                /**< Tx Octet Count Good This field indicates the number of bytes transmitted, exclusive of preamble, only in good packets. */
    } B;
} IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_t;

/**< Register MAC_TX_OCTET_COUNT_GOOD - Bytes Transmitted in Good Packets */
#define IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD     0x764
/**< Field TXOCTG - Tx Octet Count Good This field indicates the number of bytes transmitted, exclusive of preamble, only in good packets. */
#define IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_TXOCTG_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_TXOCTG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_TXOCTG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD - Good Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 TXPKTG : 32;                /**< Tx Packet Count Good This field indicates the number of good packets transmitted. */
    } B;
} IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_t;

/**< Register MAC_TX_PACKET_COUNT_GOOD - Good Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD    0x768
/**< Field TXPKTG - Tx Packet Count Good This field indicates the number of good packets transmitted. */
#define IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_TXPKTG_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_TXPKTG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_TXPKTG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_EXCESSIVE_DEFERRAL_ERROR - Packets Aborted By Excessive Deferral Error */
typedef union IMX_ENET_QOS_MAC_TX_EXCESSIVE_DEFERRAL_ERROR_union_t {
    UINT32 R;
    struct {
        UINT32 TXEXSDEF : 32;                /**< Tx Excessive Deferral Error This field indicates the number of packets aborted because of excessive deferral error (deferred for more than two max-sized packet times). */
    } B;
} IMX_ENET_QOS_MAC_TX_EXCESSIVE_DEFERRAL_ERROR_t;

/**< Register MAC_TX_EXCESSIVE_DEFERRAL_ERROR - Packets Aborted By Excessive Deferral Error */
#define IMX_ENET_QOS_MAC_TX_EXCESSIVE_DEFERRAL_ERROR 0x76C
/**< Field TXEXSDEF - Tx Excessive Deferral Error This field indicates the number of packets aborted because of excessive deferral error (deferred for more than two max-sized packet times). */
#define IMX_ENET_QOS_MAC_TX_EXCESSIVE_DEFERRAL_ERROR_TXEXSDEF_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_EXCESSIVE_DEFERRAL_ERROR_TXEXSDEF_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_EXCESSIVE_DEFERRAL_ERROR_TXEXSDEF(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_PAUSE_PACKETS - Pause Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_PAUSE_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 TXPAUSE : 32;                /**< Tx Pause Packets This field indicates the number of good Pause packets transmitted. */
    } B;
} IMX_ENET_QOS_MAC_TX_PAUSE_PACKETS_t;

/**< Register MAC_TX_PAUSE_PACKETS - Pause Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_PAUSE_PACKETS        0x770
/**< Field TXPAUSE - Tx Pause Packets This field indicates the number of good Pause packets transmitted. */
#define IMX_ENET_QOS_MAC_TX_PAUSE_PACKETS_TXPAUSE_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_PAUSE_PACKETS_TXPAUSE_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_PAUSE_PACKETS_TXPAUSE(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_VLAN_PACKETS_GOOD - Good VLAN Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_VLAN_PACKETS_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 TXVLANG : 32;                /**< Tx VLAN Packets Good This field provides the number of good VLAN packets transmitted. */
    } B;
} IMX_ENET_QOS_MAC_TX_VLAN_PACKETS_GOOD_t;

/**< Register MAC_TX_VLAN_PACKETS_GOOD - Good VLAN Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_VLAN_PACKETS_GOOD    0x774
/**< Field TXVLANG - Tx VLAN Packets Good This field provides the number of good VLAN packets transmitted. */
#define IMX_ENET_QOS_MAC_TX_VLAN_PACKETS_GOOD_TXVLANG_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_VLAN_PACKETS_GOOD_TXVLANG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_VLAN_PACKETS_GOOD_TXVLANG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_OSIZE_PACKETS_GOOD - Good Oversize Packets Transmitted */
typedef union IMX_ENET_QOS_MAC_TX_OSIZE_PACKETS_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 TXOSIZG : 32;                /**< Tx OSize Packets Good This field indicates the number of packets transmitted without errors and with length greater than the maxsize (1,518 or 1,522 bytes for VLAN tagged packets; 2000 bytes if enabled in S2KP bit of the CONFIGURATION register). */
    } B;
} IMX_ENET_QOS_MAC_TX_OSIZE_PACKETS_GOOD_t;

/**< Register MAC_TX_OSIZE_PACKETS_GOOD - Good Oversize Packets Transmitted */
#define IMX_ENET_QOS_MAC_TX_OSIZE_PACKETS_GOOD   0x778
/**< Field TXOSIZG - Tx OSize Packets Good This field indicates the number of packets transmitted without errors and with length greater than the maxsize (1,518 or 1,522 bytes for VLAN tagged packets; 2000 bytes if enabled in S2KP bit of the CONFIGURATION register). */
#define IMX_ENET_QOS_MAC_TX_OSIZE_PACKETS_GOOD_TXOSIZG_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_OSIZE_PACKETS_GOOD_TXOSIZG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_OSIZE_PACKETS_GOOD_TXOSIZG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_PACKETS_COUNT_GOOD_BAD - Good and Bad Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_PACKETS_COUNT_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 RXPKTGB : 32;                /**< Rx Packets Count Good Bad This field indicates the number of good and bad packets received. */
    } B;
} IMX_ENET_QOS_MAC_RX_PACKETS_COUNT_GOOD_BAD_t;

/**< Register MAC_RX_PACKETS_COUNT_GOOD_BAD - Good and Bad Packets Received */
#define IMX_ENET_QOS_MAC_RX_PACKETS_COUNT_GOOD_BAD 0x780
/**< Field RXPKTGB - Rx Packets Count Good Bad This field indicates the number of good and bad packets received. */
#define IMX_ENET_QOS_MAC_RX_PACKETS_COUNT_GOOD_BAD_RXPKTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_PACKETS_COUNT_GOOD_BAD_RXPKTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_PACKETS_COUNT_GOOD_BAD_RXPKTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_BAD - Bytes in Good and Bad Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 RXOCTGB : 32;                /**< Rx Octet Count Good Bad This field indicates the number of bytes received, exclusive of preamble, in good and bad packets. */
    } B;
} IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_BAD_t;

/**< Register MAC_RX_OCTET_COUNT_GOOD_BAD - Bytes in Good and Bad Packets Received */
#define IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_BAD 0x784
/**< Field RXOCTGB - Rx Octet Count Good Bad This field indicates the number of bytes received, exclusive of preamble, in good and bad packets. */
#define IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_BAD_RXOCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_BAD_RXOCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_BAD_RXOCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD - Bytes in Good Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 RXOCTG : 32;                /**< Rx Octet Count Good This field indicates the number of bytes received, exclusive of preamble, only in good packets. */
    } B;
} IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_t;

/**< Register MAC_RX_OCTET_COUNT_GOOD - Bytes in Good Packets Received */
#define IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD     0x788
/**< Field RXOCTG - Rx Octet Count Good This field indicates the number of bytes received, exclusive of preamble, only in good packets. */
#define IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_RXOCTG_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_RXOCTG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_RXOCTG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_BROADCAST_PACKETS_GOOD - Good Broadcast Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_BROADCAST_PACKETS_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 RXBCASTG : 32;                /**< Rx Broadcast Packets Good This field indicates the number of good broadcast packets received. */
    } B;
} IMX_ENET_QOS_MAC_RX_BROADCAST_PACKETS_GOOD_t;

/**< Register MAC_RX_BROADCAST_PACKETS_GOOD - Good Broadcast Packets Received */
#define IMX_ENET_QOS_MAC_RX_BROADCAST_PACKETS_GOOD 0x78C
/**< Field RXBCASTG - Rx Broadcast Packets Good This field indicates the number of good broadcast packets received. */
#define IMX_ENET_QOS_MAC_RX_BROADCAST_PACKETS_GOOD_RXBCASTG_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_BROADCAST_PACKETS_GOOD_RXBCASTG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_BROADCAST_PACKETS_GOOD_RXBCASTG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_MULTICAST_PACKETS_GOOD - Good Multicast Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_MULTICAST_PACKETS_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 RXMCASTG : 32;                /**< Rx Multicast Packets Good This field indicates the number of good multicast packets received. */
    } B;
} IMX_ENET_QOS_MAC_RX_MULTICAST_PACKETS_GOOD_t;

/**< Register MAC_RX_MULTICAST_PACKETS_GOOD - Good Multicast Packets Received */
#define IMX_ENET_QOS_MAC_RX_MULTICAST_PACKETS_GOOD 0x790
/**< Field RXMCASTG - Rx Multicast Packets Good This field indicates the number of good multicast packets received. */
#define IMX_ENET_QOS_MAC_RX_MULTICAST_PACKETS_GOOD_RXMCASTG_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_MULTICAST_PACKETS_GOOD_RXMCASTG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_MULTICAST_PACKETS_GOOD_RXMCASTG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_CRC_ERROR_PACKETS - CRC Error Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_CRC_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXCRCERR : 32;                /**< Rx CRC Error Packets This field indicates the number of packets received with CRC error. */
    } B;
} IMX_ENET_QOS_MAC_RX_CRC_ERROR_PACKETS_t;

/**< Register MAC_RX_CRC_ERROR_PACKETS - CRC Error Packets Received */
#define IMX_ENET_QOS_MAC_RX_CRC_ERROR_PACKETS    0x794
/**< Field RXCRCERR - Rx CRC Error Packets This field indicates the number of packets received with CRC error. */
#define IMX_ENET_QOS_MAC_RX_CRC_ERROR_PACKETS_RXCRCERR_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_CRC_ERROR_PACKETS_RXCRCERR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_CRC_ERROR_PACKETS_RXCRCERR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_ALIGNMENT_ERROR_PACKETS - Alignment Error Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_ALIGNMENT_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXALGNERR : 32;                /**< Rx Alignment Error Packets This field indicates the number of packets received with alignment (dribble) error. */
    } B;
} IMX_ENET_QOS_MAC_RX_ALIGNMENT_ERROR_PACKETS_t;

/**< Register MAC_RX_ALIGNMENT_ERROR_PACKETS - Alignment Error Packets Received */
#define IMX_ENET_QOS_MAC_RX_ALIGNMENT_ERROR_PACKETS 0x798
/**< Field RXALGNERR - Rx Alignment Error Packets This field indicates the number of packets received with alignment (dribble) error. */
#define IMX_ENET_QOS_MAC_RX_ALIGNMENT_ERROR_PACKETS_RXALGNERR_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_ALIGNMENT_ERROR_PACKETS_RXALGNERR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_ALIGNMENT_ERROR_PACKETS_RXALGNERR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_RUNT_ERROR_PACKETS - Runt Error Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_RUNT_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXRUNTERR : 32;                /**< Rx Runt Error Packets This field indicates the number of packets received with runt (length less than 64 bytes and CRC error) error. */
    } B;
} IMX_ENET_QOS_MAC_RX_RUNT_ERROR_PACKETS_t;

/**< Register MAC_RX_RUNT_ERROR_PACKETS - Runt Error Packets Received */
#define IMX_ENET_QOS_MAC_RX_RUNT_ERROR_PACKETS   0x79C
/**< Field RXRUNTERR - Rx Runt Error Packets This field indicates the number of packets received with runt (length less than 64 bytes and CRC error) error. */
#define IMX_ENET_QOS_MAC_RX_RUNT_ERROR_PACKETS_RXRUNTERR_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_RUNT_ERROR_PACKETS_RXRUNTERR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_RUNT_ERROR_PACKETS_RXRUNTERR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_JABBER_ERROR_PACKETS - Jabber Error Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_JABBER_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXJABERR : 32;                /**< Rx Jabber Error Packets This field indicates the number of giant packets received with length (including CRC) greater than 1,518 bytes (1,522 bytes for VLAN tagged) and with CRC error. */
    } B;
} IMX_ENET_QOS_MAC_RX_JABBER_ERROR_PACKETS_t;

/**< Register MAC_RX_JABBER_ERROR_PACKETS - Jabber Error Packets Received */
#define IMX_ENET_QOS_MAC_RX_JABBER_ERROR_PACKETS 0x7A0
/**< Field RXJABERR - Rx Jabber Error Packets This field indicates the number of giant packets received with length (including CRC) greater than 1,518 bytes (1,522 bytes for VLAN tagged) and with CRC error. */
#define IMX_ENET_QOS_MAC_RX_JABBER_ERROR_PACKETS_RXJABERR_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_JABBER_ERROR_PACKETS_RXJABERR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_JABBER_ERROR_PACKETS_RXJABERR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_UNDERSIZE_PACKETS_GOOD - Good Undersize Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_UNDERSIZE_PACKETS_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 RXUNDERSZG : 32;                /**< Rx Undersize Packets Good This field indicates the number of packets received with length less than 64 bytes, without any errors. */
    } B;
} IMX_ENET_QOS_MAC_RX_UNDERSIZE_PACKETS_GOOD_t;

/**< Register MAC_RX_UNDERSIZE_PACKETS_GOOD - Good Undersize Packets Received */
#define IMX_ENET_QOS_MAC_RX_UNDERSIZE_PACKETS_GOOD 0x7A4
/**< Field RXUNDERSZG - Rx Undersize Packets Good This field indicates the number of packets received with length less than 64 bytes, without any errors. */
#define IMX_ENET_QOS_MAC_RX_UNDERSIZE_PACKETS_GOOD_RXUNDERSZG_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_UNDERSIZE_PACKETS_GOOD_RXUNDERSZG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_UNDERSIZE_PACKETS_GOOD_RXUNDERSZG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_OVERSIZE_PACKETS_GOOD - Good Oversize Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_OVERSIZE_PACKETS_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 RXOVERSZG : 32;                /**< Rx Oversize Packets Good This field indicates the number of packets received without errors, with length greater than the maxsize (1,518 bytes or 1,522 bytes for VLAN tagged packets; 2000 bytes if enabled in the S2KP bit of the MAC_CONFIGURATION register). */
    } B;
} IMX_ENET_QOS_MAC_RX_OVERSIZE_PACKETS_GOOD_t;

/**< Register MAC_RX_OVERSIZE_PACKETS_GOOD - Good Oversize Packets Received */
#define IMX_ENET_QOS_MAC_RX_OVERSIZE_PACKETS_GOOD 0x7A8
/**< Field RXOVERSZG - Rx Oversize Packets Good This field indicates the number of packets received without errors, with length greater than the maxsize (1,518 bytes or 1,522 bytes for VLAN tagged packets; 2000 bytes if enabled in the S2KP bit of the MAC_CONFIGURATION register). */
#define IMX_ENET_QOS_MAC_RX_OVERSIZE_PACKETS_GOOD_RXOVERSZG_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_OVERSIZE_PACKETS_GOOD_RXOVERSZG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_OVERSIZE_PACKETS_GOOD_RXOVERSZG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_64OCTETS_PACKETS_GOOD_BAD - Good and Bad 64-Byte Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_64OCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 RX64OCTGB : 32;                /**< Rx 64 Octets Packets Good Bad This field indicates the number of good and bad packets received with length 64 bytes, exclusive of the preamble. */
    } B;
} IMX_ENET_QOS_MAC_RX_64OCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_RX_64OCTETS_PACKETS_GOOD_BAD - Good and Bad 64-Byte Packets Received */
#define IMX_ENET_QOS_MAC_RX_64OCTETS_PACKETS_GOOD_BAD 0x7AC
/**< Field RX64OCTGB - Rx 64 Octets Packets Good Bad This field indicates the number of good and bad packets received with length 64 bytes, exclusive of the preamble. */
#define IMX_ENET_QOS_MAC_RX_64OCTETS_PACKETS_GOOD_BAD_RX64OCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_64OCTETS_PACKETS_GOOD_BAD_RX64OCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_64OCTETS_PACKETS_GOOD_BAD_RX64OCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_65TO127OCTETS_PACKETS_GOOD_BAD - Good and Bad 64-to-127 Byte Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_65TO127OCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 RX65_127OCTGB : 32;              /**< Rx 65-127 Octets Packets Good Bad This field indicates the number of good and bad packets received with length between 65 and 127 (inclusive) bytes, exclusive of the preamble. */
    } B;
} IMX_ENET_QOS_MAC_RX_65TO127OCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_RX_65TO127OCTETS_PACKETS_GOOD_BAD - Good and Bad 64-to-127 Byte Packets Received */
#define IMX_ENET_QOS_MAC_RX_65TO127OCTETS_PACKETS_GOOD_BAD 0x7B0
/**< Field RX65_127OCTGB - Rx 65-127 Octets Packets Good Bad This field indicates the number of good and bad packets received with length between 65 and 127 (inclusive) bytes, exclusive of the preamble. */
#define IMX_ENET_QOS_MAC_RX_65TO127OCTETS_PACKETS_GOOD_BAD_RX65_127OCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_65TO127OCTETS_PACKETS_GOOD_BAD_RX65_127OCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_65TO127OCTETS_PACKETS_GOOD_BAD_RX65_127OCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_128TO255OCTETS_PACKETS_GOOD_BAD - Good and Bad 128-to-255 Byte Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_128TO255OCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 RX128_255OCTGB : 32;             /**< Rx 128-255 Octets Packets Good Bad This field indicates the number of good and bad packets received with length between 128 and 255 (inclusive) bytes, exclusive of the preamble. */
    } B;
} IMX_ENET_QOS_MAC_RX_128TO255OCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_RX_128TO255OCTETS_PACKETS_GOOD_BAD - Good and Bad 128-to-255 Byte Packets Received */
#define IMX_ENET_QOS_MAC_RX_128TO255OCTETS_PACKETS_GOOD_BAD 0x7B4
/**< Field RX128_255OCTGB - Rx 128-255 Octets Packets Good Bad This field indicates the number of good and bad packets received with length between 128 and 255 (inclusive) bytes, exclusive of the preamble. */
#define IMX_ENET_QOS_MAC_RX_128TO255OCTETS_PACKETS_GOOD_BAD_RX128_255OCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_128TO255OCTETS_PACKETS_GOOD_BAD_RX128_255OCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_128TO255OCTETS_PACKETS_GOOD_BAD_RX128_255OCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_256TO511OCTETS_PACKETS_GOOD_BAD - Good and Bad 256-to-511 Byte Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_256TO511OCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 RX256_511OCTGB : 32;             /**< Rx 256-511 Octets Packets Good Bad This field indicates the number of good and bad packets received with length between 256 and 511 (inclusive) bytes, exclusive of the preamble. */
    } B;
} IMX_ENET_QOS_MAC_RX_256TO511OCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_RX_256TO511OCTETS_PACKETS_GOOD_BAD - Good and Bad 256-to-511 Byte Packets Received */
#define IMX_ENET_QOS_MAC_RX_256TO511OCTETS_PACKETS_GOOD_BAD 0x7B8
/**< Field RX256_511OCTGB - Rx 256-511 Octets Packets Good Bad This field indicates the number of good and bad packets received with length between 256 and 511 (inclusive) bytes, exclusive of the preamble. */
#define IMX_ENET_QOS_MAC_RX_256TO511OCTETS_PACKETS_GOOD_BAD_RX256_511OCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_256TO511OCTETS_PACKETS_GOOD_BAD_RX256_511OCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_256TO511OCTETS_PACKETS_GOOD_BAD_RX256_511OCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_512TO1023OCTETS_PACKETS_GOOD_BAD - Good and Bad 512-to-1023 Byte Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_512TO1023OCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 RX512_1023OCTGB : 32;            /**< RX 512-1023 Octets Packets Good Bad This field indicates the number of good and bad packets received with length between 512 and 1023 (inclusive) bytes, exclusive of the preamble. */
    } B;
} IMX_ENET_QOS_MAC_RX_512TO1023OCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_RX_512TO1023OCTETS_PACKETS_GOOD_BAD - Good and Bad 512-to-1023 Byte Packets Received */
#define IMX_ENET_QOS_MAC_RX_512TO1023OCTETS_PACKETS_GOOD_BAD 0x7BC
/**< Field RX512_1023OCTGB - RX 512-1023 Octets Packets Good Bad This field indicates the number of good and bad packets received with length between 512 and 1023 (inclusive) bytes, exclusive of the preamble. */
#define IMX_ENET_QOS_MAC_RX_512TO1023OCTETS_PACKETS_GOOD_BAD_RX512_1023OCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_512TO1023OCTETS_PACKETS_GOOD_BAD_RX512_1023OCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_512TO1023OCTETS_PACKETS_GOOD_BAD_RX512_1023OCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD - Good and Bad 1024-to-Max Byte Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 RX1024_MAXOCTGB : 32;            /**< Rx 1024-Max Octets Good Bad This field indicates the number of good and bad packets received with length between 1024 and maxsize (inclusive) bytes, exclusive of the preamble. */
    } B;
} IMX_ENET_QOS_MAC_RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_t;

/**< Register MAC_RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD - Good and Bad 1024-to-Max Byte Packets Received */
#define IMX_ENET_QOS_MAC_RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD 0x7C0
/**< Field RX1024_MAXOCTGB - Rx 1024-Max Octets Good Bad This field indicates the number of good and bad packets received with length between 1024 and maxsize (inclusive) bytes, exclusive of the preamble. */
#define IMX_ENET_QOS_MAC_RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_RX1024_MAXOCTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_RX1024_MAXOCTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_RX1024_MAXOCTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_UNICAST_PACKETS_GOOD - Good Unicast Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_UNICAST_PACKETS_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 RXUCASTG : 32;                /**< Rx Unicast Packets Good This field indicates the number of good unicast packets received. */
    } B;
} IMX_ENET_QOS_MAC_RX_UNICAST_PACKETS_GOOD_t;

/**< Register MAC_RX_UNICAST_PACKETS_GOOD - Good Unicast Packets Received */
#define IMX_ENET_QOS_MAC_RX_UNICAST_PACKETS_GOOD 0x7C4
/**< Field RXUCASTG - Rx Unicast Packets Good This field indicates the number of good unicast packets received. */
#define IMX_ENET_QOS_MAC_RX_UNICAST_PACKETS_GOOD_RXUCASTG_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_UNICAST_PACKETS_GOOD_RXUCASTG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_UNICAST_PACKETS_GOOD_RXUCASTG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_LENGTH_ERROR_PACKETS - Length Error Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_LENGTH_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXLENERR : 32;                /**< Rx Length Error Packets This field indicates the number of packets received with length error (Length Type field not equal to packet size), for all packets with valid length field. */
    } B;
} IMX_ENET_QOS_MAC_RX_LENGTH_ERROR_PACKETS_t;

/**< Register MAC_RX_LENGTH_ERROR_PACKETS - Length Error Packets Received */
#define IMX_ENET_QOS_MAC_RX_LENGTH_ERROR_PACKETS 0x7C8
/**< Field RXLENERR - Rx Length Error Packets This field indicates the number of packets received with length error (Length Type field not equal to packet size), for all packets with valid length field. */
#define IMX_ENET_QOS_MAC_RX_LENGTH_ERROR_PACKETS_RXLENERR_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_LENGTH_ERROR_PACKETS_RXLENERR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_LENGTH_ERROR_PACKETS_RXLENERR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_OUT_OF_RANGE_TYPE_PACKETS - Out-of-range Type Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_OUT_OF_RANGE_TYPE_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXOUTOFRNG : 32;                /**< Rx Out of Range Type Packet This field indicates the number of packets received with length field not equal to the valid packet size (greater than 1,500 but less than 1,536). */
    } B;
} IMX_ENET_QOS_MAC_RX_OUT_OF_RANGE_TYPE_PACKETS_t;

/**< Register MAC_RX_OUT_OF_RANGE_TYPE_PACKETS - Out-of-range Type Packets Received */
#define IMX_ENET_QOS_MAC_RX_OUT_OF_RANGE_TYPE_PACKETS 0x7CC
/**< Field RXOUTOFRNG - Rx Out of Range Type Packet This field indicates the number of packets received with length field not equal to the valid packet size (greater than 1,500 but less than 1,536). */
#define IMX_ENET_QOS_MAC_RX_OUT_OF_RANGE_TYPE_PACKETS_RXOUTOFRNG_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_OUT_OF_RANGE_TYPE_PACKETS_RXOUTOFRNG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_OUT_OF_RANGE_TYPE_PACKETS_RXOUTOFRNG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_PAUSE_PACKETS - Pause Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_PAUSE_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXPAUSEPKT : 32;                /**< Rx Pause Packets This field indicates the number of good and valid Pause packets received. */
    } B;
} IMX_ENET_QOS_MAC_RX_PAUSE_PACKETS_t;

/**< Register MAC_RX_PAUSE_PACKETS - Pause Packets Received */
#define IMX_ENET_QOS_MAC_RX_PAUSE_PACKETS        0x7D0
/**< Field RXPAUSEPKT - Rx Pause Packets This field indicates the number of good and valid Pause packets received. */
#define IMX_ENET_QOS_MAC_RX_PAUSE_PACKETS_RXPAUSEPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_PAUSE_PACKETS_RXPAUSEPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_PAUSE_PACKETS_RXPAUSEPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_FIFO_OVERFLOW_PACKETS - Missed Packets Due to FIFO Overflow */
typedef union IMX_ENET_QOS_MAC_RX_FIFO_OVERFLOW_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXFIFOOVFL : 32;                /**< Rx FIFO Overflow Packets This field indicates the number of missed received packets because of FIFO overflow. */
    } B;
} IMX_ENET_QOS_MAC_RX_FIFO_OVERFLOW_PACKETS_t;

/**< Register MAC_RX_FIFO_OVERFLOW_PACKETS - Missed Packets Due to FIFO Overflow */
#define IMX_ENET_QOS_MAC_RX_FIFO_OVERFLOW_PACKETS 0x7D4
/**< Field RXFIFOOVFL - Rx FIFO Overflow Packets This field indicates the number of missed received packets because of FIFO overflow. */
#define IMX_ENET_QOS_MAC_RX_FIFO_OVERFLOW_PACKETS_RXFIFOOVFL_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_FIFO_OVERFLOW_PACKETS_RXFIFOOVFL_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_FIFO_OVERFLOW_PACKETS_RXFIFOOVFL(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_VLAN_PACKETS_GOOD_BAD - Good and Bad VLAN Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_VLAN_PACKETS_GOOD_BAD_union_t {
    UINT32 R;
    struct {
        UINT32 RXVLANPKTGB : 32;                /**< Rx VLAN Packets Good Bad This field indicates the number of good and bad VLAN packets received. */
    } B;
} IMX_ENET_QOS_MAC_RX_VLAN_PACKETS_GOOD_BAD_t;

/**< Register MAC_RX_VLAN_PACKETS_GOOD_BAD - Good and Bad VLAN Packets Received */
#define IMX_ENET_QOS_MAC_RX_VLAN_PACKETS_GOOD_BAD 0x7D8
/**< Field RXVLANPKTGB - Rx VLAN Packets Good Bad This field indicates the number of good and bad VLAN packets received. */
#define IMX_ENET_QOS_MAC_RX_VLAN_PACKETS_GOOD_BAD_RXVLANPKTGB_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_VLAN_PACKETS_GOOD_BAD_RXVLANPKTGB_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_VLAN_PACKETS_GOOD_BAD_RXVLANPKTGB(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_WATCHDOG_ERROR_PACKETS - Watchdog Error Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_WATCHDOG_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXWDGERR : 32;                /**< Rx Watchdog Error Packets This field indicates the number of packets received with error because of watchdog timeout error (packets with a data load larger than 2,048 bytes (when JE and WD bits are reset in MAC_CONFIGURATION register), 10,240 bytes (when JE bit is set and WD bit is reset in MAC_CONFIGURATION register), 16,384 bytes (when WD bit is set in MAC_CONFIGURATION register) or the value programmed in the MAC_WATCHDOG_TIMEOUT register). */
    } B;
} IMX_ENET_QOS_MAC_RX_WATCHDOG_ERROR_PACKETS_t;

/**< Register MAC_RX_WATCHDOG_ERROR_PACKETS - Watchdog Error Packets Received */
#define IMX_ENET_QOS_MAC_RX_WATCHDOG_ERROR_PACKETS 0x7DC
/**< Field RXWDGERR - Rx Watchdog Error Packets This field indicates the number of packets received with error because of watchdog timeout error (packets with a data load larger than 2,048 bytes (when JE and WD bits are reset in MAC_CONFIGURATION register), 10,240 bytes (when JE bit is set and WD bit is reset in MAC_CONFIGURATION register), 16,384 bytes (when WD bit is set in MAC_CONFIGURATION register) or the value programmed in the MAC_WATCHDOG_TIMEOUT register). */
#define IMX_ENET_QOS_MAC_RX_WATCHDOG_ERROR_PACKETS_RXWDGERR_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_WATCHDOG_ERROR_PACKETS_RXWDGERR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_WATCHDOG_ERROR_PACKETS_RXWDGERR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_RECEIVE_ERROR_PACKETS - Receive Error Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_RECEIVE_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXRCVERR : 32;                /**< Rx Receive Error Packets This field indicates the number of packets received with Receive error or Packet Extension error on the GMII or MII interface. */
    } B;
} IMX_ENET_QOS_MAC_RX_RECEIVE_ERROR_PACKETS_t;

/**< Register MAC_RX_RECEIVE_ERROR_PACKETS - Receive Error Packets Received */
#define IMX_ENET_QOS_MAC_RX_RECEIVE_ERROR_PACKETS 0x7E0
/**< Field RXRCVERR - Rx Receive Error Packets This field indicates the number of packets received with Receive error or Packet Extension error on the GMII or MII interface. */
#define IMX_ENET_QOS_MAC_RX_RECEIVE_ERROR_PACKETS_RXRCVERR_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_RECEIVE_ERROR_PACKETS_RXRCVERR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_RECEIVE_ERROR_PACKETS_RXRCVERR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_CONTROL_PACKETS_GOOD - Good Control Packets Received */
typedef union IMX_ENET_QOS_MAC_RX_CONTROL_PACKETS_GOOD_union_t {
    UINT32 R;
    struct {
        UINT32 RXCTRLG : 32;                /**< Rx Control Packets Good This field indicates the number of good control packets received. */
    } B;
} IMX_ENET_QOS_MAC_RX_CONTROL_PACKETS_GOOD_t;

/**< Register MAC_RX_CONTROL_PACKETS_GOOD - Good Control Packets Received */
#define IMX_ENET_QOS_MAC_RX_CONTROL_PACKETS_GOOD 0x7E4
/**< Field RXCTRLG - Rx Control Packets Good This field indicates the number of good control packets received. */
#define IMX_ENET_QOS_MAC_RX_CONTROL_PACKETS_GOOD_RXCTRLG_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_CONTROL_PACKETS_GOOD_RXCTRLG_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_CONTROL_PACKETS_GOOD_RXCTRLG(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_LPI_USEC_CNTR - Microseconds Tx LPI Asserted */
typedef union IMX_ENET_QOS_MAC_TX_LPI_USEC_CNTR_union_t {
    UINT32 R;
    struct {
        UINT32 TXLPIUSC : 32;                /**< Tx LPI Microseconds Counter This field indicates the number of microseconds Tx LPI is asserted. */
    } B;
} IMX_ENET_QOS_MAC_TX_LPI_USEC_CNTR_t;

/**< Register MAC_TX_LPI_USEC_CNTR - Microseconds Tx LPI Asserted */
#define IMX_ENET_QOS_MAC_TX_LPI_USEC_CNTR        0x7EC
/**< Field TXLPIUSC - Tx LPI Microseconds Counter This field indicates the number of microseconds Tx LPI is asserted. */
#define IMX_ENET_QOS_MAC_TX_LPI_USEC_CNTR_TXLPIUSC_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_LPI_USEC_CNTR_TXLPIUSC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_LPI_USEC_CNTR_TXLPIUSC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TX_LPI_TRAN_CNTR - Number of Times Tx LPI Asserted */
typedef union IMX_ENET_QOS_MAC_TX_LPI_TRAN_CNTR_union_t {
    UINT32 R;
    struct {
        UINT32 TXLPITRC : 32;                /**< Tx LPI Transition counter This field indicates the number of times Tx LPI Entry has occurred. */
    } B;
} IMX_ENET_QOS_MAC_TX_LPI_TRAN_CNTR_t;

/**< Register MAC_TX_LPI_TRAN_CNTR - Number of Times Tx LPI Asserted */
#define IMX_ENET_QOS_MAC_TX_LPI_TRAN_CNTR        0x7F0
/**< Field TXLPITRC - Tx LPI Transition counter This field indicates the number of times Tx LPI Entry has occurred. */
#define IMX_ENET_QOS_MAC_TX_LPI_TRAN_CNTR_TXLPITRC_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_LPI_TRAN_CNTR_TXLPITRC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_LPI_TRAN_CNTR_TXLPITRC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_LPI_USEC_CNTR - Microseconds Rx LPI Sampled */
typedef union IMX_ENET_QOS_MAC_RX_LPI_USEC_CNTR_union_t {
    UINT32 R;
    struct {
        UINT32 RXLPIUSC : 32;                /**< Rx LPI Microseconds Counter This field indicates the number of microseconds Rx LPI is asserted. */
    } B;
} IMX_ENET_QOS_MAC_RX_LPI_USEC_CNTR_t;

/**< Register MAC_RX_LPI_USEC_CNTR - Microseconds Rx LPI Sampled */
#define IMX_ENET_QOS_MAC_RX_LPI_USEC_CNTR        0x7F4
/**< Field RXLPIUSC - Rx LPI Microseconds Counter This field indicates the number of microseconds Rx LPI is asserted. */
#define IMX_ENET_QOS_MAC_RX_LPI_USEC_CNTR_RXLPIUSC_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_LPI_USEC_CNTR_RXLPIUSC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_LPI_USEC_CNTR_RXLPIUSC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RX_LPI_TRAN_CNTR - Number of Times Rx LPI Entered */
typedef union IMX_ENET_QOS_MAC_RX_LPI_TRAN_CNTR_union_t {
    UINT32 R;
    struct {
        UINT32 RXLPITRC : 32;                /**< Rx LPI Transition counter This field indicates the number of times Rx LPI Entry has occurred. */
    } B;
} IMX_ENET_QOS_MAC_RX_LPI_TRAN_CNTR_t;

/**< Register MAC_RX_LPI_TRAN_CNTR - Number of Times Rx LPI Entered */
#define IMX_ENET_QOS_MAC_RX_LPI_TRAN_CNTR        0x7F8
/**< Field RXLPITRC - Rx LPI Transition counter This field indicates the number of times Rx LPI Entry has occurred. */
#define IMX_ENET_QOS_MAC_RX_LPI_TRAN_CNTR_RXLPITRC_SHIFT 0
#define IMX_ENET_QOS_MAC_RX_LPI_TRAN_CNTR_RXLPITRC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RX_LPI_TRAN_CNTR_RXLPITRC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK - MMC IPC Receive Interrupt Mask */
typedef union IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4GPIM : 1;                 /**< MMC Receive IPV4 Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_gd_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4HERPIM : 1;                /**< MMC Receive IPV4 Header Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_hdrerr_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4NOPAYPIM : 1;              /**< MMC Receive IPV4 No Payload Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_nopay_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4FRAGPIM : 1;               /**< MMC Receive IPV4 Fragmented Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_frag_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4UDSBLPIM : 1;              /**< MMC Receive IPV4 UDP Checksum Disabled Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_udsbl_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6GPIM : 1;                 /**< MMC Receive IPV6 Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_gd_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6HERPIM : 1;                /**< MMC Receive IPV6 Header Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_hdrerr_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6NOPAYPIM : 1;              /**< MMC Receive IPV6 No Payload Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_nopay_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUDPGPIM : 1;                 /**< MMC Receive UDP Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxudp_gd_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUDPERPIM : 1;                 /**< MMC Receive UDP Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxudp_err_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXTCPGPIM : 1;                 /**< MMC Receive TCP Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxtcp_gd_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXTCPERPIM : 1;                 /**< MMC Receive TCP Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxtcp_err_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXICMPGPIM : 1;                 /**< MMC Receive ICMP Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxicmp_gd_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXICMPERPIM : 1;                 /**< MMC Receive ICMP Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxicmp_err_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused14 : 2;                 /**< Reserved. */
        UINT32 RXIPV4GOIM : 1;                 /**< MMC Receive IPV4 Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_gd_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4HEROIM : 1;                /**< MMC Receive IPV4 Header Error Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_hdrerr_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4NOPAYOIM : 1;              /**< MMC Receive IPV4 No Payload Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_nopay_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4FRAGOIM : 1;               /**< MMC Receive IPV4 Fragmented Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_frag_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4UDSBLOIM : 1;              /**< MMC Receive IPV4 UDP Checksum Disabled Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_udsbl_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6GOIM : 1;                 /**< MMC Receive IPV6 Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_gd_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6HEROIM : 1;                /**< MMC Receive IPV6 Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_hdrerr_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6NOPAYOIM : 1;              /**< MMC Receive IPV6 Header Error Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_nopay_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUDPGOIM : 1;                 /**< MMC Receive IPV6 No Payload Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxudp_gd_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUDPEROIM : 1;                 /**< MMC Receive UDP Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxudp_err_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXTCPGOIM : 1;                 /**< MMC Receive TCP Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxtcp_gd_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXTCPEROIM : 1;                 /**< MMC Receive TCP Error Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxtcp_err_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXICMPGOIM : 1;                 /**< MMC Receive ICMP Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxicmp_gd_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXICMPEROIM : 1;                 /**< MMC Receive ICMP Error Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxicmp_err_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused30 : 2;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_t;

/**< Register MAC_MMC_IPC_RX_INTERRUPT_MASK - MMC IPC Receive Interrupt Mask */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK 0x800
/**< Field RXIPV4GPIM - MMC Receive IPV4 Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_gd_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4GPIM_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4GPIM_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4GPIM(v) (((v) << 0) & 0x1)
/**< Field RXIPV4HERPIM - MMC Receive IPV4 Header Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_hdrerr_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4HERPIM_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4HERPIM_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4HERPIM(v) (((v) << 1) & 0x2)
/**< Field RXIPV4NOPAYPIM - MMC Receive IPV4 No Payload Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_nopay_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4NOPAYPIM_SHIFT 2
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4NOPAYPIM_MASK 0x4
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4NOPAYPIM(v) (((v) << 2) & 0x4)
/**< Field RXIPV4FRAGPIM - MMC Receive IPV4 Fragmented Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_frag_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4FRAGPIM_SHIFT 3
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4FRAGPIM_MASK 0x8
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4FRAGPIM(v) (((v) << 3) & 0x8)
/**< Field RXIPV4UDSBLPIM - MMC Receive IPV4 UDP Checksum Disabled Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_udsbl_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4UDSBLPIM_SHIFT 4
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4UDSBLPIM_MASK 0x10
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4UDSBLPIM(v) (((v) << 4) & 0x10)
/**< Field RXIPV6GPIM - MMC Receive IPV6 Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_gd_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6GPIM_SHIFT 5
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6GPIM_MASK 0x20
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6GPIM(v) (((v) << 5) & 0x20)
/**< Field RXIPV6HERPIM - MMC Receive IPV6 Header Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_hdrerr_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6HERPIM_SHIFT 6
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6HERPIM_MASK 0x40
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6HERPIM(v) (((v) << 6) & 0x40)
/**< Field RXIPV6NOPAYPIM - MMC Receive IPV6 No Payload Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_nopay_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6NOPAYPIM_SHIFT 7
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6NOPAYPIM_MASK 0x80
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6NOPAYPIM(v) (((v) << 7) & 0x80)
/**< Field RXUDPGPIM - MMC Receive UDP Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxudp_gd_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPGPIM_SHIFT 8
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPGPIM_MASK 0x100
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPGPIM(v) (((v) << 8) & 0x100)
/**< Field RXUDPERPIM - MMC Receive UDP Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxudp_err_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPERPIM_SHIFT 9
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPERPIM_MASK 0x200
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPERPIM(v) (((v) << 9) & 0x200)
/**< Field RXTCPGPIM - MMC Receive TCP Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxtcp_gd_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPGPIM_SHIFT 10
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPGPIM_MASK 0x400
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPGPIM(v) (((v) << 10) & 0x400)
/**< Field RXTCPERPIM - MMC Receive TCP Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxtcp_err_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPERPIM_SHIFT 11
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPERPIM_MASK 0x800
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPERPIM(v) (((v) << 11) & 0x800)
/**< Field RXICMPGPIM - MMC Receive ICMP Good Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxicmp_gd_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPGPIM_SHIFT 12
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPGPIM_MASK 0x1000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPGPIM(v) (((v) << 12) & 0x1000)
/**< Field RXICMPERPIM - MMC Receive ICMP Error Packet Counter Interrupt Mask Setting this bit masks the interrupt when the rxicmp_err_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPERPIM_SHIFT 13
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPERPIM_MASK 0x2000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPERPIM(v) (((v) << 13) & 0x2000)
/**< Field RXIPV4GOIM - MMC Receive IPV4 Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_gd_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4GOIM_SHIFT 16
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4GOIM_MASK 0x10000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4GOIM(v) (((v) << 16) & 0x10000)
/**< Field RXIPV4HEROIM - MMC Receive IPV4 Header Error Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_hdrerr_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4HEROIM_SHIFT 17
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4HEROIM_MASK 0x20000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4HEROIM(v) (((v) << 17) & 0x20000)
/**< Field RXIPV4NOPAYOIM - MMC Receive IPV4 No Payload Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_nopay_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4NOPAYOIM_SHIFT 18
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4NOPAYOIM_MASK 0x40000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4NOPAYOIM(v) (((v) << 18) & 0x40000)
/**< Field RXIPV4FRAGOIM - MMC Receive IPV4 Fragmented Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_frag_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4FRAGOIM_SHIFT 19
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4FRAGOIM_MASK 0x80000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4FRAGOIM(v) (((v) << 19) & 0x80000)
/**< Field RXIPV4UDSBLOIM - MMC Receive IPV4 UDP Checksum Disabled Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv4_udsbl_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4UDSBLOIM_SHIFT 20
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4UDSBLOIM_MASK 0x100000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV4UDSBLOIM(v) (((v) << 20) & 0x100000)
/**< Field RXIPV6GOIM - MMC Receive IPV6 Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_gd_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6GOIM_SHIFT 21
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6GOIM_MASK 0x200000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6GOIM(v) (((v) << 21) & 0x200000)
/**< Field RXIPV6HEROIM - MMC Receive IPV6 Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_hdrerr_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6HEROIM_SHIFT 22
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6HEROIM_MASK 0x400000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6HEROIM(v) (((v) << 22) & 0x400000)
/**< Field RXIPV6NOPAYOIM - MMC Receive IPV6 Header Error Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxipv6_nopay_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6NOPAYOIM_SHIFT 23
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6NOPAYOIM_MASK 0x800000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXIPV6NOPAYOIM(v) (((v) << 23) & 0x800000)
/**< Field RXUDPGOIM - MMC Receive IPV6 No Payload Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxudp_gd_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPGOIM_SHIFT 24
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPGOIM_MASK 0x1000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPGOIM(v) (((v) << 24) & 0x1000000)
/**< Field RXUDPEROIM - MMC Receive UDP Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxudp_err_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPEROIM_SHIFT 25
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPEROIM_MASK 0x2000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXUDPEROIM(v) (((v) << 25) & 0x2000000)
/**< Field RXTCPGOIM - MMC Receive TCP Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxtcp_gd_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPGOIM_SHIFT 26
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPGOIM_MASK 0x4000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPGOIM(v) (((v) << 26) & 0x4000000)
/**< Field RXTCPEROIM - MMC Receive TCP Error Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxtcp_err_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPEROIM_SHIFT 27
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPEROIM_MASK 0x8000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXTCPEROIM(v) (((v) << 27) & 0x8000000)
/**< Field RXICMPGOIM - MMC Receive ICMP Good Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxicmp_gd_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPGOIM_SHIFT 28
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPGOIM_MASK 0x10000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPGOIM(v) (((v) << 28) & 0x10000000)
/**< Field RXICMPEROIM - MMC Receive ICMP Error Octet Counter Interrupt Mask Setting this bit masks the interrupt when the rxicmp_err_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPEROIM_SHIFT 29
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPEROIM_MASK 0x20000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_RXICMPEROIM(v) (((v) << 29) & 0x20000000)

/**< IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT - MMC IPC Receive Interrupt */
typedef union IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4GPIS : 1;                 /**< MMC Receive IPV4 Good Packet Counter Interrupt Status This bit is set when the rxipv4_gd_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4HERPIS : 1;                /**< MMC Receive IPV4 Header Error Packet Counter Interrupt Status This bit is set when the rxipv4_hdrerr_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4NOPAYPIS : 1;              /**< MMC Receive IPV4 No Payload Packet Counter Interrupt Status This bit is set when the rxipv4_nopay_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4FRAGPIS : 1;               /**< MMC Receive IPV4 Fragmented Packet Counter Interrupt Status This bit is set when the rxipv4_frag_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4UDSBLPIS : 1;              /**< MMC Receive IPV4 UDP Checksum Disabled Packet Counter Interrupt Status This bit is set when the rxipv4_udsbl_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6GPIS : 1;                 /**< MMC Receive IPV6 Good Packet Counter Interrupt Status This bit is set when the rxipv6_gd_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6HERPIS : 1;                /**< MMC Receive IPV6 Header Error Packet Counter Interrupt Status This bit is set when the rxipv6_hdrerr_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6NOPAYPIS : 1;              /**< MMC Receive IPV6 No Payload Packet Counter Interrupt Status This bit is set when the rxipv6_nopay_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUDPGPIS : 1;                 /**< MC Receive UDP Good Packet Counter Interrupt Status This bit is set when the rxudp_gd_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUDPERPIS : 1;                 /**< MMC Receive UDP Error Packet Counter Interrupt Status This bit is set when the rxudp_err_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXTCPGPIS : 1;                 /**< MMC Receive TCP Good Packet Counter Interrupt Status This bit is set when the rxtcp_gd_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXTCPERPIS : 1;                 /**< MMC Receive TCP Error Packet Counter Interrupt Status This bit is set when the rxtcp_err_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXICMPGPIS : 1;                 /**< MMC Receive ICMP Good Packet Counter Interrupt Status This bit is set when the rxicmp_gd_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 RXICMPERPIS : 1;                 /**< MMC Receive ICMP Error Packet Counter Interrupt Status This bit is set when the rxicmp_err_pkts counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused14 : 2;                 /**< Reserved. */
        UINT32 RXIPV4GOIS : 1;                 /**< MMC Receive IPV4 Good Octet Counter Interrupt Status This bit is set when the rxipv4_gd_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4HEROIS : 1;                /**< MMC Receive IPV4 Header Error Octet Counter Interrupt Status This bit is set when the rxipv4_hdrerr_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4NOPAYOIS : 1;              /**< MMC Receive IPV4 No Payload Octet Counter Interrupt Status This bit is set when the rxipv4_nopay_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4FRAGOIS : 1;               /**< MMC Receive IPV4 Fragmented Octet Counter Interrupt Status This bit is set when the rxipv4_frag_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV4UDSBLOIS : 1;              /**< MMC Receive IPV4 UDP Checksum Disabled Octet Counter Interrupt Status This bit is set when the rxipv4_udsbl_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6GOIS : 1;                 /**< MMC Receive IPV6 Good Octet Counter Interrupt Status This bit is set when the rxipv6_gd_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6HEROIS : 1;                /**< MMC Receive IPV6 Header Error Octet Counter Interrupt Status This bit is set when the rxipv6_hdrerr_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXIPV6NOPAYOIS : 1;              /**< MMC Receive IPV6 No Payload Octet Counter Interrupt Status This bit is set when the rxipv6_nopay_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUDPGOIS : 1;                 /**< MMC Receive UDP Good Octet Counter Interrupt Status This bit is set when the rxudp_gd_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXUDPEROIS : 1;                 /**< MMC Receive UDP Error Octet Counter Interrupt Status This bit is set when the rxudp_err_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXTCPGOIS : 1;                 /**< MMC Receive TCP Good Octet Counter Interrupt Status This bit is set when the rxtcp_gd_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXTCPEROIS : 1;                 /**< MMC Receive TCP Error Octet Counter Interrupt Status This bit is set when the rxtcp_err_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXICMPGOIS : 1;                 /**< MMC Receive ICMP Good Octet Counter Interrupt Status This bit is set when the rxicmp_gd_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 RXICMPEROIS : 1;                 /**< MMC Receive ICMP Error Octet Counter Interrupt Status This bit is set when the rxicmp_err_octets counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused30 : 2;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_t;

/**< Register MAC_MMC_IPC_RX_INTERRUPT - MMC IPC Receive Interrupt */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT    0x808
/**< Field RXIPV4GPIS - MMC Receive IPV4 Good Packet Counter Interrupt Status This bit is set when the rxipv4_gd_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4GPIS_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4GPIS_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4GPIS(v) (((v) << 0) & 0x1)
/**< Field RXIPV4HERPIS - MMC Receive IPV4 Header Error Packet Counter Interrupt Status This bit is set when the rxipv4_hdrerr_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4HERPIS_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4HERPIS_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4HERPIS(v) (((v) << 1) & 0x2)
/**< Field RXIPV4NOPAYPIS - MMC Receive IPV4 No Payload Packet Counter Interrupt Status This bit is set when the rxipv4_nopay_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4NOPAYPIS_SHIFT 2
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4NOPAYPIS_MASK 0x4
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4NOPAYPIS(v) (((v) << 2) & 0x4)
/**< Field RXIPV4FRAGPIS - MMC Receive IPV4 Fragmented Packet Counter Interrupt Status This bit is set when the rxipv4_frag_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4FRAGPIS_SHIFT 3
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4FRAGPIS_MASK 0x8
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4FRAGPIS(v) (((v) << 3) & 0x8)
/**< Field RXIPV4UDSBLPIS - MMC Receive IPV4 UDP Checksum Disabled Packet Counter Interrupt Status This bit is set when the rxipv4_udsbl_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4UDSBLPIS_SHIFT 4
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4UDSBLPIS_MASK 0x10
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4UDSBLPIS(v) (((v) << 4) & 0x10)
/**< Field RXIPV6GPIS - MMC Receive IPV6 Good Packet Counter Interrupt Status This bit is set when the rxipv6_gd_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6GPIS_SHIFT 5
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6GPIS_MASK 0x20
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6GPIS(v) (((v) << 5) & 0x20)
/**< Field RXIPV6HERPIS - MMC Receive IPV6 Header Error Packet Counter Interrupt Status This bit is set when the rxipv6_hdrerr_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6HERPIS_SHIFT 6
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6HERPIS_MASK 0x40
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6HERPIS(v) (((v) << 6) & 0x40)
/**< Field RXIPV6NOPAYPIS - MMC Receive IPV6 No Payload Packet Counter Interrupt Status This bit is set when the rxipv6_nopay_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6NOPAYPIS_SHIFT 7
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6NOPAYPIS_MASK 0x80
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6NOPAYPIS(v) (((v) << 7) & 0x80)
/**< Field RXUDPGPIS - MC Receive UDP Good Packet Counter Interrupt Status This bit is set when the rxudp_gd_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPGPIS_SHIFT 8
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPGPIS_MASK 0x100
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPGPIS(v) (((v) << 8) & 0x100)
/**< Field RXUDPERPIS - MMC Receive UDP Error Packet Counter Interrupt Status This bit is set when the rxudp_err_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPERPIS_SHIFT 9
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPERPIS_MASK 0x200
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPERPIS(v) (((v) << 9) & 0x200)
/**< Field RXTCPGPIS - MMC Receive TCP Good Packet Counter Interrupt Status This bit is set when the rxtcp_gd_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPGPIS_SHIFT 10
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPGPIS_MASK 0x400
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPGPIS(v) (((v) << 10) & 0x400)
/**< Field RXTCPERPIS - MMC Receive TCP Error Packet Counter Interrupt Status This bit is set when the rxtcp_err_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPERPIS_SHIFT 11
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPERPIS_MASK 0x800
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPERPIS(v) (((v) << 11) & 0x800)
/**< Field RXICMPGPIS - MMC Receive ICMP Good Packet Counter Interrupt Status This bit is set when the rxicmp_gd_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPGPIS_SHIFT 12
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPGPIS_MASK 0x1000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPGPIS(v) (((v) << 12) & 0x1000)
/**< Field RXICMPERPIS - MMC Receive ICMP Error Packet Counter Interrupt Status This bit is set when the rxicmp_err_pkts counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPERPIS_SHIFT 13
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPERPIS_MASK 0x2000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPERPIS(v) (((v) << 13) & 0x2000)
/**< Field RXIPV4GOIS - MMC Receive IPV4 Good Octet Counter Interrupt Status This bit is set when the rxipv4_gd_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4GOIS_SHIFT 16
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4GOIS_MASK 0x10000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4GOIS(v) (((v) << 16) & 0x10000)
/**< Field RXIPV4HEROIS - MMC Receive IPV4 Header Error Octet Counter Interrupt Status This bit is set when the rxipv4_hdrerr_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4HEROIS_SHIFT 17
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4HEROIS_MASK 0x20000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4HEROIS(v) (((v) << 17) & 0x20000)
/**< Field RXIPV4NOPAYOIS - MMC Receive IPV4 No Payload Octet Counter Interrupt Status This bit is set when the rxipv4_nopay_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4NOPAYOIS_SHIFT 18
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4NOPAYOIS_MASK 0x40000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4NOPAYOIS(v) (((v) << 18) & 0x40000)
/**< Field RXIPV4FRAGOIS - MMC Receive IPV4 Fragmented Octet Counter Interrupt Status This bit is set when the rxipv4_frag_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4FRAGOIS_SHIFT 19
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4FRAGOIS_MASK 0x80000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4FRAGOIS(v) (((v) << 19) & 0x80000)
/**< Field RXIPV4UDSBLOIS - MMC Receive IPV4 UDP Checksum Disabled Octet Counter Interrupt Status This bit is set when the rxipv4_udsbl_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4UDSBLOIS_SHIFT 20
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4UDSBLOIS_MASK 0x100000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV4UDSBLOIS(v) (((v) << 20) & 0x100000)
/**< Field RXIPV6GOIS - MMC Receive IPV6 Good Octet Counter Interrupt Status This bit is set when the rxipv6_gd_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6GOIS_SHIFT 21
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6GOIS_MASK 0x200000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6GOIS(v) (((v) << 21) & 0x200000)
/**< Field RXIPV6HEROIS - MMC Receive IPV6 Header Error Octet Counter Interrupt Status This bit is set when the rxipv6_hdrerr_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6HEROIS_SHIFT 22
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6HEROIS_MASK 0x400000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6HEROIS(v) (((v) << 22) & 0x400000)
/**< Field RXIPV6NOPAYOIS - MMC Receive IPV6 No Payload Octet Counter Interrupt Status This bit is set when the rxipv6_nopay_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6NOPAYOIS_SHIFT 23
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6NOPAYOIS_MASK 0x800000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXIPV6NOPAYOIS(v) (((v) << 23) & 0x800000)
/**< Field RXUDPGOIS - MMC Receive UDP Good Octet Counter Interrupt Status This bit is set when the rxudp_gd_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPGOIS_SHIFT 24
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPGOIS_MASK 0x1000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPGOIS(v) (((v) << 24) & 0x1000000)
/**< Field RXUDPEROIS - MMC Receive UDP Error Octet Counter Interrupt Status This bit is set when the rxudp_err_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPEROIS_SHIFT 25
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPEROIS_MASK 0x2000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXUDPEROIS(v) (((v) << 25) & 0x2000000)
/**< Field RXTCPGOIS - MMC Receive TCP Good Octet Counter Interrupt Status This bit is set when the rxtcp_gd_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPGOIS_SHIFT 26
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPGOIS_MASK 0x4000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPGOIS(v) (((v) << 26) & 0x4000000)
/**< Field RXTCPEROIS - MMC Receive TCP Error Octet Counter Interrupt Status This bit is set when the rxtcp_err_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPEROIS_SHIFT 27
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPEROIS_MASK 0x8000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXTCPEROIS(v) (((v) << 27) & 0x8000000)
/**< Field RXICMPGOIS - MMC Receive ICMP Good Octet Counter Interrupt Status This bit is set when the rxicmp_gd_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPGOIS_SHIFT 28
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPGOIS_MASK 0x10000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPGOIS(v) (((v) << 28) & 0x10000000)
/**< Field RXICMPEROIS - MMC Receive ICMP Error Octet Counter Interrupt Status This bit is set when the rxicmp_err_octets counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPEROIS_SHIFT 29
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPEROIS_MASK 0x20000000
#define IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_RXICMPEROIS(v) (((v) << 29) & 0x20000000)

/**< IMX_ENET_QOS_MAC_RXIPV4_GOOD_PACKETS - Good IPv4 Datagrams Received */
typedef union IMX_ENET_QOS_MAC_RXIPV4_GOOD_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4GDPKT : 32;                /**< RxIPv4 Good Packets This field indicates the number of good IPv4 datagrams received with the TCP, UDP, or ICMP payload. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV4_GOOD_PACKETS_t;

/**< Register MAC_RXIPV4_GOOD_PACKETS - Good IPv4 Datagrams Received */
#define IMX_ENET_QOS_MAC_RXIPV4_GOOD_PACKETS     0x810
/**< Field RXIPV4GDPKT - RxIPv4 Good Packets This field indicates the number of good IPv4 datagrams received with the TCP, UDP, or ICMP payload. */
#define IMX_ENET_QOS_MAC_RXIPV4_GOOD_PACKETS_RXIPV4GDPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV4_GOOD_PACKETS_RXIPV4GDPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV4_GOOD_PACKETS_RXIPV4GDPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_PACKETS - IPv4 Datagrams Received with Header Errors */
typedef union IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4HDRERRPKT : 32;            /**< RxIPv4 Header Error Packets This field indicates the number of IPv4 datagrams received with header (checksum, length, or version mismatch) errors. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_PACKETS_t;

/**< Register MAC_RXIPV4_HEADER_ERROR_PACKETS - IPv4 Datagrams Received with Header Errors */
#define IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_PACKETS 0x814
/**< Field RXIPV4HDRERRPKT - RxIPv4 Header Error Packets This field indicates the number of IPv4 datagrams received with header (checksum, length, or version mismatch) errors. */
#define IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_PACKETS_RXIPV4HDRERRPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_PACKETS_RXIPV4HDRERRPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_PACKETS_RXIPV4HDRERRPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_PACKETS - IPv4 Datagrams Received with No Payload */
typedef union IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4NOPAYPKT : 32;             /**< RxIPv4 Payload Packets This field indicates the number of IPv4 datagram packets received that did not have a TCP, UDP, or ICMP payload. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_PACKETS_t;

/**< Register MAC_RXIPV4_NO_PAYLOAD_PACKETS - IPv4 Datagrams Received with No Payload */
#define IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_PACKETS 0x818
/**< Field RXIPV4NOPAYPKT - RxIPv4 Payload Packets This field indicates the number of IPv4 datagram packets received that did not have a TCP, UDP, or ICMP payload. */
#define IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_PACKETS_RXIPV4NOPAYPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_PACKETS_RXIPV4NOPAYPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_PACKETS_RXIPV4NOPAYPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_PACKETS - IPv4 Datagrams Received with Fragmentation */
typedef union IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4FRAGPKT : 32;              /**< RxIPv4 Fragmented Packets This field indicates the number of good IPv4 datagrams received with fragmentation. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_PACKETS_t;

/**< Register MAC_RXIPV4_FRAGMENTED_PACKETS - IPv4 Datagrams Received with Fragmentation */
#define IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_PACKETS 0x81C
/**< Field RXIPV4FRAGPKT - RxIPv4 Fragmented Packets This field indicates the number of good IPv4 datagrams received with fragmentation. */
#define IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_PACKETS_RXIPV4FRAGPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_PACKETS_RXIPV4FRAGPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_PACKETS_RXIPV4FRAGPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS - IPv4 Datagrams Received with UDP Checksum Disabled */
typedef union IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4UDSBLPKT : 32;             /**< RxIPv4 UDP Checksum Disabled Packets This field indicates the number of good IPv4 datagrams received that had a UDP payload with checksum disabled. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS_t;

/**< Register MAC_RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS - IPv4 Datagrams Received with UDP Checksum Disabled */
#define IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS 0x820
/**< Field RXIPV4UDSBLPKT - RxIPv4 UDP Checksum Disabled Packets This field indicates the number of good IPv4 datagrams received that had a UDP payload with checksum disabled. */
#define IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS_RXIPV4UDSBLPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS_RXIPV4UDSBLPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS_RXIPV4UDSBLPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV6_GOOD_PACKETS - Good IPv6 Datagrams Received */
typedef union IMX_ENET_QOS_MAC_RXIPV6_GOOD_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV6GDPKT : 32;                /**< RxIPv6 Good Packets This field indicates the number of good IPv6 datagrams received with the TCP, UDP, or ICMP payload. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV6_GOOD_PACKETS_t;

/**< Register MAC_RXIPV6_GOOD_PACKETS - Good IPv6 Datagrams Received */
#define IMX_ENET_QOS_MAC_RXIPV6_GOOD_PACKETS     0x824
/**< Field RXIPV6GDPKT - RxIPv6 Good Packets This field indicates the number of good IPv6 datagrams received with the TCP, UDP, or ICMP payload. */
#define IMX_ENET_QOS_MAC_RXIPV6_GOOD_PACKETS_RXIPV6GDPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV6_GOOD_PACKETS_RXIPV6GDPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV6_GOOD_PACKETS_RXIPV6GDPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_PACKETS - IPv6 Datagrams Received with Header Errors */
typedef union IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV6HDRERRPKT : 32;            /**< RxIPv6 Header Error Packets This field indicates the number of IPv6 datagrams received with header (length or version mismatch) errors. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_PACKETS_t;

/**< Register MAC_RXIPV6_HEADER_ERROR_PACKETS - IPv6 Datagrams Received with Header Errors */
#define IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_PACKETS 0x828
/**< Field RXIPV6HDRERRPKT - RxIPv6 Header Error Packets This field indicates the number of IPv6 datagrams received with header (length or version mismatch) errors. */
#define IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_PACKETS_RXIPV6HDRERRPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_PACKETS_RXIPV6HDRERRPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_PACKETS_RXIPV6HDRERRPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_PACKETS - IPv6 Datagrams Received with No Payload */
typedef union IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV6NOPAYPKT : 32;             /**< RxIPv6 Payload Packets This field indicates the number of IPv6 datagram packets received that did not have a TCP, UDP, or ICMP payload. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_PACKETS_t;

/**< Register MAC_RXIPV6_NO_PAYLOAD_PACKETS - IPv6 Datagrams Received with No Payload */
#define IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_PACKETS 0x82C
/**< Field RXIPV6NOPAYPKT - RxIPv6 Payload Packets This field indicates the number of IPv6 datagram packets received that did not have a TCP, UDP, or ICMP payload. */
#define IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_PACKETS_RXIPV6NOPAYPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_PACKETS_RXIPV6NOPAYPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_PACKETS_RXIPV6NOPAYPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXUDP_GOOD_PACKETS - IPv6 Datagrams Received with Good UDP */
typedef union IMX_ENET_QOS_MAC_RXUDP_GOOD_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXUDPGDPKT : 32;                /**< RxUDP Good Packets This field indicates the number of good IP datagrams received with a good UDP payload. */
    } B;
} IMX_ENET_QOS_MAC_RXUDP_GOOD_PACKETS_t;

/**< Register MAC_RXUDP_GOOD_PACKETS - IPv6 Datagrams Received with Good UDP */
#define IMX_ENET_QOS_MAC_RXUDP_GOOD_PACKETS      0x830
/**< Field RXUDPGDPKT - RxUDP Good Packets This field indicates the number of good IP datagrams received with a good UDP payload. */
#define IMX_ENET_QOS_MAC_RXUDP_GOOD_PACKETS_RXUDPGDPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXUDP_GOOD_PACKETS_RXUDPGDPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXUDP_GOOD_PACKETS_RXUDPGDPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXUDP_ERROR_PACKETS - IPv6 Datagrams Received with UDP Checksum Error */
typedef union IMX_ENET_QOS_MAC_RXUDP_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXUDPERRPKT : 32;                /**< RxUDP Error Packets This field indicates the number of good IP datagrams received whose UDP payload has a checksum error. */
    } B;
} IMX_ENET_QOS_MAC_RXUDP_ERROR_PACKETS_t;

/**< Register MAC_RXUDP_ERROR_PACKETS - IPv6 Datagrams Received with UDP Checksum Error */
#define IMX_ENET_QOS_MAC_RXUDP_ERROR_PACKETS     0x834
/**< Field RXUDPERRPKT - RxUDP Error Packets This field indicates the number of good IP datagrams received whose UDP payload has a checksum error. */
#define IMX_ENET_QOS_MAC_RXUDP_ERROR_PACKETS_RXUDPERRPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXUDP_ERROR_PACKETS_RXUDPERRPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXUDP_ERROR_PACKETS_RXUDPERRPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXTCP_GOOD_PACKETS - IPv6 Datagrams Received with Good TCP Payload */
typedef union IMX_ENET_QOS_MAC_RXTCP_GOOD_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXTCPGDPKT : 32;                /**< RxTCP Good Packets This field indicates the number of good IP datagrams received with a good TCP payload. */
    } B;
} IMX_ENET_QOS_MAC_RXTCP_GOOD_PACKETS_t;

/**< Register MAC_RXTCP_GOOD_PACKETS - IPv6 Datagrams Received with Good TCP Payload */
#define IMX_ENET_QOS_MAC_RXTCP_GOOD_PACKETS      0x838
/**< Field RXTCPGDPKT - RxTCP Good Packets This field indicates the number of good IP datagrams received with a good TCP payload. */
#define IMX_ENET_QOS_MAC_RXTCP_GOOD_PACKETS_RXTCPGDPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXTCP_GOOD_PACKETS_RXTCPGDPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXTCP_GOOD_PACKETS_RXTCPGDPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXTCP_ERROR_PACKETS - IPv6 Datagrams Received with TCP Checksum Error */
typedef union IMX_ENET_QOS_MAC_RXTCP_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXTCPERRPKT : 32;                /**< RxTCP Error Packets This field indicates the number of good IP datagrams received whose TCP payload has a checksum error. */
    } B;
} IMX_ENET_QOS_MAC_RXTCP_ERROR_PACKETS_t;

/**< Register MAC_RXTCP_ERROR_PACKETS - IPv6 Datagrams Received with TCP Checksum Error */
#define IMX_ENET_QOS_MAC_RXTCP_ERROR_PACKETS     0x83C
/**< Field RXTCPERRPKT - RxTCP Error Packets This field indicates the number of good IP datagrams received whose TCP payload has a checksum error. */
#define IMX_ENET_QOS_MAC_RXTCP_ERROR_PACKETS_RXTCPERRPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXTCP_ERROR_PACKETS_RXTCPERRPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXTCP_ERROR_PACKETS_RXTCPERRPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXICMP_GOOD_PACKETS - IPv6 Datagrams Received with Good ICMP Payload */
typedef union IMX_ENET_QOS_MAC_RXICMP_GOOD_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXICMPGDPKT : 32;                /**< RxICMP Good Packets This field indicates the number of good IP datagrams received with a good ICMP payload. */
    } B;
} IMX_ENET_QOS_MAC_RXICMP_GOOD_PACKETS_t;

/**< Register MAC_RXICMP_GOOD_PACKETS - IPv6 Datagrams Received with Good ICMP Payload */
#define IMX_ENET_QOS_MAC_RXICMP_GOOD_PACKETS     0x840
/**< Field RXICMPGDPKT - RxICMP Good Packets This field indicates the number of good IP datagrams received with a good ICMP payload. */
#define IMX_ENET_QOS_MAC_RXICMP_GOOD_PACKETS_RXICMPGDPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXICMP_GOOD_PACKETS_RXICMPGDPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXICMP_GOOD_PACKETS_RXICMPGDPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXICMP_ERROR_PACKETS - IPv6 Datagrams Received with ICMP Checksum Error */
typedef union IMX_ENET_QOS_MAC_RXICMP_ERROR_PACKETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXICMPERRPKT : 32;               /**< RxICMP Error Packets This field indicates the number of good IP datagrams received whose ICMP payload has a checksum error. */
    } B;
} IMX_ENET_QOS_MAC_RXICMP_ERROR_PACKETS_t;

/**< Register MAC_RXICMP_ERROR_PACKETS - IPv6 Datagrams Received with ICMP Checksum Error */
#define IMX_ENET_QOS_MAC_RXICMP_ERROR_PACKETS    0x844
/**< Field RXICMPERRPKT - RxICMP Error Packets This field indicates the number of good IP datagrams received whose ICMP payload has a checksum error. */
#define IMX_ENET_QOS_MAC_RXICMP_ERROR_PACKETS_RXICMPERRPKT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXICMP_ERROR_PACKETS_RXICMPERRPKT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXICMP_ERROR_PACKETS_RXICMPERRPKT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV4_GOOD_OCTETS - Good Bytes Received in IPv4 Datagrams */
typedef union IMX_ENET_QOS_MAC_RXIPV4_GOOD_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4GDOCT : 32;                /**< RxIPv4 Good Octets This field indicates the number of bytes received in good IPv4 datagrams encapsulating TCP, UDP, or ICMP data. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV4_GOOD_OCTETS_t;

/**< Register MAC_RXIPV4_GOOD_OCTETS - Good Bytes Received in IPv4 Datagrams */
#define IMX_ENET_QOS_MAC_RXIPV4_GOOD_OCTETS      0x850
/**< Field RXIPV4GDOCT - RxIPv4 Good Octets This field indicates the number of bytes received in good IPv4 datagrams encapsulating TCP, UDP, or ICMP data. */
#define IMX_ENET_QOS_MAC_RXIPV4_GOOD_OCTETS_RXIPV4GDOCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV4_GOOD_OCTETS_RXIPV4GDOCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV4_GOOD_OCTETS_RXIPV4GDOCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_OCTETS - Bytes Received in IPv4 Datagrams with Header Errors */
typedef union IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4HDRERROCT : 32;            /**< RxIPv4 Header Error Octets This field indicates the number of bytes received in IPv4 datagrams with header errors (checksum, length, version mismatch). */
    } B;
} IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_OCTETS_t;

/**< Register MAC_RXIPV4_HEADER_ERROR_OCTETS - Bytes Received in IPv4 Datagrams with Header Errors */
#define IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_OCTETS 0x854
/**< Field RXIPV4HDRERROCT - RxIPv4 Header Error Octets This field indicates the number of bytes received in IPv4 datagrams with header errors (checksum, length, version mismatch). */
#define IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_OCTETS_RXIPV4HDRERROCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_OCTETS_RXIPV4HDRERROCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_OCTETS_RXIPV4HDRERROCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_OCTETS - Bytes Received in IPv4 Datagrams with No Payload */
typedef union IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4NOPAYOCT : 32;             /**< RxIPv4 Payload Octets This field indicates the number of bytes received in IPv4 datagrams that did not have a TCP, UDP, or ICMP payload. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_OCTETS_t;

/**< Register MAC_RXIPV4_NO_PAYLOAD_OCTETS - Bytes Received in IPv4 Datagrams with No Payload */
#define IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_OCTETS 0x858
/**< Field RXIPV4NOPAYOCT - RxIPv4 Payload Octets This field indicates the number of bytes received in IPv4 datagrams that did not have a TCP, UDP, or ICMP payload. */
#define IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_OCTETS_RXIPV4NOPAYOCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_OCTETS_RXIPV4NOPAYOCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_OCTETS_RXIPV4NOPAYOCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_OCTETS - Bytes Received in Fragmented IPv4 Datagrams */
typedef union IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4FRAGOCT : 32;              /**< RxIPv4 Fragmented Octets This field indicates the number of bytes received in fragmented IPv4 datagrams. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_OCTETS_t;

/**< Register MAC_RXIPV4_FRAGMENTED_OCTETS - Bytes Received in Fragmented IPv4 Datagrams */
#define IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_OCTETS 0x85C
/**< Field RXIPV4FRAGOCT - RxIPv4 Fragmented Octets This field indicates the number of bytes received in fragmented IPv4 datagrams. */
#define IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_OCTETS_RXIPV4FRAGOCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_OCTETS_RXIPV4FRAGOCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_OCTETS_RXIPV4FRAGOCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS - Bytes Received with UDP Checksum Disabled */
typedef union IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV4UDSBLOCT : 32;             /**< RxIPv4 UDP Checksum Disable Octets This field indicates the number of bytes received in a UDP segment that had the UDP checksum disabled. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS_t;

/**< Register MAC_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS - Bytes Received with UDP Checksum Disabled */
#define IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS 0x860
/**< Field RXIPV4UDSBLOCT - RxIPv4 UDP Checksum Disable Octets This field indicates the number of bytes received in a UDP segment that had the UDP checksum disabled. */
#define IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS_RXIPV4UDSBLOCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS_RXIPV4UDSBLOCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS_RXIPV4UDSBLOCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV6_GOOD_OCTETS - Bytes Received in Good IPv6 Datagrams */
typedef union IMX_ENET_QOS_MAC_RXIPV6_GOOD_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV6GDOCT : 32;                /**< RxIPv6 Good Octets This field indicates the number of bytes received in good IPv6 datagrams encapsulating TCP, UDP, or ICMP data. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV6_GOOD_OCTETS_t;

/**< Register MAC_RXIPV6_GOOD_OCTETS - Bytes Received in Good IPv6 Datagrams */
#define IMX_ENET_QOS_MAC_RXIPV6_GOOD_OCTETS      0x864
/**< Field RXIPV6GDOCT - RxIPv6 Good Octets This field indicates the number of bytes received in good IPv6 datagrams encapsulating TCP, UDP, or ICMP data. */
#define IMX_ENET_QOS_MAC_RXIPV6_GOOD_OCTETS_RXIPV6GDOCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV6_GOOD_OCTETS_RXIPV6GDOCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV6_GOOD_OCTETS_RXIPV6GDOCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_OCTETS - Bytes Received in IPv6 Datagrams with Data Errors */
typedef union IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV6HDRERROCT : 32;            /**< RxIPv6 Header Error Octets This field indicates the number of bytes received in IPv6 datagrams with header errors (length, version mismatch). */
    } B;
} IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_OCTETS_t;

/**< Register MAC_RXIPV6_HEADER_ERROR_OCTETS - Bytes Received in IPv6 Datagrams with Data Errors */
#define IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_OCTETS 0x868
/**< Field RXIPV6HDRERROCT - RxIPv6 Header Error Octets This field indicates the number of bytes received in IPv6 datagrams with header errors (length, version mismatch). */
#define IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_OCTETS_RXIPV6HDRERROCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_OCTETS_RXIPV6HDRERROCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_OCTETS_RXIPV6HDRERROCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_OCTETS - Bytes Received in IPv6 Datagrams with No Payload */
typedef union IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXIPV6NOPAYOCT : 32;             /**< RxIPv6 Payload Octets This field indicates the number of bytes received in IPv6 datagrams that did not have a TCP, UDP, or ICMP payload. */
    } B;
} IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_OCTETS_t;

/**< Register MAC_RXIPV6_NO_PAYLOAD_OCTETS - Bytes Received in IPv6 Datagrams with No Payload */
#define IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_OCTETS 0x86C
/**< Field RXIPV6NOPAYOCT - RxIPv6 Payload Octets This field indicates the number of bytes received in IPv6 datagrams that did not have a TCP, UDP, or ICMP payload. */
#define IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_OCTETS_RXIPV6NOPAYOCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_OCTETS_RXIPV6NOPAYOCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_OCTETS_RXIPV6NOPAYOCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXUDP_GOOD_OCTETS - Bytes Received in Good UDP Segment */
typedef union IMX_ENET_QOS_MAC_RXUDP_GOOD_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXUDPGDOCT : 32;                /**< RxUDP Good Octets This field indicates the number of bytes received in a good UDP segment. */
    } B;
} IMX_ENET_QOS_MAC_RXUDP_GOOD_OCTETS_t;

/**< Register MAC_RXUDP_GOOD_OCTETS - Bytes Received in Good UDP Segment */
#define IMX_ENET_QOS_MAC_RXUDP_GOOD_OCTETS       0x870
/**< Field RXUDPGDOCT - RxUDP Good Octets This field indicates the number of bytes received in a good UDP segment. */
#define IMX_ENET_QOS_MAC_RXUDP_GOOD_OCTETS_RXUDPGDOCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXUDP_GOOD_OCTETS_RXUDPGDOCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXUDP_GOOD_OCTETS_RXUDPGDOCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXUDP_ERROR_OCTETS - Bytes Received in UDP Segment with Checksum Errors */
typedef union IMX_ENET_QOS_MAC_RXUDP_ERROR_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXUDPERROCT : 32;                /**< RxUDP Error Octets This field indicates the number of bytes received in a UDP segment that had checksum errors. */
    } B;
} IMX_ENET_QOS_MAC_RXUDP_ERROR_OCTETS_t;

/**< Register MAC_RXUDP_ERROR_OCTETS - Bytes Received in UDP Segment with Checksum Errors */
#define IMX_ENET_QOS_MAC_RXUDP_ERROR_OCTETS      0x874
/**< Field RXUDPERROCT - RxUDP Error Octets This field indicates the number of bytes received in a UDP segment that had checksum errors. */
#define IMX_ENET_QOS_MAC_RXUDP_ERROR_OCTETS_RXUDPERROCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXUDP_ERROR_OCTETS_RXUDPERROCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXUDP_ERROR_OCTETS_RXUDPERROCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXTCP_GOOD_OCTETS - Bytes Received in Good TCP Segment */
typedef union IMX_ENET_QOS_MAC_RXTCP_GOOD_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXTCPGDOCT : 32;                /**< RxTCP Good Octets This field indicates the number of bytes received in a good TCP segment. */
    } B;
} IMX_ENET_QOS_MAC_RXTCP_GOOD_OCTETS_t;

/**< Register MAC_RXTCP_GOOD_OCTETS - Bytes Received in Good TCP Segment */
#define IMX_ENET_QOS_MAC_RXTCP_GOOD_OCTETS       0x878
/**< Field RXTCPGDOCT - RxTCP Good Octets This field indicates the number of bytes received in a good TCP segment. */
#define IMX_ENET_QOS_MAC_RXTCP_GOOD_OCTETS_RXTCPGDOCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXTCP_GOOD_OCTETS_RXTCPGDOCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXTCP_GOOD_OCTETS_RXTCPGDOCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXTCP_ERROR_OCTETS - Bytes Received in TCP Segment with Checksum Errors */
typedef union IMX_ENET_QOS_MAC_RXTCP_ERROR_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXTCPERROCT : 32;                /**< RxTCP Error Octets This field indicates the number of bytes received in a TCP segment that had checksum errors. */
    } B;
} IMX_ENET_QOS_MAC_RXTCP_ERROR_OCTETS_t;

/**< Register MAC_RXTCP_ERROR_OCTETS - Bytes Received in TCP Segment with Checksum Errors */
#define IMX_ENET_QOS_MAC_RXTCP_ERROR_OCTETS      0x87C
/**< Field RXTCPERROCT - RxTCP Error Octets This field indicates the number of bytes received in a TCP segment that had checksum errors. */
#define IMX_ENET_QOS_MAC_RXTCP_ERROR_OCTETS_RXTCPERROCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXTCP_ERROR_OCTETS_RXTCPERROCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXTCP_ERROR_OCTETS_RXTCPERROCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXICMP_GOOD_OCTETS - Bytes Received in Good ICMP Segment */
typedef union IMX_ENET_QOS_MAC_RXICMP_GOOD_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXICMPGDOCT : 32;                /**< RxICMP Good Octets This field indicates the number of bytes received in a good ICMP segment. */
    } B;
} IMX_ENET_QOS_MAC_RXICMP_GOOD_OCTETS_t;

/**< Register MAC_RXICMP_GOOD_OCTETS - Bytes Received in Good ICMP Segment */
#define IMX_ENET_QOS_MAC_RXICMP_GOOD_OCTETS      0x880
/**< Field RXICMPGDOCT - RxICMP Good Octets This field indicates the number of bytes received in a good ICMP segment. */
#define IMX_ENET_QOS_MAC_RXICMP_GOOD_OCTETS_RXICMPGDOCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXICMP_GOOD_OCTETS_RXICMPGDOCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXICMP_GOOD_OCTETS_RXICMPGDOCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_RXICMP_ERROR_OCTETS - Bytes Received in ICMP Segment with Checksum Errors */
typedef union IMX_ENET_QOS_MAC_RXICMP_ERROR_OCTETS_union_t {
    UINT32 R;
    struct {
        UINT32 RXICMPERROCT : 32;               /**< RxICMP Error Octets This field indicates the number of bytes received in a ICMP segment that had checksum errors. */
    } B;
} IMX_ENET_QOS_MAC_RXICMP_ERROR_OCTETS_t;

/**< Register MAC_RXICMP_ERROR_OCTETS - Bytes Received in ICMP Segment with Checksum Errors */
#define IMX_ENET_QOS_MAC_RXICMP_ERROR_OCTETS     0x884
/**< Field RXICMPERROCT - RxICMP Error Octets This field indicates the number of bytes received in a ICMP segment that had checksum errors. */
#define IMX_ENET_QOS_MAC_RXICMP_ERROR_OCTETS_RXICMPERROCT_SHIFT 0
#define IMX_ENET_QOS_MAC_RXICMP_ERROR_OCTETS_RXICMPERROCT_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_RXICMP_ERROR_OCTETS_RXICMPERROCT(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT - MMC FPE Transmit Interrupt */
typedef union IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_union_t {
    UINT32 R;
    struct {
        UINT32 FCIS : 1;                 /**< MMC Tx FPE Fragment Counter Interrupt status This bit is set when the Tx_FPE_Fragment_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 HRCIS : 1;                 /**< MMC Tx Hold Request Counter Interrupt Status This bit is set when the Tx_Hold_Req_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused2 : 30;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_t;

/**< Register MAC_MMC_FPE_TX_INTERRUPT - MMC FPE Transmit Interrupt */
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT    0x8A0
/**< Field FCIS - MMC Tx FPE Fragment Counter Interrupt status This bit is set when the Tx_FPE_Fragment_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_FCIS_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_FCIS_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_FCIS(v) (((v) << 0) & 0x1)
/**< Field HRCIS - MMC Tx Hold Request Counter Interrupt Status This bit is set when the Tx_Hold_Req_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_HRCIS_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_HRCIS_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_HRCIS(v) (((v) << 1) & 0x2)

/**< IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK - MMC FPE Transmit Mask Interrupt */
typedef union IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK_union_t {
    UINT32 R;
    struct {
        UINT32 FCIM : 1;                 /**< MMC Transmit Fragment Counter Interrupt Mask Setting this bit masks the interrupt when the Tx_FPE_Fragment_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 HRCIM : 1;                 /**< MMC Transmit Hold Request Counter Interrupt Mask Setting this bit masks the interrupt when the Tx_Hold_Req_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused2 : 30;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK_t;

/**< Register MAC_MMC_FPE_TX_INTERRUPT_MASK - MMC FPE Transmit Mask Interrupt */
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK 0x8A4
/**< Field FCIM - MMC Transmit Fragment Counter Interrupt Mask Setting this bit masks the interrupt when the Tx_FPE_Fragment_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK_FCIM_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK_FCIM_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK_FCIM(v) (((v) << 0) & 0x1)
/**< Field HRCIM - MMC Transmit Hold Request Counter Interrupt Mask Setting this bit masks the interrupt when the Tx_Hold_Req_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK_HRCIM_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK_HRCIM_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK_HRCIM(v) (((v) << 1) & 0x2)

/**< IMX_ENET_QOS_MAC_MMC_TX_FPE_FRAGMENT_CNTR - MMC FPE Transmitted Fragment Counter */
typedef union IMX_ENET_QOS_MAC_MMC_TX_FPE_FRAGMENT_CNTR_union_t {
    UINT32 R;
    struct {
        UINT32 TXFFC : 32;                /**< Tx FPE Fragment counter This field indicates the number of additional mPackets that has been transmitted due to preemption Exists when any one of the RX/TX MMC counters are enabled during FPE Enabled configuration. */
    } B;
} IMX_ENET_QOS_MAC_MMC_TX_FPE_FRAGMENT_CNTR_t;

/**< Register MAC_MMC_TX_FPE_FRAGMENT_CNTR - MMC FPE Transmitted Fragment Counter */
#define IMX_ENET_QOS_MAC_MMC_TX_FPE_FRAGMENT_CNTR 0x8A8
/**< Field TXFFC - Tx FPE Fragment counter This field indicates the number of additional mPackets that has been transmitted due to preemption Exists when any one of the RX/TX MMC counters are enabled during FPE Enabled configuration. */
#define IMX_ENET_QOS_MAC_MMC_TX_FPE_FRAGMENT_CNTR_TXFFC_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_TX_FPE_FRAGMENT_CNTR_TXFFC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_MMC_TX_FPE_FRAGMENT_CNTR_TXFFC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_MMC_TX_HOLD_REQ_CNTR - MMC FPE Transmitted Hold Request Counter */
typedef union IMX_ENET_QOS_MAC_MMC_TX_HOLD_REQ_CNTR_union_t {
    UINT32 R;
    struct {
        UINT32 TXHRC : 32;                /**< Tx Hold Request Counter This field indicates count of number of a hold request is given to MAC. */
    } B;
} IMX_ENET_QOS_MAC_MMC_TX_HOLD_REQ_CNTR_t;

/**< Register MAC_MMC_TX_HOLD_REQ_CNTR - MMC FPE Transmitted Hold Request Counter */
#define IMX_ENET_QOS_MAC_MMC_TX_HOLD_REQ_CNTR    0x8AC
/**< Field TXHRC - Tx Hold Request Counter This field indicates count of number of a hold request is given to MAC. */
#define IMX_ENET_QOS_MAC_MMC_TX_HOLD_REQ_CNTR_TXHRC_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_TX_HOLD_REQ_CNTR_TXHRC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_MMC_TX_HOLD_REQ_CNTR_TXHRC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT - MMC FPE Receive Interrupt */
typedef union IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_union_t {
    UINT32 R;
    struct {
        UINT32 PAECIS : 1;                 /**< MMC Rx Packet Assembly Error Counter Interrupt Status This bit is set when the Rx_Packet_Assemble_Err_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 PSECIS : 1;                 /**< MMC Rx Packet SMD Error Counter Interrupt Status This bit is set when the Rx_Packet_SMD_Err_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 PAOCIS : 1;                 /**< MMC Rx Packet Assembly OK Counter Interrupt Status This bit is set when the Rx_Packet_Assemble_Ok_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 FCIS : 1;                 /**< MMC Rx FPE Fragment Counter Interrupt Status This bit is set when the Rx_FPE_Fragment_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused4 : 28;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_t;

/**< Register MAC_MMC_FPE_RX_INTERRUPT - MMC FPE Receive Interrupt */
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT    0x8C0
/**< Field PAECIS - MMC Rx Packet Assembly Error Counter Interrupt Status This bit is set when the Rx_Packet_Assemble_Err_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_PAECIS_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_PAECIS_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_PAECIS(v) (((v) << 0) & 0x1)
/**< Field PSECIS - MMC Rx Packet SMD Error Counter Interrupt Status This bit is set when the Rx_Packet_SMD_Err_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_PSECIS_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_PSECIS_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_PSECIS(v) (((v) << 1) & 0x2)
/**< Field PAOCIS - MMC Rx Packet Assembly OK Counter Interrupt Status This bit is set when the Rx_Packet_Assemble_Ok_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_PAOCIS_SHIFT 2
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_PAOCIS_MASK 0x4
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_PAOCIS(v) (((v) << 2) & 0x4)
/**< Field FCIS - MMC Rx FPE Fragment Counter Interrupt Status This bit is set when the Rx_FPE_Fragment_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_FCIS_SHIFT 3
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_FCIS_MASK 0x8
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_FCIS(v) (((v) << 3) & 0x8)

/**< IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK - MMC FPE Receive Interrupt Mask */
typedef union IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_union_t {
    UINT32 R;
    struct {
        UINT32 PAECIM : 1;                 /**< MMC Rx Packet Assembly Error Counter Interrupt Mask Setting this bit masks the interrupt when the R Rx_Packet_Assemble_Err_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 PSECIM : 1;                 /**< MMC Rx Packet SMD Error Counter Interrupt Mask Setting this bit masks the interrupt when the R Rx_Packet_SMD_Err_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 PAOCIM : 1;                 /**< MMC Rx Packet Assembly OK Counter Interrupt Mask Setting this bit masks the interrupt when the Rx_Packet_Assemble_Ok_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 FCIM : 1;                 /**< MMC Rx FPE Fragment Counter Interrupt Mask Setting this bit masks the interrupt when the Tx_FPE_Fragment_Cntr counter reaches half of the maximum value or the maximum value. */
        UINT32 _unused4 : 28;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_t;

/**< Register MAC_MMC_FPE_RX_INTERRUPT_MASK - MMC FPE Receive Interrupt Mask */
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK 0x8C4
/**< Field PAECIM - MMC Rx Packet Assembly Error Counter Interrupt Mask Setting this bit masks the interrupt when the R Rx_Packet_Assemble_Err_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_PAECIM_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_PAECIM_MASK 0x1
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_PAECIM(v) (((v) << 0) & 0x1)
/**< Field PSECIM - MMC Rx Packet SMD Error Counter Interrupt Mask Setting this bit masks the interrupt when the R Rx_Packet_SMD_Err_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_PSECIM_SHIFT 1
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_PSECIM_MASK 0x2
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_PSECIM(v) (((v) << 1) & 0x2)
/**< Field PAOCIM - MMC Rx Packet Assembly OK Counter Interrupt Mask Setting this bit masks the interrupt when the Rx_Packet_Assemble_Ok_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_PAOCIM_SHIFT 2
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_PAOCIM_MASK 0x4
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_PAOCIM(v) (((v) << 2) & 0x4)
/**< Field FCIM - MMC Rx FPE Fragment Counter Interrupt Mask Setting this bit masks the interrupt when the Tx_FPE_Fragment_Cntr counter reaches half of the maximum value or the maximum value. */
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_FCIM_SHIFT 3
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_FCIM_MASK 0x8
#define IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_FCIM(v) (((v) << 3) & 0x8)

/**< IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_ERR_CNTR - MMC Receive Packet Reassembly Error Counter */
typedef union IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_ERR_CNTR_union_t {
    UINT32 R;
    struct {
        UINT32 PAEC : 32;                /**< Rx Packet Assembly Error Counter This field indicates the number of MAC frames with reassembly errors on the Receiver, due to mismatch in the Fragment Count value. */
    } B;
} IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_ERR_CNTR_t;

/**< Register MAC_MMC_RX_PACKET_ASSEMBLY_ERR_CNTR - MMC Receive Packet Reassembly Error Counter */
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_ERR_CNTR 0x8C8
/**< Field PAEC - Rx Packet Assembly Error Counter This field indicates the number of MAC frames with reassembly errors on the Receiver, due to mismatch in the Fragment Count value. */
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_ERR_CNTR_PAEC_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_ERR_CNTR_PAEC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_ERR_CNTR_PAEC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_MMC_RX_PACKET_SMD_ERR_CNTR - MMC Receive Packet SMD Error Counter */
typedef union IMX_ENET_QOS_MAC_MMC_RX_PACKET_SMD_ERR_CNTR_union_t {
    UINT32 R;
    struct {
        UINT32 PSEC : 32;                /**< Rx Packet SMD Error Counter This field indicates the number of MAC frames rejected due to unknown SMD value and MAC frame fragments rejected due to arriving with an SMD-C when there was no preceding preempted frame. */
    } B;
} IMX_ENET_QOS_MAC_MMC_RX_PACKET_SMD_ERR_CNTR_t;

/**< Register MAC_MMC_RX_PACKET_SMD_ERR_CNTR - MMC Receive Packet SMD Error Counter */
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_SMD_ERR_CNTR 0x8CC
/**< Field PSEC - Rx Packet SMD Error Counter This field indicates the number of MAC frames rejected due to unknown SMD value and MAC frame fragments rejected due to arriving with an SMD-C when there was no preceding preempted frame. */
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_SMD_ERR_CNTR_PSEC_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_SMD_ERR_CNTR_PSEC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_SMD_ERR_CNTR_PSEC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_OK_CNTR - MMC Receive Packet Successful Reassembly Counter */
typedef union IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_OK_CNTR_union_t {
    UINT32 R;
    struct {
        UINT32 PAOC : 32;                /**< Rx Packet Assembly OK Counter This field indicates the number of MAC frames that were successfully reassembled and delivered to MAC. */
    } B;
} IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_OK_CNTR_t;

/**< Register MAC_MMC_RX_PACKET_ASSEMBLY_OK_CNTR - MMC Receive Packet Successful Reassembly Counter */
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_OK_CNTR 0x8D0
/**< Field PAOC - Rx Packet Assembly OK Counter This field indicates the number of MAC frames that were successfully reassembled and delivered to MAC. */
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_OK_CNTR_PAOC_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_OK_CNTR_PAOC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_OK_CNTR_PAOC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_MMC_RX_FPE_FRAGMENT_CNTR - MMC FPE Received Fragment Counter */
typedef union IMX_ENET_QOS_MAC_MMC_RX_FPE_FRAGMENT_CNTR_union_t {
    UINT32 R;
    struct {
        UINT32 FFC : 32;                /**< Rx FPE Fragment Counter This field indicates the number of additional mPackets received due to preemption Exists when at least one of the RX/TX MMC counters are enabled during FPE Enabled configuration. */
    } B;
} IMX_ENET_QOS_MAC_MMC_RX_FPE_FRAGMENT_CNTR_t;

/**< Register MAC_MMC_RX_FPE_FRAGMENT_CNTR - MMC FPE Received Fragment Counter */
#define IMX_ENET_QOS_MAC_MMC_RX_FPE_FRAGMENT_CNTR 0x8D4
/**< Field FFC - Rx FPE Fragment Counter This field indicates the number of additional mPackets received due to preemption Exists when at least one of the RX/TX MMC counters are enabled during FPE Enabled configuration. */
#define IMX_ENET_QOS_MAC_MMC_RX_FPE_FRAGMENT_CNTR_FFC_SHIFT 0
#define IMX_ENET_QOS_MAC_MMC_RX_FPE_FRAGMENT_CNTR_FFC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_MMC_RX_FPE_FRAGMENT_CNTR_FFC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_L3_L4_CONTROL0 - Layer 3 and Layer 4 Control of Filter 0 */
typedef union IMX_ENET_QOS_MAC_L3_L4_CONTROL0_union_t {
    UINT32 R;
    struct {
        UINT32 L3PEN0 : 1;                 /**< Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 L3SAM0 : 1;                 /**< Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
        UINT32 L3SAIM0 : 1;                 /**< Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
        UINT32 L3DAM0 : 1;                 /**< Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
        UINT32 L3DAIM0 : 1;                 /**< Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
        UINT32 L3HSBM0 : 5;                 /**< Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
        UINT32 L3HDBM0 : 5;                 /**< Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
        UINT32 L4PEN0 : 1;                 /**< Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
        UINT32 _unused17 : 1;                 /**< Reserved. */
        UINT32 L4SPM0 : 1;                 /**< Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
        UINT32 L4SPIM0 : 1;                 /**< Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
        UINT32 L4DPM0 : 1;                 /**< Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
        UINT32 L4DPIM0 : 1;                 /**< Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
        UINT32 _unused22 : 2;                 /**< Reserved. */
        UINT32 DMCHN0 : 3;                 /**< DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
        UINT32 _unused27 : 1;                 /**< Reserved. */
        UINT32 DMCHEN0 : 1;                 /**< DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_L3_L4_CONTROL0_t;

/**< Register MAC_L3_L4_CONTROL0 - Layer 3 and Layer 4 Control of Filter 0 */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0          0x900
/**< Field L3PEN0 - Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3PEN0_SHIFT 0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3PEN0_MASK 0x1
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3PEN0(v) (((v) << 0) & 0x1)
/**< Field L3SAM0 - Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3SAM0_SHIFT 2
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3SAM0_MASK 0x4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3SAM0(v) (((v) << 2) & 0x4)
/**< Field L3SAIM0 - Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3SAIM0_SHIFT 3
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3SAIM0_MASK 0x8
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3SAIM0(v) (((v) << 3) & 0x8)
/**< Field L3DAM0 - Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3DAM0_SHIFT 4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3DAM0_MASK 0x10
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3DAM0(v) (((v) << 4) & 0x10)
/**< Field L3DAIM0 - Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3DAIM0_SHIFT 5
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3DAIM0_MASK 0x20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3DAIM0(v) (((v) << 5) & 0x20)
/**< Field L3HSBM0 - Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3HSBM0_SHIFT 6
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3HSBM0_MASK 0x7C0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3HSBM0(v) (((v) << 6) & 0x7C0)
/**< Field L3HDBM0 - Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3HDBM0_SHIFT 11
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3HDBM0_MASK 0xF800
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L3HDBM0(v) (((v) << 11) & 0xF800)
/**< Field L4PEN0 - Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4PEN0_SHIFT 16
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4PEN0_MASK 0x10000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4PEN0(v) (((v) << 16) & 0x10000)
/**< Field L4SPM0 - Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4SPM0_SHIFT 18
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4SPM0_MASK 0x40000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4SPM0(v) (((v) << 18) & 0x40000)
/**< Field L4SPIM0 - Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4SPIM0_SHIFT 19
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4SPIM0_MASK 0x80000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4SPIM0(v) (((v) << 19) & 0x80000)
/**< Field L4DPM0 - Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4DPM0_SHIFT 20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4DPM0_MASK 0x100000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4DPM0(v) (((v) << 20) & 0x100000)
/**< Field L4DPIM0 - Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4DPIM0_SHIFT 21
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4DPIM0_MASK 0x200000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_L4DPIM0(v) (((v) << 21) & 0x200000)
/**< Field DMCHN0 - DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_DMCHN0_SHIFT 24
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_DMCHN0_MASK 0x7000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_DMCHN0(v) (((v) << 24) & 0x7000000)
/**< Field DMCHEN0 - DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_DMCHEN0_SHIFT 28
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_DMCHEN0_MASK 0x10000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL0_DMCHEN0(v) (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MAC_LAYER4_ADDRESS0 - Layer 4 Address 0 */
typedef union IMX_ENET_QOS_MAC_LAYER4_ADDRESS0_union_t {
    UINT32 R;
    struct {
        UINT32 L4SP0 : 16;                /**< Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
        UINT32 L4DP0 : 16;                /**< Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER4_ADDRESS0_t;

/**< Register MAC_LAYER4_ADDRESS0 - Layer 4 Address 0 */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS0         0x904
/**< Field L4SP0 - Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS0_L4SP0_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS0_L4SP0_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS0_L4SP0(v) (((v) << 0) & 0xFFFF)
/**< Field L4DP0 - Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS0_L4DP0_SHIFT 16
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS0_L4DP0_MASK 0xFFFF0000
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS0_L4DP0(v) (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG0 - Layer 3 Address 0 Register 0 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG0_union_t {
    UINT32 R;
    struct {
        UINT32 L3A00 : 32;                /**< Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG0_t;

/**< Register MAC_LAYER3_ADDR0_REG0 - Layer 3 Address 0 Register 0 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG0       0x910
/**< Field L3A00 - Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG0_L3A00_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG0_L3A00_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG0_L3A00(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG0 - Layer 3 Address 1 Register 0 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG0_union_t {
    UINT32 R;
    struct {
        UINT32 L3A10 : 32;                /**< Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG0_t;

/**< Register MAC_LAYER3_ADDR1_REG0 - Layer 3 Address 1 Register 0 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG0       0x914
/**< Field L3A10 - Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG0_L3A10_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG0_L3A10_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG0_L3A10(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG0 - Layer 3 Address 2 Register 0 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG0_union_t {
    UINT32 R;
    struct {
        UINT32 L3A20 : 32;                /**< Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG0_t;

/**< Register MAC_LAYER3_ADDR2_REG0 - Layer 3 Address 2 Register 0 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG0       0x918
/**< Field L3A20 - Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG0_L3A20_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG0_L3A20_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG0_L3A20(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG0 - Layer 3 Address 3 Register 0 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG0_union_t {
    UINT32 R;
    struct {
        UINT32 L3A30 : 32;                /**< Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG0_t;

/**< Register MAC_LAYER3_ADDR3_REG0 - Layer 3 Address 3 Register 0 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG0       0x91C
/**< Field L3A30 - Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG0_L3A30_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG0_L3A30_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG0_L3A30(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_L3_L4_CONTROL1 - Layer 3 and Layer 4 Control of Filter 1 */
typedef union IMX_ENET_QOS_MAC_L3_L4_CONTROL1_union_t {
    UINT32 R;
    struct {
        UINT32 L3PEN1 : 1;                 /**< Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 L3SAM1 : 1;                 /**< Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
        UINT32 L3SAIM1 : 1;                 /**< Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
        UINT32 L3DAM1 : 1;                 /**< Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
        UINT32 L3DAIM1 : 1;                 /**< Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
        UINT32 L3HSBM1 : 5;                 /**< Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
        UINT32 L3HDBM1 : 5;                 /**< Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
        UINT32 L4PEN1 : 1;                 /**< Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
        UINT32 _unused17 : 1;                 /**< Reserved. */
        UINT32 L4SPM1 : 1;                 /**< Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
        UINT32 L4SPIM1 : 1;                 /**< Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
        UINT32 L4DPM1 : 1;                 /**< Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
        UINT32 L4DPIM1 : 1;                 /**< Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
        UINT32 _unused22 : 2;                 /**< Reserved. */
        UINT32 DMCHN1 : 3;                 /**< DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
        UINT32 _unused27 : 1;                 /**< Reserved. */
        UINT32 DMCHEN1 : 1;                 /**< DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_L3_L4_CONTROL1_t;

/**< Register MAC_L3_L4_CONTROL1 - Layer 3 and Layer 4 Control of Filter 1 */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1          0x930
/**< Field L3PEN1 - Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3PEN1_SHIFT 0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3PEN1_MASK 0x1
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3PEN1(v) (((v) << 0) & 0x1)
/**< Field L3SAM1 - Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3SAM1_SHIFT 2
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3SAM1_MASK 0x4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3SAM1(v) (((v) << 2) & 0x4)
/**< Field L3SAIM1 - Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3SAIM1_SHIFT 3
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3SAIM1_MASK 0x8
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3SAIM1(v) (((v) << 3) & 0x8)
/**< Field L3DAM1 - Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3DAM1_SHIFT 4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3DAM1_MASK 0x10
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3DAM1(v) (((v) << 4) & 0x10)
/**< Field L3DAIM1 - Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3DAIM1_SHIFT 5
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3DAIM1_MASK 0x20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3DAIM1(v) (((v) << 5) & 0x20)
/**< Field L3HSBM1 - Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3HSBM1_SHIFT 6
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3HSBM1_MASK 0x7C0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3HSBM1(v) (((v) << 6) & 0x7C0)
/**< Field L3HDBM1 - Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3HDBM1_SHIFT 11
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3HDBM1_MASK 0xF800
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L3HDBM1(v) (((v) << 11) & 0xF800)
/**< Field L4PEN1 - Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4PEN1_SHIFT 16
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4PEN1_MASK 0x10000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4PEN1(v) (((v) << 16) & 0x10000)
/**< Field L4SPM1 - Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4SPM1_SHIFT 18
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4SPM1_MASK 0x40000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4SPM1(v) (((v) << 18) & 0x40000)
/**< Field L4SPIM1 - Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4SPIM1_SHIFT 19
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4SPIM1_MASK 0x80000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4SPIM1(v) (((v) << 19) & 0x80000)
/**< Field L4DPM1 - Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4DPM1_SHIFT 20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4DPM1_MASK 0x100000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4DPM1(v) (((v) << 20) & 0x100000)
/**< Field L4DPIM1 - Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4DPIM1_SHIFT 21
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4DPIM1_MASK 0x200000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_L4DPIM1(v) (((v) << 21) & 0x200000)
/**< Field DMCHN1 - DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_DMCHN1_SHIFT 24
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_DMCHN1_MASK 0x7000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_DMCHN1(v) (((v) << 24) & 0x7000000)
/**< Field DMCHEN1 - DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_DMCHEN1_SHIFT 28
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_DMCHEN1_MASK 0x10000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL1_DMCHEN1(v) (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MAC_LAYER4_ADDRESS1 - Layer 4 Address 0 */
typedef union IMX_ENET_QOS_MAC_LAYER4_ADDRESS1_union_t {
    UINT32 R;
    struct {
        UINT32 L4SP1 : 16;                /**< Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
        UINT32 L4DP1 : 16;                /**< Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER4_ADDRESS1_t;

/**< Register MAC_LAYER4_ADDRESS1 - Layer 4 Address 0 */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS1         0x934
/**< Field L4SP1 - Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS1_L4SP1_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS1_L4SP1_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS1_L4SP1(v) (((v) << 0) & 0xFFFF)
/**< Field L4DP1 - Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS1_L4DP1_SHIFT 16
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS1_L4DP1_MASK 0xFFFF0000
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS1_L4DP1(v) (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG1 - Layer 3 Address 0 Register 1 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG1_union_t {
    UINT32 R;
    struct {
        UINT32 L3A01 : 32;                /**< Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG1_t;

/**< Register MAC_LAYER3_ADDR0_REG1 - Layer 3 Address 0 Register 1 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG1       0x940
/**< Field L3A01 - Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG1_L3A01_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG1_L3A01_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG1_L3A01(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG1 - Layer 3 Address 1 Register 1 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG1_union_t {
    UINT32 R;
    struct {
        UINT32 L3A11 : 32;                /**< Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG1_t;

/**< Register MAC_LAYER3_ADDR1_REG1 - Layer 3 Address 1 Register 1 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG1       0x944
/**< Field L3A11 - Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG1_L3A11_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG1_L3A11_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG1_L3A11(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG1 - Layer 3 Address 2 Register 1 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG1_union_t {
    UINT32 R;
    struct {
        UINT32 L3A21 : 32;                /**< Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG1_t;

/**< Register MAC_LAYER3_ADDR2_REG1 - Layer 3 Address 2 Register 1 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG1       0x948
/**< Field L3A21 - Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG1_L3A21_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG1_L3A21_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG1_L3A21(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG1 - Layer 3 Address 3 Register 1 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG1_union_t {
    UINT32 R;
    struct {
        UINT32 L3A31 : 32;                /**< Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG1_t;

/**< Register MAC_LAYER3_ADDR3_REG1 - Layer 3 Address 3 Register 1 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG1       0x94C
/**< Field L3A31 - Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG1_L3A31_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG1_L3A31_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG1_L3A31(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_L3_L4_CONTROL2 - Layer 3 and Layer 4 Control of Filter 2 */
typedef union IMX_ENET_QOS_MAC_L3_L4_CONTROL2_union_t {
    UINT32 R;
    struct {
        UINT32 L3PEN2 : 1;                 /**< Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 L3SAM2 : 1;                 /**< Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
        UINT32 L3SAIM2 : 1;                 /**< Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
        UINT32 L3DAM2 : 1;                 /**< Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
        UINT32 L3DAIM2 : 1;                 /**< Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
        UINT32 L3HSBM2 : 5;                 /**< Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
        UINT32 L3HDBM2 : 5;                 /**< Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
        UINT32 L4PEN2 : 1;                 /**< Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
        UINT32 _unused17 : 1;                 /**< Reserved. */
        UINT32 L4SPM2 : 1;                 /**< Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
        UINT32 L4SPIM2 : 1;                 /**< Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
        UINT32 L4DPM2 : 1;                 /**< Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
        UINT32 L4DPIM2 : 1;                 /**< Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
        UINT32 _unused22 : 2;                 /**< Reserved. */
        UINT32 DMCHN2 : 3;                 /**< DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
        UINT32 _unused27 : 1;                 /**< Reserved. */
        UINT32 DMCHEN2 : 1;                 /**< DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_L3_L4_CONTROL2_t;

/**< Register MAC_L3_L4_CONTROL2 - Layer 3 and Layer 4 Control of Filter 2 */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2          0x960
/**< Field L3PEN2 - Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3PEN2_SHIFT 0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3PEN2_MASK 0x1
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3PEN2(v) (((v) << 0) & 0x1)
/**< Field L3SAM2 - Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3SAM2_SHIFT 2
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3SAM2_MASK 0x4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3SAM2(v) (((v) << 2) & 0x4)
/**< Field L3SAIM2 - Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3SAIM2_SHIFT 3
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3SAIM2_MASK 0x8
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3SAIM2(v) (((v) << 3) & 0x8)
/**< Field L3DAM2 - Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3DAM2_SHIFT 4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3DAM2_MASK 0x10
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3DAM2(v) (((v) << 4) & 0x10)
/**< Field L3DAIM2 - Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3DAIM2_SHIFT 5
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3DAIM2_MASK 0x20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3DAIM2(v) (((v) << 5) & 0x20)
/**< Field L3HSBM2 - Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3HSBM2_SHIFT 6
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3HSBM2_MASK 0x7C0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3HSBM2(v) (((v) << 6) & 0x7C0)
/**< Field L3HDBM2 - Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3HDBM2_SHIFT 11
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3HDBM2_MASK 0xF800
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L3HDBM2(v) (((v) << 11) & 0xF800)
/**< Field L4PEN2 - Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4PEN2_SHIFT 16
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4PEN2_MASK 0x10000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4PEN2(v) (((v) << 16) & 0x10000)
/**< Field L4SPM2 - Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4SPM2_SHIFT 18
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4SPM2_MASK 0x40000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4SPM2(v) (((v) << 18) & 0x40000)
/**< Field L4SPIM2 - Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4SPIM2_SHIFT 19
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4SPIM2_MASK 0x80000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4SPIM2(v) (((v) << 19) & 0x80000)
/**< Field L4DPM2 - Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4DPM2_SHIFT 20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4DPM2_MASK 0x100000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4DPM2(v) (((v) << 20) & 0x100000)
/**< Field L4DPIM2 - Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4DPIM2_SHIFT 21
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4DPIM2_MASK 0x200000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_L4DPIM2(v) (((v) << 21) & 0x200000)
/**< Field DMCHN2 - DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_DMCHN2_SHIFT 24
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_DMCHN2_MASK 0x7000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_DMCHN2(v) (((v) << 24) & 0x7000000)
/**< Field DMCHEN2 - DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_DMCHEN2_SHIFT 28
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_DMCHEN2_MASK 0x10000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL2_DMCHEN2(v) (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MAC_LAYER4_ADDRESS2 - Layer 4 Address 2 */
typedef union IMX_ENET_QOS_MAC_LAYER4_ADDRESS2_union_t {
    UINT32 R;
    struct {
        UINT32 L4SP2 : 16;                /**< Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
        UINT32 L4DP2 : 16;                /**< Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER4_ADDRESS2_t;

/**< Register MAC_LAYER4_ADDRESS2 - Layer 4 Address 2 */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS2         0x964
/**< Field L4SP2 - Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS2_L4SP2_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS2_L4SP2_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS2_L4SP2(v) (((v) << 0) & 0xFFFF)
/**< Field L4DP2 - Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS2_L4DP2_SHIFT 16
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS2_L4DP2_MASK 0xFFFF0000
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS2_L4DP2(v) (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG2 - Layer 3 Address 0 Register 2 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG2_union_t {
    UINT32 R;
    struct {
        UINT32 L3A02 : 32;                /**< Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG2_t;

/**< Register MAC_LAYER3_ADDR0_REG2 - Layer 3 Address 0 Register 2 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG2       0x970
/**< Field L3A02 - Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG2_L3A02_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG2_L3A02_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG2_L3A02(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG2 - Layer 3 Address 0 Register 2 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG2_union_t {
    UINT32 R;
    struct {
        UINT32 L3A12 : 32;                /**< Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG2_t;

/**< Register MAC_LAYER3_ADDR1_REG2 - Layer 3 Address 0 Register 2 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG2       0x974
/**< Field L3A12 - Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG2_L3A12_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG2_L3A12_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG2_L3A12(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG2 - Layer 3 Address 2 Register 2 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG2_union_t {
    UINT32 R;
    struct {
        UINT32 L3A22 : 32;                /**< Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG2_t;

/**< Register MAC_LAYER3_ADDR2_REG2 - Layer 3 Address 2 Register 2 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG2       0x978
/**< Field L3A22 - Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG2_L3A22_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG2_L3A22_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG2_L3A22(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG2 - Layer 3 Address 3 Register 2 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG2_union_t {
    UINT32 R;
    struct {
        UINT32 L3A32 : 32;                /**< Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG2_t;

/**< Register MAC_LAYER3_ADDR3_REG2 - Layer 3 Address 3 Register 2 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG2       0x97C
/**< Field L3A32 - Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG2_L3A32_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG2_L3A32_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG2_L3A32(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_L3_L4_CONTROL3 - Layer 3 and Layer 4 Control of Filter 3 */
typedef union IMX_ENET_QOS_MAC_L3_L4_CONTROL3_union_t {
    UINT32 R;
    struct {
        UINT32 L3PEN3 : 1;                 /**< Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 L3SAM3 : 1;                 /**< Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
        UINT32 L3SAIM3 : 1;                 /**< Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
        UINT32 L3DAM3 : 1;                 /**< Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
        UINT32 L3DAIM3 : 1;                 /**< Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
        UINT32 L3HSBM3 : 5;                 /**< Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
        UINT32 L3HDBM3 : 5;                 /**< Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
        UINT32 L4PEN3 : 1;                 /**< Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
        UINT32 _unused17 : 1;                 /**< Reserved. */
        UINT32 L4SPM3 : 1;                 /**< Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
        UINT32 L4SPIM3 : 1;                 /**< Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
        UINT32 L4DPM3 : 1;                 /**< Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
        UINT32 L4DPIM3 : 1;                 /**< Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
        UINT32 _unused22 : 2;                 /**< Reserved. */
        UINT32 DMCHN3 : 3;                 /**< DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
        UINT32 _unused27 : 1;                 /**< Reserved. */
        UINT32 DMCHEN3 : 1;                 /**< DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_L3_L4_CONTROL3_t;

/**< Register MAC_L3_L4_CONTROL3 - Layer 3 and Layer 4 Control of Filter 3 */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3          0x990
/**< Field L3PEN3 - Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3PEN3_SHIFT 0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3PEN3_MASK 0x1
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3PEN3(v) (((v) << 0) & 0x1)
/**< Field L3SAM3 - Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3SAM3_SHIFT 2
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3SAM3_MASK 0x4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3SAM3(v) (((v) << 2) & 0x4)
/**< Field L3SAIM3 - Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3SAIM3_SHIFT 3
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3SAIM3_MASK 0x8
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3SAIM3(v) (((v) << 3) & 0x8)
/**< Field L3DAM3 - Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3DAM3_SHIFT 4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3DAM3_MASK 0x10
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3DAM3(v) (((v) << 4) & 0x10)
/**< Field L3DAIM3 - Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3DAIM3_SHIFT 5
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3DAIM3_MASK 0x20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3DAIM3(v) (((v) << 5) & 0x20)
/**< Field L3HSBM3 - Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3HSBM3_SHIFT 6
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3HSBM3_MASK 0x7C0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3HSBM3(v) (((v) << 6) & 0x7C0)
/**< Field L3HDBM3 - Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3HDBM3_SHIFT 11
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3HDBM3_MASK 0xF800
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L3HDBM3(v) (((v) << 11) & 0xF800)
/**< Field L4PEN3 - Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4PEN3_SHIFT 16
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4PEN3_MASK 0x10000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4PEN3(v) (((v) << 16) & 0x10000)
/**< Field L4SPM3 - Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4SPM3_SHIFT 18
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4SPM3_MASK 0x40000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4SPM3(v) (((v) << 18) & 0x40000)
/**< Field L4SPIM3 - Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4SPIM3_SHIFT 19
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4SPIM3_MASK 0x80000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4SPIM3(v) (((v) << 19) & 0x80000)
/**< Field L4DPM3 - Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4DPM3_SHIFT 20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4DPM3_MASK 0x100000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4DPM3(v) (((v) << 20) & 0x100000)
/**< Field L4DPIM3 - Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4DPIM3_SHIFT 21
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4DPIM3_MASK 0x200000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_L4DPIM3(v) (((v) << 21) & 0x200000)
/**< Field DMCHN3 - DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_DMCHN3_SHIFT 24
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_DMCHN3_MASK 0x7000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_DMCHN3(v) (((v) << 24) & 0x7000000)
/**< Field DMCHEN3 - DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_DMCHEN3_SHIFT 28
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_DMCHEN3_MASK 0x10000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL3_DMCHEN3(v) (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MAC_LAYER4_ADDRESS3 - Layer 4 Address 3 */
typedef union IMX_ENET_QOS_MAC_LAYER4_ADDRESS3_union_t {
    UINT32 R;
    struct {
        UINT32 L4SP3 : 16;                /**< Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
        UINT32 L4DP3 : 16;                /**< Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER4_ADDRESS3_t;

/**< Register MAC_LAYER4_ADDRESS3 - Layer 4 Address 3 */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS3         0x994
/**< Field L4SP3 - Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS3_L4SP3_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS3_L4SP3_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS3_L4SP3(v) (((v) << 0) & 0xFFFF)
/**< Field L4DP3 - Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS3_L4DP3_SHIFT 16
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS3_L4DP3_MASK 0xFFFF0000
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS3_L4DP3(v) (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG3 - Layer 3 Address 0 Register 3 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG3_union_t {
    UINT32 R;
    struct {
        UINT32 L3A03 : 32;                /**< Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG3_t;

/**< Register MAC_LAYER3_ADDR0_REG3 - Layer 3 Address 0 Register 3 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG3       0x9A0
/**< Field L3A03 - Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG3_L3A03_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG3_L3A03_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG3_L3A03(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG3 - Layer 3 Address 1 Register 3 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG3_union_t {
    UINT32 R;
    struct {
        UINT32 L3A13 : 32;                /**< Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG3_t;

/**< Register MAC_LAYER3_ADDR1_REG3 - Layer 3 Address 1 Register 3 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG3       0x9A4
/**< Field L3A13 - Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG3_L3A13_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG3_L3A13_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG3_L3A13(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG3 - Layer 3 Address 2 Register 3 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG3_union_t {
    UINT32 R;
    struct {
        UINT32 L3A23 : 32;                /**< Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG3_t;

/**< Register MAC_LAYER3_ADDR2_REG3 - Layer 3 Address 2 Register 3 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG3       0x9A8
/**< Field L3A23 - Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG3_L3A23_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG3_L3A23_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG3_L3A23(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG3 - Layer 3 Address 3 Register 3 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG3_union_t {
    UINT32 R;
    struct {
        UINT32 L3A33 : 32;                /**< Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG3_t;

/**< Register MAC_LAYER3_ADDR3_REG3 - Layer 3 Address 3 Register 3 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG3       0x9AC
/**< Field L3A33 - Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG3_L3A33_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG3_L3A33_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG3_L3A33(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_L3_L4_CONTROL4 - Layer 3 and Layer 4 Control of Filter 4 */
typedef union IMX_ENET_QOS_MAC_L3_L4_CONTROL4_union_t {
    UINT32 R;
    struct {
        UINT32 L3PEN4 : 1;                 /**< Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 L3SAM4 : 1;                 /**< Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
        UINT32 L3SAIM4 : 1;                 /**< Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
        UINT32 L3DAM4 : 1;                 /**< Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
        UINT32 L3DAIM4 : 1;                 /**< Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
        UINT32 L3HSBM4 : 5;                 /**< Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
        UINT32 L3HDBM4 : 5;                 /**< Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
        UINT32 L4PEN4 : 1;                 /**< Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
        UINT32 _unused17 : 1;                 /**< Reserved. */
        UINT32 L4SPM4 : 1;                 /**< Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
        UINT32 L4SPIM4 : 1;                 /**< Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
        UINT32 L4DPM4 : 1;                 /**< Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
        UINT32 L4DPIM4 : 1;                 /**< Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
        UINT32 _unused22 : 2;                 /**< Reserved. */
        UINT32 DMCHN4 : 3;                 /**< DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
        UINT32 _unused27 : 1;                 /**< Reserved. */
        UINT32 DMCHEN4 : 1;                 /**< DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_L3_L4_CONTROL4_t;

/**< Register MAC_L3_L4_CONTROL4 - Layer 3 and Layer 4 Control of Filter 4 */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4          0x9C0
/**< Field L3PEN4 - Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3PEN4_SHIFT 0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3PEN4_MASK 0x1
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3PEN4(v) (((v) << 0) & 0x1)
/**< Field L3SAM4 - Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3SAM4_SHIFT 2
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3SAM4_MASK 0x4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3SAM4(v) (((v) << 2) & 0x4)
/**< Field L3SAIM4 - Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3SAIM4_SHIFT 3
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3SAIM4_MASK 0x8
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3SAIM4(v) (((v) << 3) & 0x8)
/**< Field L3DAM4 - Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3DAM4_SHIFT 4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3DAM4_MASK 0x10
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3DAM4(v) (((v) << 4) & 0x10)
/**< Field L3DAIM4 - Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3DAIM4_SHIFT 5
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3DAIM4_MASK 0x20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3DAIM4(v) (((v) << 5) & 0x20)
/**< Field L3HSBM4 - Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3HSBM4_SHIFT 6
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3HSBM4_MASK 0x7C0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3HSBM4(v) (((v) << 6) & 0x7C0)
/**< Field L3HDBM4 - Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3HDBM4_SHIFT 11
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3HDBM4_MASK 0xF800
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L3HDBM4(v) (((v) << 11) & 0xF800)
/**< Field L4PEN4 - Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4PEN4_SHIFT 16
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4PEN4_MASK 0x10000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4PEN4(v) (((v) << 16) & 0x10000)
/**< Field L4SPM4 - Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4SPM4_SHIFT 18
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4SPM4_MASK 0x40000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4SPM4(v) (((v) << 18) & 0x40000)
/**< Field L4SPIM4 - Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4SPIM4_SHIFT 19
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4SPIM4_MASK 0x80000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4SPIM4(v) (((v) << 19) & 0x80000)
/**< Field L4DPM4 - Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4DPM4_SHIFT 20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4DPM4_MASK 0x100000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4DPM4(v) (((v) << 20) & 0x100000)
/**< Field L4DPIM4 - Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4DPIM4_SHIFT 21
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4DPIM4_MASK 0x200000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_L4DPIM4(v) (((v) << 21) & 0x200000)
/**< Field DMCHN4 - DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_DMCHN4_SHIFT 24
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_DMCHN4_MASK 0x7000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_DMCHN4(v) (((v) << 24) & 0x7000000)
/**< Field DMCHEN4 - DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_DMCHEN4_SHIFT 28
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_DMCHEN4_MASK 0x10000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL4_DMCHEN4(v) (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MAC_LAYER4_ADDRESS4 - Layer 4 Address 4 */
typedef union IMX_ENET_QOS_MAC_LAYER4_ADDRESS4_union_t {
    UINT32 R;
    struct {
        UINT32 L4SP4 : 16;                /**< Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
        UINT32 L4DP4 : 16;                /**< Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER4_ADDRESS4_t;

/**< Register MAC_LAYER4_ADDRESS4 - Layer 4 Address 4 */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS4         0x9C4
/**< Field L4SP4 - Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS4_L4SP4_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS4_L4SP4_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS4_L4SP4(v) (((v) << 0) & 0xFFFF)
/**< Field L4DP4 - Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS4_L4DP4_SHIFT 16
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS4_L4DP4_MASK 0xFFFF0000
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS4_L4DP4(v) (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG4 - Layer 3 Address 0 Register 4 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG4_union_t {
    UINT32 R;
    struct {
        UINT32 L3A04 : 32;                /**< Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG4_t;

/**< Register MAC_LAYER3_ADDR0_REG4 - Layer 3 Address 0 Register 4 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG4       0x9D0
/**< Field L3A04 - Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG4_L3A04_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG4_L3A04_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG4_L3A04(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG4 - Layer 3 Address 1 Register 4 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG4_union_t {
    UINT32 R;
    struct {
        UINT32 L3A14 : 32;                /**< Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG4_t;

/**< Register MAC_LAYER3_ADDR1_REG4 - Layer 3 Address 1 Register 4 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG4       0x9D4
/**< Field L3A14 - Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG4_L3A14_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG4_L3A14_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG4_L3A14(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG4 - Layer 3 Address 2 Register 4 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG4_union_t {
    UINT32 R;
    struct {
        UINT32 L3A24 : 32;                /**< Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG4_t;

/**< Register MAC_LAYER3_ADDR2_REG4 - Layer 3 Address 2 Register 4 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG4       0x9D8
/**< Field L3A24 - Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG4_L3A24_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG4_L3A24_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG4_L3A24(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG4 - Layer 3 Address 3 Register 4 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG4_union_t {
    UINT32 R;
    struct {
        UINT32 L3A34 : 32;                /**< Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG4_t;

/**< Register MAC_LAYER3_ADDR3_REG4 - Layer 3 Address 3 Register 4 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG4       0x9DC
/**< Field L3A34 - Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG4_L3A34_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG4_L3A34_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG4_L3A34(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_L3_L4_CONTROL5 - Layer 3 and Layer 4 Control of Filter 5 */
typedef union IMX_ENET_QOS_MAC_L3_L4_CONTROL5_union_t {
    UINT32 R;
    struct {
        UINT32 L3PEN5 : 1;                 /**< Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 L3SAM5 : 1;                 /**< Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
        UINT32 L3SAIM5 : 1;                 /**< Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
        UINT32 L3DAM5 : 1;                 /**< Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
        UINT32 L3DAIM5 : 1;                 /**< Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
        UINT32 L3HSBM5 : 5;                 /**< Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
        UINT32 L3HDBM5 : 5;                 /**< Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
        UINT32 L4PEN5 : 1;                 /**< Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
        UINT32 _unused17 : 1;                 /**< Reserved. */
        UINT32 L4SPM5 : 1;                 /**< Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
        UINT32 L4SPIM5 : 1;                 /**< Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
        UINT32 L4DPM5 : 1;                 /**< Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
        UINT32 L4DPIM5 : 1;                 /**< Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
        UINT32 _unused22 : 2;                 /**< Reserved. */
        UINT32 DMCHN5 : 3;                 /**< DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
        UINT32 _unused27 : 1;                 /**< Reserved. */
        UINT32 DMCHEN5 : 1;                 /**< DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_L3_L4_CONTROL5_t;

/**< Register MAC_L3_L4_CONTROL5 - Layer 3 and Layer 4 Control of Filter 5 */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5          0x9F0
/**< Field L3PEN5 - Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3PEN5_SHIFT 0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3PEN5_MASK 0x1
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3PEN5(v) (((v) << 0) & 0x1)
/**< Field L3SAM5 - Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3SAM5_SHIFT 2
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3SAM5_MASK 0x4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3SAM5(v) (((v) << 2) & 0x4)
/**< Field L3SAIM5 - Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3SAIM5_SHIFT 3
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3SAIM5_MASK 0x8
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3SAIM5(v) (((v) << 3) & 0x8)
/**< Field L3DAM5 - Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3DAM5_SHIFT 4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3DAM5_MASK 0x10
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3DAM5(v) (((v) << 4) & 0x10)
/**< Field L3DAIM5 - Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3DAIM5_SHIFT 5
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3DAIM5_MASK 0x20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3DAIM5(v) (((v) << 5) & 0x20)
/**< Field L3HSBM5 - Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3HSBM5_SHIFT 6
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3HSBM5_MASK 0x7C0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3HSBM5(v) (((v) << 6) & 0x7C0)
/**< Field L3HDBM5 - Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3HDBM5_SHIFT 11
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3HDBM5_MASK 0xF800
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L3HDBM5(v) (((v) << 11) & 0xF800)
/**< Field L4PEN5 - Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4PEN5_SHIFT 16
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4PEN5_MASK 0x10000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4PEN5(v) (((v) << 16) & 0x10000)
/**< Field L4SPM5 - Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4SPM5_SHIFT 18
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4SPM5_MASK 0x40000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4SPM5(v) (((v) << 18) & 0x40000)
/**< Field L4SPIM5 - Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4SPIM5_SHIFT 19
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4SPIM5_MASK 0x80000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4SPIM5(v) (((v) << 19) & 0x80000)
/**< Field L4DPM5 - Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4DPM5_SHIFT 20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4DPM5_MASK 0x100000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4DPM5(v) (((v) << 20) & 0x100000)
/**< Field L4DPIM5 - Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4DPIM5_SHIFT 21
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4DPIM5_MASK 0x200000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_L4DPIM5(v) (((v) << 21) & 0x200000)
/**< Field DMCHN5 - DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_DMCHN5_SHIFT 24
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_DMCHN5_MASK 0x7000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_DMCHN5(v) (((v) << 24) & 0x7000000)
/**< Field DMCHEN5 - DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_DMCHEN5_SHIFT 28
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_DMCHEN5_MASK 0x10000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL5_DMCHEN5(v) (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MAC_LAYER4_ADDRESS5 - Layer 4 Address 5 */
typedef union IMX_ENET_QOS_MAC_LAYER4_ADDRESS5_union_t {
    UINT32 R;
    struct {
        UINT32 L4SP5 : 16;                /**< Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
        UINT32 L4DP5 : 16;                /**< Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER4_ADDRESS5_t;

/**< Register MAC_LAYER4_ADDRESS5 - Layer 4 Address 5 */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS5         0x9F4
/**< Field L4SP5 - Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS5_L4SP5_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS5_L4SP5_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS5_L4SP5(v) (((v) << 0) & 0xFFFF)
/**< Field L4DP5 - Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS5_L4DP5_SHIFT 16
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS5_L4DP5_MASK 0xFFFF0000
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS5_L4DP5(v) (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG5 - Layer 3 Address 0 Register 5 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG5_union_t {
    UINT32 R;
    struct {
        UINT32 L3A05 : 32;                /**< Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG5_t;

/**< Register MAC_LAYER3_ADDR0_REG5 - Layer 3 Address 0 Register 5 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG5       0xA00
/**< Field L3A05 - Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG5_L3A05_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG5_L3A05_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG5_L3A05(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG5 - Layer 3 Address 1 Register 5 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG5_union_t {
    UINT32 R;
    struct {
        UINT32 L3A15 : 32;                /**< Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG5_t;

/**< Register MAC_LAYER3_ADDR1_REG5 - Layer 3 Address 1 Register 5 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG5       0xA04
/**< Field L3A15 - Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG5_L3A15_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG5_L3A15_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG5_L3A15(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG5 - Layer 3 Address 2 Register 5 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG5_union_t {
    UINT32 R;
    struct {
        UINT32 L3A25 : 32;                /**< Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG5_t;

/**< Register MAC_LAYER3_ADDR2_REG5 - Layer 3 Address 2 Register 5 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG5       0xA08
/**< Field L3A25 - Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG5_L3A25_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG5_L3A25_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG5_L3A25(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG5 - Layer 3 Address 3 Register 5 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG5_union_t {
    UINT32 R;
    struct {
        UINT32 L3A35 : 32;                /**< Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG5_t;

/**< Register MAC_LAYER3_ADDR3_REG5 - Layer 3 Address 3 Register 5 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG5       0xA0C
/**< Field L3A35 - Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG5_L3A35_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG5_L3A35_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG5_L3A35(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_L3_L4_CONTROL6 - Layer 3 and Layer 4 Control of Filter 6 */
typedef union IMX_ENET_QOS_MAC_L3_L4_CONTROL6_union_t {
    UINT32 R;
    struct {
        UINT32 L3PEN6 : 1;                 /**< Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 L3SAM6 : 1;                 /**< Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
        UINT32 L3SAIM6 : 1;                 /**< Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
        UINT32 L3DAM6 : 1;                 /**< Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
        UINT32 L3DAIM6 : 1;                 /**< Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
        UINT32 L3HSBM6 : 5;                 /**< Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
        UINT32 L3HDBM6 : 5;                 /**< Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
        UINT32 L4PEN6 : 1;                 /**< Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
        UINT32 _unused17 : 1;                 /**< Reserved. */
        UINT32 L4SPM6 : 1;                 /**< Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
        UINT32 L4SPIM6 : 1;                 /**< Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
        UINT32 L4DPM6 : 1;                 /**< Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
        UINT32 L4DPIM6 : 1;                 /**< Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
        UINT32 _unused22 : 2;                 /**< Reserved. */
        UINT32 DMCHN6 : 3;                 /**< DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
        UINT32 _unused27 : 1;                 /**< Reserved. */
        UINT32 DMCHEN6 : 1;                 /**< DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_L3_L4_CONTROL6_t;

/**< Register MAC_L3_L4_CONTROL6 - Layer 3 and Layer 4 Control of Filter 6 */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6          0xA20
/**< Field L3PEN6 - Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3PEN6_SHIFT 0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3PEN6_MASK 0x1
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3PEN6(v) (((v) << 0) & 0x1)
/**< Field L3SAM6 - Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3SAM6_SHIFT 2
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3SAM6_MASK 0x4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3SAM6(v) (((v) << 2) & 0x4)
/**< Field L3SAIM6 - Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3SAIM6_SHIFT 3
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3SAIM6_MASK 0x8
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3SAIM6(v) (((v) << 3) & 0x8)
/**< Field L3DAM6 - Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3DAM6_SHIFT 4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3DAM6_MASK 0x10
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3DAM6(v) (((v) << 4) & 0x10)
/**< Field L3DAIM6 - Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3DAIM6_SHIFT 5
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3DAIM6_MASK 0x20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3DAIM6(v) (((v) << 5) & 0x20)
/**< Field L3HSBM6 - Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3HSBM6_SHIFT 6
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3HSBM6_MASK 0x7C0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3HSBM6(v) (((v) << 6) & 0x7C0)
/**< Field L3HDBM6 - Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3HDBM6_SHIFT 11
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3HDBM6_MASK 0xF800
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L3HDBM6(v) (((v) << 11) & 0xF800)
/**< Field L4PEN6 - Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4PEN6_SHIFT 16
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4PEN6_MASK 0x10000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4PEN6(v) (((v) << 16) & 0x10000)
/**< Field L4SPM6 - Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4SPM6_SHIFT 18
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4SPM6_MASK 0x40000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4SPM6(v) (((v) << 18) & 0x40000)
/**< Field L4SPIM6 - Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4SPIM6_SHIFT 19
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4SPIM6_MASK 0x80000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4SPIM6(v) (((v) << 19) & 0x80000)
/**< Field L4DPM6 - Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4DPM6_SHIFT 20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4DPM6_MASK 0x100000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4DPM6(v) (((v) << 20) & 0x100000)
/**< Field L4DPIM6 - Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4DPIM6_SHIFT 21
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4DPIM6_MASK 0x200000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_L4DPIM6(v) (((v) << 21) & 0x200000)
/**< Field DMCHN6 - DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_DMCHN6_SHIFT 24
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_DMCHN6_MASK 0x7000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_DMCHN6(v) (((v) << 24) & 0x7000000)
/**< Field DMCHEN6 - DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_DMCHEN6_SHIFT 28
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_DMCHEN6_MASK 0x10000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL6_DMCHEN6(v) (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MAC_LAYER4_ADDRESS6 - Layer 4 Address 6 */
typedef union IMX_ENET_QOS_MAC_LAYER4_ADDRESS6_union_t {
    UINT32 R;
    struct {
        UINT32 L4SP6 : 16;                /**< Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
        UINT32 L4DP6 : 16;                /**< Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER4_ADDRESS6_t;

/**< Register MAC_LAYER4_ADDRESS6 - Layer 4 Address 6 */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS6         0xA24
/**< Field L4SP6 - Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS6_L4SP6_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS6_L4SP6_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS6_L4SP6(v) (((v) << 0) & 0xFFFF)
/**< Field L4DP6 - Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS6_L4DP6_SHIFT 16
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS6_L4DP6_MASK 0xFFFF0000
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS6_L4DP6(v) (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG6 - Layer 3 Address 0 Register 6 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG6_union_t {
    UINT32 R;
    struct {
        UINT32 L3A06 : 32;                /**< Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG6_t;

/**< Register MAC_LAYER3_ADDR0_REG6 - Layer 3 Address 0 Register 6 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG6       0xA30
/**< Field L3A06 - Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG6_L3A06_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG6_L3A06_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG6_L3A06(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG6 - Layer 3 Address 1 Register 6 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG6_union_t {
    UINT32 R;
    struct {
        UINT32 L3A16 : 32;                /**< Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG6_t;

/**< Register MAC_LAYER3_ADDR1_REG6 - Layer 3 Address 1 Register 6 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG6       0xA34
/**< Field L3A16 - Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG6_L3A16_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG6_L3A16_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG6_L3A16(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG6 - Layer 3 Address 2 Register 6 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG6_union_t {
    UINT32 R;
    struct {
        UINT32 L3A26 : 32;                /**< Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG6_t;

/**< Register MAC_LAYER3_ADDR2_REG6 - Layer 3 Address 2 Register 6 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG6       0xA38
/**< Field L3A26 - Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG6_L3A26_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG6_L3A26_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG6_L3A26(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG6 - Layer 3 Address 3 Register 6 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG6_union_t {
    UINT32 R;
    struct {
        UINT32 L3A36 : 32;                /**< Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG6_t;

/**< Register MAC_LAYER3_ADDR3_REG6 - Layer 3 Address 3 Register 6 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG6       0xA3C
/**< Field L3A36 - Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG6_L3A36_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG6_L3A36_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG6_L3A36(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_L3_L4_CONTROL7 - Layer 3 and Layer 4 Control of Filter 0 */
typedef union IMX_ENET_QOS_MAC_L3_L4_CONTROL7_union_t {
    UINT32 R;
    struct {
        UINT32 L3PEN7 : 1;                 /**< Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 L3SAM7 : 1;                 /**< Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
        UINT32 L3SAIM7 : 1;                 /**< Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
        UINT32 L3DAM7 : 1;                 /**< Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
        UINT32 L3DAIM7 : 1;                 /**< Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
        UINT32 L3HSBM7 : 5;                 /**< Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
        UINT32 L3HDBM7 : 5;                 /**< Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
        UINT32 L4PEN7 : 1;                 /**< Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
        UINT32 _unused17 : 1;                 /**< Reserved. */
        UINT32 L4SPM7 : 1;                 /**< Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
        UINT32 L4SPIM7 : 1;                 /**< Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
        UINT32 L4DPM7 : 1;                 /**< Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
        UINT32 L4DPIM7 : 1;                 /**< Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
        UINT32 _unused22 : 2;                 /**< Reserved. */
        UINT32 DMCHN7 : 3;                 /**< DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
        UINT32 _unused27 : 1;                 /**< Reserved. */
        UINT32 DMCHEN7 : 1;                 /**< DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_L3_L4_CONTROL7_t;

/**< Register MAC_L3_L4_CONTROL7 - Layer 3 and Layer 4 Control of Filter 0 */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7          0xA50
/**< Field L3PEN7 - Layer 3 Protocol Enable When this bit is set, the Layer 3 IP Source or Destination Address matching is enabled for IPv6 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3PEN7_SHIFT 0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3PEN7_MASK 0x1
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3PEN7(v) (((v) << 0) & 0x1)
/**< Field L3SAM7 - Layer 3 IP SA Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3SAM7_SHIFT 2
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3SAM7_MASK 0x4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3SAM7(v) (((v) << 2) & 0x4)
/**< Field L3SAIM7 - Layer 3 IP SA Inverse Match Enable When this bit is set, the Layer 3 IP Source Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3SAIM7_SHIFT 3
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3SAIM7_MASK 0x8
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3SAIM7(v) (((v) << 3) & 0x8)
/**< Field L3DAM7 - Layer 3 IP DA Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3DAM7_SHIFT 4
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3DAM7_MASK 0x10
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3DAM7(v) (((v) << 4) & 0x10)
/**< Field L3DAIM7 - Layer 3 IP DA Inverse Match Enable When this bit is set, the Layer 3 IP Destination Address field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3DAIM7_SHIFT 5
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3DAIM7_MASK 0x20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3DAIM7(v) (((v) << 5) & 0x20)
/**< Field L3HSBM7 - Layer 3 IP SA Higher Bits Match IPv4 Packets: This field contains the number of lower bits of IP Source Address that are masked for matching in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3HSBM7_SHIFT 6
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3HSBM7_MASK 0x7C0
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3HSBM7(v) (((v) << 6) & 0x7C0)
/**< Field L3HDBM7 - Layer 3 IP DA Higher Bits Match IPv4 Packets: This field contains the number of higher bits of IP Destination Address that are matched in the IPv4 packets. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3HDBM7_SHIFT 11
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3HDBM7_MASK 0xF800
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L3HDBM7(v) (((v) << 11) & 0xF800)
/**< Field L4PEN7 - Layer 4 Protocol Enable When this bit is set, the Source and Destination Port number fields of UDP packets are used for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4PEN7_SHIFT 16
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4PEN7_MASK 0x10000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4PEN7(v) (((v) << 16) & 0x10000)
/**< Field L4SPM7 - Layer 4 Source Port Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4SPM7_SHIFT 18
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4SPM7_MASK 0x40000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4SPM7(v) (((v) << 18) & 0x40000)
/**< Field L4SPIM7 - Layer 4 Source Port Inverse Match Enable When this bit is set, the Layer 4 Source Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4SPIM7_SHIFT 19
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4SPIM7_MASK 0x80000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4SPIM7(v) (((v) << 19) & 0x80000)
/**< Field L4DPM7 - Layer 4 Destination Port Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4DPM7_SHIFT 20
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4DPM7_MASK 0x100000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4DPM7(v) (((v) << 20) & 0x100000)
/**< Field L4DPIM7 - Layer 4 Destination Port Inverse Match Enable When this bit is set, the Layer 4 Destination Port number field is enabled for inverse matching. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4DPIM7_SHIFT 21
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4DPIM7_MASK 0x200000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_L4DPIM7(v) (((v) << 21) & 0x200000)
/**< Field DMCHN7 - DMA Channel Number When DMCHEN is set high, this field selects the DMA Channel number to which the packet passed by this filter is routed. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_DMCHN7_SHIFT 24
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_DMCHN7_MASK 0x7000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_DMCHN7(v) (((v) << 24) & 0x7000000)
/**< Field DMCHEN7 - DMA Channel Select Enable When set, this bit enables the selection of the DMA channel number for the packet that is passed by this L3_L4 filter. */
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_DMCHEN7_SHIFT 28
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_DMCHEN7_MASK 0x10000000
#define IMX_ENET_QOS_MAC_L3_L4_CONTROL7_DMCHEN7(v) (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MAC_LAYER4_ADDRESS7 - Layer 4 Address 7 */
typedef union IMX_ENET_QOS_MAC_LAYER4_ADDRESS7_union_t {
    UINT32 R;
    struct {
        UINT32 L4SP7 : 16;                /**< Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
        UINT32 L4DP7 : 16;                /**< Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER4_ADDRESS7_t;

/**< Register MAC_LAYER4_ADDRESS7 - Layer 4 Address 7 */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS7         0xA54
/**< Field L4SP7 - Layer 4 Source Port Number Field When the L4PEN0 bit is reset and the L4SPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Source Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS7_L4SP7_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS7_L4SP7_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS7_L4SP7(v) (((v) << 0) & 0xFFFF)
/**< Field L4DP7 - Layer 4 Destination Port Number Field When the L4PEN0 bit is reset and the L4DPM0 bit is set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with the TCP Destination Port Number field in the IPv4 or IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS7_L4DP7_SHIFT 16
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS7_L4DP7_MASK 0xFFFF0000
#define IMX_ENET_QOS_MAC_LAYER4_ADDRESS7_L4DP7(v) (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG7 - Layer 3 Address 0 Register 7 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG7_union_t {
    UINT32 R;
    struct {
        UINT32 L3A07 : 32;                /**< Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG7_t;

/**< Register MAC_LAYER3_ADDR0_REG7 - Layer 3 Address 0 Register 7 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG7       0xA60
/**< Field L3A07 - Layer 3 Address 0 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[31:0] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG7_L3A07_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG7_L3A07_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG7_L3A07(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG7 - Layer 3 Address 1 Register 7 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG7_union_t {
    UINT32 R;
    struct {
        UINT32 L3A17 : 32;                /**< Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG7_t;

/**< Register MAC_LAYER3_ADDR1_REG7 - Layer 3 Address 1 Register 7 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG7       0xA64
/**< Field L3A17 - Layer 3 Address 1 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[63:32] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG7_L3A17_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG7_L3A17_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG7_L3A17(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG7 - Layer 3 Address 2 Register 7 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG7_union_t {
    UINT32 R;
    struct {
        UINT32 L3A27 : 32;                /**< Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG7_t;

/**< Register MAC_LAYER3_ADDR2_REG7 - Layer 3 Address 2 Register 7 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG7       0xA68
/**< Field L3A27 - Layer 3 Address 2 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[95:64] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG7_L3A27_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG7_L3A27_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG7_L3A27(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG7 - Layer 3 Address 3 Register 7 */
typedef union IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG7_union_t {
    UINT32 R;
    struct {
        UINT32 L3A37 : 32;                /**< Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
    } B;
} IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG7_t;

/**< Register MAC_LAYER3_ADDR3_REG7 - Layer 3 Address 3 Register 7 */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG7       0xA6C
/**< Field L3A37 - Layer 3 Address 3 Field When the L3PEN0 and L3SAM0 bits are set in the MAC_L3_L4_CONTROL0 register, this field contains the value to be matched with Bits[127:96] of the IP Source Address field in the IPv6 packets. */
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG7_L3A37_SHIFT 0
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG7_L3A37_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG7_L3A37(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL - Timestamp Control */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_union_t {
    UINT32 R;
    struct {
        UINT32 TSENA : 1;                 /**< Enable Timestamp When this bit is set, the timestamp is added for Transmit and Receive packets. */
        UINT32 TSCFUPDT : 1;                 /**< Fine or Coarse Timestamp Update When this bit is set, the Fine method is used to update system timestamp. */
        UINT32 TSINIT : 1;                 /**< Initialize Timestamp When this bit is set, the system time is initialized (overwritten) with the value specified in the MAC_System_Time_Seconds_Update and MAC_System_Time_Nanoseconds_Update registers. */
        UINT32 TSUPDT : 1;                 /**< Update Timestamp When this bit is set, the system time is updated (added or subtracted) with the value specified in MAC_System_Time_Seconds_Update and MAC_System_Time_Nanoseconds_Update registers. */
        UINT32 _unused4 : 1;                 /**< Reserved. */
        UINT32 TSADDREG : 1;                 /**< Update Addend Register When this bit is set, the content of the Timestamp Addend register is updated in the PTP block for fine correction. */
        UINT32 PTGE : 1;                 /**< Presentation Time Generation Enable When this bit is set the Presentation Time generation will be enabled. */
        UINT32 _unused7 : 1;                 /**< Reserved. */
        UINT32 TSENALL : 1;                 /**< Enable Timestamp for All Packets When this bit is set, the timestamp snapshot is enabled for all packets received by the MAC. */
        UINT32 TSCTRLSSR : 1;                 /**< Timestamp Digital or Binary Rollover Control When this bit is set, the Timestamp Low register rolls over after 0x3B9A_C9FF value (that is, 1 nanosecond accuracy) and increments the timestamp (High) seconds. */
        UINT32 TSVER2ENA : 1;                 /**< Enable PTP Packet Processing for Version 2 Format When this bit is set, the IEEE 1588 version 2 format is used to process the PTP packets. */
        UINT32 TSIPENA : 1;                 /**< Enable Processing of PTP over Ethernet Packets When this bit is set, the MAC receiver processes the PTP packets encapsulated directly in the Ethernet packets. */
        UINT32 TSIPV6ENA : 1;                 /**< Enable Processing of PTP Packets Sent over IPv6-UDP When this bit is set, the MAC receiver processes the PTP packets encapsulated in IPv6-UDP packets. */
        UINT32 TSIPV4ENA : 1;                 /**< Enable Processing of PTP Packets Sent over IPv4-UDP When this bit is set, the MAC receiver processes the PTP packets encapsulated in IPv4-UDP packets. */
        UINT32 TSEVNTENA : 1;                 /**< Enable Timestamp Snapshot for Event Messages When this bit is set, the timestamp snapshot is taken only for event messages (SYNC, Delay_Req, Pdelay_Req, or Pdelay_Resp). */
        UINT32 TSMSTRENA : 1;                 /**< Enable Snapshot for Messages Relevant to Master When this bit is set, the snapshot is taken only for the messages that are relevant to the master node. */
        UINT32 SNAPTYPSEL : 2;                 /**< Select PTP packets for Taking Snapshots These bits, along with Bits 15 and 14, decide the set of PTP packet types for which snapshot needs to be taken. */
        UINT32 TSENMACADDR : 1;                 /**< Enable MAC Address for PTP Packet Filtering When this bit is set, the DA MAC address (that matches any MAC Address register) is used to filter the PTP packets when PTP is directly sent over Ethernet. */
        UINT32 CSC : 1;                 /**< Enable checksum correction during OST for PTP over UDP/IPv4 packets When this bit is set, the last two bytes of PTP message sent over UDP/IPv4 is updated to keep the UDP checksum correct, for changes made to origin timestamp and/or correction field as part of one step timestamp operation. */
        UINT32 ESTI : 1;                 /**< External System Time Input When this bit is set, the MAC uses the external 64-bit reference System Time input for the following: - To take the timestamp provided as status - To insert the timestamp in transmit PTP packets when One-step Timestamp or Timestamp Offload feature is enabled. */
        UINT32 _unused21 : 3;                 /**< Reserved. */
        UINT32 TXTSSTSM : 1;                 /**< Transmit Timestamp Status Mode When this bit is set, the MAC overwrites the earlier transmit timestamp status even if it is not read by the software. */
        UINT32 _unused25 : 3;                 /**< Reserved. */
        UINT32 AV8021ASMEN : 1;                 /**< AV 802. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_t;

/**< Register MAC_TIMESTAMP_CONTROL - Timestamp Control */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL       0xB00
/**< Field TSENA - Enable Timestamp When this bit is set, the timestamp is added for Transmit and Receive packets. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSENA_SHIFT 0
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSENA_MASK 0x1
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSENA(v) (((v) << 0) & 0x1)
/**< Field TSCFUPDT - Fine or Coarse Timestamp Update When this bit is set, the Fine method is used to update system timestamp. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSCFUPDT_SHIFT 1
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSCFUPDT_MASK 0x2
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSCFUPDT(v) (((v) << 1) & 0x2)
/**< Field TSINIT - Initialize Timestamp When this bit is set, the system time is initialized (overwritten) with the value specified in the MAC_System_Time_Seconds_Update and MAC_System_Time_Nanoseconds_Update registers. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSINIT_SHIFT 2
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSINIT_MASK 0x4
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSINIT(v) (((v) << 2) & 0x4)
/**< Field TSUPDT - Update Timestamp When this bit is set, the system time is updated (added or subtracted) with the value specified in MAC_System_Time_Seconds_Update and MAC_System_Time_Nanoseconds_Update registers. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSUPDT_SHIFT 3
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSUPDT_MASK 0x8
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSUPDT(v) (((v) << 3) & 0x8)
/**< Field TSADDREG - Update Addend Register When this bit is set, the content of the Timestamp Addend register is updated in the PTP block for fine correction. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSADDREG_SHIFT 5
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSADDREG_MASK 0x20
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSADDREG(v) (((v) << 5) & 0x20)
/**< Field PTGE - Presentation Time Generation Enable When this bit is set the Presentation Time generation will be enabled. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_PTGE_SHIFT 6
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_PTGE_MASK 0x40
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_PTGE(v) (((v) << 6) & 0x40)
/**< Field TSENALL - Enable Timestamp for All Packets When this bit is set, the timestamp snapshot is enabled for all packets received by the MAC. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSENALL_SHIFT 8
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSENALL_MASK 0x100
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSENALL(v) (((v) << 8) & 0x100)
/**< Field TSCTRLSSR - Timestamp Digital or Binary Rollover Control When this bit is set, the Timestamp Low register rolls over after 0x3B9A_C9FF value (that is, 1 nanosecond accuracy) and increments the timestamp (High) seconds. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSCTRLSSR_SHIFT 9
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSCTRLSSR_MASK 0x200
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSCTRLSSR(v) (((v) << 9) & 0x200)
/**< Field TSVER2ENA - Enable PTP Packet Processing for Version 2 Format When this bit is set, the IEEE 1588 version 2 format is used to process the PTP packets. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSVER2ENA_SHIFT 10
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSVER2ENA_MASK 0x400
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSVER2ENA(v) (((v) << 10) & 0x400)
/**< Field TSIPENA - Enable Processing of PTP over Ethernet Packets When this bit is set, the MAC receiver processes the PTP packets encapsulated directly in the Ethernet packets. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSIPENA_SHIFT 11
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSIPENA_MASK 0x800
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSIPENA(v) (((v) << 11) & 0x800)
/**< Field TSIPV6ENA - Enable Processing of PTP Packets Sent over IPv6-UDP When this bit is set, the MAC receiver processes the PTP packets encapsulated in IPv6-UDP packets. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSIPV6ENA_SHIFT 12
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSIPV6ENA_MASK 0x1000
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSIPV6ENA(v) (((v) << 12) & 0x1000)
/**< Field TSIPV4ENA - Enable Processing of PTP Packets Sent over IPv4-UDP When this bit is set, the MAC receiver processes the PTP packets encapsulated in IPv4-UDP packets. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSIPV4ENA_SHIFT 13
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSIPV4ENA_MASK 0x2000
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSIPV4ENA(v) (((v) << 13) & 0x2000)
/**< Field TSEVNTENA - Enable Timestamp Snapshot for Event Messages When this bit is set, the timestamp snapshot is taken only for event messages (SYNC, Delay_Req, Pdelay_Req, or Pdelay_Resp). */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSEVNTENA_SHIFT 14
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSEVNTENA_MASK 0x4000
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSEVNTENA(v) (((v) << 14) & 0x4000)
/**< Field TSMSTRENA - Enable Snapshot for Messages Relevant to Master When this bit is set, the snapshot is taken only for the messages that are relevant to the master node. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSMSTRENA_SHIFT 15
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSMSTRENA_MASK 0x8000
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSMSTRENA(v) (((v) << 15) & 0x8000)
/**< Field SNAPTYPSEL - Select PTP packets for Taking Snapshots These bits, along with Bits 15 and 14, decide the set of PTP packet types for which snapshot needs to be taken. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_SNAPTYPSEL_SHIFT 16
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_SNAPTYPSEL_MASK 0x30000
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_SNAPTYPSEL(v) (((v) << 16) & 0x30000)
/**< Field TSENMACADDR - Enable MAC Address for PTP Packet Filtering When this bit is set, the DA MAC address (that matches any MAC Address register) is used to filter the PTP packets when PTP is directly sent over Ethernet. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSENMACADDR_SHIFT 18
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSENMACADDR_MASK 0x40000
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TSENMACADDR(v) (((v) << 18) & 0x40000)
/**< Field CSC - Enable checksum correction during OST for PTP over UDP/IPv4 packets When this bit is set, the last two bytes of PTP message sent over UDP/IPv4 is updated to keep the UDP checksum correct, for changes made to origin timestamp and/or correction field as part of one step timestamp operation. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_CSC_SHIFT 19
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_CSC_MASK 0x80000
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_CSC(v) (((v) << 19) & 0x80000)
/**< Field ESTI - External System Time Input When this bit is set, the MAC uses the external 64-bit reference System Time input for the following: - To take the timestamp provided as status - To insert the timestamp in transmit PTP packets when One-step Timestamp or Timestamp Offload feature is enabled. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_ESTI_SHIFT 20
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_ESTI_MASK 0x100000
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_ESTI(v) (((v) << 20) & 0x100000)
/**< Field TXTSSTSM - Transmit Timestamp Status Mode When this bit is set, the MAC overwrites the earlier transmit timestamp status even if it is not read by the software. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TXTSSTSM_SHIFT 24
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TXTSSTSM_MASK 0x1000000
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_TXTSSTSM(v) (((v) << 24) & 0x1000000)
/**< Field AV8021ASMEN - AV 802. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_AV8021ASMEN_SHIFT 28
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_AV8021ASMEN_MASK 0x10000000
#define IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_AV8021ASMEN(v) (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT - Subsecond Increment */
typedef union IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 8;                 /**< Reserved. */
        UINT32 SNSINC : 8;                 /**< Sub-nanosecond Increment Value This field contains the sub-nanosecond increment value, represented in nanoseconds multiplied by 2^8. */
        UINT32 SSINC : 8;                 /**< Sub-second Increment Value The value programmed in this field is accumulated every clock cycle (of clk_ptp_i) with the contents of the sub-second register. */
        UINT32 _unused24 : 8;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT_t;

/**< Register MAC_SUB_SECOND_INCREMENT - Subsecond Increment */
#define IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT    0xB04
/**< Field SNSINC - Sub-nanosecond Increment Value This field contains the sub-nanosecond increment value, represented in nanoseconds multiplied by 2^8. */
#define IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT_SNSINC_SHIFT 8
#define IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT_SNSINC_MASK 0xFF00
#define IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT_SNSINC(v) (((v) << 8) & 0xFF00)
/**< Field SSINC - Sub-second Increment Value The value programmed in this field is accumulated every clock cycle (of clk_ptp_i) with the contents of the sub-second register. */
#define IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT_SSINC_SHIFT 16
#define IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT_SSINC_MASK 0xFF0000
#define IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT_SSINC(v) (((v) << 16) & 0xFF0000)

/**< IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS - System Time Seconds */
typedef union IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TSS : 32;                /**< Timestamp Second The value in this field indicates the current value in seconds of the System Time maintained by the MAC. */
    } B;
} IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_t;

/**< Register MAC_SYSTEM_TIME_SECONDS - System Time Seconds */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS     0xB08
/**< Field TSS - Timestamp Second The value in this field indicates the current value in seconds of the System Time maintained by the MAC. */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_TSS_SHIFT 0
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_TSS_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_TSS(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS - System Time Nanoseconds */
typedef union IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TSSS : 31;                /**< Timestamp Sub Seconds The value in this field has the sub-second representation of time, with an accuracy of 0. */
        UINT32 _unused31 : 1;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_t;

/**< Register MAC_SYSTEM_TIME_NANOSECONDS - System Time Nanoseconds */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS 0xB0C
/**< Field TSSS - Timestamp Sub Seconds The value in this field has the sub-second representation of time, with an accuracy of 0. */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_TSSS_SHIFT 0
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_TSSS_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_TSSS(v) (((v) << 0) & 0x7FFFFFFF)

/**< IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_UPDATE - System Time Seconds Update */
typedef union IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_UPDATE_union_t {
    UINT32 R;
    struct {
        UINT32 TSS : 32;                /**< Timestamp Seconds The value in this field is the seconds part of the update. */
    } B;
} IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_UPDATE_t;

/**< Register MAC_SYSTEM_TIME_SECONDS_UPDATE - System Time Seconds Update */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_UPDATE 0xB10
/**< Field TSS - Timestamp Seconds The value in this field is the seconds part of the update. */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_UPDATE_TSS_SHIFT 0
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_UPDATE_TSS_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_UPDATE_TSS(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE - System Time Nanoseconds Update */
typedef union IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_union_t {
    UINT32 R;
    struct {
        UINT32 TSSS : 31;                /**< Timestamp Sub Seconds The value in this field is the sub-seconds part of the update. */
        UINT32 ADDSUB : 1;                 /**< Add or Subtract Time When this bit is set, the time value is subtracted with the contents of the update register. */
    } B;
} IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_t;

/**< Register MAC_SYSTEM_TIME_NANOSECONDS_UPDATE - System Time Nanoseconds Update */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE 0xB14
/**< Field TSSS - Timestamp Sub Seconds The value in this field is the sub-seconds part of the update. */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_SHIFT 0
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_TSSS(v) (((v) << 0) & 0x7FFFFFFF)
/**< Field ADDSUB - Add or Subtract Time When this bit is set, the time value is subtracted with the contents of the update register. */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_ADDSUB_SHIFT 31
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_ADDSUB_MASK 0x80000000
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_ADDSUB(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_ADDEND - Timestamp Addend */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_ADDEND_union_t {
    UINT32 R;
    struct {
        UINT32 TSAR : 32;                /**< Timestamp Addend Register This field indicates the 32-bit time value to be added to the Accumulator register to achieve time synchronization. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_ADDEND_t;

/**< Register MAC_TIMESTAMP_ADDEND - Timestamp Addend */
#define IMX_ENET_QOS_MAC_TIMESTAMP_ADDEND        0xB18
/**< Field TSAR - Timestamp Addend Register This field indicates the 32-bit time value to be added to the Accumulator register to achieve time synchronization. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_ADDEND_TSAR_SHIFT 0
#define IMX_ENET_QOS_MAC_TIMESTAMP_ADDEND_TSAR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TIMESTAMP_ADDEND_TSAR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS - System Time - Higher Word Seconds */
typedef union IMX_ENET_QOS_MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TSHWR : 16;                /**< Timestamp Higher Word Register This field contains the most-significant 16-bits of timestamp seconds value. */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS_t;

/**< Register MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS - System Time - Higher Word Seconds */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS 0xB1C
/**< Field TSHWR - Timestamp Higher Word Register This field contains the most-significant 16-bits of timestamp seconds value. */
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS_TSHWR_SHIFT 0
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS_TSHWR_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS_TSHWR(v) (((v) << 0) & 0xFFFF)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_STATUS - Timestamp Status */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 TSSOVF : 1;                 /**< Timestamp Seconds Overflow When this bit is set, it indicates that the seconds value of the timestamp (when supporting version 2 format) has overflowed beyond 32'hFFFF_FFFF. */
        UINT32 TSTARGT0 : 1;                 /**< Timestamp Target Time Reached When set, this bit indicates that the value of system time is greater than or equal to the value specified in the MAC_PPS0_Target_Time_Seconds and MAC_PPS0_Target_Time_Nanoseconds registers. */
        UINT32 AUXTSTRIG : 1;                 /**< Auxiliary Timestamp Trigger Snapshot This bit is set high when the auxiliary snapshot is written to the FIFO. */
        UINT32 TSTRGTERR0 : 1;                 /**< Timestamp Target Time Error This bit is set when the latest target time programmed in the MAC_PPS0_Target_Time_Seconds and MAC_PPS0_Target_Time_Nanoseconds registers elapses. */
        UINT32 TSTARGT1 : 1;                 /**< Timestamp Target Time Reached for Target Time PPS1 When set, this bit indicates that the value of system time is greater than or equal to the value specified in the MAC_PPS1_TARGET_TIME_SECONDS and MAC_PPS1_TARGET_TIME_NANOSECONDS registers. */
        UINT32 TSTRGTERR1 : 1;                 /**< Timestamp Target Time Error This bit is set when the latest target time programmed in the MAC_PPS1_TARGET_TIME_SECONDS and MAC_PPS1_TARGET_TIME_NANOSECONDS registers elapses. */
        UINT32 TSTARGT2 : 1;                 /**< Timestamp Target Time Reached for Target Time PPS2 When set, this bit indicates that the value of system time is greater than or equal to the value specified in the MAC_PPS2_TARGET_TIME_SECONDS and MAC_PPS2_TARGET_TIME_NANOSECONDS registers. */
        UINT32 TSTRGTERR2 : 1;                 /**< Timestamp Target Time Error This bit is set when the latest target time programmed in the MAC_PPS2_TARGET_TIME_SECONDS and MAC_PPS2_TARGET_TIME_NANOSECONDS registers elapses. */
        UINT32 TSTARGT3 : 1;                 /**< Timestamp Target Time Reached for Target Time PPS3 When this bit is set, it indicates that the value of system time is greater than or equal to the value specified in the MAC_PPS3_TARGET_TIME_SECONDS and MAC_PPS3_TARGET_TIME_NANOSECONDS registers. */
        UINT32 TSTRGTERR3 : 1;                 /**< Timestamp Target Time Error This bit is set when the latest target time programmed in the MAC_PPS3_TARGET_TIME_SECONDS and MAC_PPS3_TARGET_TIME_NANOSECONDS registers elapses. */
        UINT32 _unused10 : 5;                 /**< Reserved. */
        UINT32 TXTSSIS : 1;                 /**< Tx Timestamp Status Interrupt Status In non-EQOS_CORE configurations when drop transmit status is enabled in MTL, this bit is set when the captured transmit timestamp is updated in the MAC_TX_TIMESTAMP_STATUS_NANOSECONDS and MAC_TX_TIMESTAMP_STATUS_SECONDS registers. */
        UINT32 ATSSTN : 4;                 /**< Auxiliary Timestamp Snapshot Trigger Identifier These bits identify the Auxiliary trigger inputs for which the timestamp available in the Auxiliary Snapshot Register is applicable. */
        UINT32 _unused20 : 4;                 /**< Reserved. */
        UINT32 ATSSTM : 1;                 /**< Auxiliary Timestamp Snapshot Trigger Missed This bit is set when the Auxiliary timestamp snapshot FIFO is full and external trigger was set. */
        UINT32 ATSNS : 5;                 /**< Number of Auxiliary Timestamp Snapshots This field indicates the number of Snapshots available in the FIFO. */
        UINT32 _unused30 : 2;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_t;

/**< Register MAC_TIMESTAMP_STATUS - Timestamp Status */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS        0xB20
/**< Field TSSOVF - Timestamp Seconds Overflow When this bit is set, it indicates that the seconds value of the timestamp (when supporting version 2 format) has overflowed beyond 32'hFFFF_FFFF. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSSOVF_SHIFT 0
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSSOVF_MASK 0x1
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSSOVF(v) (((v) << 0) & 0x1)
/**< Field TSTARGT0 - Timestamp Target Time Reached When set, this bit indicates that the value of system time is greater than or equal to the value specified in the MAC_PPS0_Target_Time_Seconds and MAC_PPS0_Target_Time_Nanoseconds registers. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT0_SHIFT 1
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT0_MASK 0x2
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT0(v) (((v) << 1) & 0x2)
/**< Field AUXTSTRIG - Auxiliary Timestamp Trigger Snapshot This bit is set high when the auxiliary snapshot is written to the FIFO. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_AUXTSTRIG_SHIFT 2
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_AUXTSTRIG_MASK 0x4
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_AUXTSTRIG(v) (((v) << 2) & 0x4)
/**< Field TSTRGTERR0 - Timestamp Target Time Error This bit is set when the latest target time programmed in the MAC_PPS0_Target_Time_Seconds and MAC_PPS0_Target_Time_Nanoseconds registers elapses. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR0_SHIFT 3
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR0_MASK 0x8
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR0(v) (((v) << 3) & 0x8)
/**< Field TSTARGT1 - Timestamp Target Time Reached for Target Time PPS1 When set, this bit indicates that the value of system time is greater than or equal to the value specified in the MAC_PPS1_TARGET_TIME_SECONDS and MAC_PPS1_TARGET_TIME_NANOSECONDS registers. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT1_SHIFT 4
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT1_MASK 0x10
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT1(v) (((v) << 4) & 0x10)
/**< Field TSTRGTERR1 - Timestamp Target Time Error This bit is set when the latest target time programmed in the MAC_PPS1_TARGET_TIME_SECONDS and MAC_PPS1_TARGET_TIME_NANOSECONDS registers elapses. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR1_SHIFT 5
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR1_MASK 0x20
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR1(v) (((v) << 5) & 0x20)
/**< Field TSTARGT2 - Timestamp Target Time Reached for Target Time PPS2 When set, this bit indicates that the value of system time is greater than or equal to the value specified in the MAC_PPS2_TARGET_TIME_SECONDS and MAC_PPS2_TARGET_TIME_NANOSECONDS registers. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT2_SHIFT 6
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT2_MASK 0x40
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT2(v) (((v) << 6) & 0x40)
/**< Field TSTRGTERR2 - Timestamp Target Time Error This bit is set when the latest target time programmed in the MAC_PPS2_TARGET_TIME_SECONDS and MAC_PPS2_TARGET_TIME_NANOSECONDS registers elapses. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR2_SHIFT 7
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR2_MASK 0x80
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR2(v) (((v) << 7) & 0x80)
/**< Field TSTARGT3 - Timestamp Target Time Reached for Target Time PPS3 When this bit is set, it indicates that the value of system time is greater than or equal to the value specified in the MAC_PPS3_TARGET_TIME_SECONDS and MAC_PPS3_TARGET_TIME_NANOSECONDS registers. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT3_SHIFT 8
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT3_MASK 0x100
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTARGT3(v) (((v) << 8) & 0x100)
/**< Field TSTRGTERR3 - Timestamp Target Time Error This bit is set when the latest target time programmed in the MAC_PPS3_TARGET_TIME_SECONDS and MAC_PPS3_TARGET_TIME_NANOSECONDS registers elapses. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR3_SHIFT 9
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR3_MASK 0x200
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TSTRGTERR3(v) (((v) << 9) & 0x200)
/**< Field TXTSSIS - Tx Timestamp Status Interrupt Status In non-EQOS_CORE configurations when drop transmit status is enabled in MTL, this bit is set when the captured transmit timestamp is updated in the MAC_TX_TIMESTAMP_STATUS_NANOSECONDS and MAC_TX_TIMESTAMP_STATUS_SECONDS registers. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TXTSSIS_SHIFT 15
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TXTSSIS_MASK 0x8000
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_TXTSSIS(v) (((v) << 15) & 0x8000)
/**< Field ATSSTN - Auxiliary Timestamp Snapshot Trigger Identifier These bits identify the Auxiliary trigger inputs for which the timestamp available in the Auxiliary Snapshot Register is applicable. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_ATSSTN_SHIFT 16
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_ATSSTN_MASK 0xF0000
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_ATSSTN(v) (((v) << 16) & 0xF0000)
/**< Field ATSSTM - Auxiliary Timestamp Snapshot Trigger Missed This bit is set when the Auxiliary timestamp snapshot FIFO is full and external trigger was set. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_ATSSTM_SHIFT 24
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_ATSSTM_MASK 0x1000000
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_ATSSTM(v) (((v) << 24) & 0x1000000)
/**< Field ATSNS - Number of Auxiliary Timestamp Snapshots This field indicates the number of Snapshots available in the FIFO. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_ATSNS_SHIFT 25
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_ATSNS_MASK 0x3E000000
#define IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_ATSNS(v) (((v) << 25) & 0x3E000000)

/**< IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS - Transmit Timestamp Status Nanoseconds */
typedef union IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TXTSSLO : 31;                /**< Transmit Timestamp Status Low This field contains the 31 bits of the Nanoseconds field of the Transmit packet's captured timestamp. */
        UINT32 TXTSSMIS : 1;                 /**< Transmit Timestamp Status Missed When this bit is set, it indicates one of the following: - The timestamp of the current packet is ignored if TXTSSTSM bit of the TIMESTAMP_CONTROL register is reset - The timestamp of the previous packet is overwritten with timestamp of the current packet if TXTSSTSM bit of the MAC_TIMESTAMP_CONTROL register is set. */
    } B;
} IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS_t;

/**< Register MAC_TX_TIMESTAMP_STATUS_NANOSECONDS - Transmit Timestamp Status Nanoseconds */
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS 0xB30
/**< Field TXTSSLO - Transmit Timestamp Status Low This field contains the 31 bits of the Nanoseconds field of the Transmit packet's captured timestamp. */
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS_TXTSSLO_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS_TXTSSLO_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS_TXTSSLO(v) (((v) << 0) & 0x7FFFFFFF)
/**< Field TXTSSMIS - Transmit Timestamp Status Missed When this bit is set, it indicates one of the following: - The timestamp of the current packet is ignored if TXTSSTSM bit of the TIMESTAMP_CONTROL register is reset - The timestamp of the previous packet is overwritten with timestamp of the current packet if TXTSSTSM bit of the MAC_TIMESTAMP_CONTROL register is set. */
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS_TXTSSMIS_SHIFT 31
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS_TXTSSMIS_MASK 0x80000000
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS_TXTSSMIS(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_SECONDS - Transmit Timestamp Status Seconds */
typedef union IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_SECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TXTSSHI : 32;                /**< Transmit Timestamp Status High This field contains the lower 32 bits of the Seconds field of Transmit packet's captured timestamp. */
    } B;
} IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_SECONDS_t;

/**< Register MAC_TX_TIMESTAMP_STATUS_SECONDS - Transmit Timestamp Status Seconds */
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_SECONDS 0xB34
/**< Field TXTSSHI - Transmit Timestamp Status High This field contains the lower 32 bits of the Seconds field of Transmit packet's captured timestamp. */
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_SECONDS_TXTSSHI_SHIFT 0
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_SECONDS_TXTSSHI_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_SECONDS_TXTSSHI(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_AUXILIARY_CONTROL - Auxiliary Timestamp Control */
typedef union IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_union_t {
    UINT32 R;
    struct {
        UINT32 ATSFC : 1;                 /**< Auxiliary Snapshot FIFO Clear When set, this bit resets the pointers of the Auxiliary Snapshot FIFO. */
        UINT32 _unused1 : 3;                 /**< Reserved. */
        UINT32 ATSEN0 : 1;                 /**< Auxiliary Snapshot 0 Enable This bit controls the capturing of Auxiliary Snapshot Trigger 0. */
        UINT32 ATSEN1 : 1;                 /**< Auxiliary Snapshot 1 Enable This bit controls the capturing of Auxiliary Snapshot Trigger 1. */
        UINT32 ATSEN2 : 1;                 /**< Auxiliary Snapshot 2 Enable This bit controls the capturing of Auxiliary Snapshot Trigger 2. */
        UINT32 ATSEN3 : 1;                 /**< Auxiliary Snapshot 3 Enable This bit controls the capturing of Auxiliary Snapshot Trigger 3. */
        UINT32 _unused8 : 24;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_t;

/**< Register MAC_AUXILIARY_CONTROL - Auxiliary Timestamp Control */
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL       0xB40
/**< Field ATSFC - Auxiliary Snapshot FIFO Clear When set, this bit resets the pointers of the Auxiliary Snapshot FIFO. */
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSFC_SHIFT 0
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSFC_MASK 0x1
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSFC(v) (((v) << 0) & 0x1)
/**< Field ATSEN0 - Auxiliary Snapshot 0 Enable This bit controls the capturing of Auxiliary Snapshot Trigger 0. */
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN0_SHIFT 4
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN0_MASK 0x10
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN0(v) (((v) << 4) & 0x10)
/**< Field ATSEN1 - Auxiliary Snapshot 1 Enable This bit controls the capturing of Auxiliary Snapshot Trigger 1. */
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN1_SHIFT 5
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN1_MASK 0x20
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN1(v) (((v) << 5) & 0x20)
/**< Field ATSEN2 - Auxiliary Snapshot 2 Enable This bit controls the capturing of Auxiliary Snapshot Trigger 2. */
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN2_SHIFT 6
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN2_MASK 0x40
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN2(v) (((v) << 6) & 0x40)
/**< Field ATSEN3 - Auxiliary Snapshot 3 Enable This bit controls the capturing of Auxiliary Snapshot Trigger 3. */
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN3_SHIFT 7
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN3_MASK 0x80
#define IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_ATSEN3(v) (((v) << 7) & 0x80)

/**< IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_NANOSECONDS - Auxiliary Timestamp Nanoseconds */
typedef union IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_NANOSECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 AUXTSLO : 31;                /**< Auxiliary Timestamp Contains the lower 31 bits (nanoseconds field) of the auxiliary timestamp. */
        UINT32 _unused31 : 1;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_NANOSECONDS_t;

/**< Register MAC_AUXILIARY_TIMESTAMP_NANOSECONDS - Auxiliary Timestamp Nanoseconds */
#define IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_NANOSECONDS 0xB48
/**< Field AUXTSLO - Auxiliary Timestamp Contains the lower 31 bits (nanoseconds field) of the auxiliary timestamp. */
#define IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_NANOSECONDS_AUXTSLO_SHIFT 0
#define IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_NANOSECONDS_AUXTSLO_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_NANOSECONDS_AUXTSLO(v) (((v) << 0) & 0x7FFFFFFF)

/**< IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_SECONDS - Auxiliary Timestamp Seconds */
typedef union IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_SECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 AUXTSHI : 32;                /**< Auxiliary Timestamp Contains the lower 32 bits of the Seconds field of the auxiliary timestamp. */
    } B;
} IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_SECONDS_t;

/**< Register MAC_AUXILIARY_TIMESTAMP_SECONDS - Auxiliary Timestamp Seconds */
#define IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_SECONDS 0xB4C
/**< Field AUXTSHI - Auxiliary Timestamp Contains the lower 32 bits of the Seconds field of the auxiliary timestamp. */
#define IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_SECONDS_AUXTSHI_SHIFT 0
#define IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_SECONDS_AUXTSHI_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_SECONDS_AUXTSHI(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_ASYM_CORR - Timestamp Ingress Asymmetry Correction */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_ASYM_CORR_union_t {
    UINT32 R;
    struct {
        UINT32 OSTIAC : 32;                /**< One-Step Timestamp Ingress Asymmetry Correction This field contains the ingress path asymmetry value to be added to correctionField of Pdelay_Resp PTP packet. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_ASYM_CORR_t;

/**< Register MAC_TIMESTAMP_INGRESS_ASYM_CORR - Timestamp Ingress Asymmetry Correction */
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_ASYM_CORR 0xB50
/**< Field OSTIAC - One-Step Timestamp Ingress Asymmetry Correction This field contains the ingress path asymmetry value to be added to correctionField of Pdelay_Resp PTP packet. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_ASYM_CORR_OSTIAC_SHIFT 0
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_ASYM_CORR_OSTIAC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_ASYM_CORR_OSTIAC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_ASYM_CORR - imestamp Egress Asymmetry Correction */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_ASYM_CORR_union_t {
    UINT32 R;
    struct {
        UINT32 OSTEAC : 32;                /**< One-Step Timestamp Egress Asymmetry Correction This field contains the egress path asymmetry value to be subtracted from correctionField of Pdelay_Resp PTP packet. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_ASYM_CORR_t;

/**< Register MAC_TIMESTAMP_EGRESS_ASYM_CORR - imestamp Egress Asymmetry Correction */
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_ASYM_CORR 0xB54
/**< Field OSTEAC - One-Step Timestamp Egress Asymmetry Correction This field contains the egress path asymmetry value to be subtracted from correctionField of Pdelay_Resp PTP packet. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_ASYM_CORR_OSTEAC_SHIFT 0
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_ASYM_CORR_OSTEAC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_ASYM_CORR_OSTEAC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND - Timestamp Ingress Correction Nanosecond */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND_union_t {
    UINT32 R;
    struct {
        UINT32 TSIC : 32;                /**< Timestamp Ingress Correction This field contains the ingress path correction value as defined by the Ingress Correction expression. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND_t;

/**< Register MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND - Timestamp Ingress Correction Nanosecond */
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND 0xB58
/**< Field TSIC - Timestamp Ingress Correction This field contains the ingress path correction value as defined by the Ingress Correction expression. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND_TSIC_SHIFT 0
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND_TSIC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND_TSIC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND - Timestamp Egress Correction Nanosecond */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND_union_t {
    UINT32 R;
    struct {
        UINT32 TSEC : 32;                /**< Timestamp Egress Correction This field contains the nanoseconds part of the egress path correction value as defined by the Egress Correction expression. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND_t;

/**< Register MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND - Timestamp Egress Correction Nanosecond */
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND 0xB5C
/**< Field TSEC - Timestamp Egress Correction This field contains the nanoseconds part of the egress path correction value as defined by the Egress Correction expression. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND_TSEC_SHIFT 0
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND_TSEC_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND_TSEC(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC - Timestamp Ingress Correction Subnanosecond */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 8;                 /**< Reserved. */
        UINT32 TSICSNS : 8;                 /**< Timestamp Ingress Correction, sub-nanoseconds This field contains the sub-nanoseconds part of the ingress path correction value as defined by the "Ingress Correction" expression. */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC_t;

/**< Register MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC - Timestamp Ingress Correction Subnanosecond */
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC 0xB60
/**< Field TSICSNS - Timestamp Ingress Correction, sub-nanoseconds This field contains the sub-nanoseconds part of the ingress path correction value as defined by the "Ingress Correction" expression. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC_TSICSNS_SHIFT 8
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC_TSICSNS_MASK 0xFF00
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC_TSICSNS(v) (((v) << 8) & 0xFF00)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC - Timestamp Egress Correction Subnanosecond */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 8;                 /**< Reserved. */
        UINT32 TSECSNS : 8;                 /**< Timestamp Egress Correction, sub-nanoseconds This field contains the sub-nanoseconds part of the egress path correction value as defined by the "Egress Correction" expression. */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC_t;

/**< Register MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC - Timestamp Egress Correction Subnanosecond */
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC 0xB64
/**< Field TSECSNS - Timestamp Egress Correction, sub-nanoseconds This field contains the sub-nanoseconds part of the egress path correction value as defined by the "Egress Correction" expression. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC_TSECSNS_SHIFT 8
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC_TSECSNS_MASK 0xFF00
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC_TSECSNS(v) (((v) << 8) & 0xFF00)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY - Timestamp Ingress Latency */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 8;                 /**< Reserved. */
        UINT32 ITLSNS : 8;                 /**< Ingress Timestamp Latency, in nanoseconds This register holds the average latency in nanoseconds between the input ports (phy_rxd_i) of MAC and the actual point (GMII/MII) where the ingress timestamp is taken. */
        UINT32 ITLNS : 12;                /**< Ingress Timestamp Latency, in sub-nanoseconds This register holds the average latency in sub-nanoseconds between the input ports (phy_rxd_i) of MAC and the actual point (GMII/MII) where the ingress timestamp is taken. */
        UINT32 _unused28 : 4;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY_t;

/**< Register MAC_TIMESTAMP_INGRESS_LATENCY - Timestamp Ingress Latency */
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY 0xB68
/**< Field ITLSNS - Ingress Timestamp Latency, in nanoseconds This register holds the average latency in nanoseconds between the input ports (phy_rxd_i) of MAC and the actual point (GMII/MII) where the ingress timestamp is taken. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY_ITLSNS_SHIFT 8
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY_ITLSNS_MASK 0xFF00
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY_ITLSNS(v) (((v) << 8) & 0xFF00)
/**< Field ITLNS - Ingress Timestamp Latency, in sub-nanoseconds This register holds the average latency in sub-nanoseconds between the input ports (phy_rxd_i) of MAC and the actual point (GMII/MII) where the ingress timestamp is taken. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY_ITLNS_SHIFT 16
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY_ITLNS_MASK 0xFFF0000
#define IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY_ITLNS(v) (((v) << 16) & 0xFFF0000)

/**< IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY - Timestamp Egress Latency */
typedef union IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 8;                 /**< Reserved. */
        UINT32 ETLSNS : 8;                 /**< Egress Timestamp Latency, in sub-nanoseconds This register holds the average latency in sub-nanoseconds between the actual point (GMII/MII) where the egress timestamp is taken and the output ports (phy_txd_o) of the MAC. */
        UINT32 ETLNS : 12;                /**< Egress Timestamp Latency, in nanoseconds This register holds the average latency in nanoseconds between the actual point (GMII/MII) where the egress timestamp is taken and the output ports (phy_txd_o) of the MAC. */
        UINT32 _unused28 : 4;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY_t;

/**< Register MAC_TIMESTAMP_EGRESS_LATENCY - Timestamp Egress Latency */
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY 0xB6C
/**< Field ETLSNS - Egress Timestamp Latency, in sub-nanoseconds This register holds the average latency in sub-nanoseconds between the actual point (GMII/MII) where the egress timestamp is taken and the output ports (phy_txd_o) of the MAC. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY_ETLSNS_SHIFT 8
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY_ETLSNS_MASK 0xFF00
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY_ETLSNS(v) (((v) << 8) & 0xFF00)
/**< Field ETLNS - Egress Timestamp Latency, in nanoseconds This register holds the average latency in nanoseconds between the actual point (GMII/MII) where the egress timestamp is taken and the output ports (phy_txd_o) of the MAC. */
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY_ETLNS_SHIFT 16
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY_ETLNS_MASK 0xFFF0000
#define IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY_ETLNS(v) (((v) << 16) & 0xFFF0000)

/**< IMX_ENET_QOS_MAC_PPS_CONTROL - PPS Control */
typedef union IMX_ENET_QOS_MAC_PPS_CONTROL_union_t {
    UINT32 R;
    struct {
        UINT32 PPSCTRL_PPSCMD : 4;              /**< PPS Output Frequency Control This field controls the frequency of the PPS0 output (ptp_pps_o[0]) signal. */
        UINT32 PPSEN0 : 1;                 /**< Flexible PPS Output Mode Enable When this bit is set, Bits[3:0] function as PPSCMD. */
        UINT32 TRGTMODSEL0 : 2;                 /**< Target Time Register Mode for PPS0 Output This field indicates the Target Time registers (MAC_PPS0_TARGET_TIME_SECONDS and MAC_PPS0_TARGET_TIME_NANOSECONDS) mode for PPS0 output signal: */
        UINT32 MCGREN0 : 1;                 /**< MCGR Mode Enable for PPS0 Output This field enables the 0th PPS instance to operate in PPS or MCGR mode. */
        UINT32 PPSCMD1 : 4;                 /**< Flexible PPS1 Output Control This field controls the flexible PPS1 output (ptp_pps_o[1]) signal. */
        UINT32 _unused12 : 1;                 /**< Reserved. */
        UINT32 TRGTMODSEL1 : 2;                 /**< Target Time Register Mode for PPS1 Output This field indicates the Target Time registers (MAC_PPS1_TARGET_TIME_SECONDS and MAC_PPS1_TARGET_TIME_NANOSECONDS) mode for PPS1 output signal. */
        UINT32 MCGREN1 : 1;                 /**< MCGR Mode Enable for PPS1 Output This field enables the 1st PPS instance to operate in PPS or MCGR mode. */
        UINT32 PPSCMD2 : 4;                 /**< Flexible PPS2 Output Control This field controls the flexible PPS2 output (ptp_pps_o[2]) signal. */
        UINT32 _unused20 : 1;                 /**< Reserved. */
        UINT32 TRGTMODSEL2 : 2;                 /**< Target Time Register Mode for PPS2 Output This field indicates the Target Time registers (MAC_PPS2_TARGET_TIME_SECONDS and MAC_PPS2_TARGET_TIME_NANOSECONDS) mode for PPS2 output signal. */
        UINT32 MCGREN2 : 1;                 /**< MCGR Mode Enable for PPS2 Output This field enables the 2nd PPS instance to operate in PPS or MCGR mode. */
        UINT32 PPSCMD3 : 4;                 /**< Flexible PPS3 Output Control This field controls the flexible PPS3 output (ptp_pps_o[3]) signal. */
        UINT32 _unused28 : 1;                 /**< Reserved. */
        UINT32 TRGTMODSEL3 : 2;                 /**< Target Time Register Mode for PPS3 Output This field indicates the Target Time registers (MAC_PPS3_TARGET_TIME_SECONDS and MAC_PPS3_TARGET_TIME_NANOSECONDS) mode for PPS3 output signal. */
        UINT32 MCGREN3 : 1;                 /**< MCGR Mode Enable for PPS3 Output This field enables the 3rd PPS instance to operate in PPS or MCGR mode. */
    } B;
} IMX_ENET_QOS_MAC_PPS_CONTROL_t;

/**< Register MAC_PPS_CONTROL - PPS Control */
#define IMX_ENET_QOS_MAC_PPS_CONTROL             0xB70
/**< Field PPSCTRL_PPSCMD - PPS Output Frequency Control This field controls the frequency of the PPS0 output (ptp_pps_o[0]) signal. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCTRL_PPSCMD_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCTRL_PPSCMD_MASK 0xF
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCTRL_PPSCMD(v) (((v) << 0) & 0xF)
/**< Field PPSEN0 - Flexible PPS Output Mode Enable When this bit is set, Bits[3:0] function as PPSCMD. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSEN0_SHIFT 4
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSEN0_MASK 0x10
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSEN0(v)   (((v) << 4) & 0x10)
/**< Field TRGTMODSEL0 - Target Time Register Mode for PPS0 Output This field indicates the Target Time registers (MAC_PPS0_TARGET_TIME_SECONDS and MAC_PPS0_TARGET_TIME_NANOSECONDS) mode for PPS0 output signal: */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL0_SHIFT 5
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL0_MASK 0x60
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL0(v) (((v) << 5) & 0x60)
/**< Field MCGREN0 - MCGR Mode Enable for PPS0 Output This field enables the 0th PPS instance to operate in PPS or MCGR mode. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN0_SHIFT 7
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN0_MASK 0x80
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN0(v)  (((v) << 7) & 0x80)
/**< Field PPSCMD1 - Flexible PPS1 Output Control This field controls the flexible PPS1 output (ptp_pps_o[1]) signal. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCMD1_SHIFT 8
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCMD1_MASK 0xF00
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCMD1(v)  (((v) << 8) & 0xF00)
/**< Field TRGTMODSEL1 - Target Time Register Mode for PPS1 Output This field indicates the Target Time registers (MAC_PPS1_TARGET_TIME_SECONDS and MAC_PPS1_TARGET_TIME_NANOSECONDS) mode for PPS1 output signal. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL1_SHIFT 13
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL1_MASK 0x6000
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL1(v) (((v) << 13) & 0x6000)
/**< Field MCGREN1 - MCGR Mode Enable for PPS1 Output This field enables the 1st PPS instance to operate in PPS or MCGR mode. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN1_SHIFT 15
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN1_MASK 0x8000
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN1(v)  (((v) << 15) & 0x8000)
/**< Field PPSCMD2 - Flexible PPS2 Output Control This field controls the flexible PPS2 output (ptp_pps_o[2]) signal. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCMD2_SHIFT 16
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCMD2_MASK 0xF0000
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCMD2(v)  (((v) << 16) & 0xF0000)
/**< Field TRGTMODSEL2 - Target Time Register Mode for PPS2 Output This field indicates the Target Time registers (MAC_PPS2_TARGET_TIME_SECONDS and MAC_PPS2_TARGET_TIME_NANOSECONDS) mode for PPS2 output signal. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL2_SHIFT 21
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL2_MASK 0x600000
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL2(v) (((v) << 21) & 0x600000)
/**< Field MCGREN2 - MCGR Mode Enable for PPS2 Output This field enables the 2nd PPS instance to operate in PPS or MCGR mode. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN2_SHIFT 23
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN2_MASK 0x800000
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN2(v)  (((v) << 23) & 0x800000)
/**< Field PPSCMD3 - Flexible PPS3 Output Control This field controls the flexible PPS3 output (ptp_pps_o[3]) signal. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCMD3_SHIFT 24
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCMD3_MASK 0xF000000
#define IMX_ENET_QOS_MAC_PPS_CONTROL_PPSCMD3(v)  (((v) << 24) & 0xF000000)
/**< Field TRGTMODSEL3 - Target Time Register Mode for PPS3 Output This field indicates the Target Time registers (MAC_PPS3_TARGET_TIME_SECONDS and MAC_PPS3_TARGET_TIME_NANOSECONDS) mode for PPS3 output signal. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL3_SHIFT 29
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL3_MASK 0x60000000
#define IMX_ENET_QOS_MAC_PPS_CONTROL_TRGTMODSEL3(v) (((v) << 29) & 0x60000000)
/**< Field MCGREN3 - MCGR Mode Enable for PPS3 Output This field enables the 3rd PPS instance to operate in PPS or MCGR mode. */
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN3_SHIFT 31
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN3_MASK 0x80000000
#define IMX_ENET_QOS_MAC_PPS_CONTROL_MCGREN3(v)  (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_SECONDS - PPS0 Target Time Seconds */
typedef union IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_SECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TSTRH0 : 32;                /**< PPS Target Time Seconds Register This field stores the time in seconds. */
    } B;
} IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_SECONDS_t;

/**< Register MAC_PPS0_TARGET_TIME_SECONDS - PPS0 Target Time Seconds */
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_SECONDS 0xB80
/**< Field TSTRH0 - PPS Target Time Seconds Register This field stores the time in seconds. */
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_SECONDS_TSTRH0_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_SECONDS_TSTRH0_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_SECONDS_TSTRH0(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS - PPS0 Target Time Nanoseconds */
typedef union IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TTSL0 : 31;                /**< Target Time Low for PPS Register This register stores the time in (signed) nanoseconds. */
        UINT32 TRGTBUSY0 : 1;                 /**< PPS Target Time Register Busy The MAC sets this bit when the PPSCMD0 field in the PPS_CONTROL register is programmed to 010 or 011. */
    } B;
} IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS_t;

/**< Register MAC_PPS0_TARGET_TIME_NANOSECONDS - PPS0 Target Time Nanoseconds */
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS 0xB84
/**< Field TTSL0 - Target Time Low for PPS Register This register stores the time in (signed) nanoseconds. */
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS_TTSL0_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS_TTSL0_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS_TTSL0(v) (((v) << 0) & 0x7FFFFFFF)
/**< Field TRGTBUSY0 - PPS Target Time Register Busy The MAC sets this bit when the PPSCMD0 field in the PPS_CONTROL register is programmed to 010 or 011. */
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS_TRGTBUSY0_SHIFT 31
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS_TRGTBUSY0_MASK 0x80000000
#define IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS_TRGTBUSY0(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_PPS0_INTERVAL - PPS0 Interval */
typedef union IMX_ENET_QOS_MAC_PPS0_INTERVAL_union_t {
    UINT32 R;
    struct {
        UINT32 PPSINT0 : 32;                /**< PPS Output Signal Interval These bits store the interval between the rising edges of PPS0 signal output. */
    } B;
} IMX_ENET_QOS_MAC_PPS0_INTERVAL_t;

/**< Register MAC_PPS0_INTERVAL - PPS0 Interval */
#define IMX_ENET_QOS_MAC_PPS0_INTERVAL           0xB88
/**< Field PPSINT0 - PPS Output Signal Interval These bits store the interval between the rising edges of PPS0 signal output. */
#define IMX_ENET_QOS_MAC_PPS0_INTERVAL_PPSINT0_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS0_INTERVAL_PPSINT0_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS0_INTERVAL_PPSINT0(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS0_WIDTH - PPS0 Width */
typedef union IMX_ENET_QOS_MAC_PPS0_WIDTH_union_t {
    UINT32 R;
    struct {
        UINT32 PPSWIDTH0 : 32;                /**< PPS Output Signal Width These bits store the width between the rising edge and corresponding falling edge of PPS0 signal output. */
    } B;
} IMX_ENET_QOS_MAC_PPS0_WIDTH_t;

/**< Register MAC_PPS0_WIDTH - PPS0 Width */
#define IMX_ENET_QOS_MAC_PPS0_WIDTH              0xB8C
/**< Field PPSWIDTH0 - PPS Output Signal Width These bits store the width between the rising edge and corresponding falling edge of PPS0 signal output. */
#define IMX_ENET_QOS_MAC_PPS0_WIDTH_PPSWIDTH0_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS0_WIDTH_PPSWIDTH0_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS0_WIDTH_PPSWIDTH0(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_SECONDS - PPS1 Target Time Seconds */
typedef union IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_SECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TSTRH1 : 32;                /**< PPS Target Time Seconds Register This field stores the time in seconds. */
    } B;
} IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_SECONDS_t;

/**< Register MAC_PPS1_TARGET_TIME_SECONDS - PPS1 Target Time Seconds */
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_SECONDS 0xB90
/**< Field TSTRH1 - PPS Target Time Seconds Register This field stores the time in seconds. */
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_SECONDS_TSTRH1_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_SECONDS_TSTRH1_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_SECONDS_TSTRH1(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS - PPS1 Target Time Nanoseconds */
typedef union IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TTSL1 : 31;                /**< Target Time Low for PPS Register This register stores the time in (signed) nanoseconds. */
        UINT32 TRGTBUSY1 : 1;                 /**< PPS Target Time Register Busy The MAC sets this bit when the PPSCMD0 field in the PPS_CONTROL register is programmed to 010 or 011. */
    } B;
} IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS_t;

/**< Register MAC_PPS1_TARGET_TIME_NANOSECONDS - PPS1 Target Time Nanoseconds */
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS 0xB94
/**< Field TTSL1 - Target Time Low for PPS Register This register stores the time in (signed) nanoseconds. */
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS_TTSL1_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS_TTSL1_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS_TTSL1(v) (((v) << 0) & 0x7FFFFFFF)
/**< Field TRGTBUSY1 - PPS Target Time Register Busy The MAC sets this bit when the PPSCMD0 field in the PPS_CONTROL register is programmed to 010 or 011. */
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS_TRGTBUSY1_SHIFT 31
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS_TRGTBUSY1_MASK 0x80000000
#define IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS_TRGTBUSY1(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_PPS1_INTERVAL - PPS1 Interval */
typedef union IMX_ENET_QOS_MAC_PPS1_INTERVAL_union_t {
    UINT32 R;
    struct {
        UINT32 PPSINT1 : 32;                /**< PPS Output Signal Interval These bits store the interval between the rising edges of PPS0 signal output. */
    } B;
} IMX_ENET_QOS_MAC_PPS1_INTERVAL_t;

/**< Register MAC_PPS1_INTERVAL - PPS1 Interval */
#define IMX_ENET_QOS_MAC_PPS1_INTERVAL           0xB98
/**< Field PPSINT1 - PPS Output Signal Interval These bits store the interval between the rising edges of PPS0 signal output. */
#define IMX_ENET_QOS_MAC_PPS1_INTERVAL_PPSINT1_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS1_INTERVAL_PPSINT1_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS1_INTERVAL_PPSINT1(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS1_WIDTH - PPS1 Width */
typedef union IMX_ENET_QOS_MAC_PPS1_WIDTH_union_t {
    UINT32 R;
    struct {
        UINT32 PPSWIDTH1 : 32;                /**< PPS Output Signal Width These bits store the width between the rising edge and corresponding falling edge of PPS0 signal output. */
    } B;
} IMX_ENET_QOS_MAC_PPS1_WIDTH_t;

/**< Register MAC_PPS1_WIDTH - PPS1 Width */
#define IMX_ENET_QOS_MAC_PPS1_WIDTH              0xB9C
/**< Field PPSWIDTH1 - PPS Output Signal Width These bits store the width between the rising edge and corresponding falling edge of PPS0 signal output. */
#define IMX_ENET_QOS_MAC_PPS1_WIDTH_PPSWIDTH1_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS1_WIDTH_PPSWIDTH1_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS1_WIDTH_PPSWIDTH1(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_SECONDS - PPS2 Target Time Seconds */
typedef union IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_SECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TSTRH2 : 32;                /**< PPS Target Time Seconds Register This field stores the time in seconds. */
    } B;
} IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_SECONDS_t;

/**< Register MAC_PPS2_TARGET_TIME_SECONDS - PPS2 Target Time Seconds */
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_SECONDS 0xBA0
/**< Field TSTRH2 - PPS Target Time Seconds Register This field stores the time in seconds. */
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_SECONDS_TSTRH2_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_SECONDS_TSTRH2_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_SECONDS_TSTRH2(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS - PPS2 Target Time Nanoseconds */
typedef union IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TTSL2 : 31;                /**< Target Time Low for PPS Register This register stores the time in (signed) nanoseconds. */
        UINT32 TRGTBUSY2 : 1;                 /**< PPS Target Time Register Busy The MAC sets this bit when the PPSCMD0 field in the PPS_CONTROL register is programmed to 010 or 011. */
    } B;
} IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS_t;

/**< Register MAC_PPS2_TARGET_TIME_NANOSECONDS - PPS2 Target Time Nanoseconds */
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS 0xBA4
/**< Field TTSL2 - Target Time Low for PPS Register This register stores the time in (signed) nanoseconds. */
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS_TTSL2_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS_TTSL2_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS_TTSL2(v) (((v) << 0) & 0x7FFFFFFF)
/**< Field TRGTBUSY2 - PPS Target Time Register Busy The MAC sets this bit when the PPSCMD0 field in the PPS_CONTROL register is programmed to 010 or 011. */
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS_TRGTBUSY2_SHIFT 31
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS_TRGTBUSY2_MASK 0x80000000
#define IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS_TRGTBUSY2(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_PPS2_INTERVAL - PPS2 Interval */
typedef union IMX_ENET_QOS_MAC_PPS2_INTERVAL_union_t {
    UINT32 R;
    struct {
        UINT32 PPSINT2 : 32;                /**< PPS Output Signal Interval These bits store the interval between the rising edges of PPS0 signal output. */
    } B;
} IMX_ENET_QOS_MAC_PPS2_INTERVAL_t;

/**< Register MAC_PPS2_INTERVAL - PPS2 Interval */
#define IMX_ENET_QOS_MAC_PPS2_INTERVAL           0xBA8
/**< Field PPSINT2 - PPS Output Signal Interval These bits store the interval between the rising edges of PPS0 signal output. */
#define IMX_ENET_QOS_MAC_PPS2_INTERVAL_PPSINT2_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS2_INTERVAL_PPSINT2_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS2_INTERVAL_PPSINT2(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS2_WIDTH - PPS2 Width */
typedef union IMX_ENET_QOS_MAC_PPS2_WIDTH_union_t {
    UINT32 R;
    struct {
        UINT32 PPSWIDTH2 : 32;                /**< PPS Output Signal Width These bits store the width between the rising edge and corresponding falling edge of PPS0 signal output. */
    } B;
} IMX_ENET_QOS_MAC_PPS2_WIDTH_t;

/**< Register MAC_PPS2_WIDTH - PPS2 Width */
#define IMX_ENET_QOS_MAC_PPS2_WIDTH              0xBAC
/**< Field PPSWIDTH2 - PPS Output Signal Width These bits store the width between the rising edge and corresponding falling edge of PPS0 signal output. */
#define IMX_ENET_QOS_MAC_PPS2_WIDTH_PPSWIDTH2_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS2_WIDTH_PPSWIDTH2_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS2_WIDTH_PPSWIDTH2(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_SECONDS - PPS3 Target Time Seconds */
typedef union IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_SECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TSTRH3 : 32;                /**< PPS Target Time Seconds Register This field stores the time in seconds. */
    } B;
} IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_SECONDS_t;

/**< Register MAC_PPS3_TARGET_TIME_SECONDS - PPS3 Target Time Seconds */
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_SECONDS 0xBB0
/**< Field TSTRH3 - PPS Target Time Seconds Register This field stores the time in seconds. */
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_SECONDS_TSTRH3_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_SECONDS_TSTRH3_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_SECONDS_TSTRH3(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS - PPS3 Target Time Nanoseconds */
typedef union IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS_union_t {
    UINT32 R;
    struct {
        UINT32 TTSL3 : 31;                /**< Target Time Low for PPS Register This register stores the time in (signed) nanoseconds. */
        UINT32 TRGTBUSY3 : 1;                 /**< PPS Target Time Register Busy The MAC sets this bit when the PPSCMD0 field in the PPS_CONTROL register is programmed to 010 or 011. */
    } B;
} IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS_t;

/**< Register MAC_PPS3_TARGET_TIME_NANOSECONDS - PPS3 Target Time Nanoseconds */
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS 0xBB4
/**< Field TTSL3 - Target Time Low for PPS Register This register stores the time in (signed) nanoseconds. */
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS_TTSL3_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS_TTSL3_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS_TTSL3(v) (((v) << 0) & 0x7FFFFFFF)
/**< Field TRGTBUSY3 - PPS Target Time Register Busy The MAC sets this bit when the PPSCMD0 field in the PPS_CONTROL register is programmed to 010 or 011. */
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS_TRGTBUSY3_SHIFT 31
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS_TRGTBUSY3_MASK 0x80000000
#define IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS_TRGTBUSY3(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MAC_PPS3_INTERVAL - PPS3 Interval */
typedef union IMX_ENET_QOS_MAC_PPS3_INTERVAL_union_t {
    UINT32 R;
    struct {
        UINT32 PPSINT3 : 32;                /**< PPS Output Signal Interval These bits store the interval between the rising edges of PPS0 signal output. */
    } B;
} IMX_ENET_QOS_MAC_PPS3_INTERVAL_t;

/**< Register MAC_PPS3_INTERVAL - PPS3 Interval */
#define IMX_ENET_QOS_MAC_PPS3_INTERVAL           0xBB8
/**< Field PPSINT3 - PPS Output Signal Interval These bits store the interval between the rising edges of PPS0 signal output. */
#define IMX_ENET_QOS_MAC_PPS3_INTERVAL_PPSINT3_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS3_INTERVAL_PPSINT3_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS3_INTERVAL_PPSINT3(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PPS3_WIDTH - PPS3 Width */
typedef union IMX_ENET_QOS_MAC_PPS3_WIDTH_union_t {
    UINT32 R;
    struct {
        UINT32 PPSWIDTH3 : 32;                /**< PPS Output Signal Width These bits store the width between the rising edge and corresponding falling edge of PPS0 signal output. */
    } B;
} IMX_ENET_QOS_MAC_PPS3_WIDTH_t;

/**< Register MAC_PPS3_WIDTH - PPS3 Width */
#define IMX_ENET_QOS_MAC_PPS3_WIDTH              0xBBC
/**< Field PPSWIDTH3 - PPS Output Signal Width These bits store the width between the rising edge and corresponding falling edge of PPS0 signal output. */
#define IMX_ENET_QOS_MAC_PPS3_WIDTH_PPSWIDTH3_SHIFT 0
#define IMX_ENET_QOS_MAC_PPS3_WIDTH_PPSWIDTH3_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_PPS3_WIDTH_PPSWIDTH3(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_PTO_CONTROL - PTP Offload Engine Control */
typedef union IMX_ENET_QOS_MAC_PTO_CONTROL_union_t {
    UINT32 R;
    struct {
        UINT32 PTOEN : 1;                 /**< PTP Offload Enable When this bit is set, the PTP Offload feature is enabled. */
        UINT32 ASYNCEN : 1;                 /**< Automatic PTP SYNC message Enable When this bit is set, PTP SYNC message is generated periodically based on interval programmed or trigger from application, when the MAC is programmed to be in Clock Master mode. */
        UINT32 APDREQEN : 1;                 /**< Automatic PTP Pdelay_Req message Enable When this bit is set, PTP Pdelay_Req message is generated periodically based on interval programmed or trigger from application, when the MAC is programmed to be in Peer-to-Peer Transparent mode. */
        UINT32 _unused3 : 1;                 /**< Reserved. */
        UINT32 ASYNCTRIG : 1;                 /**< Automatic PTP SYNC message Trigger When this bit is set, one PTP SYNC message is transmitted. */
        UINT32 APDREQTRIG : 1;                 /**< Automatic PTP Pdelay_Req message Trigger When this bit is set, one PTP Pdelay_Req message is transmitted. */
        UINT32 DRRDIS : 1;                 /**< Disable PTO Delay Request/Response response generation When this bit is set, the Delay Request and Delay response is not generated for received SYNC and Delay request packet respectively, as required by the programmed mode. */
        UINT32 PDRDIS : 1;                 /**< Disable Peer Delay Response response generation When this bit is set, the Peer Delay Response (Pdelay_Resp) response is not be generated for received Peer Delay Request (Pdelay_Req) request packet, as required by the programmed mode. */
        UINT32 DN : 8;                 /**< Domain Number This field indicates the domain Number in which the PTP node is operating. */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_PTO_CONTROL_t;

/**< Register MAC_PTO_CONTROL - PTP Offload Engine Control */
#define IMX_ENET_QOS_MAC_PTO_CONTROL             0xBC0
/**< Field PTOEN - PTP Offload Enable When this bit is set, the PTP Offload feature is enabled. */
#define IMX_ENET_QOS_MAC_PTO_CONTROL_PTOEN_SHIFT 0
#define IMX_ENET_QOS_MAC_PTO_CONTROL_PTOEN_MASK  0x1
#define IMX_ENET_QOS_MAC_PTO_CONTROL_PTOEN(v)    (((v) << 0) & 0x1)
/**< Field ASYNCEN - Automatic PTP SYNC message Enable When this bit is set, PTP SYNC message is generated periodically based on interval programmed or trigger from application, when the MAC is programmed to be in Clock Master mode. */
#define IMX_ENET_QOS_MAC_PTO_CONTROL_ASYNCEN_SHIFT 1
#define IMX_ENET_QOS_MAC_PTO_CONTROL_ASYNCEN_MASK 0x2
#define IMX_ENET_QOS_MAC_PTO_CONTROL_ASYNCEN(v)  (((v) << 1) & 0x2)
/**< Field APDREQEN - Automatic PTP Pdelay_Req message Enable When this bit is set, PTP Pdelay_Req message is generated periodically based on interval programmed or trigger from application, when the MAC is programmed to be in Peer-to-Peer Transparent mode. */
#define IMX_ENET_QOS_MAC_PTO_CONTROL_APDREQEN_SHIFT 2
#define IMX_ENET_QOS_MAC_PTO_CONTROL_APDREQEN_MASK 0x4
#define IMX_ENET_QOS_MAC_PTO_CONTROL_APDREQEN(v) (((v) << 2) & 0x4)
/**< Field ASYNCTRIG - Automatic PTP SYNC message Trigger When this bit is set, one PTP SYNC message is transmitted. */
#define IMX_ENET_QOS_MAC_PTO_CONTROL_ASYNCTRIG_SHIFT 4
#define IMX_ENET_QOS_MAC_PTO_CONTROL_ASYNCTRIG_MASK 0x10
#define IMX_ENET_QOS_MAC_PTO_CONTROL_ASYNCTRIG(v) (((v) << 4) & 0x10)
/**< Field APDREQTRIG - Automatic PTP Pdelay_Req message Trigger When this bit is set, one PTP Pdelay_Req message is transmitted. */
#define IMX_ENET_QOS_MAC_PTO_CONTROL_APDREQTRIG_SHIFT 5
#define IMX_ENET_QOS_MAC_PTO_CONTROL_APDREQTRIG_MASK 0x20
#define IMX_ENET_QOS_MAC_PTO_CONTROL_APDREQTRIG(v) (((v) << 5) & 0x20)
/**< Field DRRDIS - Disable PTO Delay Request/Response response generation When this bit is set, the Delay Request and Delay response is not generated for received SYNC and Delay request packet respectively, as required by the programmed mode. */
#define IMX_ENET_QOS_MAC_PTO_CONTROL_DRRDIS_SHIFT 6
#define IMX_ENET_QOS_MAC_PTO_CONTROL_DRRDIS_MASK 0x40
#define IMX_ENET_QOS_MAC_PTO_CONTROL_DRRDIS(v)   (((v) << 6) & 0x40)
/**< Field PDRDIS - Disable Peer Delay Response response generation When this bit is set, the Peer Delay Response (Pdelay_Resp) response is not be generated for received Peer Delay Request (Pdelay_Req) request packet, as required by the programmed mode. */
#define IMX_ENET_QOS_MAC_PTO_CONTROL_PDRDIS_SHIFT 7
#define IMX_ENET_QOS_MAC_PTO_CONTROL_PDRDIS_MASK 0x80
#define IMX_ENET_QOS_MAC_PTO_CONTROL_PDRDIS(v)   (((v) << 7) & 0x80)
/**< Field DN - Domain Number This field indicates the domain Number in which the PTP node is operating. */
#define IMX_ENET_QOS_MAC_PTO_CONTROL_DN_SHIFT    8
#define IMX_ENET_QOS_MAC_PTO_CONTROL_DN_MASK     0xFF00
#define IMX_ENET_QOS_MAC_PTO_CONTROL_DN(v)       (((v) << 8) & 0xFF00)

/**< IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY0 - Source Port Identity 0 */
typedef union IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY0_union_t {
    UINT32 R;
    struct {
        UINT32 SPI0 : 32;                /**< Source Port Identity 0 This field indicates bits [31:0] of sourcePortIdentity of PTP node. */
    } B;
} IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY0_t;

/**< Register MAC_SOURCE_PORT_IDENTITY0 - Source Port Identity 0 */
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY0   0xBC4
/**< Field SPI0 - Source Port Identity 0 This field indicates bits [31:0] of sourcePortIdentity of PTP node. */
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY0_SPI0_SHIFT 0
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY0_SPI0_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY0_SPI0(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY1 - Source Port Identity 1 */
typedef union IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY1_union_t {
    UINT32 R;
    struct {
        UINT32 SPI1 : 32;                /**< Source Port Identity 1 This field indicates bits [63:32] of sourcePortIdentity of PTP node. */
    } B;
} IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY1_t;

/**< Register MAC_SOURCE_PORT_IDENTITY1 - Source Port Identity 1 */
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY1   0xBC8
/**< Field SPI1 - Source Port Identity 1 This field indicates bits [63:32] of sourcePortIdentity of PTP node. */
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY1_SPI1_SHIFT 0
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY1_SPI1_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY1_SPI1(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY2 - Source Port Identity 2 */
typedef union IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY2_union_t {
    UINT32 R;
    struct {
        UINT32 SPI2 : 16;                /**< Source Port Identity 2 This field indicates bits [79:64] of sourcePortIdentity of PTP node. */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY2_t;

/**< Register MAC_SOURCE_PORT_IDENTITY2 - Source Port Identity 2 */
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY2   0xBCC
/**< Field SPI2 - Source Port Identity 2 This field indicates bits [79:64] of sourcePortIdentity of PTP node. */
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY2_SPI2_SHIFT 0
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY2_SPI2_MASK 0xFFFF
#define IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY2_SPI2(v) (((v) << 0) & 0xFFFF)

/**< IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL - Log Message Interval */
typedef union IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_union_t {
    UINT32 R;
    struct {
        UINT32 LSI : 8;                 /**< Log Sync Interval This field indicates the periodicity of the automatically generated SYNC message when the PTP node is Master. */
        UINT32 DRSYNCR : 3;                 /**< Delay_Req to SYNC Ratio In Slave mode, it is used for controlling frequency of Delay_Req messages transmitted. */
        UINT32 _unused11 : 13;                /**< Reserved. */
        UINT32 LMPDRI : 8;                 /**< Log Min Pdelay_Req Interval This field indicates logMinPdelayReqInterval of PTP node. */
    } B;
} IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_t;

/**< Register MAC_LOG_MESSAGE_INTERVAL - Log Message Interval */
#define IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL    0xBD0
/**< Field LSI - Log Sync Interval This field indicates the periodicity of the automatically generated SYNC message when the PTP node is Master. */
#define IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_LSI_SHIFT 0
#define IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_LSI_MASK 0xFF
#define IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_LSI(v) (((v) << 0) & 0xFF)
/**< Field DRSYNCR - Delay_Req to SYNC Ratio In Slave mode, it is used for controlling frequency of Delay_Req messages transmitted. */
#define IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_DRSYNCR_SHIFT 8
#define IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_DRSYNCR_MASK 0x700
#define IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_DRSYNCR(v) (((v) << 8) & 0x700)
/**< Field LMPDRI - Log Min Pdelay_Req Interval This field indicates logMinPdelayReqInterval of PTP node. */
#define IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_LMPDRI_SHIFT 24
#define IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_LMPDRI_MASK 0xFF000000
#define IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_LMPDRI(v) (((v) << 24) & 0xFF000000)

/**< IMX_ENET_QOS_MTL_OPERATION_MODE - MTL Operation Mode */
typedef union IMX_ENET_QOS_MTL_OPERATION_MODE_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 1;                 /**< Reserved. */
        UINT32 DTXSTS : 1;                 /**< Drop Transmit Status When this bit is set, the Tx packet status received from the MAC is dropped in the MTL. */
        UINT32 RAA : 1;                 /**< Receive Arbitration Algorithm This field is used to select the arbitration algorithm for the Rx side. */
        UINT32 _unused3 : 2;                 /**< Reserved. */
        UINT32 SCHALG : 2;                 /**< Tx Scheduling Algorithm This field indicates the algorithm for Tx scheduling: */
        UINT32 _unused7 : 1;                 /**< Reserved. */
        UINT32 CNTPRST : 1;                 /**< Counters Preset When this bit is set, - MTL_TxQ[0-7]_Underflow register is initialized/preset to 12'h7F0. */
        UINT32 CNTCLR : 1;                 /**< Counters Reset When this bit is set, all counters are reset. */
        UINT32 _unused10 : 5;                 /**< Reserved. */
        UINT32 FRPE : 1;                 /**< Flexible Rx parser Enable When this bit is set to 1, the Programmable Rx Parser functionality is enabled. */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_OPERATION_MODE_t;

/**< Register MTL_OPERATION_MODE - MTL Operation Mode */
#define IMX_ENET_QOS_MTL_OPERATION_MODE          0xC00
/**< Field DTXSTS - Drop Transmit Status When this bit is set, the Tx packet status received from the MAC is dropped in the MTL. */
#define IMX_ENET_QOS_MTL_OPERATION_MODE_DTXSTS_SHIFT 1
#define IMX_ENET_QOS_MTL_OPERATION_MODE_DTXSTS_MASK 0x2
#define IMX_ENET_QOS_MTL_OPERATION_MODE_DTXSTS(v) (((v) << 1) & 0x2)
/**< Field RAA - Receive Arbitration Algorithm This field is used to select the arbitration algorithm for the Rx side. */
#define IMX_ENET_QOS_MTL_OPERATION_MODE_RAA_SHIFT 2
#define IMX_ENET_QOS_MTL_OPERATION_MODE_RAA_MASK 0x4
#define IMX_ENET_QOS_MTL_OPERATION_MODE_RAA(v)   (((v) << 2) & 0x4)
/**< Field SCHALG - Tx Scheduling Algorithm This field indicates the algorithm for Tx scheduling: */
#define IMX_ENET_QOS_MTL_OPERATION_MODE_SCHALG_SHIFT 5
#define IMX_ENET_QOS_MTL_OPERATION_MODE_SCHALG_MASK 0x60
#define IMX_ENET_QOS_MTL_OPERATION_MODE_SCHALG(v) (((v) << 5) & 0x60)
/**< Field CNTPRST - Counters Preset When this bit is set, - MTL_TxQ[0-7]_Underflow register is initialized/preset to 12'h7F0. */
#define IMX_ENET_QOS_MTL_OPERATION_MODE_CNTPRST_SHIFT 8
#define IMX_ENET_QOS_MTL_OPERATION_MODE_CNTPRST_MASK 0x100
#define IMX_ENET_QOS_MTL_OPERATION_MODE_CNTPRST(v) (((v) << 8) & 0x100)
/**< Field CNTCLR - Counters Reset When this bit is set, all counters are reset. */
#define IMX_ENET_QOS_MTL_OPERATION_MODE_CNTCLR_SHIFT 9
#define IMX_ENET_QOS_MTL_OPERATION_MODE_CNTCLR_MASK 0x200
#define IMX_ENET_QOS_MTL_OPERATION_MODE_CNTCLR(v) (((v) << 9) & 0x200)
/**< Field FRPE - Flexible Rx parser Enable When this bit is set to 1, the Programmable Rx Parser functionality is enabled. */
#define IMX_ENET_QOS_MTL_OPERATION_MODE_FRPE_SHIFT 15
#define IMX_ENET_QOS_MTL_OPERATION_MODE_FRPE_MASK 0x8000
#define IMX_ENET_QOS_MTL_OPERATION_MODE_FRPE(v)  (((v) << 15) & 0x8000)

/**< IMX_ENET_QOS_MTL_DBG_CTL - FIFO Debug Access Control and Status */
typedef union IMX_ENET_QOS_MTL_DBG_CTL_union_t {
    UINT32 R;
    struct {
        UINT32 FDBGEN : 1;                 /**< FIFO Debug Access Enable When this bit is set, it indicates that the debug mode access to the FIFO is enabled. */
        UINT32 DBGMOD : 1;                 /**< Debug Mode Access to FIFO When this bit is set, it indicates that the current access to the FIFO is read, write, and debug access. */
        UINT32 BYTEEN : 2;                 /**< Byte Enables This field indicates the number of data bytes valid in the data register during Write operation. */
        UINT32 _unused4 : 1;                 /**< Reserved. */
        UINT32 PKTSTATE : 2;                 /**< Encoded Packet State This field is used to write the control information to the Tx FIFO or Rx FIFO. */
        UINT32 _unused7 : 1;                 /**< Reserved. */
        UINT32 RSTALL : 1;                 /**< Reset All Pointers When this bit is set, the pointers of all FIFOs are reset when FIFO Debug Access is enabled. */
        UINT32 RSTSEL : 1;                 /**< Reset Pointers of Selected FIFO When this bit is set, the pointers of the currently-selected FIFO are reset when FIFO Debug Access is enabled. */
        UINT32 FIFORDEN : 1;                 /**< FIFO Read Enable When this bit is set, it enables the Read operation on selected FIFO when FIFO Debug Access is enabled. */
        UINT32 FIFOWREN : 1;                 /**< FIFO Write Enable When this bit is set, it enables the Write operation on selected FIFO when FIFO Debug Access is enabled. */
        UINT32 FIFOSEL : 2;                 /**< FIFO Selected for Access This field indicates the FIFO selected for debug access: */
        UINT32 PKTIE : 1;                 /**< Receive Packet Available Interrupt Status Enable When this bit is set, an interrupt is generated when EOP of received packet is written to the Rx FIFO. */
        UINT32 STSIE : 1;                 /**< Transmit Status Available Interrupt Status Enable When this bit is set, an interrupt is generated when Transmit status is available in slave mode. */
        UINT32 _unused16 : 1;                 /**< Reserved. */
        UINT32 _unused17 : 2;                 /**< Reserved. */
        UINT32 _unused19 : 13;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_DBG_CTL_t;

/**< Register MTL_DBG_CTL - FIFO Debug Access Control and Status */
#define IMX_ENET_QOS_MTL_DBG_CTL                 0xC08
/**< Field FDBGEN - FIFO Debug Access Enable When this bit is set, it indicates that the debug mode access to the FIFO is enabled. */
#define IMX_ENET_QOS_MTL_DBG_CTL_FDBGEN_SHIFT    0
#define IMX_ENET_QOS_MTL_DBG_CTL_FDBGEN_MASK     0x1
#define IMX_ENET_QOS_MTL_DBG_CTL_FDBGEN(v)       (((v) << 0) & 0x1)
/**< Field DBGMOD - Debug Mode Access to FIFO When this bit is set, it indicates that the current access to the FIFO is read, write, and debug access. */
#define IMX_ENET_QOS_MTL_DBG_CTL_DBGMOD_SHIFT    1
#define IMX_ENET_QOS_MTL_DBG_CTL_DBGMOD_MASK     0x2
#define IMX_ENET_QOS_MTL_DBG_CTL_DBGMOD(v)       (((v) << 1) & 0x2)
/**< Field BYTEEN - Byte Enables This field indicates the number of data bytes valid in the data register during Write operation. */
#define IMX_ENET_QOS_MTL_DBG_CTL_BYTEEN_SHIFT    2
#define IMX_ENET_QOS_MTL_DBG_CTL_BYTEEN_MASK     0xC
#define IMX_ENET_QOS_MTL_DBG_CTL_BYTEEN(v)       (((v) << 2) & 0xC)
/**< Field PKTSTATE - Encoded Packet State This field is used to write the control information to the Tx FIFO or Rx FIFO. */
#define IMX_ENET_QOS_MTL_DBG_CTL_PKTSTATE_SHIFT  5
#define IMX_ENET_QOS_MTL_DBG_CTL_PKTSTATE_MASK   0x60
#define IMX_ENET_QOS_MTL_DBG_CTL_PKTSTATE(v)     (((v) << 5) & 0x60)
/**< Field RSTALL - Reset All Pointers When this bit is set, the pointers of all FIFOs are reset when FIFO Debug Access is enabled. */
#define IMX_ENET_QOS_MTL_DBG_CTL_RSTALL_SHIFT    8
#define IMX_ENET_QOS_MTL_DBG_CTL_RSTALL_MASK     0x100
#define IMX_ENET_QOS_MTL_DBG_CTL_RSTALL(v)       (((v) << 8) & 0x100)
/**< Field RSTSEL - Reset Pointers of Selected FIFO When this bit is set, the pointers of the currently-selected FIFO are reset when FIFO Debug Access is enabled. */
#define IMX_ENET_QOS_MTL_DBG_CTL_RSTSEL_SHIFT    9
#define IMX_ENET_QOS_MTL_DBG_CTL_RSTSEL_MASK     0x200
#define IMX_ENET_QOS_MTL_DBG_CTL_RSTSEL(v)       (((v) << 9) & 0x200)
/**< Field FIFORDEN - FIFO Read Enable When this bit is set, it enables the Read operation on selected FIFO when FIFO Debug Access is enabled. */
#define IMX_ENET_QOS_MTL_DBG_CTL_FIFORDEN_SHIFT  10
#define IMX_ENET_QOS_MTL_DBG_CTL_FIFORDEN_MASK   0x400
#define IMX_ENET_QOS_MTL_DBG_CTL_FIFORDEN(v)     (((v) << 10) & 0x400)
/**< Field FIFOWREN - FIFO Write Enable When this bit is set, it enables the Write operation on selected FIFO when FIFO Debug Access is enabled. */
#define IMX_ENET_QOS_MTL_DBG_CTL_FIFOWREN_SHIFT  11
#define IMX_ENET_QOS_MTL_DBG_CTL_FIFOWREN_MASK   0x800
#define IMX_ENET_QOS_MTL_DBG_CTL_FIFOWREN(v)     (((v) << 11) & 0x800)
/**< Field FIFOSEL - FIFO Selected for Access This field indicates the FIFO selected for debug access: */
#define IMX_ENET_QOS_MTL_DBG_CTL_FIFOSEL_SHIFT   12
#define IMX_ENET_QOS_MTL_DBG_CTL_FIFOSEL_MASK    0x3000
#define IMX_ENET_QOS_MTL_DBG_CTL_FIFOSEL(v)      (((v) << 12) & 0x3000)
/**< Field PKTIE - Receive Packet Available Interrupt Status Enable When this bit is set, an interrupt is generated when EOP of received packet is written to the Rx FIFO. */
#define IMX_ENET_QOS_MTL_DBG_CTL_PKTIE_SHIFT     14
#define IMX_ENET_QOS_MTL_DBG_CTL_PKTIE_MASK      0x4000
#define IMX_ENET_QOS_MTL_DBG_CTL_PKTIE(v)        (((v) << 14) & 0x4000)
/**< Field STSIE - Transmit Status Available Interrupt Status Enable When this bit is set, an interrupt is generated when Transmit status is available in slave mode. */
#define IMX_ENET_QOS_MTL_DBG_CTL_STSIE_SHIFT     15
#define IMX_ENET_QOS_MTL_DBG_CTL_STSIE_MASK      0x8000
#define IMX_ENET_QOS_MTL_DBG_CTL_STSIE(v)        (((v) << 15) & 0x8000)

/**< IMX_ENET_QOS_MTL_DBG_STS - FIFO Debug Status */
typedef union IMX_ENET_QOS_MTL_DBG_STS_union_t {
    UINT32 R;
    struct {
        UINT32 FIFOBUSY : 1;                 /**< FIFO Busy When set, this bit indicates that a FIFO operation is in progress in the MAC and content of the following fields is not valid: - All other fields of this register - All fields of the MTL_FIFO_DEBUG_DATA register */
        UINT32 PKTSTATE : 2;                 /**< Encoded Packet State This field is used to get the control or status information of the selected FIFO. */
        UINT32 BYTEEN : 2;                 /**< Byte Enables This field indicates the number of data bytes valid in the data register during Read operation. */
        UINT32 _unused5 : 3;                 /**< Reserved. */
        UINT32 PKTI : 1;                 /**< Receive Packet Available Interrupt Status When set, this bit indicates that MAC layer has written the EOP of received packet to the Rx FIFO. */
        UINT32 STSI : 1;                 /**< Transmit Status Available Interrupt Status When set, this bit indicates that the Slave mode Tx packet is transmitted, and the status is available in Tx Status FIFO. */
        UINT32 _unused10 : 5;                 /**< Reserved. */
        UINT32 LOCR : 17;                /**< Remaining Locations in the FIFO Slave Access Mode: This field indicates the space available in selected FIFO. */
    } B;
} IMX_ENET_QOS_MTL_DBG_STS_t;

/**< Register MTL_DBG_STS - FIFO Debug Status */
#define IMX_ENET_QOS_MTL_DBG_STS                 0xC0C
/**< Field FIFOBUSY - FIFO Busy When set, this bit indicates that a FIFO operation is in progress in the MAC and content of the following fields is not valid: - All other fields of this register - All fields of the MTL_FIFO_DEBUG_DATA register */
#define IMX_ENET_QOS_MTL_DBG_STS_FIFOBUSY_SHIFT  0
#define IMX_ENET_QOS_MTL_DBG_STS_FIFOBUSY_MASK   0x1
#define IMX_ENET_QOS_MTL_DBG_STS_FIFOBUSY(v)     (((v) << 0) & 0x1)
/**< Field PKTSTATE - Encoded Packet State This field is used to get the control or status information of the selected FIFO. */
#define IMX_ENET_QOS_MTL_DBG_STS_PKTSTATE_SHIFT  1
#define IMX_ENET_QOS_MTL_DBG_STS_PKTSTATE_MASK   0x6
#define IMX_ENET_QOS_MTL_DBG_STS_PKTSTATE(v)     (((v) << 1) & 0x6)
/**< Field BYTEEN - Byte Enables This field indicates the number of data bytes valid in the data register during Read operation. */
#define IMX_ENET_QOS_MTL_DBG_STS_BYTEEN_SHIFT    3
#define IMX_ENET_QOS_MTL_DBG_STS_BYTEEN_MASK     0x18
#define IMX_ENET_QOS_MTL_DBG_STS_BYTEEN(v)       (((v) << 3) & 0x18)
/**< Field PKTI - Receive Packet Available Interrupt Status When set, this bit indicates that MAC layer has written the EOP of received packet to the Rx FIFO. */
#define IMX_ENET_QOS_MTL_DBG_STS_PKTI_SHIFT      8
#define IMX_ENET_QOS_MTL_DBG_STS_PKTI_MASK       0x100
#define IMX_ENET_QOS_MTL_DBG_STS_PKTI(v)         (((v) << 8) & 0x100)
/**< Field STSI - Transmit Status Available Interrupt Status When set, this bit indicates that the Slave mode Tx packet is transmitted, and the status is available in Tx Status FIFO. */
#define IMX_ENET_QOS_MTL_DBG_STS_STSI_SHIFT      9
#define IMX_ENET_QOS_MTL_DBG_STS_STSI_MASK       0x200
#define IMX_ENET_QOS_MTL_DBG_STS_STSI(v)         (((v) << 9) & 0x200)
/**< Field LOCR - Remaining Locations in the FIFO Slave Access Mode: This field indicates the space available in selected FIFO. */
#define IMX_ENET_QOS_MTL_DBG_STS_LOCR_SHIFT      15
#define IMX_ENET_QOS_MTL_DBG_STS_LOCR_MASK       0xFFFF8000
#define IMX_ENET_QOS_MTL_DBG_STS_LOCR(v)         (((v) << 15) & 0xFFFF8000)

/**< IMX_ENET_QOS_MTL_FIFO_DEBUG_DATA - FIFO Debug Data */
typedef union IMX_ENET_QOS_MTL_FIFO_DEBUG_DATA_union_t {
    UINT32 R;
    struct {
        UINT32 FDBGDATA : 32;                /**< FIFO Debug Data During debug or slave access write operation, this field contains the data to be written to the Tx FIFO, Rx FIFO, or TSO FIFO. */
    } B;
} IMX_ENET_QOS_MTL_FIFO_DEBUG_DATA_t;

/**< Register MTL_FIFO_DEBUG_DATA - FIFO Debug Data */
#define IMX_ENET_QOS_MTL_FIFO_DEBUG_DATA         0xC10
/**< Field FDBGDATA - FIFO Debug Data During debug or slave access write operation, this field contains the data to be written to the Tx FIFO, Rx FIFO, or TSO FIFO. */
#define IMX_ENET_QOS_MTL_FIFO_DEBUG_DATA_FDBGDATA_SHIFT 0
#define IMX_ENET_QOS_MTL_FIFO_DEBUG_DATA_FDBGDATA_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MTL_FIFO_DEBUG_DATA_FDBGDATA(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MTL_INTERRUPT_STATUS - MTL Interrupt Status */
typedef union IMX_ENET_QOS_MTL_INTERRUPT_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 Q0IS : 1;                 /**< Queue 0 Interrupt status This bit indicates that there is an interrupt from Queue 0. */
        UINT32 Q1IS : 1;                 /**< Queue 1 Interrupt status This bit indicates that there is an interrupt from Queue 1. */
        UINT32 Q2IS : 1;                 /**< Queue 2 Interrupt status This bit indicates that there is an interrupt from Queue 2. */
        UINT32 Q3IS : 1;                 /**< Queue 3 Interrupt status This bit indicates that there is an interrupt from Queue 3. */
        UINT32 Q4IS : 1;                 /**< Queue 4 Interrupt status This bit indicates that there is an interrupt from Queue 4. */
        UINT32 _unused5 : 1;                 /**< Reserved. */
        UINT32 _unused6 : 1;                 /**< Reserved. */
        UINT32 _unused7 : 1;                 /**< Reserved. */
        UINT32 _unused8 : 8;                 /**< Reserved. */
        UINT32 _unused16 : 1;                 /**< Reserved. */
        UINT32 DBGIS : 1;                 /**< Debug Interrupt status This bit indicates an interrupt event during the slave access. */
        UINT32 ESTIS : 1;                 /**< EST (TAS- 802. */
        UINT32 _unused19 : 4;                 /**< Reserved. */
        UINT32 MTLPIS : 1;                 /**< MTL Rx Parser Interrupt Status This bit indicates that there is an interrupt from Rx Parser Block. */
        UINT32 _unused24 : 8;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_INTERRUPT_STATUS_t;

/**< Register MTL_INTERRUPT_STATUS - MTL Interrupt Status */
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS        0xC20
/**< Field Q0IS - Queue 0 Interrupt status This bit indicates that there is an interrupt from Queue 0. */
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q0IS_SHIFT 0
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q0IS_MASK 0x1
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q0IS(v) (((v) << 0) & 0x1)
/**< Field Q1IS - Queue 1 Interrupt status This bit indicates that there is an interrupt from Queue 1. */
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q1IS_SHIFT 1
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q1IS_MASK 0x2
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q1IS(v) (((v) << 1) & 0x2)
/**< Field Q2IS - Queue 2 Interrupt status This bit indicates that there is an interrupt from Queue 2. */
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q2IS_SHIFT 2
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q2IS_MASK 0x4
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q2IS(v) (((v) << 2) & 0x4)
/**< Field Q3IS - Queue 3 Interrupt status This bit indicates that there is an interrupt from Queue 3. */
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q3IS_SHIFT 3
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q3IS_MASK 0x8
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q3IS(v) (((v) << 3) & 0x8)
/**< Field Q4IS - Queue 4 Interrupt status This bit indicates that there is an interrupt from Queue 4. */
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q4IS_SHIFT 4
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q4IS_MASK 0x10
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_Q4IS(v) (((v) << 4) & 0x10)
/**< Field DBGIS - Debug Interrupt status This bit indicates an interrupt event during the slave access. */
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_DBGIS_SHIFT 17
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_DBGIS_MASK 0x20000
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_DBGIS(v) (((v) << 17) & 0x20000)
/**< Field ESTIS - EST (TAS- 802. */
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_ESTIS_SHIFT 18
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_ESTIS_MASK 0x40000
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_ESTIS(v) (((v) << 18) & 0x40000)
/**< Field MTLPIS - MTL Rx Parser Interrupt Status This bit indicates that there is an interrupt from Rx Parser Block. */
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_MTLPIS_SHIFT 23
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_MTLPIS_MASK 0x800000
#define IMX_ENET_QOS_MTL_INTERRUPT_STATUS_MTLPIS(v) (((v) << 23) & 0x800000)

/**< IMX_ENET_QOS_MTL_RXQ_DMA_MAP0 - Receive Queue and DMA Channel Mapping 0 */
typedef union IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_union_t {
    UINT32 R;
    struct {
        UINT32 Q0MDMACH : 3;                 /**< Queue 0 Mapped to DMA Channel This field controls the routing of the packet received in Queue 0 to the DMA channel: - 000: DMA Channel 0 - 001: DMA Channel 1 - 010: DMA Channel 2 - 011: DMA Channel 3 - 100: DMA Channel 4 - 101: Reserved - 110: Reserved - 111: Reserved This field is valid when the Q0DDMACH field is reset. */
        UINT32 _unused3 : 1;                 /**< Reserved. */
        UINT32 Q0DDMACH : 1;                 /**< Queue 0 Enabled for DA-based DMA Channel Selection When set, this bit indicates that the packets received in Queue 0 are routed to a particular DMA channel as decided in the MAC Receiver based on the DMA channel number programmed in the L3-L4 filter registers, or the Ethernet DA address. */
        UINT32 _unused5 : 3;                 /**< Reserved. */
        UINT32 Q1MDMACH : 3;                 /**< Queue 1 Mapped to DMA Channel This field controls the routing of the received packet in Queue 1 to the DMA channel: - 000: DMA Channel 0 - 001: DMA Channel 1 - 010: DMA Channel 2 - 011: DMA Channel 3 - 100: DMA Channel 4 - 101: Reserved - 110: Reserved - 111: Reserved This field is valid when the Q1DDMACH field is reset. */
        UINT32 _unused11 : 1;                 /**< Reserved. */
        UINT32 Q1DDMACH : 1;                 /**< Queue 1 Enabled for DA-based DMA Channel Selection When set, this bit indicates that the packets received in Queue 1 are routed to a particular DMA channel as decided in the MAC Receiver based on the DMA channel number programmed in the L3-L4 filter registers, or the Ethernet DA address. */
        UINT32 _unused13 : 3;                 /**< Reserved. */
        UINT32 Q2MDMACH : 3;                 /**< Queue 2 Mapped to DMA Channel This field controls the routing of the received packet in Queue 2 to the DMA channel: - 000: DMA Channel 0 - 001: DMA Channel 1 - 010: DMA Channel 2 - 011: DMA Channel 3 - 100: DMA Channel 4 - 101: Reserved - 110: Reserved - 111: Reserved This field is valid when the Q2DDMACH field is reset. */
        UINT32 _unused19 : 1;                 /**< Reserved. */
        UINT32 Q2DDMACH : 1;                 /**< Queue 2 Enabled for DA-based DMA Channel Selection When set, this bit indicates that the packets received in Queue 2 are routed to a particular DMA channel as decided in the MAC Receiver based on the DMA channel number programmed in the L3-L4 filter registers, or the Ethernet DA address. */
        UINT32 _unused21 : 3;                 /**< Reserved. */
        UINT32 Q3MDMACH : 3;                 /**< Queue 3 Mapped to DMA Channel This field controls the routing of the received packet in Queue 3 to the DMA channel: - 000: DMA Channel 0 - 001: DMA Channel 1 - 010: DMA Channel 2 - 011: DMA Channel 3 - 100: DMA Channel 4 - 101: Reserved - 110: Reserved - 111: Reserved This field is valid when the Q3DDMACH field is reset. */
        UINT32 _unused27 : 1;                 /**< Reserved. */
        UINT32 Q3DDMACH : 1;                 /**< Queue 3 Enabled for Dynamic (per packet) DMA Channel Selection When set, this bit indicates that the packets received in Queue 3 are routed to a particular DMA channel as decided in the MAC Receiver based on the DMA channel number programmed in the L3-L4 filter registers, or the Ethernet DA address. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_t;

/**< Register MTL_RXQ_DMA_MAP0 - Receive Queue and DMA Channel Mapping 0 */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0            0xC30
/**< Field Q0MDMACH - Queue 0 Mapped to DMA Channel This field controls the routing of the packet received in Queue 0 to the DMA channel: - 000: DMA Channel 0 - 001: DMA Channel 1 - 010: DMA Channel 2 - 011: DMA Channel 3 - 100: DMA Channel 4 - 101: Reserved - 110: Reserved - 111: Reserved This field is valid when the Q0DDMACH field is reset. */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q0MDMACH_SHIFT 0
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q0MDMACH_MASK 0x7
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q0MDMACH(v) (((v) << 0) & 0x7)
/**< Field Q0DDMACH - Queue 0 Enabled for DA-based DMA Channel Selection When set, this bit indicates that the packets received in Queue 0 are routed to a particular DMA channel as decided in the MAC Receiver based on the DMA channel number programmed in the L3-L4 filter registers, or the Ethernet DA address. */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q0DDMACH_SHIFT 4
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q0DDMACH_MASK 0x10
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q0DDMACH(v) (((v) << 4) & 0x10)
/**< Field Q1MDMACH - Queue 1 Mapped to DMA Channel This field controls the routing of the received packet in Queue 1 to the DMA channel: - 000: DMA Channel 0 - 001: DMA Channel 1 - 010: DMA Channel 2 - 011: DMA Channel 3 - 100: DMA Channel 4 - 101: Reserved - 110: Reserved - 111: Reserved This field is valid when the Q1DDMACH field is reset. */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q1MDMACH_SHIFT 8
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q1MDMACH_MASK 0x700
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q1MDMACH(v) (((v) << 8) & 0x700)
/**< Field Q1DDMACH - Queue 1 Enabled for DA-based DMA Channel Selection When set, this bit indicates that the packets received in Queue 1 are routed to a particular DMA channel as decided in the MAC Receiver based on the DMA channel number programmed in the L3-L4 filter registers, or the Ethernet DA address. */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q1DDMACH_SHIFT 12
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q1DDMACH_MASK 0x1000
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q1DDMACH(v) (((v) << 12) & 0x1000)
/**< Field Q2MDMACH - Queue 2 Mapped to DMA Channel This field controls the routing of the received packet in Queue 2 to the DMA channel: - 000: DMA Channel 0 - 001: DMA Channel 1 - 010: DMA Channel 2 - 011: DMA Channel 3 - 100: DMA Channel 4 - 101: Reserved - 110: Reserved - 111: Reserved This field is valid when the Q2DDMACH field is reset. */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q2MDMACH_SHIFT 16
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q2MDMACH_MASK 0x70000
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q2MDMACH(v) (((v) << 16) & 0x70000)
/**< Field Q2DDMACH - Queue 2 Enabled for DA-based DMA Channel Selection When set, this bit indicates that the packets received in Queue 2 are routed to a particular DMA channel as decided in the MAC Receiver based on the DMA channel number programmed in the L3-L4 filter registers, or the Ethernet DA address. */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q2DDMACH_SHIFT 20
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q2DDMACH_MASK 0x100000
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q2DDMACH(v) (((v) << 20) & 0x100000)
/**< Field Q3MDMACH - Queue 3 Mapped to DMA Channel This field controls the routing of the received packet in Queue 3 to the DMA channel: - 000: DMA Channel 0 - 001: DMA Channel 1 - 010: DMA Channel 2 - 011: DMA Channel 3 - 100: DMA Channel 4 - 101: Reserved - 110: Reserved - 111: Reserved This field is valid when the Q3DDMACH field is reset. */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q3MDMACH_SHIFT 24
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q3MDMACH_MASK 0x7000000
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q3MDMACH(v) (((v) << 24) & 0x7000000)
/**< Field Q3DDMACH - Queue 3 Enabled for Dynamic (per packet) DMA Channel Selection When set, this bit indicates that the packets received in Queue 3 are routed to a particular DMA channel as decided in the MAC Receiver based on the DMA channel number programmed in the L3-L4 filter registers, or the Ethernet DA address. */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q3DDMACH_SHIFT 28
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q3DDMACH_MASK 0x10000000
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_Q3DDMACH(v) (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MTL_RXQ_DMA_MAP1 - Receive Queue and DMA Channel Mapping 1 */
typedef union IMX_ENET_QOS_MTL_RXQ_DMA_MAP1_union_t {
    UINT32 R;
    struct {
        UINT32 Q4MDMACH : 3;                 /**< Queue 4 Mapped to DMA Channel This field controls the routing of the packet received in Queue 4 to the DMA channel: - 000: DMA Channel 0 - 001: DMA Channel 1 - 010: DMA Channel 2 - 011: DMA Channel 3 - 100: DMA Channel 4 - 101: Reserved - 110: Reserved - 111: Reserved This field is valid when the Q4DDMACH field is reset. */
        UINT32 _unused3 : 1;                 /**< Reserved. */
        UINT32 Q4DDMACH : 1;                 /**< Queue 4 Enabled for DA-based DMA Channel Selection When set, this bit indicates that the packets received in Queue 4 are routed to a particular DMA channel as decided in the MAC Receiver based on the DMA channel number programmed in the L3-L4 filter registers, or the Ethernet DA address. */
        UINT32 _unused5 : 3;                 /**< Reserved. */
        UINT32 _unused8 : 3;                 /**< Reserved. */
        UINT32 _unused11 : 1;                 /**< Reserved. */
        UINT32 _unused12 : 1;                 /**< Reserved. */
        UINT32 _unused13 : 3;                 /**< Reserved. */
        UINT32 _unused16 : 3;                 /**< Reserved. */
        UINT32 _unused19 : 1;                 /**< Reserved. */
        UINT32 _unused20 : 1;                 /**< Reserved. */
        UINT32 _unused21 : 3;                 /**< Reserved. */
        UINT32 _unused24 : 3;                 /**< Reserved. */
        UINT32 _unused27 : 1;                 /**< Reserved. */
        UINT32 _unused28 : 1;                 /**< Reserved. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_RXQ_DMA_MAP1_t;

/**< Register MTL_RXQ_DMA_MAP1 - Receive Queue and DMA Channel Mapping 1 */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP1            0xC34
/**< Field Q4MDMACH - Queue 4 Mapped to DMA Channel This field controls the routing of the packet received in Queue 4 to the DMA channel: - 000: DMA Channel 0 - 001: DMA Channel 1 - 010: DMA Channel 2 - 011: DMA Channel 3 - 100: DMA Channel 4 - 101: Reserved - 110: Reserved - 111: Reserved This field is valid when the Q4DDMACH field is reset. */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP1_Q4MDMACH_SHIFT 0
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP1_Q4MDMACH_MASK 0x7
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP1_Q4MDMACH(v) (((v) << 0) & 0x7)
/**< Field Q4DDMACH - Queue 4 Enabled for DA-based DMA Channel Selection When set, this bit indicates that the packets received in Queue 4 are routed to a particular DMA channel as decided in the MAC Receiver based on the DMA channel number programmed in the L3-L4 filter registers, or the Ethernet DA address. */
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP1_Q4DDMACH_SHIFT 4
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP1_Q4DDMACH_MASK 0x10
#define IMX_ENET_QOS_MTL_RXQ_DMA_MAP1_Q4DDMACH(v) (((v) << 4) & 0x10)

/**< IMX_ENET_QOS_MTL_TBS_CTRL - Time Based Scheduling Control */
typedef union IMX_ENET_QOS_MTL_TBS_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 ESTM : 1;                 /**< EST offset Mode When this bit is set, the Launch Time value used in Time Based Scheduling is interpreted as an EST offset value and is added to the Base Time Register (BTR) of the current list. */
        UINT32 LEOV : 1;                 /**< Launch Expiry Offset Valid When set indicates the LEOS field is valid. */
        UINT32 _unused2 : 2;                 /**< Reserved. */
        UINT32 LEGOS : 3;                 /**< Launch Expiry GSN Offset The number GSN slots that has to be added to the Launch GSN to compute the Launch Expiry time. */
        UINT32 _unused7 : 1;                 /**< Reserved. */
        UINT32 LEOS : 24;                /**< Launch Expiry Offset The value in units of 256 nanoseconds that has to be added to the Launch time to compute the Launch Expiry time. */
    } B;
} IMX_ENET_QOS_MTL_TBS_CTRL_t;

/**< Register MTL_TBS_CTRL - Time Based Scheduling Control */
#define IMX_ENET_QOS_MTL_TBS_CTRL                0xC40
/**< Field ESTM - EST offset Mode When this bit is set, the Launch Time value used in Time Based Scheduling is interpreted as an EST offset value and is added to the Base Time Register (BTR) of the current list. */
#define IMX_ENET_QOS_MTL_TBS_CTRL_ESTM_SHIFT     0
#define IMX_ENET_QOS_MTL_TBS_CTRL_ESTM_MASK      0x1
#define IMX_ENET_QOS_MTL_TBS_CTRL_ESTM(v)        (((v) << 0) & 0x1)
/**< Field LEOV - Launch Expiry Offset Valid When set indicates the LEOS field is valid. */
#define IMX_ENET_QOS_MTL_TBS_CTRL_LEOV_SHIFT     1
#define IMX_ENET_QOS_MTL_TBS_CTRL_LEOV_MASK      0x2
#define IMX_ENET_QOS_MTL_TBS_CTRL_LEOV(v)        (((v) << 1) & 0x2)
/**< Field LEGOS - Launch Expiry GSN Offset The number GSN slots that has to be added to the Launch GSN to compute the Launch Expiry time. */
#define IMX_ENET_QOS_MTL_TBS_CTRL_LEGOS_SHIFT    4
#define IMX_ENET_QOS_MTL_TBS_CTRL_LEGOS_MASK     0x70
#define IMX_ENET_QOS_MTL_TBS_CTRL_LEGOS(v)       (((v) << 4) & 0x70)
/**< Field LEOS - Launch Expiry Offset The value in units of 256 nanoseconds that has to be added to the Launch time to compute the Launch Expiry time. */
#define IMX_ENET_QOS_MTL_TBS_CTRL_LEOS_SHIFT     8
#define IMX_ENET_QOS_MTL_TBS_CTRL_LEOS_MASK      0xFFFFFF00
#define IMX_ENET_QOS_MTL_TBS_CTRL_LEOS(v)        (((v) << 8) & 0xFFFFFF00)

/**< IMX_ENET_QOS_MTL_EST_CONTROL - Enhancements to Scheduled Transmission Control */
typedef union IMX_ENET_QOS_MTL_EST_CONTROL_union_t {
    UINT32 R;
    struct {
        UINT32 EEST : 1;                 /**< Enable EST When reset, the gate control list processing is halted and all gates are assumed to be in Open state. */
        UINT32 SSWL : 1;                 /**< Switch to S/W owned list When set indicates that the software has programmed that list that it currently owns (SWOL) and the hardware should switch to the new list based on the new BTR. */
        UINT32 _unused2 : 2;                 /**< Reserved. */
        UINT32 DDBF : 1;                 /**< Do not Drop frames during Frame Size Error When set, frames are not be dropped during Head-of-Line blocking due to Frame Size Error (HLBF field of MTL_EST_STATUS register). */
        UINT32 DFBS : 1;                 /**< Drop Frames causing Scheduling Error When set frames reported to cause HOL Blocking due to not getting scheduled (HLBS field of EST_STATUS register) after 4,8,16,32 (based on LCSE field of this register) GCL iterations are dropped. */
        UINT32 LCSE : 2;                 /**< Loop Count to report Scheduling Error Programmable number of GCL list iterations before reporting an HLBS error defined in EST_STATUS register. */
        UINT32 TILS : 3;                 /**< Time Interval Left Shift Amount This field provides the left shift amount for the programmed Time Interval values used in the Gate Control Lists. */
        UINT32 _unused11 : 1;                 /**< Reserved. */
        UINT32 CTOV : 12;                /**< Current Time Offset Value Provides a 12 bit time offset value in nano second that is added to the current time to compensate for all the implementation pipeline delays such as the CDC sync delay, buffering delays, data path delays etc. */
        UINT32 PTOV : 8;                 /**< PTP Time Offset Value The value of PTP Clock period multiplied by 6 in nanoseconds. */
    } B;
} IMX_ENET_QOS_MTL_EST_CONTROL_t;

/**< Register MTL_EST_CONTROL - Enhancements to Scheduled Transmission Control */
#define IMX_ENET_QOS_MTL_EST_CONTROL             0xC50
/**< Field EEST - Enable EST When reset, the gate control list processing is halted and all gates are assumed to be in Open state. */
#define IMX_ENET_QOS_MTL_EST_CONTROL_EEST_SHIFT  0
#define IMX_ENET_QOS_MTL_EST_CONTROL_EEST_MASK   0x1
#define IMX_ENET_QOS_MTL_EST_CONTROL_EEST(v)     (((v) << 0) & 0x1)
/**< Field SSWL - Switch to S/W owned list When set indicates that the software has programmed that list that it currently owns (SWOL) and the hardware should switch to the new list based on the new BTR. */
#define IMX_ENET_QOS_MTL_EST_CONTROL_SSWL_SHIFT  1
#define IMX_ENET_QOS_MTL_EST_CONTROL_SSWL_MASK   0x2
#define IMX_ENET_QOS_MTL_EST_CONTROL_SSWL(v)     (((v) << 1) & 0x2)
/**< Field DDBF - Do not Drop frames during Frame Size Error When set, frames are not be dropped during Head-of-Line blocking due to Frame Size Error (HLBF field of MTL_EST_STATUS register). */
#define IMX_ENET_QOS_MTL_EST_CONTROL_DDBF_SHIFT  4
#define IMX_ENET_QOS_MTL_EST_CONTROL_DDBF_MASK   0x10
#define IMX_ENET_QOS_MTL_EST_CONTROL_DDBF(v)     (((v) << 4) & 0x10)
/**< Field DFBS - Drop Frames causing Scheduling Error When set frames reported to cause HOL Blocking due to not getting scheduled (HLBS field of EST_STATUS register) after 4,8,16,32 (based on LCSE field of this register) GCL iterations are dropped. */
#define IMX_ENET_QOS_MTL_EST_CONTROL_DFBS_SHIFT  5
#define IMX_ENET_QOS_MTL_EST_CONTROL_DFBS_MASK   0x20
#define IMX_ENET_QOS_MTL_EST_CONTROL_DFBS(v)     (((v) << 5) & 0x20)
/**< Field LCSE - Loop Count to report Scheduling Error Programmable number of GCL list iterations before reporting an HLBS error defined in EST_STATUS register. */
#define IMX_ENET_QOS_MTL_EST_CONTROL_LCSE_SHIFT  6
#define IMX_ENET_QOS_MTL_EST_CONTROL_LCSE_MASK   0xC0
#define IMX_ENET_QOS_MTL_EST_CONTROL_LCSE(v)     (((v) << 6) & 0xC0)
/**< Field TILS - Time Interval Left Shift Amount This field provides the left shift amount for the programmed Time Interval values used in the Gate Control Lists. */
#define IMX_ENET_QOS_MTL_EST_CONTROL_TILS_SHIFT  8
#define IMX_ENET_QOS_MTL_EST_CONTROL_TILS_MASK   0x700
#define IMX_ENET_QOS_MTL_EST_CONTROL_TILS(v)     (((v) << 8) & 0x700)
/**< Field CTOV - Current Time Offset Value Provides a 12 bit time offset value in nano second that is added to the current time to compensate for all the implementation pipeline delays such as the CDC sync delay, buffering delays, data path delays etc. */
#define IMX_ENET_QOS_MTL_EST_CONTROL_CTOV_SHIFT  12
#define IMX_ENET_QOS_MTL_EST_CONTROL_CTOV_MASK   0xFFF000
#define IMX_ENET_QOS_MTL_EST_CONTROL_CTOV(v)     (((v) << 12) & 0xFFF000)
/**< Field PTOV - PTP Time Offset Value The value of PTP Clock period multiplied by 6 in nanoseconds. */
#define IMX_ENET_QOS_MTL_EST_CONTROL_PTOV_SHIFT  24
#define IMX_ENET_QOS_MTL_EST_CONTROL_PTOV_MASK   0xFF000000
#define IMX_ENET_QOS_MTL_EST_CONTROL_PTOV(v)     (((v) << 24) & 0xFF000000)

/**< IMX_ENET_QOS_MTL_EST_STATUS - Enhancements to Scheduled Transmission Status */
typedef union IMX_ENET_QOS_MTL_EST_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 SWLC : 1;                 /**< Switch to S/W owned list Complete When "1" indicates the hardware has successfully switched to the SWOL, and the SWOL bit has been updated to that effect. */
        UINT32 BTRE : 1;                 /**< BTR Error When "1" indicates a programming error in the BTR of SWOL where the programmed value is less than current time. */
        UINT32 HLBF : 1;                 /**< Head-Of-Line Blocking due to Frame Size Set when HOL Blocking is noticed on one or more Queues as a result of none of the Time Intervals of gate open in the GCL being greater than or equal to the duration needed for frame size (or frame fragment size when preemption is enabled) transmission. */
        UINT32 HLBS : 1;                 /**< Head-Of-Line Blocking due to Scheduling Set when the frame is not able to win arbitration and get scheduled even after 4 iterations of the GCL. */
        UINT32 CGCE : 1;                 /**< Constant Gate Control Error This error occurs when the list length (LLR) is 1 and the programmed Time Interval (TI) value after the optional Left Shifting is less than or equal to the Cycle Time (CTR). */
        UINT32 _unused5 : 2;                 /**< Reserved. */
        UINT32 SWOL : 1;                 /**< S/W owned list When '0' indicates Gate control list number "0" is owned by software and when "1" indicates the Gate Control list "1" is owned by the software. */
        UINT32 BTRL : 4;                 /**< BTR Error Loop Count Provides the minimum count (N) for which the equation Current Time =< New BTR + (N * New Cycle Time) becomes true. */
        UINT32 _unused12 : 4;                 /**< Reserved. */
        UINT32 CGSN : 4;                 /**< Current GCL Slot Number Indicates the slot number of the GCL list. */
        UINT32 _unused20 : 12;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_EST_STATUS_t;

/**< Register MTL_EST_STATUS - Enhancements to Scheduled Transmission Status */
#define IMX_ENET_QOS_MTL_EST_STATUS              0xC58
/**< Field SWLC - Switch to S/W owned list Complete When "1" indicates the hardware has successfully switched to the SWOL, and the SWOL bit has been updated to that effect. */
#define IMX_ENET_QOS_MTL_EST_STATUS_SWLC_SHIFT   0
#define IMX_ENET_QOS_MTL_EST_STATUS_SWLC_MASK    0x1
#define IMX_ENET_QOS_MTL_EST_STATUS_SWLC(v)      (((v) << 0) & 0x1)
/**< Field BTRE - BTR Error When "1" indicates a programming error in the BTR of SWOL where the programmed value is less than current time. */
#define IMX_ENET_QOS_MTL_EST_STATUS_BTRE_SHIFT   1
#define IMX_ENET_QOS_MTL_EST_STATUS_BTRE_MASK    0x2
#define IMX_ENET_QOS_MTL_EST_STATUS_BTRE(v)      (((v) << 1) & 0x2)
/**< Field HLBF - Head-Of-Line Blocking due to Frame Size Set when HOL Blocking is noticed on one or more Queues as a result of none of the Time Intervals of gate open in the GCL being greater than or equal to the duration needed for frame size (or frame fragment size when preemption is enabled) transmission. */
#define IMX_ENET_QOS_MTL_EST_STATUS_HLBF_SHIFT   2
#define IMX_ENET_QOS_MTL_EST_STATUS_HLBF_MASK    0x4
#define IMX_ENET_QOS_MTL_EST_STATUS_HLBF(v)      (((v) << 2) & 0x4)
/**< Field HLBS - Head-Of-Line Blocking due to Scheduling Set when the frame is not able to win arbitration and get scheduled even after 4 iterations of the GCL. */
#define IMX_ENET_QOS_MTL_EST_STATUS_HLBS_SHIFT   3
#define IMX_ENET_QOS_MTL_EST_STATUS_HLBS_MASK    0x8
#define IMX_ENET_QOS_MTL_EST_STATUS_HLBS(v)      (((v) << 3) & 0x8)
/**< Field CGCE - Constant Gate Control Error This error occurs when the list length (LLR) is 1 and the programmed Time Interval (TI) value after the optional Left Shifting is less than or equal to the Cycle Time (CTR). */
#define IMX_ENET_QOS_MTL_EST_STATUS_CGCE_SHIFT   4
#define IMX_ENET_QOS_MTL_EST_STATUS_CGCE_MASK    0x10
#define IMX_ENET_QOS_MTL_EST_STATUS_CGCE(v)      (((v) << 4) & 0x10)
/**< Field SWOL - S/W owned list When '0' indicates Gate control list number "0" is owned by software and when "1" indicates the Gate Control list "1" is owned by the software. */
#define IMX_ENET_QOS_MTL_EST_STATUS_SWOL_SHIFT   7
#define IMX_ENET_QOS_MTL_EST_STATUS_SWOL_MASK    0x80
#define IMX_ENET_QOS_MTL_EST_STATUS_SWOL(v)      (((v) << 7) & 0x80)
/**< Field BTRL - BTR Error Loop Count Provides the minimum count (N) for which the equation Current Time =< New BTR + (N * New Cycle Time) becomes true. */
#define IMX_ENET_QOS_MTL_EST_STATUS_BTRL_SHIFT   8
#define IMX_ENET_QOS_MTL_EST_STATUS_BTRL_MASK    0xF00
#define IMX_ENET_QOS_MTL_EST_STATUS_BTRL(v)      (((v) << 8) & 0xF00)
/**< Field CGSN - Current GCL Slot Number Indicates the slot number of the GCL list. */
#define IMX_ENET_QOS_MTL_EST_STATUS_CGSN_SHIFT   16
#define IMX_ENET_QOS_MTL_EST_STATUS_CGSN_MASK    0xF0000
#define IMX_ENET_QOS_MTL_EST_STATUS_CGSN(v)      (((v) << 16) & 0xF0000)

/**< IMX_ENET_QOS_MTL_EST_SCH_ERROR - EST Scheduling Error */
typedef union IMX_ENET_QOS_MTL_EST_SCH_ERROR_union_t {
    UINT32 R;
    struct {
        UINT32 SEQN : 5;                 /**< Schedule Error Queue Number The One Hot Encoded Queue Numbers that have experienced error/timeout described in HLBS field of status register. */
        UINT32 _unused5 : 27;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_EST_SCH_ERROR_t;

/**< Register MTL_EST_SCH_ERROR - EST Scheduling Error */
#define IMX_ENET_QOS_MTL_EST_SCH_ERROR           0xC60
/**< Field SEQN - Schedule Error Queue Number The One Hot Encoded Queue Numbers that have experienced error/timeout described in HLBS field of status register. */
#define IMX_ENET_QOS_MTL_EST_SCH_ERROR_SEQN_SHIFT 0
#define IMX_ENET_QOS_MTL_EST_SCH_ERROR_SEQN_MASK 0x1F
#define IMX_ENET_QOS_MTL_EST_SCH_ERROR_SEQN(v)   (((v) << 0) & 0x1F)

/**< IMX_ENET_QOS_MTL_EST_FRM_SIZE_ERROR - EST Frame Size Error */
typedef union IMX_ENET_QOS_MTL_EST_FRM_SIZE_ERROR_union_t {
    UINT32 R;
    struct {
        UINT32 FEQN : 5;                 /**< Frame Size Error Queue Number The One Hot Encoded Queue Numbers that have experienced error described in HLBF field of status register. */
        UINT32 _unused5 : 27;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_EST_FRM_SIZE_ERROR_t;

/**< Register MTL_EST_FRM_SIZE_ERROR - EST Frame Size Error */
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_ERROR      0xC64
/**< Field FEQN - Frame Size Error Queue Number The One Hot Encoded Queue Numbers that have experienced error described in HLBF field of status register. */
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_ERROR_FEQN_SHIFT 0
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_ERROR_FEQN_MASK 0x1F
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_ERROR_FEQN(v) (((v) << 0) & 0x1F)

/**< IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE - EST Frame Size Capture */
typedef union IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE_union_t {
    UINT32 R;
    struct {
        UINT32 HBFS : 15;                /**< Frame Size of HLBF Captures the Frame Size of the dropped frame related to queue number indicated in HBFQ field of this register. */
        UINT32 _unused15 : 1;                 /**< Reserved. */
        UINT32 HBFQ : 3;                 /**< Queue Number of HLBF Captures the binary value of the of the first Queue (number) experiencing HLBF error (see HLBF field of status register). */
        UINT32 _unused19 : 13;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE_t;

/**< Register MTL_EST_FRM_SIZE_CAPTURE - EST Frame Size Capture */
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE    0xC68
/**< Field HBFS - Frame Size of HLBF Captures the Frame Size of the dropped frame related to queue number indicated in HBFQ field of this register. */
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE_HBFS_SHIFT 0
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE_HBFS_MASK 0x7FFF
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE_HBFS(v) (((v) << 0) & 0x7FFF)
/**< Field HBFQ - Queue Number of HLBF Captures the binary value of the of the first Queue (number) experiencing HLBF error (see HLBF field of status register). */
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE_HBFQ_SHIFT 16
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE_HBFQ_MASK 0x70000
#define IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE_HBFQ(v) (((v) << 16) & 0x70000)

/**< IMX_ENET_QOS_MTL_EST_INTR_ENABLE - EST Interrupt Enable */
typedef union IMX_ENET_QOS_MTL_EST_INTR_ENABLE_union_t {
    UINT32 R;
    struct {
        UINT32 IECC : 1;                 /**< Interrupt Enable for Switch List When set, generates interrupt when the configuration change is successful and the hardware has switched to the new list. */
        UINT32 IEBE : 1;                 /**< Interrupt Enable for BTR Error When set, generates interrupt when the BTR Error occurs and is indicated in the status. */
        UINT32 IEHF : 1;                 /**< Interrupt Enable for HLBF When set, generates interrupt when the Head-of-Line Blocking due to Frame Size error occurs and is indicated in the status. */
        UINT32 IEHS : 1;                 /**< Interrupt Enable for HLBS When set, generates interrupt when the Head-of-Line Blocking due to Scheduling issue and is indicated in the status. */
        UINT32 CGCE : 1;                 /**< Interrupt Enable for CGCE When set, generates interrupt when the Constant Gate Control Error occurs and is indicated in the status. */
        UINT32 _unused5 : 27;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_EST_INTR_ENABLE_t;

/**< Register MTL_EST_INTR_ENABLE - EST Interrupt Enable */
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE         0xC70
/**< Field IECC - Interrupt Enable for Switch List When set, generates interrupt when the configuration change is successful and the hardware has switched to the new list. */
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IECC_SHIFT 0
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IECC_MASK 0x1
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IECC(v) (((v) << 0) & 0x1)
/**< Field IEBE - Interrupt Enable for BTR Error When set, generates interrupt when the BTR Error occurs and is indicated in the status. */
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IEBE_SHIFT 1
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IEBE_MASK 0x2
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IEBE(v) (((v) << 1) & 0x2)
/**< Field IEHF - Interrupt Enable for HLBF When set, generates interrupt when the Head-of-Line Blocking due to Frame Size error occurs and is indicated in the status. */
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IEHF_SHIFT 2
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IEHF_MASK 0x4
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IEHF(v) (((v) << 2) & 0x4)
/**< Field IEHS - Interrupt Enable for HLBS When set, generates interrupt when the Head-of-Line Blocking due to Scheduling issue and is indicated in the status. */
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IEHS_SHIFT 3
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IEHS_MASK 0x8
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_IEHS(v) (((v) << 3) & 0x8)
/**< Field CGCE - Interrupt Enable for CGCE When set, generates interrupt when the Constant Gate Control Error occurs and is indicated in the status. */
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_CGCE_SHIFT 4
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_CGCE_MASK 0x10
#define IMX_ENET_QOS_MTL_EST_INTR_ENABLE_CGCE(v) (((v) << 4) & 0x10)

/**< IMX_ENET_QOS_MTL_EST_GCL_CONTROL - EST GCL Control */
typedef union IMX_ENET_QOS_MTL_EST_GCL_CONTROL_union_t {
    UINT32 R;
    struct {
        UINT32 SRWO : 1;                 /**< Start Read/Write Op When set indicates a Read/Write Op has started and is in progress. */
        UINT32 R1W0 : 1;                 /**< Read '1', Write '0': When set to '1': Read Operation When set to '0': Write Operation. */
        UINT32 GCRR : 1;                 /**< Gate Control Related Registers When set to "1" indicates the R/W access is for the GCL related registers (BTR, CTR, TER, LLR) whose address is provided by GCRA. */
        UINT32 _unused3 : 1;                 /**< Reserved. */
        UINT32 DBGM : 1;                 /**< Debug Mode When set to "1" indicates R/W in debug mode where the memory bank (for GCL and Time related registers) is explicitly provided by DBGB value, when set to "0" SWOL bit is used to determine which bank to use. */
        UINT32 DBGB : 1;                 /**< Debug Mode Bank Select When set to "0" indicates R/W in debug mode should be directed to Bank 0 (GCL0 and corresponding Time related registers). */
        UINT32 _unused6 : 2;                 /**< Reserved. */
        UINT32 ADDR : 9;                 /**< Gate Control List Address: (GCLA when GCRR is "0"). */
        UINT32 _unused17 : 3;                 /**< Reserved. */
        UINT32 ERR0 : 1;                 /**< When set indicates the last write operation was aborted as software writes to GCL and GCL registers is prohibited when SSWL bit of MTL_EST_CONTROL Register is set. */
        UINT32 ESTEIEE : 1;                 /**< EST ECC Inject Error Enable When set along with EEST bit of MTL_EST_CONTROL register, enables the ECC error injection feature. */
        UINT32 ESTEIEC : 2;                 /**< ECC Inject Error Control for EST Memory When EIEE bit of this register is set, following are the errors inserted based on the value encoded in this field. */
        UINT32 _unused24 : 8;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_EST_GCL_CONTROL_t;

/**< Register MTL_EST_GCL_CONTROL - EST GCL Control */
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL         0xC80
/**< Field SRWO - Start Read/Write Op When set indicates a Read/Write Op has started and is in progress. */
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_SRWO_SHIFT 0
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_SRWO_MASK 0x1
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_SRWO(v) (((v) << 0) & 0x1)
/**< Field R1W0 - Read '1', Write '0': When set to '1': Read Operation When set to '0': Write Operation. */
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_R1W0_SHIFT 1
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_R1W0_MASK 0x2
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_R1W0(v) (((v) << 1) & 0x2)
/**< Field GCRR - Gate Control Related Registers When set to "1" indicates the R/W access is for the GCL related registers (BTR, CTR, TER, LLR) whose address is provided by GCRA. */
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_GCRR_SHIFT 2
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_GCRR_MASK 0x4
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_GCRR(v) (((v) << 2) & 0x4)
/**< Field DBGM - Debug Mode When set to "1" indicates R/W in debug mode where the memory bank (for GCL and Time related registers) is explicitly provided by DBGB value, when set to "0" SWOL bit is used to determine which bank to use. */
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_DBGM_SHIFT 4
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_DBGM_MASK 0x10
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_DBGM(v) (((v) << 4) & 0x10)
/**< Field DBGB - Debug Mode Bank Select When set to "0" indicates R/W in debug mode should be directed to Bank 0 (GCL0 and corresponding Time related registers). */
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_DBGB_SHIFT 5
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_DBGB_MASK 0x20
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_DBGB(v) (((v) << 5) & 0x20)
/**< Field ADDR - Gate Control List Address: (GCLA when GCRR is "0"). */
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ADDR_SHIFT 8
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ADDR_MASK 0x1FF00
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ADDR(v) (((v) << 8) & 0x1FF00)
/**< Field ERR0 - When set indicates the last write operation was aborted as software writes to GCL and GCL registers is prohibited when SSWL bit of MTL_EST_CONTROL Register is set. */
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ERR0_SHIFT 20
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ERR0_MASK 0x100000
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ERR0(v) (((v) << 20) & 0x100000)
/**< Field ESTEIEE - EST ECC Inject Error Enable When set along with EEST bit of MTL_EST_CONTROL register, enables the ECC error injection feature. */
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ESTEIEE_SHIFT 21
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ESTEIEE_MASK 0x200000
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ESTEIEE(v) (((v) << 21) & 0x200000)
/**< Field ESTEIEC - ECC Inject Error Control for EST Memory When EIEE bit of this register is set, following are the errors inserted based on the value encoded in this field. */
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ESTEIEC_SHIFT 22
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ESTEIEC_MASK 0xC00000
#define IMX_ENET_QOS_MTL_EST_GCL_CONTROL_ESTEIEC(v) (((v) << 22) & 0xC00000)

/**< IMX_ENET_QOS_MTL_EST_GCL_DATA - EST GCL Data */
typedef union IMX_ENET_QOS_MTL_EST_GCL_DATA_union_t {
    UINT32 R;
    struct {
        UINT32 GCD : 32;                /**< Gate Control Data The data corresponding to the address selected in the MTL_GCL_CONTROL register. */
    } B;
} IMX_ENET_QOS_MTL_EST_GCL_DATA_t;

/**< Register MTL_EST_GCL_DATA - EST GCL Data */
#define IMX_ENET_QOS_MTL_EST_GCL_DATA            0xC84
/**< Field GCD - Gate Control Data The data corresponding to the address selected in the MTL_GCL_CONTROL register. */
#define IMX_ENET_QOS_MTL_EST_GCL_DATA_GCD_SHIFT  0
#define IMX_ENET_QOS_MTL_EST_GCL_DATA_GCD_MASK   0xFFFFFFFF
#define IMX_ENET_QOS_MTL_EST_GCL_DATA_GCD(v)     (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MTL_FPE_CTRL_STS - Frame Preemption Control and Status */
typedef union IMX_ENET_QOS_MTL_FPE_CTRL_STS_union_t {
    UINT32 R;
    struct {
        UINT32 AFSZ : 2;                 /**< Additional Fragment Size used to indicate, in units of 64 bytes, the minimum number of bytes over 64 bytes required in non-final fragments of preempted frames. */
        UINT32 _unused2 : 6;                 /**< Reserved. */
        UINT32 PEC : 5;                 /**< Preemption Classification When set indicates the corresponding Queue must be classified as preemptable, when '0' Queue is classified as express. */
        UINT32 _unused13 : 3;                 /**< Reserved. */
        UINT32 _unused16 : 12;                /**< Reserved. */
        UINT32 HRS : 1;                 /**< Hold/Release Status - 1: Indicates a Set-and-Hold-MAC operation was last executed and the pMAC is in Hold State. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_FPE_CTRL_STS_t;

/**< Register MTL_FPE_CTRL_STS - Frame Preemption Control and Status */
#define IMX_ENET_QOS_MTL_FPE_CTRL_STS            0xC90
/**< Field AFSZ - Additional Fragment Size used to indicate, in units of 64 bytes, the minimum number of bytes over 64 bytes required in non-final fragments of preempted frames. */
#define IMX_ENET_QOS_MTL_FPE_CTRL_STS_AFSZ_SHIFT 0
#define IMX_ENET_QOS_MTL_FPE_CTRL_STS_AFSZ_MASK  0x3
#define IMX_ENET_QOS_MTL_FPE_CTRL_STS_AFSZ(v)    (((v) << 0) & 0x3)
/**< Field PEC - Preemption Classification When set indicates the corresponding Queue must be classified as preemptable, when '0' Queue is classified as express. */
#define IMX_ENET_QOS_MTL_FPE_CTRL_STS_PEC_SHIFT  8
#define IMX_ENET_QOS_MTL_FPE_CTRL_STS_PEC_MASK   0x1F00
#define IMX_ENET_QOS_MTL_FPE_CTRL_STS_PEC(v)     (((v) << 8) & 0x1F00)
/**< Field HRS - Hold/Release Status - 1: Indicates a Set-and-Hold-MAC operation was last executed and the pMAC is in Hold State. */
#define IMX_ENET_QOS_MTL_FPE_CTRL_STS_HRS_SHIFT  28
#define IMX_ENET_QOS_MTL_FPE_CTRL_STS_HRS_MASK   0x10000000
#define IMX_ENET_QOS_MTL_FPE_CTRL_STS_HRS(v)     (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_MTL_FPE_ADVANCE - Frame Preemption Hold and Release Advance */
typedef union IMX_ENET_QOS_MTL_FPE_ADVANCE_union_t {
    UINT32 R;
    struct {
        UINT32 HADV : 16;                /**< Hold Advance The maximum time in nanoseconds that can elapse between issuing a HOLD to the MAC and the MAC ceasing to transmit any preemptable frame that is in the process of transmission or any preemptable frames that are queued for transmission. */
        UINT32 RADV : 16;                /**< Release Advance The maximum time in nanoseconds that can elapse between issuing a RELEASE to the MAC and the MAC being ready to resume transmission of preemptable frames, in the absence of there being any express frames available for transmission. */
    } B;
} IMX_ENET_QOS_MTL_FPE_ADVANCE_t;

/**< Register MTL_FPE_ADVANCE - Frame Preemption Hold and Release Advance */
#define IMX_ENET_QOS_MTL_FPE_ADVANCE             0xC94
/**< Field HADV - Hold Advance The maximum time in nanoseconds that can elapse between issuing a HOLD to the MAC and the MAC ceasing to transmit any preemptable frame that is in the process of transmission or any preemptable frames that are queued for transmission. */
#define IMX_ENET_QOS_MTL_FPE_ADVANCE_HADV_SHIFT  0
#define IMX_ENET_QOS_MTL_FPE_ADVANCE_HADV_MASK   0xFFFF
#define IMX_ENET_QOS_MTL_FPE_ADVANCE_HADV(v)     (((v) << 0) & 0xFFFF)
/**< Field RADV - Release Advance The maximum time in nanoseconds that can elapse between issuing a RELEASE to the MAC and the MAC being ready to resume transmission of preemptable frames, in the absence of there being any express frames available for transmission. */
#define IMX_ENET_QOS_MTL_FPE_ADVANCE_RADV_SHIFT  16
#define IMX_ENET_QOS_MTL_FPE_ADVANCE_RADV_MASK   0xFFFF0000
#define IMX_ENET_QOS_MTL_FPE_ADVANCE_RADV(v)     (((v) << 16) & 0xFFFF0000)

/**< IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS - RXP Control Status */
typedef union IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 NVE : 8;                 /**< Number of valid entries in the Instruction table This control indicates the number of valid entries in the Instruction Memory. */
        UINT32 _unused8 : 8;                 /**< Reserved. */
        UINT32 NPE : 8;                 /**< Number of parsable entries in the Instruction table This control indicates the number of parsable entries in the Instruction Memory. */
        UINT32 _unused24 : 7;                 /**< Reserved. */
        UINT32 RXPI : 1;                 /**< RX Parser in Idle state This status bit is set to 1 when the Rx parser is in Idle State and waiting for a new packet for processing. */
    } B;
} IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_t;

/**< Register MTL_RXP_CONTROL_STATUS - RXP Control Status */
#define IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS      0xCA0
/**< Field NVE - Number of valid entries in the Instruction table This control indicates the number of valid entries in the Instruction Memory. */
#define IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_NVE_SHIFT 0
#define IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_NVE_MASK 0xFF
#define IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_NVE(v) (((v) << 0) & 0xFF)
/**< Field NPE - Number of parsable entries in the Instruction table This control indicates the number of parsable entries in the Instruction Memory. */
#define IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_NPE_SHIFT 16
#define IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_NPE_MASK 0xFF0000
#define IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_NPE(v) (((v) << 16) & 0xFF0000)
/**< Field RXPI - RX Parser in Idle state This status bit is set to 1 when the Rx parser is in Idle State and waiting for a new packet for processing. */
#define IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_RXPI_SHIFT 31
#define IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_RXPI_MASK 0x80000000
#define IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_RXPI(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS - RXP Interrupt Control Status */
typedef union IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 NVEOVIS : 1;                 /**< Number of Valid Entries Overflow Interrupt Status While parsing if the Instruction address found to be more than NVE (Number of Valid Entries in MTL_RXP_CONTROL register), then this bit is set to 1. */
        UINT32 NPEOVIS : 1;                 /**< Number of Parsable Entries Overflow Interrupt Status While parsing a packet if the number of parsed entries found to be more than NPE[] (Number of Parseable Entries in MTL_RXP_CONTROL register),then this bit is set to 1. */
        UINT32 FOOVIS : 1;                 /**< Frame Offset Overflow Interrupt Status While parsing if the Instruction table entry's 'Frame Offset' found to be more than EOF offset, then then this bit is set. */
        UINT32 PDRFIS : 1;                 /**< Packet Dropped due to RF Interrupt Status If the Rx Parser result says to drop the packet by setting RF=1 in the instruction memory, then this bit is set to 1. */
        UINT32 _unused4 : 12;                /**< Reserved. */
        UINT32 NVEOVIE : 1;                 /**< Number of Valid Entries Overflow Interrupt Enable When this bit is set, the NVEOVIS interrupt is enabled. */
        UINT32 NPEOVIE : 1;                 /**< Number of Parsable Entries Overflow Interrupt Enable When this bit is set, the NPEOVIS interrupt is enabled. */
        UINT32 FOOVIE : 1;                 /**< Frame Offset Overflow Interrupt Enable When this bit is set, the FOOVIS interrupt is enabled. */
        UINT32 PDRFIE : 1;                 /**< Packet Drop due to RF Interrupt Enable When this bit is set, the PDRFIS interrupt is enabled. */
        UINT32 _unused20 : 12;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_t;

/**< Register MTL_RXP_INTERRUPT_CONTROL_STATUS - RXP Interrupt Control Status */
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS 0xCA4
/**< Field NVEOVIS - Number of Valid Entries Overflow Interrupt Status While parsing if the Instruction address found to be more than NVE (Number of Valid Entries in MTL_RXP_CONTROL register), then this bit is set to 1. */
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NVEOVIS_SHIFT 0
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NVEOVIS_MASK 0x1
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NVEOVIS(v) (((v) << 0) & 0x1)
/**< Field NPEOVIS - Number of Parsable Entries Overflow Interrupt Status While parsing a packet if the number of parsed entries found to be more than NPE[] (Number of Parseable Entries in MTL_RXP_CONTROL register),then this bit is set to 1. */
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NPEOVIS_SHIFT 1
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NPEOVIS_MASK 0x2
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NPEOVIS(v) (((v) << 1) & 0x2)
/**< Field FOOVIS - Frame Offset Overflow Interrupt Status While parsing if the Instruction table entry's 'Frame Offset' found to be more than EOF offset, then then this bit is set. */
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_FOOVIS_SHIFT 2
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_FOOVIS_MASK 0x4
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_FOOVIS(v) (((v) << 2) & 0x4)
/**< Field PDRFIS - Packet Dropped due to RF Interrupt Status If the Rx Parser result says to drop the packet by setting RF=1 in the instruction memory, then this bit is set to 1. */
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_PDRFIS_SHIFT 3
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_PDRFIS_MASK 0x8
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_PDRFIS(v) (((v) << 3) & 0x8)
/**< Field NVEOVIE - Number of Valid Entries Overflow Interrupt Enable When this bit is set, the NVEOVIS interrupt is enabled. */
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NVEOVIE_SHIFT 16
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NVEOVIE_MASK 0x10000
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NVEOVIE(v) (((v) << 16) & 0x10000)
/**< Field NPEOVIE - Number of Parsable Entries Overflow Interrupt Enable When this bit is set, the NPEOVIS interrupt is enabled. */
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NPEOVIE_SHIFT 17
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NPEOVIE_MASK 0x20000
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_NPEOVIE(v) (((v) << 17) & 0x20000)
/**< Field FOOVIE - Frame Offset Overflow Interrupt Enable When this bit is set, the FOOVIS interrupt is enabled. */
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_FOOVIE_SHIFT 18
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_FOOVIE_MASK 0x40000
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_FOOVIE(v) (((v) << 18) & 0x40000)
/**< Field PDRFIE - Packet Drop due to RF Interrupt Enable When this bit is set, the PDRFIS interrupt is enabled. */
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_PDRFIE_SHIFT 19
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_PDRFIE_MASK 0x80000
#define IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_PDRFIE(v) (((v) << 19) & 0x80000)

/**< IMX_ENET_QOS_MTL_RXP_DROP_CNT - RXP Drop Count */
typedef union IMX_ENET_QOS_MTL_RXP_DROP_CNT_union_t {
    UINT32 R;
    struct {
        UINT32 RXPDC : 31;                /**< Rx Parser Drop count This 31-bit counter is implemented whenever a Rx Parser Drops a packet due to RF =1. */
        UINT32 RXPDCOVF : 1;                 /**< Rx Parser Drop Counter Overflow Bit When set, this bit indicates that the MTL_RXP_DROP_CNT (RXPDC) Counter field crossed the maximum limit. */
    } B;
} IMX_ENET_QOS_MTL_RXP_DROP_CNT_t;

/**< Register MTL_RXP_DROP_CNT - RXP Drop Count */
#define IMX_ENET_QOS_MTL_RXP_DROP_CNT            0xCA8
/**< Field RXPDC - Rx Parser Drop count This 31-bit counter is implemented whenever a Rx Parser Drops a packet due to RF =1. */
#define IMX_ENET_QOS_MTL_RXP_DROP_CNT_RXPDC_SHIFT 0
#define IMX_ENET_QOS_MTL_RXP_DROP_CNT_RXPDC_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_MTL_RXP_DROP_CNT_RXPDC(v)   (((v) << 0) & 0x7FFFFFFF)
/**< Field RXPDCOVF - Rx Parser Drop Counter Overflow Bit When set, this bit indicates that the MTL_RXP_DROP_CNT (RXPDC) Counter field crossed the maximum limit. */
#define IMX_ENET_QOS_MTL_RXP_DROP_CNT_RXPDCOVF_SHIFT 31
#define IMX_ENET_QOS_MTL_RXP_DROP_CNT_RXPDCOVF_MASK 0x80000000
#define IMX_ENET_QOS_MTL_RXP_DROP_CNT_RXPDCOVF(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MTL_RXP_ERROR_CNT - RXP Error Count */
typedef union IMX_ENET_QOS_MTL_RXP_ERROR_CNT_union_t {
    UINT32 R;
    struct {
        UINT32 RXPEC : 31;                /**< Rx Parser Error count This 31-bit counter is implemented whenever a Rx Parser encounters following Error scenarios - Entry address >= NVE[] - Number Parsed Entries >= NPE[] - Entry address > EOF data entry address The counter is cleared when the register is read. */
        UINT32 RXPECOVF : 1;                 /**< Rx Parser Error Counter Overflow Bit When set, this bit indicates that the MTL_RXP_ERROR_CNT (RXPEC) Counter field crossed the maximum limit. */
    } B;
} IMX_ENET_QOS_MTL_RXP_ERROR_CNT_t;

/**< Register MTL_RXP_ERROR_CNT - RXP Error Count */
#define IMX_ENET_QOS_MTL_RXP_ERROR_CNT           0xCAC
/**< Field RXPEC - Rx Parser Error count This 31-bit counter is implemented whenever a Rx Parser encounters following Error scenarios - Entry address >= NVE[] - Number Parsed Entries >= NPE[] - Entry address > EOF data entry address The counter is cleared when the register is read. */
#define IMX_ENET_QOS_MTL_RXP_ERROR_CNT_RXPEC_SHIFT 0
#define IMX_ENET_QOS_MTL_RXP_ERROR_CNT_RXPEC_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_MTL_RXP_ERROR_CNT_RXPEC(v)  (((v) << 0) & 0x7FFFFFFF)
/**< Field RXPECOVF - Rx Parser Error Counter Overflow Bit When set, this bit indicates that the MTL_RXP_ERROR_CNT (RXPEC) Counter field crossed the maximum limit. */
#define IMX_ENET_QOS_MTL_RXP_ERROR_CNT_RXPECOVF_SHIFT 31
#define IMX_ENET_QOS_MTL_RXP_ERROR_CNT_RXPECOVF_MASK 0x80000000
#define IMX_ENET_QOS_MTL_RXP_ERROR_CNT_RXPECOVF(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS - RXP Indirect Access Control and Status */
typedef union IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 ADDR : 10;                /**< FRP Instruction Table Offset Address This field indicates the ADDR of the 32-bit entry in Rx parser instruction table. */
        UINT32 _unused10 : 6;                 /**< Reserved. */
        UINT32 WRRDN : 1;                 /**< Read Write Control When this bit is set to 1 indicates the write operation to the Rx Parser Memory. */
        UINT32 _unused17 : 3;                 /**< Reserved. */
        UINT32 _unused20 : 1;                 /**< Reserved. */
        UINT32 _unused21 : 2;                 /**< Reserved. */
        UINT32 _unused23 : 8;                 /**< Reserved. */
        UINT32 STARTBUSY : 1;                 /**< FRP Instruction Table Access Busy When this bit is set to 1 by the software then it indicates to start the Read/Write operation from/to the Rx Parser Memory. */
    } B;
} IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_t;

/**< Register MTL_RXP_INDIRECT_ACC_CONTROL_STATUS - RXP Indirect Access Control and Status */
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS 0xCB0
/**< Field ADDR - FRP Instruction Table Offset Address This field indicates the ADDR of the 32-bit entry in Rx parser instruction table. */
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_ADDR_SHIFT 0
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_ADDR_MASK 0x3FF
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_ADDR(v) (((v) << 0) & 0x3FF)
/**< Field WRRDN - Read Write Control When this bit is set to 1 indicates the write operation to the Rx Parser Memory. */
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_WRRDN_SHIFT 16
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_WRRDN_MASK 0x10000
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_WRRDN(v) (((v) << 16) & 0x10000)
/**< Field STARTBUSY - FRP Instruction Table Access Busy When this bit is set to 1 by the software then it indicates to start the Read/Write operation from/to the Rx Parser Memory. */
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_STARTBUSY_SHIFT 31
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_STARTBUSY_MASK 0x80000000
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_STARTBUSY(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_DATA - RXP Indirect Access Data */
typedef union IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_DATA_union_t {
    UINT32 R;
    struct {
        UINT32 DATA : 32;                /**< FRP Instruction Table Write/Read Data Software should write this register before issuing any write command. */
    } B;
} IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_DATA_t;

/**< Register MTL_RXP_INDIRECT_ACC_DATA - RXP Indirect Access Data */
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_DATA   0xCB4
/**< Field DATA - FRP Instruction Table Write/Read Data Software should write this register before issuing any write command. */
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_DATA_DATA_SHIFT 0
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_DATA_DATA_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_DATA_DATA(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_MTL_TXQX_OP_MODE - Queue 1 Transmit Operation Mode */
typedef union IMX_ENET_QOS_MTL_TXQX_OP_MODE_union_t {
    UINT32 R;
    struct {
        UINT32 FTQ : 1;                 /**< Flush Transmit Queue When this bit is set, the Tx queue controller logic is reset to its default values. */
        UINT32 TSF : 1;                 /**< Transmit Store and Forward When this bit is set, the transmission starts when a full packet resides in the MTL Tx queue. */
        UINT32 TXQEN : 2;                 /**< Transmit Queue Enable This field is used to enable/disable the transmit queue 0. */
        UINT32 TTC : 3;                 /**< Transmit Threshold Control These bits control the threshold level of the MTL Tx Queue. */
        UINT32 _unused7 : 9;                 /**< Reserved. */
        UINT32 TQS : 5;                 /**< Transmit Queue Size This field indicates the size of the allocated Transmit queues in blocks of 256 bytes. */
        UINT32 _unused21 : 11;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_OP_MODE_t;

/**< Register MTL_TXQX_OP_MODE - Queue 1 Transmit Operation Mode */
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE            0xD40
/**< Field FTQ - Flush Transmit Queue When this bit is set, the Tx queue controller logic is reset to its default values. */
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_FTQ_SHIFT  0
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_FTQ_MASK   0x1
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_FTQ(v)     (((v) << 0) & 0x1)
/**< Field TSF - Transmit Store and Forward When this bit is set, the transmission starts when a full packet resides in the MTL Tx queue. */
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TSF_SHIFT  1
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TSF_MASK   0x2
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TSF(v)     (((v) << 1) & 0x2)
/**< Field TXQEN - Transmit Queue Enable This field is used to enable/disable the transmit queue 0. */
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TXQEN_SHIFT 2
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TXQEN_MASK 0xC
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TXQEN(v)   (((v) << 2) & 0xC)
/**< Field TTC - Transmit Threshold Control These bits control the threshold level of the MTL Tx Queue. */
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TTC_SHIFT  4
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TTC_MASK   0x70
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TTC(v)     (((v) << 4) & 0x70)
/**< Field TQS - Transmit Queue Size This field indicates the size of the allocated Transmit queues in blocks of 256 bytes. */
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TQS_SHIFT  16
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TQS_MASK   0x1F0000
#define IMX_ENET_QOS_MTL_TXQX_OP_MODE_TQS(v)     (((v) << 16) & 0x1F0000)

/**< IMX_ENET_QOS_MTL_TXQX_UNDRFLW - Queue 1 Underflow Counter */
typedef union IMX_ENET_QOS_MTL_TXQX_UNDRFLW_union_t {
    UINT32 R;
    struct {
        UINT32 UFFRMCNT : 11;                /**< Underflow Packet Counter This field indicates the number of packets aborted by the controller because of Tx Queue Underflow. */
        UINT32 UFCNTOVF : 1;                 /**< Overflow Bit for Underflow Packet Counter This bit is set every time the Tx queue Underflow Packet Counter field overflows, that is, it has crossed the maximum count. */
        UINT32 _unused12 : 20;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_UNDRFLW_t;

/**< Register MTL_TXQX_UNDRFLW - Queue 1 Underflow Counter */
#define IMX_ENET_QOS_MTL_TXQX_UNDRFLW            0xD44
/**< Field UFFRMCNT - Underflow Packet Counter This field indicates the number of packets aborted by the controller because of Tx Queue Underflow. */
#define IMX_ENET_QOS_MTL_TXQX_UNDRFLW_UFFRMCNT_SHIFT 0
#define IMX_ENET_QOS_MTL_TXQX_UNDRFLW_UFFRMCNT_MASK 0x7FF
#define IMX_ENET_QOS_MTL_TXQX_UNDRFLW_UFFRMCNT(v) (((v) << 0) & 0x7FF)
/**< Field UFCNTOVF - Overflow Bit for Underflow Packet Counter This bit is set every time the Tx queue Underflow Packet Counter field overflows, that is, it has crossed the maximum count. */
#define IMX_ENET_QOS_MTL_TXQX_UNDRFLW_UFCNTOVF_SHIFT 11
#define IMX_ENET_QOS_MTL_TXQX_UNDRFLW_UFCNTOVF_MASK 0x800
#define IMX_ENET_QOS_MTL_TXQX_UNDRFLW_UFCNTOVF(v) (((v) << 11) & 0x800)

/**< IMX_ENET_QOS_MTL_TXQX_DBG - Queue 1 Transmit Debug */
typedef union IMX_ENET_QOS_MTL_TXQX_DBG_union_t {
    UINT32 R;
    struct {
        UINT32 TXQPAUSED : 1;                 /**< Transmit Queue in Pause When this bit is high and the Rx flow control is enabled, it indicates that the Tx Queue is in the Pause condition (in the full-duplex only mode) because of the following: - Reception of the PFC packet for the priorities assigned to the Tx Queue when PFC is enabled - Reception of 802. */
        UINT32 TRCSTS : 2;                 /**< MTL Tx Queue Read Controller Status This field indicates the state of the Tx Queue Read Controller: */
        UINT32 TWCSTS : 1;                 /**< MTL Tx Queue Write Controller Status When high, this bit indicates that the MTL Tx Queue Write Controller is active, and it is transferring the data to the Tx Queue. */
        UINT32 TXQSTS : 1;                 /**< MTL Tx Queue Not Empty Status When this bit is high, it indicates that the MTL Tx Queue is not empty and some data is left for transmission. */
        UINT32 TXSTSFSTS : 1;                 /**< MTL Tx Status FIFO Full Status When high, this bit indicates that the MTL Tx Status FIFO is full. */
        UINT32 _unused6 : 10;                /**< Reserved. */
        UINT32 PTXQ : 3;                 /**< Number of Packets in the Transmit Queue This field indicates the current number of packets in the Tx Queue. */
        UINT32 _unused19 : 1;                 /**< Reserved. */
        UINT32 STXSTSF : 3;                 /**< Number of Status Words in Tx Status FIFO of Queue This field indicates the current number of status in the Tx Status FIFO of this queue. */
        UINT32 _unused23 : 9;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_DBG_t;

/**< Register MTL_TXQX_DBG - Queue 1 Transmit Debug */
#define IMX_ENET_QOS_MTL_TXQX_DBG                0xD48
/**< Field TXQPAUSED - Transmit Queue in Pause When this bit is high and the Rx flow control is enabled, it indicates that the Tx Queue is in the Pause condition (in the full-duplex only mode) because of the following: - Reception of the PFC packet for the priorities assigned to the Tx Queue when PFC is enabled - Reception of 802. */
#define IMX_ENET_QOS_MTL_TXQX_DBG_TXQPAUSED_SHIFT 0
#define IMX_ENET_QOS_MTL_TXQX_DBG_TXQPAUSED_MASK 0x1
#define IMX_ENET_QOS_MTL_TXQX_DBG_TXQPAUSED(v)   (((v) << 0) & 0x1)
/**< Field TRCSTS - MTL Tx Queue Read Controller Status This field indicates the state of the Tx Queue Read Controller: */
#define IMX_ENET_QOS_MTL_TXQX_DBG_TRCSTS_SHIFT   1
#define IMX_ENET_QOS_MTL_TXQX_DBG_TRCSTS_MASK    0x6
#define IMX_ENET_QOS_MTL_TXQX_DBG_TRCSTS(v)      (((v) << 1) & 0x6)
/**< Field TWCSTS - MTL Tx Queue Write Controller Status When high, this bit indicates that the MTL Tx Queue Write Controller is active, and it is transferring the data to the Tx Queue. */
#define IMX_ENET_QOS_MTL_TXQX_DBG_TWCSTS_SHIFT   3
#define IMX_ENET_QOS_MTL_TXQX_DBG_TWCSTS_MASK    0x8
#define IMX_ENET_QOS_MTL_TXQX_DBG_TWCSTS(v)      (((v) << 3) & 0x8)
/**< Field TXQSTS - MTL Tx Queue Not Empty Status When this bit is high, it indicates that the MTL Tx Queue is not empty and some data is left for transmission. */
#define IMX_ENET_QOS_MTL_TXQX_DBG_TXQSTS_SHIFT   4
#define IMX_ENET_QOS_MTL_TXQX_DBG_TXQSTS_MASK    0x10
#define IMX_ENET_QOS_MTL_TXQX_DBG_TXQSTS(v)      (((v) << 4) & 0x10)
/**< Field TXSTSFSTS - MTL Tx Status FIFO Full Status When high, this bit indicates that the MTL Tx Status FIFO is full. */
#define IMX_ENET_QOS_MTL_TXQX_DBG_TXSTSFSTS_SHIFT 5
#define IMX_ENET_QOS_MTL_TXQX_DBG_TXSTSFSTS_MASK 0x20
#define IMX_ENET_QOS_MTL_TXQX_DBG_TXSTSFSTS(v)   (((v) << 5) & 0x20)
/**< Field PTXQ - Number of Packets in the Transmit Queue This field indicates the current number of packets in the Tx Queue. */
#define IMX_ENET_QOS_MTL_TXQX_DBG_PTXQ_SHIFT     16
#define IMX_ENET_QOS_MTL_TXQX_DBG_PTXQ_MASK      0x70000
#define IMX_ENET_QOS_MTL_TXQX_DBG_PTXQ(v)        (((v) << 16) & 0x70000)
/**< Field STXSTSF - Number of Status Words in Tx Status FIFO of Queue This field indicates the current number of status in the Tx Status FIFO of this queue. */
#define IMX_ENET_QOS_MTL_TXQX_DBG_STXSTSF_SHIFT  20
#define IMX_ENET_QOS_MTL_TXQX_DBG_STXSTSF_MASK   0x700000
#define IMX_ENET_QOS_MTL_TXQX_DBG_STXSTSF(v)     (((v) << 20) & 0x700000)

/**< IMX_ENET_QOS_MTL_TXQX_ETS_CTRL - Queue 1 ETS Control */
typedef union IMX_ENET_QOS_MTL_TXQX_ETS_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 2;                 /**< Reserved. */
        UINT32 AVALG : 1;                 /**< AV Algorithm When Queue 1 is programmed for AV, this field configures the scheduling algorithm for this queue: This bit when set, indicates credit based shaper algorithm (CBS) is selected for Queue 1 traffic. */
        UINT32 CC : 1;                 /**< Credit Control When this bit is set, the accumulated credit parameter in the credit-based shaper algorithm logic is not reset to zero when there is positive credit and no packet to transmit in Channel 1. */
        UINT32 SLC : 3;                 /**< Slot Count If the credit-based shaper algorithm is enabled, the software can program the number of slots (of duration programmed in DMA_CH[n]_Slot_Interval register) over which the average transmitted bits per slot, provided in the MTL_TXQ[N]_ETS_STATUS register, need to be computed for Queue. */
        UINT32 _unused7 : 25;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_ETS_CTRL_t;

/**< Register MTL_TXQX_ETS_CTRL - Queue 1 ETS Control */
#define IMX_ENET_QOS_MTL_TXQX_ETS_CTRL           0xD50
/**< Field AVALG - AV Algorithm When Queue 1 is programmed for AV, this field configures the scheduling algorithm for this queue: This bit when set, indicates credit based shaper algorithm (CBS) is selected for Queue 1 traffic. */
#define IMX_ENET_QOS_MTL_TXQX_ETS_CTRL_AVALG_SHIFT 2
#define IMX_ENET_QOS_MTL_TXQX_ETS_CTRL_AVALG_MASK 0x4
#define IMX_ENET_QOS_MTL_TXQX_ETS_CTRL_AVALG(v)  (((v) << 2) & 0x4)
/**< Field CC - Credit Control When this bit is set, the accumulated credit parameter in the credit-based shaper algorithm logic is not reset to zero when there is positive credit and no packet to transmit in Channel 1. */
#define IMX_ENET_QOS_MTL_TXQX_ETS_CTRL_CC_SHIFT  3
#define IMX_ENET_QOS_MTL_TXQX_ETS_CTRL_CC_MASK   0x8
#define IMX_ENET_QOS_MTL_TXQX_ETS_CTRL_CC(v)     (((v) << 3) & 0x8)
/**< Field SLC - Slot Count If the credit-based shaper algorithm is enabled, the software can program the number of slots (of duration programmed in DMA_CH[n]_Slot_Interval register) over which the average transmitted bits per slot, provided in the MTL_TXQ[N]_ETS_STATUS register, need to be computed for Queue. */
#define IMX_ENET_QOS_MTL_TXQX_ETS_CTRL_SLC_SHIFT 4
#define IMX_ENET_QOS_MTL_TXQX_ETS_CTRL_SLC_MASK  0x70
#define IMX_ENET_QOS_MTL_TXQX_ETS_CTRL_SLC(v)    (((v) << 4) & 0x70)

/**< IMX_ENET_QOS_MTL_TXQX_ETS_STAT - Queue 1 ETS Status */
typedef union IMX_ENET_QOS_MTL_TXQX_ETS_STAT_union_t {
    UINT32 R;
    struct {
        UINT32 ABS : 24;                /**< Average Bits per Slot This field contains the average transmitted bits per slot. */
        UINT32 _unused24 : 8;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_ETS_STAT_t;

/**< Register MTL_TXQX_ETS_STAT - Queue 1 ETS Status */
#define IMX_ENET_QOS_MTL_TXQX_ETS_STAT           0xD54
/**< Field ABS - Average Bits per Slot This field contains the average transmitted bits per slot. */
#define IMX_ENET_QOS_MTL_TXQX_ETS_STAT_ABS_SHIFT 0
#define IMX_ENET_QOS_MTL_TXQX_ETS_STAT_ABS_MASK  0xFFFFFF
#define IMX_ENET_QOS_MTL_TXQX_ETS_STAT_ABS(v)    (((v) << 0) & 0xFFFFFF)

/**< IMX_ENET_QOS_MTL_TXQX_QNTM_WGHT - Queue 1 idleSlopeCredit, Quantum or Weights */
typedef union IMX_ENET_QOS_MTL_TXQX_QNTM_WGHT_union_t {
    UINT32 R;
    struct {
        UINT32 ISCQW : 21;                /**< idleSlopeCredit, Quantum or Weights - idleSlopeCredit When AV feature is enabled, this field contains the idleSlopeCredit value required for the credit-based shaper algorithm for Queue 1. */
        UINT32 _unused21 : 11;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_QNTM_WGHT_t;

/**< Register MTL_TXQX_QNTM_WGHT - Queue 1 idleSlopeCredit, Quantum or Weights */
#define IMX_ENET_QOS_MTL_TXQX_QNTM_WGHT          0xD58
/**< Field ISCQW - idleSlopeCredit, Quantum or Weights - idleSlopeCredit When AV feature is enabled, this field contains the idleSlopeCredit value required for the credit-based shaper algorithm for Queue 1. */
#define IMX_ENET_QOS_MTL_TXQX_QNTM_WGHT_ISCQW_SHIFT 0
#define IMX_ENET_QOS_MTL_TXQX_QNTM_WGHT_ISCQW_MASK 0x1FFFFF
#define IMX_ENET_QOS_MTL_TXQX_QNTM_WGHT_ISCQW(v) (((v) << 0) & 0x1FFFFF)

/**< IMX_ENET_QOS_MTL_TXQX_SNDSLP_CRDT - Queue 1 sendSlopeCredit */
typedef union IMX_ENET_QOS_MTL_TXQX_SNDSLP_CRDT_union_t {
    UINT32 R;
    struct {
        UINT32 SSC : 14;                /**< sendSlopeCredit Value When AV operation is enabled, this field contains the sendSlopeCredit value required for credit-based shaper algorithm for Queue 1. */
        UINT32 _unused14 : 18;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_SNDSLP_CRDT_t;

/**< Register MTL_TXQX_SNDSLP_CRDT - Queue 1 sendSlopeCredit */
#define IMX_ENET_QOS_MTL_TXQX_SNDSLP_CRDT        0xD5C
/**< Field SSC - sendSlopeCredit Value When AV operation is enabled, this field contains the sendSlopeCredit value required for credit-based shaper algorithm for Queue 1. */
#define IMX_ENET_QOS_MTL_TXQX_SNDSLP_CRDT_SSC_SHIFT 0
#define IMX_ENET_QOS_MTL_TXQX_SNDSLP_CRDT_SSC_MASK 0x3FFF
#define IMX_ENET_QOS_MTL_TXQX_SNDSLP_CRDT_SSC(v) (((v) << 0) & 0x3FFF)

/**< IMX_ENET_QOS_MTL_TXQX_HI_CRDT - Queue 1 hiCredit */
typedef union IMX_ENET_QOS_MTL_TXQX_HI_CRDT_union_t {
    UINT32 R;
    struct {
        UINT32 HC : 29;                /**< hiCredit Value When the AV feature is enabled, this field contains the hiCredit value required for the credit-based shaper algorithm. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_HI_CRDT_t;

/**< Register MTL_TXQX_HI_CRDT - Queue 1 hiCredit */
#define IMX_ENET_QOS_MTL_TXQX_HI_CRDT            0xD60
/**< Field HC - hiCredit Value When the AV feature is enabled, this field contains the hiCredit value required for the credit-based shaper algorithm. */
#define IMX_ENET_QOS_MTL_TXQX_HI_CRDT_HC_SHIFT   0
#define IMX_ENET_QOS_MTL_TXQX_HI_CRDT_HC_MASK    0x1FFFFFFF
#define IMX_ENET_QOS_MTL_TXQX_HI_CRDT_HC(v)      (((v) << 0) & 0x1FFFFFFF)

/**< IMX_ENET_QOS_MTL_TXQX_LO_CRDT - Queue 1 loCredit */
typedef union IMX_ENET_QOS_MTL_TXQX_LO_CRDT_union_t {
    UINT32 R;
    struct {
        UINT32 LC : 29;                /**< loCredit Value When AV operation is enabled, this field contains the loCredit value required for the credit-based shaper algorithm. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_LO_CRDT_t;

/**< Register MTL_TXQX_LO_CRDT - Queue 1 loCredit */
#define IMX_ENET_QOS_MTL_TXQX_LO_CRDT            0xD64
/**< Field LC - loCredit Value When AV operation is enabled, this field contains the loCredit value required for the credit-based shaper algorithm. */
#define IMX_ENET_QOS_MTL_TXQX_LO_CRDT_LC_SHIFT   0
#define IMX_ENET_QOS_MTL_TXQX_LO_CRDT_LC_MASK    0x1FFFFFFF
#define IMX_ENET_QOS_MTL_TXQX_LO_CRDT_LC(v)      (((v) << 0) & 0x1FFFFFFF)

/**< IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT - Queue 1 Interrupt Control Status */
typedef union IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_union_t {
    UINT32 R;
    struct {
        UINT32 TXUNFIS : 1;                 /**< Transmit Queue Underflow Interrupt Status This bit indicates that the Transmit Queue had an underflow while transmitting the packet. */
        UINT32 ABPSIS : 1;                 /**< Average Bits Per Slot Interrupt Status When set, this bit indicates that the MAC has updated the ABS value. */
        UINT32 _unused2 : 6;                 /**< Reserved. */
        UINT32 TXUIE : 1;                 /**< Transmit Queue Underflow Interrupt Enable When this bit is set, the Transmit Queue Underflow interrupt is enabled. */
        UINT32 ABPSIE : 1;                 /**< Average Bits Per Slot Interrupt Enable When this bit is set, the MAC asserts the sbd_intr_o or mci_intr_o interrupt when the average bits per slot status is updated. */
        UINT32 _unused10 : 6;                 /**< Reserved. */
        UINT32 RXOVFIS : 1;                 /**< Receive Queue Overflow Interrupt Status This bit indicates that the Receive Queue had an overflow while receiving the packet. */
        UINT32 _unused17 : 7;                 /**< Reserved. */
        UINT32 RXOIE : 1;                 /**< Receive Queue Overflow Interrupt Enable When this bit is set, the Receive Queue Overflow interrupt is enabled. */
        UINT32 _unused25 : 7;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_INTCTRL_STAT_t;

/**< Register MTL_TXQX_INTCTRL_STAT - Queue 1 Interrupt Control Status */
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT       0xD6C
/**< Field TXUNFIS - Transmit Queue Underflow Interrupt Status This bit indicates that the Transmit Queue had an underflow while transmitting the packet. */
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_TXUNFIS_SHIFT 0
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_TXUNFIS_MASK 0x1
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_TXUNFIS(v) (((v) << 0) & 0x1)
/**< Field ABPSIS - Average Bits Per Slot Interrupt Status When set, this bit indicates that the MAC has updated the ABS value. */
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_ABPSIS_SHIFT 1
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_ABPSIS_MASK 0x2
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_ABPSIS(v) (((v) << 1) & 0x2)
/**< Field TXUIE - Transmit Queue Underflow Interrupt Enable When this bit is set, the Transmit Queue Underflow interrupt is enabled. */
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_TXUIE_SHIFT 8
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_TXUIE_MASK 0x100
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_TXUIE(v) (((v) << 8) & 0x100)
/**< Field ABPSIE - Average Bits Per Slot Interrupt Enable When this bit is set, the MAC asserts the sbd_intr_o or mci_intr_o interrupt when the average bits per slot status is updated. */
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_ABPSIE_SHIFT 9
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_ABPSIE_MASK 0x200
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_ABPSIE(v) (((v) << 9) & 0x200)
/**< Field RXOVFIS - Receive Queue Overflow Interrupt Status This bit indicates that the Receive Queue had an overflow while receiving the packet. */
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_RXOVFIS_SHIFT 16
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_RXOVFIS_MASK 0x10000
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_RXOVFIS(v) (((v) << 16) & 0x10000)
/**< Field RXOIE - Receive Queue Overflow Interrupt Enable When this bit is set, the Receive Queue Overflow interrupt is enabled. */
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_RXOIE_SHIFT 24
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_RXOIE_MASK 0x1000000
#define IMX_ENET_QOS_MTL_TXQX_INTCTRL_STAT_RXOIE(v) (((v) << 24) & 0x1000000)

/**< IMX_ENET_QOS_MTL_RXQX_OP_MODE - Queue 1 Receive Operation Mode */
typedef union IMX_ENET_QOS_MTL_RXQX_OP_MODE_union_t {
    UINT32 R;
    struct {
        UINT32 RTC : 2;                 /**< Receive Queue Threshold Control These bits control the threshold level of the MTL Rx queue (in bytes): The received packet is transferred to the application or DMA when the packet size within the MTL Rx queue is larger than the threshold. */
        UINT32 _unused2 : 1;                 /**< Reserved. */
        UINT32 FUP : 1;                 /**< Forward Undersized Good Packets When this bit is set, the Rx queue forwards the undersized good packets (packets with no error and length less than 64 bytes), including pad-bytes and CRC. */
        UINT32 FEP : 1;                 /**< Forward Error Packets When this bit is reset, the Rx queue drops packets with error status (CRC error, GMII_ER, watchdog timeout, or overflow). */
        UINT32 RSF : 1;                 /**< Receive Queue Store and Forward When this bit is set, the DWC_ether_qos reads a packet from the Rx queue only after the complete packet has been written to it, ignoring the RTC field of this register. */
        UINT32 DIS_TCP_EF : 1;                 /**< Disable Dropping of TCP/IP Checksum Error Packets When this bit is set, the MAC does not drop the packets which only have the errors detected by the Receive Checksum Offload engine. */
        UINT32 EHFC : 1;                 /**< Enable Hardware Flow Control When this bit is set, the flow control signal operation, based on the fill-level of Rx queue, is enabled. */
        UINT32 RFA : 4;                 /**< Threshold for Activating Flow Control (in half-duplex and full-duplex These bits control the threshold (fill-level of Rx queue) at which the flow control is activated: For more information on encoding for this field, see RFD. */
        UINT32 _unused12 : 2;                 /**< Reserved. */
        UINT32 RFD : 4;                 /**< Threshold for Deactivating Flow Control (in half-duplex and full-duplex modes) These bits control the threshold (fill-level of Rx queue) at which the flow control is de-asserted after activation: - 0: Full minus 1 KB, that is, FULL 1 KB - 1: Full minus 1. */
        UINT32 _unused18 : 2;                 /**< Reserved. */
        UINT32 RQS : 5;                 /**< Receive Queue Size This field indicates the size of the allocated Receive queues in blocks of 256 bytes. */
        UINT32 _unused25 : 7;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_RXQX_OP_MODE_t;

/**< Register MTL_RXQX_OP_MODE - Queue 1 Receive Operation Mode */
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE            0xD70
/**< Field RTC - Receive Queue Threshold Control These bits control the threshold level of the MTL Rx queue (in bytes): The received packet is transferred to the application or DMA when the packet size within the MTL Rx queue is larger than the threshold. */
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RTC_SHIFT  0
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RTC_MASK   0x3
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RTC(v)     (((v) << 0) & 0x3)
/**< Field FUP - Forward Undersized Good Packets When this bit is set, the Rx queue forwards the undersized good packets (packets with no error and length less than 64 bytes), including pad-bytes and CRC. */
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_FUP_SHIFT  3
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_FUP_MASK   0x8
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_FUP(v)     (((v) << 3) & 0x8)
/**< Field FEP - Forward Error Packets When this bit is reset, the Rx queue drops packets with error status (CRC error, GMII_ER, watchdog timeout, or overflow). */
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_FEP_SHIFT  4
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_FEP_MASK   0x10
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_FEP(v)     (((v) << 4) & 0x10)
/**< Field RSF - Receive Queue Store and Forward When this bit is set, the DWC_ether_qos reads a packet from the Rx queue only after the complete packet has been written to it, ignoring the RTC field of this register. */
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RSF_SHIFT  5
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RSF_MASK   0x20
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RSF(v)     (((v) << 5) & 0x20)
/**< Field DIS_TCP_EF - Disable Dropping of TCP/IP Checksum Error Packets When this bit is set, the MAC does not drop the packets which only have the errors detected by the Receive Checksum Offload engine. */
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_DIS_TCP_EF_SHIFT 6
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_DIS_TCP_EF_MASK 0x40
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_DIS_TCP_EF(v) (((v) << 6) & 0x40)
/**< Field EHFC - Enable Hardware Flow Control When this bit is set, the flow control signal operation, based on the fill-level of Rx queue, is enabled. */
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_EHFC_SHIFT 7
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_EHFC_MASK  0x80
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_EHFC(v)    (((v) << 7) & 0x80)
/**< Field RFA - Threshold for Activating Flow Control (in half-duplex and full-duplex These bits control the threshold (fill-level of Rx queue) at which the flow control is activated: For more information on encoding for this field, see RFD. */
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RFA_SHIFT  8
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RFA_MASK   0xF00
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RFA(v)     (((v) << 8) & 0xF00)
/**< Field RFD - Threshold for Deactivating Flow Control (in half-duplex and full-duplex modes) These bits control the threshold (fill-level of Rx queue) at which the flow control is de-asserted after activation: - 0: Full minus 1 KB, that is, FULL 1 KB - 1: Full minus 1. */
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RFD_SHIFT  14
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RFD_MASK   0x3C000
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RFD(v)     (((v) << 14) & 0x3C000)
/**< Field RQS - Receive Queue Size This field indicates the size of the allocated Receive queues in blocks of 256 bytes. */
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RQS_SHIFT  20
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RQS_MASK   0x1F00000
#define IMX_ENET_QOS_MTL_RXQX_OP_MODE_RQS(v)     (((v) << 20) & 0x1F00000)

/**< IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT - Queue 1 Missed Packet and Overflow Counter */
typedef union IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_union_t {
    UINT32 R;
    struct {
        UINT32 OVFPKTCNT : 11;                /**< Overflow Packet Counter This field indicates the number of packets discarded by the DWC_ether_qos because of Receive queue overflow. */
        UINT32 OVFCNTOVF : 1;                 /**< Overflow Counter Overflow Bit When set, this bit indicates that the Rx Queue Overflow Packet Counter field crossed the maximum limit. */
        UINT32 _unused12 : 4;                 /**< Reserved. */
        UINT32 MISPKTCNT : 11;                /**< Missed Packet Counter This field indicates the number of packets missed by the DWC_ether_qos because the application asserted ari_pkt_flush_i[] for this queue. */
        UINT32 MISCNTOVF : 1;                 /**< Missed Packet Counter Overflow Bit When set, this bit indicates that the Rx Queue Missed Packet Counter crossed the maximum limit. */
        UINT32 _unused28 : 4;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_RXQX_MISSPKT_OVRFLW_CNT_t;

/**< Register MTL_RXQX_MISSPKT_OVRFLW_CNT - Queue 1 Missed Packet and Overflow Counter */
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT 0xD74
/**< Field OVFPKTCNT - Overflow Packet Counter This field indicates the number of packets discarded by the DWC_ether_qos because of Receive queue overflow. */
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_OVFPKTCNT_SHIFT 0
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_OVFPKTCNT_MASK 0x7FF
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_OVFPKTCNT(v) (((v) << 0) & 0x7FF)
/**< Field OVFCNTOVF - Overflow Counter Overflow Bit When set, this bit indicates that the Rx Queue Overflow Packet Counter field crossed the maximum limit. */
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_OVFCNTOVF_SHIFT 11
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_OVFCNTOVF_MASK 0x800
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_OVFCNTOVF(v) (((v) << 11) & 0x800)
/**< Field MISPKTCNT - Missed Packet Counter This field indicates the number of packets missed by the DWC_ether_qos because the application asserted ari_pkt_flush_i[] for this queue. */
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_MISPKTCNT_SHIFT 16
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_MISPKTCNT_MASK 0x7FF0000
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_MISPKTCNT(v) (((v) << 16) & 0x7FF0000)
/**< Field MISCNTOVF - Missed Packet Counter Overflow Bit When set, this bit indicates that the Rx Queue Missed Packet Counter crossed the maximum limit. */
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_MISCNTOVF_SHIFT 27
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_MISCNTOVF_MASK 0x8000000
#define IMX_ENET_QOS_MTL_RXQX_MISSPKT_OVRFLW_CNT_MISCNTOVF(v) (((v) << 27) & 0x8000000)

/**< IMX_ENET_QOS_MTL_RXQX_DBG - Queue 1 Receive Debug */
typedef union IMX_ENET_QOS_MTL_RXQX_DBG_union_t {
    UINT32 R;
    struct {
        UINT32 RWCSTS : 1;                 /**< MTL Rx Queue Write Controller Active Status When high, this bit indicates that the MTL Rx queue Write controller is active, and it is transferring a received packet to the Rx Queue. */
        UINT32 RRCSTS : 2;                 /**< MTL Rx Queue Read Controller State This field gives the state of the Rx queue Read controller: */
        UINT32 _unused3 : 1;                 /**< Reserved. */
        UINT32 RXQSTS : 2;                 /**< MTL Rx Queue Fill-Level Status This field gives the status of the fill-level of the Rx Queue: */
        UINT32 _unused6 : 10;                /**< Reserved. */
        UINT32 PRXQ : 14;                /**< Number of Packets in Receive Queue This field indicates the current number of packets in the Rx Queue. */
        UINT32 _unused30 : 2;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_RXQX_DBG_t;

/**< Register MTL_RXQX_DBG - Queue 1 Receive Debug */
#define IMX_ENET_QOS_MTL_RXQX_DBG                0xD78
/**< Field RWCSTS - MTL Rx Queue Write Controller Active Status When high, this bit indicates that the MTL Rx queue Write controller is active, and it is transferring a received packet to the Rx Queue. */
#define IMX_ENET_QOS_MTL_RXQX_DBG_RWCSTS_SHIFT   0
#define IMX_ENET_QOS_MTL_RXQX_DBG_RWCSTS_MASK    0x1
#define IMX_ENET_QOS_MTL_RXQX_DBG_RWCSTS(v)      (((v) << 0) & 0x1)
/**< Field RRCSTS - MTL Rx Queue Read Controller State This field gives the state of the Rx queue Read controller: */
#define IMX_ENET_QOS_MTL_RXQX_DBG_RRCSTS_SHIFT   1
#define IMX_ENET_QOS_MTL_RXQX_DBG_RRCSTS_MASK    0x6
#define IMX_ENET_QOS_MTL_RXQX_DBG_RRCSTS(v)      (((v) << 1) & 0x6)
/**< Field RXQSTS - MTL Rx Queue Fill-Level Status This field gives the status of the fill-level of the Rx Queue: */
#define IMX_ENET_QOS_MTL_RXQX_DBG_RXQSTS_SHIFT   4
#define IMX_ENET_QOS_MTL_RXQX_DBG_RXQSTS_MASK    0x30
#define IMX_ENET_QOS_MTL_RXQX_DBG_RXQSTS(v)      (((v) << 4) & 0x30)
/**< Field PRXQ - Number of Packets in Receive Queue This field indicates the current number of packets in the Rx Queue. */
#define IMX_ENET_QOS_MTL_RXQX_DBG_PRXQ_SHIFT     16
#define IMX_ENET_QOS_MTL_RXQX_DBG_PRXQ_MASK      0x3FFF0000
#define IMX_ENET_QOS_MTL_RXQX_DBG_PRXQ(v)        (((v) << 16) & 0x3FFF0000)

/**< IMX_ENET_QOS_MTL_RXQX_CTRL - Queue 1 Receive Control */
typedef union IMX_ENET_QOS_MTL_RXQX_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 RXQ_WEGT : 3;                 /**< Receive Queue Weight This field indicates the weight assigned to the Rx Queue 0. */
        UINT32 RXQ_FRM_ARBIT : 1;               /**< Receive Queue Packet Arbitration When this bit is set, the DWC_ether_qos drives the packet data to the ARI interface such that the entire packet data of currently-selected queue is transmitted before switching to other queue. */
        UINT32 _unused4 : 28;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_MTL_QUEUE_MTL_RXQX_CTRL_t;

/**< Register MTL_RXQX_CTRL - Queue 1 Receive Control */
#define IMX_ENET_QOS_MTL_RXQX_CTRL               0xD7C
/**< Field RXQ_WEGT - Receive Queue Weight This field indicates the weight assigned to the Rx Queue 0. */
#define IMX_ENET_QOS_MTL_RXQX_CTRL_RXQ_WEGT_SHIFT 0
#define IMX_ENET_QOS_MTL_RXQX_CTRL_RXQ_WEGT_MASK 0x7
#define IMX_ENET_QOS_MTL_RXQX_CTRL_RXQ_WEGT(v)   (((v) << 0) & 0x7)
/**< Field RXQ_FRM_ARBIT - Receive Queue Packet Arbitration When this bit is set, the DWC_ether_qos drives the packet data to the ARI interface such that the entire packet data of currently-selected queue is transmitted before switching to other queue. */
#define IMX_ENET_QOS_MTL_RXQX_CTRL_RXQ_FRM_ARBIT_SHIFT 3
#define IMX_ENET_QOS_MTL_RXQX_CTRL_RXQ_FRM_ARBIT_MASK 0x8
#define IMX_ENET_QOS_MTL_RXQX_CTRL_RXQ_FRM_ARBIT(v) (((v) << 3) & 0x8)

typedef struct IMX_ENET_QOS_MTL_QUEUE_struct_t {
    volatile IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_OP_MODE_t MTL_TXQX_OP_MODE;  /**< Queue 1 Transmit Operation Mode */
    volatile IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_UNDRFLW_t MTL_TXQX_UNDRFLW;  /**< Queue 1 Underflow Counter */
    volatile IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_DBG_t MTL_TXQX_DBG;  /**< Queue 1 Transmit Debug */
    UINT32 MTL_QUEUE_reserved0[2];
    //volatile IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_ETS_CTRL_t MTL_TXQX_ETS_CTRL;  /**< Queue 1 ETS Control */
    volatile IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_ETS_STAT_t MTL_TXQX_ETS_STAT;  /**< Queue 1 ETS Status */
    volatile IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_QNTM_WGHT_t MTL_TXQX_QNTM_WGHT;  /**< Queue 1 idleSlopeCredit, Quantum or Weights */
    UINT32 MTL_QUEUE_reserved1[4];
    //volatile IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_SNDSLP_CRDT_t MTL_TXQX_SNDSLP_CRDT;  /**< Queue 1 sendSlopeCredit */
    //volatile IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_HI_CRDT_t MTL_TXQX_HI_CRDT;  /**< Queue 1 hiCredit */
    //volatile IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_LO_CRDT_t MTL_TXQX_LO_CRDT;  /**< Queue 1 loCredit */
    //UINT32 MTL_QUEUE_reserved1[4];
    volatile IMX_ENET_QOS_MTL_QUEUE_MTL_TXQX_INTCTRL_STAT_t MTL_TXQX_INTCTRL_STAT;  /**< Queue 1 Interrupt Control Status */
    volatile IMX_ENET_QOS_MTL_QUEUE_MTL_RXQX_OP_MODE_t MTL_RXQX_OP_MODE;  /**< Queue 1 Receive Operation Mode */
    volatile IMX_ENET_QOS_MTL_QUEUE_MTL_RXQX_MISSPKT_OVRFLW_CNT_t MTL_RXQX_MISSPKT_OVRFLW_CNT;  /**< Queue 1 Missed Packet and Overflow Counter */
    volatile IMX_ENET_QOS_MTL_QUEUE_MTL_RXQX_DBG_t MTL_RXQX_DBG;  /**< Queue 1 Receive Debug */
    volatile IMX_ENET_QOS_MTL_QUEUE_MTL_RXQX_CTRL_t MTL_RXQX_CTRL;  /**< Queue 1 Receive Control */
} IMX_ENET_QOS_MTL_QUEUE_t;

/**< IMX_ENET_QOS_DMA_MODE - DMA Bus Mode */
typedef union IMX_ENET_QOS_DMA_MODE_union_t {
    UINT32 R;
    struct {
        UINT32 SWR : 1;                 /**< Software Reset When this bit is set, the MAC and the DMA controller reset the logic and all internal registers of the DMA, MTL, and MAC. */
        UINT32 _unused1 : 1;                 /**< Reserved. */
        UINT32 _unused2 : 3;                 /**< Reserved. */
        UINT32 _unused5 : 3;                 /**< Reserved. */
        UINT32 DSPW : 1;                 /**< Descriptor Posted Write When this bit is set to 0, the descriptor writes are always non-posted. */
        UINT32 _unused9 : 1;                 /**< Reserved. */
        UINT32 _unused10 : 1;                 /**< Reserved. */
        UINT32 _unused11 : 1;                 /**< Reserved. */
        UINT32 _unused12 : 3;                 /**< Reserved. */
        UINT32 _unused15 : 1;                 /**< Reserved. */
        UINT32 INTM : 2;                 /**< Interrupt Mode This field defines the interrupt mode of DWC_ether_qos. */
        UINT32 _unused18 : 14;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_MODE_t;

/**< Register DMA_MODE - DMA Bus Mode */
#define IMX_ENET_QOS_DMA_MODE                    0x1000
/**< Field SWR - Software Reset When this bit is set, the MAC and the DMA controller reset the logic and all internal registers of the DMA, MTL, and MAC. */
#define IMX_ENET_QOS_DMA_MODE_SWR_SHIFT          0
#define IMX_ENET_QOS_DMA_MODE_SWR_MASK           0x1
#define IMX_ENET_QOS_DMA_MODE_SWR(v)             (((v) << 0) & 0x1)
/**< Field DSPW - Descriptor Posted Write When this bit is set to 0, the descriptor writes are always non-posted. */
#define IMX_ENET_QOS_DMA_MODE_DSPW_SHIFT         8
#define IMX_ENET_QOS_DMA_MODE_DSPW_MASK          0x100
#define IMX_ENET_QOS_DMA_MODE_DSPW(v)            (((v) << 8) & 0x100)
/**< Field INTM - Interrupt Mode This field defines the interrupt mode of DWC_ether_qos. */
#define IMX_ENET_QOS_DMA_MODE_INTM_SHIFT         16
#define IMX_ENET_QOS_DMA_MODE_INTM_MASK          0x30000
#define IMX_ENET_QOS_DMA_MODE_INTM(v)            (((v) << 16) & 0x30000)

/**< IMX_ENET_QOS_DMA_SYSBUS_MODE - DMA System Bus Mode */
typedef union IMX_ENET_QOS_DMA_SYSBUS_MODE_union_t {
    UINT32 R;
    struct {
        UINT32 FB : 1;                 /**< Fixed Burst Length When this bit is set to 1, the EQOS-AXI master initiates burst transfers of specified lengths as given below. */
        UINT32 BLEN4 : 1;                 /**< AXI Burst Length 4 When this bit is set to 1 or the FB bit is set to 0, the EQOS-AXI master can select a burst length of 4 on the AXI interface. */
        UINT32 BLEN8 : 1;                 /**< AXI Burst Length 8 When this bit is set to 1 or the FB bit is set to 0, the EQOS-AXI master can select a burst length of 8 on the AXI interface. */
        UINT32 BLEN16 : 1;                 /**< AXI Burst Length 16 When this bit is set to 1 or the FB bit is set to 0, the EQOS-AXI master can select a burst length of 16 on the AXI interface. */
        UINT32 _unused4 : 1;                 /**< Reserved. */
        UINT32 _unused5 : 1;                 /**< Reserved. */
        UINT32 _unused6 : 1;                 /**< Reserved. */
        UINT32 _unused7 : 1;                 /**< Reserved. */
        UINT32 _unused8 : 2;                 /**< Reserved. */
        UINT32 AALE : 1;                 /**< Automatic AXI LPI enable When set to 1, enables the AXI master to enter into LPI state when there is no activity in the DWC_ether_qos for number of system clock cycles programmed in the LPIEI field of DMA_AXI_LPI_ENTRY_INTERVAL register. */
        UINT32 _unused11 : 1;                 /**< Reserved. */
        UINT32 AAL : 1;                 /**< Address-Aligned Beats When this bit is set to 1, the EQOS-AXI or EQOS-AHB master performs address-aligned burst transfers on Read and Write channels. */
        UINT32 ONEKBBE : 1;                 /**< 1 KB Boundary Crossing Enable for the EQOS-AXI Master When set, the burst transfers performed by the EQOS-AXI master do not cross 1 KB boundary. */
        UINT32 _unused14 : 1;                 /**< Reserved. */
        UINT32 _unused15 : 1;                 /**< Reserved. */
        UINT32 RD_OSR_LMT : 4;                 /**< AXI Maximum Read Outstanding Request Limit This value limits the maximum outstanding request on the AXI read interface. */
        UINT32 _unused20 : 4;                 /**< Reserved. */
        UINT32 WR_OSR_LMT : 4;                 /**< AXI Maximum Write Outstanding Request Limit This value limits the maximum outstanding request on the AXI write interface. */
        UINT32 _unused28 : 2;                 /**< Reserved. */
        UINT32 LPI_XIT_PKT : 1;                 /**< Unlock on Magic Packet or Remote Wake-Up Packet When set to 1, this bit enables the AXI master to come out of the LPI mode only when the magic packet or remote wake-up packet is received. */
        UINT32 EN_LPI : 1;                 /**< Enable Low Power Interface (LPI) When set to 1, this bit enables the LPI mode supported by the EQOS-AXI configuration and accepts the LPI request from the AXI System Clock controller. */
    } B;
} IMX_ENET_QOS_DMA_SYSBUS_MODE_t;

/**< Register DMA_SYSBUS_MODE - DMA System Bus Mode */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE             0x1004
/**< Field FB - Fixed Burst Length When this bit is set to 1, the EQOS-AXI master initiates burst transfers of specified lengths as given below. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_FB_SHIFT    0
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_FB_MASK     0x1
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_FB(v)       (((v) << 0) & 0x1)
/**< Field BLEN4 - AXI Burst Length 4 When this bit is set to 1 or the FB bit is set to 0, the EQOS-AXI master can select a burst length of 4 on the AXI interface. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN4_SHIFT 1
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN4_MASK  0x2
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN4(v)    (((v) << 1) & 0x2)
/**< Field BLEN8 - AXI Burst Length 8 When this bit is set to 1 or the FB bit is set to 0, the EQOS-AXI master can select a burst length of 8 on the AXI interface. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN8_SHIFT 2
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN8_MASK  0x4
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN8(v)    (((v) << 2) & 0x4)
/**< Field BLEN16 - AXI Burst Length 16 When this bit is set to 1 or the FB bit is set to 0, the EQOS-AXI master can select a burst length of 16 on the AXI interface. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN16_SHIFT 3
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN16_MASK 0x8
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_BLEN16(v)   (((v) << 3) & 0x8)
/**< Field AALE - Automatic AXI LPI enable When set to 1, enables the AXI master to enter into LPI state when there is no activity in the DWC_ether_qos for number of system clock cycles programmed in the LPIEI field of DMA_AXI_LPI_ENTRY_INTERVAL register. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_AALE_SHIFT  10
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_AALE_MASK   0x400
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_AALE(v)     (((v) << 10) & 0x400)
/**< Field AAL - Address-Aligned Beats When this bit is set to 1, the EQOS-AXI or EQOS-AHB master performs address-aligned burst transfers on Read and Write channels. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_AAL_SHIFT   12
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_AAL_MASK    0x1000
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_AAL(v)      (((v) << 12) & 0x1000)
/**< Field ONEKBBE - 1 KB Boundary Crossing Enable for the EQOS-AXI Master When set, the burst transfers performed by the EQOS-AXI master do not cross 1 KB boundary. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_ONEKBBE_SHIFT 13
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_ONEKBBE_MASK 0x2000
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_ONEKBBE(v)  (((v) << 13) & 0x2000)
/**< Field RD_OSR_LMT - AXI Maximum Read Outstanding Request Limit This value limits the maximum outstanding request on the AXI read interface. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT 16
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_RD_OSR_LMT_MASK 0xF0000
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_RD_OSR_LMT(v) (((v) << 16) & 0xF0000)
/**< Field WR_OSR_LMT - AXI Maximum Write Outstanding Request Limit This value limits the maximum outstanding request on the AXI write interface. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_WR_OSR_LMT_SHIFT 24
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_WR_OSR_LMT_MASK 0xF000000
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_WR_OSR_LMT(v) (((v) << 24) & 0xF000000)
/**< Field LPI_XIT_PKT - Unlock on Magic Packet or Remote Wake-Up Packet When set to 1, this bit enables the AXI master to come out of the LPI mode only when the magic packet or remote wake-up packet is received. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_LPI_XIT_PKT_SHIFT 30
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_LPI_XIT_PKT_MASK 0x40000000
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_LPI_XIT_PKT(v) (((v) << 30) & 0x40000000)
/**< Field EN_LPI - Enable Low Power Interface (LPI) When set to 1, this bit enables the LPI mode supported by the EQOS-AXI configuration and accepts the LPI request from the AXI System Clock controller. */
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_EN_LPI_SHIFT 31
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_EN_LPI_MASK 0x80000000
#define IMX_ENET_QOS_DMA_SYSBUS_MODE_EN_LPI(v)   (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_DMA_INTERRUPT_STATUS - DMA Interrupt Status */
typedef union IMX_ENET_QOS_DMA_INTERRUPT_STATUS_union_t {
    UINT32 R;
    struct {
        UINT32 DC0IS : 1;                 /**< DMA Channel 0 Interrupt Status This bit indicates an interrupt event in DMA Channel 0. */
        UINT32 DC1IS : 1;                 /**< DMA Channel 1 Interrupt Status This bit indicates an interrupt event in DMA Channel 1. */
        UINT32 DC2IS : 1;                 /**< DMA Channel 2 Interrupt Status This bit indicates an interrupt event in DMA Channel 2. */
        UINT32 DC3IS : 1;                 /**< DMA Channel 3 Interrupt Status This bit indicates an interrupt event in DMA Channel 3. */
        UINT32 DC4IS : 1;                 /**< DMA Channel 4 Interrupt Status This bit indicates an interrupt event in DMA Channel 4. */
        UINT32 _unused5 : 1;                 /**< Reserved. */
        UINT32 _unused6 : 1;                 /**< Reserved. */
        UINT32 _unused7 : 1;                 /**< Reserved. */
        UINT32 _unused8 : 8;                 /**< Reserved. */
        UINT32 MTLIS : 1;                 /**< MTL Interrupt Status This bit indicates an interrupt event in the MTL. */
        UINT32 MACIS : 1;                 /**< MAC Interrupt Status This bit indicates an interrupt event in the MAC. */
        UINT32 _unused18 : 14;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_INTERRUPT_STATUS_t;

/**< Register DMA_INTERRUPT_STATUS - DMA Interrupt Status */
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS        0x1008
/**< Field DC0IS - DMA Channel 0 Interrupt Status This bit indicates an interrupt event in DMA Channel 0. */
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC0IS_SHIFT 0
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC0IS_MASK 0x1
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC0IS(v) (((v) << 0) & 0x1)
/**< Field DC1IS - DMA Channel 1 Interrupt Status This bit indicates an interrupt event in DMA Channel 1. */
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC1IS_SHIFT 1
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC1IS_MASK 0x2
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC1IS(v) (((v) << 1) & 0x2)
/**< Field DC2IS - DMA Channel 2 Interrupt Status This bit indicates an interrupt event in DMA Channel 2. */
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC2IS_SHIFT 2
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC2IS_MASK 0x4
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC2IS(v) (((v) << 2) & 0x4)
/**< Field DC3IS - DMA Channel 3 Interrupt Status This bit indicates an interrupt event in DMA Channel 3. */
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC3IS_SHIFT 3
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC3IS_MASK 0x8
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC3IS(v) (((v) << 3) & 0x8)
/**< Field DC4IS - DMA Channel 4 Interrupt Status This bit indicates an interrupt event in DMA Channel 4. */
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC4IS_SHIFT 4
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC4IS_MASK 0x10
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_DC4IS(v) (((v) << 4) & 0x10)
/**< Field MTLIS - MTL Interrupt Status This bit indicates an interrupt event in the MTL. */
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_MTLIS_SHIFT 16
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_MTLIS_MASK 0x10000
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_MTLIS(v) (((v) << 16) & 0x10000)
/**< Field MACIS - MAC Interrupt Status This bit indicates an interrupt event in the MAC. */
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_MACIS_SHIFT 17
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_MACIS_MASK 0x20000
#define IMX_ENET_QOS_DMA_INTERRUPT_STATUS_MACIS(v) (((v) << 17) & 0x20000)

/**< IMX_ENET_QOS_DMA_DEBUG_STATUS0 - DMA Debug Status 0 */
typedef union IMX_ENET_QOS_DMA_DEBUG_STATUS0_union_t {
    UINT32 R;
    struct {
        UINT32 AXWHSTS : 1;                 /**< AXI Master Write Channel When high, this bit indicates that the write channel of the AXI master is active, and it is transferring data. */
        UINT32 AXRHSTS : 1;                 /**< AXI Master Read Channel Status When high, this bit indicates that the read channel of the AXI master is active, and it is transferring the data. */
        UINT32 _unused2 : 6;                 /**< Reserved. */
        UINT32 RPS0 : 4;                 /**< DMA Channel 0 Receive Process State This field indicates the Rx DMA FSM state for Channel 0. */
        UINT32 TPS0 : 4;                 /**< DMA Channel 0 Transmit Process State This field indicates the Tx DMA FSM state for Channel 0. */
        UINT32 RPS1 : 4;                 /**< DMA Channel 1 Receive Process State This field indicates the Rx DMA FSM state for Channel 1. */
        UINT32 TPS1 : 4;                 /**< DMA Channel 1 Transmit Process State This field indicates the Tx DMA FSM state for Channel 1. */
        UINT32 RPS2 : 4;                 /**< DMA Channel 2 Receive Process State This field indicates the Rx DMA FSM state for Channel 2. */
        UINT32 TPS2 : 4;                 /**< DMA Channel 2 Transmit Process State This field indicates the Tx DMA FSM state for Channel 2. */
    } B;
} IMX_ENET_QOS_DMA_DEBUG_STATUS0_t;

/**< Register DMA_DEBUG_STATUS0 - DMA Debug Status 0 */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0           0x100C
/**< Field AXWHSTS - AXI Master Write Channel When high, this bit indicates that the write channel of the AXI master is active, and it is transferring data. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_AXWHSTS_SHIFT 0
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_AXWHSTS_MASK 0x1
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_AXWHSTS(v) (((v) << 0) & 0x1)
/**< Field AXRHSTS - AXI Master Read Channel Status When high, this bit indicates that the read channel of the AXI master is active, and it is transferring the data. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_AXRHSTS_SHIFT 1
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_AXRHSTS_MASK 0x2
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_AXRHSTS(v) (((v) << 1) & 0x2)
/**< Field RPS0 - DMA Channel 0 Receive Process State This field indicates the Rx DMA FSM state for Channel 0. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_RPS0_SHIFT 8
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_RPS0_MASK 0xF00
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_RPS0(v)   (((v) << 8) & 0xF00)
/**< Field TPS0 - DMA Channel 0 Transmit Process State This field indicates the Tx DMA FSM state for Channel 0. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_TPS0_SHIFT 12
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_TPS0_MASK 0xF000
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_TPS0(v)   (((v) << 12) & 0xF000)
/**< Field RPS1 - DMA Channel 1 Receive Process State This field indicates the Rx DMA FSM state for Channel 1. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_RPS1_SHIFT 16
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_RPS1_MASK 0xF0000
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_RPS1(v)   (((v) << 16) & 0xF0000)
/**< Field TPS1 - DMA Channel 1 Transmit Process State This field indicates the Tx DMA FSM state for Channel 1. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_TPS1_SHIFT 20
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_TPS1_MASK 0xF00000
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_TPS1(v)   (((v) << 20) & 0xF00000)
/**< Field RPS2 - DMA Channel 2 Receive Process State This field indicates the Rx DMA FSM state for Channel 2. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_RPS2_SHIFT 24
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_RPS2_MASK 0xF000000
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_RPS2(v)   (((v) << 24) & 0xF000000)
/**< Field TPS2 - DMA Channel 2 Transmit Process State This field indicates the Tx DMA FSM state for Channel 2. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_TPS2_SHIFT 28
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_TPS2_MASK 0xF0000000
#define IMX_ENET_QOS_DMA_DEBUG_STATUS0_TPS2(v)   (((v) << 28) & 0xF0000000)

/**< IMX_ENET_QOS_DMA_DEBUG_STATUS1 - DMA Debug Status 1 */
typedef union IMX_ENET_QOS_DMA_DEBUG_STATUS1_union_t {
    UINT32 R;
    struct {
        UINT32 RPS3 : 4;                 /**< DMA Channel 3 Receive Process State This field indicates the Rx DMA FSM state for Channel 3. */
        UINT32 TPS3 : 4;                 /**< DMA Channel 3 Transmit Process State This field indicates the Tx DMA FSM state for Channel 3. */
        UINT32 RPS4 : 4;                 /**< DMA Channel 4 Receive Process State This field indicates the Rx DMA FSM state for Channel 4. */
        UINT32 TPS4 : 4;                 /**< DMA Channel 4 Transmit Process State This field indicates the Tx DMA FSM state for Channel 4. */
        UINT32 _unused16 : 4;                 /**< Reserved. */
        UINT32 _unused20 : 4;                 /**< Reserved. */
        UINT32 _unused24 : 4;                 /**< Reserved. */
        UINT32 _unused28 : 4;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_DEBUG_STATUS1_t;

/**< Register DMA_DEBUG_STATUS1 - DMA Debug Status 1 */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1           0x1010
/**< Field RPS3 - DMA Channel 3 Receive Process State This field indicates the Rx DMA FSM state for Channel 3. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_RPS3_SHIFT 0
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_RPS3_MASK 0xF
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_RPS3(v)   (((v) << 0) & 0xF)
/**< Field TPS3 - DMA Channel 3 Transmit Process State This field indicates the Tx DMA FSM state for Channel 3. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_TPS3_SHIFT 4
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_TPS3_MASK 0xF0
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_TPS3(v)   (((v) << 4) & 0xF0)
/**< Field RPS4 - DMA Channel 4 Receive Process State This field indicates the Rx DMA FSM state for Channel 4. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_RPS4_SHIFT 8
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_RPS4_MASK 0xF00
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_RPS4(v)   (((v) << 8) & 0xF00)
/**< Field TPS4 - DMA Channel 4 Transmit Process State This field indicates the Tx DMA FSM state for Channel 4. */
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_TPS4_SHIFT 12
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_TPS4_MASK 0xF000
#define IMX_ENET_QOS_DMA_DEBUG_STATUS1_TPS4(v)   (((v) << 12) & 0xF000)

/**< IMX_ENET_QOS_DMA_AXI_LPI_ENTRY_INTERVAL - AXI LPI Entry Interval Control */
typedef union IMX_ENET_QOS_DMA_AXI_LPI_ENTRY_INTERVAL_union_t {
    UINT32 R;
    struct {
        UINT32 LPIEI : 4;                 /**< LPI Entry Interval Contains the number of system clock cycles, multiplied by 64, to wait for an activity in the DWC_ether_qos to enter into the AXI low power state 0 indicates 64 clock cycles */
        UINT32 _unused4 : 28;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_AXI_LPI_ENTRY_INTERVAL_t;

/**< Register DMA_AXI_LPI_ENTRY_INTERVAL - AXI LPI Entry Interval Control */
#define IMX_ENET_QOS_DMA_AXI_LPI_ENTRY_INTERVAL  0x1040
/**< Field LPIEI - LPI Entry Interval Contains the number of system clock cycles, multiplied by 64, to wait for an activity in the DWC_ether_qos to enter into the AXI low power state 0 indicates 64 clock cycles */
#define IMX_ENET_QOS_DMA_AXI_LPI_ENTRY_INTERVAL_LPIEI_SHIFT 0
#define IMX_ENET_QOS_DMA_AXI_LPI_ENTRY_INTERVAL_LPIEI_MASK 0xF
#define IMX_ENET_QOS_DMA_AXI_LPI_ENTRY_INTERVAL_LPIEI(v) (((v) << 0) & 0xF)

/**< IMX_ENET_QOS_DMA_TBS_CTRL - TBS Control */
typedef union IMX_ENET_QOS_DMA_TBS_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 FTOV : 1;                 /**< Fetch Time Offset Valid When set indicates the FTOS field is valid. */
        UINT32 _unused1 : 3;                 /**< Reserved. */
        UINT32 FGOS : 3;                 /**< Fetch GSN Offset The number GSN slots that must be deducted from the Launch GSN to compute the Fetch GSN. */
        UINT32 _unused7 : 1;                 /**< Reserved. */
        UINT32 FTOS : 24;                /**< Fetch Time Offset The value in units of 256 nanoseconds, that has to be deducted from the Launch time to compute the Fetch Time. */
    } B;
} IMX_ENET_QOS_DMA_TBS_CTRL_t;

/**< Register DMA_TBS_CTRL - TBS Control */
#define IMX_ENET_QOS_DMA_TBS_CTRL                0x1050
/**< Field FTOV - Fetch Time Offset Valid When set indicates the FTOS field is valid. */
#define IMX_ENET_QOS_DMA_TBS_CTRL_FTOV_SHIFT     0
#define IMX_ENET_QOS_DMA_TBS_CTRL_FTOV_MASK      0x1
#define IMX_ENET_QOS_DMA_TBS_CTRL_FTOV(v)        (((v) << 0) & 0x1)
/**< Field FGOS - Fetch GSN Offset The number GSN slots that must be deducted from the Launch GSN to compute the Fetch GSN. */
#define IMX_ENET_QOS_DMA_TBS_CTRL_FGOS_SHIFT     4
#define IMX_ENET_QOS_DMA_TBS_CTRL_FGOS_MASK      0x70
#define IMX_ENET_QOS_DMA_TBS_CTRL_FGOS(v)        (((v) << 4) & 0x70)
/**< Field FTOS - Fetch Time Offset The value in units of 256 nanoseconds, that has to be deducted from the Launch time to compute the Fetch Time. */
#define IMX_ENET_QOS_DMA_TBS_CTRL_FTOS_SHIFT     8
#define IMX_ENET_QOS_DMA_TBS_CTRL_FTOS_MASK      0xFFFFFF00
#define IMX_ENET_QOS_DMA_TBS_CTRL_FTOS(v)        (((v) << 8) & 0xFFFFFF00)

/**< IMX_ENET_QOS_DMA_CHX_CTRL - DMA Channel 0 Control */
typedef union IMX_ENET_QOS_DMA_CHX_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 14;                /**< Reserved. */
        UINT32 _unused14 : 2;                 /**< Reserved. */
        UINT32 PBLX8 : 1;                 /**< 8xPBL mode When this bit is set, the PBL value programmed in Bits[21:16] in DMA_CH0_TX_CONTROL and Bits[21:16] in DMA_CH0_RX_CONTROL is multiplied by eight times. */
        UINT32 _unused17 : 1;                 /**< Reserved. */
        UINT32 DSL : 3;                 /**< Descriptor Skip Length This bit specifies the Word, Dword, or Lword number (depending on the 32-bit, 64-bit, or 128-bit bus) to skip between two unchained descriptors. */
        UINT32 _unused21 : 3;                 /**< Reserved. */
        UINT32 _unused24 : 1;                 /**< Reserved. */
        UINT32 _unused25 : 7;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_CTRL_t;

/**< Register DMA_CHX_CTRL - DMA Channel 0 Control */
#define IMX_ENET_QOS_DMA_CHX_CTRL                0x1100
/**< Field PBLX8 - 8xPBL mode When this bit is set, the PBL value programmed in Bits[21:16] in DMA_CH0_TX_CONTROL and Bits[21:16] in DMA_CH0_RX_CONTROL is multiplied by eight times. */
#define IMX_ENET_QOS_DMA_CHX_CTRL_PBLX8_SHIFT    16
#define IMX_ENET_QOS_DMA_CHX_CTRL_PBLX8_MASK     0x10000
#define IMX_ENET_QOS_DMA_CHX_CTRL_PBLX8(v)       (((v) << 16) & 0x10000)
/**< Field DSL - Descriptor Skip Length This bit specifies the Word, Dword, or Lword number (depending on the 32-bit, 64-bit, or 128-bit bus) to skip between two unchained descriptors. */
#define IMX_ENET_QOS_DMA_CHX_CTRL_DSL_SHIFT      18
#define IMX_ENET_QOS_DMA_CHX_CTRL_DSL_MASK       0x1C0000
#define IMX_ENET_QOS_DMA_CHX_CTRL_DSL(v)         (((v) << 18) & 0x1C0000)

/**< IMX_ENET_QOS_DMA_CHX_TX_CTRL - DMA Channel 0 Transmit Control */
typedef union IMX_ENET_QOS_DMA_CHX_TX_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 ST : 1;                 /**< Start or Stop Transmission Command When this bit is set, transmission is placed in the Running state. */
        UINT32 _unused1 : 3;                 /**< Reserved. */
        UINT32 OSF : 1;                 /**< Operate on Second Packet When this bit is set, it instructs the DMA to process the second packet of the Transmit data even before the status for the first packet is obtained. */
        UINT32 _unused5 : 7;                 /**< Reserved. */
        UINT32 _unused12 : 1;                 /**< Reserved. */
        UINT32 _unused13 : 2;                 /**< Reserved. */
        UINT32 IPBL : 1;                 /**< Ignore PBL Requirement When this bit is set, the DMA does not check for PBL number of locations in the MTL before initiating a transfer. */
        UINT32 TXPBL : 6;                 /**< Transmit Programmable Burst Length These bits indicate the maximum number of beats to be transferred in one DMA block data transfer. */
        UINT32 _unused22 : 1;                 /**< Reserved. */
        UINT32 _unused23 : 1;                 /**< Reserved. */
        UINT32 _unused24 : 4;                 /**< Reserved. */
        UINT32 EDSE : 1;                 /**< Enhanced Descriptor Enable When this bit is set, the corresponding channel uses Enhanced Descriptors that are 32 Bytes for both Normal and Context Descriptors. */
        UINT32 _unused29 : 3;                 /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_TX_CTRL_t;

/**< Register DMA_CHX_TX_CTRL - DMA Channel 0 Transmit Control */
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL             0x1104
/**< Field ST - Start or Stop Transmission Command When this bit is set, transmission is placed in the Running state. */
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_ST_SHIFT    0
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_ST_MASK     0x1
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_ST(v)       (((v) << 0) & 0x1)
/**< Field OSF - Operate on Second Packet When this bit is set, it instructs the DMA to process the second packet of the Transmit data even before the status for the first packet is obtained. */
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_OSF_SHIFT   4
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_OSF_MASK    0x10
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_OSF(v)      (((v) << 4) & 0x10)
/**< Field IPBL - Ignore PBL Requirement When this bit is set, the DMA does not check for PBL number of locations in the MTL before initiating a transfer. */
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_IPBL_SHIFT  15
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_IPBL_MASK   0x8000
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_IPBL(v)     (((v) << 15) & 0x8000)
/**< Field TXPBL - Transmit Programmable Burst Length These bits indicate the maximum number of beats to be transferred in one DMA block data transfer. */
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_TXPBL_SHIFT 16
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_TXPBL_MASK  0x3F0000
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_TXPBL(v)    (((v) << 16) & 0x3F0000)
/**< Field EDSE - Enhanced Descriptor Enable When this bit is set, the corresponding channel uses Enhanced Descriptors that are 32 Bytes for both Normal and Context Descriptors. */
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_EDSE_SHIFT  28
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_EDSE_MASK   0x10000000
#define IMX_ENET_QOS_DMA_CHX_TX_CTRL_EDSE(v)     (((v) << 28) & 0x10000000)

/**< IMX_ENET_QOS_DMA_CHX_RX_CTRL - DMA Channel 0 Receive Control */
typedef union IMX_ENET_QOS_DMA_CHX_RX_CTRL_union_t {
    UINT32 R;
    struct {
        UINT32 SR : 1;                 /**< Start or Stop Receive When this bit is set, the DMA tries to acquire the descriptor from the Receive list and processes the incoming packets. */
        UINT32 RBSZ_X_0 : 3;                 /**< Receive Buffer size Low RBSZ[13:0] is split into two fields RBSZ_13_y and RBSZ_x_0. */
        UINT32 RBSZ_13_Y : 11;                /**< Receive Buffer size High RBSZ[13:0] is split into two fields higher RBSZ_13_y and lower RBSZ_x_0. */
        UINT32 _unused15 : 1;                 /**< Reserved. */
        UINT32 RXPBL : 6;                 /**< Receive Programmable Burst Length These bits indicate the maximum number of beats to be transferred in one DMA block data transfer. */
        UINT32 _unused22 : 1;                 /**< Reserved. */
        UINT32 _unused23 : 1;                 /**< Reserved. */
        UINT32 _unused24 : 4;                 /**< Reserved. */
        UINT32 _unused28 : 3;                 /**< Reserved. */
        UINT32 RPF : 1;                 /**< Rx Packet Flush. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_RX_CTRL_t;

/**< Register DMA_CHX_RX_CTRL - DMA Channel 0 Receive Control */
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL             0x1108
/**< Field SR - Start or Stop Receive When this bit is set, the DMA tries to acquire the descriptor from the Receive list and processes the incoming packets. */
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_SR_SHIFT    0
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_SR_MASK     0x1
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_SR(v)       (((v) << 0) & 0x1)
/**< Field RBSZ_X_0 - Receive Buffer size Low RBSZ[13:0] is split into two fields RBSZ_13_y and RBSZ_x_0. */
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RBSZ_X_0_SHIFT 1
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RBSZ_X_0_MASK 0xE
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RBSZ_X_0(v) (((v) << 1) & 0xE)
/**< Field RBSZ_13_Y - Receive Buffer size High RBSZ[13:0] is split into two fields higher RBSZ_13_y and lower RBSZ_x_0. */
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RBSZ_13_Y_SHIFT 4
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RBSZ_13_Y_MASK 0x7FF0
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RBSZ_13_Y(v) (((v) << 4) & 0x7FF0)
/**< Field RXPBL - Receive Programmable Burst Length These bits indicate the maximum number of beats to be transferred in one DMA block data transfer. */
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RXPBL_SHIFT 16
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RXPBL_MASK  0x3F0000
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RXPBL(v)    (((v) << 16) & 0x3F0000)
/**< Field RPF - Rx Packet Flush. */
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RPF_SHIFT   31
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RPF_MASK    0x80000000
#define IMX_ENET_QOS_DMA_CHX_RX_CTRL_RPF(v)      (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_DMA_CHX_TXDESC_LIST_ADDR - Channel 0 Tx Descriptor List Address register */
typedef union IMX_ENET_QOS_DMA_CHX_TXDESC_LIST_ADDR_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 3;
        UINT32 TDESLA : 29;                /**< Start of Transmit List This field contains the base address of the first descriptor in the Transmit descriptor list. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_TXDESC_LIST_ADDR_t;

/**< Register DMA_CHX_TXDESC_LIST_ADDR - Channel 0 Tx Descriptor List Address register */
#define IMX_ENET_QOS_DMA_CHX_TXDESC_LIST_ADDR    0x1114
/**< Field TDESLA - Start of Transmit List This field contains the base address of the first descriptor in the Transmit descriptor list. */
#define IMX_ENET_QOS_DMA_CHX_TXDESC_LIST_ADDR_TDESLA_SHIFT 3
#define IMX_ENET_QOS_DMA_CHX_TXDESC_LIST_ADDR_TDESLA_MASK 0xFFFFFFF8
#define IMX_ENET_QOS_DMA_CHX_TXDESC_LIST_ADDR_TDESLA(v) (((v) << 3) & 0xFFFFFFF8)

/**< IMX_ENET_QOS_DMA_CHX_RXDESC_LIST_ADDR - Channel 0 Rx Descriptor List Address register */
typedef union IMX_ENET_QOS_DMA_CHX_RXDESC_LIST_ADDR_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 3;
        UINT32 RDESLA : 29;                /**< Start of Receive List This field contains the base address of the first descriptor in the Rx Descriptor list. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_RXDESC_LIST_ADDR_t;

/**< Register DMA_CHX_RXDESC_LIST_ADDR - Channel 0 Rx Descriptor List Address register */
#define IMX_ENET_QOS_DMA_CHX_RXDESC_LIST_ADDR    0x111C
/**< Field RDESLA - Start of Receive List This field contains the base address of the first descriptor in the Rx Descriptor list. */
#define IMX_ENET_QOS_DMA_CHX_RXDESC_LIST_ADDR_RDESLA_SHIFT 3
#define IMX_ENET_QOS_DMA_CHX_RXDESC_LIST_ADDR_RDESLA_MASK 0xFFFFFFF8
#define IMX_ENET_QOS_DMA_CHX_RXDESC_LIST_ADDR_RDESLA(v) (((v) << 3) & 0xFFFFFFF8)

/**< IMX_ENET_QOS_DMA_CHX_TXDESC_TAIL_PTR - Channel 0 Tx Descriptor Tail Pointer */
typedef union IMX_ENET_QOS_DMA_CHX_TXDESC_TAIL_PTR_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 3;
        UINT32 TDTP : 29;                /**< Transmit Descriptor Tail Pointer This field contains the tail pointer for the Tx descriptor ring. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_TXDESC_TAIL_PTR_t;

/**< Register DMA_CHX_TXDESC_TAIL_PTR - Channel 0 Tx Descriptor Tail Pointer */
#define IMX_ENET_QOS_DMA_CHX_TXDESC_TAIL_PTR     0x1120
/**< Field TDTP - Transmit Descriptor Tail Pointer This field contains the tail pointer for the Tx descriptor ring. */
#define IMX_ENET_QOS_DMA_CHX_TXDESC_TAIL_PTR_TDTP_SHIFT 3
#define IMX_ENET_QOS_DMA_CHX_TXDESC_TAIL_PTR_TDTP_MASK 0xFFFFFFF8
#define IMX_ENET_QOS_DMA_CHX_TXDESC_TAIL_PTR_TDTP(v) (((v) << 3) & 0xFFFFFFF8)

/**< IMX_ENET_QOS_DMA_CHX_RXDESC_TAIL_PTR - Channel 0 Rx Descriptor Tail Pointer */
typedef union IMX_ENET_QOS_DMA_CHX_RXDESC_TAIL_PTR_union_t {
    UINT32 R;
    struct {
        UINT32 _unused0 : 3;
        UINT32 RDTP : 29;                /**< Receive Descriptor Tail Pointer This field contains the tail pointer for the Rx descriptor ring. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_RXDESC_TAIL_PTR_t;

/**< Register DMA_CHX_RXDESC_TAIL_PTR - Channel 0 Rx Descriptor Tail Pointer */
#define IMX_ENET_QOS_DMA_CHX_RXDESC_TAIL_PTR     0x1128
/**< Field RDTP - Receive Descriptor Tail Pointer This field contains the tail pointer for the Rx descriptor ring. */
#define IMX_ENET_QOS_DMA_CHX_RXDESC_TAIL_PTR_RDTP_SHIFT 3
#define IMX_ENET_QOS_DMA_CHX_RXDESC_TAIL_PTR_RDTP_MASK 0xFFFFFFF8
#define IMX_ENET_QOS_DMA_CHX_RXDESC_TAIL_PTR_RDTP(v) (((v) << 3) & 0xFFFFFFF8)

/**< IMX_ENET_QOS_DMA_CHX_TXDESC_RING_LENGTH - Channel 0 Tx Descriptor Ring Length */
typedef union IMX_ENET_QOS_DMA_CHX_TXDESC_RING_LENGTH_union_t {
    UINT32 R;
    struct {
        UINT32 TDRL : 10;                /**< Transmit Descriptor Ring Length This field sets the maximum number of Tx descriptors in the circular descriptor ring. */
        UINT32 _unused10 : 22;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_TXDESC_RING_LENGTH_t;

/**< Register DMA_CHX_TXDESC_RING_LENGTH - Channel 0 Tx Descriptor Ring Length */
#define IMX_ENET_QOS_DMA_CHX_TXDESC_RING_LENGTH  0x112C
/**< Field TDRL - Transmit Descriptor Ring Length This field sets the maximum number of Tx descriptors in the circular descriptor ring. */
#define IMX_ENET_QOS_DMA_CHX_TXDESC_RING_LENGTH_TDRL_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_TXDESC_RING_LENGTH_TDRL_MASK 0x3FF
#define IMX_ENET_QOS_DMA_CHX_TXDESC_RING_LENGTH_TDRL(v) (((v) << 0) & 0x3FF)

/**< IMX_ENET_QOS_DMA_CHX_RXDESC_RING_LENGTH - Channel 0 Rx Descriptor Ring Length */
typedef union IMX_ENET_QOS_DMA_CHX_RXDESC_RING_LENGTH_union_t {
    UINT32 R;
    struct {
        UINT32 RDRL : 10;                /**< Receive Descriptor Ring Length This register sets the maximum number of Rx descriptors in the circular descriptor ring. */
        UINT32 _unused10 : 22;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_RXDESC_RING_LENGTH_t;

/**< Register DMA_CHX_RXDESC_RING_LENGTH - Channel 0 Rx Descriptor Ring Length */
#define IMX_ENET_QOS_DMA_CHX_RXDESC_RING_LENGTH  0x1130
/**< Field RDRL - Receive Descriptor Ring Length This register sets the maximum number of Rx descriptors in the circular descriptor ring. */
#define IMX_ENET_QOS_DMA_CHX_RXDESC_RING_LENGTH_RDRL_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_RXDESC_RING_LENGTH_RDRL_MASK 0x3FF
#define IMX_ENET_QOS_DMA_CHX_RXDESC_RING_LENGTH_RDRL(v) (((v) << 0) & 0x3FF)

/**< IMX_ENET_QOS_DMA_CHX_INT_EN - Channel 0 Interrupt Enable */
typedef union IMX_ENET_QOS_DMA_CHX_INT_EN_union_t {
    UINT32 R;
    struct {
        UINT32 TIE : 1;                 /**< Transmit Interrupt Enable When this bit is set along with the NIE bit, the Transmit Interrupt is enabled. */
        UINT32 TXSE : 1;                 /**< Transmit Stopped Enable When this bit is set along with the AIE bit, the Transmission Stopped interrupt is enabled. */
        UINT32 TBUE : 1;                 /**< Transmit Buffer Unavailable Enable When this bit is set along with the NIE bit, the Transmit Buffer Unavailable interrupt is enabled. */
        UINT32 _unused3 : 3;                 /**< Reserved. */
        UINT32 RIE : 1;                 /**< Receive Interrupt Enable When this bit is set along with the NIE bit, the Receive Interrupt is enabled. */
        UINT32 RBUE : 1;                 /**< Receive Buffer Unavailable Enable When this bit is set along with the AIE bit, the Receive Buffer Unavailable interrupt is enabled. */
        UINT32 RSE : 1;                 /**< Receive Stopped Enable When this bit is set along with the AIE bit, the Receive Stopped Interrupt is enabled. */
        UINT32 RWTE : 1;                 /**< Receive Watchdog Timeout Enable When this bit is set along with the AIE bit, the Receive Watchdog Timeout interrupt is enabled. */
        UINT32 ETIE : 1;                 /**< Early Transmit Interrupt Enable When this bit is set along with the AIE bit, the Early Transmit interrupt is enabled. */
        UINT32 ERIE : 1;                 /**< Early Receive Interrupt Enable When this bit is set along with the NIE bit, the Early Receive interrupt is enabled. */
        UINT32 FBEE : 1;                 /**< Fatal Bus Error Enable When this bit is set along with the AIE bit, the Fatal Bus error interrupt is enabled. */
        UINT32 CDEE : 1;                 /**< Context Descriptor Error Enable When this bit is set along with the AIE bit, the Descriptor error interrupt is enabled. */
        UINT32 AIE : 1;                 /**< Abnormal Interrupt Summary Enable When this bit is set, the abnormal interrupt summary is enabled. */
        UINT32 NIE : 1;                 /**< Normal Interrupt Summary Enable When this bit is set, the normal interrupt summary is enabled. */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_INT_EN_t;

/**< Register DMA_CHX_INT_EN - Channel 0 Interrupt Enable */
#define IMX_ENET_QOS_DMA_CHX_INT_EN              0x1134
/**< Field TIE - Transmit Interrupt Enable When this bit is set along with the NIE bit, the Transmit Interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_TIE_SHIFT    0
#define IMX_ENET_QOS_DMA_CHX_INT_EN_TIE_MASK     0x1
#define IMX_ENET_QOS_DMA_CHX_INT_EN_TIE(v)       (((v) << 0) & 0x1)
/**< Field TXSE - Transmit Stopped Enable When this bit is set along with the AIE bit, the Transmission Stopped interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_TXSE_SHIFT   1
#define IMX_ENET_QOS_DMA_CHX_INT_EN_TXSE_MASK    0x2
#define IMX_ENET_QOS_DMA_CHX_INT_EN_TXSE(v)      (((v) << 1) & 0x2)
/**< Field TBUE - Transmit Buffer Unavailable Enable When this bit is set along with the NIE bit, the Transmit Buffer Unavailable interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_TBUE_SHIFT   2
#define IMX_ENET_QOS_DMA_CHX_INT_EN_TBUE_MASK    0x4
#define IMX_ENET_QOS_DMA_CHX_INT_EN_TBUE(v)      (((v) << 2) & 0x4)
/**< Field RIE - Receive Interrupt Enable When this bit is set along with the NIE bit, the Receive Interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RIE_SHIFT    6
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RIE_MASK     0x40
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RIE(v)       (((v) << 6) & 0x40)
/**< Field RBUE - Receive Buffer Unavailable Enable When this bit is set along with the AIE bit, the Receive Buffer Unavailable interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RBUE_SHIFT   7
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RBUE_MASK    0x80
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RBUE(v)      (((v) << 7) & 0x80)
/**< Field RSE - Receive Stopped Enable When this bit is set along with the AIE bit, the Receive Stopped Interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RSE_SHIFT    8
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RSE_MASK     0x100
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RSE(v)       (((v) << 8) & 0x100)
/**< Field RWTE - Receive Watchdog Timeout Enable When this bit is set along with the AIE bit, the Receive Watchdog Timeout interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RWTE_SHIFT   9
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RWTE_MASK    0x200
#define IMX_ENET_QOS_DMA_CHX_INT_EN_RWTE(v)      (((v) << 9) & 0x200)
/**< Field ETIE - Early Transmit Interrupt Enable When this bit is set along with the AIE bit, the Early Transmit interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_ETIE_SHIFT   10
#define IMX_ENET_QOS_DMA_CHX_INT_EN_ETIE_MASK    0x400
#define IMX_ENET_QOS_DMA_CHX_INT_EN_ETIE(v)      (((v) << 10) & 0x400)
/**< Field ERIE - Early Receive Interrupt Enable When this bit is set along with the NIE bit, the Early Receive interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_ERIE_SHIFT   11
#define IMX_ENET_QOS_DMA_CHX_INT_EN_ERIE_MASK    0x800
#define IMX_ENET_QOS_DMA_CHX_INT_EN_ERIE(v)      (((v) << 11) & 0x800)
/**< Field FBEE - Fatal Bus Error Enable When this bit is set along with the AIE bit, the Fatal Bus error interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_FBEE_SHIFT   12
#define IMX_ENET_QOS_DMA_CHX_INT_EN_FBEE_MASK    0x1000
#define IMX_ENET_QOS_DMA_CHX_INT_EN_FBEE(v)      (((v) << 12) & 0x1000)
/**< Field CDEE - Context Descriptor Error Enable When this bit is set along with the AIE bit, the Descriptor error interrupt is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_CDEE_SHIFT   13
#define IMX_ENET_QOS_DMA_CHX_INT_EN_CDEE_MASK    0x2000
#define IMX_ENET_QOS_DMA_CHX_INT_EN_CDEE(v)      (((v) << 13) & 0x2000)
/**< Field AIE - Abnormal Interrupt Summary Enable When this bit is set, the abnormal interrupt summary is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_AIE_SHIFT    14
#define IMX_ENET_QOS_DMA_CHX_INT_EN_AIE_MASK     0x4000
#define IMX_ENET_QOS_DMA_CHX_INT_EN_AIE(v)       (((v) << 14) & 0x4000)
/**< Field NIE - Normal Interrupt Summary Enable When this bit is set, the normal interrupt summary is enabled. */
#define IMX_ENET_QOS_DMA_CHX_INT_EN_NIE_SHIFT    15
#define IMX_ENET_QOS_DMA_CHX_INT_EN_NIE_MASK     0x8000
#define IMX_ENET_QOS_DMA_CHX_INT_EN_NIE(v)       (((v) << 15) & 0x8000)

/**< IMX_ENET_QOS_DMA_CHX_RX_INT_WDTIMER - Channel 0 Receive Interrupt Watchdog Timer */
typedef union IMX_ENET_QOS_DMA_CHX_RX_INT_WDTIMER_union_t {
    UINT32 R;
    struct {
        UINT32 RWT : 8;                 /**< Receive Interrupt Watchdog Timer Count This field indicates the number of system clock cycles, multiplied by factor indicated in RWTU field, for which the watchdog timer is set. */
        UINT32 _unused8 : 8;                 /**< Reserved. */
        UINT32 RWTU : 2;                 /**< Receive Interrupt Watchdog Timer Count Units This fields indicates the number of system clock cycles corresponding to one unit in RWT field. */
        UINT32 _unused18 : 14;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_RX_INT_WDTIMER_t;

/**< Register DMA_CHX_RX_INT_WDTIMER - Channel 0 Receive Interrupt Watchdog Timer */
#define IMX_ENET_QOS_DMA_CHX_RX_INT_WDTIMER      0x1138
/**< Field RWT - Receive Interrupt Watchdog Timer Count This field indicates the number of system clock cycles, multiplied by factor indicated in RWTU field, for which the watchdog timer is set. */
#define IMX_ENET_QOS_DMA_CHX_RX_INT_WDTIMER_RWT_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_RX_INT_WDTIMER_RWT_MASK 0xFF
#define IMX_ENET_QOS_DMA_CHX_RX_INT_WDTIMER_RWT(v) (((v) << 0) & 0xFF)
/**< Field RWTU - Receive Interrupt Watchdog Timer Count Units This fields indicates the number of system clock cycles corresponding to one unit in RWT field. */
#define IMX_ENET_QOS_DMA_CHX_RX_INT_WDTIMER_RWTU_SHIFT 16
#define IMX_ENET_QOS_DMA_CHX_RX_INT_WDTIMER_RWTU_MASK 0x30000
#define IMX_ENET_QOS_DMA_CHX_RX_INT_WDTIMER_RWTU(v) (((v) << 16) & 0x30000)

/**< IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT - Channel 0 Slot Function Control and Status */
typedef union IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_union_t {
    UINT32 R;
    struct {
        UINT32 ESC : 1;                 /**< Enable Slot Comparison When set, this bit enables the checking of the slot numbers programmed in the Tx descriptor with the current reference given in the RSN field. */
        UINT32 ASC : 1;                 /**< Advance Slot Check When set, this bit enables the DMA to fetch the data from the buffer when the slot number (SLOTNUM) programmed in the Tx descriptor is - equal to the reference slot number given in the RSN field or - ahead of the reference slot number by up to two slots This bit is applicable only when the ESC bit is set. */
        UINT32 _unused2 : 2;                 /**< Reserved. */
        UINT32 SIV : 12;                /**< Slot Interval Value This field controls the period of the slot interval in which the TxDMA fetches the scheduled packets. */
        UINT32 RSN : 4;                 /**< Reference Slot Number This field gives the current value of the reference slot number in the DMA. */
        UINT32 _unused20 : 12;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_SLOT_FUNC_CTRL_STAT_t;

/**< Register DMA_CHX_SLOT_FUNC_CTRL_STAT - Channel 0 Slot Function Control and Status */
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT 0x113C
/**< Field ESC - Enable Slot Comparison When set, this bit enables the checking of the slot numbers programmed in the Tx descriptor with the current reference given in the RSN field. */
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_ESC_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_ESC_MASK 0x1
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_ESC(v) (((v) << 0) & 0x1)
/**< Field ASC - Advance Slot Check When set, this bit enables the DMA to fetch the data from the buffer when the slot number (SLOTNUM) programmed in the Tx descriptor is - equal to the reference slot number given in the RSN field or - ahead of the reference slot number by up to two slots This bit is applicable only when the ESC bit is set. */
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_ASC_SHIFT 1
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_ASC_MASK 0x2
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_ASC(v) (((v) << 1) & 0x2)
/**< Field SIV - Slot Interval Value This field controls the period of the slot interval in which the TxDMA fetches the scheduled packets. */
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_SIV_SHIFT 4
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_SIV_MASK 0xFFF0
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_SIV(v) (((v) << 4) & 0xFFF0)
/**< Field RSN - Reference Slot Number This field gives the current value of the reference slot number in the DMA. */
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_RSN_SHIFT 16
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_RSN_MASK 0xF0000
#define IMX_ENET_QOS_DMA_CHX_SLOT_FUNC_CTRL_STAT_RSN(v) (((v) << 16) & 0xF0000)

/**< IMX_ENET_QOS_DMA_CHX_CUR_APP_TXDESC - Channel 0 Current Application Transmit Descriptor */
typedef union IMX_ENET_QOS_DMA_CHX_CUR_APP_TXDESC_union_t {
    UINT32 R;
    struct {
        UINT32 CURTDESAPTR : 32;                /**< Application Transmit Descriptor Address Pointer The DMA updates this pointer during Tx operation. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_CUR_APP_TXDESC_t;

/**< Register DMA_CHX_CUR_APP_TXDESC - Channel 0 Current Application Transmit Descriptor */
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_TXDESC      0x1144
/**< Field CURTDESAPTR - Application Transmit Descriptor Address Pointer The DMA updates this pointer during Tx operation. */
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_TXDESC_CURTDESAPTR_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_TXDESC_CURTDESAPTR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_TXDESC_CURTDESAPTR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_DMA_CHX_CUR_APP_RXDESC - Channel 0 Current Application Receive Descriptor */
typedef union IMX_ENET_QOS_DMA_CHX_CUR_APP_RXDESC_union_t {
    UINT32 R;
    struct {
        UINT32 CURRDESAPTR : 32;                /**< Application Receive Descriptor Address Pointer The DMA updates this pointer during Rx operation. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_CUR_APP_RXDESC_t;

/**< Register DMA_CHX_CUR_APP_RXDESC - Channel 0 Current Application Receive Descriptor */
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_RXDESC      0x114C
/**< Field CURRDESAPTR - Application Receive Descriptor Address Pointer The DMA updates this pointer during Rx operation. */
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_RXDESC_CURRDESAPTR_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_RXDESC_CURRDESAPTR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_RXDESC_CURRDESAPTR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_DMA_CHX_CUR_APP_TXBUF - Channel 0 Current Application Transmit Buffer Address */
typedef union IMX_ENET_QOS_DMA_CHX_CUR_APP_TXBUF_union_t {
    UINT32 R;
    struct {
        UINT32 CURTBUFAPTR : 32;                /**< Application Transmit Buffer Address Pointer The DMA updates this pointer during Tx operation. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_CUR_APP_TXBUF_t;

/**< Register DMA_CHX_CUR_APP_TXBUF - Channel 0 Current Application Transmit Buffer Address */
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_TXBUF       0x1154
/**< Field CURTBUFAPTR - Application Transmit Buffer Address Pointer The DMA updates this pointer during Tx operation. */
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_TXBUF_CURTBUFAPTR_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_TXBUF_CURTBUFAPTR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_TXBUF_CURTBUFAPTR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_DMA_CHX_CUR_APP_RXBUF - Channel 0 Current Application Receive Buffer Address */
typedef union IMX_ENET_QOS_DMA_CHX_CUR_APP_RXBUF_union_t {
    UINT32 R;
    struct {
        UINT32 CURRBUFAPTR : 32;                /**< Application Receive Buffer Address Pointer The DMA updates this pointer during Rx operation. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_CUR_APP_RXBUF_t;

/**< Register DMA_CHX_CUR_APP_RXBUF - Channel 0 Current Application Receive Buffer Address */
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_RXBUF       0x115C
/**< Field CURRBUFAPTR - Application Receive Buffer Address Pointer The DMA updates this pointer during Rx operation. */
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_RXBUF_CURRBUFAPTR_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_RXBUF_CURRBUFAPTR_MASK 0xFFFFFFFF
#define IMX_ENET_QOS_DMA_CHX_CUR_APP_RXBUF_CURRBUFAPTR(v) (((v) << 0) & 0xFFFFFFFF)

/**< IMX_ENET_QOS_DMA_CHX_STAT - DMA Channel 0 Status */
typedef union IMX_ENET_QOS_DMA_CHX_STAT_union_t {
    UINT32 R;
    struct {
        UINT32 TI : 1;                 /**< Transmit Interrupt This bit indicates that the packet transmission is complete. */
        UINT32 TPS : 1;                 /**< Transmit Process Stopped This bit is set when the transmission is stopped. */
        UINT32 TBU : 1;                 /**< Transmit Buffer Unavailable This bit indicates that the application owns the next descriptor in the Transmit list, and the DMA cannot acquire it. */
        UINT32 _unused3 : 3;                 /**< Reserved. */
        UINT32 RI : 1;                 /**< Receive Interrupt This bit indicates that the packet reception is complete. */
        UINT32 RBU : 1;                 /**< Receive Buffer Unavailable This bit indicates that the application owns the next descriptor in the Receive list, and the DMA cannot acquire it. */
        UINT32 RPS : 1;                 /**< Receive Process Stopped This bit is asserted when the Rx process enters the Stopped state. */
        UINT32 RWT : 1;                 /**< Receive Watchdog Timeout This bit is asserted when a packet with length greater than 2,048 bytes (10,240 bytes when Jumbo Packet mode is enabled) is received. */
        UINT32 ETI : 1;                 /**< Early Transmit Interrupt This bit when set indicates that the TxDMA has completed the transfer of packet data to the MTL TXFIFO memory. */
        UINT32 ERI : 1;                 /**< Early Receive Interrupt This bit when set indicates that the RxDMA has completed the transfer of packet data to the memory. */
        UINT32 FBE : 1;                 /**< Fatal Bus Error This bit indicates that a bus error occurred (as described in the EB field). */
        UINT32 CDE : 1;                 /**< Context Descriptor Error This bit indicates that the DMA Tx/Rx engine received a descriptor error, which indicates invalid context in the middle of packet flow (intermediate descriptor) or all ones descriptor in Tx case and on Rx side it indicates DMA has read a descriptor with either of the buffer address as ones which is considered to be invalid. */
        UINT32 AIS : 1;                 /**< Abnormal Interrupt Summary Abnormal Interrupt Summary bit value is the logical OR of the following when the corresponding interrupt bits are enabled in the INTERRUPT_ENABLE register: - Bit 1: Transmit Process Stopped - Bit 7: Receive Buffer Unavailable - Bit 8: Receive Process Stopped - Bit 10: Early Transmit Interrupt - Bit 12: Fatal Bus Error - Bit 13: Context Descriptor Error Only unmasked bits affect the Abnormal Interrupt Summary bit. */
        UINT32 NIS : 1;                 /**< Normal Interrupt Summary Normal Interrupt Summary bit value is the logical OR of the following bits when the corresponding interrupt bits are enabled in the DMA_CH0_INTERRUPT_ENABLE register: - Bit 0: Transmit Interrupt - Bit 2: Transmit Buffer Unavailable - Bit 6: Receive Interrupt - Bit 11: Early Receive Interrupt Only unmasked bits (interrupts for which interrupt enable is set in INTERRUPT_ENABLE register) affect the Normal Interrupt Summary bit. */
        UINT32 TEB : 3;                 /**< Tx DMA Error Bits This field indicates the type of error that caused a Bus Error. */
        UINT32 REB : 3;                 /**< Rx DMA Error Bits This field indicates the type of error that caused a Bus Error. */
        UINT32 _unused22 : 10;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_STAT_t;

/**< Register DMA_CHX_STAT - DMA Channel 0 Status */
#define IMX_ENET_QOS_DMA_CHX_STAT                0x1160
/**< Field TI - Transmit Interrupt This bit indicates that the packet transmission is complete. */
#define IMX_ENET_QOS_DMA_CHX_STAT_TI_SHIFT       0
#define IMX_ENET_QOS_DMA_CHX_STAT_TI_MASK        0x1
#define IMX_ENET_QOS_DMA_CHX_STAT_TI(v)          (((v) << 0) & 0x1)
/**< Field TPS - Transmit Process Stopped This bit is set when the transmission is stopped. */
#define IMX_ENET_QOS_DMA_CHX_STAT_TPS_SHIFT      1
#define IMX_ENET_QOS_DMA_CHX_STAT_TPS_MASK       0x2
#define IMX_ENET_QOS_DMA_CHX_STAT_TPS(v)         (((v) << 1) & 0x2)
/**< Field TBU - Transmit Buffer Unavailable This bit indicates that the application owns the next descriptor in the Transmit list, and the DMA cannot acquire it. */
#define IMX_ENET_QOS_DMA_CHX_STAT_TBU_SHIFT      2
#define IMX_ENET_QOS_DMA_CHX_STAT_TBU_MASK       0x4
#define IMX_ENET_QOS_DMA_CHX_STAT_TBU(v)         (((v) << 2) & 0x4)
/**< Field RI - Receive Interrupt This bit indicates that the packet reception is complete. */
#define IMX_ENET_QOS_DMA_CHX_STAT_RI_SHIFT       6
#define IMX_ENET_QOS_DMA_CHX_STAT_RI_MASK        0x40
#define IMX_ENET_QOS_DMA_CHX_STAT_RI(v)          (((v) << 6) & 0x40)
/**< Field RBU - Receive Buffer Unavailable This bit indicates that the application owns the next descriptor in the Receive list, and the DMA cannot acquire it. */
#define IMX_ENET_QOS_DMA_CHX_STAT_RBU_SHIFT      7
#define IMX_ENET_QOS_DMA_CHX_STAT_RBU_MASK       0x80
#define IMX_ENET_QOS_DMA_CHX_STAT_RBU(v)         (((v) << 7) & 0x80)
/**< Field RPS - Receive Process Stopped This bit is asserted when the Rx process enters the Stopped state. */
#define IMX_ENET_QOS_DMA_CHX_STAT_RPS_SHIFT      8
#define IMX_ENET_QOS_DMA_CHX_STAT_RPS_MASK       0x100
#define IMX_ENET_QOS_DMA_CHX_STAT_RPS(v)         (((v) << 8) & 0x100)
/**< Field RWT - Receive Watchdog Timeout This bit is asserted when a packet with length greater than 2,048 bytes (10,240 bytes when Jumbo Packet mode is enabled) is received. */
#define IMX_ENET_QOS_DMA_CHX_STAT_RWT_SHIFT      9
#define IMX_ENET_QOS_DMA_CHX_STAT_RWT_MASK       0x200
#define IMX_ENET_QOS_DMA_CHX_STAT_RWT(v)         (((v) << 9) & 0x200)
/**< Field ETI - Early Transmit Interrupt This bit when set indicates that the TxDMA has completed the transfer of packet data to the MTL TXFIFO memory. */
#define IMX_ENET_QOS_DMA_CHX_STAT_ETI_SHIFT      10
#define IMX_ENET_QOS_DMA_CHX_STAT_ETI_MASK       0x400
#define IMX_ENET_QOS_DMA_CHX_STAT_ETI(v)         (((v) << 10) & 0x400)
/**< Field ERI - Early Receive Interrupt This bit when set indicates that the RxDMA has completed the transfer of packet data to the memory. */
#define IMX_ENET_QOS_DMA_CHX_STAT_ERI_SHIFT      11
#define IMX_ENET_QOS_DMA_CHX_STAT_ERI_MASK       0x800
#define IMX_ENET_QOS_DMA_CHX_STAT_ERI(v)         (((v) << 11) & 0x800)
/**< Field FBE - Fatal Bus Error This bit indicates that a bus error occurred (as described in the EB field). */
#define IMX_ENET_QOS_DMA_CHX_STAT_FBE_SHIFT      12
#define IMX_ENET_QOS_DMA_CHX_STAT_FBE_MASK       0x1000
#define IMX_ENET_QOS_DMA_CHX_STAT_FBE(v)         (((v) << 12) & 0x1000)
/**< Field CDE - Context Descriptor Error This bit indicates that the DMA Tx/Rx engine received a descriptor error, which indicates invalid context in the middle of packet flow (intermediate descriptor) or all ones descriptor in Tx case and on Rx side it indicates DMA has read a descriptor with either of the buffer address as ones which is considered to be invalid. */
#define IMX_ENET_QOS_DMA_CHX_STAT_CDE_SHIFT      13
#define IMX_ENET_QOS_DMA_CHX_STAT_CDE_MASK       0x2000
#define IMX_ENET_QOS_DMA_CHX_STAT_CDE(v)         (((v) << 13) & 0x2000)
/**< Field AIS - Abnormal Interrupt Summary Abnormal Interrupt Summary bit value is the logical OR of the following when the corresponding interrupt bits are enabled in the INTERRUPT_ENABLE register: - Bit 1: Transmit Process Stopped - Bit 7: Receive Buffer Unavailable - Bit 8: Receive Process Stopped - Bit 10: Early Transmit Interrupt - Bit 12: Fatal Bus Error - Bit 13: Context Descriptor Error Only unmasked bits affect the Abnormal Interrupt Summary bit. */
#define IMX_ENET_QOS_DMA_CHX_STAT_AIS_SHIFT      14
#define IMX_ENET_QOS_DMA_CHX_STAT_AIS_MASK       0x4000
#define IMX_ENET_QOS_DMA_CHX_STAT_AIS(v)         (((v) << 14) & 0x4000)
/**< Field NIS - Normal Interrupt Summary Normal Interrupt Summary bit value is the logical OR of the following bits when the corresponding interrupt bits are enabled in the DMA_CH0_INTERRUPT_ENABLE register: - Bit 0: Transmit Interrupt - Bit 2: Transmit Buffer Unavailable - Bit 6: Receive Interrupt - Bit 11: Early Receive Interrupt Only unmasked bits (interrupts for which interrupt enable is set in INTERRUPT_ENABLE register) affect the Normal Interrupt Summary bit. */
#define IMX_ENET_QOS_DMA_CHX_STAT_NIS_SHIFT      15
#define IMX_ENET_QOS_DMA_CHX_STAT_NIS_MASK       0x8000
#define IMX_ENET_QOS_DMA_CHX_STAT_NIS(v)         (((v) << 15) & 0x8000)
/**< Field TEB - Tx DMA Error Bits This field indicates the type of error that caused a Bus Error. */
#define IMX_ENET_QOS_DMA_CHX_STAT_TEB_SHIFT      16
#define IMX_ENET_QOS_DMA_CHX_STAT_TEB_MASK       0x70000
#define IMX_ENET_QOS_DMA_CHX_STAT_TEB(v)         (((v) << 16) & 0x70000)
/**< Field REB - Rx DMA Error Bits This field indicates the type of error that caused a Bus Error. */
#define IMX_ENET_QOS_DMA_CHX_STAT_REB_SHIFT      19
#define IMX_ENET_QOS_DMA_CHX_STAT_REB_MASK       0x380000
#define IMX_ENET_QOS_DMA_CHX_STAT_REB(v)         (((v) << 19) & 0x380000)

/**< IMX_ENET_QOS_DMA_CHX_MISS_FRAME_CNT - Channel 0 Missed Frame Counter */
typedef union IMX_ENET_QOS_DMA_CHX_MISS_FRAME_CNT_union_t {
    UINT32 R;
    struct {
        UINT32 MFC : 11;                /**< Dropped Packet Counters This counter indicates the number of packet counters that are dropped by the DMA either because of bus error or because of programming RPF field in DMA_CH0_RX_CONTROL register. */
        UINT32 _unused11 : 4;                 /**< Reserved. */
        UINT32 MFCO : 1;                 /**< Overflow status of the MFC Counter When this bit is set then the MFC counter does not get incremented further. */
        UINT32 _unused16 : 16;                /**< Reserved. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_MISS_FRAME_CNT_t;

/**< Register DMA_CHX_MISS_FRAME_CNT - Channel 0 Missed Frame Counter */
#define IMX_ENET_QOS_DMA_CHX_MISS_FRAME_CNT      0x1164
/**< Field MFC - Dropped Packet Counters This counter indicates the number of packet counters that are dropped by the DMA either because of bus error or because of programming RPF field in DMA_CH0_RX_CONTROL register. */
#define IMX_ENET_QOS_DMA_CHX_MISS_FRAME_CNT_MFC_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_MISS_FRAME_CNT_MFC_MASK 0x7FF
#define IMX_ENET_QOS_DMA_CHX_MISS_FRAME_CNT_MFC(v) (((v) << 0) & 0x7FF)
/**< Field MFCO - Overflow status of the MFC Counter When this bit is set then the MFC counter does not get incremented further. */
#define IMX_ENET_QOS_DMA_CHX_MISS_FRAME_CNT_MFCO_SHIFT 15
#define IMX_ENET_QOS_DMA_CHX_MISS_FRAME_CNT_MFCO_MASK 0x8000
#define IMX_ENET_QOS_DMA_CHX_MISS_FRAME_CNT_MFCO(v) (((v) << 15) & 0x8000)

/**< IMX_ENET_QOS_DMA_CHX_RXP_ACCEPT_CNT - Channel 0 RXP Frames Accepted Counter */
typedef union IMX_ENET_QOS_DMA_CHX_RXP_ACCEPT_CNT_union_t {
    UINT32 R;
    struct {
        UINT32 RXPAC : 31;                /**< Rx Parser Accept Counter This 31-bit counter is implemented whenever a Rx Parser Accept a packet due to AF =1. */
        UINT32 RXPACOF : 1;                 /**< Rx Parser Accept Counter Overflow Bit When set, this bit indicates that the RXPAC Counter field crossed the maximum limit. */
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_RXP_ACCEPT_CNT_t;

/**< Register DMA_CHX_RXP_ACCEPT_CNT - Channel 0 RXP Frames Accepted Counter */
#define IMX_ENET_QOS_DMA_CHX_RXP_ACCEPT_CNT      0x1168
/**< Field RXPAC - Rx Parser Accept Counter This 31-bit counter is implemented whenever a Rx Parser Accept a packet due to AF =1. */
#define IMX_ENET_QOS_DMA_CHX_RXP_ACCEPT_CNT_RXPAC_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_RXP_ACCEPT_CNT_RXPAC_MASK 0x7FFFFFFF
#define IMX_ENET_QOS_DMA_CHX_RXP_ACCEPT_CNT_RXPAC(v) (((v) << 0) & 0x7FFFFFFF)
/**< Field RXPACOF - Rx Parser Accept Counter Overflow Bit When set, this bit indicates that the RXPAC Counter field crossed the maximum limit. */
#define IMX_ENET_QOS_DMA_CHX_RXP_ACCEPT_CNT_RXPACOF_SHIFT 31
#define IMX_ENET_QOS_DMA_CHX_RXP_ACCEPT_CNT_RXPACOF_MASK 0x80000000
#define IMX_ENET_QOS_DMA_CHX_RXP_ACCEPT_CNT_RXPACOF(v) (((v) << 31) & 0x80000000)

/**< IMX_ENET_QOS_DMA_CHX_RX_ERI_CNT - Channel 0 Receive ERI Counter */
typedef union IMX_ENET_QOS_DMA_CHX_RX_ERI_CNT_union_t {
    UINT32 R;
    struct {
        UINT32 ECNT : 12;                /**< ERI Counter When ERIC bit of RX_CONTROL register is set, this counter increments for burst transfer completed by the Rx DMA from the start of packet transfer. */
        UINT32 _unused12 : 20;
    } B;
} IMX_ENET_QOS_DMA_CH_DMA_CHX_RX_ERI_CNT_t;

/**< Register DMA_CHX_RX_ERI_CNT - Channel 0 Receive ERI Counter */
#define IMX_ENET_QOS_DMA_CHX_RX_ERI_CNT          0x116C
/**< Field ECNT - ERI Counter When ERIC bit of RX_CONTROL register is set, this counter increments for burst transfer completed by the Rx DMA from the start of packet transfer. */
#define IMX_ENET_QOS_DMA_CHX_RX_ERI_CNT_ECNT_SHIFT 0
#define IMX_ENET_QOS_DMA_CHX_RX_ERI_CNT_ECNT_MASK 0xFFF
#define IMX_ENET_QOS_DMA_CHX_RX_ERI_CNT_ECNT(v)  (((v) << 0) & 0xFFF)

typedef struct IMX_ENET_QOS_DMA_CH_struct_t {
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_CTRL_t DMA_CHX_CTRL;  /**< DMA Channel 0 Control */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_TX_CTRL_t DMA_CHX_TX_CTRL;  /**< DMA Channel 0 Transmit Control */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_RX_CTRL_t DMA_CHX_RX_CTRL;  /**< DMA Channel 0 Receive Control */
    volatile UINT8 DMA_CH_reserved0[8];
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_TXDESC_LIST_ADDR_t DMA_CHX_TXDESC_LIST_ADDR;  /**< Channel 0 Tx Descriptor List Address register */
    volatile UINT8 DMA_CH_reserved1[4];
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_RXDESC_LIST_ADDR_t DMA_CHX_RXDESC_LIST_ADDR;  /**< Channel 0 Rx Descriptor List Address register */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_TXDESC_TAIL_PTR_t DMA_CHX_TXDESC_TAIL_PTR;  /**< Channel 0 Tx Descriptor Tail Pointer */
    volatile UINT8 DMA_CH_reserved2[4];
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_RXDESC_TAIL_PTR_t DMA_CHX_RXDESC_TAIL_PTR;  /**< Channel 0 Rx Descriptor Tail Pointer */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_TXDESC_RING_LENGTH_t DMA_CHX_TXDESC_RING_LENGTH;  /**< Channel 0 Tx Descriptor Ring Length */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_RXDESC_RING_LENGTH_t DMA_CHX_RXDESC_RING_LENGTH;  /**< Channel 0 Rx Descriptor Ring Length */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_INT_EN_t DMA_CHX_INT_EN;  /**< Channel 0 Interrupt Enable */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_RX_INT_WDTIMER_t DMA_CHX_RX_INT_WDTIMER;  /**< Channel 0 Receive Interrupt Watchdog Timer */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_SLOT_FUNC_CTRL_STAT_t DMA_CHX_SLOT_FUNC_CTRL_STAT;  /**< Channel 0 Slot Function Control and Status */
    volatile UINT8 DMA_CH_reserved3[4];
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_CUR_APP_TXDESC_t DMA_CHX_CUR_APP_TXDESC;  /**< Channel 0 Current Application Transmit Descriptor */
    volatile UINT8 DMA_CH_reserved4[4];
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_CUR_APP_RXDESC_t DMA_CHX_CUR_APP_RXDESC;  /**< Channel 0 Current Application Receive Descriptor */
    volatile UINT8 DMA_CH_reserved5[4];
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_CUR_APP_TXBUF_t DMA_CHX_CUR_APP_TXBUF;  /**< Channel 0 Current Application Transmit Buffer Address */
    volatile UINT8 DMA_CH_reserved6[4];
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_CUR_APP_RXBUF_t DMA_CHX_CUR_APP_RXBUF;  /**< Channel 0 Current Application Receive Buffer Address */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_STAT_t DMA_CHX_STAT;  /**< DMA Channel 0 Status */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_MISS_FRAME_CNT_t DMA_CHX_MISS_FRAME_CNT;  /**< Channel 0 Missed Frame Counter */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_RXP_ACCEPT_CNT_t DMA_CHX_RXP_ACCEPT_CNT;  /**< Channel 0 RXP Frames Accepted Counter */
    volatile IMX_ENET_QOS_DMA_CH_DMA_CHX_RX_ERI_CNT_t DMA_CHX_RX_ERI_CNT;  /**< Channel 0 Receive ERI Counter */
    volatile UINT8 DMA_CH_reserved7[16];
} IMX_ENET_QOS_DMA_CH_t;

/**< ENET_QOS peripheral register structure */
typedef struct IMX_ENET_QOS_struct_t {
    volatile IMX_ENET_QOS_MAC_CONFIGURATION_t MAC_CONFIGURATION;  /**< MAC Configuration Register */
    volatile IMX_ENET_QOS_MAC_EXT_CONFIGURATION_t MAC_EXT_CONFIGURATION;  /**< MAC Extended Configuration Register */
    volatile IMX_ENET_QOS_MAC_PACKET_FILTER_t MAC_PACKET_FILTER;  /**< MAC Packet Filter */
    volatile IMX_ENET_QOS_MAC_WATCHDOG_TIMEOUT_t MAC_WATCHDOG_TIMEOUT;  /**< Watchdog Timeout */
    volatile IMX_ENET_QOS_MAC_HASH_TABLE_REG0_t MAC_HASH_TABLE_REG0;  /**< MAC Hash Table Register 0 */
    volatile IMX_ENET_QOS_MAC_HASH_TABLE_REG1_t MAC_HASH_TABLE_REG1;  /**< MAC Hash Table Register 1 */
    volatile UINT8 _reserved0[56];
    volatile IMX_ENET_QOS_MAC_VLAN_TAG_CTRL_t MAC_VLAN_TAG_CTRL;  /**< MAC VLAN Tag Control */
    volatile IMX_ENET_QOS_MAC_VLAN_TAG_DATA_t MAC_VLAN_TAG_DATA;  /**< MAC VLAN Tag Data */
    volatile IMX_ENET_QOS_MAC_VLAN_HASH_TABLE_t MAC_VLAN_HASH_TABLE;  /**< MAC VLAN Hash Table */
    volatile UINT8 _reserved1[4];
    volatile IMX_ENET_QOS_MAC_VLAN_INCL_t MAC_VLAN_INCL;    /**< VLAN Tag Inclusion or Replacement */
    volatile IMX_ENET_QOS_MAC_INNER_VLAN_INCL_t MAC_INNER_VLAN_INCL;  /**< MAC Inner VLAN Tag Inclusion or Replacement */
    volatile UINT8 _reserved2[8];
    volatile IMX_ENET_QOS_MAC_TX_FLOW_CTRL_Q_t MAC_TX_FLOW_CTRL_Q[5];  /**< MAC Q0 Tx Flow Control */
    volatile UINT8 _reserved3[12];
    volatile IMX_ENET_QOS_MAC_RX_FLOW_CTRL_t MAC_RX_FLOW_CTRL;  /**< MAC Rx Flow Control */
    volatile IMX_ENET_QOS_MAC_RXQ_CTRL4_t MAC_RXQ_CTRL4;    /**< Receive Queue Control 4 */
    volatile IMX_ENET_QOS_MAC_TXQ_PRTY_MAP0_t MAC_TXQ_PRTY_MAP0;  /**< Transmit Queue Priority Mapping 0 */
    volatile IMX_ENET_QOS_MAC_TXQ_PRTY_MAP1_t MAC_TXQ_PRTY_MAP1;  /**< Transmit Queue Priority Mapping 1 */
    volatile IMX_ENET_QOS_MAC_RXQ_CTRL_t MAC_RXQ_CTRL[4];   /**< Receive Queue Control 0 */
    volatile IMX_ENET_QOS_MAC_INTERRUPT_STATUS_t MAC_INTERRUPT_STATUS;  /**< Interrupt Status */
    volatile IMX_ENET_QOS_MAC_INTERRUPT_ENABLE_t MAC_INTERRUPT_ENABLE;  /**< Interrupt Enable */
    volatile IMX_ENET_QOS_MAC_RX_TX_STATUS_t MAC_RX_TX_STATUS;  /**< Receive Transmit Status */
    volatile UINT8 _reserved4[4];
    volatile IMX_ENET_QOS_MAC_PMT_CONTROL_STATUS_t MAC_PMT_CONTROL_STATUS;  /**< PMT Control and Status */
    volatile IMX_ENET_QOS_MAC_RWK_PACKET_FILTER_t MAC_RWK_PACKET_FILTER;  /**< Remote Wakeup Filter */
    volatile UINT8 _reserved5[8];
    volatile IMX_ENET_QOS_MAC_LPI_CONTROL_STATUS_t MAC_LPI_CONTROL_STATUS;  /**< LPI Control and Status */
    volatile IMX_ENET_QOS_MAC_LPI_TIMERS_CONTROL_t MAC_LPI_TIMERS_CONTROL;  /**< LPI Timers Control */
    volatile IMX_ENET_QOS_MAC_LPI_ENTRY_TIMER_t MAC_LPI_ENTRY_TIMER;  /**< Tx LPI Entry Timer Control */
    volatile IMX_ENET_QOS_MAC_ONEUS_TIC_COUNTER_t MAC_ONEUS_TIC_COUNTER;  /**< One-microsecond Reference Timer */
    volatile UINT8 _reserved6[24];
    volatile IMX_ENET_QOS_MAC_PHYIF_CONTROL_STATUS_t MAC_PHYIF_CONTROL_STATUS;  /**< PHY Interface Control and Status */
    volatile UINT8 _reserved7[20];
    volatile IMX_ENET_QOS_MAC_VERSION_t MAC_VERSION;        /**< MAC Version */
    volatile IMX_ENET_QOS_MAC_DEBUG_t MAC_DEBUG;            /**< MAC Debug */
    volatile UINT8 _reserved8[4];
    volatile IMX_ENET_QOS_MAC_HW_FEAT_t MAC_HW_FEAT[4];     /**< Optional Features or Functions 0 */
    volatile UINT8 _reserved9[212];
    volatile IMX_ENET_QOS_MAC_MDIO_ADDRESS_t MAC_MDIO_ADDRESS;  /**< MDIO Address */
    volatile IMX_ENET_QOS_MAC_MDIO_DATA_t MAC_MDIO_DATA;    /**< MAC MDIO Data */
    volatile UINT8 _reserved10[40];
    volatile IMX_ENET_QOS_MAC_CSR_SW_CTRL_t MAC_CSR_SW_CTRL;  /**< CSR Software Control */
    volatile IMX_ENET_QOS_MAC_FPE_CTRL_STS_t MAC_FPE_CTRL_STS;  /**< Frame Preemption Control */
    volatile UINT8 _reserved11[8];
    volatile IMX_ENET_QOS_MAC_PRESN_TIME_NS_t MAC_PRESN_TIME_NS;  /**< 32-bit Binary Rollover Equivalent Time */
    volatile IMX_ENET_QOS_MAC_PRESN_TIME_UPDT_t MAC_PRESN_TIME_UPDT;  /**< MAC 1722 Presentation Time */
    volatile UINT8 _reserved12[184];
    volatile IMX_ENET_QOS_MAC_ADDRESS_t MAC_ADDRESS[64];
    volatile UINT8 _reserved13[512];
    volatile IMX_ENET_QOS_MAC_MMC_CONTROL_t MAC_MMC_CONTROL;  /**< MMC Control */
    volatile IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_t MAC_MMC_RX_INTERRUPT;  /**< MMC Rx Interrupt */
    volatile IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_t MAC_MMC_TX_INTERRUPT;  /**< MMC Tx Interrupt */
    volatile IMX_ENET_QOS_MAC_MMC_RX_INTERRUPT_MASK_t MAC_MMC_RX_INTERRUPT_MASK;  /**< MMC Rx Interrupt Mask */
    volatile IMX_ENET_QOS_MAC_MMC_TX_INTERRUPT_MASK_t MAC_MMC_TX_INTERRUPT_MASK;  /**< MMC Tx Interrupt Mask */
    volatile IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_BAD_t MAC_TX_OCTET_COUNT_GOOD_BAD;  /**< Tx Octet Count Good and Bad */
    volatile IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_BAD_t MAC_TX_PACKET_COUNT_GOOD_BAD;  /**< Tx Packet Count Good and Bad */
    volatile IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_t MAC_TX_BROADCAST_PACKETS_GOOD;  /**< Tx Broadcast Packets Good */
    volatile IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_t MAC_TX_MULTICAST_PACKETS_GOOD;  /**< Tx Multicast Packets Good */
    volatile IMX_ENET_QOS_MAC_TX_64OCTETS_PACKETS_GOOD_BAD_t MAC_TX_64OCTETS_PACKETS_GOOD_BAD;  /**< Tx Good and Bad 64-Byte Packets */
    volatile IMX_ENET_QOS_MAC_TX_65TO127OCTETS_PACKETS_GOOD_BAD_t MAC_TX_65TO127OCTETS_PACKETS_GOOD_BAD;  /**< Tx Good and Bad 65 to 127-Byte Packets */
    volatile IMX_ENET_QOS_MAC_TX_128TO255OCTETS_PACKETS_GOOD_BAD_t MAC_TX_128TO255OCTETS_PACKETS_GOOD_BAD;  /**< Tx Good and Bad 128 to 255-Byte Packets */
    volatile IMX_ENET_QOS_MAC_TX_256TO511OCTETS_PACKETS_GOOD_BAD_t MAC_TX_256TO511OCTETS_PACKETS_GOOD_BAD;  /**< Tx Good and Bad 256 to 511-Byte Packets */
    volatile IMX_ENET_QOS_MAC_TX_512TO1023OCTETS_PACKETS_GOOD_BAD_t MAC_TX_512TO1023OCTETS_PACKETS_GOOD_BAD;  /**< Tx Good and Bad 512 to 1023-Byte Packets */
    volatile IMX_ENET_QOS_MAC_TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_t MAC_TX_1024TOMAXOCTETS_PACKETS_GOOD_BAD;  /**< Tx Good and Bad 1024 to Max-Byte Packets */
    volatile IMX_ENET_QOS_MAC_TX_UNICAST_PACKETS_GOOD_BAD_t MAC_TX_UNICAST_PACKETS_GOOD_BAD;  /**< Good and Bad Unicast Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_MULTICAST_PACKETS_GOOD_BAD_t MAC_TX_MULTICAST_PACKETS_GOOD_BAD;  /**< Good and Bad Multicast Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_BROADCAST_PACKETS_GOOD_BAD_t MAC_TX_BROADCAST_PACKETS_GOOD_BAD;  /**< Good and Bad Broadcast Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_UNDERFLOW_ERROR_PACKETS_t MAC_TX_UNDERFLOW_ERROR_PACKETS;  /**< Tx Packets Aborted By Underflow Error */
    volatile IMX_ENET_QOS_MAC_TX_SINGLE_COLLISION_GOOD_PACKETS_t MAC_TX_SINGLE_COLLISION_GOOD_PACKETS;  /**< Single Collision Good Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS_t MAC_TX_MULTIPLE_COLLISION_GOOD_PACKETS;  /**< Multiple Collision Good Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_DEFERRED_PACKETS_t MAC_TX_DEFERRED_PACKETS;  /**< Deferred Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_LATE_COLLISION_PACKETS_t MAC_TX_LATE_COLLISION_PACKETS;  /**< Late Collision Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_EXCESSIVE_COLLISION_PACKETS_t MAC_TX_EXCESSIVE_COLLISION_PACKETS;  /**< Excessive Collision Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_CARRIER_ERROR_PACKETS_t MAC_TX_CARRIER_ERROR_PACKETS;  /**< Carrier Error Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_OCTET_COUNT_GOOD_t MAC_TX_OCTET_COUNT_GOOD;  /**< Bytes Transmitted in Good Packets */
    volatile IMX_ENET_QOS_MAC_TX_PACKET_COUNT_GOOD_t MAC_TX_PACKET_COUNT_GOOD;  /**< Good Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_EXCESSIVE_DEFERRAL_ERROR_t MAC_TX_EXCESSIVE_DEFERRAL_ERROR;  /**< Packets Aborted By Excessive Deferral Error */
    volatile IMX_ENET_QOS_MAC_TX_PAUSE_PACKETS_t MAC_TX_PAUSE_PACKETS;  /**< Pause Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_VLAN_PACKETS_GOOD_t MAC_TX_VLAN_PACKETS_GOOD;  /**< Good VLAN Packets Transmitted */
    volatile IMX_ENET_QOS_MAC_TX_OSIZE_PACKETS_GOOD_t MAC_TX_OSIZE_PACKETS_GOOD;  /**< Good Oversize Packets Transmitted */
    volatile UINT8 _reserved14[4];
    volatile IMX_ENET_QOS_MAC_RX_PACKETS_COUNT_GOOD_BAD_t MAC_RX_PACKETS_COUNT_GOOD_BAD;  /**< Good and Bad Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_BAD_t MAC_RX_OCTET_COUNT_GOOD_BAD;  /**< Bytes in Good and Bad Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_OCTET_COUNT_GOOD_t MAC_RX_OCTET_COUNT_GOOD;  /**< Bytes in Good Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_BROADCAST_PACKETS_GOOD_t MAC_RX_BROADCAST_PACKETS_GOOD;  /**< Good Broadcast Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_MULTICAST_PACKETS_GOOD_t MAC_RX_MULTICAST_PACKETS_GOOD;  /**< Good Multicast Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_CRC_ERROR_PACKETS_t MAC_RX_CRC_ERROR_PACKETS;  /**< CRC Error Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_ALIGNMENT_ERROR_PACKETS_t MAC_RX_ALIGNMENT_ERROR_PACKETS;  /**< Alignment Error Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_RUNT_ERROR_PACKETS_t MAC_RX_RUNT_ERROR_PACKETS;  /**< Runt Error Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_JABBER_ERROR_PACKETS_t MAC_RX_JABBER_ERROR_PACKETS;  /**< Jabber Error Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_UNDERSIZE_PACKETS_GOOD_t MAC_RX_UNDERSIZE_PACKETS_GOOD;  /**< Good Undersize Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_OVERSIZE_PACKETS_GOOD_t MAC_RX_OVERSIZE_PACKETS_GOOD;  /**< Good Oversize Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_64OCTETS_PACKETS_GOOD_BAD_t MAC_RX_64OCTETS_PACKETS_GOOD_BAD;  /**< Good and Bad 64-Byte Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_65TO127OCTETS_PACKETS_GOOD_BAD_t MAC_RX_65TO127OCTETS_PACKETS_GOOD_BAD;  /**< Good and Bad 64-to-127 Byte Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_128TO255OCTETS_PACKETS_GOOD_BAD_t MAC_RX_128TO255OCTETS_PACKETS_GOOD_BAD;  /**< Good and Bad 128-to-255 Byte Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_256TO511OCTETS_PACKETS_GOOD_BAD_t MAC_RX_256TO511OCTETS_PACKETS_GOOD_BAD;  /**< Good and Bad 256-to-511 Byte Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_512TO1023OCTETS_PACKETS_GOOD_BAD_t MAC_RX_512TO1023OCTETS_PACKETS_GOOD_BAD;  /**< Good and Bad 512-to-1023 Byte Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD_t MAC_RX_1024TOMAXOCTETS_PACKETS_GOOD_BAD;  /**< Good and Bad 1024-to-Max Byte Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_UNICAST_PACKETS_GOOD_t MAC_RX_UNICAST_PACKETS_GOOD;  /**< Good Unicast Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_LENGTH_ERROR_PACKETS_t MAC_RX_LENGTH_ERROR_PACKETS;  /**< Length Error Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_OUT_OF_RANGE_TYPE_PACKETS_t MAC_RX_OUT_OF_RANGE_TYPE_PACKETS;  /**< Out-of-range Type Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_PAUSE_PACKETS_t MAC_RX_PAUSE_PACKETS;  /**< Pause Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_FIFO_OVERFLOW_PACKETS_t MAC_RX_FIFO_OVERFLOW_PACKETS;  /**< Missed Packets Due to FIFO Overflow */
    volatile IMX_ENET_QOS_MAC_RX_VLAN_PACKETS_GOOD_BAD_t MAC_RX_VLAN_PACKETS_GOOD_BAD;  /**< Good and Bad VLAN Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_WATCHDOG_ERROR_PACKETS_t MAC_RX_WATCHDOG_ERROR_PACKETS;  /**< Watchdog Error Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_RECEIVE_ERROR_PACKETS_t MAC_RX_RECEIVE_ERROR_PACKETS;  /**< Receive Error Packets Received */
    volatile IMX_ENET_QOS_MAC_RX_CONTROL_PACKETS_GOOD_t MAC_RX_CONTROL_PACKETS_GOOD;  /**< Good Control Packets Received */
    volatile UINT8 _reserved15[4];
    volatile IMX_ENET_QOS_MAC_TX_LPI_USEC_CNTR_t MAC_TX_LPI_USEC_CNTR;  /**< Microseconds Tx LPI Asserted */
    volatile IMX_ENET_QOS_MAC_TX_LPI_TRAN_CNTR_t MAC_TX_LPI_TRAN_CNTR;  /**< Number of Times Tx LPI Asserted */
    volatile IMX_ENET_QOS_MAC_RX_LPI_USEC_CNTR_t MAC_RX_LPI_USEC_CNTR;  /**< Microseconds Rx LPI Sampled */
    volatile IMX_ENET_QOS_MAC_RX_LPI_TRAN_CNTR_t MAC_RX_LPI_TRAN_CNTR;  /**< Number of Times Rx LPI Entered */
    volatile UINT8 _reserved16[4];
    volatile IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_MASK_t MAC_MMC_IPC_RX_INTERRUPT_MASK;  /**< MMC IPC Receive Interrupt Mask */
    volatile UINT8 _reserved17[4];
    volatile IMX_ENET_QOS_MAC_MMC_IPC_RX_INTERRUPT_t MAC_MMC_IPC_RX_INTERRUPT;  /**< MMC IPC Receive Interrupt */
    volatile UINT8 _reserved18[4];
    volatile IMX_ENET_QOS_MAC_RXIPV4_GOOD_PACKETS_t MAC_RXIPV4_GOOD_PACKETS;  /**< Good IPv4 Datagrams Received */
    volatile IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_PACKETS_t MAC_RXIPV4_HEADER_ERROR_PACKETS;  /**< IPv4 Datagrams Received with Header Errors */
    volatile IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_PACKETS_t MAC_RXIPV4_NO_PAYLOAD_PACKETS;  /**< IPv4 Datagrams Received with No Payload */
    volatile IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_PACKETS_t MAC_RXIPV4_FRAGMENTED_PACKETS;  /**< IPv4 Datagrams Received with Fragmentation */
    volatile IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS_t MAC_RXIPV4_UDP_CHECKSUM_DISABLED_PACKETS;  /**< IPv4 Datagrams Received with UDP Checksum Disabled */
    volatile IMX_ENET_QOS_MAC_RXIPV6_GOOD_PACKETS_t MAC_RXIPV6_GOOD_PACKETS;  /**< Good IPv6 Datagrams Received */
    volatile IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_PACKETS_t MAC_RXIPV6_HEADER_ERROR_PACKETS;  /**< IPv6 Datagrams Received with Header Errors */
    volatile IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_PACKETS_t MAC_RXIPV6_NO_PAYLOAD_PACKETS;  /**< IPv6 Datagrams Received with No Payload */
    volatile IMX_ENET_QOS_MAC_RXUDP_GOOD_PACKETS_t MAC_RXUDP_GOOD_PACKETS;  /**< IPv6 Datagrams Received with Good UDP */
    volatile IMX_ENET_QOS_MAC_RXUDP_ERROR_PACKETS_t MAC_RXUDP_ERROR_PACKETS;  /**< IPv6 Datagrams Received with UDP Checksum Error */
    volatile IMX_ENET_QOS_MAC_RXTCP_GOOD_PACKETS_t MAC_RXTCP_GOOD_PACKETS;  /**< IPv6 Datagrams Received with Good TCP Payload */
    volatile IMX_ENET_QOS_MAC_RXTCP_ERROR_PACKETS_t MAC_RXTCP_ERROR_PACKETS;  /**< IPv6 Datagrams Received with TCP Checksum Error */
    volatile IMX_ENET_QOS_MAC_RXICMP_GOOD_PACKETS_t MAC_RXICMP_GOOD_PACKETS;  /**< IPv6 Datagrams Received with Good ICMP Payload */
    volatile IMX_ENET_QOS_MAC_RXICMP_ERROR_PACKETS_t MAC_RXICMP_ERROR_PACKETS;  /**< IPv6 Datagrams Received with ICMP Checksum Error */
    volatile UINT8 _reserved19[8];
    volatile IMX_ENET_QOS_MAC_RXIPV4_GOOD_OCTETS_t MAC_RXIPV4_GOOD_OCTETS;  /**< Good Bytes Received in IPv4 Datagrams */
    volatile IMX_ENET_QOS_MAC_RXIPV4_HEADER_ERROR_OCTETS_t MAC_RXIPV4_HEADER_ERROR_OCTETS;  /**< Bytes Received in IPv4 Datagrams with Header Errors */
    volatile IMX_ENET_QOS_MAC_RXIPV4_NO_PAYLOAD_OCTETS_t MAC_RXIPV4_NO_PAYLOAD_OCTETS;  /**< Bytes Received in IPv4 Datagrams with No Payload */
    volatile IMX_ENET_QOS_MAC_RXIPV4_FRAGMENTED_OCTETS_t MAC_RXIPV4_FRAGMENTED_OCTETS;  /**< Bytes Received in Fragmented IPv4 Datagrams */
    volatile IMX_ENET_QOS_MAC_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS_t MAC_RXIPV4_UDP_CHECKSUM_DISABLE_OCTETS;  /**< Bytes Received with UDP Checksum Disabled */
    volatile IMX_ENET_QOS_MAC_RXIPV6_GOOD_OCTETS_t MAC_RXIPV6_GOOD_OCTETS;  /**< Bytes Received in Good IPv6 Datagrams */
    volatile IMX_ENET_QOS_MAC_RXIPV6_HEADER_ERROR_OCTETS_t MAC_RXIPV6_HEADER_ERROR_OCTETS;  /**< Bytes Received in IPv6 Datagrams with Data Errors */
    volatile IMX_ENET_QOS_MAC_RXIPV6_NO_PAYLOAD_OCTETS_t MAC_RXIPV6_NO_PAYLOAD_OCTETS;  /**< Bytes Received in IPv6 Datagrams with No Payload */
    volatile IMX_ENET_QOS_MAC_RXUDP_GOOD_OCTETS_t MAC_RXUDP_GOOD_OCTETS;  /**< Bytes Received in Good UDP Segment */
    volatile IMX_ENET_QOS_MAC_RXUDP_ERROR_OCTETS_t MAC_RXUDP_ERROR_OCTETS;  /**< Bytes Received in UDP Segment with Checksum Errors */
    volatile IMX_ENET_QOS_MAC_RXTCP_GOOD_OCTETS_t MAC_RXTCP_GOOD_OCTETS;  /**< Bytes Received in Good TCP Segment */
    volatile IMX_ENET_QOS_MAC_RXTCP_ERROR_OCTETS_t MAC_RXTCP_ERROR_OCTETS;  /**< Bytes Received in TCP Segment with Checksum Errors */
    volatile IMX_ENET_QOS_MAC_RXICMP_GOOD_OCTETS_t MAC_RXICMP_GOOD_OCTETS;  /**< Bytes Received in Good ICMP Segment */
    volatile IMX_ENET_QOS_MAC_RXICMP_ERROR_OCTETS_t MAC_RXICMP_ERROR_OCTETS;  /**< Bytes Received in ICMP Segment with Checksum Errors */
    volatile UINT8 _reserved20[24];
    volatile IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_t MAC_MMC_FPE_TX_INTERRUPT;  /**< MMC FPE Transmit Interrupt */
    volatile IMX_ENET_QOS_MAC_MMC_FPE_TX_INTERRUPT_MASK_t MAC_MMC_FPE_TX_INTERRUPT_MASK;  /**< MMC FPE Transmit Mask Interrupt */
    volatile IMX_ENET_QOS_MAC_MMC_TX_FPE_FRAGMENT_CNTR_t MAC_MMC_TX_FPE_FRAGMENT_CNTR;  /**< MMC FPE Transmitted Fragment Counter */
    volatile IMX_ENET_QOS_MAC_MMC_TX_HOLD_REQ_CNTR_t MAC_MMC_TX_HOLD_REQ_CNTR;  /**< MMC FPE Transmitted Hold Request Counter */
    volatile UINT8 _reserved21[16];
    volatile IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_t MAC_MMC_FPE_RX_INTERRUPT;  /**< MMC FPE Receive Interrupt */
    volatile IMX_ENET_QOS_MAC_MMC_FPE_RX_INTERRUPT_MASK_t MAC_MMC_FPE_RX_INTERRUPT_MASK;  /**< MMC FPE Receive Interrupt Mask */
    volatile IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_ERR_CNTR_t MAC_MMC_RX_PACKET_ASSEMBLY_ERR_CNTR;  /**< MMC Receive Packet Reassembly Error Counter */
    volatile IMX_ENET_QOS_MAC_MMC_RX_PACKET_SMD_ERR_CNTR_t MAC_MMC_RX_PACKET_SMD_ERR_CNTR;  /**< MMC Receive Packet SMD Error Counter */
    volatile IMX_ENET_QOS_MAC_MMC_RX_PACKET_ASSEMBLY_OK_CNTR_t MAC_MMC_RX_PACKET_ASSEMBLY_OK_CNTR;  /**< MMC Receive Packet Successful Reassembly Counter */
    volatile IMX_ENET_QOS_MAC_MMC_RX_FPE_FRAGMENT_CNTR_t MAC_MMC_RX_FPE_FRAGMENT_CNTR;  /**< MMC FPE Received Fragment Counter */
    volatile UINT8 _reserved22[40];
    volatile IMX_ENET_QOS_MAC_L3_L4_CONTROL0_t MAC_L3_L4_CONTROL0;  /**< Layer 3 and Layer 4 Control of Filter 0 */
    volatile IMX_ENET_QOS_MAC_LAYER4_ADDRESS0_t MAC_LAYER4_ADDRESS0;  /**< Layer 4 Address 0 */
    volatile UINT8 _reserved23[8];
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG0_t MAC_LAYER3_ADDR0_REG0;  /**< Layer 3 Address 0 Register 0 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG0_t MAC_LAYER3_ADDR1_REG0;  /**< Layer 3 Address 1 Register 0 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG0_t MAC_LAYER3_ADDR2_REG0;  /**< Layer 3 Address 2 Register 0 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG0_t MAC_LAYER3_ADDR3_REG0;  /**< Layer 3 Address 3 Register 0 */
    volatile UINT8 _reserved24[16];
    volatile IMX_ENET_QOS_MAC_L3_L4_CONTROL1_t MAC_L3_L4_CONTROL1;  /**< Layer 3 and Layer 4 Control of Filter 1 */
    volatile IMX_ENET_QOS_MAC_LAYER4_ADDRESS1_t MAC_LAYER4_ADDRESS1;  /**< Layer 4 Address 0 */
    volatile UINT8 _reserved25[8];
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG1_t MAC_LAYER3_ADDR0_REG1;  /**< Layer 3 Address 0 Register 1 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG1_t MAC_LAYER3_ADDR1_REG1;  /**< Layer 3 Address 1 Register 1 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG1_t MAC_LAYER3_ADDR2_REG1;  /**< Layer 3 Address 2 Register 1 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG1_t MAC_LAYER3_ADDR3_REG1;  /**< Layer 3 Address 3 Register 1 */
    volatile UINT8 _reserved26[16];
    volatile IMX_ENET_QOS_MAC_L3_L4_CONTROL2_t MAC_L3_L4_CONTROL2;  /**< Layer 3 and Layer 4 Control of Filter 2 */
    volatile IMX_ENET_QOS_MAC_LAYER4_ADDRESS2_t MAC_LAYER4_ADDRESS2;  /**< Layer 4 Address 2 */
    volatile UINT8 _reserved27[8];
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG2_t MAC_LAYER3_ADDR0_REG2;  /**< Layer 3 Address 0 Register 2 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG2_t MAC_LAYER3_ADDR1_REG2;  /**< Layer 3 Address 0 Register 2 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG2_t MAC_LAYER3_ADDR2_REG2;  /**< Layer 3 Address 2 Register 2 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG2_t MAC_LAYER3_ADDR3_REG2;  /**< Layer 3 Address 3 Register 2 */
    volatile UINT8 _reserved28[16];
    volatile IMX_ENET_QOS_MAC_L3_L4_CONTROL3_t MAC_L3_L4_CONTROL3;  /**< Layer 3 and Layer 4 Control of Filter 3 */
    volatile IMX_ENET_QOS_MAC_LAYER4_ADDRESS3_t MAC_LAYER4_ADDRESS3;  /**< Layer 4 Address 3 */
    volatile UINT8 _reserved29[8];
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG3_t MAC_LAYER3_ADDR0_REG3;  /**< Layer 3 Address 0 Register 3 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG3_t MAC_LAYER3_ADDR1_REG3;  /**< Layer 3 Address 1 Register 3 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG3_t MAC_LAYER3_ADDR2_REG3;  /**< Layer 3 Address 2 Register 3 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG3_t MAC_LAYER3_ADDR3_REG3;  /**< Layer 3 Address 3 Register 3 */
    volatile UINT8 _reserved30[16];
    volatile IMX_ENET_QOS_MAC_L3_L4_CONTROL4_t MAC_L3_L4_CONTROL4;  /**< Layer 3 and Layer 4 Control of Filter 4 */
    volatile IMX_ENET_QOS_MAC_LAYER4_ADDRESS4_t MAC_LAYER4_ADDRESS4;  /**< Layer 4 Address 4 */
    volatile UINT8 _reserved31[8];
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG4_t MAC_LAYER3_ADDR0_REG4;  /**< Layer 3 Address 0 Register 4 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG4_t MAC_LAYER3_ADDR1_REG4;  /**< Layer 3 Address 1 Register 4 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG4_t MAC_LAYER3_ADDR2_REG4;  /**< Layer 3 Address 2 Register 4 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG4_t MAC_LAYER3_ADDR3_REG4;  /**< Layer 3 Address 3 Register 4 */
    volatile UINT8 _reserved32[16];
    volatile IMX_ENET_QOS_MAC_L3_L4_CONTROL5_t MAC_L3_L4_CONTROL5;  /**< Layer 3 and Layer 4 Control of Filter 5 */
    volatile IMX_ENET_QOS_MAC_LAYER4_ADDRESS5_t MAC_LAYER4_ADDRESS5;  /**< Layer 4 Address 5 */
    volatile UINT8 _reserved33[8];
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG5_t MAC_LAYER3_ADDR0_REG5;  /**< Layer 3 Address 0 Register 5 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG5_t MAC_LAYER3_ADDR1_REG5;  /**< Layer 3 Address 1 Register 5 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG5_t MAC_LAYER3_ADDR2_REG5;  /**< Layer 3 Address 2 Register 5 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG5_t MAC_LAYER3_ADDR3_REG5;  /**< Layer 3 Address 3 Register 5 */
    volatile UINT8 _reserved34[16];
    volatile IMX_ENET_QOS_MAC_L3_L4_CONTROL6_t MAC_L3_L4_CONTROL6;  /**< Layer 3 and Layer 4 Control of Filter 6 */
    volatile IMX_ENET_QOS_MAC_LAYER4_ADDRESS6_t MAC_LAYER4_ADDRESS6;  /**< Layer 4 Address 6 */
    volatile UINT8 _reserved35[8];
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG6_t MAC_LAYER3_ADDR0_REG6;  /**< Layer 3 Address 0 Register 6 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG6_t MAC_LAYER3_ADDR1_REG6;  /**< Layer 3 Address 1 Register 6 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG6_t MAC_LAYER3_ADDR2_REG6;  /**< Layer 3 Address 2 Register 6 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG6_t MAC_LAYER3_ADDR3_REG6;  /**< Layer 3 Address 3 Register 6 */
    volatile UINT8 _reserved36[16];
    volatile IMX_ENET_QOS_MAC_L3_L4_CONTROL7_t MAC_L3_L4_CONTROL7;  /**< Layer 3 and Layer 4 Control of Filter 0 */
    volatile IMX_ENET_QOS_MAC_LAYER4_ADDRESS7_t MAC_LAYER4_ADDRESS7;  /**< Layer 4 Address 7 */
    volatile UINT8 _reserved37[8];
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR0_REG7_t MAC_LAYER3_ADDR0_REG7;  /**< Layer 3 Address 0 Register 7 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR1_REG7_t MAC_LAYER3_ADDR1_REG7;  /**< Layer 3 Address 1 Register 7 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR2_REG7_t MAC_LAYER3_ADDR2_REG7;  /**< Layer 3 Address 2 Register 7 */
    volatile IMX_ENET_QOS_MAC_LAYER3_ADDR3_REG7_t MAC_LAYER3_ADDR3_REG7;  /**< Layer 3 Address 3 Register 7 */
    volatile UINT8 _reserved38[144];
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_CONTROL_t MAC_TIMESTAMP_CONTROL;  /**< Timestamp Control */
    volatile IMX_ENET_QOS_MAC_SUB_SECOND_INCREMENT_t MAC_SUB_SECOND_INCREMENT;  /**< Subsecond Increment */
    volatile IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_t MAC_SYSTEM_TIME_SECONDS;  /**< System Time Seconds */
    volatile IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_t MAC_SYSTEM_TIME_NANOSECONDS;  /**< System Time Nanoseconds */
    volatile IMX_ENET_QOS_MAC_SYSTEM_TIME_SECONDS_UPDATE_t MAC_SYSTEM_TIME_SECONDS_UPDATE;  /**< System Time Seconds Update */
    volatile IMX_ENET_QOS_MAC_SYSTEM_TIME_NANOSECONDS_UPDATE_t MAC_SYSTEM_TIME_NANOSECONDS_UPDATE;  /**< System Time Nanoseconds Update */
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_ADDEND_t MAC_TIMESTAMP_ADDEND;  /**< Timestamp Addend */
    volatile IMX_ENET_QOS_MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS_t MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS;  /**< System Time - Higher Word Seconds */
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_STATUS_t MAC_TIMESTAMP_STATUS;  /**< Timestamp Status */
    volatile UINT8 _reserved39[12];
    volatile IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_NANOSECONDS_t MAC_TX_TIMESTAMP_STATUS_NANOSECONDS;  /**< Transmit Timestamp Status Nanoseconds */
    volatile IMX_ENET_QOS_MAC_TX_TIMESTAMP_STATUS_SECONDS_t MAC_TX_TIMESTAMP_STATUS_SECONDS;  /**< Transmit Timestamp Status Seconds */
    volatile UINT8 _reserved40[8];
    volatile IMX_ENET_QOS_MAC_AUXILIARY_CONTROL_t MAC_AUXILIARY_CONTROL;  /**< Auxiliary Timestamp Control */
    volatile UINT8 _reserved41[4];
    volatile IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_NANOSECONDS_t MAC_AUXILIARY_TIMESTAMP_NANOSECONDS;  /**< Auxiliary Timestamp Nanoseconds */
    volatile IMX_ENET_QOS_MAC_AUXILIARY_TIMESTAMP_SECONDS_t MAC_AUXILIARY_TIMESTAMP_SECONDS;  /**< Auxiliary Timestamp Seconds */
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_ASYM_CORR_t MAC_TIMESTAMP_INGRESS_ASYM_CORR;  /**< Timestamp Ingress Asymmetry Correction */
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_ASYM_CORR_t MAC_TIMESTAMP_EGRESS_ASYM_CORR;  /**< imestamp Egress Asymmetry Correction */
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND_t MAC_TIMESTAMP_INGRESS_CORR_NANOSECOND;  /**< Timestamp Ingress Correction Nanosecond */
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND_t MAC_TIMESTAMP_EGRESS_CORR_NANOSECOND;  /**< Timestamp Egress Correction Nanosecond */
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC_t MAC_TIMESTAMP_INGRESS_CORR_SUBNANOSEC;  /**< Timestamp Ingress Correction Subnanosecond */
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC_t MAC_TIMESTAMP_EGRESS_CORR_SUBNANOSEC;  /**< Timestamp Egress Correction Subnanosecond */
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_INGRESS_LATENCY_t MAC_TIMESTAMP_INGRESS_LATENCY;  /**< Timestamp Ingress Latency */
    volatile IMX_ENET_QOS_MAC_TIMESTAMP_EGRESS_LATENCY_t MAC_TIMESTAMP_EGRESS_LATENCY;  /**< Timestamp Egress Latency */
    volatile IMX_ENET_QOS_MAC_PPS_CONTROL_t MAC_PPS_CONTROL;  /**< PPS Control */
    volatile UINT8 _reserved42[12];
    volatile IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_SECONDS_t MAC_PPS0_TARGET_TIME_SECONDS;  /**< PPS0 Target Time Seconds */
    volatile IMX_ENET_QOS_MAC_PPS0_TARGET_TIME_NANOSECONDS_t MAC_PPS0_TARGET_TIME_NANOSECONDS;  /**< PPS0 Target Time Nanoseconds */
    volatile IMX_ENET_QOS_MAC_PPS0_INTERVAL_t MAC_PPS0_INTERVAL;  /**< PPS0 Interval */
    volatile IMX_ENET_QOS_MAC_PPS0_WIDTH_t MAC_PPS0_WIDTH;  /**< PPS0 Width */
    volatile IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_SECONDS_t MAC_PPS1_TARGET_TIME_SECONDS;  /**< PPS1 Target Time Seconds */
    volatile IMX_ENET_QOS_MAC_PPS1_TARGET_TIME_NANOSECONDS_t MAC_PPS1_TARGET_TIME_NANOSECONDS;  /**< PPS1 Target Time Nanoseconds */
    volatile IMX_ENET_QOS_MAC_PPS1_INTERVAL_t MAC_PPS1_INTERVAL;  /**< PPS1 Interval */
    volatile IMX_ENET_QOS_MAC_PPS1_WIDTH_t MAC_PPS1_WIDTH;  /**< PPS1 Width */
    volatile IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_SECONDS_t MAC_PPS2_TARGET_TIME_SECONDS;  /**< PPS2 Target Time Seconds */
    volatile IMX_ENET_QOS_MAC_PPS2_TARGET_TIME_NANOSECONDS_t MAC_PPS2_TARGET_TIME_NANOSECONDS;  /**< PPS2 Target Time Nanoseconds */
    volatile IMX_ENET_QOS_MAC_PPS2_INTERVAL_t MAC_PPS2_INTERVAL;  /**< PPS2 Interval */
    volatile IMX_ENET_QOS_MAC_PPS2_WIDTH_t MAC_PPS2_WIDTH;  /**< PPS2 Width */
    volatile IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_SECONDS_t MAC_PPS3_TARGET_TIME_SECONDS;  /**< PPS3 Target Time Seconds */
    volatile IMX_ENET_QOS_MAC_PPS3_TARGET_TIME_NANOSECONDS_t MAC_PPS3_TARGET_TIME_NANOSECONDS;  /**< PPS3 Target Time Nanoseconds */
    volatile IMX_ENET_QOS_MAC_PPS3_INTERVAL_t MAC_PPS3_INTERVAL;  /**< PPS3 Interval */
    volatile IMX_ENET_QOS_MAC_PPS3_WIDTH_t MAC_PPS3_WIDTH;  /**< PPS3 Width */
    volatile IMX_ENET_QOS_MAC_PTO_CONTROL_t MAC_PTO_CONTROL;  /**< PTP Offload Engine Control */
    volatile IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY0_t MAC_SOURCE_PORT_IDENTITY0;  /**< Source Port Identity 0 */
    volatile IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY1_t MAC_SOURCE_PORT_IDENTITY1;  /**< Source Port Identity 1 */
    volatile IMX_ENET_QOS_MAC_SOURCE_PORT_IDENTITY2_t MAC_SOURCE_PORT_IDENTITY2;  /**< Source Port Identity 2 */
    volatile IMX_ENET_QOS_MAC_LOG_MESSAGE_INTERVAL_t MAC_LOG_MESSAGE_INTERVAL;  /**< Log Message Interval */
    volatile UINT8 _reserved43[44];
    volatile IMX_ENET_QOS_MTL_OPERATION_MODE_t MTL_OPERATION_MODE;  /**< MTL Operation Mode */
    volatile UINT8 _reserved44[4];
    volatile IMX_ENET_QOS_MTL_DBG_CTL_t MTL_DBG_CTL;        /**< FIFO Debug Access Control and Status */
    volatile IMX_ENET_QOS_MTL_DBG_STS_t MTL_DBG_STS;        /**< FIFO Debug Status */
    volatile IMX_ENET_QOS_MTL_FIFO_DEBUG_DATA_t MTL_FIFO_DEBUG_DATA;  /**< FIFO Debug Data */
    volatile UINT8 _reserved45[12];
    volatile IMX_ENET_QOS_MTL_INTERRUPT_STATUS_t MTL_INTERRUPT_STATUS;  /**< MTL Interrupt Status */
    volatile UINT8 _reserved46[12];
    volatile IMX_ENET_QOS_MTL_RXQ_DMA_MAP0_t MTL_RXQ_DMA_MAP0;  /**< Receive Queue and DMA Channel Mapping 0 */
    volatile IMX_ENET_QOS_MTL_RXQ_DMA_MAP1_t MTL_RXQ_DMA_MAP1;  /**< Receive Queue and DMA Channel Mapping 1 */
    volatile UINT8 _reserved47[8];
    volatile IMX_ENET_QOS_MTL_TBS_CTRL_t MTL_TBS_CTRL;      /**< Time Based Scheduling Control */
    volatile UINT8 _reserved48[12];
    volatile IMX_ENET_QOS_MTL_EST_CONTROL_t MTL_EST_CONTROL;  /**< Enhancements to Scheduled Transmission Control */
    volatile UINT8 _reserved49[4];
    volatile IMX_ENET_QOS_MTL_EST_STATUS_t MTL_EST_STATUS;  /**< Enhancements to Scheduled Transmission Status */
    volatile UINT8 _reserved50[4];
    volatile IMX_ENET_QOS_MTL_EST_SCH_ERROR_t MTL_EST_SCH_ERROR;  /**< EST Scheduling Error */
    volatile IMX_ENET_QOS_MTL_EST_FRM_SIZE_ERROR_t MTL_EST_FRM_SIZE_ERROR;  /**< EST Frame Size Error */
    volatile IMX_ENET_QOS_MTL_EST_FRM_SIZE_CAPTURE_t MTL_EST_FRM_SIZE_CAPTURE;  /**< EST Frame Size Capture */
    volatile UINT8 _reserved51[4];
    volatile IMX_ENET_QOS_MTL_EST_INTR_ENABLE_t MTL_EST_INTR_ENABLE;  /**< EST Interrupt Enable */
    volatile UINT8 _reserved52[12];
    volatile IMX_ENET_QOS_MTL_EST_GCL_CONTROL_t MTL_EST_GCL_CONTROL;  /**< EST GCL Control */
    volatile IMX_ENET_QOS_MTL_EST_GCL_DATA_t MTL_EST_GCL_DATA;  /**< EST GCL Data */
    volatile UINT8 _reserved53[8];
    volatile IMX_ENET_QOS_MTL_FPE_CTRL_STS_t MTL_FPE_CTRL_STS;  /**< Frame Preemption Control and Status */
    volatile IMX_ENET_QOS_MTL_FPE_ADVANCE_t MTL_FPE_ADVANCE;  /**< Frame Preemption Hold and Release Advance */
    volatile UINT8 _reserved54[8];
    volatile IMX_ENET_QOS_MTL_RXP_CONTROL_STATUS_t MTL_RXP_CONTROL_STATUS;  /**< RXP Control Status */
    volatile IMX_ENET_QOS_MTL_RXP_INTERRUPT_CONTROL_STATUS_t MTL_RXP_INTERRUPT_CONTROL_STATUS;  /**< RXP Interrupt Control Status */
    volatile IMX_ENET_QOS_MTL_RXP_DROP_CNT_t MTL_RXP_DROP_CNT;  /**< RXP Drop Count */
    volatile IMX_ENET_QOS_MTL_RXP_ERROR_CNT_t MTL_RXP_ERROR_CNT;  /**< RXP Error Count */
    volatile IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_CONTROL_STATUS_t MTL_RXP_INDIRECT_ACC_CONTROL_STATUS;  /**< RXP Indirect Access Control and Status */
    volatile IMX_ENET_QOS_MTL_RXP_INDIRECT_ACC_DATA_t MTL_RXP_INDIRECT_ACC_DATA;  /**< RXP Indirect Access Data */
    volatile UINT8 _reserved55[72];
    volatile IMX_ENET_QOS_MTL_QUEUE_t MTL_QUEUE[5];
    volatile UINT8 _reserved56[448];
    volatile IMX_ENET_QOS_DMA_MODE_t DMA_MODE;              /**< DMA Bus Mode */
    volatile IMX_ENET_QOS_DMA_SYSBUS_MODE_t DMA_SYSBUS_MODE;  /**< DMA System Bus Mode */
    volatile IMX_ENET_QOS_DMA_INTERRUPT_STATUS_t DMA_INTERRUPT_STATUS;  /**< DMA Interrupt Status */
    volatile IMX_ENET_QOS_DMA_DEBUG_STATUS0_t DMA_DEBUG_STATUS0;  /**< DMA Debug Status 0 */
    volatile IMX_ENET_QOS_DMA_DEBUG_STATUS1_t DMA_DEBUG_STATUS1;  /**< DMA Debug Status 1 */
    volatile UINT8 _reserved57[44];
    volatile IMX_ENET_QOS_DMA_AXI_LPI_ENTRY_INTERVAL_t DMA_AXI_LPI_ENTRY_INTERVAL;  /**< AXI LPI Entry Interval Control */
    volatile UINT8 _reserved58[12];
    volatile IMX_ENET_QOS_DMA_TBS_CTRL_t DMA_TBS_CTRL;      /**< TBS Control */
    volatile UINT8 _reserved59[172];
    volatile IMX_ENET_QOS_DMA_CH_t DMA_CH[5];
} CSP_ENET_REGS, * PCSP_ENET_REGS;

/* **************************************************************************** */
/* Transmit descriptors */
/* **************************************************************************** */
/* TDES0 - Read format */
#define TDES0_BUF1AP_MASK       0xFFFFFFFF  /**< Buffer 1 address pointer */
#define TDES0_BUF1AP_SHIFT      0U
/* TDES1 - Read format */
#define TDES1_BUF2AP_MASK       0xFFFFFFFF  /**< Buffer 2 address pointer */
#define TDES1_BUF2AP_SHIFT      0U
/* TDES2 - Read format */
#define TDES2_HL_B1L_MASK       0x3FFF      /**< Header Length or Buffer 1 Length */
#define TDES2_HL_B1L_SHIFT      0U
#define TDES2_VTIR_MASK         0xC000      /**< VLAN Tag Insertion or Replacement */
#define TDES2_VTIR_SHIFT        14U
#define TDES2_B2L_MASK          0x3FFF0000  /**< Buffer 2 Length */
#define TDES2_B2L_SHIFT         16U
#define TDES2_TTSE_TMWD_MASK    0x40000000  /**< Transmit Timestamp Enable or External TSO Memory Write Enable */
#define TDES2_TTSE_TMWD_SHIFT   30U
#define TDES2_IOC_MASK          0x80000000  /**< Interrupt on Completion */
#define TDES2_IOC_SHIFT         31U

/* TDES3 - Read format */
#define TDES3_FL_TPL_MASK       0x000007FF  /**< Frame Length or TCP Payload Length */
#define TDES3_FL_TPL_SHIFT      0U
#define TDES3_TPL_MASK          0x00008000  /**< TCP Payload Length */
#define TDES3_TPL_SHIFT         15U
#define TDES3_CIC_TPL_MASK      0x00030000  /**< Checksum Insertion Control or TCP Payload Length */
#define TDES3_CIC_TPL_SHIFT     16U
#define TDES3_TSE_MASK          0x00040000  /**< TSO Split header is not supported. The value of this bit is always zero */
#define TDES3_TSE_SHIFT         18U
#define TDES3_SLOTNUM_THL_MASK  0x00780000  /**< SLOTNUM: Slot Number Control Bits in AV Mode */
#define TDES3_SLOTNUM_THL_SHIFT 19U
#define TDES3_SAIC_MASK         0x03800000  /**< SA Insertion Control */
#define TDES3_SAIC_SHIFT        23U
#define TDES3_CPC_MASK          0x0C000000  /**< CRC Pad Control */
#define TDES3_CPC_SHIFT         26U

/* TDES0 - Write-Back format */
#define TDES0_TIMESTAMP_LOW_MASK  0xFFFFFFFF
#define TDES0_TIMESTAMP_LOW_SHIFT 0U
/* TDES1 - Write-Back format */
#define TDES1_TIMESTAMP_HIGH_MASK  0xFFFFFFFF
#define TDES1_TIMESTAMP_HIGH_SHIFT 0U
/* TDES2 - Write-Back format - RESERVED*/
/* TDES3 - Write-Back format */
#define TDES3_IHE_MASK      0x00000001  /**< IP Header Error */
#define TDES3_IHE_SHIFT     0U
#define TDES3_DB_MASK       0x00000002  /**< Deferred Bit */
#define TDES3_DB_SHIFT      1U
#define TDES3_UF_MASK       0x00000004  /**< Underflow error */
#define TDES3_UF_SHIFT      2U
#define TDES3_ED_MASK       0x00000008  /**< Excessive Deferral */
#define TDES3_ED_SHIFT      3U
#define TDES3_CC_MASK       0x000000F0  /**< Excessive Deferral */
#define TDES3_CC_SHIFT      4U
#define TDES3_EC_MASK       0x00000100  /**< Excessive Collision */
#define TDES3_EC_SHIFT      8U
#define TDES3_LC_MASK       0x00000200  /**< Late Collision */
#define TDES3_LC_SHIFT      9U
#define TDES3_NC_MASK       0x00000400  /**< No Carrier */
#define TDES3_NC_SHIFT      10U
#define TDES3_LOC_MASK       0x00000800  /**< Loss of Carrier  */
#define TDES3_LOC_SHIFT     11U
#define TDES3_PCE_MASK      0x00001000  /**< Payload Checksum Error */
#define TDES3_PCE_SHIFT     12U
#define TDES3_FF_MASK       0x00002000  /**< Packet Flushed */
#define TDES3_FF_SHIFT      13U
#define TDES3_JT_MASK       0x00004000  /**< Jabber Timeout */
#define TDES3_JT_SHIFT      14U
#define TDES3_ES_MASK       0x00008000  /**< Error Summary */
#define TDES3_ES_SHIFT      15U
#define TDES3_EUE_MASK      0x00010000  /**< ECC Uncorrectable Error Status */
#define TDES3_EUE_SHIFT     16U
#define TDES3_TTSS_MASK     0x00020000  /**< Tx Timestamp Status */
#define TDES3_TTSS_SHIFT    17U
#define TDES3_DE_MASK       0x00800000  /**< Descriptor Error */
#define TDES3_DE_SHIFT      23U

/* TDES3 - Both Read and Write-Back format */
#define TDES3_LD_MASK       0x10000000  /**< Last Descriptor */
#define TDES3_LD_SHIFT      28U
#define TDES3_FD_MASK       0x20000000  /**< First Descriptor */
#define TDES3_FD_SHIFT      29U
#define TDES3_CTXT_MASK     0x40000000  /**< Context Type */
#define TDES3_CTXT_SHIFT    30U
#define TDES3_OWN_MASK      0x80000000  /**< Own Bit */
#define TDES3_OWN_SHIFT     31U
/* **************************************************************************** */


/* **************************************************************************** */
/* Receive descriptors */
/* **************************************************************************** */
/* RDES0 - Read format */
#define RDES0_BUF1AP_MASK       0xFFFFFFFF  /**< Header or Buffer 1 Address Pointer */
#define RDES0_BUF1AP_SHIFT      0U
/* RDES1 - Read format */
#define RDES1_BUF1AP_MASK       0xFFFFFFFF  /**< Buffer 1 Address (in 64bit addressing mode) Pointer or reserved */
#define RDES1_BUF1AP_SHIFT      0U
/* RDES2 - Read format */
#define RDES2_BUF2AP_MASK       0xFFFFFFFF  /**< Buffer 2 Address Pointer */
#define RDES2_BUF2AP_SHIFT      0U
/* RDES3 - Read format */
#define RDES3_BUF1V_MASK        0x01000000  /**< Buffer 1 Address Valid */
#define RDES3_BUF1V_SHIFT       24U
#define RDES3_BUF2V_MASK        0x02000000  /**< Buffer 2 Address Valid */
#define RDES3_BUF2V_SHIFT       25U
#define RDES3_IOC_MASK          0x40000000  /**< Interrupt Enabled on Completition */
#define RDES3_IOC_SHIFT         30U

/* RDES0 - Write-Back format */
#define RDES0_OVT_MASK          0x0000FFFF  /**< Outer VLAN Tag */
#define RDES0_OVT_SHIFT         0U
#define RDES0_IVT_MASK          0xFFFF0000  /**< Inner VLAN Tag */
#define RDES0_IVT_SHIFT         16U

/* RDES1 - Write-Back format */
#define RDES1_PT_MASK           0x00000007  /**< Payload Type */
#define RDES1_PT_SHIFT          0U
#define RDES1_IPHE_MASK         0x00000008  /**< IP Header Error */
#define RDES1_IPHE_SHIFT        3U
#define RDES1_IPV4_MASK         0x00000010  /**< IPV4 Header Present */
#define RDES1_IPV4_SHIFT        4U
#define RDES1_IPV6_MASK         0x00000020  /**< IPv6 header Present */
#define RDES1_IPV6_SHIFT        5U
#define RDES1_IPCB_MASK         0x00000040  /**< IP Checksum Bypassed */
#define RDES1_IPCB_SHIFT        6U
#define RDES1_IPCE_MASK         0x00000080  /**< IP Payload Error */
#define RDES1_IPCE_SHIFT        7U
#define RDES1_PMT_MASK          0x00000F00  /**< PTP Message Type */
#define RDES1_PMT_SHIFT         8U
#define RDES1_PFT_MASK          0x00001000  /**< PTP Packet Type */
#define RDES1_PFT_SHIFT         12U
#define RDES1_PV_MASK           0x00002000  /**< PTP Version */
#define RDES1_PV_SHIFT          13U
#define RDES1_TSA_MASK          0x00004000  /**< Timestamp Available */
#define RDES1_TSA_SHIFT         14U
#define RDES1_TD_MASK           0x00008000  /**< Timestamp Dropped */
#define RDES1_TD_SHIFT          15U
#define RDES1_OPC_MASK          0xFFFF0000  /**< OAM Sub-Type Code, or MAC Control Packet opcode */
#define RDES1_OPC_SHIFT         16U

/* RDES2 - Write-Back format */
#define RDES2_HL_MASK           0x000003FF  /**< L3/L4 Header Length */
#define RDES2_HL_SHIFT          0U
#define RDES2_ARPNR_MASK        0x00000400  /**< ARP Reply Not Generated */
#define RDES2_ARPNR_SHIFT       10U
#define RDES2_ITS_MASK          0x00004000  /**< Inner VLAN Tag Filter Status (ITS) */
#define RDES2_ITS_SHIFT         14U
#define RDES2_OTS_MASK          0x00008000  /**< VLAN Filter Status */
#define RDES2_OTS_SHIFT         15U
#define RDES2_SAF_MASK          0x00010000  /**< SA Address Filter Fail */
#define RDES2_SAF_SHIFT         16U
#define RDES2_DAF_MASK          0x00020000  /**< Destination Address Filter Fail */
#define RDES2_DAF_SHIFT         17U
#define RDES2_HF_MASK           0x00040000  /**< Hash Filter Status */
#define RDES2_HF_SHIFT          18U
#define RDES2_MADRM_MASK        0x07F80000  /**< MAC Address Match or Hash Value */
#define RDES2_MADRM_SHIFT       19U
#define RDES2_L3FM_MASK         0x08000000  /**< Layer 3 Filter Match */
#define RDES2_L3FM_SHIFT        27U
#define RDES2_L4FM_MASK         0x10000000  /**< Layer 4 Filter Match */
#define RDES2_L4FM_SHIFT        28U
#define RDES2_L3L4FM_MASK       0xE0000000  /**< Layer 3 and Layer 4 Filter Number Matched */
#define RDES2_L3L4FM_SHIFT      29U

/* RDES3 - Write-Back format */
#define RDES3_PL_MASK           0x00007FFF  /**< L3/L4 Header Length */
#define RDES3_PL_SHIFT          0U
#define RDES3_ES_MASK           0x00008000  /**< Error Summary */
#define RDES3_ES_SHIFT          15U
#define RDES3_LT_MASK           0x00070000  /**< Length/Type Field */
#define RDES3_LT_SHIFT          16U
#define RDES3_DE_MASK           0x00080000  /**< Dribble Bit Error */
#define RDES3_DE_SHIFT          19U
#define RDES3_RE_MASK           0x00100000  /**< Receive Error */
#define RDES3_RE_SHIFT          20U
#define RDES3_OE_MASK           0x00200000  /**< Overflow Error */
#define RDES3_OE_SHIFT          21U
#define RDES3_RWT_MASK          0x00400000  /**< Receive Watchdog Timeout */
#define RDES3_RWT_SHIFT         22U
#define RDES3_GP_MASK           0x00800000  /**< Giant Packet */
#define RDES3_GP_SHIFT          23U
#define RDES3_CE_MASK           0x01000000  /**< CRC Error */
#define RDES3_CE_SHIFT          24U
#define RDES3_RS0V_MASK         0x02000000  /**< Receive Status RDES0 Valid */
#define RDES3_RS0V_SHIFT        25U
#define RDES3_RS1V_MASK         0x04000000  /**< Receive Status RDES1 Valid */
#define RDES3_RS1V_SHIFT        26U
#define RDES3_RS2V_MASK         0x08000000  /**< Receive Status RDES2 Valid */
#define RDES3_RS2V_SHIFT        27U
#define RDES3_LD_MASK           0x10000000  /**< Last Descriptor */
#define RDES3_LD_SHIFT          28U
#define RDES3_FD_MASK           0x20000000  /**< First Descriptor */
#define RDES3_FD_SHIFT          29U
#define RDES3_CTXT_MASK         0x40000000  /**< Receive Context Descriptor */
#define RDES3_CTXT_SHIFT        30U

/* RDES3 - Both Read and Write-Back format */
#define RDES3_OWN_MASK         0x80000000  /**< Own Bit */
#define RDES3_OWN_SHIFT        31U
/* **************************************************************************** */

#pragma pack(push, 1)
/* Transmit and receive descriptors */
typedef struct {
    UINT32 des0;
    UINT32 des1;
    UINT32 des2;
    UINT32 des3;
} ENET_BD, * PENET_BD;
#pragma pack(pop)

#endif /* _ENET_QOS_IOMAP_H */