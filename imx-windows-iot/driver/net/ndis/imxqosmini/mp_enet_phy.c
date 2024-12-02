/*
* Copyright 2019,2022-2023 NXP
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

#include "precomp.h"

#define DBG_PHY_DEV_PRINT_LINK_PROPERTIES(_msg_, _LinkProperties_)  \
    DBG_PHY_DEV_PRINT_INFO(_msg_##" - AutoNego:%d, Pause:%d, AsymPause:%d, T_10_HD=%d, T_10_FD=%d, TX_100_HD=%d, TX_100_FD=%d, T_1000_HD=%d, T_1000_FD=%d", \
        _LinkProperties_.B.AUTO_NEGOTIATION, \
        _LinkProperties_.B.PAUSE, \
        _LinkProperties_.B.ASYMETRIC_PAUSE, \
        _LinkProperties_.B.BASE_T_10_HD, \
        _LinkProperties_.B.BASE_T_10_FD, \
        _LinkProperties_.B.BASE_TX_100_HD, \
        _LinkProperties_.B.BASE_TX_100_FD, \
        _LinkProperties_.B.BASE_T_1000_HD, \
        _LinkProperties_.B.BASE_T_1000_FD \
)

/*++
Routine Description:
    MII read function to read data from the PHY.
Argument:
    Handle - Adapter context structure
    PhyAddr - PHY ID
    RegAddr - MII registre value
Return:
    Data read from the PHY.
--*/
static UINT16 MII_Read(PMP_ADAPTER pAdapter, UINT8 PhyAddr, UINT8 RegAddr)
{
    volatile CSP_ENET_REGS* ENETRegBase = pAdapter->ENETRegBase;
    UINT16 Data = 0x00U;
    UINT32 MdioAddr;
    UINT32 Timeout = 10000;

    MdioAddr = IMX_ENET_QOS_MAC_MDIO_ADDRESS_PA(PhyAddr) |
        IMX_ENET_QOS_MAC_MDIO_ADDRESS_RDA(RegAddr) |
        IMX_ENET_QOS_MAC_MDIO_ADDRESS_CR(0x05) |   // MDC clock divider = /124 (266MHz / 124 = 2.15MHz)
        IMX_ENET_QOS_MAC_MDIO_ADDRESS_GOC_1_MASK | // MII Read command
        IMX_ENET_QOS_MAC_MDIO_ADDRESS_GOC_0_MASK | // MII Read command
        IMX_ENET_QOS_MAC_MDIO_ADDRESS_GB_MASK;

    // Write MDIO Address register to initiate READ command
    ENETRegBase->MAC_MDIO_ADDRESS.R = MdioAddr;

    while (Timeout > 0) {
        if ((ENETRegBase->MAC_MDIO_ADDRESS.R & IMX_ENET_QOS_MAC_MDIO_ADDRESS_GB_MASK) == 0U) {
            break;
        }
        Timeout--;
    }

    if (Timeout == 0) {
        //DBG_PHY_DEV_PRINT_INFO("%s() MII READ cmd timeout");
    }

    Data = ENETRegBase->MAC_MDIO_DATA.R & IMX_ENET_QOS_MAC_MDIO_DATA_GD_MASK;

    return Data;
}

/*++
Routine Description:
    MII write function to write data into the PHY.
Argument:
    Handle - Adapter context structure
    PhyAddr - PHY ID
    RegAddr - MII registre value
    Data - Data to be programmed into the PHY
--*/
static void MII_Write(PMP_ADAPTER pAdapter, UINT8 PhyAddr, UINT8 RegAddr, UINT16 Data)
{
    volatile CSP_ENET_REGS* ENETRegBase = pAdapter->ENETRegBase;
    UINT32 MdioAddr;
    UINT32 Timeout = 10000;

    // Write MDIO data register
    ENETRegBase->MAC_MDIO_DATA.R = (UINT32) Data;

    MdioAddr = IMX_ENET_QOS_MAC_MDIO_ADDRESS_PA(PhyAddr) |
        IMX_ENET_QOS_MAC_MDIO_ADDRESS_RDA(RegAddr) |
        IMX_ENET_QOS_MAC_MDIO_ADDRESS_CR(0x05) |    // MDC clock divider = /124 (266MHz / 124 = 2.15MHz)
        IMX_ENET_QOS_MAC_MDIO_ADDRESS_GOC_0_MASK |  // MII Write command
        IMX_ENET_QOS_MAC_MDIO_ADDRESS_GB_MASK;

    // Write MDIO Address register to initiate WRITE command
    ENETRegBase->MAC_MDIO_ADDRESS.R = MdioAddr;

    while (Timeout > 0) {
        if ((ENETRegBase->MAC_MDIO_ADDRESS.R & IMX_ENET_QOS_MAC_MDIO_ADDRESS_GB_MASK) == 0U) {
            break;
        }
        Timeout--;
    }

    if (Timeout == 0) {
        //DBG_PHY_DEV_PRINT_INFO("%s() MII WRITE cmd timeout");
    }

    return;
}

/*++
Routine Description:
    RTL8211 PHY Initialization.
Argument:
    pAdapter  - Context parameter.
Return:
    Execution status.
--*/
NTSTATUS MII_Rtl8211fInit(PMP_ADAPTER pAdapter)
{
    NTSTATUS Status = STATUS_SUCCESS;
    UINT16 Val;
    UINT8 PhyAddr = pAdapter->MiiCfg.PhyAddr;

    // Select Page 0x0d08*/
    MII_Write(pAdapter, PhyAddr, 0x1F, 0x0d08);

    // Enable TX-delay for rgmii-id and rgmii-txid
    Val = MII_Read(pAdapter, PhyAddr, 0x11);
    if (pAdapter->MiiCfg.MiiInterfaceType == RGMII) {
        // RGMII config
        Val |= 0x0100;
    } else {
        Val &= ~0x0100;
    }
    MII_Write(pAdapter, PhyAddr, 0x11, Val);

    // Enable RX-delay for rgmii-id and rgmii-rxid
    Val = MII_Read(pAdapter, PhyAddr, 0x15);
    if (pAdapter->MiiCfg.MiiInterfaceType == RGMII) {
        // RGMII config
        Val |= 0x0008;
    } else {
        Val &= ~0x0008;
    }
    MII_Write(pAdapter, PhyAddr, 0x15, Val);

    // Restore to default page 0
    MII_Write(pAdapter, PhyAddr, 0x1F, 0x0000);

    // Set green LED for Link, yellow LED for Active
    MII_Write(pAdapter, PhyAddr, 0x1F, 0x0D04);
    MII_Write(pAdapter, PhyAddr, 0x10, 0x617F);
    MII_Write(pAdapter, PhyAddr, 0x1F, 0x0000);

    return Status;
}

/*++
Routine Description:
    Configure autonegotiation advertisement register.
Argument:
    pAdapter  - Context parameter.
    PhyAddr   - Required PHY address.
Return:
    Execution status.
--*/
NTSTATUS MII_DumpPhy(PMP_ADAPTER pAdapter)
{
    NTSTATUS Status = STATUS_SUCCESS;
    UINT32 RegVal;

    DBG_PHY_DEV_PRINT_INFO("============================================");
    DBG_PHY_DEV_PRINT_INFO("===          Dump PHY registers          ===");
    DBG_PHY_DEV_PRINT_INFO("============================================");
    for (UINT8 idx = 0; idx <= 0xF; idx++) {
        RegVal = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, idx);
        DBG_PHY_DEV_PRINT_INFO("phy[0x%02X] = 0x%04X", idx, RegVal);
    }
    DBG_PHY_DEV_PRINT_INFO("============================================");

    return Status;
}

/*++
Routine Description:
    Resolve speed and duplex mode based on current link status.
Argument:
    pPHYDev - Pointer to Phy device structure.
Return:
    Execution status.
--*/
UINT32 MII_ResolveSpeedMode(MP_PHY_DEVICE* pPHYDev)
{
    IMX_MII_LINK_PROPERTIES_t  LinkProperties;
    UINT32                     LinkState = 0;

    LinkProperties.R = pPHYDev->PHYDev_AdvertisedLinkProperties.R & pPHYDev->PHYDev_LinkPartnerLinkProperties.R;
    if (LinkProperties.B.BASE_T_1000_FD) { // 1000BASE-T full duplex
        LinkState = IMX_MII_LINK_STATE_DUPLEX_MODE_SET(MediaDuplexStateFull) |
                    IMX_MII_LINK_STATE_SPEED_MHZ_SET(1000) |
                    IMX_MII_LINK_STATE_CONNECT_STATE_SET(MediaConnectStateConnected);
    } else if (LinkProperties.B.BASE_T_1000_HD) { // 1000BASE-T half duplex
        LinkState = IMX_MII_LINK_STATE_DUPLEX_MODE_SET(MediaDuplexStateHalf) |
                    IMX_MII_LINK_STATE_SPEED_MHZ_SET(1000) |
                    IMX_MII_LINK_STATE_CONNECT_STATE_SET(MediaConnectStateConnected);
    } else if (LinkProperties.B.BASE_TX_100_FD) { // 100BASE-TX full duplex
        LinkState = IMX_MII_LINK_STATE_DUPLEX_MODE_SET(MediaDuplexStateFull) |
                    IMX_MII_LINK_STATE_SPEED_MHZ_SET(100) |
                    IMX_MII_LINK_STATE_CONNECT_STATE_SET(MediaConnectStateConnected);
    } else if (LinkProperties.B.BASE_TX_100_HD) { // 100BASE-TX half duplex
        LinkState = IMX_MII_LINK_STATE_DUPLEX_MODE_SET(MediaDuplexStateHalf) |
                    IMX_MII_LINK_STATE_SPEED_MHZ_SET(100) |
                    IMX_MII_LINK_STATE_CONNECT_STATE_SET(MediaConnectStateConnected);
    } else if (LinkProperties.B.BASE_T_10_FD) { // 10BASE-T full duplex
        LinkState = IMX_MII_LINK_STATE_DUPLEX_MODE_SET(MediaDuplexStateFull) |
                    IMX_MII_LINK_STATE_SPEED_MHZ_SET(10) |
                    IMX_MII_LINK_STATE_CONNECT_STATE_SET(MediaConnectStateConnected);
    } else if (LinkProperties.B.BASE_T_10_HD) { // 10BASE-T half duplex
        LinkState = IMX_MII_LINK_STATE_DUPLEX_MODE_SET(MediaDuplexStateHalf) |
                    IMX_MII_LINK_STATE_SPEED_MHZ_SET(10) |
                    IMX_MII_LINK_STATE_CONNECT_STATE_SET(MediaConnectStateConnected);
    } else {
        LinkState = IMX_MII_LINK_STATE_DUPLEX_MODE_SET(MediaDuplexStateUnknown) |
                    IMX_MII_LINK_STATE_SPEED_MHZ_SET(0) |
                    IMX_MII_LINK_STATE_CONNECT_STATE_SET(MediaConnectStateDisconnected);
    }

    DBG_PHY_DEV_PRINT_INFO("Selected Media type %d Mbs, %s duplex.", IMX_MII_LINK_STATE_SPEED_MHZ_GET(LinkState), 
        (IMX_MII_LINK_STATE_DUPLEX_MODE_GET(LinkState) == MediaDuplexStateFull ? "Full" : (IMX_MII_LINK_STATE_DUPLEX_MODE_GET(LinkState) ? "Half" : "Unknown")));

    return LinkState;
}

/*++
Routine Description:
    Get current link status.
Argument:
    pAdapter  - Context parameter.
    State     - Link state;
Return:
    Execution status.
--*/
NTSTATUS MII_GetLinkStatus(PMP_ADAPTER pAdapter, BOOLEAN *StateChanged, MP_PHY_LINK_STATE *NewState)
{
    MII_REG_S_t StatusReg;
    NTSTATUS Status = STATUS_SUCCESS;
    static MP_PHY_LINK_STATE PreviousLinkState = MP_PHY_LINK_DOWN;

    // Read Phy Status register twice*/
    StatusReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_S);
    StatusReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_S);

    *NewState = StatusReg.B.LINK_STATUS;

    if (PreviousLinkState != *NewState) {
        *StateChanged = TRUE;
    } else {
        *StateChanged = FALSE;
    }
    PreviousLinkState = *NewState;

    return Status;
}

/*++
Routine Description:
    Returns link partner advertised media type.
Argument:
    pAdapter  - Context parameter.
Return Value:
    Media type advertised by the link partner
--*/
UINT32 MII_GetLinkPartnerLinkProperties(PMP_ADAPTER pAdapter)
{
    IMX_MII_LINK_PROPERTIES_t  LinkProperties;
    MII_REG_S_t   StatusReg;
    MII_REG_LPA_t LinkPartnerAbilityReg;
    MII_REG_BASET_1000_S_t LinkPartnerBase1000AbilityReg;

    LinkProperties.R = 0;

    // Read Status register
    StatusReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_S);
    // Read Link Partner ability register
    LinkPartnerAbilityReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_LPA);

    if (StatusReg.B.AUTO_NEGOTIATION_COMPLETE) {
        LinkProperties.B.AUTO_NEGOTIATION = 1;
        LinkProperties.B.ASYMETRIC_PAUSE = LinkPartnerAbilityReg.B.ASYMETRIC_PAUSE;           // Link partner has advertised Asymetric pause.
        LinkProperties.B.PAUSE = LinkPartnerAbilityReg.B.PAUSE;                               // Link partner has advertised pause frames.
        LinkProperties.B.BASE_T_10_HD = LinkPartnerAbilityReg.B.BASE_T_10_FD;                 // Link partner has advertised 10BASE-T half duplex.
        LinkProperties.B.BASE_T_10_FD = LinkPartnerAbilityReg.B.BASE_T_10_HD;                 // Link partner has advertised 10BASE-T full duplex.
        LinkProperties.B.BASE_TX_100_HD = LinkPartnerAbilityReg.B.BASE_TX_100_HD;             // Link partner has advertised 100BASE-TX half duplex.
        LinkProperties.B.BASE_TX_100_FD = LinkPartnerAbilityReg.B.BASE_TX_100_FD;             // Link partner has advertised 100BASE-TX full duplex.
        if (StatusReg.B.EXTENDED_STATUS) {
            // Read Base 1000 Status register to get base1000 Link partner abilities
            LinkPartnerBase1000AbilityReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_BASET_1000_S);
            LinkProperties.B.BASE_T_1000_HD = LinkPartnerBase1000AbilityReg.B.BASE_T_1000_HD;  // Link partner has advertised 1000BASE-T half duplex.
            LinkProperties.B.BASE_T_1000_FD = LinkPartnerBase1000AbilityReg.B.BASE_T_1000_FD;  // Link partner has advertised 1000BASE-T full duplex.
        }
    }

    return LinkProperties.R;
}

/*++
Routine Description:
    Returns advertised media type.
Argument:
    pAdapter  - Context parameter.
Return Value:
    Media type advertised by the PHY
--*/
UINT32 MII_GetAdvertisedLinkProperties(PMP_ADAPTER pAdapter)
{
    IMX_MII_LINK_PROPERTIES_t  LinkProperties;
    MII_REG_C_t   ControlReg;
    MII_REG_S_t   StatusReg;
    MII_REG_ANA_t AdvertisingReg;
    MII_REG_BASET_1000_C_t Base1000TControlReg;

    LinkProperties.R = 0;

    // Read Control register
    ControlReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_C);
    // Read Status register
    StatusReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_S);
    // Read Link Partner ability register
    AdvertisingReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_ANA);

    if (ControlReg.B.AUTO_NEGOTIATION_ENABLE) {
        // Auto negotiation is enabled, read advertised values from ANA and BASE_T_1000 registers.
        LinkProperties.B.AUTO_NEGOTIATION = 1;                                         // Local device has advertised Auto negotiation ability.
        LinkProperties.B.ASYMETRIC_PAUSE = AdvertisingReg.B.ASYMETRIC_PAUSE;           // Local device has advertised Asymetric pause.
        LinkProperties.B.PAUSE = AdvertisingReg.B.PAUSE;                               // Local device has advertised Pause.
        LinkProperties.B.BASE_TX_100_FD = AdvertisingReg.B.BASE_TX_100_FD;             // Local device has advertised 100BASE-TX full duplex.
        LinkProperties.B.BASE_TX_100_HD = AdvertisingReg.B.BASE_TX_100_HD;             // Local device has advertised 100BASE-TX half duplex.
        LinkProperties.B.BASE_T_10_FD = AdvertisingReg.B.BASE_T_10_HD;                 // Local device has advertised 10BASE-T full duplex.
        LinkProperties.B.BASE_T_10_HD = AdvertisingReg.B.BASE_T_10_FD;                 // Local device has advertised 10BASE-T half duplex.
        if (StatusReg.B.EXTENDED_STATUS) {
            Base1000TControlReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_BASET_1000_C);
            LinkProperties.B.BASE_T_1000_FD = Base1000TControlReg.B.BASE_T_1000_FD;  // Local device has advertised 1000BASE-T full duplex.
            LinkProperties.B.BASE_T_1000_HD = Base1000TControlReg.B.BASE_T_1000_HD;  // Local device has advertised 1000BASE-T half duplex.
        }
    } else {
        // Auto negotiation is disabled, decode "advertised values" from Control register
        switch (ControlReg.R & (MII_REG_C_DUPLEX_MODE_MASK | MII_REG_C_SPEED_SELECTION_MSB_MASK | MII_REG_C_SPEED_SELECTION_LSB_MASK)) {
        case 0:
            LinkProperties.B.BASE_T_10_HD = 1;
            break;
        case MII_REG_C_DUPLEX_MODE_MASK:
            LinkProperties.B.BASE_T_10_FD = 1;
            break;
        case MII_REG_C_SPEED_SELECTION_LSB_MASK:
            LinkProperties.B.BASE_TX_100_HD = 1;
            break;
        case MII_REG_C_DUPLEX_MODE_MASK | MII_REG_C_SPEED_SELECTION_LSB_MASK:
            LinkProperties.B.BASE_TX_100_FD = 1;
            break;
        case MII_REG_C_SPEED_SELECTION_MSB_MASK:
            LinkProperties.B.BASE_T_1000_HD = 1;
            break;
        case MII_REG_C_DUPLEX_MODE_MASK | MII_REG_C_SPEED_SELECTION_MSB_MASK:
            LinkProperties.B.BASE_T_1000_FD = 1;
            break;
        }
    }
    return LinkProperties.R;
}

/*++
Routine Description:
    Returns PHY supported media type.
Argument:
    Adapter      - Context parameter.
    Capabilities - PHY supported capabilities.
Return Value:
    Execution status
--*/
NTSTATUS MII_GetPhyCapatibilities(PMP_ADAPTER pAdapter)
{
    NTSTATUS Status = STATUS_SUCCESS;
    MII_REG_S_t StatusReg;
    MII_REG_ES_t ExStatusReg;

    // Read Phy Status register*/
    StatusReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_S);

    pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.R = IMX_MII_LINK_PROPERTIES_PAUSE | IMX_MII_LINK_PROPERTIES_ASYMETRIC_PAUSE; // Local device supports both Pause Rx and Pause Tx.
    pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.B.AUTO_NEGOTIATION = StatusReg.B.AUTO_NEGOTIATION_ABILITY;                   // Local device supports Local device supports Auto negotiation ability.
    pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.B.BASE_T_10_HD = StatusReg.B.BASE_T_10_FD;                                   // Local device supports 10BASE-T half duplex.
    pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.B.BASE_T_10_FD = StatusReg.B.BASE_T_10_HD;                                   // Local device supports 10BASE-T full duplex.
    pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.B.BASE_TX_100_HD = StatusReg.B.BASE_X_100_HD;                                // Local device supports 100BASE-TX half duplex.
    pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.B.BASE_TX_100_FD = StatusReg.B.BASE_X_100_FD;                                // Local device supports 100BASE-TX full duplex.
    if (StatusReg.B.EXTENDED_STATUS) {
        // Read Extended Status register
        ExStatusReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_ES);
        pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.B.BASE_T_1000_HD = ExStatusReg.B.BASE_T_1000_HD;                          // Local device supports 1000BASE-T half duplex.
        pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.B.BASE_T_1000_FD = ExStatusReg.B.BASE_T_1000_FD;                          // Local device supports 1000BASE-T full duplex.
    }
    return Status;
}

/*++
Routine Description:
    Trigger PHY Auto-Negotiation process.
Argument:
    pAdapter  - Context parameter.
Return:
    Execution status.
--*/
NTSTATUS MII_Autonegotiate(PMP_ADAPTER pAdapter)
{
    NTSTATUS Status = STATUS_SUCCESS;
    MII_Write(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_C, MII_REG_C_RESTART_AUTO_NEGOTIATION_MASK | MII_REG_C_AUTO_NEGOTIATION_ENABLE_MASK);
    return Status;
}


/*++
Routine Description:
    Configure autonegotiation advertisement register.
Argument:
    pAdapter  - Context parameter.
Return:
    Execution status.
--*/
NTSTATUS MII_SetAdvert(PMP_ADAPTER pAdapter)
{
    NTSTATUS Status = STATUS_SUCCESS;
    IMX_MII_LINK_PROPERTIES_t  LinkProperties;
    MII_REG_ANA_t          AdvertisingReg;
    MII_REG_S_t            StatusReg;
    MII_REG_BASET_1000_C_t Base1000TControlReg;

    // Logical AND of Required Properties from .INF file and current PHY capabilities
    LinkProperties.R = pAdapter->ENETDev_PHYDevice.PHYDev_LinkPropertiesToAdvertise.R & pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.R;
    
    DBG_PHY_DEV_PRINT_INFO("Required PHY capabilities: 0x%04X", pAdapter->ENETDev_PHYDevice.PHYDev_LinkPropertiesToAdvertise.R);
    DBG_PHY_DEV_PRINT_INFO("Supported PHY capabilities: 0x%04X", pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.R);
    DBG_PHY_DEV_PRINT_INFO("PHY capabilities to advertise: 0x%04X", LinkProperties.R);

    AdvertisingReg.R = 1;                                                                  // Selector field = 802.3
    AdvertisingReg.B.ASYMETRIC_PAUSE = (UINT16)LinkProperties.B.ASYMETRIC_PAUSE;           // Asymmetric pause.
    AdvertisingReg.B.PAUSE = (UINT16)LinkProperties.B.PAUSE;                               // Pause.
    AdvertisingReg.B.BASE_TX_100_FD = (UINT16)LinkProperties.B.BASE_TX_100_FD;             // 100BASE-TX full duplex.
    AdvertisingReg.B.BASE_TX_100_HD = (UINT16)LinkProperties.B.BASE_TX_100_HD;             // 100BASE-TX half duplex.
    AdvertisingReg.B.BASE_T_10_HD = (UINT16)LinkProperties.B.BASE_T_10_FD;                 // 10BASE-T full duplex.
    AdvertisingReg.B.BASE_T_10_FD = (UINT16)LinkProperties.B.BASE_T_10_HD;                 // 10BASE-T half duplex.
    MII_Write(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_ANA, AdvertisingReg.R);

    // Read Phy Status register*/
    StatusReg.R = MII_Read(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_S);

    if (StatusReg.B.EXTENDED_STATUS) {
        Base1000TControlReg.R = 0;
        Base1000TControlReg.B.BASE_T_1000_FD = (UINT16)LinkProperties.B.BASE_T_1000_FD;  // 1000BASE-T full duplex.
        Base1000TControlReg.B.BASE_T_1000_HD = (UINT16)LinkProperties.B.BASE_T_1000_HD;  // 1000BASE-T half duplex.
        MII_Write(pAdapter, pAdapter->MiiCfg.PhyAddr, MII_REG_BASET_1000_C, Base1000TControlReg.R);
    }

    return Status;
}

/*++
Routine Description:
    Reset the PHY.
Argument:
    pAdapter  - Context parameter.
    PhyAddr   - Required PHY address.
Return:
    Execution status.
--*/
NTSTATUS MII_ResetPhy(PMP_ADAPTER pAdapter, UINT8 PhyAddr)
{
    NTSTATUS Status = STATUS_SUCCESS;

    MII_Write(pAdapter, PhyAddr, MII_REG_C, MII_REG_C_RESET_MASK);
    return Status;
}

/*++
Routine Description:
    Vendor specific PHY initialization.
Argument:
    pAdapter  - Context parameter.
    VendorOUI - Phy VendorOUI.
    Model     - PHY Model.
Return:
    Execution status.
--*/
NTSTATUS MII_PhySpecificInit(PMP_ADAPTER pAdapter)
{
    NTSTATUS Status = STATUS_SUCCESS;

    switch (pAdapter->ENETDev_PHYDevice.PhyVendor) {
        case REALTEK:
            switch (pAdapter->ENETDev_PHYDevice.PhyModel) {
                case RTL8211F:
                case RTL8211F_VD_CG:
                    DBG_PHY_DEV_PRINT_INFO("Detected Realtek RTL8211F");
                    MII_Rtl8211fInit(pAdapter);
                break;
                default:
                    DBG_PHY_DEV_PRINT_WARNING("Unknown Realtek PHY Model: 0x%02X", pAdapter->ENETDev_PHYDevice.PhyModel);
                    break;
            }
        break;
        default:
            DBG_PHY_DEV_PRINT_WARNING("Unknown PHY vendor: 0x%02X", pAdapter->ENETDev_PHYDevice.PhyVendor);
            break;
    }
    return Status;
}

/*++
Routine Description:
    Detect the PHY on the MDIO line.
Argument:
    pAdapter  - Context parameter.
    PhyAddr   - Required PHY address.
    VendorOUI - Pointer to obtained VendorOUI.
    Model     - Pointer to obtained PHY Model.
Return:
    Execution status.
--*/
NTSTATUS MII_FindPhy(PMP_ADAPTER pAdapter, UINT8 PhyAddr)
{
    NTSTATUS Status = STATUS_NO_SUCH_DEVICE;
    UINT16 phyid1 = 0,phyid2 = 0;

    phyid1 = MII_Read(pAdapter, PhyAddr, MII_REG_PHYID1);
    phyid2 = MII_Read(pAdapter, PhyAddr, MII_REG_PHYID2);

    if (((phyid1 != 0) && (phyid1 != 0xFFFF)) &&
        ((phyid2 != 0) && (phyid2 != 0xFFFF))) {
        pAdapter->ENETDev_PHYDevice.PhyVendor = (phyid1 << 6U) | (UINT8) ((phyid2 >> 10U) & 0x3F);
        pAdapter->ENETDev_PHYDevice.PhyModel = (phyid2 >> 4U) & 0x3F;
        Status = STATUS_SUCCESS;
    }
    return Status;
}

/*++
Routine Description:
    Initialize PHY.
Argument:
    MiniportAdapterHandle - Context parameter.
--*/
NTSTATUS MII_PhyInit(void* handle)
{
    NTSTATUS Status = STATUS_SUCCESS;
    PMP_ADAPTER pAdapter = (PMP_ADAPTER)handle;

    // Find PHY
    for ( ; pAdapter->MiiCfg.PhyAddr < 32; pAdapter->MiiCfg.PhyAddr++) {
        if ((Status = MII_FindPhy(pAdapter, pAdapter->MiiCfg.PhyAddr)) == STATUS_SUCCESS) {
            // Reset Phy
            MII_ResetPhy(pAdapter, pAdapter->MiiCfg.PhyAddr);

            // Phy specific initialization
            MII_PhySpecificInit(pAdapter);

            // Get PHY Capabilities
            MII_GetPhyCapatibilities(pAdapter);

            if (pAdapter->ENETDev_PHYDevice.PHYDev_SupportedLinkProperties.B.AUTO_NEGOTIATION) {
                // Set Auto-Negotiation Advertising register*/
                MII_SetAdvert(pAdapter);

                // Start Autonegotiation
                MII_Autonegotiate(pAdapter);
            } else {
                // Forced mode
                DBG_PHY_DEV_PRINT_WARNING("PHY is not able to autonegotiate");
                Status = STATUS_NOT_SUPPORTED;
                break;
            }
#if DBG
            MII_DumpPhy(pAdapter);
#endif
            break;
        }
    }
    return Status;
}

/*++
Routine Description:
    MII state engine, Should be called periodically from a system thread.
Argument:
    MiniportAdapterHandle - Context parameter.
--*/
void MII_StateEngine(NDIS_HANDLE MiniportAdapterHandle)
{
    PMP_ADAPTER pAdapter = (PMP_ADAPTER)MiniportAdapterHandle;
    MP_PHY_DEVICE* pPHYDev = &((PMP_ADAPTER)MiniportAdapterHandle)->ENETDev_PHYDevice;
    MP_PHY_LINK_STATE LinkState;
    BOOLEAN LinkStateChanged = FALSE;

    // Get current link state
    MII_GetLinkStatus(pAdapter, &LinkStateChanged, &LinkState);

    if (LinkStateChanged == TRUE) {
        DBG_PHY_DEV_PRINT_INFO("Link state change detected !!!");
        switch (LinkState) {
            case MP_PHY_LINK_UP:
                // Link is UP
                DBG_PHY_DEV_PRINT_INFO("New state is Link UP, resolving speed and duplex mode...");
                pAdapter->ENETDev_PHYDevice.PHYDev_AdvertisedLinkProperties.R = MII_GetAdvertisedLinkProperties(pAdapter);
                pAdapter->ENETDev_PHYDevice.PHYDev_LinkPartnerLinkProperties.R = MII_GetLinkPartnerLinkProperties(pAdapter);
                DBG_PHY_DEV_PRINT_LINK_PROPERTIES("AdvertisedLinkProperties  ", pPHYDev->PHYDev_AdvertisedLinkProperties);
                DBG_PHY_DEV_PRINT_LINK_PROPERTIES("LinkPartnerLinkProperties ", pPHYDev->PHYDev_LinkPartnerLinkProperties);
                // Compare locally advertised and remote link partner negotiated abilities. Choose the fastest one and return choosen mode to indicate to NDIS
                pPHYDev->PHYDev_LinkState.R = MII_ResolveSpeedMode(pPHYDev);
                // Set new speed and duplex mode in HW
                EnetQos_OnLinkStateChanged(pAdapter);
            break;

            case MP_PHY_LINK_DOWN:
                // Link is Down
                DBG_PHY_DEV_PRINT_INFO("New state is Link DOWN");

                // Set internal Link state variable accordingly to indicate to NDIS
                pPHYDev->PHYDev_LinkState.R = IMX_MII_LINK_STATE_DUPLEX_MODE_SET(MediaDuplexStateUnknown) |
                                              IMX_MII_LINK_STATE_SPEED_MHZ_SET(0) |
                                              IMX_MII_LINK_STATE_CONNECT_STATE_SET(MediaConnectStateDisconnected);

                // Restart Autonegotiation
                MII_Autonegotiate(pAdapter);
            break;
        }
    }
    else {
        DBG_PHY_DEV_PRINT_INFO("Current link state: %d", LinkState);
    }

#if DBG
    //MII_DumpPhy(pAdapter);
#endif
}