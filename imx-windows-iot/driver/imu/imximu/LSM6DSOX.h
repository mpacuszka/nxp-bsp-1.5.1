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

#pragma once

#include "WTypesbase.h"

// DATA register bits - the data report includes adjacent data registers, for a total of 6 bytes
#define LSM6DSOX_DATA_REPORT_SIZE_BYTES         6
#define LSM6DSOX_DATA_REPORT_SIZE_BYTES_HYBRID  12


#define LSM6DSOX_FUNC_CFG_ACCESS                0x01
#define LSM6DSOX_PIN_CTRL                       0x02
/* RESERVED */
#define LSM6DSOX_S4S_TPH_L                      0x04
#define LSM6DSOX_S4S_TPH_H                      0x05
#define LSM6DSOX_S4S_RR                         0x06
#define LSM6DSOX_FIFO_CTRL1                     0x07
#define LSM6DSOX_FIFO_CTRL2                     0x08
#define LSM6DSOX_FIFO_CTRL3                     0x09
#define LSM6DSOX_FIFO_CTRL4                     0x0A
#define LSM6DSOX_COUNTER_BDR_REG1               0x0B
#define LSM6DSOX_COUNTER_BDR_REG2               0x0C
#define LSM6DSOX_INT1_CTRL                      0x0D
#define LSM6DSOX_INT2_CTRL                      0x0E
#define LSM6DSOX_WHO_AM_I                       0x0F
#define LSM6DSOX_CTRL1_XL                       0x10
#define LSM6DSOX_CTRL2_G                        0x11
#define LSM6DSOX_CTRL3_C                        0x12
#define LSM6DSOX_CTRL4_C                        0x13
#define LSM6DSOX_CTRL5_C                        0x14
#define LSM6DSOX_CTRL6_C                        0x15
#define LSM6DSOX_CTRL7_G                        0x16
#define LSM6DSOX_CTRL8_XL                       0x17
#define LSM6DSOX_CTRL9_XL                       0x18
#define LSM6DSOX_CTRL10_C                       0x19
#define LSM6DSOX_ALL_INT_SRC                    0x1A
#define LSM6DSOX_WAKE_UP_SRC                    0x1B
#define LSM6DSOX_TAP_SRC                        0x1C
#define LSM6DSOX_D6D_SRC                        0x1D
#define LSM6DSOX_STATUS_REG                     0x1E
/* RESERVED */
#define LSM6DSOX_OUT_TEMP_L                     0x20
#define LSM6DSOX_OUT_TEMP_                      0x21
#define LSM6DSOX_OUTX_L_G                       0x22
#define LSM6DSOX_OUTX_H_G                       0x23
#define LSM6DSOX_OUTY_L_G                       0x24
#define LSM6DSOX_OUTY_H_G                       0x25
#define LSM6DSOX_OUTZ_L_G                       0x26
#define LSM6DSOX_OUTZ_H_G                       0x27
#define LSM6DSOX_OUTX_L_A                       0x28
#define LSM6DSOX_OUTX_H_A                       0x29
#define LSM6DSOX_OUTY_L_A                       0x2A
#define LSM6DSOX_OUTY_H_A                       0x2B
#define LSM6DSOX_OUTZ_L_A                       0x2C
#define LSM6DSOX_OUTZ_H_A                       0x2D
/* RESERVED */
#define LSM6DSOX_EMB_FUNC_STATUS_MAINPAGE       0x35
#define LSM6DSOX_FSM_STATUS_A_MAINPAGE          0x36
#define LSM6DSOX_FSM_STATUS_B_MAINPAGE          0x37
#define LSM6DSOX_MLC_STATUS_MAINPAGE            0x38
#define LSM6DSOX_STATUS_MASTER_MAINPAGE         0x39 
#define LSM6DSOX_FIFO_STATUS1                   0x3A 
#define LSM6DSOX_FIFO_STATUS2                   0x3B 
/* RESERVED */
#define LSM6DSOX_TIMESTAMP0                     0x40
#define LSM6DSOX_TIMESTAMP1                     0x41
#define LSM6DSOX_TIMESTAMP2                     0x42
#define LSM6DSOX_TIMESTAMP3                     0x43
/* RESERVED */
#define LSM6DSOX_UI_STATUS_REG_OIS              0x49
#define LSM6DSOX_UI_OUTX_L_G_OIS                0x4A
#define LSM6DSOX_UI_OUTX_H_G_OIS                0x4B
#define LSM6DSOX_UI_OUTY_L_G_OIS                0x4C
#define LSM6DSOX_UI_OUTY_H_G_OIS                0x4D
#define LSM6DSOX_UI_OUTZ_L_G_OIS                0x4E
#define LSM6DSOX_UI_OUTZ_H_G_OIS                0x4F
#define LSM6DSOX_UI_OUTX_L_A_OIS                0x50
#define LSM6DSOX_UI_OUTX_H_A_OIS                0x51
#define LSM6DSOX_UI_OUTY_L_A_OIS                0x52
#define LSM6DSOX_UI_OUTY_H_A_OIS                0x53
#define LSM6DSOX_UI_OUTZ_L_A_OIS                0x54
#define LSM6DSOX_UI_OUTZ_H_A_OIS                0x55
#define LSM6DSOX_TAP_CFG0                       0x56
#define LSM6DSOX_TAP_CFG1                       0x57
#define LSM6DSOX_TAP_CFG2                       0x58
#define LSM6DSOX_TAP_THS_6D                     0x59
#define LSM6DSOX_INT_DUR2                       0x5A
#define LSM6DSOX_WAKE_UP_THS                    0x5B
#define LSM6DSOX_WAKE_UP_DUR                    0x5C
#define LSM6DSOX_FREE_FALL                      0x5D
#define LSM6DSOX_MD1_CFG                        0x5E
#define LSM6DSOX_MD2_CFG                        0x5F
#define LSM6DSOX_S4S_ST_CMD_CODE                0x60
#define LSM6DSOX_S4S_DT_REG                     0x61
#define LSM6DSOX_I3C_BUS_AVB                    0x62
#define LSM6DSOX_INTERNAL_FREQ_FINE             0x63
/* RESERVED */
#define LSM6DSOX_UI_INT_OIS                     0x6F
#define LSM6DSOX_UI_CTRL1_OIS                   0x70
#define LSM6DSOX_UI_CTRL2_OIS                   0x71
#define LSM6DSOX_UI_CTRL3_OIS                   0x72
#define LSM6DSOX_X_OFS_USR                      0x73
#define LSM6DSOX_Y_OFS_USR                      0x74
#define LSM6DSOX_Z_OFS_USR                      0x75
/* RESERVED */
#define LSM6DSOX_FIFO_DATA_OUT_TAG              0x78
#define LSM6DSOX_FIFO_DATA_OUT_X_L              0x79
#define LSM6DSOX_FIFO_DATA_OUT_X_H              0x7A
#define LSM6DSOX_FIFO_DATA_OUT_Y_L              0x7B
#define LSM6DSOX_FIFO_DATA_OUT_Y_H              0x7C
#define LSM6DSOX_FIFO_DATA_OUT_Z_L              0x7D
#define LSM6DSOX_FIFO_DATA_OUT_Z_H              0x7E

/* I2C Addresses */
#define LSM6DSOX_DEVICE_ADDR_SA_0              (0x6A) 
#define LSM6DSOX_DEVICE_ADDR_SA_1              (0x6B) 
/* WHO AM I fixed value */
#define LSM6DSOX_WHO_AM_I_PROD_VALUE           (0x6C) 


/**
 * The following are the macro definitions to address each bit and its value in the hardware registers.
 */

 /*--------------------------------
 ** Register: FIFO_CTRL2
 ** Enum: LSM6DSOX_FIFO_CTRL2
 ** --
 ** Offset : 0x08 - FIFO control register 2.
 ** ------------------------------*/

#define LSM6DSOX_FIFO_CTRL2_STOP_ON_WTM         ((uint8_t) 0x80)
#define LSM6DSOX_FIFO_CTRL2_FIFO_COMPR_RT_EN    ((uint8_t) 0x40)

 /*--------------------------------
 ** Register: FIFO_CTRL3
 ** Enum: LSM6DSOX_FIFO_CTRL3
 ** --
 ** Offset : 0x09 - FIFO control register 3.
 ** ------------------------------*/
typedef union {
    struct {
        uint8_t         BDR_XL : 4;
        uint8_t         BDR_GY : 4;
    } b;
    uint8_t w;
} LSM6DSOX_FIFO_CTRL3_t;


/*
** FIFO_CTRL3 - Bit field mask definitions
*/
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_MASK        ((uint8_t) 0x0F)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_SHIFT       ((uint8_t)    0)

#define LSM6DSOX_FIFO_CTRL3_BDR_GY_MASK        ((uint8_t) 0xF0)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_SHIFT       ((uint8_t)    4)

#define LSM6DSOX_FIFO_CTRL3_XL_DATARATE_SHIFT  ((uint8_t)    4)
/*------------------------------*/

/*
** FIFO_CTRL3 - Bit field value definitions
*/
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_NOT_BATCHED    ((uint8_t) 0x00)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_12P5_HZ        ((uint8_t) 0x01)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_26_HZ          ((uint8_t) 0x02)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_52_HZ          ((uint8_t) 0x03)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_104_HZ         ((uint8_t) 0x04)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_208_HZ         ((uint8_t) 0x05)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_417_HZ         ((uint8_t) 0x06)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_833_HZ         ((uint8_t) 0x07)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_1667_HZ        ((uint8_t) 0x08)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_3333_HZ        ((uint8_t) 0x09)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_6667_HZ        ((uint8_t) 0x0A)
#define LSM6DSOX_FIFO_CTRL3_BDR_XL_1P6_HZ         ((uint8_t) 0x0B)

#define LSM6DSOX_FIFO_CTRL3_BDR_GY_NOT_BATCHED    ((uint8_t) 0x00)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_12P5_HZ        ((uint8_t) 0x10)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_26_HZ          ((uint8_t) 0x20)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_52_HZ          ((uint8_t) 0x30)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_104_HZ         ((uint8_t) 0x40)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_208_HZ         ((uint8_t) 0x50)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_417_HZ         ((uint8_t) 0x60)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_833_HZ         ((uint8_t) 0x70)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_1667_HZ        ((uint8_t) 0x80)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_3333_HZ        ((uint8_t) 0x90)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_6667_HZ        ((uint8_t) 0xA0)
#define LSM6DSOX_FIFO_CTRL3_BDR_GY_6P5_HZ         ((uint8_t) 0xB0)
/*------------------------------*/



 /*--------------------------------
 ** Register: FIFO_CTRL4
 ** Enum: LSM6DSOX_FIFO_CTRL4
 ** --
 ** Offset : 0x0A - FIFO control register 4.
 ** ------------------------------*/
typedef union {
    struct {
        uint8_t            FIFO_MODE : 3;
        uint8_t _reserved_ : 1;
        uint8_t          ODR_T_BATCH : 2;
        uint8_t         DEC_TS_BATCH : 2;
    } b;
    uint8_t w;
} LSM6DSOX_FIFO_CTRL4_t;


/*
** FIFO_CTRL4 - Bit field mask definitions
*/
#define LSM6DSOX_FIFO_CTRL4_FIFO_MODE_MASK        ((uint8_t) 0x07)
#define LSM6DSOX_FIFO_CTRL4_FIFO_MODE_SHIFT       ((uint8_t)    0)

#define LSM6DSOX_FIFO_CTRL4_ODR_T_BATCH_MASK      ((uint8_t) 0x30)
#define LSM6DSOX_FIFO_CTRL4_ODR_T_BATCH_SHIFT     ((uint8_t)    4)

#define LSM6DSOX_FIFO_CTRL4_DEC_TS_BATCH_MASK     ((uint8_t) 0xC0)
#define LSM6DSOX_FIFO_CTRL4_DEC_TS_BATCH_SHIFT    ((uint8_t)    6)
/*------------------------------*/

/*
** FIFO_CTRL4 - Bit field value definitions
*/
#define LSM6DSOX_FIFO_CTRL4_FIFO_MODE_DISABLE       ((uint8_t) 0x00)
#define LSM6DSOX_FIFO_CTRL4_FIFO_MODE_ENABLE        ((uint8_t) 0x01)
/*------------------------------*/



/*--------------------------------
** Register: INT1_CTRL
** Enum: LSM6DSOX_INT1_CTRL
** --
** Offset : 0x0D - Interrupt Control 1 register.
** ------------------------------*/
typedef union {
    struct {
        uint8_t               DRDY_XL : 1;   
        uint8_t                DRDY_G : 1;
        uint8_t                  BOOT : 1;
        uint8_t               FIFO_TH : 1;
        uint8_t              FIFO_OVR : 1;
        uint8_t             FIFO_FULL : 1;
        uint8_t               CNT_BDR : 1;
        uint8_t         DEN_DRDY_flag : 1;
    } b;
    uint8_t w;
} LSM6DSOX_INT1_CTRL_t;


/*
** INT1_CTRL - Bit field mask definitions
*/
#define LSM6DSOX_INT1_CTRL_DRDY_XL_MASK        ((uint8_t) 0x01)
#define LSM6DSOX_INT1_CTRL_DRDY_XL_SHIFT       ((uint8_t)    0)

#define LSM6DSOX_INT1_CTRL_DRDY_G_MASK         ((uint8_t) 0x02)
#define LSM6DSOX_INT1_CTRL_DRDY_G_SHIFT        ((uint8_t)    1)

#define LSM6DSOX_INT1_CTRL_BOOT_MASK           ((uint8_t) 0x04)
#define LSM6DSOX_INT1_CTRL_BOOT_SHIFT          ((uint8_t)    2)

#define LSM6DSOX_INT1_CTRL_FIFO_TH_MASK        ((uint8_t) 0x08)
#define LSM6DSOX_INT1_CTRL_FIFO_TH_SHIFT       ((uint8_t)    3)

#define LSM6DSOX_INT1_CTRL_FIFO_OVR_MASK       ((uint8_t) 0x10)
#define LSM6DSOX_INT1_CTRL_FIFO_OVR_SHIFT      ((uint8_t)    4)

#define LSM6DSOX_INT1_CTRL_FIFO_FULL_MASK      ((uint8_t) 0x20)
#define LSM6DSOX_INT1_CTRL_FIFO_FULL_SHIFT     ((uint8_t)    5)

#define LSM6DSOX_INT1_CTRL_CNT_BDR_MASK        ((uint8_t) 0x40)
#define LSM6DSOX_INT1_CTRL_CNT_BDR_SHIFT       ((uint8_t)    6)

#define LSM6DSOX_INT1_CTRL_DEN_DRDY_FLAG_MASK  ((uint8_t) 0x80)
#define LSM6DSOX_INT1_CTRL_DEN_DRDY_FLAG_SHIFT ((uint8_t)    7)
/*------------------------------*/

/*
** INT1_CTRL - Bit field value definitions
*/
#define LSM6DSOX_INT1_CTRL_DRDY_XL_DISABLE       ((uint8_t) 0x00)
#define LSM6DSOX_INT1_CTRL_DRDY_XL_ENABLE        ((uint8_t) 0x01)
#define LSM6DSOX_INT1_CTRL_DRDY_G_DISABLE        ((uint8_t) 0x00)
#define LSM6DSOX_INT1_CTRL_DRDY_G_ENABLE         ((uint8_t) 0x02)
#define LSM6DSOX_INT1_CTRL_DRDY_TEMP_DISABLE     ((uint8_t) 0x00)
#define LSM6DSOX_INT1_CTRL_DRDY_TEMP_ENABLE      ((uint8_t) 0x04)
#define LSM6DSOX_INT1_CTRL_FIFO_TH_DISABLE       ((uint8_t) 0x00)
#define LSM6DSOX_INT1_CTRL_FIFO_TH_ENABLE        ((uint8_t) 0x08)
#define LSM6DSOX_INT1_CTRL_FIFO_OVR_DISABLE      ((uint8_t) 0x00)
#define LSM6DSOX_INT1_CTRL_FIFO_OVR_ENABLE       ((uint8_t) 0x10)
#define LSM6DSOX_INT1_CTRL_FIFO_FULL_DISABLE     ((uint8_t) 0x00)
#define LSM6DSOX_INT1_CTRL_FIFO_FULL_ENABLE      ((uint8_t) 0x20)
#define LSM6DSOX_INT1_CTRL_CNT_BDR_DISABLE       ((uint8_t) 0x00)
#define LSM6DSOX_INT1_CTRL_CNT_BDR_ENABLE        ((uint8_t) 0x40)
#define LSM6DSOX_INT1_CTRL_DEN_DRDY_FLAG_DISABLE ((uint8_t) 0x00)
#define LSM6DSOX_INT1_CTRL_DEN_DRDY_FLAG_ENABLE  ((uint8_t) 0x80)
/*------------------------------*/



/*--------------------------------
** Register: INT2_CTRL
** Enum: LSM6DSOX_INT2_CTRL
** --
** Offset : 0x0D - Interrupt Control 1 register.
** ------------------------------*/
typedef union {
    struct {
        uint8_t               DRDY_XL : 1;
        uint8_t                DRDY_G : 1;
        uint8_t             DRDY_TEMP : 1;
        uint8_t               FIFO_TH : 1;
        uint8_t              FIFO_OVR : 1;
        uint8_t             FIFO_FULL : 1;
        uint8_t               CNT_BDR : 1;
    } b;
    uint8_t w;
} LSM6DSOX_INT2_CTRL_t;


/*
** INT2_CTRL - Bit field mask definitions
*/
#define LSM6DSOX_INT2_CTRL_DRDY_XL_MASK        ((uint8_t) 0x01)
#define LSM6DSOX_INT2_CTRL_DRDY_XL_SHIFT       ((uint8_t)    0)

#define LSM6DSOX_INT2_CTRL_DRDY_G_MASK         ((uint8_t) 0x02)
#define LSM6DSOX_INT2_CTRL_DRDY_G_SHIFT        ((uint8_t)    1)

#define LSM6DSOX_INT2_CTRL_DRDY_TEMP_MASK      ((uint8_t) 0x04)
#define LSM6DSOX_INT2_CTRL_DRDY_TEMP_SHIFT     ((uint8_t)    2)

#define LSM6DSOX_INT2_CTRL_FIFO_TH_MASK        ((uint8_t) 0x08)
#define LSM6DSOX_INT2_CTRL_FIFO_TH_SHIFT       ((uint8_t)    3)

#define LSM6DSOX_INT2_CTRL_FIFO_OVR_MASK       ((uint8_t) 0x10)
#define LSM6DSOX_INT2_CTRL_FIFO_OVR_SHIFT      ((uint8_t)    4)

#define LSM6DSOX_INT2_CTRL_FIFO_FULL_MASK      ((uint8_t) 0x20)
#define LSM6DSOX_INT2_CTRL_FIFO_FULL_SHIFT     ((uint8_t)    5)

#define LSM6DSOX_INT2_CTRL_CNT_BDR_MASK        ((uint8_t) 0x40)
#define LSM6DSOX_INT2_CTRL_CNT_BDR_SHIFT       ((uint8_t)    6)
/*------------------------------*/

/*
** INT2_CTRL - Bit field value definitions
*/
#define LSM6DSOX_INT2_CTRL_DRDY_XL_DISABLE   ((uint8_t) 0x00)
#define LSM6DSOX_INT2_CTRL_DRDY_XL_ENABLE    ((uint8_t) 0x01)
#define LSM6DSOX_INT2_CTRL_DRDY_G_DISABLE    ((uint8_t) 0x00)
#define LSM6DSOX_INT2_CTRL_DRDY_G_ENABLE     ((uint8_t) 0x02)
#define LSM6DSOX_INT2_CTRL_DRDY_TEMP_DISABLE ((uint8_t) 0x00)
#define LSM6DSOX_INT2_CTRL_DRDY_TEMP_ENABLE  ((uint8_t) 0x04)
#define LSM6DSOX_INT2_CTRL_FIFO_TH_DISABLE   ((uint8_t) 0x00)
#define LSM6DSOX_INT2_CTRL_FIFO_TH_ENABLE    ((uint8_t) 0x08)
#define LSM6DSOX_INT2_CTRL_FIFO_OVR_DISABLE  ((uint8_t) 0x00)
#define LSM6DSOX_INT2_CTRL_FIFO_OVR_ENABLE   ((uint8_t) 0x10)
#define LSM6DSOX_INT2_CTRL_FIFO_FULL_DISABLE ((uint8_t) 0x00)
#define LSM6DSOX_INT2_CTRL_FIFO_FULL_ENABLE  ((uint8_t) 0x20)
#define LSM6DSOX_INT2_CTRL_CNT_BDR_DISABLE   ((uint8_t) 0x00)
#define LSM6DSOX_INT2_CTRL_CNT_BDR_ENABLE    ((uint8_t) 0x40)
/*------------------------------*/



/*--------------------------------
** Register: CTRL1_XL
** Enum: LSM6DSOX_CTRL1_XL
** --
** Offset : 0x10 - Accelerometer control register 1.
** ------------------------------*/
typedef union {
    struct {
        uint8_t _reserved_ : 1;
        uint8_t            LPF2_XL_EN : 1;
        uint8_t                 FS_XL : 2;
        uint8_t                ODR_XL : 4;
    } b;
    uint8_t w;
} LSM6DSOX_CTRL1_XL_t;


/*
** CTRL1_XL - Bit field mask definitions
*/
#define LSM6DSOX_CTRL1_XL_LPF_EN_MASK     ((uint8_t) 0x02)
#define LSM6DSOX_CTRL1_XL_LPF_EN_SHIFT    ((uint8_t)    1)

#define LSM6DSOX_CTRL1_XL_FS_MASK         ((uint8_t) 0x0C)
#define LSM6DSOX_CTRL1_XL_FS_SHIFT        ((uint8_t)    6)

#define LSM6DSOX_CTRL1_XL_ODR_MASK        ((uint8_t) 0xF0)
#define LSM6DSOX_CTRL1_XL_ODR_SHIFT       ((uint8_t)    7)
/*------------------------------*/

/*
** CTRL1_XL - Bit field value definitions
*/
#define LSM6DSOX_CTRL1_XL_LPF2_EN_DISABLE    ((uint8_t) 0x00)
#define LSM6DSOX_CTRL1_XL_LPF2_EN_ENABLE     ((uint8_t) 0x02)

#define LSM6DSOX_CTRL1_XL_FS_MODE_OLD_2G     ((uint8_t) 0x00)
#define LSM6DSOX_CTRL1_XL_FS_MODE_OLD_16G    ((uint8_t) 0x04)
#define LSM6DSOX_CTRL1_XL_FS_MODE_OLD_4G     ((uint8_t) 0x08)
#define LSM6DSOX_CTRL1_XL_FS_MODE_OLD_8G     ((uint8_t) 0x0C)
#define LSM6DSOX_CTRL1_XL_FS_MODE_NEW_2G     ((uint8_t) 0x00)
#define LSM6DSOX_CTRL1_XL_FS_MODE_NEW_4G     ((uint8_t) 0x08)
#define LSM6DSOX_CTRL1_XL_FS_MODE_NEW_8G     ((uint8_t) 0x0C)

#define LSM6DSOX_CTRL1_XL_ODR_POWER_DOWN     ((uint8_t) 0x00)
#define LSM6DSOX_CTRL1_XL_ODR_1P6_HZ         ((uint8_t) 0xB0)
#define LSM6DSOX_CTRL1_XL_ODR_12P5_HZ        ((uint8_t) 0x10)
#define LSM6DSOX_CTRL1_XL_ODR_26_HZ          ((uint8_t) 0x20)
#define LSM6DSOX_CTRL1_XL_ODR_52_HZ          ((uint8_t) 0x30)
#define LSM6DSOX_CTRL1_XL_ODR_104_HZ         ((uint8_t) 0x40)
#define LSM6DSOX_CTRL1_XL_ODR_208_HZ         ((uint8_t) 0x50)
#define LSM6DSOX_CTRL1_XL_ODR_416_HZ         ((uint8_t) 0x60)
#define LSM6DSOX_CTRL1_XL_ODR_833_HZ         ((uint8_t) 0x70)
#define LSM6DSOX_CTRL1_XL_ODR_1P6_KHZ        ((uint8_t) 0x80)
#define LSM6DSOX_CTRL1_XL_ODR_3P33_KHZ       ((uint8_t) 0x90)
#define LSM6DSOX_CTRL1_XL_ODR_6P66_KHZ       ((uint8_t) 0xA0)
/*------------------------------*/



/*--------------------------------
** Register: CTRL2_G
** Enum: LSM6DSOX_CTRL2_G
** --
** Offset : 0x11 - Gyroscope control register 2.
** ------------------------------*/
typedef union {
    struct {
        uint8_t _reserved_ : 1;
        uint8_t               FS_125 : 1;
        uint8_t                 FS_G : 2;
        uint8_t                ODR_G : 4;
    } b;
    uint8_t w;
} LSM6DSOX_CTRL2_G_t;


/*
** CTRL2_G - Bit field mask definitions
*/
#define LSM6DSOX_CTRL2_G_FS_125_MASK     ((uint8_t) 0x02)
#define LSM6DSOX_CTRL2_G_FS_125_SHIFT    ((uint8_t)    1)

#define LSM6DSOX_CTRL2_G_FS_MASK         ((uint8_t) 0x0C)
#define LSM6DSOX_CTRL2_G_FS_SHIFT        ((uint8_t)    6)

#define LSM6DSOX_CTRL2_G_ODR_MASK        ((uint8_t) 0xF0)
#define LSM6DSOX_CTRL2_G_ODR_SHIFT       ((uint8_t)    7)
/*------------------------------*/

/*
** CTRL2_G - Bit field value definitions
*/
#define LSM6DSOX_CTRL2_G_FS_125_SELECT_FS_G ((uint8_t) 0x00)
#define LSM6DSOX_CTRL2_G_FS_125_DPS         ((uint8_t) 0x02)

#define LSM6DSOX_CTRL2_G_FS_MODE_250_DPS    ((uint8_t) 0x00)
#define LSM6DSOX_CTRL2_G_FS_MODE_500_DPS    ((uint8_t) 0x04)
#define LSM6DSOX_CTRL2_G_FS_MODE_1000_DPS   ((uint8_t) 0x08)
#define LSM6DSOX_CTRL2_G_FS_MODE_2000_DPS   ((uint8_t) 0x0C)

#define LSM6DSOX_CTRL2_G_ODR_POWER_DOWN     ((uint8_t) 0x00)
#define LSM6DSOX_CTRL2_G_ODR_12P5_HZ        ((uint8_t) 0x10)
#define LSM6DSOX_CTRL2_G_ODR_26_HZ          ((uint8_t) 0x20)
#define LSM6DSOX_CTRL2_G_ODR_52_HZ          ((uint8_t) 0x30)
#define LSM6DSOX_CTRL2_G_ODR_104_HZ         ((uint8_t) 0x40)
#define LSM6DSOX_CTRL2_G_ODR_208_HZ         ((uint8_t) 0x50)
#define LSM6DSOX_CTRL2_G_ODR_416_HZ         ((uint8_t) 0x60)
#define LSM6DSOX_CTRL2_G_ODR_833_HZ         ((uint8_t) 0x70)
#define LSM6DSOX_CTRL2_G_ODR_1P6_KHZ        ((uint8_t) 0x80)
#define LSM6DSOX_CTRL2_G_ODR_3P33_KHZ       ((uint8_t) 0x90)
#define LSM6DSOX_CTRL2_G_ODR_6P66_KHZ       ((uint8_t) 0xA0)
/*------------------------------*/



/*--------------------------------
** Register: CTRL3_C
** Enum: LSM6DSOX_CTRL3_C
** --
** Offset : 0x12 - Control register 3.
** ------------------------------*/
typedef union {
    struct {
        uint8_t           SW_RESET : 1;
        uint8_t _reserved_ : 1;
        uint8_t             IF_INC : 1;
        uint8_t                SIM : 1;
        uint8_t              PP_OD : 1;
        uint8_t          H_LACTIVE : 1;
        uint8_t                BDU : 1;
        uint8_t               BOOT : 1;
    } b;
    uint8_t w;
} LSM6DSOX_CTRL3_C_t;


/*
** CTRL3_C - Bit field mask definitions
*/
#define LSM6DSOX_CTRL3_C_SW_RESET_MASK   ((uint8_t) 0x01)
#define LSM6DSOX_CTRL3_C_SW_RESET_SHIFT  ((uint8_t)    0)

#define LSM6DSOX_CTRL3_C_IF_INC_MASK     ((uint8_t) 0x04)
#define LSM6DSOX_CTRL3_C_IF_INC_SHIFT    ((uint8_t)    2)

#define LSM6DSOX_CTRL3_C_SIM_MASK        ((uint8_t) 0x08)
#define LSM6DSOX_CTRL3_C_SIM_SHIFT       ((uint8_t)    3)

#define LSM6DSOX_CTRL3_C_PP_OD_MASK      ((uint8_t) 0x10)
#define LSM6DSOX_CTRL3_C_PP_OD_SHIFT     ((uint8_t)    4)

#define LSM6DSOX_CTRL3_C_H_LACTIVE_MASK  ((uint8_t) 0x20)
#define LSM6DSOX_CTRL3_C_H_LACTIVE_SHIFT ((uint8_t)    5)

#define LSM6DSOX_CTRL3_C_BDU_MASK        ((uint8_t) 0x40)
#define LSM6DSOX_CTRL3_C_BDU_SHIFT       ((uint8_t)    6)

#define LSM6DSOX_CTRL3_C_BOOT_MASK       ((uint8_t) 0x80)
#define LSM6DSOX_CTRL3_C_BOOT_SHIFT      ((uint8_t)    7)
/*------------------------------*/

/*
** CTRL3_C - Bit field value definitions
*/
#define LSM6DSOX_CTRL3_C_SW_RESET_NORMAL_MODE  ((uint8_t) 0x00)
#define LSM6DSOX_CTRL3_C_SW_RESET_RESET_DEVICE ((uint8_t) 0x01)

#define LSM6DSOX_CTRL3_C_IF_INC_DISABLE        ((uint8_t) 0x00)
#define LSM6DSOX_CTRL3_C_IF_INC_ENABLE         ((uint8_t) 0x04)

#define LSM6DSOX_CTRL3_C_SIM_4_WIRE            ((uint8_t) 0x00)
#define LSM6DSOX_CTRL3_C_SIM_3_WIRE            ((uint8_t) 0x08)

#define LSM6DSOX_CTRL3_C_PP_OD_PUSH_PULL       ((uint8_t) 0x00)
#define LSM6DSOX_CTRL3_C_PP_OD_OPEN_DRAIN      ((uint8_t) 0x10)

#define LSM6DSOX_CTRL3_C_H_LACTIVE_ACTIVE_HIGH ((uint8_t) 0x00)
#define LSM6DSOX_CTRL3_C_H_LACTIVE_ACTIVE_LOW  ((uint8_t) 0x20)

#define LSM6DSOX_CTRL3_C_BDU_CONTINUOUS        ((uint8_t) 0x00)
#define LSM6DSOX_CTRL3_C_BDU_UNTIL_READ        ((uint8_t) 0x40)

#define LSM6DSOX_CTRL3_C_BOOT_NORMAL_MODE      ((uint8_t) 0x00)
#define LSM6DSOX_CTRL3_C_BOOT_REBOOT_MEMORY    ((uint8_t) 0x80)
/*------------------------------*/

/*--------------------------------
** Register: CTRL4_C
** Enum: LSM6DSOX_CTRL4_C
** --
** Offset : 0x14 - Control register 5.
** ------------------------------*/
typedef union {
    struct {
        uint8_t _reserved1_ : 1;
        uint8_t               LPF1_SEL_G : 1;
        uint8_t              I2C_DISABLE : 1;
        uint8_t                DRDY_MASK : 1;
        uint8_t _reserved2_ : 1;
        uint8_t             INT2_ON_INT1 : 1;
        uint8_t                  SLEEP_G : 1;
        uint8_t _reserved3_ : 1;
    } b;
    uint8_t w;
} LSM6DSOX_CTRL4_C_t;


/*
** CTRL4_C - Bit field mask definitions
*/
#define LSM6DSOX_CTRL4_C_LPF1_SEL_G_MASK       ((uint8_t) 0x02)
#define LSM6DSOX_CTRL4_C_LPF1_SEL_G_SHIFT      ((uint8_t)    1)

#define LSM6DSOX_CTRL4_C_I2C_DISABLE_MASK      ((uint8_t) 0x04)
#define LSM6DSOX_CTRL4_C_I2C_DISABLE_SHIFT     ((uint8_t)    2)

#define LSM6DSOX_CTRL4_C_DRDY_MASK_MASK        ((uint8_t) 0x08)
#define LSM6DSOX_CTRL4_C_DRDY_MASK_SHIFT       ((uint8_t)    3)

#define LSM6DSOX_CTRL4_C_INT2_ON_INT1_MASK     ((uint8_t) 0x20)
#define LSM6DSOX_CTRL4_C_INT2_ON_INT1_SHIFT    ((uint8_t)    5)

#define LSM6DSOX_CTRL4_C_SLEEP_G_MASK          ((uint8_t) 0x40)
#define LSM6DSOX_CTRL4_C_SLEEP_G_SHIFT         ((uint8_t)    6)
/*------------------------------*/

/*
** CTRL4_C - Bit field value definitions
*/
#define LSM6DSOX_CTRL4_C_DRDY_MASKY_DISABLE ((uint8_t) 0x00)
#define LSM6DSOX_CTRL4_C_DRDY_MASK_ENABLE   ((uint8_t) 0x08)
/*------------------------------*/

/*--------------------------------
** Register: CTRL5_C
** Enum: LSM6DSOX_CTRL5_C
** --
** Offset : 0x14 - Control register 5.
** ------------------------------*/
typedef union {
    struct {
        uint8_t                ST_XL : 2;
        uint8_t                 ST_G : 2;
        uint8_t      ROUNDING_STATUS : 1;
        uint8_t             ROUNDING : 2;
        uint8_t            XL_ULP_EN : 1;
    } b;
    uint8_t w;
} LSM6DSOX_CTRL5_C_t;


/*
** CTRL5_C - Bit field mask definitions
*/
#define LSM6DSOX_CTRL5_C_ST_XL_MASK            ((uint8_t) 0x07)
#define LSM6DSOX_CTRL5_C_ST_XL_SHIFT           ((uint8_t)    0)

#define LSM6DSOX_CTRL5_C_ST_G_MASK             ((uint8_t) 0x08)
#define LSM6DSOX_CTRL5_C_ST_G_SHIFT            ((uint8_t)    3)

#define LSM6DSOX_CTRL5_C_ROUNDING_STATUS_MASK  ((uint8_t) 0x10)
#define LSM6DSOX_CTRL5_C_ROUNDING_STATUS_SHIFT ((uint8_t)    4)

#define LSM6DSOX_CTRL5_C_ROUNDING_MASK         ((uint8_t) 0x60)
#define LSM6DSOX_CTRL5_C_ROUNDING_SHIFT        ((uint8_t)    5)

#define LSM6DSOX_CTRL5_C_XL_ULP_EN_MASK        ((uint8_t) 0x80)
#define LSM6DSOX_CTRL5_C_XL_ULP_EN_SHIFT       ((uint8_t)    7)
/*------------------------------*/

/*
** CTRL5_C - Bit field value definitions
*/
#define LSM6DSOX_CTRL5_C_ROUNDING_DISABLED ((uint8_t) 0x00)
#define LSM6DSOX_CTRL5_C_ROUNDING_ACC_ONLY ((uint8_t) 0x20)
#define LSM6DSOX_CTRL5_C_ROUNDING_GYR_ONLY ((uint8_t) 0x40)
#define LSM6DSOX_CTRL5_C_ROUNDING_ACC_GYR  ((uint8_t) 0x60)

#define LSM6DSOX_CTRL5_C_XL_ULP_DISABLE    ((uint8_t) 0x00)
#define LSM6DSOX_CTRL5_C_XL_ULP_ENABLE     ((uint8_t) 0x80)
/*------------------------------*/



/*--------------------------------
** Register: CTRL6_C
** Enum: LSM6DSOX_CTRL6_C
** --
** Offset : 0x15 - Control register 6.
** ------------------------------*/
typedef union {
    struct {
        uint8_t           FTYPE : 3;
        uint8_t       USR_OFF_W : 1;
        uint8_t      XL_HM_MODE : 1;
        uint8_t          LVL_EN : 2;
        uint8_t         TRIG_EN : 1;
    } b;
    uint8_t w;
} LSM6DSOX_CTRL6_C_t;


/*
** CTRL6_C - Bit field mask definitions
*/
#define LSM6DSOX_CTRL6_C_FTYPE_MASK       ((uint8_t) 0x07)
#define LSM6DSOX_CTRL6_C_FTYPE_SHIFT      ((uint8_t)    0)

#define LSM6DSOX_CTRL6_C_USR_OFF_W_MASK   ((uint8_t) 0x08)
#define LSM6DSOX_CTRL6_C_USR_OFF_W_SHIFT  ((uint8_t)    3)

#define LSM6DSOX_CTRL6_C_XL_HM_MODE_MASK  ((uint8_t) 0x10)
#define LSM6DSOX_CTRL6_C_XL_HM_MODE_SHIFT ((uint8_t)    4)

#define LSM6DSOX_CTRL6_C_LVL_EN_MASK      ((uint8_t) 0x60)
#define LSM6DSOX_CTRL6_C_LVL_EN_SHIFT     ((uint8_t)    5)

#define LSM6DSOX_CTRL6_C_TRIG_EN_MASK     ((uint8_t) 0x80)
#define LSM6DSOX_CTRL6_C_TRIG_EN_SHIFT    ((uint8_t)    7)
/*------------------------------*/

/*
** CTRL6_C - Bit field value definitions
*/
#define LSM6DSOX_CTRL6_C_HM_MODE_ENABLED  ((uint8_t) 0x00)
#define LSM6DSOX_CTRL6_C_HM_MODE_DISABLED ((uint8_t) 0x10)
/*------------------------------*/



/*--------------------------------
** Register: CTRL7_G
** Enum: LSM6DSOX_CTRL7_G
** --
** Offset : 0x16 - Control register 7.
** ------------------------------*/
typedef union {
    struct {
        uint8_t          OIS_ON : 1;
        uint8_t  USR_OFF_ON_OUT : 1;
        uint8_t       OIS_ON_EN : 1;
        uint8_t _reserved_ : 1;
        uint8_t           HPM_G : 2;
        uint8_t         HP_EN_G : 1;
        uint8_t       G_HM_MODE : 1;
    } b;
    uint8_t w;
} LSM6DSOX_CTRL7_G_t;


/*
** CTRL7_G - Bit field mask definitions
*/
#define LSM6DSOX_CTRL7_G_OIS_ON_MASK          ((uint8_t) 0x01)
#define LSM6DSOX_CTRL7_G_OIS_ON_SHIFT         ((uint8_t)    0)

#define LSM6DSOX_CTRL7_G_USR_OFF_ON_OUT_MASK  ((uint8_t) 0x02)
#define LSM6DSOX_CTRL7_G_USR_OFF_ON_OUT_SHIFT ((uint8_t)    1)

#define LSM6DSOX_CTRL7_G_XL_OIS_ON_EN_MASK    ((uint8_t) 0x04)
#define LSM6DSOX_CTRL7_G_XL_OIS_ON_EN_SHIFT   ((uint8_t)    2)

#define LSM6DSOX_CTRL7_G_HPM_G_MASK           ((uint8_t) 0x30)
#define LSM6DSOX_CTRL7_G_HPM_G_SHIFT          ((uint8_t)    4)

#define LSM6DSOX_CTRL7_G_HP_EN_G_MASK         ((uint8_t) 0x40)
#define LSM6DSOX_CTRL7_G_HP_EN_G_SHIFT        ((uint8_t)    6)

#define LSM6DSOX_CTRL7_G_HM_MODE_MASK         ((uint8_t) 0x80)
#define LSM6DSOX_CTRL7_G_HM_MODE_SHIFT        ((uint8_t)    7)
/*------------------------------*/

/*
** CTRL7_G - Bit field value definitions
*/
#define LSM6DSOX_CTRL7_G_HM_MODE_ENABLED  ((uint8_t) 0x00)
#define LSM6DSOX_CTRL7_G_HM_MODE_DISABLED ((uint8_t) 0x80)
/*------------------------------*/



/*--------------------------------
** Register: CTRL9_XL
** Enum: LSM6DSOX_CTRL9_XL
** --
** Offset : 0x18 - Control register 9.
** ------------------------------*/
typedef union {
    struct {
        uint8_t _reserved_ : 1;
        uint8_t     I3C_DISABLE : 1;
        uint8_t          DEN_LH : 1;
        uint8_t       DEN_XL_EN : 1;
        uint8_t        DEN_XL_G : 1;
        uint8_t           DEN_Z : 1;
        uint8_t           DEN_Y : 1;
        uint8_t           DEN_X : 1;
    } b;
    uint8_t w;
} LSM6DSOX_CTRL9_XL_t;


/*
** CTRL9_XL - Bit field mask definitions
*/
#define LSM6DSOX_CTRL9_XL_I3C_DISABLE_MASK  ((uint8_t) 0x02)
#define LSM6DSOX_CTRL9_XL_I3C_DISABLE_SHIFT ((uint8_t)    1)

#define LSM6DSOX_CTRL9_XL_DEN_LH_MASK       ((uint8_t) 0x04)
#define LSM6DSOX_CTRL9_XL_DEN_LH_SHIFT      ((uint8_t)    2)

#define LSM6DSOX_CTRL9_XL_DEN_XL_EN_MASK    ((uint8_t) 0x08)
#define LSM6DSOX_CTRL9_XL_DEN_XL_EN_SHIFT   ((uint8_t)    3)

#define LSM6DSOX_CTRL9_XL_DEN_XL_G_MASK     ((uint8_t) 0x10)
#define LSM6DSOX_CTRL9_XL_DEN_XL_G_SHIFT    ((uint8_t)    4)

#define LSM6DSOX_CTRL9_XL_DEN_Z_MASK        ((uint8_t) 0x20)
#define LSM6DSOX_CTRL9_XL_DEN_Z_SHIFT       ((uint8_t)    5)

#define LSM6DSOX_CTRL9_XL_DEN_Y_MASK        ((uint8_t) 0x40)
#define LSM6DSOX_CTRL9_XL_DEN_Y_SHIFT       ((uint8_t)    6)

#define LSM6DSOX_CTRL9_XL_DEN_X_MASK        ((uint8_t) 0x80)
#define LSM6DSOX_CTRL9_XL_DEN_X_SHIFT       ((uint8_t)    7)
/*------------------------------*/

/*
** CTRL9_XL - Bit field value definitions
*/
#define LSM6DSOX_CTRL9_XL_I3C_ENABLE    ((uint8_t) 0x00)
#define LSM6DSOX_CTRL9_XL_I3C_DISABLE   ((uint8_t) 0x20)
/*------------------------------*/



/*--------------------------------
** Register: ALL_INT_SRC
** Enum: LSM6DSOX_ALL_INT_SRC
** --
** Offset : 0x1A - Source register for all interrupts.
** ------------------------------*/
typedef union {
    struct {
        uint8_t                  FF_IA : 1;
        uint8_t                  WU_IA : 1;
        uint8_t             SINGLE_TAP : 1;
        uint8_t             DOUBLE_TAP : 1;
        uint8_t                 D6D_IA : 1;
        uint8_t        SLEEP_CHANGE_IA : 1;
        uint8_t _reserved_ : 1;
        uint8_t     TIMESTAMP_ENDCOUNT : 1;
    } b;
    uint8_t w;
} LSM6DSOX_ALL_INT_SRC_t;


/*
** ALL_INT_SRC - Bit field mask definitions
*/
#define LSM6DSOX_ALL_INT_SRC_FF_IA_MASK               ((uint8_t) 0x01)
#define LSM6DSOX_ALL_INT_SRC_FF_IA_SHIFT              ((uint8_t)    0)

#define LSM6DSOX_ALL_INT_SRC_WU_IA_MASK               ((uint8_t) 0x02)
#define LSM6DSOX_ALL_INT_SRC_WU_IA_SHIFT              ((uint8_t)    1)

#define LSM6DSOX_ALL_INT_SRC_SINGLE_TAP_MASK          ((uint8_t) 0x04)
#define LSM6DSOX_ALL_INT_SRC_SINGLE_TAP_SHIFT         ((uint8_t)    2)

#define LSM6DSOX_ALL_INT_SRC_DOUBLE_TAP_MASK          ((uint8_t) 0x80)
#define LSM6DSOX_ALL_INT_SRC_DOUBLE_TAP_SHIFT         ((uint8_t)    3)

#define LSM6DSOX_ALL_INT_SRC_D6D_IA_MASK              ((uint8_t) 0x10)
#define LSM6DSOX_ALL_INT_SRC_D6D_IA_SHIFT             ((uint8_t)    4)

#define LSM6DSOX_ALL_INT_SRC_SLEEP_CHANGE_IA_MASK     ((uint8_t) 0x20)
#define LSM6DSOX_ALL_INT_SRC_SLEEP_CHANGE_IA_SHIFT    ((uint8_t)    5)

#define LSM6DSOX_ALL_INT_SRC_TIMESTAMP_ENDCOUNT_MASK  ((uint8_t) 0x80)
#define LSM6DSOX_ALL_INT_SRC_TIMESTAMP_ENDCOUNT_SHIFT ((uint8_t)    7)
/*------------------------------*/



/*--------------------------------
** Register: D6D_SRC
** Enum: LSM6DSOX_D6D_SRC
** --
** Offset : 0x1D - Portrait, landscape, face-up and face-down source register.
** ------------------------------*/
typedef union {
    struct {
        uint8_t            XL : 1;
        uint8_t            XH : 1;
        uint8_t            YL : 1;
        uint8_t            YH : 1;
        uint8_t            ZL : 1;
        uint8_t            ZH : 1;
        uint8_t            D6D_IA : 1;
        uint8_t            DEN_DRDY : 1;
    } b;
    uint8_t w;
} LSM6DSOX_D6D_SRC_t;


/*
** D6D_SRC - Bit field mask definitions
*/
#define LSM6DSOX_D6D_SRC_XL_MASK        ((uint8_t) 0x01)
#define LSM6DSOX_D6D_SRC_XL_SHIFT       ((uint8_t)    0)

#define LSM6DSOX_D6D_SRC_XH_MASK        ((uint8_t) 0x02)
#define LSM6DSOX_D6D_SRC_XH_SHIFT       ((uint8_t)    1)

#define LSM6DSOX_D6D_SRC_YL_MASK        ((uint8_t) 0x04)
#define LSM6DSOX_D6D_SRC_YL_SHIFT       ((uint8_t)    2)

#define LSM6DSOX_D6D_SRC_YH_MASK        ((uint8_t) 0x80)
#define LSM6DSOX_D6D_SRC_YH_SHIFT       ((uint8_t)    3)

#define LSM6DSOX_D6D_SRC_ZL_MASK        ((uint8_t) 0x10)
#define LSM6DSOX_D6D_SRC_ZL_SHIFT       ((uint8_t)    4)

#define LSM6DSOX_D6D_SRC_ZH_MASK        ((uint8_t) 0x20)
#define LSM6DSOX_D6D_SRC_ZH_SHIFT       ((uint8_t)    5)

#define LSM6DSOX_D6D_SRC_D6D_IA_MASK    ((uint8_t) 0x40)
#define LSM6DSOX_D6D_SRC_D6D_IA_SHIFT   ((uint8_t)    6)

#define LSM6DSOX_D6D_SRC_DEN_DRDY_MASK  ((uint8_t) 0x80)
#define LSM6DSOX_D6D_SRC_DEN_DRDY_SHIFT ((uint8_t)    7)
/*------------------------------*/



/*--------------------------------
** Register: STATUS_REG
** Enum: LSM6DSOX_STATUS_REG
** --
** Offset : 0x1E - Portrait, landscape, face-up and face-down source register.
** ------------------------------*/
typedef union {
    struct {
        uint8_t           XLDA : 1;
        uint8_t            GDA : 1;
        uint8_t            TDA : 1;
        uint8_t _reserved_ : 5;
    } b;
    uint8_t w;
} LSM6DSOX_STATUS_REG_t;


/*
** STATUS_REG - Bit field mask definitions
*/
#define LSM6DSOX_STATUS_REG_XLDA_MASK       ((uint8_t) 0x01)
#define LSM6DSOX_STATUS_REG_XLDA_SHIFT      ((uint8_t)    0)

#define LSM6DSOX_STATUS_REG_GDA_MASK        ((uint8_t) 0x02)
#define LSM6DSOX_STATUS_REG_GDA_SHIFT       ((uint8_t)    1)

#define LSM6DSOX_STATUS_REG_TDA_MASK        ((uint8_t) 0x04)
#define LSM6DSOX_STATUS_REG_TDA_SHIFT       ((uint8_t)    2)
/*------------------------------*/




/*
** COUNTER_BDR_REG1 - Bit field mask definitions
*/
#define LSM6DSOX_COUNTER_BDR_REG1_CNT_BDR_TH_MASK   ((uint8_t) 0x07)
#define LSM6DSOX_COUNTER_BDR_REG1_CNT_BDR_TH_SHIFT  ((uint8_t)    0)
/*------------------------------*/

/*
** FIFO_STATUS2 - Bit field value definitions
*/
#define LSM6DSOX_FIFO_STATUS2_COUNTER_BDR_IA       ((uint8_t) 0x10)
#define LSM6DSOX_FIFO_STATUS2_FIFO_WTM_IA          ((uint8_t) 0x80)
/*------------------------------*/


/* Bus address */
const unsigned short SENSOR_BUS_ADDRESS[] = L"6A";

/* Default property values */
typedef struct _DATA_RATE
{
    ULONG  DataRateInterval;
    BYTE   RateCode;
} DATA_RATE, * PDATA_RATE;

const unsigned short SENSOR_NAME[] = L"IMU";
const unsigned short SENSOR_DESCRIPTION[] = L"Accelerometer and Gyroscope Sensor";
const unsigned short SENSOR_ID[] = L"LSM6DSOX";
const unsigned short SENSOR_MANUFACTURER[] = L"STMicroelectronics";
const unsigned short SENSOR_MODEL[] = L"LSM6DSOXCQ";


const ULONG SENSOR_MIN_REPORT_INTERVAL = 1;
const ULONG SENSOR_HIGH_PERFORMANCE_THRESHOLD_INTERVAL = 3;

const ULONG DEFAULT_BATCH_SAMPLE_COUNT = 1;
const ULONG DEFAULT_BATCH_LATENCY = 0;
const UINT32 SENSOR_FIFORESERVEDSIZE_SAMPLES_ACC = 511;
const UINT32 SENSOR_FIFORESERVEDSIZE_SAMPLES_GYR = 0;
const UINT32 SENSOR_FIFO_MAXSIZE_SAMPLES = 511;
const BOOL SENSOR_WAKE_CAPABLE = FALSE;