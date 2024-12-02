//Copyright (C) Microsoft Corporation, All Rights Reserved
//
//Abstract:
//
//      This module contains the constant definitions for the ambient light sensor's
//      register interface and default property values.

/*
* Copyright 2022 NXP
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


// Register interface
#define ISL29023_COMMAND_I                   0x00
#define ISL29023_COMMAND_II                  0x01
#define ISL29023_DATA_LSB                    0x02
#define ISL29023_DATA_MSB                    0x03
#define ISL29023_INT_LT_LSB                  0x04
#define ISL29023_INT_LT_MSB                  0x05
#define ISL29023_INT_HT_LSB                  0x06
#define ISL29023_INT_HT_MSB                  0x07
#define ISL29023_TEST                        0x08

/*--------------------------------
** Register: COMMAND_I
** Enum: ISL29023_COMMAND_I
** --
** Offset : 0x01 - COMMAND_I REGISTER ADDRESS
** ------------------------------*/
typedef union {
    struct {
        uint8_t           int_persist : 2;
        uint8_t              int_flag : 1;
        uint8_t _reserved_ : 2;
        uint8_t                  mode : 3;
    } b;
    uint8_t w;
} ISL29023_COMMAND_I_t;


/*
** COMMAND_I - Bit field mask definitions
*/
#define ISL29023_COMMAND_I_INT_PERSIST_MASK  ((uint8_t) 0x03)
#define ISL29023_COMMAND_I_INT_PERSIST_SHIFT ((uint8_t)    0)

#define ISL29023_COMMAND_I_INT_FLAG_MASK     ((uint8_t) 0x04)
#define ISL29023_COMMAND_I_INT_FLAG_SHIFT    ((uint8_t)    2)

#define ISL29023_COMMAND_I_MODE_MASK         ((uint8_t) 0xE0)
#define ISL29023_COMMAND_I_MODE_SHIFT        ((uint8_t)    5)

/*
** COMMAND_I - Bit field value definitions
*/
#define ISL29023_COMMAND_I_INT_PERSIST_1         ((uint8_t) 0x00) 
#define ISL29023_COMMAND_I_INT_PERSIST_4         ((uint8_t) 0x01) 
#define ISL29023_COMMAND_I_INT_PERSIST_8         ((uint8_t) 0x02) 
#define ISL29023_COMMAND_I_INT_PERSIST_16        ((uint8_t) 0x03) 

#define ISL29023_COMMAND_I_MODE_POWER_DOWN       ((uint8_t) 0x00)
#define ISL29023_COMMAND_I_MODE_ALS_ONCE         ((uint8_t) 0x20) 
#define ISL29023_COMMAND_I_MODE_ALS_CONTINUOUS   ((uint8_t) 0xA0) 
#define ISL29023_COMMAND_I_MODE_IR_ONCE          ((uint8_t) 0x40) 
#define ISL29023_COMMAND_I_MODE_IR_CONTINUOUS    ((uint8_t) 0xC0) 

/*--------------------------------
** Register: COMMAND_II
** Enum: ISL29023_COMMAND_II
** --
** Offset : 0x02 - COMMAND_II REGISTER ADDRESS
** ------------------------------*/
typedef union {
    struct {
        uint8_t                 range : 2;
        uint8_t            resolution : 2;
        uint8_t _reserved_ : 4;
    } b;
    uint8_t w;
} ISL29023_COMMAND_II_t;


/*
** COMMAND_II - Bit field mask definitions
*/
#define ISL29023_COMMAND_II_RANGE_MASK        ((uint8_t) 0x03)
#define ISL29023_COMMAND_II_RANGE_SHIFT       ((uint8_t)    0)

#define ISL29023_COMMAND_II_RESOLUTION_MASK   ((uint8_t) 0x0B)
#define ISL29023_COMMAND_II_RESOLUTION_SHIFT  ((uint8_t)    2)

/*
** COMMAND_II - Bit field value definitions
*/
#define ISL29023_COMMAND_II_RANGE_1000LUX         ((uint8_t) 0x00) 
#define ISL29023_COMMAND_II_RANGE_4000LUX         ((uint8_t) 0x01) 
#define ISL29023_COMMAND_II_RANGE_16000LUX        ((uint8_t) 0x02) 
#define ISL29023_COMMAND_II_RANGE_64000LUX        ((uint8_t) 0x03) 

#define ISL29023_COMMAND_II_RESOLUTION_16BIT      ((uint8_t) 0x00)
#define ISL29023_COMMAND_II_RESOLUTION_12BIT      ((uint8_t) 0x04) 
#define ISL29023_COMMAND_II_RESOLUTION_8BIT       ((uint8_t) 0x08) 
#define ISL29023_COMMAND_II_RESOLUTION_4BIT       ((uint8_t) 0x0A)

/*--------------------------------
** Register: DATA_LSB
** Enum: ISL29023_DATA_LSB
** --
** Offset : 0x03 - DATA_MSB REGISTER ADDRESS
** ------------------------------*/
typedef union {
    struct {
        uint8_t                 d;
    } b;
    uint8_t w;
} ISL29023_DATA_LSB_t;


/*
** DATA_MSB - Bit field mask definitions
*/
#define ISL29023_DATA_MSB_MASK        ((uint8_t) 0xFF)
#define ISL29023_DATA_MSB_SHIFT       ((uint8_t)    0)

/*--------------------------------
** Register: DATA_MSB
** Enum: ISL29023_DATA_MSB
** --
** Offset : 0x04 - DATA_MSB REGISTER ADDRESS
** ------------------------------*/
typedef union {
    struct {
        uint8_t                 d;
    } b;
    uint8_t w;
} ISL29023_DATA_MSB_t;


/*
** DATA_MSB - Bit field mask definitions
*/
#define ISL29023_DATA_MSB_MASK        ((uint8_t) 0xFF)
#define ISL29023_DATA_MSB_SHIFT       ((uint8_t)    0)

/*
** DATA_MSB - Bit field mask definitions
*/
#define ISL29023_DATA_MSB_MASK        ((uint8_t) 0xFF)
#define ISL29023_DATA_MSB_SHIFT       ((uint8_t)    0)

/*--------------------------------
** Register: INT_LT_LSB
** Enum: ISL29023_INT_LT_LSB
** --
** Offset : 0x05 - INT_LT_LSB REGISTER ADDRESS
** ------------------------------*/
typedef union {
    struct {
        uint8_t                 ths;
    } b;
    uint8_t w;
} ISL29023_INT_LT_LSB_t;


/*
** INT_LT_LSB - Bit field mask definitions
*/
#define ISL29023_INT_LT_LSB_MASK        ((uint8_t) 0xFF)
#define ISL29023_INT_LT_LSB_SHIFT       ((uint8_t)    0)


/*--------------------------------
** Register: INT_LT_MSB
** Enum: ISL29023_INT_LT_MSB
** --
** Offset : 0x05 - INT_LT_MSB REGISTER ADDRESS
** ------------------------------*/
typedef union {
    struct {
        uint8_t                 ths;
    } b;
    uint8_t w;
} ISL29023_INT_LT_MSB_t;


/*
** INT_LT_MSB - Bit field mask definitions
*/
#define ISL29023_INT_LT_MSB_MASK        ((uint8_t) 0xFF)
#define ISL29023_INT_LT_MSB_SHIFT       ((uint8_t)    0)

/*--------------------------------
** Register: INT_HT_LSB
** Enum: ISL29023_INT_HT_LSB
** --
** Offset : 0x06 - INT_HT_LSB REGISTER ADDRESS
** ------------------------------*/
typedef union {
    struct {
        uint8_t                 ths;
    } b;
    uint8_t w;
} ISL29023_INT_HT_LSB_t;


/*
** INT_HT_LSB - Bit field mask definitions
*/
#define ISL29023_INT_HT_LSB_MASK        ((uint8_t) 0xFF)
#define ISL29023_INT_HT_LSB_SHIFT       ((uint8_t)    0)


/*--------------------------------
** Register: INT_HT_MSB
** Enum: ISL29023_INT_HT_MSB
** --
** Offset : 0x07 - INT_HT_MSB REGISTER ADDRESS
** ------------------------------*/
typedef union {
    struct {
        uint8_t                 ths;
    } b;
    uint8_t w;
} ISL29023_INT_HT_MSB_t;


/*
** INT_HT_MSB - Bit field mask definitions
*/
#define ISL29023_INT_HT_MSB_MASK        ((uint8_t) 0xFF)
#define ISL29023_INT_HT_MSB_SHIFT       ((uint8_t)    0)

// DATA register bits - the data report includes adjacent data registers, for a total of 6 bytes
#define ISL29023_DATA_REPORT_SIZE_BYTES      2

// Bus address
const unsigned short SENSOR_BUS_ADDRESS[] = L"44";

// Default property values
typedef struct _DATA_RATE
{
    ULONG  DataRateInterval;
    BYTE   RateCode;
    FLOAT  DataResolution;
} DATA_RATE, *PDATA_RATE;

const unsigned short SENSOR_NAME[] = L"Ambient Light Sensor";
const unsigned short SENSOR_DESCRIPTION[] = L"Ambient Light Sensor";
const unsigned short SENSOR_ID[] = L"ISL29023";
const unsigned short SENSOR_MANUFACTURER[] = L"Intersil";
const unsigned short SENSOR_MODEL[] = L"ISL29023";

const DOUBLE SENSOR_MIN_LUX = 0;
const DOUBLE SENSOR_MAX_LUX = 64000.0f;
const DOUBLE SENSOR_RANGE_LUX = SENSOR_MAX_LUX - SENSOR_MIN_LUX;

const ULONG SENSOR_MIN_REPORT_INTERVAL = 15;
const ULONG DEFAULT_SENSOR_REPORT_INTERVAL = 90;
const DATA_RATE SENSOR_SUPPORTED_DATA_RATES[] = { { 90, ISL29023_COMMAND_II_RESOLUTION_16BIT | ISL29023_COMMAND_II_RANGE_64000LUX, SENSOR_RANGE_LUX / 65536.0f },
                                                  {  5, ISL29023_COMMAND_II_RESOLUTION_12BIT | ISL29023_COMMAND_II_RANGE_64000LUX, SENSOR_RANGE_LUX /  4096.0f },
                                                  {  1, ISL29023_COMMAND_II_RESOLUTION_8BIT  | ISL29023_COMMAND_II_RANGE_64000LUX, SENSOR_RANGE_LUX /   256.0f } };

const ULONG SENSOR_SUPPORTED_DATA_RATES_COUNT = ARRAYSIZE(SENSOR_SUPPORTED_DATA_RATES);

const DOUBLE SENSOR_CHANGE_SENSITIVITY_RESOLUTION = SENSOR_RANGE_LUX / 65536.0f;
const DOUBLE SENSOR_DEFAULT_AXIS_THRESHOLD = SENSOR_CHANGE_SENSITIVITY_RESOLUTION;
const DOUBLE DEFAULT_SENSOR_CHANGE_SENSITIVITY = SENSOR_CHANGE_SENSITIVITY_RESOLUTION;