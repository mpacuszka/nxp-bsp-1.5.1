/** @file
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
*  Copyright 2020, 2022 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#ifndef _IMX_I2C_H_
#define _IMX_I2C_H_

#include <stdint.h>

typedef struct {
  uintptr_t ControllerAddress;   //I2C base address
  UINT32 ControllerSlaveAddress;
  UINT32 ReferenceFrequency;
  UINT32 TargetFrequency;
  UINT32 SlaveAddress;
  UINT32 TimeoutInUs;
} IMX_I2C_CONTEXT;

/**
  Perform I2C read operation.

  The iMXI2cRead perform I2C read operation by programming the I2C controller.
  The caller is responsible to provide I2C controller configuration.

  @param[in]    I2cContext        Pointer to structure containing the targeted
                                  I2C controller to be used for I2C operation.
  @param[in]    RegisterAddress   Targeted device register address to start read.
  @param[out]   ReadBufferPtr     Caller supplied buffer that would be written
                                  into with data from the read operation.
  @param[in]    ReadBufferSize    Size of caller supplied buffer.

  @retval   RETURN_SUCCESS        I2C Read operation succeeded.
  @retval   RETURN_DEVICE_ERROR   The I2C device is not functioning correctly.

**/
RETURN_STATUS
iMXI2cRead(
  IN IMX_I2C_CONTEXT  *I2cContext,
  IN UINT8            RegisterAddress,
  OUT UINT8           *ReadBufferPtr,
  IN UINT32           ReadBufferSize
);

/**
  Perform I2C write operation.

  The iMXI2cWrite perform I2C write operation by programming the I2C
  controller. The caller is responsible to provide I2C controller
  configuration.

  @param[in]    IMX_I2C_CONFIG    Pointer to structure containing the targeted
                                  I2C controller to be used for I2C operation.
  @param[in]    RegisterAddress   Targeted device register address to start write.
  @param[out]   WriteBufferPtr    Caller supplied buffer that contained data that
                                  would be read from for I2C write operation.
  @param[in]    WriteBufferSize   Size of caller supplied buffer.

  @retval   RETURN_SUCCESS        I2C Write operation succeeded.
  @retval   RETURN_DEVICE_ERROR   The I2C device is not functioning correctly.

**/
RETURN_STATUS
iMXI2cWrite(
  IN IMX_I2C_CONTEXT  *I2cContext,
  IN UINT8            RegisterAddress,
  IN UINT8            *WriteBufferPtr,
  IN UINT32           WriteBufferSize
);

/**
  Perform I2C read operation.

  The iMXI2cRead perform I2C read operation by programming the I2C controller.
  The caller is responsible to provide I2C controller configuration.

  @param[in]    I2cContext        Pointer to structure containing the targeted
                                  I2C controller to be used for I2C operation.
  @param[in]    RegisterAddress   Targeted device register address to start read.
  @param[in]    RegisterAddressSize  Length of sub address to send in bytes. Maximum size is 4 bytes.
  @param[out]   ReadBufferPtr     Caller supplied buffer that would be written
                                  into with data from the read operation.
  @param[in]    ReadBufferSize    Size of caller supplied buffer.

  @retval   RETURN_SUCCESS        I2C Read operation succeeded.
  @retval   RETURN_DEVICE_ERROR   The I2C device is not functioning correctly.

**/
RETURN_STATUS
iMXI2cReadE(
    IN IMX_I2C_CONTEXT* I2cContext,
    IN UINT32 RegisterAddress,
    IN UINT8 RegisterAddressSize,
    OUT UINT8* ReadBufferPtr,
    IN UINT32 ReadBufferSize
);

/**
  Perform I2C write operation.

  The iMXI2cWrite perform I2C write operation by programming the I2C
  controller. The caller is responsible to provide I2C controller
  configuration.

  @param[in]    IMX_I2C_CONFIG    Pointer to structure containing the targeted
                                  I2C controller to be used for I2C operation.
  @param[in]    RegisterAddress   Targeted device register address to start write.
  @param[in]    RegisterAddressSize  Length of sub address to send in bytes. Maximum size is 4 bytes.
  @param[out]   WriteBufferPtr    Caller supplied buffer that contained data that
                                  would be read from for I2C write operation.
  @param[in]    WriteBufferSize   Size of caller supplied buffer.

  @retval   RETURN_SUCCESS        I2C Write operation succeeded.
  @retval   RETURN_DEVICE_ERROR   The I2C device is not functioning correctly.

**/
RETURN_STATUS
iMXI2cWriteE(
    IN IMX_I2C_CONTEXT* I2cContext,
    IN UINT32 RegisterAddress,
    IN UINT8 RegisterAddressSize,
    IN UINT8* WriteBufferPtr,
    IN UINT32 WriteBufferSize
);

#endif
