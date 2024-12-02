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

#include "GcKmdDevice.h"

class GcKm7LDevice : public GcKmDevice
{
public:

    virtual NTSTATUS
    CreateContext(
        DXGKARG_CREATECONTEXT*  pCreateContext) = NULL;
};


class GcKm7LMQDevice : public GcKm7LDevice
{
public:

    virtual NTSTATUS
    CreateContext(
        DXGKARG_CREATECONTEXT*  pCreateContext);
};


class GcKm7LMPDevice : public GcKm7LDevice
{
public:

    virtual NTSTATUS
    CreateContext(
        DXGKARG_CREATECONTEXT*  pCreateContext);
};


