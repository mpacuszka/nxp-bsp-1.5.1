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

#include "GcKmdSwQueueNode.h"
#include "GcKmd7LEngine.h"

class GcKm7L3DNode : public GcKmSwQueueNode
{
public:

    GcKm7L3DNode(
        GcKmMmu    *pMmu);

private:

    GcKm7L3DEngine  m_3DEngine;
};


class GcKmCopyNode : public GcKmSwQueueNode
{
public:

    GcKmCopyNode(
        GcKmMmu    *pMmu);

private:

    GcKmCopyEngine  m_CopyEngine;
};


