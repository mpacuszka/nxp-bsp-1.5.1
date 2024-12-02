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

#include "precomp.h"

#include "GcKmd.h"

#include "GcKmdLogging.h"
#include "GcKmdEmuNode.tmh"

#include "GcKmdEmuNode.h"

GcKmEmu3DNode::GcKmEmu3DNode(
    GcKmMmu    *pMmu) :
    GcKmSwQueueNode(pMmu)
{
    RegisterEngine(&m_3DEngine);
}


