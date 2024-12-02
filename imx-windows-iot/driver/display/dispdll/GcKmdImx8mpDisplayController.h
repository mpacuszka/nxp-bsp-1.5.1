/* Copyright (c) Microsoft Corporation.
 * Copyright 2023 NXP
   Licensed under the MIT License. */

#pragma once

#include "GcKmdImx8Display.h"
#include "GcKmdBaseDisplayController.h"
#include "GcKmdImx8mpDisplay.h"
#include "GcKmdImx8mpHdmiDisplay.h"
#include "GcKmdImx8mpMipiDsiDisplay.h"


class GcKmImx8mpDisplayController : public GcKmBaseDisplayController
{
public:

    GcKmImx8mpDisplayController(
        DXGKRNL_INTERFACE*  pDxgkInterface);

    ~GcKmImx8mpDisplayController();

private:

    static const INT MP_MAX_MULTIPLE_DISPLAYS = 3;
    GcKmImx8mpDisplay*        p_LvdsDisplay;
    GcKmImx8mpHdmiDisplay*    p_HdmiDisplay;
    GcKmImx8mpMipiDsiDisplay* p_MipiDisplay;
};
