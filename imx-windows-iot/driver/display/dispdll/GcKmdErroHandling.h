/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#define RETURN_ON_FAILURE(Status)   \
    do                              \
    {                               \
        if (!NT_SUCCESS(Status))    \
        {                           \
            return Status;          \
        }                           \
    } while (0)

#define DEBUG_CHECK(Status)                 \
    do                                      \
    {                                       \
        UNREFERENCED_PARAMETER(Status);     \
        NT_ASSERT(NT_SUCCESS(Status));      \
    } while (0)
