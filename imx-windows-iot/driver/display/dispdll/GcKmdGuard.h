/* Copyright (c) Microsoft Corporation.
   Licensed under the MIT License. */

#pragma once

#include <ntddk.h>

class LockGuard
{
public:
    LockGuard(FAST_MUTEX& mutex)
        : m_Mutex(mutex)
    {
        ExAcquireFastMutex(&m_Mutex);
    }

    ~LockGuard()
    {
        ExReleaseFastMutex(&m_Mutex);
    }

private:
    FAST_MUTEX& m_Mutex;
};

template <typename Callable>
class ScopeExitGuard
{
public:
    ScopeExitGuard(Callable ReleaseResource, BOOLEAN Active = TRUE)
        : Active{Active}
        , ReleaseResource{ReleaseResource}
    { }

    ~ScopeExitGuard()
    {
        if (Active)
        {
            ReleaseResource();
        }
    }

    void Activate()
    {
        Active = TRUE;
    }

    void Deactivate()
    {
        Active = FALSE;
    }

    BOOLEAN State()
    {
        return Activate;
    }

private:
    BOOLEAN Active;
    Callable ReleaseResource;
};

template <typename Callable>
ScopeExitGuard<Callable> MakeScopeExitGuard(Callable ReleaseResource)
{
    return ScopeExitGuard<Callable>{ReleaseResource};
}