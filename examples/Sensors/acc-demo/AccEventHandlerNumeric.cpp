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


#include "AccEventHandlerNumeric.h"

AccEventHandlerNumeric::AccEventHandlerNumeric()
    :ISensorEvents()
{
}

STDMETHODIMP AccEventHandlerNumeric::QueryInterface(REFIID iid, void** ppv)
{
    if (ppv == NULL)
    {
        return E_POINTER;
    }
    if (iid == __uuidof(IUnknown))
    {
        *ppv = static_cast<IUnknown*>(this);
    }
    else if (iid == __uuidof(ISensorEvents))
    {
        *ppv = static_cast<ISensorEvents*>(this);
    }
    else
    {
        *ppv = NULL;
        return E_NOINTERFACE;
    }
    AddRef();
    return S_OK;
}

STDMETHODIMP_(ULONG) AccEventHandlerNumeric::AddRef()
{
    return InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) AccEventHandlerNumeric::Release()
{
    ULONG count = InterlockedDecrement(&m_cRef);
    if (count == 0)
    {
        delete this;
        return 0;
    }
    return count;
}

STDMETHODIMP AccEventHandlerNumeric::OnLeave(REFSENSOR_ID sensorID)
{
    HRESULT hr = S_OK;

    // Peform any housekeeping tasks for the sensor that is leaving.
    // For example, if you have maintained a reference to the sensor,
    // release it now and set the pointer to NULL.

    return hr;
}

STDMETHODIMP AccEventHandlerNumeric::OnEvent(
    ISensor* pSensor,
    REFGUID eventID,
    IPortableDeviceValues* pEventData)
{
    HRESULT hr = S_OK;

    // Handle custom events here.
    return hr;
}


STDMETHODIMP AccEventHandlerNumeric::OnDataUpdated(
    ISensor* pSensor,
    ISensorDataReport* pNewData)
{
    // Measure time passed after last event
    long long time_diff = 0;
    std::chrono::steady_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    if (!first_event) {
        time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();
    }
    else {
        first_event = false;
    }
    last_time = current_time;

    HRESULT hr = S_OK;

    if (NULL == pNewData ||
        NULL == pSensor)
    {
        return E_INVALIDARG;
    }

    double dX = 0;
    double dY = 0;
    double dZ = 0;

    SYSTEMTIME timeStamp;
    PROPVARIANT var = {};

    hr = pNewData->GetSensorValue(SENSOR_DATA_TYPE_ACCELERATION_X_G, &var);

    VARTYPE vt;

    if (SUCCEEDED(hr))
    {
        vt = var.vt;
        if (var.vt == VT_R8)
        {
            dX = var.dblVal;
        }
        else {
            hr = E_UNEXPECTED;
        }
    }

    PropVariantClear(&var);

    if (SUCCEEDED(hr))
    {
        hr = pNewData->GetSensorValue(SENSOR_DATA_TYPE_ACCELERATION_Y_G, &var);
    }

    if (SUCCEEDED(hr))
    {
        if (var.vt == VT_R8)
        {
            dY = var.dblVal;
        }
        else {
            hr = E_UNEXPECTED;
        }
    }

    PropVariantClear(&var);

    if (SUCCEEDED(hr))
    {
        hr = pNewData->GetSensorValue(SENSOR_DATA_TYPE_ACCELERATION_Z_G, &var);
    }

    if (SUCCEEDED(hr))
    {
        if (var.vt == VT_R8)
        {
            dZ = var.dblVal;
        }
        else {
            hr = E_UNEXPECTED;
        }
    }

    PropVariantClear(&var);
    hr = pNewData->GetTimestamp(&timeStamp);
    if (SUCCEEDED(hr))
    {
        //// Print
        mvprintw(1,1,"Timestamp: %d-%02d-%02d %02d:%02d:%02d.%03d",
            timeStamp.wYear,
            timeStamp.wMonth,
            timeStamp.wDay,
            timeStamp.wHour,
            timeStamp.wMinute,
            timeStamp.wSecond,
            timeStamp.wMilliseconds);
        mvprintw(2,1,"Retrieved Values are:");
        //wprintf_s(L"X:%f\nY:%f\nZ:%f\n", dX, dY, dZ);
        mvprintw(4, 1, "X: %f", dX);
        mvprintw(5, 1, "Y: %f", dY);
        mvprintw(6, 1, "Z: %f", dZ);
        mvprintw(7,1,"Time from last event: %-*d", 10, time_diff);
        refresh();
    }

    return hr;
}

STDMETHODIMP AccEventHandlerNumeric::OnStateChanged(
    ISensor* pSensor,
    SensorState state)
{
    HRESULT hr = S_OK;

    if (NULL == pSensor)
    {
        return E_INVALIDARG;
    }


    if (state == SENSOR_STATE_READY)
    {
        wprintf_s(L"\nAccelerometer sensor is now ready.");
    }
    else if (state == SENSOR_STATE_ACCESS_DENIED)
    {
        wprintf_s(L"\nNo permission for the accelerometer sensor.\n");
        wprintf_s(L"Enable the sensor in the control panel.\n");
    }

    return hr;
}