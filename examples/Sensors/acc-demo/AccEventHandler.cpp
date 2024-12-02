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

#include "AccEventHandler.h"

AccEventHandler::AccEventHandler(ISensor* pSensor)
    :ISensorEvents()
{
    PROPVARIANT pv = {};
    IPortableDeviceValues* pInnerValues;
    HRESULT hr = S_OK;

    const PROPERTYKEY SensorProperties[] =
    {
        SENSOR_PROPERTY_RANGE_MAXIMUM,
        SENSOR_PROPERTY_RANGE_MINIMUM
    };

    const PROPERTYKEY accelerationAxes[] =
    {
        SENSOR_DATA_TYPE_ACCELERATION_X_G,
        SENSOR_DATA_TYPE_ACCELERATION_Y_G,
        SENSOR_DATA_TYPE_ACCELERATION_Z_G
    };

    float dummyVal = 0;
    hr = pSensor->GetProperty(SENSOR_PROPERTY_RANGE_MAXIMUM, &pv);
    VARTYPE vt = pv.vt;
    if (pv.vt == VT_UNKNOWN)
    {
        pInnerValues = (IPortableDeviceValues*)pv.punkVal;
        DWORD cInnerVals;
        for (int i = 0; i < 3; i++)
        {
            PropVariantClear(&pv);
            pInnerValues->GetFloatValue(accelerationAxes[i], &dummyVal);
        }
    }

    hr = pSensor->GetProperty(SENSOR_PROPERTY_RANGE_MINIMUM, &pv);
    vt = pv.vt;
    if (pv.vt == VT_UNKNOWN)
    {
        pInnerValues = (IPortableDeviceValues*)pv.punkVal;
        DWORD cInnerVals;
        for (int i = 0; i < 3; i++)
        {
            PropVariantClear(&pv);
            pInnerValues->GetFloatValue(accelerationAxes[i], &(rangesMin[i]));
        }
    }
}

STDMETHODIMP AccEventHandler::QueryInterface(REFIID iid, void** ppv)
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

STDMETHODIMP_(ULONG) AccEventHandler::AddRef()
{
    return InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) AccEventHandler::Release()
{
    ULONG count = InterlockedDecrement(&m_cRef);
    if (count == 0)
    {
        delete this;
        return 0;
    }
    return count;
}

STDMETHODIMP AccEventHandler::OnLeave(REFSENSOR_ID sensorID)
{
    HRESULT hr = S_OK;

    // Peform any housekeeping tasks for the sensor that is leaving.
    // For example, if you have maintained a reference to the sensor,
    // release it now and set the pointer to NULL.

    return hr;
}

STDMETHODIMP AccEventHandler::OnEvent(
    ISensor* pSensor,
    REFGUID eventID,
    IPortableDeviceValues* pEventData)
{
    HRESULT hr = S_OK;

    // Handle custom events here.

    return hr;
}


STDMETHODIMP AccEventHandler::OnDataUpdated(
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


    

    /// Visualize data here
    if (SUCCEEDED(hr))
    {
        mvprintw(1,1,"Timestamp: %d-%02d-%02d %02d:%02d:%02d.%03d",
            timeStamp.wYear,
            timeStamp.wMonth,
            timeStamp.wDay,
            timeStamp.wHour,
            timeStamp.wMinute,
            timeStamp.wSecond,
            timeStamp.wMilliseconds);
        mvprintw(2, 1, "X: %f", dX);
        mvprintw(3, 1, "Y: %f", dY);
        mvprintw(4, 1, "Z: %f", dZ);
        mvprintw(5,1,"Time from last event (ms): %-*d", 10, time_diff);

        int middleCol = COLS / 2;
        
        //transate values to vectors that will be visualized
        int vX = dX / rangesMax[0] * middleCol * VISUAL_EVENT_VECTOR_MULTIPLIER;
        int vY = dY / rangesMax[1] * middleCol * VISUAL_EVENT_VECTOR_MULTIPLIER;
        int vZ = dZ / rangesMax[2] * middleCol * VISUAL_EVENT_VECTOR_MULTIPLIER;

        // Apply thresholds to prevent arrows overflowing to next line
        {
            int maxVal = middleCol - 2;
            int minVal = -1 * (middleCol - 2);
            if (vX > maxVal) vX = maxVal;
            if (vY > maxVal) vY = maxVal;
            if (vZ > maxVal) vZ = maxVal;

            if (vX < minVal) vX = minVal;
            if (vY < minVal) vY = minVal;
            if (vZ < minVal) vZ = minVal;
        }


        int frac = (LINES - 2) / 4;

        if (vX > 0) 
        {
            mvprintw(frac * 1 + 4, middleCol + 1, "%-*s", middleCol - 2, (std::string(vX - 1, '=') + '>').c_str());
            mvprintw(frac * 1 + 4, 1, "%*s", middleCol - 1, " ");
        }
        else if (vX < 0) 
        {
            mvprintw(frac * 1 + 4, 1, "%*s", middleCol - 1 , ('<' + std::string(abs(vX - 1), '=')).c_str());
            mvprintw(frac * 1 + 4, middleCol + 1, "%-*s", middleCol - 2, " ");
        }

        if (vY > 0)
        {
            mvprintw(frac * 2 + 4, middleCol + 1, "%-*s", middleCol - 2, (std::string(vY - 1, '=') + '>').c_str());
            mvprintw(frac * 2 + 4, 1, "%*s", middleCol - 1, " ");
        }
        else if (vY < 0)
        {
            mvprintw(frac * 2 + 4, 1, "%*s", middleCol - 1, ('<' + std::string(abs(vY - 1), '=')).c_str());
            mvprintw(frac * 2 + 4, middleCol + 1, "%-*s", middleCol - 2, " ");
        }

        if (vZ > 0)
        {
            mvprintw(frac * 3 + 4, middleCol + 1, "%-*s", middleCol - 2, (std::string(vZ - 1, '=') + '>').c_str());
            mvprintw(frac * 3 + 4, 1, "%*s", middleCol - 1, " ");
        }
        else if (vZ < 0)
        {
            mvprintw(frac * 3 + 4, 1, "%*s", middleCol - 1, ('<' + std::string(abs(vZ - 1), '=')).c_str());
            mvprintw(frac * 3 + 4, middleCol + 1, "%-*s", middleCol - 2, " ");
        }

        //mvprintw(frac * 2 + 2, middleCol - 2, "Y");
        //mvprintw(frac * 3 + 2, middleCol - 2, "Z");


        refresh();


    }

    return hr;
}

STDMETHODIMP AccEventHandler::OnStateChanged(
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

void AccEventHandler::SeizeTerminal()
{
    initscr();
    clear();
    noecho();
    cbreak();

    unsigned int middleCol = COLS / 2;



    mvprintw(0, 0, "%s", std::string(COLS, '=').c_str());
    mvprintw(LINES - 1, 0, "%s", std::string(COLS, '=').c_str());

    for (int i = 1; i < LINES - 1; i++)
    {
        mvprintw(i, 0, "|");
        mvprintw(i, COLS - 1, "|");
        mvprintw(i, middleCol, "|");
    }

    int frac = (LINES - 2) / 4;

    refresh();
}

void AccEventHandler::ReleaseTerminal()
{
    endwin();
}