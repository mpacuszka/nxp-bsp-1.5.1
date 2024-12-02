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

#pragma once

#include <windows.h>
#include <SensorsApi.h>
#include <Sensors.h>
#include <chrono>
#include <curses.h>
#include <string>

#define VISUAL_EVENT_VECTOR_MULTIPLIER 2

class AccEventHandler : public ISensorEvents
{
public:

    AccEventHandler(ISensor* pSensor);

    STDMETHODIMP QueryInterface(REFIID iid, void** ppv);

    STDMETHODIMP_(ULONG) AddRef();

    STDMETHODIMP_(ULONG) Release();

    //
    // ISensorEvents methods.
    //

    STDMETHODIMP OnEvent(ISensor* pSensor, REFGUID eventID, IPortableDeviceValues* pEventData);

    STDMETHODIMP OnDataUpdated(ISensor* pSensor, ISensorDataReport* pNewData);

    STDMETHODIMP OnLeave(REFSENSOR_ID sensorID);

    STDMETHODIMP OnStateChanged(ISensor* pSensor, SensorState state);
    
    
    // Method that prepares terminal for sensor data display using the curses library. After calling this, it is impossible to print anything to terminal until it is relased. Use before subscribing to sensor events.
    void SeizeTerminal();

    // Frees terminal for normal use. Use after unsubscribing from sensor events.
    void ReleaseTerminal();

private:
    long m_cRef = 0;
    std::chrono::steady_clock::time_point last_time = std::chrono::high_resolution_clock::now();
    bool first_event = true;

    float rangesMax[3] = { 4.0,4.0,4.0 };
    float rangesMin[3] = { -4.0,-4.0,-4.0 };
};