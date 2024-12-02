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

// acc-demo.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <windows.h>
#include <SensorsApi.h>
#include <sensorsdef.h>
#include <assert.h>
#include <sensors.h>
#include <conio.h>
#include <string>

#include "AccEventHandler.h"


HRESULT PrintSensorProperties(ISensor* pSensor);
HRESULT SetSensorProperties(ISensor* pSensor);
HRESULT VisualizeEvents(ISensor* pSensor);
template <class T> void SafeRelease(T** ppT);
const wchar_t* pkey2str(PROPERTYKEY pkey);


int main()
{
    HRESULT hr = S_OK;
    ISensorManager* pSensorManager = NULL;
    ISensorCollection* pSensorColl = NULL;
    ISensor* pSensor = NULL;

    hr = CoInitialize(nullptr);

    

    // Create sesor manager instance
    if (SUCCEEDED(hr))
    {
        hr = CoCreateInstance(CLSID_SensorManager, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pSensorManager));
    }

    if (hr == HRESULT_FROM_WIN32(ERROR_ACCESS_DISABLED_BY_POLICY))
    {
        // Unable to retrieve sensor manager due to 
        // group policy settings. Alert the user.
        wprintf_s(L"\nUnable to retrieve sensor manager due to group policy settings.\n");
        wprintf_s(L"\nPres any key to continue...\n");
        _getch();
    }

    if (!SUCCEEDED(hr))
    {
        wprintf_s(L"\nUnable to retrieve sensor manager.\n");
        wprintf_s(L"\nPres any key to continue...\n");
        _getch();
    }

    // Get all linear accelerometer senensors into a collection
    if (SUCCEEDED(hr)) {
        hr = pSensorManager->GetSensorsByType(GUID_SensorType_LinearAccelerometer, &pSensorColl);
    }

    if (!SUCCEEDED(hr))
    {
        wprintf_s(L"\nUnable to retrieve linear accelerometer sensor.\n");
        wprintf_s(L"\nPres any key to continue...\n");
        _getch();
    }

    ULONG ulSensorCount = 0;
    if (SUCCEEDED(hr))
    {
        

        // Verify that the collection contains
        // at least one sensor.
        hr = pSensorColl->GetCount(&ulSensorCount);

        if (SUCCEEDED(hr))
        {
            if (ulSensorCount < 1)
            {
                wprintf_s(L"\nNo sensors of the 'LinearAccelerometer' category.\n");
                hr = E_UNEXPECTED;
            }
        }
    }

    if (SUCCEEDED(hr))
    {
        // Retrieve first sensor from collection
        hr = pSensorColl->GetAt(0, &pSensor);
        
        char chOpt = '\0';
        std::string buffer;

        

        while (chOpt != 'q' && chOpt != 'Q' && SUCCEEDED(hr)) {
            std::cout << "\n\nACCELERATOR DEMO APP\n\n";
            std::cout << "Select option:\n";
            std::cout << "1. Data vizualization\n";
            std::cout << "2. View sensor properties\n";
            std::cout << "3. Set sensor properties\n";
            std::cout << "\nUse 'Q' to exit.\n\n";
            std::cout << "Select: ";
            

            std::cin >> buffer;
            if (buffer.empty()) 
            {
                chOpt = '\0';
            } 
            else
            {
                chOpt = buffer[0];
            }

            if (chOpt == '1')
            {
                VisualizeEvents(pSensor);
            }
            else if (chOpt == '2')
            {
                hr = PrintSensorProperties(pSensor);
            }
            else if (chOpt == '3')
            {
                hr = SetSensorProperties(pSensor);
            }
            else if (chOpt != 'q' && chOpt != 'Q')
            {
                std::cout << "Invalid option!\n";
            }
        }
    }

    CoUninitialize();
    return 0;    
}

HRESULT PrintSensorProperties(ISensor* pSensor)
{
    assert(pSensor);

    HRESULT hr = S_OK;

    DWORD cVals = 0; // Count of returned properties.
    IPortableDeviceKeyCollection* pKeys = NULL; // Input - property keys of properties to retrieve
    IPortableDeviceValues* pValues = NULL;  // Output - retrieved values

    // Properties to retrieve
    // List of all sensor properties can be found at https://learn.microsoft.com/en-us/windows-hardware/drivers/sensors/sensor-properties
    const PROPERTYKEY SensorProperties[] =
    {
        SENSOR_PROPERTY_TYPE,
        SENSOR_PROPERTY_DEVICE_PATH,
        SENSOR_PROPERTY_PERSISTENT_UNIQUE_ID,
        SENSOR_PROPERTY_MANUFACTURER,
        SENSOR_PROPERTY_MODEL,
        SENSOR_PROPERTY_CONNECTION_TYPE,
        SENSOR_PROPERTY_CHANGE_SENSITIVITY,
        SENSOR_PROPERTY_STATE,
        SENSOR_PROPERTY_MIN_REPORT_INTERVAL,
        SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL,
        SENSOR_PROPERTY_RANGE_MAXIMUM,
        SENSOR_PROPERTY_RANGE_MINIMUM,
        SENSOR_PROPERTY_RESOLUTION,
        SENSOR_PROPERTY_CHANGE_SENSITIVITY,
        SENSOR_PROPERTY_DESCRIPTION,
        SENSOR_PROPERTY_SERIAL_NUMBER
    };



    // CoCreate a key collection to store property keys.
    hr = CoCreateInstance(CLSID_PortableDeviceKeyCollection,
        NULL,
        CLSCTX_INPROC_SERVER,
        IID_PPV_ARGS(&pKeys));

    if (SUCCEEDED(hr))
    {
        // Add the keys to the key collection.
        for (DWORD dwIndex = 0; dwIndex < ARRAYSIZE(SensorProperties); dwIndex++)
        {
            hr = pKeys->Add(SensorProperties[dwIndex]);
            if (!SUCCEEDED(hr))
            {
                break;
            }
        }
    }

    if (SUCCEEDED(hr))
    {
        // Retrieve the properties from the sensor.
        hr = pSensor->GetProperties(pKeys, &pValues);
    }

    if (SUCCEEDED(hr))
    {
        // Get the number of pValues returned.        
        hr = pValues->GetCount(&cVals);
    }

    if (SUCCEEDED(hr))
    {
        PROPERTYKEY pk; // Keys
        PROPVARIANT pv = {}; // Values

        // Loop through the pValues;
        for (DWORD i = 0; i < cVals; i++)
        {
            // Get the value at the current index.
            hr = pValues->GetAt(i, &pk, &pv);

            if (SUCCEEDED(hr))
            {
                VARTYPE vt = pv.vt;
                // Get name of current property
                const wchar_t* propertyName = pkey2str(pk);

                // Based on data type of the current property, print out its value
                // List of all data types can be found at https://learn.microsoft.com/en-us/windows/win32/api/propidlbase/ns-propidlbase-propvariant
                if (vt == VT_LPWSTR) 
                {
                    LPWSTR val = pv.pwszVal;
                    wprintf_s(L"%s: %s\n", propertyName, val);
                }
                else if (vt == VT_UI4) 
                {
                    UINT32 val = pv.uiVal;
                    wprintf_s(L"%s: %u\n", propertyName, val);
                }
                else if (vt == VT_ERROR) 
                {
                    DWORD val = pv.scode;
                }
                else if (vt == VT_CLSID) 
                {
                    CLSID* val = pv.puuid;
                    int iRet = 0;

                    // Convert the GUID to a string.
                    OLECHAR wszGuid[39] = {}; // Buffer for string.
                    iRet = ::StringFromGUID2(*val, wszGuid, 39);

                    assert(39 == iRet); // Count of characters returned for GUID.

                    wprintf_s(L"%s: %s\n", propertyName, wszGuid);
                }
                // Some properties can contain another properties list, so its printed out in same way
                else if (vt == VT_UNKNOWN) 
                {
                    wprintf_s(L"%s:\n",propertyName);
                    IPortableDeviceValues* pInnerValues;
                    pInnerValues = (IPortableDeviceValues*)pv.punkVal;
                    DWORD cInnerVals;
                    pInnerValues->GetCount(&cInnerVals);
                    for (DWORD i = 0; i < cInnerVals; i++)
                    {
                        PropVariantClear(&pv);
                        hr = pInnerValues->GetAt(i, &pk, &pv);
                        if (SUCCEEDED(hr)) {
                            vt = pv.vt;
                            if (vt == VT_R8)
                            {
                                double val = 0;
                                propertyName = pkey2str(pk);
                                val = pv.dblVal;
                                wprintf_s(L"\t%s: %f\n",propertyName, val);
                            }
                            else if (vt == VT_R4)
                            {
                                FLOAT val = 0;
                                propertyName = pkey2str(pk);
                                val = pv.fltVal;
                                wprintf_s(L"\t%s: %f\n", propertyName, val);
                            }
                        }
                        else {
                            break;
                        }
                    }                    
                    SafeRelease(&pInnerValues);
                }
                PropVariantClear(&pv);
            }
            else 
            {
                break;
            }

        }
        // Pause to show values
        if (SUCCEEDED(hr)) 
        {
            std::cout << "\nPress any key to continue...\n";
            _getch();
        }
               
    }
    SafeRelease(&pKeys);
    //SafeRelease(&pValues);
      
    return hr;
};


HRESULT SetSensorProperties(ISensor* pSensor) {

    assert(pSensor);
    HRESULT hr = S_OK;


    char chOpt = '\0';

    std::string strNewVal = "";
    IPortableDeviceValues* pPropsToSet = NULL; //Input
    IPortableDeviceValues* pPropsReturn = NULL; //Output

    // Create the input object.
    hr = CoCreateInstance(__uuidof(PortableDeviceValues),
        NULL,
        CLSCTX_INPROC_SERVER,
        IID_PPV_ARGS(&pPropsToSet));

    wprintf_s(L"Select property to set (q to Quit):\n");
    wprintf_s(L"1. SENSOR_PROPERTY_CHANGE_SENSITIVITY\n");
    wprintf_s(L"2. SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL\n");
    wprintf_s(L"Select: ");
    std::cin >> chOpt;
    
    if (chOpt == '1') {

        PROPVARIANT pvSensitivityCollection = {};
        PROPERTYKEY pk;
        const wchar_t* propertyName = L"";
        IPortableDeviceValues* pSensValues;

        hr = pSensor->GetProperty(SENSOR_PROPERTY_CHANGE_SENSITIVITY, &pvSensitivityCollection);
        if (SUCCEEDED(hr))
        {
            // SENSOR_PROPERTY_CHANGE_SENSITIVITY contains IPortableDeviceValues collection
            pSensValues = (IPortableDeviceValues*)pvSensitivityCollection.punkVal;
            DWORD cInnerVals = 0;
            pSensValues->GetCount(&cInnerVals);
                
            // Iterate over retrieved values, and prompt user on setting a new value for each of them
            wprintf_s(L"\nSelect new values. (Input any non-number to keep the old value)\n");
            for (DWORD i = 0; i < cInnerVals; i++)
            {
                PROPVARIANT pvSensitivity = {};
                hr = pSensValues->GetAt(i, &pk, &pvSensitivity);
                if (SUCCEEDED(hr)) {
                    VARTYPE vt = pvSensitivity.vt;
                    if (vt == VT_R8)
                    {
                        propertyName = pkey2str(pk);
                        double dblNewVal = 0;
                        double dblOldVal = pvSensitivity.dblVal;
                        wprintf_s(L"\t%s(%f):", propertyName, dblOldVal);
                        std::cin >> strNewVal;
                        try 
                        {
                            dblNewVal = std::stod(strNewVal);
                        }
                        catch (std::invalid_argument e)
                        {
                            dblNewVal = dblOldVal;
                        }
                        
                        // Set new value into propvariant
                        pvSensitivity.dblVal = dblNewVal;

                        // Insert propvariant into pSensValues collection
                        pSensValues->SetValue(pk, &pvSensitivity);
                    }
                    else {
                        hr = E_UNEXPECTED;
                        break;
                    }
                }
                else {
                    break;
                }
            }
            if (SUCCEEDED(hr))
            {
                // Apply changed pSensValues collection
                hr = pPropsToSet->SetIPortableDeviceValuesValue(SENSOR_PROPERTY_CHANGE_SENSITIVITY, pSensValues);
                pSensor->SetProperties(pPropsToSet, &pPropsReturn);
            }            
        }
         SafeRelease(&pSensValues);
    } 
    else if (chOpt == '2') 
    {
        ULONG ulNewVal = 0, ulOldVal = 0;
        PROPVARIANT pv = {};
        wprintf_s(L"\nSelect new value. (Input any non-number to keep the old value)\n");
        hr = pSensor->GetProperty(SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL, &pv);
        if (SUCCEEDED(hr))
        {
            ulOldVal = pv.ulVal;
            wprintf(L"SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL (%u): ", ulOldVal);
            std::cin >> strNewVal;

            try
            {
                ulNewVal = std::stoul(strNewVal);
            }
            catch (std::invalid_argument e)
            {
                ulNewVal = ulOldVal;
            }

            // Add the current report interval property into properties collection
            hr = pPropsToSet->SetUnsignedIntegerValue(SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL, ulNewVal);
        }

        if (SUCCEEDED(hr))
        {
            hr = pSensor->SetProperties(pPropsToSet, &pPropsReturn);
        }
    }
    else
    {
        wprintf_s(L"Invalid option.\n");
    }

    SafeRelease(&pPropsToSet);
    SafeRelease(&pPropsReturn);
    return hr;
}

void InputThread(std::atomic<bool>* keepRunning)
{
    while (*keepRunning)
    {
        if (_kbhit()) 
        {
            char key = _getch();
            if (key == 'q')
            {
                *keepRunning = FALSE;
            }
        }
    }
}

void EventThread(ISensor* pSensor)
{

}

HRESULT VisualizeEvents(ISensor* pSensor)
{
    assert(pSensor);

    AccEventHandler* pEventH = NULL;
    ISensorEvents*  pMyEvents = NULL;

    HRESULT hr = S_OK;


    //PrintSensorProperties(pSensor);
    // Create an instance of the event class.
    pEventH = new(std::nothrow) AccEventHandler(pSensor);

    // Retrieve the pointer to the callback interface.
    hr = pEventH->QueryInterface(IID_PPV_ARGS(&pMyEvents));    
    
    pEventH->SeizeTerminal();

    // Start receiving events.
    hr = pSensor->SetEventSink(pMyEvents);    

    bool repeat = TRUE;

    MSG msg;
    BOOL bRet;

    int i = 0;

    
    while (repeat)
    {

        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        if (_kbhit())
        {
            char key = _getch();
            if (key == 'q')
            {
                repeat = FALSE;
            }
        }
    }

    if (SUCCEEDED(hr))
    {
        hr = pSensor->SetEventSink(NULL);
    }

    pEventH->ReleaseTerminal();

    return hr;
}


// Helper function for converting property key to string
const wchar_t* pkey2str(PROPERTYKEY pkey) 
{
    if (pkey == SENSOR_PROPERTY_TYPE) {
        return L"SENSOR_PROPERTY_TYPE";
    }
    else if (pkey == SENSOR_PROPERTY_DEVICE_PATH) {
        return L"SENSOR_PROPERTY_DEVICE_PATH";
    }
    else if (pkey == SENSOR_PROPERTY_PERSISTENT_UNIQUE_ID) {
        return L"SENSOR_PROPERTY_PERSISTENT_UNIQUE_ID";
    }
    else if (pkey == SENSOR_PROPERTY_MANUFACTURER) {
        return L"SENSOR_PROPERTY_MANUFACTURER";
    }
    else if (pkey == SENSOR_PROPERTY_MODEL) {
        return L"SENSOR_PROPERTY_MODEL";
    }
    else if (pkey == SENSOR_PROPERTY_CONNECTION_TYPE) {
        return L"SENSOR_PROPERTY_CONNECTION_TYPE";
    }
    else if (pkey == SENSOR_PROPERTY_CHANGE_SENSITIVITY) {
        return L"SENSOR_PROPERTY_CHANGE_SENSITIVITY";
    }
    else if (pkey == SENSOR_PROPERTY_STATE) {
        return L"SENSOR_PROPERTY_STATE";
    }
    if (pkey == SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL) {
        return L"SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL";
    }
    else if (pkey == SENSOR_PROPERTY_MIN_REPORT_INTERVAL) {
        return L"SENSOR_PROPERTY_MIN_REPORT_INTERVAL";
    }
    else if (pkey == SENSOR_PROPERTY_RANGE_MAXIMUM) {
        return L"SENSOR_PROPERTY_RANGE_MAXIMUM";
    }
    else if (pkey == SENSOR_PROPERTY_RANGE_MINIMUM) {
        return L"SENSOR_PROPERTY_RANGE_MINIMUM";
    }
    else if (pkey == SENSOR_PROPERTY_RESOLUTION) {
        return L"SENSOR_PROPERTY_RESOLUTION";
    }
    else if (pkey == SENSOR_PROPERTY_SERIAL_NUMBER) {
        return L"SENSOR_PROPERTY_SERIAL_NUMBER";
    }
    else if (pkey == SENSOR_PROPERTY_CHANGE_SENSITIVITY) {
        return L"SENSOR_PROPERTY_CHANGE_SENSITIVITY";
    }
    else if (pkey == SENSOR_PROPERTY_ACCURACY) {
        return L"SENSOR_PROPERTY_ACCURACY";
    }
    else if (pkey == SENSOR_PROPERTY_DESCRIPTION) {
        return L"SENSOR_PROPERTY_DESCRIPTION";
    }
    else if (pkey == SENSOR_DATA_TYPE_ACCELERATION_X_G) {
        return L"SENSOR_DATA_TYPE_ACCELERATION_X_G";
    }
    else if (pkey == SENSOR_DATA_TYPE_ACCELERATION_Y_G) {
        return L"SENSOR_DATA_TYPE_ACCELERATION_Y_G";
    }
    else if (pkey == SENSOR_DATA_TYPE_ACCELERATION_Z_G) {
        return L"SENSOR_DATA_TYPE_ACCELERATION_Z_G";
    }
    else if (pkey == PKEY_Sensor_FifoReservedSize_Samples) {
        return L"PKEY_Sensor_FifoReservedSize_Samples";
    }
    return L"";
}

// This function releases the pointer ppT and sets it equal to NULL
template <class T> void SafeRelease(T** ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = NULL;
    }
}
