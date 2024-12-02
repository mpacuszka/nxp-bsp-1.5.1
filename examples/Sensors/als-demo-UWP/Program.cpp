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

#include "pch.h"

#include <collection.h>
#include <ppltasks.h>
#include <iostream>
#include <string>
#include <conio.h>
#include <timezoneapi.h>

ULONG sampleCounter = 1;
using namespace Windows::Devices::Sensors;

void PrintProperties(LightSensor^ lightSensor);
void SetProperties(LightSensor^ lightSensor);
void RegisterEvents(LightSensor^ lightSensor);

int main()
{
    Windows::Foundation::Initialize(RO_INIT_MULTITHREADED);
    LightSensor^ lightSensor = LightSensor::GetDefault();

    if (lightSensor == nullptr)
    {
        printf("No ambient light sensor instance found. \n Press any key to continue...");
        _getch();
        return 1;
    }

    char chOpt = '\0';

    while (chOpt != 'q' && chOpt != 'Q')
    {
        printf("\n\AMBIENT LIGHT SENSOR DEMO APP\n\n");
        printf("Select option:\n");
        printf("1. Register data events\n");
        printf("2. View sensor properties\n");
        printf("3. Set sensor properties\n");
        printf("\nUse 'Q' to exit.\n\n");
        printf("Select: ");

        std::cin >> chOpt;

        if (chOpt == '1')
        {
            RegisterEvents(lightSensor);
        }
        else if (chOpt == '2')
        {
            PrintProperties(lightSensor);
        }
        else if (chOpt == '3')
        {
            SetProperties(lightSensor);
        }
        else if (chOpt != 'q' && chOpt != 'Q')
        {
            std::cout << "Invalid option!\n";
        }
    }
}

void SetProperties(LightSensor^ lightSensor)
{
    char chOpt = '\0';

    printf("Select property to set (q to Quit):\n");
    printf("Hint: While setting new value, input any non-number to keep the old one.\n\n");
    printf("1. Sensor threshold\n");
    printf("2. Report interval\n");
    printf("3. Report latency\n");
    printf("Select: ");

    std::cin >> chOpt;

    if (chOpt == '1')
    {
        std::string strNewVal;
        double dblNewVal, dblOldVal;

        LightSensorDataThreshold^ thresholds = lightSensor->ReportThreshold;

        dblOldVal = thresholds->AbsoluteLux;
        printf("Absolute lux[%f]: \n", dblOldVal);
        std::cin >> strNewVal;
        try
        {
            dblNewVal = std::stod(strNewVal);
        }
        catch (std::invalid_argument e)
        {
            dblNewVal = dblOldVal;
        }

        thresholds->AbsoluteLux = dblNewVal;
        
    }
    else if (chOpt == '2')
    {
        std::string strNewVal;
        unsigned int uiNewVal;

        printf_s("\nSelect new value[%u]: \n", lightSensor->ReportInterval);
        std::cin >> strNewVal;
        try
        {
            uiNewVal = std::stoul(strNewVal);
        }
        catch (std::invalid_argument e)
        {
            return;
        }

        lightSensor->ReportInterval = uiNewVal;


    }
    else if (chOpt == '3')
    {
        std::string strNewVal;
        unsigned int uiNewVal;

        printf_s("\nSelect new value[%u]: \n", lightSensor->ReportLatency);
        std::cin >> strNewVal;
        try
        {
            uiNewVal = std::stoul(strNewVal);
        }
        catch (std::invalid_argument e)
        {
            return;
        }

        lightSensor->ReportLatency = uiNewVal;
    }

}

void PrintProperties(LightSensor^ lightSensor)
{
    if (lightSensor == nullptr)
    {
        return;
    }

    Platform::String^ deviceId = lightSensor->DeviceId;
    unsigned int minReportInt = lightSensor->MinimumReportInterval;
    unsigned int reportInt = lightSensor->ReportInterval;
    LightSensorDataThreshold^ reportThreshold = lightSensor->ReportThreshold;

    unsigned int maxBatchSize = lightSensor->MaxBatchSize;
    unsigned int reportLatency = lightSensor->ReportLatency;


    printf("Device ID: %s\n", deviceId);
    printf("Minimum report interval: %u\n", minReportInt);
    printf("Report interval: %u\n", reportInt);
    printf("Report latency: %u\n", reportLatency);
    printf("Max batch size: %u\n", maxBatchSize);

    printf("Report thresholds:\n");
    printf("\tAbsoluteLux: %f\n", reportThreshold->AbsoluteLux);
    printf("\tLuxPercentage: %f\n", reportThreshold->LuxPercentage);

    std::cout << "\nPress any key to continue...\n";
    _getch();
}

void ReadingChanged(LightSensor^ sender, LightSensorReadingChangedEventArgs^ e)
{

    LightSensorReading^ reading = e->Reading;
    double lux = reading->IlluminanceInLux;
    Windows::Foundation::DateTime timestamp = reading->Timestamp;


    int64 ut = timestamp.UniversalTime;
    FILETIME ft;
    ULARGE_INTEGER ularge;

    ularge.QuadPart = ut;
    ft.dwLowDateTime = ularge.LowPart;
    ft.dwHighDateTime = ularge.HighPart;

    SYSTEMTIME sysTime;
    FileTimeToSystemTime(&ft, &sysTime);

    printf("\n\n%u:\n", sampleCounter);
    printf("LUX: %f\n", lux);
    printf("Timestamp: %d-%02d-%02d %02d:%02d:%02d.%03d\n",
        sysTime.wYear,
        sysTime.wMonth,
        sysTime.wDay,
        sysTime.wHour,
        sysTime.wMinute,
        sysTime.wSecond,
        sysTime.wMilliseconds);
    printf("Press any key to stop.\n");
    sampleCounter++;
}

void RegisterEvents(LightSensor^ lightSensor)
{
    Windows::Foundation::EventRegistrationToken readingToken = lightSensor->ReadingChanged += ref new Windows::Foundation::TypedEventHandler<LightSensor^, LightSensorReadingChangedEventArgs^>(ReadingChanged);
    _getch();
    lightSensor->ReadingChanged -= readingToken;
    sampleCounter = 1;
}