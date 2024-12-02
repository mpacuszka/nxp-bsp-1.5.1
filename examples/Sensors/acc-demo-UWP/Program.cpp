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

void PrintProperties(Accelerometer^ acc);
void SetProperties(Accelerometer^ accelerometer);
void RegisterEvents(Accelerometer^ accelerometer);

unsigned int reportInterval;

int main()
{
    Windows::Foundation::Initialize(RO_INIT_MULTITHREADED);

    Accelerometer^ accelerometer = Accelerometer::GetDefault(AccelerometerReadingType::Linear);

    if (accelerometer == nullptr)
    {
        printf("No acclereometer instance found. \n Press any key to continue...");
        _getch();
        return 1;
    }

    char chOpt = '\0';

    while (chOpt != 'q' && chOpt != 'Q')
    {
        printf("\n\nACCELERATOR DEMO APP\n\n");
        printf("Select option:\n");
        printf("1. Register data events\n");
        printf("2. View sensor properties\n");
        printf("3. Set sensor properties\n");
        printf("\nUse 'Q' to exit.\n\n");
        printf("Select: ");

        std::cin >> chOpt;

        if (chOpt == '1')
        {
            RegisterEvents(accelerometer);
        }
        else if (chOpt == '2')
        {
            PrintProperties(accelerometer);
        }
        else if (chOpt == '3')
        {
            SetProperties(accelerometer);
        }
        else if (chOpt != 'q' && chOpt != 'Q')
        {
            std::cout << "Invalid option!\n";
        }
    }
}

void SetProperties(Accelerometer^ accelerometer)
{

    char chOpt = '\0';

    printf("Select property to set (q to Quit):\n");
    printf("Hint: While setting new value, input any non-number to keep the old one.\n\n");
    printf("1. Sensor thresholds\n");
    printf("2. Report interval\n");
    printf("3. Report latency\n");
    printf("Select: ");

    std::cin >> chOpt;

    if (chOpt == '1')
    {
        std::string strNewVal;
        double dblNewVal, dblOldVal;

        AccelerometerDataThreshold^ thresholds = accelerometer->ReportThreshold;

        printf_s("\nSelect new values:\n");

        dblOldVal = thresholds->XAxisInGForce;
        printf("X[%f]: \n", dblOldVal);
        std::cin >> strNewVal;
        try
        {
            dblNewVal = std::stod(strNewVal);
        }
        catch (std::invalid_argument e)
        {
            dblNewVal = dblOldVal;
        }

        thresholds->XAxisInGForce = dblNewVal;

        dblOldVal = thresholds->YAxisInGForce;
        printf("Y[%f]: \n", dblOldVal);
        std::cin >> strNewVal;
        try
        {
            dblNewVal = std::stod(strNewVal);
        }
        catch (std::invalid_argument e)
        {
            dblNewVal = dblOldVal;
        }

        thresholds->YAxisInGForce = dblNewVal;

        dblOldVal = thresholds->ZAxisInGForce;
        printf("Z[%f]: \n", dblOldVal);
        std::cin >> strNewVal;
        try
        {
            dblNewVal = std::stod(strNewVal);
        }
        catch (std::invalid_argument e)
        {
            dblNewVal = dblOldVal;
        }

        thresholds->ZAxisInGForce = dblNewVal;


    }
    else if (chOpt == '2')
    {
        std::string strNewVal;
        unsigned int uiNewVal;

        printf_s("\nSelect new value[%u]: \n", accelerometer->ReportInterval);
        std::cin >> strNewVal;
        try
        {
            uiNewVal = std::stoul(strNewVal);
        }
        catch (std::invalid_argument e)
        {
            return;
        }

        accelerometer->ReportInterval = uiNewVal;


    }
    else if (chOpt == '3')
    {
        std::string strNewVal;
        unsigned int uiNewVal;

        printf_s("\nSelect new value[%u]: \n", accelerometer->ReportLatency);
        std::cin >> strNewVal;
        try
        {
            uiNewVal = std::stoul(strNewVal);
        }
        catch (std::invalid_argument e)
        {
            return;
        }

        accelerometer->ReportLatency = uiNewVal;
    }

}

void PrintProperties(Accelerometer^ accelerometer)
{
    if (accelerometer == nullptr)
    {
        return;
    }

    Platform::String^ deviceId = accelerometer->DeviceId;
    unsigned int minReportInt = accelerometer->MinimumReportInterval;
    unsigned int reportInt = accelerometer->ReportInterval;
    AccelerometerDataThreshold^ reportThreshold = accelerometer->ReportThreshold;

    unsigned int maxBatchSize = accelerometer->MaxBatchSize;
    unsigned int reportLatency = accelerometer->ReportLatency;

    printf("Device ID: %s\n", deviceId);
    printf("Minimum report interval: %u\n", minReportInt);
    printf("Report interval: %u\n", reportInt);
    printf("Report latency: %u\n", reportLatency);
    printf("Max batch size: %u\n", maxBatchSize);

    printf("Report thresholds:\n");
    printf("\tX: %f\n", reportThreshold->XAxisInGForce);
    printf("\tY: %f\n", reportThreshold->YAxisInGForce);
    printf("\tZ: %f\n", reportThreshold->ZAxisInGForce);

    std::cout << "\nPress any key to continue...\n";
    _getch();
}

void ReadingChanged(Accelerometer^ sender, AccelerometerReadingChangedEventArgs^ e)
{
    AccelerometerReading^ reading = e->Reading;
    double x = reading->AccelerationX;
    double y = reading->AccelerationY;
    double z = reading->AccelerationZ;
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
    printf("X: %f\n", x);
    printf("Y: %f\n", y);
    printf("Z: %f\n", z);
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

void RegisterEvents(Accelerometer^ accelerometer)
{
    Windows::Foundation::EventRegistrationToken readingToken = accelerometer->ReadingChanged += ref new Windows::Foundation::TypedEventHandler<Accelerometer^, AccelerometerReadingChangedEventArgs^>(ReadingChanged);
    _getch();
    accelerometer->ReadingChanged -= readingToken;
    sampleCounter = 1;
}


