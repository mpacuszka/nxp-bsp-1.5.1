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


// FlexCAN_interrupt.c : FlexCAN interrupt communication - example application 
// This file contains the '_tmain' function. Program execution begins and ends there.

#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <strsafe.h>
#include <conio.h>
#include <SetupAPI.h>
#include <Shlwapi.h>
#include <initguid.h>
#include "../../../../imx-windows-iot/driver/can/imxcan_mc/can.h"


//
// Definitions
//
#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)
#define USE_CANFD             (0)
 /*
  *    DWORD_IN_MB    DLC    BYTES_IN_MB             Maximum MBs
  *    2              8      kFLEXCAN_8BperMB        64
  *    4              10     kFLEXCAN_16BperMB       42
  *    8              13     kFLEXCAN_32BperMB       24
  *    16             15     kFLEXCAN_64BperMB       14
  *
  * Dword in each message buffer, Length of data in bytes, Payload size must align,
  * and the Message Buffers are limited corresponding to each payload configuration:
  */

// Set FlexCAN frame data length in bytes
#define DLC (8)
// Set FlexCAN Frame Identifier
#define CAN_ID_STD_SHIFT (18U)

// Get frequency of FlexCAN clock
// ACPI configures reference clock to 40 MHz
#define EXAMPLE_CAN_CLK0_FREQ (40000000U)
// Peripheral clock is 66MHz 
#define EXAMPLE_CAN_CLK1_FREQ (66000000U)

// FlexCAN example bitRate 
#define EXAMPLE_CAN_BITRATE (125000U)
// Set USE_IMPROVED_TIMING_CONFIG macro to use api to calculate the improved CAN / CAN FD timing values
#define USE_IMPROVED_TIMING_CONFIG (1U)

// Example number of data bytes to transmit
#define EXAMPLE_NUM_OF_DATA_BYTES (3)

#define CHARACTER_ASCII (204)


typedef struct _OVL_WRAPPER {
    OVERLAPPED                          Overlapped;
    CAN_CONTROLLER_NOTIFICATION_OUTPUT  ReturnedSequence;
} OVL_WRAPPER, * POVL_WRAPPER;


//
// Variables
//
volatile BOOLEAN txComplete = FALSE;
volatile BOOLEAN rxComplete = FALSE;
volatile BOOLEAN wakenUp = FALSE;

CAN_CONTROLLER_NB_RECEIVE_INPUT     txXfer, rxXfer;
CAN_CONTROLLER_STATUS_OUTPUT        retStatus;
CAN_CONTROLLER_FRAME                frame;
HANDLE                              handle;
PSP_DEVICE_INTERFACE_DETAIL_DATA    devInterfaceDetailData;


//
// Thread routine
// 
// create thread, wait for notification that it worked, print result and status of the flexcan and process events
//
DWORD WINAPI CompletionPortThread(LPVOID PortHandle) {
    DWORD           byteCount = 0;
    ULONG_PTR       compKey = 0;
    OVERLAPPED*     overlapped = NULL;
    POVL_WRAPPER    wrap;
    DWORD           code;
    DWORD           returned;
    POVL_WRAPPER    memwr;
    BOOLEAN         fSuccess;
    DWORD           Error;

    // Create memory wrapper, use when creating thread to store returned buffer and overlapped structure
    memwr = (POVL_WRAPPER)(malloc(sizeof(OVL_WRAPPER)));
    if (memwr == NULL) {
        Error = GetLastError();
        printf("Memory allocation failed with error 0x%x\n", Error);
        return (Error);
    }

    while (TRUE) {
		if (memwr != 0) {
			memset(memwr, 0, sizeof(OVL_WRAPPER));
		}
		fSuccess = DeviceIoControl(handle, IOCTL_CAN_INVERT_NOTIFICATION, NULL, 0, &memwr->ReturnedSequence, sizeof(data_callback), &returned, &memwr->Overlapped);
		if (fSuccess) {
			printf("Thread NTF\n");
		}
		else {
			Error = GetLastError();
			if (Error == ERROR_IO_PENDING) {
			// printf("Thread(NTF) pending\n");
			}
		}

        // Wait for a completion notification.
        overlapped = NULL;
        BOOL worked = GetQueuedCompletionStatus(
            PortHandle,                // Completion port handle
            &byteCount,                // Bytes transferred
            &compKey,                  // Completion key... don't care
            &overlapped,               // OVERLAPPED structure
            INFINITE);                 // Notification time-out interval

        // If it's our notification ioctl that's just been completed...  don't do anything special. 
        if (byteCount == 0) {
            printf("GetQueuedCompletionStatus() byteCount = 0x%x\n", byteCount);
            continue;
        }
        if (overlapped == NULL) {
            printf("GetQueuedCompletionStatus() overlapped == NULL\n");
            // An unrecoverable error occurred in the completion port. Wait for the next notification.
            continue;
        }

        // The wrapper structure starts with the OVERLAPPED structure, the pointers are the same. 
        wrap = (POVL_WRAPPER)(overlapped);
        code = GetLastError();
        printf(">>> Notification received.  result = 0x%x  status = 0x%x  \n", (UINT32)(wrap->ReturnedSequence.result), wrap->ReturnedSequence.status);

        switch (wrap->ReturnedSequence.status) {
        // Process FlexCAN Rx event. Update FlexCAN controller frame.
        case kStatus_FLEXCAN_RxIdle:
            if (RX_MESSAGE_BUFFER_NUM == wrap->ReturnedSequence.result) {
                frame = wrap->ReturnedSequence.frame;
                rxComplete = TRUE;
            }
            break;
        // Process FlexCAN Tx event.
        case kStatus_FLEXCAN_TxIdle:
            if (TX_MESSAGE_BUFFER_NUM == wrap->ReturnedSequence.result) {
                txComplete = TRUE;
            }
            break;
        // Process FlexCAN WakeUp event.
        case kStatus_FLEXCAN_WakeUp:
            wakenUp = TRUE;
            break;
        default:
            break;
        }
    }
}


//
// Get the count of all CAN device interfaces
//
ULONG CountCans(HDEVINFO DeviceInfoSet) {
    SP_DEVICE_INTERFACE_DATA    DeviceInterfaceData;
    ULONG                       NumberDevices;
    BOOL                        fSuccess = FALSE;
    DWORD                       Error;

    ZeroMemory(&DeviceInterfaceData, sizeof(SP_DEVICE_INTERFACE_DATA));
    NumberDevices = 0;
    do {
        DeviceInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

        // Function returns information about device interfaces exposed by one or more devices.
        // Each call returns information about one interface
        fSuccess = SetupDiEnumDeviceInterfaces(DeviceInfoSet, NULL, &GUID_DEVINTERFACE_CAN_CONTROLLER, NumberDevices, &DeviceInterfaceData);
        NumberDevices++;
    } while (fSuccess);

    Error = GetLastError();
    if (Error == ERROR_NO_MORE_ITEMS) {
        NumberDevices--;    // Number of CAN interfaces found.
        return NumberDevices;
    }
    else {
        return -1;
    }
}


//
// Find required CAN device interface
// 
// return the ID and number of CAN interface if found
//
PWSTR FindCan(PCWCHAR name) {
    HDEVINFO                    DeviceInfoSet;
    SP_DEVICE_INTERFACE_DATA    DeviceInterfaceData;
    int                         idxList;
    ULONG                       NumberDevices, i, Size;
    PWSTR                       sstr = NULL;
    BOOL                        fSuccess = FALSE;
    DWORD                       Error;
    #define NUMLIST 3     // Number of physical FlexCAN devices - i.MX8MPlus has 2
    WCHAR* Rlist[NUMLIST] = { L"_CAN2_", L"_CAN1_", L"_CAN0_" };   // acceptable parameters
    WCHAR* Alist[NUMLIST] = { L"#2#", L"#1#", L"#0#" };        // Unique ID for CAN device instances in the ACPI table - _UID entry.


    // Finds the first occurrence of a substring within a string (unicode).
    for (idxList = 0; idxList < NUMLIST; idxList++) {
        sstr = StrStrIW(name, Rlist[idxList]);   // The comparison is case-insensitive and substring is not the null-terminated. StrStrIW vs. StrStrNIW
        if (sstr) {
            break;
        }
    }

    // Get a list of all devices of class 'GUID_DEVINTERFACE_CAN_CONTROLLER'
    DeviceInfoSet = SetupDiGetClassDevs(&GUID_DEVINTERFACE_CAN_CONTROLLER, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (INVALID_HANDLE_VALUE == DeviceInfoSet) {
        _tprintf(TEXT("SetupDiGetClassDevs()  ERROR.\n"));
        Error = GetLastError();
        return (NULL);
    }

    // The first step is to get the count of CAN devices
    NumberDevices = CountCans(DeviceInfoSet);
        // Find the required CAN
        if (NumberDevices > 0) {
            i = 0;
            sstr = NULL;
            while (i < NumberDevices) {
                ZeroMemory(&DeviceInterfaceData, sizeof(SP_DEVICE_INTERFACE_DATA));
                DeviceInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
                fSuccess = SetupDiEnumDeviceInterfaces(DeviceInfoSet, NULL, &GUID_DEVINTERFACE_CAN_CONTROLLER, i++, &DeviceInterfaceData);
                if (fSuccess) {
                    if (!SetupDiGetDeviceInterfaceDetail(DeviceInfoSet, &DeviceInterfaceData, NULL, 0, &Size, NULL)) {
                        Error = GetLastError();
                        if (Error != ERROR_INSUFFICIENT_BUFFER) {
                            printf("SetupDiGetDeviceInterfaceDetail failed with error 0x%x\n", Error);
                            SetupDiDestroyDeviceInfoList(DeviceInfoSet);
                            return INVALID_HANDLE_VALUE;
                        }
                    }
                    // Buffer was too small anyway and we can immediately try again
                    devInterfaceDetailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(Size);
                    if (!devInterfaceDetailData) {
                        printf("Unable to allocate resources...Exiting\n");
                        SetupDiDestroyDeviceInfoList(DeviceInfoSet);
                        return INVALID_HANDLE_VALUE;
                    }
                    devInterfaceDetailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
                    if (!SetupDiGetDeviceInterfaceDetail(DeviceInfoSet, &DeviceInterfaceData, devInterfaceDetailData, Size, &Size, NULL)) {
                        printf("SetupDiGetDeviceInterfaceDetail failed with error 0x%x\n", GetLastError());
                        SetupDiDestroyDeviceInfoList(DeviceInfoSet);
                        free(devInterfaceDetailData);
                        return INVALID_HANDLE_VALUE;
                    }
                    // The string in 'DevicePath' must contain a substring with a _UID from the ACPI table. 
                    sstr = StrStrIW(devInterfaceDetailData->DevicePath, Alist[idxList]);
                    if (sstr) {
                        break; // Required CAN interface found. 
                    }
                }
            }
        }

    // release the device info list
    SetupDiDestroyDeviceInfoList(DeviceInfoSet);

    return sstr;
}


//
// Print the list of all found FlexCAN device interfaces
//
VOID ListCans(PCWSTR name0, PCWSTR name1, PCWSTR name2) {
    HDEVINFO    DeviceInfoSet;
    PWSTR       sstr;
    DWORD       Error;

    // Get a list of all devices of class 'GUID_DEVINTERFACE_CAN_CONTROLLER'
    DeviceInfoSet = SetupDiGetClassDevs(&GUID_DEVINTERFACE_CAN_CONTROLLER, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (INVALID_HANDLE_VALUE == DeviceInfoSet) {
        _tprintf(TEXT("SetupDiGetClassDevs()  ERROR.\n"));
        Error = GetLastError();
        return;
    }

    ULONG NumberDevices = CountCans(DeviceInfoSet);
    printf("Found %d CAN interfaces.\n", NumberDevices);
    
	sstr = NULL;

	sstr = FindCan(name0);
	if (sstr != NULL) {
		printf("%ls:     %ls\n", name0, sstr);
	}

	sstr = FindCan(name1);
	if (sstr != NULL) {
		printf("%ls:     %ls\n", name1, sstr);
	}

	sstr = FindCan(name2);
	if (sstr != NULL) {
		printf("%ls:     %ls\n", name2, sstr);
	}

    return;
}


//
// Get and print information about opened FlexCAN
//
VOID GetInfo(HANDLE handle) {
    CAN_CONTROLLER_INFO     info;
    DWORD                   returned;
    ULONG                   outSize;
    BOOLEAN                 fSuccess;

    outSize = sizeof(info);
    fSuccess = DeviceIoControl(handle, IOCTL_CAN_CONTROLLER_GET_INFO, NULL, 0, &info, outSize, &returned, NULL);
    printf("Controller info->Sequence = 0x%x\r\n", info.Sequence);
    printf("Controller info->RequestCnt = 0x%x\r\n", info.RequestCnt);
}


VOID PrintHelp() {
    printf("CAN_transfer.exe - example flexCAN interrupt communication\n\n");
    printf("Usage:  CAN_transfer.exe [-h | -help] [-l | -list]\n");
    printf("-h | -help              print this help message and exit\n");
    printf("-l | -list              list all available CAN interfaces and exit\n\n");
    printf("Terminate the communication by using Ctrl+C interrupt signal.\n");
    return;
}


int __cdecl _tmain(int argc, TCHAR *argv[]) {
    HANDLE          hFile, hCFile, hComplPort;
    DWORD           Error;
    BOOLEAN         fSuccess;
    HANDLE          hThread;
    DWORD           dwThreadId;
    POVL_WRAPPER    wrap;
    DWORD           returned;
    PWSTR           sstr;
    PCWSTR          name0 = L"_CAN0_";
    PCWSTR          name1 = L"_CAN1_";
    PCWSTR          name2 = L"_CAN2_";
    char            nodeType, nodeCan, key, clkSource;
    unsigned int    bitrate;
    int             i, frequency;
    UINT32          txIdentifier, rxIdentifier;
    UINT8           data[EXAMPLE_NUM_OF_DATA_BYTES];
    CAN_CONTROLLER_GET_CONFIG_OUTPUT        flexcanConfig;
    ULONG                                   outSize, inSize;
    CAN_CONTROLLER_SOURCE_CLOCK_INPUT       clockHz;
    CAN_CONTROLLER_IMPROVED_TIMING_OUTPUT   retTiming;
    CAN_CONTROLLER_GLOBAL_MASK_INPUT        rxGlobalMask;
    CAN_HELPER_FLEXCAN_RX_MB_STD_MASK_INPUT stdmask;
    CAN_CONTROLLER_SET_RXMB_CONFIG          fsrxmbc;
    flexcan_rx_mb_config_t                  mbConfig;
    CAN_CONTROLLER_SET_TXMB_CONFIG          fstxmbc;
    CAN_HELPER_FLEXCAN_ID_STD               tmp;


    //------------------------------------------------------------

    for(int i = 1; i < argc; i++) {
        if (!_tcscmp(argv[i], _T("-h")) || !_tcscmp(argv[i], _T("-help"))) {
            PrintHelp();
            return 0;
        }
        else if (!_tcscmp(argv[i], _T("-l")) || !_tcscmp(argv[i], _T("-list"))) {
            ListCans(name0, name1, name2);
            return 0;
        }
    }

    printf("********* FLEXCAN Interrupt EXAMPLE *********\r\n");
    printf("    Message format: Standard (11 bit id)\r\n");
    printf("    Message buffer %d used for Rx.\r\n", RX_MESSAGE_BUFFER_NUM);
    printf("    Message buffer %d used for Tx.\r\n", TX_MESSAGE_BUFFER_NUM);
    printf("    Interrupt Mode: Enabled\r\n");
    printf("    Operation Mode: TX and RX --> Normal\r\n");
    printf("*********************************************\r\n\r\n");

    do {
        printf("Please select local node as A or B:\r\n");
        printf("Note: Node B should start first.\r\n");
        printf("Node: ");
        nodeType = getchar();
        printf("\r\n");
    } while ((nodeType != 'A') && (nodeType != 'B') && (nodeType != 'a') && (nodeType != 'b') && (nodeType != '0'));
    if (nodeType == '0') {
        return 1;
    }
    key = getchar();
    do {
        printf("Please select CAN 0, 1 or 2\r\n");
        printf("CAN: ");
        nodeCan = getchar();
        printf("\r\n");
    } while ((nodeCan != '0') && (nodeCan != '1') && (nodeCan != '2'));

    key = getchar();

    // CAN handles
	if (nodeCan == '0') {
		// Find CAN0
		devInterfaceDetailData = NULL;
		sstr = NULL;
		sstr = FindCan(name0);
	}
	else if (nodeCan == '1') {
		// Find CAN1
		devInterfaceDetailData = NULL;
		sstr = NULL;
		sstr = FindCan(name1);
	}
	else {
		// Find CAN2
		devInterfaceDetailData = NULL;
		sstr = NULL;
		sstr = FindCan(name2);
	}
	if (sstr == NULL) {
		printf("Unable to find any matching devices!\n");
		return ERROR_BAD_UNIT;  //The system cannot find the device specified.
	}
	if (devInterfaceDetailData == NULL) {
		printf("Unable to find any matching devices!\n");
		return INVALID_HANDLE_VALUE;
	}

	// Open FlexCAN 
	hFile = CreateFile(devInterfaceDetailData->DevicePath,
		GENERIC_READ | GENERIC_WRITE,
		FILE_SHARE_READ,
		0,
		OPEN_EXISTING,
		0,
		0);
	if (hFile == INVALID_HANDLE_VALUE) {
		Error = GetLastError();
		printf("CreateFile failed with error 0x%lx\n", Error);
		return(Error);
	}

    // Overlapped handle
	hCFile = CreateFile(devInterfaceDetailData->DevicePath,
		GENERIC_READ,
		FILE_SHARE_READ,
		0,
		OPEN_EXISTING,
		FILE_FLAG_OVERLAPPED,
		0);
	if (hCFile == INVALID_HANDLE_VALUE) {
		Error = GetLastError();
		printf("CreateFile failed with error 0x%lx\n", Error);
		return(Error);
	}

    // Completion port handle
	hComplPort = CreateIoCompletionPort(hCFile, NULL, 0, 0);
	if (hComplPort == NULL) {
		Error = GetLastError();
		printf("CreateIoCompletionPort failed with error 0x%lx\n", Error);
		return(Error);
	}

    handle = hCFile;

    // Select mailbox ID
    if ((nodeType == 'A') || (nodeType == 'a')) {
        txIdentifier = 0x321;
        rxIdentifier = 0x123;
    }
    else {
        txIdentifier = 0x123;
        rxIdentifier = 0x321;
    }

    // Create thread
	hThread = CreateThread(
		NULL,                   // Default thread security descriptor
		0,                      // Default stack size
		CompletionPortThread,   // Start routine
		hComplPort,             // Start routine parameter
		0,                      // Run immediately
		&dwThreadId);           // Thread ID
	if (hThread == NULL) {
		Error = GetLastError();
		printf("CreateThread failed with error 0x%lx\n", Error);
		return;
	}
	wrap = (POVL_WRAPPER)(malloc(sizeof(OVL_WRAPPER)));
	memset(wrap, 0, sizeof(OVL_WRAPPER));
	fSuccess = DeviceIoControl(hCFile, IOCTL_CAN_INVERT_NOTIFICATION, NULL, 0, &wrap->ReturnedSequence, sizeof(CAN_CONTROLLER_NOTIFICATION_OUTPUT), &returned, &wrap->Overlapped);
	if (fSuccess) {
		printf("DeviceIoControl() IOCTL_CAN_INVERT_NOTIFICATION\n");
	}
	else {
		Error = GetLastError();
		if (Error == ERROR_IO_PENDING) {
			// printf("DeviceIoControl(IOCTL_CAN_INVERT_NOTIFICATION) pending ...\n");
		}
	}

    // Get FlexCAN configuration structure with default values
    outSize = sizeof(flexcanConfig);
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_GET_CONFIG, NULL, 0, &flexcanConfig, outSize, &returned, NULL);
    if (!fSuccess) {
        Error = GetLastError();
        printf("IOCTL_CAN_CONTROLLER_GET_CONFIG failed with error 0x%x\n", Error);
    }

    // Adjust configuration - clock source and bitrate, according to user input 
    do {
        printf("Please select clock source 0 or 1:\r\n");
        printf("Clock source: ");
        clkSource = getchar();
        printf("\r\n");
    } while ((clkSource != '0') && (clkSource != '1'));
    if (clkSource == '0') {
        flexcanConfig.clkSrc = kFLEXCAN_ClkSrc0;
        frequency = EXAMPLE_CAN_CLK0_FREQ;
    }
    else if (clkSource == '1') {
        flexcanConfig.clkSrc = kFLEXCAN_ClkSrc1;
        frequency = EXAMPLE_CAN_CLK1_FREQ;
    }
    else {
        return 1;
    }

    inSize = sizeof(flexcanConfig.clkSrc);
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_SET_CLKSRC, &flexcanConfig.clkSrc, inSize, NULL, 0, &returned, NULL);
    if (!fSuccess) {
        Error = GetLastError();
        printf("IOCTL_CAN_CONTROLLER_SET_CLKSRC failed with error 0x%x\n", Error);
    }
    
    while(1) {
        printf("Please select communication bitrate (expecting integer in range (0;1000000>):\r\n");
        printf("Bitrate: ");

        scanf_s("%u", &bitrate);
        if (bitrate > 0 && bitrate <= 1000000) {
            printf("\r\n");
            break;
        }
        else {
            bitrate = EXAMPLE_CAN_BITRATE;
            printf("\r\n");
            break;
        }
    }
    
    flexcanConfig.bitRate = bitrate;
    inSize = sizeof(flexcanConfig.bitRate);
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_SET_BITRATE, &flexcanConfig.bitRate, inSize, NULL, 0, &returned, NULL);
    if (!fSuccess) {
        Error = GetLastError();
        printf("IOCTL_CAN_CONTROLLER_SET_BITRATE failed with error 0x%x\n", Error);
    }

    printf("**************** Timing ****************\r\n");
    printf("    Clock source CTRL1[clksrc] = %d \r\n", flexcanConfig.clkSrc);
    printf("    FlexCAN frequency = %d \r\n", frequency);
    printf("    Bitrate = %d \r\n", flexcanConfig.bitRate);
    printf("****************************************\r\n");

    
    // Find improved controller timing configuration
    if (USE_IMPROVED_TIMING_CONFIG) {
        memset(&retTiming, 0, sizeof(retTiming));
        clockHz = frequency;
        inSize = sizeof(clockHz);
        outSize = sizeof(retTiming);
        fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_IMPROVED_TIMING, &clockHz, inSize, &retTiming, outSize, &returned, NULL);
        if (retTiming.status) {
            // Update the timing configuration
            memcpy(&(flexcanConfig.timingConfig), &retTiming.tcfg, sizeof(flexcan_timing_config_t));
        }
        else {
            printf("No found Improved Timing Configuration. Used default configuration.\r\n\r\n");
        }
    }

    // Initialize FlexCAN controller
    clockHz = frequency;
    inSize = sizeof(clockHz);
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_INIT, &clockHz, inSize, NULL, 0, &returned, NULL);
    if (!fSuccess) {
        Error = GetLastError();
        printf("IOCTL_CAN_CONTROLLER_INIT failed with error 0x%x\n", Error);
    }

    // Create controller handle
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_CREATE_HANDLE, NULL, 0, NULL, 0, &returned, NULL);
    if (!fSuccess) {
        Error = GetLastError();
        printf("IOCTL_CAN_CONTROLLER_CREATE_HANDLE failed with error 0x%x\n", Error);
    }

    // Handle FlexCAN Error
    memset(wrap, 0, sizeof(OVL_WRAPPER));
    fSuccess = DeviceIoControl(hCFile, IOCTL_CAN_INVERT_NOTIFICATION, NULL, 0, &wrap->ReturnedSequence, sizeof(data_callback), &returned, &wrap->Overlapped);
    if (fSuccess) {
        printf("DeviceIoControl(hCFile) IOCTL_CAN_INVERT_NOTIFICATION\n");
    }
    else {
        Error = GetLastError();
        if (Error == ERROR_IO_PENDING) {
            printf("DeviceIoControl(IOCTL_CAN_INVERT_NOTIFICATION)  pending ...\n");
        }
    }

    // Set Rx Masking mechanism
    // Masking allows frames with IDs that respect a pattern to be received in the same message buffer
    stdmask.id = rxIdentifier;
    stdmask.rtr = 0;
    stdmask.ide = 0;
    inSize = sizeof(stdmask);
    outSize = sizeof(rxGlobalMask);
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_HELPER_FLEXCAN_RX_MB_STD_MASK, &stdmask, inSize, &rxGlobalMask, outSize, &returned, NULL);
    if (!fSuccess) {
        Error = GetLastError();
        printf("IOCTL_CAN_HELPER_FLEXCAN_RX_MB_STD_MASK failed with error 0x%x\n", Error);
    }
    // Global mask applies to all configured MBs
    inSize = sizeof(rxGlobalMask);
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_SET_RXMB_GLOBAL_MASK, &rxGlobalMask, inSize, NULL, 0, &returned, NULL);
    if (!fSuccess) {
        Error = GetLastError();
        printf("IOCTL_CAN_CONTROLLER_SET_RXMB_GLOBAL_MASK failed with error 0x%x\n", Error);
    }

    // Setup and configure Rx Message Buffer
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type = kFLEXCAN_FrameTypeData;
    inSize = sizeof(rxIdentifier);
    outSize = sizeof(mbConfig.id);
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_HELPER_FLEXCAN_ID_STD, &rxIdentifier, inSize, &mbConfig.id, outSize, &returned, NULL);
    if (!fSuccess) {
        Error = GetLastError();
        printf("IOCTL_CAN_HELPER_FLEXCAN_ID_STD failed with error 0x%x\n", Error);
    }
    fsrxmbc.mbidx = RX_MESSAGE_BUFFER_NUM;
    fsrxmbc.mbConfig = mbConfig;
    fsrxmbc.enable = TRUE;
    inSize = sizeof(fsrxmbc);
    // Cleans a FlexCAN build-in Message Buffer and configures it as a Receive Message Buffer
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_SET_RXMB_CONFIG, &fsrxmbc, inSize, NULL, 0, &returned, NULL);
    if (!fSuccess) {
        Error = GetLastError();
        printf("IOCTL_CAN_CONTROLLER_SET_RXMB_CONFIG failed with error 0x%x\n", Error);
    }

    // Setup and configure Tx Message Buffer
    fstxmbc.mbidx = TX_MESSAGE_BUFFER_NUM;
    fstxmbc.enable = TRUE;
    inSize = sizeof(fstxmbc);
    // Cleans a FlexCAN build-in Message Buffer and configures it as a Transmit Message Buffer
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_SET_TXMB_CONFIG, &fstxmbc, inSize, NULL, 0, &returned, NULL);
    if (!fSuccess) {
        Error = GetLastError();
        printf("IOCTL_CAN_CONTROLLER_SET_TXMB_CONFIG failed with error 0x%x\n", Error);
    }

    // Define data for transmission
    if ((nodeType == 'A') || (nodeType == 'a')) {
        printf("Please select data to transmit:\r\n");
        for (int i = 0; i < EXAMPLE_NUM_OF_DATA_BYTES; i++) {
            printf("> Data byte %d: ", i);
            while (getchar() != '\n') {
                continue;
            }

            scanf_s("%hhu", &data[i]);

            if (data[i] == CHARACTER_ASCII) {
                data[i] = getchar();
            }
            printf("\r\n");
        }
        
        printf("Press any key to trigger one-shot transmission\r\n\r\n");

        frame.mfp.mfp_b.dataByte0 = data[0];
        frame.mfp.mfp_b.dataByte1 = data[1];
        frame.mfp.mfp_b.dataByte2 = data[2];
    }
    else {
        printf("Wait for data from Node A\r\n\r\n");
    }

    //
    // The data transmission cycle, sends and receives data, stops after interrupt signal/Backspace/Esc/Tab
    //
    while (TRUE) {
        if ((nodeType == 'A') || (nodeType == 'a')) {
            i = _getch();
            if (i < 10 || i == 27) {
                break;
            }

            // Setup transmission frame and start sending data through Tx message buffer
            inSize = sizeof(txIdentifier);
            outSize = sizeof(tmp);
            fSuccess = DeviceIoControl(hFile, IOCTL_CAN_HELPER_FLEXCAN_ID_STD, &txIdentifier, inSize, &tmp, outSize, &returned, NULL);
            if (!fSuccess) {
                Error = GetLastError();
                printf("IOCTL_CAN_HELPER_FLEXCAN_ID_STD failed with error 0x%x\n", Error);
            }
            frame.mfs_1.id = tmp;
            frame.mfs_0.format = (UINT8)kFLEXCAN_FrameFormatStandard;
            frame.mfs_0.type = (UINT8)kFLEXCAN_FrameTypeData;
            frame.mfs_0.length = (UINT8)DLC;
            txXfer.mbIdx = (UINT8)TX_MESSAGE_BUFFER_NUM;
            txXfer.frame = frame;
            inSize = sizeof(txXfer);
            outSize = sizeof(retStatus);
            fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_SEND_NON_BLOCKING, &txXfer, inSize, &retStatus, outSize, &returned, NULL);
            if (!fSuccess) {
                Error = GetLastError();
                printf("IOCTL_CAN_CONTROLLER_SEND_NON_BLOCKING failed with error 0x%x\n", Error);
            }

            // Wait for all data to be sent
            while (!txComplete) { };
            txComplete = FALSE;

            // Start receiving data through Rx message buffer
            rxXfer.mbIdx = (UINT8)RX_MESSAGE_BUFFER_NUM;
            rxXfer.frame = frame;
            inSize = sizeof(rxXfer);
            outSize = sizeof(retStatus);
            fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_RECEIVE_NON_BLOCKING, &rxXfer, inSize, &retStatus, outSize, &returned, NULL);
            if (!fSuccess) {
                Error = GetLastError();
                printf("IOCTL_CAN_CONTROLLER_RECEIVE_NON_BLOCKING failed with error 0x%x\n", Error);
            }

            // Wait until Rx MB is full
            while (!rxComplete) { };
            rxComplete = FALSE;

            printf("Rx MB ID: 0x%3x, Rx MB data: 0x%x 0x%x 0x%x, Time stamp: %d\r\n", frame.mfs_1.id >> CAN_ID_STD_SHIFT, frame.mfp.mfp_b.dataByte0, frame.mfp.mfp_b.dataByte1, frame.mfp.mfp_b.dataByte2, frame.mfs_0.timestamp);

            GetInfo(hFile);

            printf("Press any key to trigger the next transmission!\r\n\r\n");

            // Update the data 
            frame.mfp.mfp_b.dataByte0++;
            frame.mfp.mfp_b.dataByte1 = data[1];
            frame.mfp.mfp_b.dataByte2--;
        }
        else {
            // Before this , should first make node B enter STOP mode after FlexCAN
            // initialized with enableSelfWakeup=true and Rx MB configured, then A
            // sends frame N which wakes up node B. A will continue to send frame N
            // since no acknowledgement, then B received the second frame N(In the
            // application it seems that B received the frame that woke it up which
            // is not expected as stated in the reference manual, but actually the
            // output in the terminal B received is the same second frame N).
            if (wakenUp) {
                printf("B has been waken up!\r\n\r\n");
            }

            // Start receiving data through Rx Message Buffer
            rxXfer.mbIdx = (UINT8)RX_MESSAGE_BUFFER_NUM;
            rxXfer.frame = frame;
            inSize = sizeof(rxXfer);
            outSize = sizeof(retStatus);
            fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_RECEIVE_NON_BLOCKING, &rxXfer, inSize, &retStatus, outSize, &returned, NULL);
            if (!fSuccess) {
                Error = GetLastError();
                printf("IOCTL_CAN_CONTROLLER_RECEIVE_NON_BLOCKING failed with error 0x%x\n", Error);
            }

            // Wait until Rx message buffer is full
            while (!rxComplete) {};
            rxComplete = FALSE;

            printf("Rx MB ID: 0x%3x, Rx MB data: 0x%x 0x%x 0x%x, Time stamp: %d\r\n", frame.mfs_1.id >> CAN_ID_STD_SHIFT, frame.mfp.mfp_b.dataByte0, frame.mfp.mfp_b.dataByte1, frame.mfp.mfp_b.dataByte2, frame.mfs_0.timestamp);

            // Setup transmission frame and start sending data through Tx message buffer
            inSize = sizeof(txIdentifier);
            outSize = sizeof(tmp);
            fSuccess = DeviceIoControl(hFile, IOCTL_CAN_HELPER_FLEXCAN_ID_STD, &txIdentifier, inSize, &tmp, outSize, &returned, NULL);
            if (!fSuccess) {
                Error = GetLastError();
                printf("IOCTL_CAN_HELPER_FLEXCAN_ID_STD failed with error 0x%x\n", Error);
            }
            frame.mfs_1.id = tmp;
            txXfer.mbIdx = (UINT8)TX_MESSAGE_BUFFER_NUM;
            txXfer.frame = frame;
            inSize = sizeof(txXfer);
            outSize = sizeof(retStatus);
            fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_SEND_NON_BLOCKING, &txXfer, inSize, &retStatus, outSize, &returned, NULL);
            if (!fSuccess) {
                Error = GetLastError();
                printf("IOCTL_CAN_CONTROLLER_SEND_NON_BLOCKING failed with error 0x%x\n", Error);
            }

            // Wait for all data to be sent
            while (!txComplete) {};
            txComplete = FALSE;

            GetInfo(hFile);

            printf("Wait for Node A to trigger the next transmission!\r\n\r\n");
        }
    }

    Sleep(1000);
    // De-initializes a FlexCAN instance
    fSuccess = DeviceIoControl(hFile, IOCTL_CAN_CONTROLLER_DEINIT, NULL, 0, NULL, 0, &returned, NULL);

    // ---------------------------------
    fSuccess = CloseHandle(hFile);
    if (!fSuccess) {
        printf("Unable to find any matching devices!\n");
        return ERROR_INVALID_TARGET_HANDLE;
    }
    fSuccess = CloseHandle(hCFile);
    if (!fSuccess) {
        printf("Unable to find any matching devices!\n");
        return ERROR_INVALID_TARGET_HANDLE;
    }
}