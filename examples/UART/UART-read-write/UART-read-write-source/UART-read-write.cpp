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


// UART-read-write.cpp : serial communication example
// This file contains the 'wmain' function. Program execution begins and ends there.

#include <windows.h>
#include <cfgmgr32.h>
#include <propkey.h>
#include <initguid.h>
#include <devpkey.h>
#include <strsafe.h>
#include <wrl.h>
#include <ppltasks.h>

#include <vector>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <cwctype>
#include <string>


PCWSTR Help =
L"UART-read-write.exe\n"
L"\n"
L"Usage: %s [-list] device_path [parity=<P>] [data=<D>] [stop=<S>] [xon={on|off}] [odsr={on|off}] [octs={on|off}] [dtr={on|off|hs}] [rts={on|off|hs|tg}] [idsr={on|off}] [-help]\n"
L"\n"
L"  -list                List all available serial ports on the system and exit.\n"
L"  device_path          Device path or name of UART device interface over which you wish to communicate (e.g. UART2)\n"
L"  parity={n|e|o|m|s}   Specifies how the system uses the parity bit to check\n"
L"                       for transmission errors. The abbreviations stand for\n"
L"                       none, even, odd, mark, and space. Default value is e (even).\n"
L"  data={5|6|7|8}       Specifies the number of data bits in a character.\n"
L"                       The default value is 7.\n"
L"  stop={1|1.5|2}       Specifies the number of stop bits that define the end of\n"
L"                       a character. The default value is 1.\n"
L"  xon={on|off}         Specifies whether the xon or xoff protocol for data-flow\n"
L"                       control is on or off. By default it is off.\n"
L"  odsr={on|off}        Specifies whether output handshaking that uses the\n"
L"                       Data Set Ready (DSR) circuit is on or off. By default it is off.\n"
L"  octs={on|off}        Specifies whether output handshaking that uses the\n"
L"                       Clear To Send (CTS) circuit is on or off. By default it is off.\n"
L"  dtr={on|off|hs}      Specifies whether the Data Terminal Ready (DTR) circuit\n"
L"                       is on or off or set to handshake. By default it is off.\n"
L"  rts={on|off|hs|tg}   Specifies whether the Request To Send (RTS) circuit is\n"
L"                       set to on, off, handshake, or toggle. By default it is off.\n"
L"  idsr={on|off}        Specifies whether the DSR circuit sensitivity is on\n"
L"                       or off. By default it is off.\n"
L"  -help                Display usage and exit.\n"
L"\n"
L"Parameters that are not specified will default to the port's current\n"
L"configuration. For more information on the connection parameters, see:\n"
L"  https://learn.microsoft.com/en-us/windows-server/administration/windows-commands/mode"
L"\n"
L"Examples:\n"
L"  Connect to the serial port with friendly name UART2 with default configuration:\n"
L"    %s UART2\n"
L"\n"
L"  List all serial ports on the system:\n"
L"    %s -list\n"
L"\n"
L"  Open COM1 in 115200 8N1 configuration:\n"
L"    %s COM1 baud=115200 parity=n data=8 stop=1\n"
L"\n"
L"  Connect to UART2 in 8N1 configuration:\n"
L"    %s UART2 parity=n data=8 stop=1\n";


typedef Microsoft::WRL::Wrappers::HandleT<Microsoft::WRL::Wrappers::HandleTraits::HANDLENullTraits> ThreadHandle;

#define MAX_DATA_LEN 128

//
// Global variables with default values
//
PCWSTR appName;
bool forceQuit = false;
bool quit = false;
std::wstring deviceName;
int glob_Baud = 115200;
BYTE glob_Parity = 2;
BYTE glob_Data = 8;
BYTE glob_Stop = 0;
DWORD glob_XON = FALSE;
DWORD glob_oDSR = FALSE;
DWORD glob_oCTS = FALSE;
DWORD glob_DTR = DTR_CONTROL_DISABLE;
DWORD glob_RTS = RTS_CONTROL_DISABLE;
DWORD glob_iDSR = FALSE;
char send_buffer[MAX_DATA_LEN+1] = {};


enum class SerialStopBits {
    One,
    OnePointFive,
    Two,
};


//
// Structure for masking parameters
//
struct SerialParamMask {
    ULONG BaudSet = false;
    ULONG ParitySet = false;
    ULONG DataLengthSet = false;
    ULONG StopBitsSet = false;
    ULONG XonSet = false;
    ULONG OdsrSet = false;
    ULONG OctsSet = false;
    ULONG DtrSet = false;
    ULONG RtsSet = false;
    ULONG IdsrSet = false;
};


// {00000000-0000-0000-FFFF-FFFFFFFFFFFF}
DEFINE_GUID(SystemContainerId, 0x00000000, 0x0000, 0x0000, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff);


class wexception {
public:
    explicit wexception(const std::wstring& Msg) : msg(Msg) { }

    wexception(const wchar_t* Msg, ...) {
        HRESULT hr;
        WCHAR buf[512];

        va_list argList;
        va_start(argList, Msg);
        hr = StringCchVPrintfW(buf, ARRAYSIZE(buf), Msg, argList);
        va_end(argList);

        this->msg = SUCCEEDED(hr) ? buf : Msg;
    }

    virtual ~wexception() { /*empty*/ }

    virtual const wchar_t* wwhat() const {
        return this->msg.c_str();
    }

private:
    std::wstring msg;
};


void PrintUsage() {
    wprintf(Help, appName, appName, appName, appName, appName, appName);
}


//
// Help functions used when printing current configuration
//
PCWSTR StringFromParity(BYTE parity) {
    switch (parity) {
        case 0: 
            return L"none";
        case 1:
            return L"odd";
        case 2: 
            return L"even";
        case 3: 
            return L"mark";
        case 4: 
            return L"space";
        default: 
            return L"[invalid parity]";
    }
}

PCWSTR StringFromStopBits(BYTE StopBits) {
    switch (StopBits) {
        case BYTE(SerialStopBits::One): 
            return L"1";
        case BYTE(SerialStopBits::OnePointFive): 
            return L"1.5";
        case BYTE(SerialStopBits::Two): 
            return L"2";
        default: 
            return L"[invalid serial stop bits]";
    }
}

PCWSTR StringFromDtrControl(DWORD DtrControl) {
    switch (DtrControl) {
        case DTR_CONTROL_ENABLE: 
            return L"on";
        case DTR_CONTROL_DISABLE: 
            return L"off";
        case DTR_CONTROL_HANDSHAKE: 
            return L"handshake";
        default: 
            return L"[invalid DtrControl value]";
    }
}

PCWSTR StringFromRtsControl(DWORD RtsControl) {
    switch (RtsControl) {
        case RTS_CONTROL_ENABLE: 
            return L"on";
        case RTS_CONTROL_DISABLE: 
            return L"off";
        case RTS_CONTROL_HANDSHAKE: 
            return L"handshake";
        case RTS_CONTROL_TOGGLE: 
            return L"toggle";
        default: 
            return L"[invalid RtsControl value]";
    }
}


//
// Get port name of the selected device interface
//
std::wstring GetPortName(PCWSTR DeviceInterface) {
    // Serial device friendly name
    const DEVPROPKEY propkey = {
        PKEY_DeviceInterface_Serial_PortName.fmtid,
        PKEY_DeviceInterface_Serial_PortName.pid
    };

    // Type definition for property data types
    DEVPROPTYPE propertyType;

    WCHAR portName[512];
    ULONG propertyBufferSize = sizeof(portName);

    // Get the port name
    CONFIGRET cr = CM_Get_Device_Interface_PropertyW(
        DeviceInterface,
        &propkey,
        &propertyType,
        reinterpret_cast<BYTE*>(&portName),
        &propertyBufferSize,
        0); // ulFlags

    if ((cr != CR_SUCCESS) || (propertyType != DEVPROP_TYPE_STRING)) {
        return std::wstring();
    }

    return std::wstring(portName);
}


//
// Find out, if using the selected device interface is/is not restricted
//
bool IsRestricted(PCWSTR DeviceInterface) {
    // Type definition for property data types
    DEVPROPTYPE propertyType;

    DEVPROP_BOOLEAN isRestricted;
    ULONG propertyBufferSize = sizeof(isRestricted);

    CONFIGRET cr = CM_Get_Device_Interface_PropertyW(
        DeviceInterface,
        &DEVPKEY_DeviceInterface_Restricted,
        &propertyType,
        reinterpret_cast<BYTE*>(&isRestricted),
        &propertyBufferSize,
        0); // ulFlags

    if ((cr != CR_SUCCESS) || (propertyType != DEVPROP_TYPE_BOOLEAN)) {
        return true;
    }

    return isRestricted != DEVPROP_FALSE;
}


//
// Find out, if the selected device interface is/is not in the system container
//
bool IsInSystemContainer(PCWSTR DeviceInterface) {
    // Type definition for property data types
    DEVPROPTYPE propertyType;

    DEVPROPGUID containerId;
    ULONG propertyBufferSize = sizeof(containerId);

    CONFIGRET cr = CM_Get_Device_Interface_PropertyW(
        DeviceInterface,
        &DEVPKEY_Device_ContainerId,
        &propertyType,
        reinterpret_cast<BYTE*>(&containerId),
        &propertyBufferSize,
        0); // ulFlags

    if ((cr != CR_SUCCESS) || (propertyType != DEVPROP_TYPE_GUID)) {
        throw wexception(L"Failed to get containerId. (cr = 0x%x, DeviceInterface = %s)\n", cr, DeviceInterface);
    }

    return containerId == SystemContainerId;
}


//
// Get the list of all available UART device interfaces
//
std::vector<std::wstring> GetDeviceList(ULONG* length, CONFIGRET* cr) {
    std::vector<std::wstring> interfaces;

    // Get the size of the list of available device interfaces
    *cr = CM_Get_Device_Interface_List_SizeW(
        length,
        const_cast<GUID*>(&GUID_DEVINTERFACE_COMPORT),
        nullptr,        // pDeviceID
        CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

    if ((*cr != CR_SUCCESS) || (*length == 0)) {
        throw wexception(L"Failed to get size of device interface list. (length = %lu, cr = 0x%x)\n", *length, *cr);
    }

    std::vector<WCHAR> buf(*length);

    // Get the list of available device interfaces and store it to buffer
    *cr = CM_Get_Device_Interface_ListW(
        const_cast<GUID*>(&GUID_DEVINTERFACE_COMPORT),
        nullptr,        // pDeviceID
        buf.data(),
        static_cast<ULONG>(buf.size()),
        CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

    if ((*cr != CR_SUCCESS) || (*length == 0)) {
        throw wexception(L"Failed to get device interface list. (length = %lu, cr = 0x%x)\n", *length, *cr);
    }

    if (!buf[0]) {
        wprintf(L"No serial devices were found.\n");
        return interfaces;
    }

    *buf.rbegin() = UNICODE_NULL;

    for (PCWSTR deviceInterface = buf.data(); *deviceInterface; deviceInterface += wcslen(deviceInterface) + 1) {
        interfaces.push_back(deviceInterface);
    }
    std::sort(interfaces.begin(), interfaces.end());

    return interfaces;
}


//
// Print the list of all available UART device interfaces and information about them
//
void ListDevices(ULONG *length, CONFIGRET *cr) {
    std::vector<std::wstring> interfaces = GetDeviceList(length, cr);

    for (auto wstrDeviceInterface : interfaces) {
        PCWSTR deviceInterface = wstrDeviceInterface.c_str();
        auto portName = GetPortName(deviceInterface);
        bool isRestricted = IsRestricted(deviceInterface);
        bool isInSystemContainer = IsInSystemContainer(deviceInterface);
        bool isUwpAccessible = !isInSystemContainer ||
            (isInSystemContainer && !isRestricted);

        wprintf(
            L"%s\n"
            L"    PortName: %s\n"
            L"    In System Container: %s\n"
            L"    Restricted: %s\n"
            L"    UWP Accessible: %s\n"
            L"\n",
            deviceInterface,
            portName.empty() ? L"(not set)" : portName.c_str(),
            isInSystemContainer ? L"TRUE" : L"FALSE",
            isRestricted ? L"TRUE" : L"FALSE",
            isUwpAccessible ? L"TRUE" : L"FALSE");
    }
    return;
}


//
// Get the first device from the list of devices
//
std::wstring GetFirstDevice(ULONG* length, CONFIGRET* cr) {
    std::vector<std::wstring> interfaces = GetDeviceList(length, cr);

    PCWSTR deviceInterface = interfaces[0].c_str();

    return GetPortName(deviceInterface);
}


//
// Check and parse configuration parameters
//
bool CheckParams(int argc, _In_reads_(argc) const wchar_t* argv[], _Out_ SerialParamMask* maskPtr) {
    *maskPtr = SerialParamMask();

    for (int i = 0; i < argc; i++) {
        std::wstring parameter(argv[i]);
        std::wstring::size_type found = parameter.find_first_of(L'=');

        if (found == parameter.npos) {
            // ignore
        }

        if ((found + 1) >= parameter.length()) {
            fwprintf(stderr, L"Expecting value after '=': %s", argv[i]);
            return false;
        }

        std::wstring name = parameter.substr(0, found);
        std::wstring value = parameter.substr(found + 1);

        if (name == L"parity") {
            // parity={n|e|o|m|s}
            if (value.length() != 1) {
                fwprintf(stderr, L"Expecting n|e|o|m|s for parity: %s", parameter.c_str());
                return false;
            }

            switch (value[0]) {
            case L'n':
                glob_Parity = 0;
                break;
            case L'o':
                glob_Parity = 1;
                break;
            case L'e':
                glob_Parity = 2;
                break;
            case L'm':
                glob_Parity = 3;
                break;
            case L's':
                glob_Parity = 4;
                break;
            default:
                fwprintf(stderr, L"Expecting n|e|o|m|s for parity: %s", parameter.c_str());
                return false;
            }

            maskPtr->ParitySet = true;
        }
        else if (name == L"data") {
            // data={5|6|7|8}
            if (value.length() != 1) {
                fwprintf(stderr, L"Expecting 5|6|7|8 for data length: %s", parameter.c_str());
                return false;
            }

            switch (value[0]) {
            case L'5':
                glob_Data = 5;
                break;
            case L'6':
                glob_Data = 6;
                break;
            case L'7':
                glob_Data = 7;
                break;
            case L'8':
                glob_Data = 8;
                break;
            default:
                fwprintf(stderr, L"Expecting 5|6|7|8 for data length: %s", parameter.c_str());
                return false;
            }

            maskPtr->DataLengthSet = true;
        }
        else if (name == L"stop") {
            // stop={1|1.5|2}
            if (value == L"1") {
                glob_Stop = 0;
            }
            else if (value == L"1.5") {
                glob_Stop = 1;
            }
            else if (value == L"2") {
                glob_Stop = 2;
            }
            else {
                fwprintf(stderr, L"Expecting 1|1.5|2 for stop bits: %s", parameter.c_str());
                return false;
            }

            maskPtr->StopBitsSet = true;
        }
        else if (name == L"xon") {
            // xon={on|off}
            if (value == L"on") {
                glob_XON = true;
            }
            else if (value == L"off") {
                glob_XON = false;
            }
            else {
                fwprintf(stderr, L"Expecting on|off for xon: %s", parameter.c_str());
                return false;
            }

            maskPtr->XonSet = true;
        }
        else if (name == L"odsr") {
            // odsr={on|off}
            if (value == L"on") {
                glob_oDSR = true;
            }
            else if (value == L"off") {
                glob_oDSR = false;
            }
            else {
                fwprintf(stderr, L"Expecting on|off for odsr: %s", parameter.c_str());
                return false;
            }

            maskPtr->OdsrSet = true;
        }
        else if (name == L"octs") {
            // octs={on|off}
            if (value == L"on") {
                glob_oCTS = true;
            }
            else if (value == L"off") {
                glob_oCTS = false;
            }
            else {
                fwprintf(stderr, L"Expecting on|off for octs: %s", parameter.c_str());
                return false;
            }

            maskPtr->OctsSet = true;
        }
        else if (name == L"dtr") {
            // dtr={on|off|hs}
            if (value == L"on") {
                glob_DTR = 1;
            }
            else if (value == L"off") {
                glob_DTR = 0;
            }
            else if (value == L"hs") {
                glob_DTR = 2;
            }
            else {
                fwprintf(stderr, L"Expecting on|off|hs for dtr: %s", parameter.c_str());
                return false;
            }

            maskPtr->DtrSet = true;
        }
        else if (name == L"rts") {
            // rts={on|off|hs|tg}
            if (value == L"on") {
                glob_RTS = 1;
            }
            else if (value == L"off") {
                glob_RTS = 0;
            }
            else if (value == L"hs") {
                glob_RTS = 2;
            }
            else if (value == L"tg") {
                glob_RTS = 3;
            }
            else {
                fwprintf(stderr, L"Expecting on|off|hs|tg for rts: %s", parameter.c_str());
                return false;
            }

            maskPtr->RtsSet = true;
        }
        else if (name == L"idsr") {
            // idsr={on|off}
            if (value == L"on") {
                glob_iDSR = true;
            }
            else if (value == L"off") {
                glob_iDSR = false;
            }
            else {
                fwprintf(stderr, L"Expecting on|off for idsr: %s", parameter.c_str());
                return false;
            }

            maskPtr->IdsrSet = true;
        }
    }
    return true;
}


//
// Write message to serial device
// 
// create overlapped event and write data to serial device
//
void WriteSerial(HANDLE serialHandle) {
    Sleep(1000);
    Microsoft::WRL::Wrappers::Event overlappedEvent(CreateEventW(NULL, TRUE, FALSE, NULL));
    if (!overlappedEvent.IsValid()) {
        throw wexception(L"Failed to create event for overlapped IO. (GetLastError() = 0x%x)", GetLastError());
    }

    auto overlapped = OVERLAPPED();
    overlapped.hEvent = overlappedEvent.Get();
    DWORD toWrite;
    DWORD bytesWritten = 0;

    // for loopback communication - set fixed number of bytes to write (workaround to avoid timing out)
    if (deviceName == L"UART1" || deviceName == L"UART3") {
        toWrite = MAX_DATA_LEN;
    }
    // for communication with COM ports - adjust the bytes to write based on buffer length
    else {
        toWrite = DWORD(strlen(send_buffer));
    }
    
    // continue to write data until the whole buffer is written
    while (toWrite - bytesWritten > 0) {
        DWORD tmpBytesWritten;
        if (!WriteFile(serialHandle, send_buffer + bytesWritten, toWrite - bytesWritten, &tmpBytesWritten, &overlapped) 
            && (GetLastError() != ERROR_IO_PENDING)) {
            throw wexception(L"Write to serial device failed. (GetLastError() = 0x%x)", GetLastError());
        }
        bytesWritten = bytesWritten + tmpBytesWritten;
        if (tmpBytesWritten > 0) {
            wprintf(L"Wrote: %d B.\n", tmpBytesWritten);
            fflush(stdout);
        }

        if (!GetOverlappedResult(serialHandle, &overlapped, &tmpBytesWritten, TRUE)) {
            throw wexception(L"GetOverlappedResult() for SetCommTimeouts() failed. (GetLastError() = 0x%x)", GetLastError());
        }
        if (tmpBytesWritten > 0) {
            wprintf(L"Wrote: %d B.\n", tmpBytesWritten);
            fflush(stdout);
        }
        bytesWritten = bytesWritten + tmpBytesWritten;
    }
    wprintf(L"Finished writing.\n\n");
    quit = true;
}


//
// Thread procedure for writing
//
DWORD CALLBACK WriteSerialThreadProc(PVOID Param) {
    try {
        WriteSerial(static_cast<HANDLE>(Param));
    }
    catch (const wexception& ex) {
        std::wcerr << L"Error: " << ex.wwhat() << L"\n";
        return 1;
    }

    return 0;
}


//
// Read message from serial device
// 
// create overlapped event and read data
//
void ReadSerial(HANDLE serialHandle) {
    Microsoft::WRL::Wrappers::Event overlappedEvent(CreateEventW(NULL, TRUE, FALSE, NULL));
    if (!overlappedEvent.IsValid()) {
        throw wexception(L"Failed to create event for overlapped IO. (GetLastError() = 0x%x)", GetLastError());
    }

    
    unsigned bytesRead = 0;
    // continue to read data until the message is too long or until timeout
    while (!forceQuit) {
        auto reader = OVERLAPPED();
        reader.hEvent = overlappedEvent.Get();
        BYTE buf[128];
        DWORD tmpBytesRead = 0;

        // Set up read
        if (!ReadFile(serialHandle, buf, sizeof(buf), &tmpBytesRead, &reader) && (GetLastError() != ERROR_IO_PENDING)) {
            DWORD error = GetLastError();
			if (error == ERROR_OPERATION_ABORTED) {
				fwprintf(stderr, L"ReadFile: ERROR_OPERATION_ABORTED\n");
				return; // error is due to application exit
			}
			throw wexception(L"Failed to read from serial device. (GetLastError = 0x%x)", error);
        }
        bytesRead += tmpBytesRead;

        Sleep(100);
        #define READ_TIMEOUT 3000 // ms
        DWORD status;
        std::string readMsg;
        status = WaitForSingleObject(reader.hEvent, READ_TIMEOUT);
        switch (status) {
            // Reading
            case WAIT_OBJECT_0:
                if (!GetOverlappedResult(serialHandle, &reader, &tmpBytesRead, TRUE)) {
                    // Error in communication, report it.
                    DWORD error = GetLastError();
                    if (error == ERROR_OPERATION_ABORTED) {
                        fwprintf(stderr, L"GetOverlappedResult: ERROR_OPERATION_ABORTED\n");
                        return; // error is due to application exit
                    }
                    throw wexception(L"GetOverlappedResult() for ReadFile() failed. (GetLastError() = 0x%x)", error);
                }
                else {
                    for (unsigned int i = 0; i < tmpBytesRead; i++) {
                        readMsg.push_back(buf[i]);
                    }
                    std::cout << ("%s", readMsg) << std::endl;
                    fflush(stdout);
                }

                bytesRead += tmpBytesRead;
                fflush(stdout);

                if (bytesRead == MAX_DATA_LEN) {
                    quit = true;
                    wprintf(L"\n");
                    wprintf(L"Read: %d B.\n", bytesRead);
                    wprintf(L"Finished reading.\n\n");
                    fflush(stdout);
                    return;
                }
                break;

            case WAIT_TIMEOUT:
                if (forceQuit || quit) {
                    wprintf(L"\n");
                    wprintf(L"Read: %d B.\n", bytesRead);
                    wprintf(L"Finished reading.\n\n");
                    fflush(stdout);
                    return;
                }
                else {
                    forceQuit = true;
                }
                break;

            default:
                forceQuit = true;
                DWORD error = GetLastError();
                fwprintf(stderr, L"WaitForSingleObject() for ReadFile() failed. (GetLastError() = 0x%x)", error);
                throw wexception(L"WaitForSingleObject() for ReadFile() failed. (GetLastError() = 0x%x)", error);
                break;
        }
    }
}


//
// Set up and start the serial communication
// 
// create and set up a handle to the selected serial device,
// set the configuration of the device and timeouts,
// write and read messages
//
void StartCommunication(PCWSTR deviceInterface, _In_ DCB *dcbPtr, SerialParamMask paramMask) {
    DCB dcb = {sizeof(DCB)};

	Microsoft::WRL::Wrappers::FileHandle serialHandle(CreateFileW(
		deviceInterface,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
		NULL));

	if (!serialHandle.IsValid()) {
		throw wexception(L"Failed to open handle to serial device.(DevicePath = %s, GetLastError() = 0x%x)", deviceInterface, GetLastError());
	}

    // initialize the communications parameters for the device
	if (!SetupComm(serialHandle.Get(), 512, 512)) {
		throw wexception(L"SetupComm() failed. (GetLastError() = 0x%x)", GetLastError());
	}

    // get the current configuration of the device
    if (!GetCommState(serialHandle.Get(), &dcb)) {
        throw wexception(L"GetCommState() failed. Failed to get current device configuration. (GetLastError() = 0x%x)", GetLastError());
    }

    // set the device configuration - if a parameter is not set, keep the default value of the parameter
    
    // baudrate
    std::wcout << L"Select baudrate: ";
    std::wstring line;
    if (std::getline(std::wcin, line)) {
        std::wistringstream linestream(line);
        std::wstring baudrateString;
        linestream >> baudrateString;
        if (!baudrateString.empty()) {
            try {
                glob_Baud = stoi(baudrateString);
            }
            catch (const wexception& ex) {
                std::wcout << L"Expecting integer.\n";
                std::wcerr << L"Error: " << ex.wwhat() << L"\n";
                return;
            }
            paramMask.BaudSet = true;
        }
    }
    std::wcout << L"\n";

    if (paramMask.BaudSet) {
        dcb.BaudRate = dcbPtr->BaudRate;
    }
    dcb.BaudRate = glob_Baud;

    // parity
    if (paramMask.ParitySet) {
        dcb.Parity = dcbPtr->Parity;
        dcb.fParity = dcbPtr->fParity;
    }
    dcb.fParity = TRUE;
    dcb.Parity = glob_Parity;

    // data length
    if (paramMask.DataLengthSet) {
        dcb.ByteSize = dcbPtr->ByteSize;
    }
    dcb.ByteSize = glob_Data;

    // stop bits
    if (paramMask.StopBitsSet) {
        dcb.StopBits = dcbPtr->StopBits;
    }
    if (glob_Stop == 0) {
        dcb.StopBits = BYTE(SerialStopBits::One);
    }
    else if (glob_Stop == 1) {
        dcb.StopBits = BYTE(SerialStopBits::OnePointFive);
    }
    else if (glob_Stop == 2) {
        dcb.StopBits = BYTE(SerialStopBits::Two);
    }

    // XON
    if (paramMask.XonSet) {
        dcb.fInX = dcbPtr->fInX;
        dcb.fOutX = dcbPtr->fOutX;
    }
    dcb.fInX = glob_XON;
    dcb.fOutX = glob_XON;

    // oDSR
    if (paramMask.OdsrSet) {
        dcb.fOutxDsrFlow = dcbPtr->fOutxDsrFlow;
    }
    dcb.fOutxDsrFlow = glob_oDSR;

    // oCTS
    if (paramMask.OctsSet) {
        dcb.fOutxCtsFlow = dcbPtr->fOutxCtsFlow;
    }
    dcb.fOutxCtsFlow = glob_oCTS;

    // DTR
    if (paramMask.DtrSet) {
        dcb.fDtrControl = dcbPtr->fDtrControl;
    }
    dcb.fDtrControl = glob_DTR;

    // RTS
    if (paramMask.RtsSet) {
        dcb.fRtsControl = dcbPtr->fRtsControl;
    }
    dcb.fRtsControl = glob_RTS;

    // iDSR
    if (paramMask.IdsrSet) {
        dcb.fDsrSensitivity = dcbPtr->fDsrSensitivity;
    }
    dcb.fDsrSensitivity = glob_iDSR;

    // configure the device
    if (!SetCommState(serialHandle.Get(), &dcb)) {
        throw wexception(L"SetCommState() failed. Failed to set device to desired configuration. (GetLastError() = 0x%x)", GetLastError());
    }

    // set the communication timeouts
    auto commTimeouts = COMMTIMEOUTS();
	commTimeouts.ReadIntervalTimeout = 10;              // The maximum time allowed to elapse before the arrival of the next byte on the communications line, in milliseconds.
	commTimeouts.ReadTotalTimeoutConstant = 0;          // The multiplier used to calculate the total time-out period for read operations, in milliseconds.
	commTimeouts.ReadTotalTimeoutMultiplier = 0;        // A constant used to calculate the total time-out period for read operations, in milliseconds.
	commTimeouts.WriteTotalTimeoutConstant = 0;         // The multiplier used to calculate the total time-out period for write operations, in milliseconds.
	commTimeouts.WriteTotalTimeoutMultiplier = 0;       // A constant used to calculate the total time-out period for write operations, in milliseconds.

    if (!SetCommTimeouts(serialHandle.Get(), &commTimeouts)) {
        throw wexception(L"SetCommTimeouts failed. (GetLastError() = 0x%x)", GetLastError());
    }

    DCB actualDcb;
    if (!GetCommState(serialHandle.Get(), &actualDcb)) {
        throw wexception(L"Failed to get current comm state. (GetLastError() = 0x%x)", GetLastError());
    }

    wprintf(
        L"CONFIGURATION:               \n"
        L"                    baud = %d\n"
        L"                  parity = %s\n"
        L"               data bits = %d\n"
        L"               stop bits = %s\n"
        L"   XON/XOFF flow control = %s\n"
        L" output DSR flow control = %s\n"
        L" output CTS flow control = %s\n"
        L"             DTR control = %s\n"
        L"             RTS control = %s\n"
        L" DSR circuit sensitivity = %s\n",
        actualDcb.BaudRate,
        StringFromParity(actualDcb.Parity),
        actualDcb.ByteSize,
        StringFromStopBits(actualDcb.StopBits),
        (actualDcb.fInX && actualDcb.fOutX) ? L"on" : L"off",
        actualDcb.fOutxDsrFlow ? L"on" : L"off",
        actualDcb.fOutxCtsFlow ? L"on" : L"off",
        StringFromDtrControl(actualDcb.fDtrControl),
        StringFromRtsControl(actualDcb.fRtsControl),
        actualDcb.fDsrSensitivity ? L"on" : L"off");

    wprintf(L"\n=====================   Connected - hit ctrl+c to exit   =====================\n");

    // repeat until the user stops the communication by pressing ctrl+c
    while (1) {
        forceQuit = false;

        std::wcout << L"Send message: ";
        std::wstring messageLine;
        if (std::getline(std::wcin, messageLine)) {
            std::wistringstream linestream(messageLine);
            std::wstring message;
            linestream >> message;
            if (message.empty()) {
                continue;
            }
            else {
                memset(send_buffer, '\0', MAX_DATA_LEN);
                for (int i = 0; i < message.length()+1; i++) {
                    send_buffer[i] = static_cast<char>(message[i]);
                }
            }
        }

        // create thread for writing the message
        ThreadHandle thread(CreateThread(NULL, 0, WriteSerialThreadProc, serialHandle.Get(), 0, NULL));
        if (!thread.IsValid()) {
            throw wexception(L"Failed to create WriteSerialThreadProc thread. (GetLastError() = 0x%x)",
                GetLastError());
        }
        
        // read the message from other device
        ReadSerial(serialHandle.Get());

        WaitForSingleObject(thread.Get(), 1000);
        thread.Close();
    }
}


//
// Set up parameters for the serial communication
//
void SetUpCommunication(std::wstring device, ULONG* length, CONFIGRET* cr) {
    memset(send_buffer, '\0', MAX_DATA_LEN);

    for (auto wstrDeviceInterface : GetDeviceList(length, cr)) {
        PCWSTR deviceInterface = wstrDeviceInterface.c_str();
        try {
            if (GetPortName(deviceInterface) == deviceName) {
                auto paramMask = SerialParamMask();
                auto dcb = DCB();
                dcb.DCBlength = sizeof(DCB);
                paramMask.DtrSet = 0;
                StartCommunication(deviceInterface, &dcb, paramMask);
                break;
            }
        }
        catch (const wexception& ex) {
            std::wcerr << L"Error: " << ex.wwhat() << L"\n";
            return;
        }
    }
}


int __cdecl wmain(int argc, _In_reads_(argc) const wchar_t* argv[]) {
    ULONG length = 0;
    CONFIGRET cr = CR_DEFAULT;
    auto paramMask = SerialParamMask();
    auto dcb = DCB();
    dcb.DCBlength = sizeof(DCB);
    appName = argv[0];

    if (argc < 2) {
        std::wcerr << L"Missing required command line parameter device_path.\n";
        PrintUsage();
        return 1;
    }

    for (int i = 0; i < argc; i++) {
        if (!_wcsicmp(argv[i], L"-?") || !_wcsicmp(argv[i], L"-h") || !_wcsicmp(argv[i], L"-help")) {
            PrintUsage();
            return 0;
        }
        else if (!_wcsicmp(argv[i], L"-list") || !_wcsicmp(argv[i], L"-l")) {
            try {
                ListDevices(&length, &cr);
            }
            catch (const wexception& ex) {
                std::wcerr << L"Error: " << ex.wwhat() << L"\n";
                return 1;
            }
            return 0;
        }
    }

    int optind = 1;
    deviceName = argv[optind];

    // if the first parameter is the device name, increment option index for parsing configuration parameters
    if (deviceName.find_first_of(L'=') == deviceName.npos) {
        optind++;
    }
    else {
        // if the device interface is not specified, use the first one from the list of available devices
        deviceName = GetFirstDevice(&length, &cr);
    }

    // check all the parameters
    if (!CheckParams(argc - optind, argv + optind, &paramMask)) {
        return 1;
    }
    else {
        try {
            // set up and start the serial communication
            SetUpCommunication(deviceName, &length, &cr);
        }
        catch (const wexception& ex) {
            std::wcerr << L"Error: " << ex.wwhat() << L"\n";
            return 1;
        }
    }
    
    return 0;
}