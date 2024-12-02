/*
 * Copyright 2022 NXP
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
//
#define _GNU_SOURCE
#include <wrl.h>

#include <string>

#include <strsafe.h>
#include <cfgmgr32.h>
#include <ppl.h>
#include <wrl.h>
#include <winioctl.h>
#include <cfgmgr32.h>
#include <initguid.h>

#include <d2d1.h>
#include <d2d1helper.h>
#include <dwrite.h>
#include <wincodec.h>

#include "imxcsi\Public.h"
#include "imxmipi\Public.h"
#include "imxsns_ov5640\Public.h"
#include "Resource.h"


#pragma comment(lib,"msimg32")
 // #pragma comment(lib,"msimg")
#pragma comment(lib,"Cfgmgr32")
 // #pragma comment(lib,"Cfgmgr")

using namespace Microsoft::WRL;
using namespace Microsoft::WRL::Wrappers;

#define MsgBuffLen 1024
char *MsgBuff = NULL;

bool dontExit = true;
bool requested = false;
HANDLE hRequestedMutex;
bool WaitFrame = false;
DWORD WaitFrameId = 0xffff;
int run = 0;

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

// Forward declarations of functions included in this code module:
// LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

// HWND hWnd;

// Util ---


class wexception {
public:
	explicit wexception(_In_range_(< , 0) HRESULT Hr) noexcept : hr(Hr) {}

	wexception(wexception&& other) noexcept :
		hr(other.hr),
		msg(std::move(other.msg))
	{}

	static wexception make(
		HRESULT HResult,
		_In_ _Printf_format_string_ PCWSTR Format,
		...
	) noexcept
	{
		wexception ex(HResult);

		enum : ULONG { BUF_LEN = 512 };
		std::unique_ptr<WCHAR[]> buf(new (std::nothrow) WCHAR[BUF_LEN]);
		if (!buf) {
			return ex;
		}

		va_list argList;
		va_start(argList, Format);

		HRESULT printfHr = StringCchVPrintfW(
			buf.get(),
			BUF_LEN,
			Format,
			argList);

		va_end(argList);

		if (SUCCEEDED(printfHr)) {
			ex.msg = std::move(buf);
		}

		return ex;
	}

	const wchar_t* wwhat() const
	{
		return msg ? msg.get() : L"";
	}

	HRESULT HResult() const
	{
		return this->hr;
	}

	// disable copy
	// wexception (const wexception&) = delete;
	wexception& operator= (const wexception&) = delete;

private:
	HRESULT hr;
	std::unique_ptr<WCHAR[]> msg;
};

// Image buffers -----

BITMAPINFO bmi;
BITMAPINFO bmi2;
const unsigned wi = 1280;
const unsigned he = 720;

const unsigned screen_wi = 640;
const unsigned screen_he = 480;

void initBmi()
{
	memset(&bmi, 0, sizeof(bmi));
	bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmi.bmiHeader.biWidth = wi;
	bmi.bmiHeader.biHeight = he; // top-down image 
	bmi.bmiHeader.biPlanes = 1;
	bmi.bmiHeader.biBitCount = 24;
	bmi.bmiHeader.biCompression = BI_RGB;
	bmi.bmiHeader.biSizeImage = bmi.bmiHeader.biWidth * bmi.bmiHeader.biHeight * bmi.bmiHeader.biBitCount;
}

void initBmi2()
{
	memset(&bmi2, 0, sizeof(bmi2));
	bmi2.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmi2.bmiHeader.biWidth = screen_wi;
	bmi2.bmiHeader.biHeight = screen_he; // top-down image 
	bmi2.bmiHeader.biPlanes = 1;
	bmi2.bmiHeader.biBitCount = 24;
	bmi2.bmiHeader.biCompression = BI_RGB;
	bmi2.bmiHeader.biSizeImage = bmi2.bmiHeader.biWidth * bmi2.bmiHeader.biHeight * bmi2.bmiHeader.biBitCount;
}

PUCHAR lpvBits = NULL;
PUCHAR dump = NULL;
PUCHAR screen = NULL;

bool loadTestFile() {
	const char * const file = "fri3_720.yuv";
	FILE *f;
	errno_t e = fopen_s(&f, file, "rb");
	if (e != 0)
		return true;
	unsigned loaded = (unsigned)fread(dump, sizeof(char), bmi.bmiHeader.biSizeImage, f);
	fclose(f);
	printf("Loaded %d (max %d) from %s.\n", loaded, bmi.bmiHeader.biSizeImage, file);
	return false;
}


char cfg[200];
unsigned readCfg() {
    const char* const file = "cfg.txt";
    FILE* f;
    errno_t e = fopen_s(&f, file, "rt");
    if (e != 0) {
        volatile auto error = GetLastError();
        error;
        return 0;
    }

    unsigned loaded = (unsigned)fread(cfg, sizeof(char), sizeof(cfg), f);
    // fclose(f);
    // printf("Loaded %d (max %d) from %s.\n", loaded, bmi.bmiHeader.biSizeImage, file);
    return loaded;
}


template<class Interface>
inline void SafeRelease(
	Interface **ppInterfaceToRelease
)
{
	if (*ppInterfaceToRelease != NULL)
	{
		(*ppInterfaceToRelease)->Release();

		(*ppInterfaceToRelease) = NULL;
	}
}


#ifndef Assert
#if defined( DEBUG ) || defined( _DEBUG )
#define Assert(b) do {if (!(b)) {OutputDebugStringA("Assert: " #b "\n");}} while(0)
#else
#define Assert(b)
#endif //DEBUG || _DEBUG
#endif



#ifndef HINST_THISCOMPONENT
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#define HINST_THISCOMPONENT ((HINSTANCE)&__ImageBase)
#endif

// Peroformance counter, FPS report --
LARGE_INTEGER cnt, StartingTime, EndingTime, ElapsedMicroseconds;
#define FILTER_LEN 30
unsigned sumElapsedMs[FILTER_LEN];
unsigned filter_cnt = 0;
unsigned long elapsedSeconds = 0;
LARGE_INTEGER Frequency;

// Camera controls ---
UINT8 csiLanes = 2;
// camera_config_t cam_cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatUYVY , kVIDEO_PixelFormatUYVY, 25, 1, csiLanes, 0x9 };
// camera_config_t cam_cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 25, 0, csiLanes, 0x9 };
// 
camera_config_t cam_cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatNV12, 25, 0, csiLanes };
// camera_config_t cam_cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatUYVY, 25, 0, csiLanes, 0x9 };
// camera_config_t cam_cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 25, 0, csiLanes, 0x9 };

// camera_config_t cam_cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatNV12 , kVIDEO_PixelFormatNV12, 25, 1, csiLanes, 0x9 };
// camera_config_t cam_cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatRGB565 , kVIDEO_PixelFormatRGB565, 25, 1, csiLanes, 0x9 };
// camera_config_t cam_cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatRGB888 , kVIDEO_PixelFormatRGB888, 25, 1, csiLanes, 0x9 };

void ReportError(char *msg = "")
{
	// LPWSTR lpMessage;
	DWORD dwErrCode = GetLastError();

	LPWSTR lpBuff;


	FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,
		NULL, // no source buffer needed
		dwErrCode, // error code for this message
		NULL, // default language ID
		(LPTSTR)&lpBuff, // allocated by fcn
		NULL, // minimum size of buffer
		NULL); // no inserts

	// wprintf(L"Cam Error: 0x%x: %ws\n", dwErrCode, lpMessage);
	snprintf(MsgBuff, MsgBuffLen, "Execution failed HR=%08x\n" "%s\n" "%S\n", dwErrCode, msg, lpBuff);
	MessageBoxA(NULL, MsgBuff, "Error", MB_ICONERROR | MB_OK);

	LocalFree(lpBuff); // Free the memory allocated by FormatMessage

}

// const wchar_t CameraDevicePath[] = L"\\??\\ACPI#NXP5640A#0#{3b4611d3-4330-4800-8a7a-855f98444a59}";
const wchar_t CameraDevicePath[] = L"\\\\?\\ACPI#NXP5640A#0#{3b4611d3-4330-4800-8a7a-855f98444a59}";
wchar_t Camera_driver_id_str = '0'; // User can specify instance

#if 0
bool Camera_do_write_irp(UCHAR *Value, DWORD NumBytes, DWORD CtlCode)
{
	bool Status = 0;

	const size_t pathLen = sizeof(CameraDevicePath) / sizeof(CameraDevicePath[0]); // wcslen(DevicePath);
	const size_t pathSpace = pathLen + 4;
	wchar_t devicePath[pathSpace];// = new wchar_t[pathSpace];
	if (devicePath != NULL) {
		wcscpy_s(devicePath, pathLen, &CameraDevicePath[0]);
		// devicePath[pathLen -1] = '\\';
		// devicePath[pathLen +0] = driver_id_str;
		// devicePath[pathLen +1] = 0;
		devicePath[18] = Camera_driver_id_str;
		wprintf(L"CreateFileW => %s\r\n", devicePath);
		// DWORD	junk;
		// int		outBuff[64] = { IOCTL_DISK_GET_DRIVE_GEOMETRY };
		HANDLE hDevice = CreateFileW(devicePath, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
		// HANDLE hDevice = CreateFileA(DevicePath, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
		if (hDevice == INVALID_HANDLE_VALUE) {
			ReportError();
			Status = 1;
		}
		else {
			wprintf(L"Cam Driver se otevrel!\n");
			// std::cout << "Driver se otevrel!\n";
			Status = DeviceIoControl(	hDevice,                       // device to be queried
										CtlCode, // operation to perform 458752
										Value, NumBytes,                       // no input buffer
										NULL, 0,             // output buffer
										NULL,// &junk,                         // # bytes returned
										(LPOVERLAPPED)NULL);
			if (Status == 0) {         // synchronous I/O
				ReportError();
			}
			else {
				wprintf(L"Cam Iorequest prosel\n");
				// std::cout << "Iorequest prosel\n";
			}

			if (CloseHandle(hDevice) == 0) {
				ReportError();
				Status = 1;
			}
			wprintf(L"Cam Driver se zavrel!\n");
			// std::cout << "Driver se zavrel!\n";
		}
	}
	return Status;
}

bool CameraStart() {
	// UCHAR Value = 0;

	// camera_config_t cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 15, 1, 2, 0x12 };
	//camera_config_t cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatRGB565 , kVIDEO_PixelFormatRGB565, 15, 1, 2, 0x12 };
	// cfg.tHsSettle_EscClk = (UINT8)GetSettle(cfg.resolution, cfg.framePerSec);

	return Camera_do_write_irp((PUCHAR)&cam_cfg, sizeof(cam_cfg), IOCTL_SNS_CONFIGURE);
}
#endif
// CSI controls --
FileHandle CsiHandle;
FileHandle MipiHandle;
FileHandle CamHandle;
HANDLE CsiOverlappedEvent = NULL;
auto CsiOverlappedContext = OVERLAPPED();

std::wstring GetInterfacePath(const GUID& InterfaceGuid)
{
	ULONG length;
	CONFIGRET cr = CM_Get_Device_Interface_List_SizeW(
		&length,
		const_cast<GUID*>(&InterfaceGuid),
		nullptr,        // pDeviceID
		CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

	if (cr != CR_SUCCESS) {
		throw wexception::make(
			HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_NOT_FOUND)),
			L"Failed to get size of device interface list. (cr = 0x%x)",
			cr);
	}

	if (length < 2) {
		throw wexception::make(
			HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_NOT_FOUND)),
			L"The IMXVPUKM device was not found on this system. (cr = 0x%x)",
			cr);
	}

	std::unique_ptr<WCHAR[]> buf(new WCHAR[length]);
	cr = CM_Get_Device_Interface_ListW(
		const_cast<GUID*>(&InterfaceGuid),
		nullptr,        // pDeviceID
		buf.get(),
		length,
		CM_GET_DEVICE_INTERFACE_LIST_PRESENT);

	if (cr != CR_SUCCESS) {
		throw wexception::make(
			HRESULT_FROM_WIN32(CM_MapCrToWin32Err(cr, ERROR_NOT_FOUND)),
			L"Failed to get device interface list. (cr = 0x%x)",
			cr);
	}

	// Return the first string in the multistring
	return std::wstring(buf.get());
}

FileHandle OpenCsiHandle(DWORD additionalFlags = 0) //, const wchar_t id = 0)
{
	auto interfacePath = GetInterfacePath(GUID_DEVINTERFACE_IMXCSI);
	// This could be used to chose camera device
	// if (id != 0) {
	// 	interfacePath.back() = id;
	// }
	printf("  Opening device %S\n", interfacePath.c_str());

	FileHandle fileHandle(CreateFile(// CreateFile2
		interfacePath.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,          // dwShareMode
		nullptr,    // lpSecurityAttributes
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | additionalFlags,
		nullptr));  // hTemplateFile

	if (!fileHandle.IsValid()) {
		if (GetLastError() == ERROR_ACCESS_DENIED) {
			// Try opening read-only
			fileHandle.Attach(CreateFile(
				interfacePath.c_str(),
				GENERIC_READ,
				0,          // dwShareMode
				nullptr,    // lpSecurityAttributes
				OPEN_EXISTING,
				FILE_ATTRIBUTE_NORMAL | additionalFlags,
				nullptr));  // hTemplateFile

			if (fileHandle.IsValid()) {
				return fileHandle;
			}
		}

		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"Failed to open a handle to the VPU device. "
			L"(hr = 0x%x, interfacePath = %s)",
			HRESULT_FROM_WIN32(GetLastError()),
			interfacePath.c_str());
	}

	return fileHandle;
}

FileHandle OpenMipiHandle(DWORD additionalFlags = 0, const wchar_t id = 0)
{
	auto interfacePath = GetInterfacePath(GUID_DEVINTERFACE_IMXMIPI);
	if (id != 0) {
		interfacePath.back() = id;
	}
	printf("  Opening device %S\n", interfacePath.c_str());

	FileHandle fileHandle(CreateFile(// CreateFile2
		interfacePath.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,          // dwShareMode
		nullptr,    // lpSecurityAttributes
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | additionalFlags,
		nullptr));  // hTemplateFile

	if (!fileHandle.IsValid()) {
		if (GetLastError() == ERROR_ACCESS_DENIED) {
			// Try opening read-only
			fileHandle.Attach(CreateFile(
				interfacePath.c_str(),
				GENERIC_READ,
				0,          // dwShareMode
				nullptr,    // lpSecurityAttributes
				OPEN_EXISTING,
				FILE_ATTRIBUTE_NORMAL | additionalFlags,
				nullptr));  // hTemplateFile

			if (fileHandle.IsValid()) {
				return fileHandle;
			}
		}

		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"Failed to open a handle to the VPU device. "
			L"(hr = 0x%x, interfacePath = %s)",
			HRESULT_FROM_WIN32(GetLastError()),
			interfacePath.c_str());
	}

	return fileHandle;
}

FileHandle OpenCameraHandle(DWORD additionalFlags = 0, const wchar_t id = 0)
{
	auto interfacePath = GetInterfacePath(GUID_DEVINTERFACE_SNS0_DRIVER);
	if (id != 0) {
		interfacePath.back() = id;
	}
	printf("  Opening device %S\n", interfacePath.c_str());

	FileHandle fileHandle(CreateFile(// CreateFile2
		interfacePath.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,          // dwShareMode
		nullptr,    // lpSecurityAttributes
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | additionalFlags,
		nullptr));  // hTemplateFile

	if (!fileHandle.IsValid()) {
		if (GetLastError() == ERROR_ACCESS_DENIED) {
			// Try opening read-only
			fileHandle.Attach(CreateFile(
				interfacePath.c_str(),
				GENERIC_READ,
				0,          // dwShareMode
				nullptr,    // lpSecurityAttributes
				OPEN_EXISTING,
				FILE_ATTRIBUTE_NORMAL | additionalFlags,
				nullptr));  // hTemplateFile

			if (fileHandle.IsValid()) {
				return fileHandle;
			}
		}

		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"Failed to open a handle to the VPU device. "
			L"(hr = 0x%x, interfacePath = %s)",
			HRESULT_FROM_WIN32(GetLastError()),
			interfacePath.c_str());
	}

	return fileHandle;
}


void CsiRequest(
	_In_ HANDLE hDevice,
	_In_ DWORD dwIoControlCode,
	_In_reads_bytes_opt_(nInBufferSize) LPVOID lpInBuffer,
	_In_ DWORD nInBufferSize,
	_Out_writes_bytes_to_opt_(nOutBufferSize, *lpBytesReturned) LPVOID lpOutBuffer,
	_In_ DWORD nOutBufferSize,
	_Out_opt_ LPDWORD lpBytesReturned,
	_Out_opt_ LPDWORD lpBytesTransferred,
	bool wait = false,
	_Inout_opt_ LPOVERLAPPED lpOverlapped = &CsiOverlappedContext
	)
{
	if (CsiOverlappedContext.hEvent == NULL) {
		throw wexception::make(HRESULT_FROM_WIN32(GetLastError()), L"CsiOverlappedContext.hEvent == NULL");
	}
	if (!DeviceIoControl(hDevice, dwIoControlCode, lpInBuffer, nInBufferSize, lpOutBuffer, nOutBufferSize, lpBytesReturned, lpOverlapped)
			&& (GetLastError() != ERROR_IO_PENDING)) { // || (bytes != sizeof(output))) {

		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"CsiRequest failed. "
			L"(hr = 0x%x, bytes = %d of %d)",
			HRESULT_FROM_WIN32(GetLastError()),
			lpBytesReturned, nOutBufferSize); // sizeof(imageSizeB));
	}
	else
	{
		if (wait) {
			if ((!GetOverlappedResult(hDevice, lpOverlapped, lpBytesTransferred, TRUE)) && (GetLastError() != ERROR_IO_PENDING))
				throw wexception::make(
					HRESULT_FROM_WIN32(GetLastError()),
					L"CsiRequest failed. "
					L"(hr = 0x%x, bytes = %d of %d)",
					HRESULT_FROM_WIN32(GetLastError()),
					lpBytesTransferred, nOutBufferSize); // sizeof(imageSizeB));

			printf("    CsiRequest passed 0x%x\n", lpOutBuffer!=NULL?*(PUINT32)lpOutBuffer:0x0);
		}
		else {
			// waitHandles[waitNum] = CsiOverlappedContext.hEvent;
			// ++waitNum;
			WaitFrame = true;
		}
	}
}
void FrameCompletition(bool wait = false);

void requestFrame(bool Wait = false) {
	PVOID image = dump;
	DWORD NumberOfBytesReturned;
	DWORD NumberOfBytesTransferred;

    FrameInfo_t fInfo;

	fInfo.m_Virtual = (PUCHAR)1;

	CsiRequest(CsiHandle.Get(), IOCTL_CSI_DRIVER_GET_FRAME,
		&fInfo, sizeof(fInfo),
		image, bmi.bmiHeader.biSizeImage, // imageSizeB,
		&NumberOfBytesReturned, &NumberOfBytesTransferred);

	if (Wait) {
		FrameCompletition(true);
	}
}

void FrameCompletition(bool wait)
{
	DWORD NumberOfBytesTransferred;
	PVOID image = dump;

	if (!GetOverlappedResult(CsiHandle.Get(), &CsiOverlappedContext, &NumberOfBytesTransferred, wait ? TRUE : FALSE)) {
		ReportError("FrameCompletition has failed.");
	}
	else {
		printf("    FrameCompletition passed 0x%x\n", *(PUINT32)image);
	}
	WaitForSingleObject(hRequestedMutex, 10000);
	requested = false;
	ReleaseMutex(hRequestedMutex);
}

bool testInitCsi() {
	printf("Tinit_cam\n");
	//const size_t imageSizeB = 1280 * 720 * 2;
	// PVOID image = malloc(imageSizeB);
	PVOID image = dump;
	if (image == NULL) {
		throw wexception::make(
			HRESULT_FROM_WIN32(GetLastError()),
			L"malloc(%d) failed. "
			L"(hr = 0x%x)",
			// imageSizeB,
			bmi.bmiHeader.biSizeImage,
			HRESULT_FROM_WIN32(GetLastError()));
	}
	else {
		DWORD NumberOfBytesReturned;

		CsiHandle = OpenCsiHandle(FILE_FLAG_OVERLAPPED); // , Camera_driver_id_str + 1);
		MipiHandle = OpenMipiHandle(0);
		CamHandle = OpenCameraHandle(0);

		CsiOverlappedEvent = CreateEventW(NULL, TRUE, FALSE, NULL);
		CsiOverlappedContext.hEvent = CsiOverlappedEvent;
		DWORD NumberOfBytesTransferred;
		if (CsiOverlappedEvent == NULL) {
			throw wexception::make(HRESULT_FROM_WIN32(GetLastError()), L"Failed to create event for overlapped IO. " L"(hr = 0x%x)", HRESULT_FROM_WIN32(GetLastError()));
		}
        ULONG output;
        {
            // camera_config_t cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatYUYV , kVIDEO_PixelFormatYUYV, 25, 1, csiLanes, 0x9 /* 0x12 */ };
            // camera_config_t cfg{ kVIDEO_Resolution720P, kVIDEO_PixelFormatRGB888 , kVIDEO_PixelFormatRGB888, 25, 1, csiLanes, 0x9 /* 0x12 */ };
            // camera_config_t cfg{ resolution, cameraPixelFormat , resultPixelFormat, framePerSec, mipiChannel, csiLanes, tHsSettle_EscClk, InterfaceMIPI,  };

            // cfg.tHsSettle_EscClk = (UINT8)GetSettle(cfg.resolution, cfg.framePerSec);
            // ULONG output;
        }
        {
            video_pixel_format_t requiredCameraFmt = kVIDEO_PixelFormatNone;
            try
            {
                CsiRequest(CsiHandle.Get(), IOCTL_CSI_REQUIRED_FMT, (LPVOID)&cam_cfg.resultPixelFormat, sizeof(cam_cfg.resultPixelFormat), &requiredCameraFmt, sizeof(requiredCameraFmt), &NumberOfBytesReturned, &NumberOfBytesTransferred, true);
                cam_cfg.cameraPixelFormat = requiredCameraFmt;
            }
            catch (const std::exception&)
            {
                printf("Failed IOCTL_CSI_REQUIRED_FMT\n");
            }
            
            printf("IOCTL_CSI_REQUIRED_FMT done\n");
        // }
		// {
			DWORD bytes = 0;

			output = 0;
			if (!DeviceIoControl(MipiHandle.Get(), IOCTL_MIPI_REQUIRED_FMT, (PUCHAR)&cam_cfg.cameraPixelFormat, sizeof(cam_cfg.cameraPixelFormat), &requiredCameraFmt, sizeof(requiredCameraFmt), &bytes, nullptr )) {
				// throw wexception::make( HRESULT_FROM_WIN32(GetLastError()), L"IOCTL_CONFIGURE failed. " L"(hr = 0x%x, bytes = %d of %d)", HRESULT_FROM_WIN32(GetLastError()), bytes, sizeof(output));
                printf("Failed IOCTL_CSI_REQUIRED_FMT\n");
			}
			else {
                cam_cfg.cameraPixelFormat = requiredCameraFmt;
			}
            printf("IOCTL_MIPI_REQUIRED_FMT done\n");


            bytes = 0;
            output = 0;
            UINT32 mipiLaneClk = 0;
            if (!DeviceIoControl(CamHandle.Get(), IOCTL_SNS_CONFIGURE, (PUCHAR)&cam_cfg, sizeof(cam_cfg), &mipiLaneClk, sizeof(mipiLaneClk), &bytes, nullptr)) {
                throw wexception::make( HRESULT_FROM_WIN32(GetLastError()), L"IOCTL_SNS_CONFIGURE failed. " L"(hr = 0x%x, bytes = %d of %d)", HRESULT_FROM_WIN32(GetLastError()), bytes, sizeof(output));
            }
            else {
                printf("mipiLaneClk %d\n", mipiLaneClk);
                cam_cfg.mipiLaneClk = mipiLaneClk;
            }
            printf("IOCTL_SNS_CONFIGURE done\n");


            bytes = 0;
            output = 0;
            if (!DeviceIoControl(MipiHandle.Get(), IOCTL_MIPI_CONFIGURE, (LPVOID)&cam_cfg, sizeof(cam_cfg), &output, sizeof(output), &bytes, nullptr)) {
                throw wexception::make(HRESULT_FROM_WIN32(GetLastError()), L"IOCTL_MIPI_CONFIGURE failed. " L"(hr = 0x%x, bytes = %d of %d)", HRESULT_FROM_WIN32(GetLastError()), bytes, sizeof(output));
            }
            else {
                printf("    IOCTL_MIPI_CONFIGURE output = %d\n", output);
            }

            CsiRequest(CsiHandle.Get(), IOCTL_CSI_DRIVER_INIT, (LPVOID)&cam_cfg, sizeof(cam_cfg), &output, sizeof(output), &NumberOfBytesReturned, &NumberOfBytesTransferred, true);

            bytes = 0;
            output = 0;
            if (!DeviceIoControl(CamHandle.Get(), IOCTL_SNS_START, NULL, 0, NULL, 0, &bytes, nullptr)) {
                throw wexception::make(HRESULT_FROM_WIN32(GetLastError()), L"IOCTL_SNS_START failed. " L"(hr = 0x%x, bytes = %d of %d)", HRESULT_FROM_WIN32(GetLastError()), bytes, sizeof(output));
            }
            else {
                printf("    IOCTL_SNS_START output = %d\n", output);
            }
        }
		printf("Test: IOCTL_CSI_DRIVER_FRAME_FILL\n");
		

		printf("    IOCTL_CSI_DRIVER_FRAME_FILL Test Default image: 0x%x\n", *(PUINT32)image);
		for (int i = 0; i < 3; ++i) {
			requestFrame(true);
		}
	}
	return true;
}


// UYVY to RGB888 -----

// unsigned char* rgb_image = new unsigned char[width * height * 3]; //width and height of the image to be converted
void uyvyToRgb(PUINT8 rgb_image, PUINT8 yuyv_image, unsigned width, unsigned height) {
	int y;
	int y1;
	int y2;
	int cr;
	int cb;

	double r;
	double g;
	double b;

	for (unsigned i = 0, j = 0; i < width * height * 3; i += 6, j += 4) {
		//first pixel
		// y = yuyv_image[j];
		// cb = yuyv_image[j + 1];
		// cr = yuyv_image[j + 3];

		y1 = yuyv_image[j + 1];
		cr = yuyv_image[j];
		cb = yuyv_image[j + 2];
		y2 = yuyv_image[j + 3];

		y = max(min(y1, 235), 16);
		cb = max(min(cb, 240), 16);
		cr = max(min(cr, 240), 16);

		r = y + (1.4015 * (cr - 128));
		g = y - (0.3455 * (cb - 128)) - (0.7169 * (cr - 128));
		b = y + (1.7790 * (cb - 128));
		// r = b = g = y;
		//This prevents colour distortions in your rgb image
		if (r < 0) r = 0;
		else if (r > 255) r = 255;
		if (g < 0) g = 0;
		else if (g > 255) g = 255;
		if (b < 0) b = 0;
		else if (b > 255) b = 255;

		rgb_image[i] = (unsigned char)r;
		rgb_image[i + 1] = (unsigned char)g;
		rgb_image[i + 2] = (unsigned char)b;

		//second pixel
		// cb = yuyv_image[j + 1];
		// y = yuyv_image[j + 2];
		// cr = yuyv_image[j + 3];
		// cb = yuyv_image[j];
		// y = yuyv_image[j + 3];
		// cr = yuyv_image[j + 2];
		y = max(min(y2, 235), 16);
		cb = max(min(cb, 240), 16);
		cr = max(min(cr, 240), 16);

		r = y + (1.4065 * (cr - 128));
		g = y - (0.3455 * (cb - 128)) - (0.7169 * (cr - 128));
		b = y + (1.7790 * (cb - 128));

		if (r < 0) r = 0;
		else if (r > 255) r = 255;
		if (g < 0) g = 0;
		else if (g > 255) g = 255;
		if (b < 0) b = 0;
		else if (b > 255) b = 255;
		// r = b = g = y;

		rgb_image[i + 3] = (unsigned char)r;
		rgb_image[i + 4] = (unsigned char)g;
		rgb_image[i + 5] = (unsigned char)b;
	}
}
void uyvyToRgba(PUINT8 rgb_image, PUINT8 yuyv_image, unsigned width, unsigned height) {
	int y;
	int y1;
	int y2;
	int cr;
	int cb;

	double r;
	double g;
	double b;

	for (unsigned i = 0, j = 0; i < width * height * 4; i += 8, j += 4) {
		//first pixel
		// y = yuyv_image[j];
		// cb = yuyv_image[j + 1];
		// cr = yuyv_image[j + 3];

		y1 = yuyv_image[j + 1];
		cr = yuyv_image[j];
		cb = yuyv_image[j + 2];
		y2 = yuyv_image[j + 3];

		y = max(min(y1, 235), 16);
		cb = max(min(cb, 240), 16);
		cr = max(min(cr, 240), 16);

		r = y + (1.4015 * (cr - 128));
		g = y - (0.3455 * (cb - 128)) - (0.7169 * (cr - 128));
		b = y + (1.7790 * (cb - 128));
		// r = b = g = y;
		//This prevents colour distortions in your rgb image
		if (r < 0) r = 0;
		else if (r > 255) r = 255;
		if (g < 0) g = 0;
		else if (g > 255) g = 255;
		if (b < 0) b = 0;
		else if (b > 255) b = 255;

		rgb_image[i + 2] = (unsigned char)r;
		rgb_image[i + 1] = (unsigned char)g;
		rgb_image[i + 0] = (unsigned char)b;

		//second pixel
		// cb = yuyv_image[j + 1];
		// y = yuyv_image[j + 2];
		// cr = yuyv_image[j + 3];
		// cb = yuyv_image[j];
		// y = yuyv_image[j + 3];
		// cr = yuyv_image[j + 2];

		y = max(min(y2, 235), 16);
		cb = max(min(cb, 240), 16);
		cr = max(min(cr, 240), 16);

		r = y + (1.4065 * (cr - 128));
		g = y - (0.3455 * (cb - 128)) - (0.7169 * (cr - 128));
		b = y + (1.7790 * (cb - 128));

		if (r < 0) r = 0;
		else if (r > 255) r = 255;
		if (g < 0) g = 0;
		else if (g > 255) g = 255;
		if (b < 0) b = 0;
		else if (b > 255) b = 255;
		// r = b = g = y;

		rgb_image[i + 6] = (unsigned char)r;
		rgb_image[i + 5] = (unsigned char)g;
		rgb_image[i + 4] = (unsigned char)b;
	}
}

void yuyvToRgba(PUINT8 rgb_image, PUINT8 yuyv_image, unsigned width, unsigned height) {
    int y;
    int y1;
    int y2;
    int cr;
    int cb;

    double r;
    double g;
    double b;

    for (unsigned i = 0, j = 0; i < width * height * 4; i += 8, j += 4) {
        //first pixel
        y1 = yuyv_image[j];
        cr = yuyv_image[j + 1];
        y2 = yuyv_image[j + 2];
        cb = yuyv_image[j + 3];

        // y1 = yuyv_image[j + 1];
        // cr = yuyv_image[j];
        // cb = yuyv_image[j + 2];
        // y2 = yuyv_image[j + 3];

        y = max(min(y1, 235), 16);
        cb = max(min(cb, 240), 16);
        cr = max(min(cr, 240), 16);

        r = y + (1.4015 * (cr - 128));
        g = y - (0.3455 * (cb - 128)) - (0.7169 * (cr - 128));
        b = y + (1.7790 * (cb - 128));
        // r = b = g = y;
        //This prevents colour distortions in your rgb image
        if (r < 0) r = 0;
        else if (r > 255) r = 255;
        if (g < 0) g = 0;
        else if (g > 255) g = 255;
        if (b < 0) b = 0;
        else if (b > 255) b = 255;

        rgb_image[i + 2] = (unsigned char)r;
        rgb_image[i + 1] = (unsigned char)g;
        rgb_image[i + 0] = (unsigned char)b;

        //second pixel
        // cb = yuyv_image[j + 1];
        // y = yuyv_image[j + 2];
        // cr = yuyv_image[j + 3];
        // cb = yuyv_image[j];
        // y = yuyv_image[j + 3];
        // cr = yuyv_image[j + 2];

        y = max(min(y2, 235), 16);
        cb = max(min(cb, 240), 16);
        cr = max(min(cr, 240), 16);

        r = y + (1.4065 * (cr - 128));
        g = y - (0.3455 * (cb - 128)) - (0.7169 * (cr - 128));
        b = y + (1.7790 * (cb - 128));

        if (r < 0) r = 0;
        else if (r > 255) r = 255;
        if (g < 0) g = 0;
        else if (g > 255) g = 255;
        if (b < 0) b = 0;
        else if (b > 255) b = 255;
        // r = b = g = y;

        rgb_image[i + 6] = (unsigned char)r;
        rgb_image[i + 5] = (unsigned char)g;
        rgb_image[i + 4] = (unsigned char)b;
    }
}


void resample(PUCHAR dst, PUCHAR src, int oldw, int oldh, int neww, int newh) {
 // https://stackoverflow.com/questions/9570895/image-downscaling-algorithm
	int l_src;
	int c_src;

	int l_dst;
	int c_dst;
	float tmp;

	for (l_dst = 0; l_dst < newh; l_dst++) {
		for (c_dst = 0; c_dst < neww; c_dst++) {
			tmp = (float)(l_dst) / (float)(newh - 1) * (oldh - 1);
			l_src = (int)floor(tmp);
			if (l_src < 0) {
				l_src = 0;
			}
			else {
				if (l_src >= oldh - 1) {
					l_src = oldh - 2;
				}
			}

			tmp = (float)(c_dst) / (float)(neww - 1) * (oldw - 1);
			c_src = (int)floor(tmp);
			if (c_src < 0) {
				c_src = 0;
			}
			else {
				if (c_src >= oldw - 1) {
					c_src = oldw - 2;
				}
			}

			// a[i][j]
			{
				PUCHAR src1 = (src + 3*((l_src * oldw) + c_src));
				PUCHAR dst1 = (dst + 3*((l_dst * neww) + c_dst));

				*dst1 = *src1;
				*(dst1 + 1) = *(src1 + 1);
				*(dst1 + 2) = *(src1 + 2);

			}
			// p2 = *(a + (l * oldw) + c + 1);
			// p3 = *(a + ((l + 1)* oldw) + c + 1);
			// p4 = *(a + ((l + 1)* oldw) + c);

		}
	}
}

void resample1bRgba(PUCHAR dst, PUCHAR src, int oldw, int oldh, int neww, int newh) {
	// https://stackoverflow.com/questions/9570895/image-downscaling-algorithm
	int l_src;
	int c_src;

	int l_dst;
	int c_dst;
	float tmp;

	for (l_dst = 0; l_dst < newh; l_dst++) {
		for (c_dst = 0; c_dst < neww; c_dst++) {
			tmp = (float)(l_dst) / (float)(newh - 1) * (oldh - 1);
			l_src = (int)floor(tmp);
			if (l_src < 0) {
				l_src = 0;
			}
			else {
				if (l_src >= oldh - 1) {
					l_src = oldh - 2;
				}
			}

			tmp = (float)(c_dst) / (float)(neww - 1) * (oldw - 1);
			c_src = (int)floor(tmp);
			if (c_src < 0) {
				c_src = 0;
			}
			else {
				if (c_src >= oldw - 1) {
					c_src = oldw - 2;
				}
			}

			// a[i][j]
			{
				PUCHAR src1 = (src + 1 * ((l_src * oldw) + c_src));
				PUCHAR dst1 = (dst + 4 * ((l_dst * neww) + c_dst));

				*dst1 = *src1;
				*(dst1 + 1) = *(src1 + 1);
				*(dst1 + 2) = *(src1 + 2);

			}
			// p2 = *(a + (l * oldw) + c + 1);
			// p3 = *(a + ((l + 1)* oldw) + c + 1);
			// p4 = *(a + ((l + 1)* oldw) + c);

		}
	}
}

void resample2bRgba(PUCHAR dst, PUCHAR src, int oldw, int oldh, int neww, int newh) {
    // https://stackoverflow.com/questions/9570895/image-downscaling-algorithm
    int l_src;
    int c_src;

    int l_dst;
    int c_dst;
    float tmp;

    for (l_dst = 0; l_dst < newh; l_dst++) {
        for (c_dst = 0; c_dst < neww; c_dst++) {
            tmp = (float)(l_dst) / (float)(newh - 1) * (oldh - 1);
            l_src = (int)floor(tmp);
            if (l_src < 0) {
                l_src = 0;
            }
            else {
                if (l_src >= oldh - 1) {
                    l_src = oldh - 2;
                }
            }

            tmp = (float)(c_dst) / (float)(neww - 1) * (oldw - 1);
            c_src = (int)floor(tmp);
            if (c_src < 0) {
                c_src = 0;
            }
            else {
                if (c_src >= oldw - 1) {
                    c_src = oldw - 2;
                }
            }

            // a[i][j]
            {
                PUCHAR src1 = (src + 4 * ((l_src * oldw) + c_src));
                PUCHAR dst1 = (dst + 4 * ((l_dst * neww) + c_dst));

                *dst1 = *src1;
                *(dst1 + 1) = *(src1 + 1);
                *(dst1 + 2) = *(src1 + 2);

            }
            // p2 = *(a + (l * oldw) + c + 1);
            // p3 = *(a + ((l + 1)* oldw) + c + 1);
            // p4 = *(a + ((l + 1)* oldw) + c);

        }
    }
}

void resample4bRgba(PUCHAR dst, PUCHAR src, int oldw, int oldh, int neww, int newh) {
    // https://stackoverflow.com/questions/9570895/image-downscaling-algorithm
    int l_src;
    int c_src;

    int l_dst;
    int c_dst;
    float tmp;

    for (l_dst = 0; l_dst < newh; l_dst++) {
        for (c_dst = 0; c_dst < neww; c_dst++) {
            tmp = (float)(l_dst) / (float)(newh - 1) * (oldh - 1);
            l_src = (int)floor(tmp);
            if (l_src < 0) {
                l_src = 0;
            }
            else {
                if (l_src >= oldh - 1) {
                    l_src = oldh - 2;
                }
            }

            tmp = (float)(c_dst) / (float)(neww - 1) * (oldw - 1);
            c_src = (int)floor(tmp);
            if (c_src < 0) {
                c_src = 0;
            }
            else {
                if (c_src >= oldw - 1) {
                    c_src = oldw - 2;
                }
            }

            // a[i][j]
            {
                PUCHAR src1 = (src + 4 * ((l_src * oldw) + c_src));
                PUCHAR dst1 = (dst + 4 * ((l_dst * neww) + c_dst));

                *dst1 = *src1;
                *(dst1 + 1) = *(src1 + 1);
                *(dst1 + 2) = *(src1 + 2);
                *(dst1 + 3) = *(src1 + 3);

            }
            // p2 = *(a + (l * oldw) + c + 1);
            // p3 = *(a + ((l + 1)* oldw) + c + 1);
            // p4 = *(a + ((l + 1)* oldw) + c);

        }
    }
}

class DemoApp
{
public:
	DemoApp();
	~DemoApp();

	// Register the window class and call methods for instantiating drawing resources
	HRESULT Initialize();

	// Process and dispatch messages
	void RunMessageLoop();

private:
	HRESULT InitCameras();

	// Initialize device-independent resources.
	HRESULT CreateDeviceIndependentResources();

	// Initialize device-dependent resources.
	HRESULT CreateDeviceResources();

	// Release device-dependent resource.
	void DiscardDeviceResources();

	// Draw content.
	HRESULT OnRender();

	// Resize the render target.
	void OnResize(
		UINT width,
		UINT height
	);

	// The windows procedure.
	static LRESULT CALLBACK WndProc(
		HWND hWnd,
		UINT message,
		WPARAM wParam,
		LPARAM lParam
	);

private:
	HWND m_hwnd;
	ID2D1Factory* m_pDirect2dFactory;
    IDWriteFactory* m_pWriteFactory;
	ID2D1HwndRenderTarget* m_pRenderTarget;
	ID2D1SolidColorBrush* m_pLightSlateGrayBrush;
	ID2D1SolidColorBrush* m_pCornflowerBlueBrush;

	ID2D1Bitmap* m_pBitmap;
    IDWriteTextFormat *m_WriteTextFormat;
    RECT m_rc;
};

DemoApp::DemoApp() :
	m_hwnd(NULL),
	m_pDirect2dFactory(NULL),
    m_pWriteFactory(NULL),
	m_pRenderTarget(NULL),
	m_pLightSlateGrayBrush(NULL),
	m_pCornflowerBlueBrush(NULL)
{
}


DemoApp::~DemoApp()
{
	SafeRelease(&m_pDirect2dFactory);
	SafeRelease(&m_pWriteFactory);
	SafeRelease(&m_pRenderTarget);
	SafeRelease(&m_pLightSlateGrayBrush);
	SafeRelease(&m_pCornflowerBlueBrush);
}

static wchar_t tittleW[100]; // = "Cam 720p 10000000 fps";
bool updateTitle = false;
unsigned tittleLen = 0;


void DemoApp::RunMessageLoop()
{
	// Some earlier version implemented accelerators - I believe keyboard shorcuts.
	// HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_CAMGUI));

	MSG msg;

	run = 3;
	unsigned i = 0;
	// Main message loop:
	{
		HANDLE waitHandles[4] = { 0 };
		volatile DWORD waitNum = 0; // FIXME if multithreaded use locks
		while (dontExit) {
			waitNum = 0; // Reconfigure wait events each loop
			if (WaitFrame) {
				waitHandles[waitNum] = CsiOverlappedContext.hEvent;
				WaitFrameId = waitNum;
				++waitNum;
				// WaitFrame = true;
			}
			else {
				WaitFrameId = 0xffff;
			}
			DWORD evtType = MsgWaitForMultipleObjects(waitNum, &waitHandles[0], FALSE, INFINITE, QS_ALLEVENTS | QS_ALLPOSTMESSAGE | QS_SENDMESSAGE);

			if (evtType < (WAIT_OBJECT_0 + waitNum)) {
				if (WaitFrame && ((evtType - WAIT_OBJECT_0) == WaitFrameId)) {
					WaitFrame = false;
					FrameCompletition();
					if (i == 0) {
						// Use method as needed.
						// resample() is useless as the API used requires Alpha
						// resampleRgba() Fixes that
						// CopyMemory() is fast but leaves crappy image
						// resampleRgba() is somewhat slower and downsizes image to chosen resolution
						// uyvyToRgba() Takes 400 ms on 720p image. This is price for correct colors on display.

						// CopyMemory(lpvBits, dump, bmi.bmiHeader.biWidth * bmi.bmiHeader.biHeight * 2);
						// uyvyToRgba(lpvBits, dump, bmi.bmiHeader.biWidth, bmi.bmiHeader.biHeight);
						// resample(screen, lpvBits, bmi.bmiHeader.biWidth, bmi.bmiHeader.biHeight, bmi2.bmiHeader.biWidth, bmi2.bmiHeader.biHeight);
						// resampleRgba(screen, lpvBits, bmi.bmiHeader.biWidth, bmi.bmiHeader.biHeight, bmi2.bmiHeader.biWidth, bmi2.bmiHeader.biHeight);
                        if (cam_cfg.resultPixelFormat == kVIDEO_PixelFormatNV12) {
                            resample1bRgba(screen, dump, bmi.bmiHeader.biWidth, bmi.bmiHeader.biHeight, bmi2.bmiHeader.biWidth, bmi2.bmiHeader.biHeight);
                        }
                        else {
                            resample4bRgba(lpvBits, dump, bmi.bmiHeader.biWidth, bmi.bmiHeader.biHeight, bmi2.bmiHeader.biWidth, bmi2.bmiHeader.biHeight);
                            if (cam_cfg.resultPixelFormat == kVIDEO_PixelFormatUYVY) {
                                uyvyToRgba(screen, lpvBits, bmi2.bmiHeader.biWidth, bmi2.bmiHeader.biHeight);
                            }
                            else if (cam_cfg.resultPixelFormat == kVIDEO_PixelFormatYUYV) {
                                yuyvToRgba(screen, lpvBits, bmi2.bmiHeader.biWidth, bmi2.bmiHeader.biHeight);
                            }
                            // uyvyToRgba(lpvBits, dump, bmi2.bmiHeader.biWidth, bmi2.bmiHeader.biHeight);
                        }
                        
                        {
                            

                            StartingTime = EndingTime;
                            QueryPerformanceCounter(&EndingTime);
                            ElapsedMicroseconds.QuadPart = EndingTime.QuadPart - StartingTime.QuadPart;
                            ElapsedMicroseconds.QuadPart *= 1000;
                            ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;
                            cnt.QuadPart += ElapsedMicroseconds.QuadPart;

                            // sumElapsedMs[0] += () ElapsedMicroseconds.QuadPart;
                            ++filter_cnt;

                            if (cnt.QuadPart > 3000) {
                                // ++elapsedSeconds;
                                elapsedSeconds += 3;

                                float fps = (float)filter_cnt / ((float)(cnt.QuadPart/1000));
                                filter_cnt = 0;
                                cnt.QuadPart = 0;
                                // sumElapsedMs[0] = 0;


                                tittleLen = _snwprintf_s(tittleW, 100, _TRUNCATE, L"Cam 720p %5.2f fps", fps);
                                updateTitle = true;
                                // SetWindowTextA(m_hwnd, tittle);
                            }
                        }
                        InvalidateRect(m_hwnd, NULL, TRUE);

						// i = 10;
					}
					else {
						--i;
						requested = true;
						requestFrame();
					}
				}
			}
			else if (evtType == WAIT_IO_COMPLETION) {
				printf("Warning: Unhandled WAIT_IO_COMPLETION.");
			}
			else if (evtType == WAIT_FAILED) {

			}
			else if (evtType == WAIT_OBJECT_0 + waitNum) {
				dontExit = GetMessage(&msg, nullptr, 0, 0);
				{
					// Some earlier version implemented accelerators - I believe keyboard shorcuts.
					// if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
					// {
						TranslateMessage(&msg);
						DispatchMessage(&msg);
					// }
				}
			}
		}
	}
}

HRESULT DemoApp::InitCameras()
{
	HRESULT hr = S_OK;
	try {
		MsgBuff = (char *)malloc(MsgBuffLen);
		// TODO: Place code here.
		initBmi();
		initBmi2();
		lpvBits = (PUCHAR)malloc(bmi.bmiHeader.biSizeImage);
		if (lpvBits == NULL)
			throw wexception::make(HRESULT_FROM_WIN32(GetLastError()),
				L"Failed to malloc lpvBits. ");
		
		dump = (PUCHAR)malloc(bmi.bmiHeader.biSizeImage);
		if (dump == NULL)
			throw wexception::make(HRESULT_FROM_WIN32(GetLastError()),
				L"Failed to malloc dump. ");

// Uncoment if you want to benchmark memcopy, and image conversion methods
// on 8MQ these were actually super slow. The colorspace conversion took 420 ms on 720p image.
#if 0
		{
			LARGE_INTEGER StartingTime, FirstTime, EndingTime, Elapsed1000Microseconds, ElapsedFirstMicroseconds;
			LARGE_INTEGER Frequency;

			QueryPerformanceFrequency(&Frequency);
			QueryPerformanceCounter(&StartingTime);
			CopyMemory(lpvBits, dump, bmi.bmiHeader.biWidth * bmi.bmiHeader.biHeight * 2);
			QueryPerformanceCounter(&FirstTime);
			for (int i = 1000; i > 0; --i) {
				CopyMemory(lpvBits, dump, bmi.bmiHeader.biWidth * bmi.bmiHeader.biHeight * 2);
			}
			// Activity to be timed

			QueryPerformanceCounter(&EndingTime);
			Elapsed1000Microseconds.QuadPart = EndingTime.QuadPart - StartingTime.QuadPart;
			ElapsedFirstMicroseconds.QuadPart = FirstTime.QuadPart - StartingTime.QuadPart;


			//
			// We now have the elapsed number of ticks, along with the
			// number of ticks-per-second. We use these values
			// to convert to the number of elapsed microseconds.
			// To guard against loss-of-precision, we convert
			// to microseconds *before* dividing by ticks-per-second.
			//

			Elapsed1000Microseconds.QuadPart *= 1000000;
			ElapsedFirstMicroseconds.QuadPart *= 1000000;
			Elapsed1000Microseconds.QuadPart /= Frequency.QuadPart;
			ElapsedFirstMicroseconds.QuadPart /= Frequency.QuadPart;

			snprintf(MsgBuff, MsgBuffLen, "1000 CopyMemory Took %lld us ie %lld us per memcopy\n First CopyMemory Took %lld us\n", Elapsed1000Microseconds.QuadPart, Elapsed1000Microseconds.QuadPart / 1000, ElapsedFirstMicroseconds.QuadPart);
			MessageBoxA(NULL, MsgBuff, "Elapsed Time", MB_ICONERROR | MB_OK);
		}
		{
			LARGE_INTEGER StartingTime, FirstTime, EndingTime, Elapsed1000Microseconds, ElapsedFirstMicroseconds;
			LARGE_INTEGER Frequency;

			QueryPerformanceFrequency(&Frequency);
			QueryPerformanceCounter(&StartingTime);
			uyvyToRgba(lpvBits, dump, bmi.bmiHeader.biWidth, bmi.bmiHeader.biHeight);
			QueryPerformanceCounter(&FirstTime);
			for (int i = 100; i > 0; --i) {
				uyvyToRgba(lpvBits, dump, bmi.bmiHeader.biWidth, bmi.bmiHeader.biHeight);
			}
			// Activity to be timed

			QueryPerformanceCounter(&EndingTime);
			Elapsed1000Microseconds.QuadPart = EndingTime.QuadPart - StartingTime.QuadPart;
			ElapsedFirstMicroseconds.QuadPart = FirstTime.QuadPart - StartingTime.QuadPart;


			//
			// We now have the elapsed number of ticks, along with the
			// number of ticks-per-second. We use these values
			// to convert to the number of elapsed microseconds.
			// To guard against loss-of-precision, we convert
			// to microseconds *before* dividing by ticks-per-second.
			//

			Elapsed1000Microseconds.QuadPart *= 1000000;
			ElapsedFirstMicroseconds.QuadPart *= 1000000;
			Elapsed1000Microseconds.QuadPart /= Frequency.QuadPart;
			ElapsedFirstMicroseconds.QuadPart /= Frequency.QuadPart;

			snprintf(MsgBuff, MsgBuffLen, "100 uyvyToRgba Took %lld us ie %lld us per uyvyToRgba\n First uyvyToRgba Took %lld us\n", Elapsed1000Microseconds.QuadPart, Elapsed1000Microseconds.QuadPart / 100, ElapsedFirstMicroseconds.QuadPart);
			MessageBoxA(NULL, MsgBuff, "Elapsed Time", MB_ICONERROR | MB_OK);
		}
#endif

		// screen = (PUCHAR)malloc(screen_wi * screen_he * 3); // Without alpha
		screen = (PUCHAR)malloc(screen_wi * screen_he * 4);
		if (screen == NULL)
			throw wexception::make(HRESULT_FROM_WIN32(GetLastError()),
				L"Failed to malloc screen. ");
		// Initial image and program debugging can be done on with test file containing image.
		// if (loadTestFile())
		// 	throw wexception::make(HRESULT_FROM_WIN32(GetLastError()),
		// 		L"Failed to loadTestFile. ");

		// if (!CameraStart())
		// 	throw wexception::make(HRESULT_FROM_WIN32(GetLastError()),
		// 		L"Failed to CameraStart. ");
		if (!testInitCsi())
			throw wexception::make(HRESULT_FROM_WIN32(GetLastError()),
				L"Failed to testInitCsi. ");
	}
	catch (wexception &err)
	{
		LPWSTR lpBuff;

		FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, NULL, err.HResult(),
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPWSTR)&lpBuff, 0, NULL);

		snprintf(MsgBuff, MsgBuffLen, "Execution failed HR=%08x\n" "%S\n%S\n", err.HResult(), err.wwhat(), lpBuff);
		MessageBoxA(NULL, MsgBuff, "Error", MB_ICONERROR | MB_OK);

		return err.HResult();
	}

	return hr;
}

HRESULT DemoApp::Initialize()
{
	HRESULT hr;
	// Initialize global strings
	// LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	// LoadStringW(hInstance, IDC_CAMGUI, szWindowClass, MAX_LOADSTRING);
	hRequestedMutex = CreateMutex(
		NULL,              // default security attributes
		FALSE,             // initially not owned
		NULL);             // unnamed mutex

	// Initialize device-indpendent resources, such
	// as the Direct2D factory.
	hr = CreateDeviceIndependentResources();

	InitCameras();

	if (SUCCEEDED(hr))
	{
		// Register the window class.
		WNDCLASSEX wcex = { sizeof(WNDCLASSEX) };
		wcex.style = CS_HREDRAW | CS_VREDRAW;
		wcex.lpfnWndProc = DemoApp::WndProc;
		wcex.cbClsExtra = 0;
		wcex.cbWndExtra = sizeof(LONG_PTR);
		wcex.hInstance = HINST_THISCOMPONENT;
		wcex.hbrBackground = NULL;
		wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_CAMGUI);
		wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
		wcex.lpszClassName = L"D2DDemoApp";
		wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

		RegisterClassEx(&wcex);


		// Because the CreateWindow function takes its size in pixels,
		// obtain the system DPI and use it to scale the window size.
		FLOAT dpiX, dpiY;

		// The factory returns the current system DPI. This is also the value it will use
		// to create its own windows.
		// m_pDirect2dFactory->GetDesktopDpi(&dpiX, &dpiY);
          dpiX = 100;
          dpiY = 100;
          // DisplayInformation::LogicalDpi()
		// Create the window.
		m_hwnd = CreateWindow(
			L"D2DDemoApp",
			L"Direct2D Demo App",
			WS_OVERLAPPEDWINDOW,
			CW_USEDEFAULT,
			CW_USEDEFAULT,
			static_cast<UINT>(ceil(640.f * dpiX / 96.f)),
			static_cast<UINT>(ceil(480.f * dpiY / 96.f)),
			NULL,
			NULL,
			HINST_THISCOMPONENT,
			this
		);


		hr = m_hwnd ? S_OK : E_FAIL;
		if (SUCCEEDED(hr))
		{
			ShowWindow(m_hwnd, SW_SHOWNORMAL);
			UpdateWindow(m_hwnd);
		}
	}

	return hr;
}

HRESULT DemoApp::CreateDeviceIndependentResources()
{
	HRESULT hr = S_OK;

	// Create a Direct2D factory.
	hr = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pDirect2dFactory);
    hr = DWriteCreateFactory(
        DWRITE_FACTORY_TYPE_SHARED,
        __uuidof(IDWriteFactory),
        reinterpret_cast<IUnknown**>(&m_pWriteFactory)
    );
	// hr = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pWriteFactory);

	return hr;
}

HRESULT DemoApp::CreateDeviceResources()
{
	HRESULT hr = S_OK;

	if (!m_pRenderTarget)
	{
		
		GetClientRect(m_hwnd, &m_rc);

		D2D1_SIZE_U size = D2D1::SizeU(
            m_rc.right - m_rc.left,
            m_rc.bottom - m_rc.top
		);

		// Create a Direct2D render target.
		hr = m_pDirect2dFactory->CreateHwndRenderTarget(
			D2D1::RenderTargetProperties(),
			D2D1::HwndRenderTargetProperties(m_hwnd, size),
			&m_pRenderTarget
		);

        
        if (SUCCEEDED(hr))
        {
            hr = m_pWriteFactory->CreateTextFormat(
                L"Times New Roman",
                NULL,
                DWRITE_FONT_WEIGHT_NORMAL,
                DWRITE_FONT_STYLE_NORMAL,
                DWRITE_FONT_STRETCH_NORMAL,
                14.0f,
                L"EN-US",
                &m_WriteTextFormat
            );
        }
		if (SUCCEEDED(hr))
		{
			// FIXME replace brushes
			// Create a gray brush.
			hr = m_pRenderTarget->CreateSolidColorBrush(
				D2D1::ColorF(D2D1::ColorF::LightSlateGray),
				&m_pLightSlateGrayBrush
			);
		}
		if (SUCCEEDED(hr))
		{
			// Create a blue brush.
			hr = m_pRenderTarget->CreateSolidColorBrush(
				D2D1::ColorF(D2D1::ColorF::CornflowerBlue),
				&m_pCornflowerBlueBrush
			);
		}
        if (SUCCEEDED(hr))
        {
            hr = m_pRenderTarget->CreateBitmap(
                D2D1_SIZE_U{ screen_wi , screen_he },// size, 
                D2D1_BITMAP_PROPERTIES{ D2D1::PixelFormat(DXGI_FORMAT_R8G8B8A8_UNORM, D2D1_ALPHA_MODE_IGNORE), 300, 300 },
                &m_pBitmap
            );
        }
	}

	return hr;
}

void DemoApp::DiscardDeviceResources()
{
	SafeRelease(&m_pRenderTarget);
	SafeRelease(&m_pLightSlateGrayBrush);
	SafeRelease(&m_pCornflowerBlueBrush);
}

LRESULT CALLBACK DemoApp::WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	LRESULT result = 0;

	if (message == WM_CREATE)
	{
		LPCREATESTRUCT pcs = (LPCREATESTRUCT)lParam;
		DemoApp *pDemoApp = (DemoApp *)pcs->lpCreateParams;

		::SetWindowLongPtrW(
			hwnd,
			GWLP_USERDATA,
			reinterpret_cast<LONG_PTR>(pDemoApp)
		);

		result = 1;
	}
	else
	{
		DemoApp *pDemoApp = reinterpret_cast<DemoApp *>(static_cast<LONG_PTR>(
			::GetWindowLongPtrW(
				hwnd,
				GWLP_USERDATA
			)));

		bool wasHandled = false;

		if (pDemoApp)
		{
			switch (message)
			{
			case WM_COMMAND:
			{
				int wmId = LOWORD(wParam);
				// Parse the menu selections:
				switch (wmId)
				{
				case IDM_ABOUT:
					DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hwnd, About);
					wasHandled = true;
					break;
				case IDM_EXIT:
					DestroyWindow(hwnd);
					PostQuitMessage(0);
					wasHandled = true;
					break;
				default:
					wasHandled = false;
					// return DefWindowProc(hwnd, message, wParam, lParam);
				}
			}
			break;
			case WM_SIZE:
			{
				// FIXME Use this for image resampling instead of hardcoded size
				UINT width = LOWORD(lParam);
				UINT height = HIWORD(lParam);
				pDemoApp->OnResize(width, height);
			}
			result = 0;
			wasHandled = true;
			break;

			case WM_DISPLAYCHANGE:
			{
				InvalidateRect(hwnd, NULL, FALSE);
			}
			result = 0;
			wasHandled = true;
			break;

			case WM_PAINT:
			{
				pDemoApp->OnRender();
               
				ValidateRect(hwnd, NULL);
                auto r = WaitForSingleObject(hRequestedMutex, 10000);
                if (r == WAIT_OBJECT_0) {
                    if (!requested) {
                        // run--;
                        requested = true;
                        requestFrame();
                    }
                    else {
                        // WaitFrame = false;
                    }
                }
                ReleaseMutex(hRequestedMutex);
			}
			result = 0;
			wasHandled = true;
			break;

			case WM_DESTROY:
			{
				PostQuitMessage(0);
			}
			result = 1;
			wasHandled = true;
			break;
			}
		}

		if (!wasHandled)
		{
			result = DefWindowProc(hwnd, message, wParam, lParam);
		}
	}

	return result;
}

HRESULT DemoApp::OnRender()
{
	HRESULT hr = S_OK;

	hr = CreateDeviceResources();
	if (SUCCEEDED(hr))
	{
		m_pRenderTarget->BeginDraw();

		m_pRenderTarget->SetTransform(D2D1::Matrix3x2F::Identity());

		// m_pRenderTarget->Clear(D2D1::ColorF(D2D1::ColorF::White));

		D2D1_SIZE_F rtSize = m_pRenderTarget->GetSize();

		// Paint a grid background.
		// m_pRenderTarget->FillRectangle(
		// 	D2D1::RectF(0.0f, 0.0f, rtSize.width, rtSize.height),
		// 	m_pLightSlateGrayBrush
		// );

		// Create a blue brush.

		if (SUCCEEDED(hr))
		{
			hr = m_pBitmap->CopyFromMemory(NULL, screen, bmi2.bmiHeader.biWidth * 4);
			m_pRenderTarget->DrawBitmap(
				m_pBitmap,
				D2D1::RectF(
					0,
					0,
					screen_wi,
					screen_he)
					// rtSize.width,
					// rtSize.height)
			);
		}

        if (updateTitle) {
            // updateTitle = true;
            D2D1_RECT_F layoutRect = D2D1::RectF((float)m_rc.top, (float)m_rc.left, 200, 20);

            // IDWriteTextFormat* pTextFormat;
            // ::CreateTextFormat(
            //     L"Gabriola",
            //     NULL,
            //     DWRITE_FONT_WEIGHT_REGULAR,
            //     DWRITE_FONT_STYLE_NORMAL,
            //     DWRITE_FONT_STRETCH_NORMAL,
            //     72.0f,
            //     L"en-us",
            //     &pTextFormat
            // );
            m_pRenderTarget->DrawTextW(tittleW, tittleLen, m_WriteTextFormat, layoutRect, m_pLightSlateGrayBrush);
            //  DrawText(hdc, L"Hello World!", -1, &rect, DT_SINGLELINE | DT_NOCLIP);
            // SetWindowTextA(hwnd, tittle);
            // TextOut(hdc, rect.right / 2, rect.bottom / 2, text.c_str(), 1);
        }


#if 0
		// Draw a grid background.
		int width = static_cast<int>(rtSize.width);
		int height = static_cast<int>(rtSize.height);

		for (int x = 0; x < width; x += 10)
		{
			m_pRenderTarget->DrawLine(
				D2D1::Point2F(static_cast<FLOAT>(x), 0.0f),
				D2D1::Point2F(static_cast<FLOAT>(x), rtSize.height),
				m_pLightSlateGrayBrush,
				0.5f
			);
		}

		for (int y = 0; y < height; y += 10)
		{
			m_pRenderTarget->DrawLine(
				D2D1::Point2F(0.0f, static_cast<FLOAT>(y)),
				D2D1::Point2F(rtSize.width, static_cast<FLOAT>(y)),
				m_pLightSlateGrayBrush,
				0.5f
			);
		}

		// Draw two rectangles.
		D2D1_RECT_F rectangle1 = D2D1::RectF(
			rtSize.width / 2 - 50.0f,
			rtSize.height / 2 - 50.0f,
			rtSize.width / 2 + 50.0f,
			rtSize.height / 2 + 50.0f
		);
		
		D2D1_RECT_F rectangle2 = D2D1::RectF(
			rtSize.width / 2 - 100.0f,
			rtSize.height / 2 - 100.0f,
			rtSize.width / 2 + 100.0f,
			rtSize.height / 2 + 100.0f
		);
		// Draw a filled rectangle.
		m_pRenderTarget->FillRectangle(&rectangle1, m_pLightSlateGrayBrush);
		// Draw the outline of a rectangle.
		m_pRenderTarget->DrawRectangle(&rectangle2, m_pCornflowerBlueBrush);
#endif

		hr = m_pRenderTarget->EndDraw();

	}
	if (hr == D2DERR_RECREATE_TARGET)
	{
		hr = S_OK;
		DiscardDeviceResources();
	}

	return hr;
}

void DemoApp::OnResize(UINT width, UINT height)
{
	if (m_pRenderTarget)
	{
		// Note: This method can fail, but it's okay to ignore the
		// error here, because the error will be returned again
		// the next time EndDraw is called.
		m_pRenderTarget->Resize(D2D1::SizeU(width, height));
	}
}

int WINAPI WinMain(
	HINSTANCE /* hInstance */,
	HINSTANCE /* hPrevInstance */,
	LPSTR /* lpCmdLine */,
	int /* nCmdShow */
)
{
	// Use HeapSetInformation to specify that the process should
	// terminate if the heap manager detects an error in any heap used
	// by the process.
	// The return value is ignored, because we want to continue running in the
	// unlikely event that HeapSetInformation fails.
	HeapSetInformation(NULL, HeapEnableTerminationOnCorruption, NULL, 0);

    if (readCfg() > 0) {
        switch (*((UINT32*)cfg)) {
        case 'vyuy': // YUYV
            cam_cfg.resultPixelFormat = kVIDEO_PixelFormatYUYV;
            break;
        case 'yvyu': // UYVY
            cam_cfg.resultPixelFormat = kVIDEO_PixelFormatUYVY;
            break;
        case '21VN': // NV12
            cam_cfg.resultPixelFormat = kVIDEO_PixelFormatNV12;
            break;
        default:
            break;
        }
    }
    QueryPerformanceFrequency(&Frequency);
    QueryPerformanceCounter(&EndingTime);

	if (SUCCEEDED(CoInitialize(NULL)))
	{
		{
			DemoApp app;

			if (SUCCEEDED(app.Initialize()))
			{
				app.RunMessageLoop();
			}
		}
		CoUninitialize();
	}

	return 0;
}


// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}