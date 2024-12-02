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

/*
  Implement EFI RealTimeClock runtime services via RTC Lib.

  The goal is to use both secure (LP) and nonsecure (HP) parts of the RTC to keep data when EVK is turned off but still under power.

  1. After restart, init will be done on SRTC + RTC if SRTC is NOT running, else RTC will have synchronized time from SRTC.
  2. For SetTime, the value will be set to SRTC and synced to RTC.
  3. For GetTime, the value will be taken from RTC.
  
  Code implements basic functionality.
*/


#include <stdint.h>
#include <math.h>
#include <PiDxe.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/RealTimeClockLib.h>

#include <Protocol/RealTimeClock.h>
#include <Library/TimeBaseLib.h>

#include <iMXRealTimeClockLib.h>

/*Set of helpful routines to work with the RTC:
*
* DisableTimer
* EnableTimer
* WriteTimer
*
*/

//Routine to disable S/RTC 
void DisableTimer(UINT64 TrgRegister) {
  UINT32 CR_value = MmioRead32 (iMX_SNVS_Base_Address + TrgRegister);

  if ( (CR_value & iMX_RTC_ENABLED) != 0) {
    MmioWrite32 (iMX_SNVS_Base_Address + TrgRegister, ~iMX_RTC_ENABLED & CR_value);
    //verify again
    if ((MmioRead32 (iMX_SNVS_Base_Address + TrgRegister) & iMX_RTC_ENABLED) != 0) {
      MmioWrite32 (iMX_SNVS_Base_Address + TrgRegister, ~iMX_RTC_ENABLED & CR_value);
    }
  }
  return;
}

//Routine to enable S/RTC 
void EnableTimer(UINT64 TrgRegister) {
  //re-enable the RTC
  UINT32 CR_value = MmioRead32 (iMX_SNVS_Base_Address + TrgRegister) | iMX_RTC_ENABLED;

  MmioWrite32(iMX_SNVS_Base_Address + TrgRegister , CR_value);
  //verify again
  if ( (CR_value & iMX_RTC_ENABLED) != iMX_RTC_ENABLED) {
    MmioWrite32(iMX_SNVS_Base_Address + TrgRegister , CR_value);
  }
  return;
}

//Routine to write seconds to S/RTC
void WriteTimer(TimerType TimTyp, UINT64 Seconds) {
  UINT32 TopSeconds = 0 + (Seconds >> 32);
  UINT32 BottomSeconds = Seconds; 
  
  UINT64 RegTop, RegBottom; 
  if (TimTyp == RTC) {
    RegTop = iMX_SNVS_HPRTCMR;
    RegBottom = iMX_SNVS_HPRTCLR;
  } else {
    RegTop = iMX_SNVS_LPSRTCMR;
    RegBottom = iMX_SNVS_LPSRTCLR;
  }
  //write MSB
  UINT32 msb = MmioRead32 (iMX_SNVS_Base_Address + RegTop);
  MmioWrite32 (iMX_SNVS_Base_Address + RegTop, (msb & iMX_RESET_RTC_MSB) + TopSeconds);
  //write LSB
  MmioWrite32 (iMX_SNVS_Base_Address + RegBottom, BottomSeconds);
  return;
}

/**
  Here, the RTC device will be initialized.
*/
EFI_STATUS iMXInit(IN EFI_HANDLE ImageHandle, IN EFI_SYSTEM_TABLE *SystemTable) {
  
  DEBUG ((EFI_D_INFO, "iMX8RTC Entering iMX Init<<<\n"));
  //read LPCR
  UINT32 LPCR_value = MmioRead32 (iMX_SNVS_Base_Address + iMX_SNVS_LPCR);
  
  //if SRTC is not running, then set bogus stat time to init
  if ((LPCR_value & iMX_RTC_ENABLED) != 0x0000001) {
    DEBUG ((EFI_D_INFO, "iMX8RTC INIT: Setting sample time 1.7.2023 9:00:00 UTC, DST.\n"));
    DebugPrint(0xffffffff, "iMX8RTC Initialized with sample time 1.7.2023 9:00:00 UTC, DST.\n");
    EFI_TIME * MyTime = &(EFI_TIME) {
      .Year = 2023,
      .Month = 7,
      .Day = 1,
      .Hour = 9,
      .Minute = 0,
      .Second = 0,
      .Nanosecond = 0,
      .TimeZone = 0,
      .Daylight = TRUE,
      .Pad1 = 0,
      .Pad2 = 0
    };

    EFI_STATUS Status = LibSetTime(MyTime);
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "iMX8RTC INIT: Setting sample time failed with message: %s\n", Status));
      return Status;
    }
  //SRTC is running, need to sync and restart RTC
  } else {
    //synchronize time LP->HP
    UINT32 CtrlReg = MmioRead32 (iMX_SNVS_Base_Address + iMX_SNVS_HPCR);
    CtrlReg = CtrlReg | iMX_SNVS_TS_ENABLE;
    DEBUG ((EFI_D_INFO, "iMX8RTC INIT: Trying to synchronize - HPCR new state: %08x\n", CtrlReg));
    MmioWrite32 (iMX_SNVS_Base_Address + iMX_SNVS_HPCR, CtrlReg);

    EnableTimer(iMX_SNVS_HPCR);

    EFI_TIME Time;
    EFI_TIME_CAPABILITIES Capabilities;
    if (LibGetTime (&Time, &Capabilities) == EFI_SUCCESS) {
      DebugPrint(0xffffffff, "iMX8RTC Started with saved time: %02d/%02d/%d %02d:%02d:%02d\n",
          Time.Day, Time.Month, Time.Year, Time.Hour, Time.Minute, Time.Second);
    } else {
      DebugPrint(0xffffffff, "iMX8RTC Started with saved time.\n");
    }
  }
  return EFI_SUCCESS;
}

/**
  Returns the current time and date information, and the time-keeping capabilities
  of the hardware platform.

  @param  Time                  A pointer to storage to receive a snapshot of the current time.
  @param  Capabilities          An optional pointer to a buffer to receive the real time clock
                                device's capabilities.

  @retval EFI_SUCCESS           The operation completed successfully.
  @retval EFI_INVALID_PARAMETER Time is NULL.
  @retval EFI_DEVICE_ERROR      The time could not be retrieved due to hardware error.

**/
EFI_STATUS
EFIAPI
LibGetTime (
  OUT EFI_TIME                *Time,
  OUT  EFI_TIME_CAPABILITIES  *Capabilities
  )
{
  DEBUG ((EFI_D_INFO, "iMX8RTC GET TIME\n"));

  UINT64 TopHalf1 = 0;
  UINT64 TopHalf2 = 0;
  UINT32 BottomHalf1 = 0;
  UINT32 BottomHalf2 = 0;

  //read the data twice, compare the results
  BOOLEAN ReadRTC(TimerType TimTyp) {
    UINT64 RegTop, RegBottom; 
    if (TimTyp == RTC) {
      RegTop = iMX_SNVS_HPRTCMR;
      RegBottom = iMX_SNVS_HPRTCLR;
    } else {
      RegTop = iMX_SNVS_LPSRTCMR;
      RegBottom = iMX_SNVS_LPSRTCLR;
    }    
    
    TopHalf1 = MmioRead32 (iMX_SNVS_Base_Address + RegTop) & ~iMX_RESET_RTC_MSB;
    BottomHalf1 = MmioRead32 (iMX_SNVS_Base_Address + RegBottom);

    TopHalf2 = MmioRead32 (iMX_SNVS_Base_Address + RegTop) & ~iMX_RESET_RTC_MSB;
    BottomHalf2 = MmioRead32 (iMX_SNVS_Base_Address + RegBottom);

    if (TopHalf1 != TopHalf2 || BottomHalf1 != BottomHalf2) {
      return FALSE;
    }
    return TRUE;
  }

  //reading data + check, max 2 wrong reads
  //hardcoded RTC type here. Values: "RTC" or "SRTC"
  //change as required
  int counter = 0;
  while (!ReadRTC(RTC)) {
    counter++;
    if (counter >= 3) {return EFI_DEVICE_ERROR;}
  }

  //transfer data to epoch time and return
  UINT64 EpochSeconds = (TopHalf1 << 32) + BottomHalf1;
  //shift - divide the number of tics by 32768 to get seconds
  EpochSeconds = (EpochSeconds+2) >> 15;
  //might need to increment here?  manual says 2 tics to compensate
  EpochToEfiTime (EpochSeconds + 2, Time);
  DEBUG ((EFI_D_INFO, "iMX8RTC GET TIME: Reading TIME converted as %02d:%02d:%02d\n", Time->Hour, Time->Minute, Time->Second));

  return EFI_SUCCESS;
}

/**
  Sets the current local time and date information.

  @param  Time                  A pointer to the current time.

  @retval EFI_SUCCESS           The operation completed successfully.
  @retval EFI_INVALID_PARAMETER A time field is out of range.
  @retval EFI_DEVICE_ERROR      The time could not be set due due to hardware error.

**/
EFI_STATUS
EFIAPI
LibSetTime (
  IN EFI_TIME                *Time
  )
{
  DEBUG ((EFI_D_INFO, "iMX8RTC SET TIME\n"));

  if ((Time->Year < 1970) || (Time->Year >= 2106)) {
    return EFI_UNSUPPORTED;
  }

  UINT64 EpochSeconds = (UINT64)EfiTimeToEpoch (Time);
  EpochSeconds = EpochSeconds << 15;

  //are we disabling SRTC or RTC?
  DisableTimer(iMX_SNVS_LPCR);
  DisableTimer(iMX_SNVS_HPCR);

  TimerType TimTyp = SRTC;

  WriteTimer(TimTyp, EpochSeconds);

  //synchronize time LP->HP
  UINT32 CtrlReg = MmioRead32 (iMX_SNVS_Base_Address + iMX_SNVS_HPCR);
  DEBUG ((EFI_D_INFO, "iMX8RTC SET TIME Trying to synchronize HPCR: %08x<<<\n", CtrlReg));
  CtrlReg = CtrlReg | iMX_SNVS_TS_ENABLE;
  MmioWrite32 (iMX_SNVS_Base_Address + iMX_SNVS_HPCR, CtrlReg);
  
  EnableTimer(iMX_SNVS_LPCR);
  EnableTimer(iMX_SNVS_HPCR);
 
  return EFI_SUCCESS;
}


/**
  Returns the current wakeup alarm clock setting.

  @param  Enabled               Indicates if the alarm is currently enabled or disabled.
  @param  Pending               Indicates if the alarm signal is pending and requires acknowledgement.
  @param  Time                  The current alarm setting.

  @retval EFI_SUCCESS           The alarm settings were returned.
  @retval EFI_INVALID_PARAMETER Any parameter is NULL.
  @retval EFI_DEVICE_ERROR      The wakeup time could not be retrieved due to a hardware error.

**/
EFI_STATUS
EFIAPI
LibGetWakeupTime (
  OUT BOOLEAN     *Enabled,
  OUT BOOLEAN     *Pending,
  OUT EFI_TIME    *Time
  )
{
  // Not a required feature
  return EFI_UNSUPPORTED;
}


/**
  Sets the system wakeup alarm clock time.

  @param  Enabled               Enable or disable the wakeup alarm.
  @param  Time                  If Enable is TRUE, the time to set the wakeup alarm for.

  @retval EFI_SUCCESS           If Enable is TRUE, then the wakeup alarm was enabled. If
                                Enable is FALSE, then the wakeup alarm was disabled.
  @retval EFI_INVALID_PARAMETER A time field is out of range.
  @retval EFI_DEVICE_ERROR      The wakeup time could not be set due to a hardware error.
  @retval EFI_UNSUPPORTED       A wakeup timer is not supported on this platform.

**/
EFI_STATUS
EFIAPI
LibSetWakeupTime (
  IN BOOLEAN      Enabled,
  OUT EFI_TIME    *Time
  )
{
  // Not a required feature
  return EFI_UNSUPPORTED;
}



/**
  This is the declaration of an EFI image entry point. This can be the entry point to an application
  written to this specification, an EFI boot service driver, or an EFI runtime driver.

  @param  ImageHandle           Handle that identifies the loaded image.
  @param  SystemTable           System Table for this image.

  @retval EFI_SUCCESS           The operation completed successfully.

**/
EFI_STATUS
EFIAPI
LibRtcInitialize (
  IN EFI_HANDLE                            ImageHandle,
  IN EFI_SYSTEM_TABLE                      *SystemTable
  )
{
  DEBUG ((EFI_D_INFO, "iMX8RTC Initializing RTC from iMXRealTimeClockLib.c\n"));
  //
  // Do some initialization if required to turn on the RTC
  //
  EFI_STATUS    Status;

  Status = iMXInit(ImageHandle, SystemTable);
  DEBUG ((EFI_D_INFO, "iMX8RTC Init ended with status %s.\n", Status));
  
  return Status;
}


/**
  Fixup internal data so that EFI can be call in virtual mode.
  Call the passed in Child Notify event and convert any pointers in
  lib to virtual mode.

  @param[in]    Event   The Event that is being processed
  @param[in]    Context Event Context
**/
VOID
EFIAPI
LibRtcVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  )
{
  //
  // Only needed if you are going to support the OS calling RTC functions in virtual mode.
  // You will need to call EfiConvertPointer (). To convert any stored physical addresses
  // to virtual address. After the OS transitions to calling in virtual mode, all future
  // runtime calls will be made in virtual mode.
  //
  return;
}

