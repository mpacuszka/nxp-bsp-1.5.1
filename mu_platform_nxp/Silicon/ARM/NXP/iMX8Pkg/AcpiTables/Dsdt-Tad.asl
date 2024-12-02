/*
* Description: iMX8M Time and Alarm Device (TAD) form i.MX8 MScale-based boards
* (MX8M_EVK, MX8M_MINI_EVK, MX8M_NANO_EVK, MX8M_PLUS_EVK)
*
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
*
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

Device(\_SB.TAD) { //The Time and Alarm Device
    Name (_HID, "ACPI000E")

    Method (_GCP, 0x0, NotSerialized)
    {
        Return(0x4) //Implements Real Time interface, but no alarms
    }

    OperationRegion (SNVS, SystemMemory, 0x30370000, 0x10000)

    // High power SNVS counter MSB/LSB fields
    Field (SNVS, DWordAcc, NoLock, Preserve)
    {
        Offset(0x0024),
        HPCM, 32, // iMX_SNVS_HPRTCMR RTC MSB 0-14
    }
    Field (SNVS, DWordAcc, NoLock, Preserve)
    {
        Offset(0x0028),
        HPCL, 32, // iMX_SNVS_HPRTCLR RTC LSB 0-31
    }

    // Low power SNVS counter MSB/LSB fields
    Field (SNVS, DWordAcc, NoLock, Preserve)
    {
        Offset(0x0050),
        LPCM, 32, // iMX_SNVS_LPSRTCMR RTC MSB 0-14
    }
    Field (SNVS, DWordAcc, NoLock, Preserve)
    {
        Offset(0x0054),
        LPCL, 32, // iMX_SNVS_LPSRTCLR RTC LSB 0-31
    }
    Field (SNVS, DWordAcc, NoLock, Preserve)
    {
        Offset(0x0008),
        HPCR, 32, // iMX_SNVS_HPCR SNVS High Power Control Register
    }
    Field (SNVS, DWordAcc, NoLock, Preserve)
    {
        Offset(0x0038),
        LPCR, 32, // iMX_SNVS_LPCR SNVS Low Power Control Register
    }

    Method(_GRT, 0x0, NotSerialized)
    {
        // Create buffer to hold the Real Time structure BUF
        Name(RTC,Buffer(0x10){}) 
        
        //The leading 'r' in names means 'rtc'
        CreateWordField(RTC,0x0,rY) // Year 1900 - 9999
        CreateByteField(RTC,0x2,rM) // Month 1 - 12
        CreateByteField(RTC,0x3,rD) // Day   1 - 31
        CreateByteField(RTC,0x4,rH) // Hour  0 - 23
        CreateByteField(RTC,0x5,rMin) // Min   0 - 59
        CreateByteField(RTC,0x6,rS) // Sec   0 - 59
        CreateByteField(RTC,0x7,rV) // Time is not valid (request failed); 1 - Time is valid
        CreateWordField(RTC,0x8,rMil) // Milliseconds 1-1000
        CreateWordField(RTC,0xa,rTZ)  // TimeZone -1440 to 1440 or 2047 (unspecified)
        CreateByteField(RTC,0xc,rDl)  // Dl
        CreateByteField(RTC,0xd,rP2)  // Pad2
        
        // Create temporary buffer for the 16 Qword variables (16*8=128=0x80)
        Name(TMPB,Buffer(0x80){}) 
        CreateQwordField(TMPB,0x00,a)
        CreateQwordField(TMPB,0x08,b)
        CreateQwordField(TMPB,0x10,c)
        CreateQwordField(TMPB,0x18,d)
        CreateQwordField(TMPB,0x20,g)
        CreateQwordField(TMPB,0x28,j)
        CreateQwordField(TMPB,0x30,m)
        CreateQwordField(TMPB,0x38,y)
        CreateQwordField(TMPB,0x40,da)
        CreateQwordField(TMPB,0x48,db)
        CreateQwordField(TMPB,0x50,dc)
        CreateQwordField(TMPB,0x58,dg)
        CreateQwordField(TMPB,0x60,hh)
        CreateQwordField(TMPB,0x68,mm)
        CreateQwordField(TMPB,0x70,ss)
        CreateQwordField(TMPB,0x78,K)

        // Recalculate SNVS counters to the output buffer, according to EpochToEfiTime in Edk2. Algorithm from:
        // https://github.com/microsoft/mu_tiano_plus/blob/release/202302/EmbeddedPkg/Library/TimeBaseLib/TimeBaseLib.c#L25
        //double read, three attempts for success
        Local3 = 0
        For (Local3 = 0, Local3 < 3, Local3++) {
            Store(HPCM,Local0)
            Local0 &= 0x7FFF //mask the reserved bits
            Store(HPCM,Local1)
            Local1 &= 0x7FFF //mask the reserved bits

            If (Local0 != Local1) {
                If (Local3 == 2) {
                    rY = 0xFFFF
                    rM = 0xFF
                    rD = 0xFF
                    rH = 0xFF
                    rMin = 0xFF
                    rS = 0xFF
                    rV = 0xFF
                    rMil = 0xFFFF
                    rTZ = 0xFFFF
                    rDl = 0xFF
                    rP2 = 0xFF
                    return(RTC)
                }
            } Else {
                Break
            }
        }
        
        Local0 <<= 32
        Local0 += HPCL + (Local3 * 2) //compensate read/compare time
        Local0 >>= 15 //Local0 = EpochSeconds - divide by 32768

        K  = (Local0 / 86400) + 2440588
        j  = K + 32044
        g  = j / 146097
        dg = j % 146097
        c  = (((dg / 36524) + 1) * 3) / 4
        dc = dg - (c * 36524)
        b  = dc / 1461
        db = dc % 1461
        a  = (((db / 365) + 1) * 3) / 4
        da = db - (a * 365)
        y  = (g * 400) + (c * 100) + (b * 4) + a
        m  = (((da * 5) + 308) / 153) - 2
        d  = da - (((m + 4) * 153) / 5) + 122

        rY = y - 4800 + ((m + 2) / 12)
        rM = ((m + 2) % 12) + 1
        rD = d + 1

        ss = Local0 % 60
        a  = (Local0 - ss) / 60
        mm = a % 60
        b = (a - mm) / 60
        hh = b % 24
      
        rH   = hh
        rMin = mm
        rS   = ss
        rMil = 0
        
        rV  = 1 //Valid
        rTZ = 0 //Timezone
        rDl = 1 //Daylight
        rP2 = 0 //Padding
        
        Return(RTC) // Success -> return what was last in buffer
    }

    Method(_SRT, 0x1, NotSerialized)
    {
        //The leading 'r' in names means 'rtc'
        CreateWordField(Arg0,0x0,rY) // Year 1900 - 9999
        CreateByteField(Arg0,0x2,rM) // Month 1 - 12
        CreateByteField(Arg0,0x3,rD) // Day   1 - 31
        CreateByteField(Arg0,0x4,rH) // Hour  0 - 23
        CreateByteField(Arg0,0x5,rMin) // Min   0 - 59
        CreateByteField(Arg0,0x6,rS) // Sec   0 - 59
        CreateByteField(Arg0,0x7,rV) // Time is not valid (request failed); 1 - Time is valid
        CreateWordField(Arg0,0x8,rMil) // Milliseconds 1-1000
        CreateWordField(Arg0,0xa,rTZ)  // TimeZone -1440 to 1440 or 2047 (unspecified)
        CreateByteField(Arg0,0xc,rDl)  // Dl
        CreateByteField(Arg0,0xd,rP2)  // Pad2

        //debugging variables
        //Local3 = rH
        //Local4 = rMin
        //Local5 = rS

        if ((rY < 1970) || (rY >= 2106)) {
            return(0xFFFFFFFF) //Sanity check failed
        }

        //Algorithm from https://github.com/microsoft/mu_tiano_plus/blob/release/202302/EmbeddedPkg/Library/TimeBaseLib/TimeBaseLib.c#L119
        //EfiTimeToEpoch which uses     
        // Create temporary buffer for the above 5 Qword variables (16*8=128=0x80)
        Name(TMPB,Buffer(0x28){}) 
        CreateQwordField(TMPB,0x00,a)
        CreateQwordField(TMPB,0x08,y)
        CreateQwordField(TMPB,0x10,m)
        CreateQwordField(TMPB,0x18,JulD) // Absolute Julian Date representation of the supplied Time
        CreateQwordField(TMPB,0x20,EpoD) // Number of days elapsed since EPOCH_JULIAN_DAY
        
        a = (14 - rM) / 12
        y = rY + 4800 - a
        m = rM + (12*a) - 3
      
        JulD = rD + ((153*m + 2)/5) + (365*y) + (y/4) - (y/100) + (y/400) - 32045
      
        // Define EPOCH (1970-JANUARY-01) in the Julian Date representation
        //#define EPOCH_JULIAN_DATE                               2440588
        //ASSERT (JulianDate >= EPOCH_JULIAN_DATE);        
        EpoD = JulD - 2440588
      
        //EpochSeconds is Local0
        // Seconds per unit
        //#define SEC_PER_MIN                                     ((UINTN)    60)
        //#define SEC_PER_HOUR                                    ((UINTN)  3600)
        //#define SEC_PER_DAY                                     ((UINTN) 86400)
        Local0 = (EpoD * 86400) + (rH * 3600) + (rMin * 60) + rS
        Local0 <<= 15 //Now this is the value to write to the SNVS HP/LP registers
        
        //Disable the HighPower and LowPower Counters
        HPCR &= 0xFFFFFFFE //Disable the High Power Counter
        LPCR &= 0xFFFFFFFE //Disable the Low Power Counter

        Local6 = 0
        For (Local6 = 0, Local6 < 3, Local6++) { //3 attempts
            if ((LPCR == 0xFFFFFFFE) & (HPCR == 0xFFFFFFFE)) {//check if the counters are disabled
                Break
            } Else { //force it
                HPCR &= 0xFFFFFFFE //Disable the High Power Counter
                LPCR &= 0xFFFFFFFE //Disable the Low Power Counter
            }
        }
                
        Store(LPCM,Local2) //read the LPCM due to reserved bits in the upper part of the register
        Local2 &=0xFFFF8000 //mask the reserved bits in the upper part of the register
        LPCM = (Local0 >> 32) + Local2  //Set the most  significant bits of the LP counter register, add the reserved bits
        LPCL = (Local0 + 2 + Local6 * 2) & 0xFFFFFFFF  //Set the least significant bits of the LP counter register and add compensation for disable/compare and enable

        //synchronize time LP->HP
        HPCR |= 0x00010000 //iMX_SNVS_TS_ENABLE bit

        //Enable the HighPower and LowPower Counters
        LPCR |= 0x00000001 //Enable the Low Power Counter
        HPCR |= 0x00000001 //Enable the High Power Counter

        return(0) //0 - success 0xFFFFFFFF- Failed 
    }

} // End of Time and Alarm Device definition
