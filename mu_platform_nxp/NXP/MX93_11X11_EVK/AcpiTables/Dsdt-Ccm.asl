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


#define CCM_BASE 0x44450000

#define ROOT_STATUS0_CHANGE_WAIT_TIMES 100000U
#define ROOT_STATUS0_CHANGING_MASK   (0x80000000U)

#define LPCG_REG_OFFSET 0x8000
#define LPCG_REG_STEP 0x80
#define LPCG_STATUS0_ON_MASK 0x1

/* Methods in this file use Local0 .. LocalN to ease the debugging.
 * Use "!amli set verboseon traceon errbrkon spewon" to get debugger print following output.
 *
 *      Store(CRCA(Arg0=0x1b)
 *      {
 *      | Store(0x44450000,Local0)=0x44450000
 *      | Multiply(0x80,Arg0=0x1b,Local1)=0xd80
 *      | Add(Local1=0xd80,Local0=0x44450000,Local2)=0x44450d80
 *      | Return(Local2=0x44450d80)
 *      },Local0)=0x44450d80
 */

/* LPCG Control */

Method (CGDA, 1, Serialized)
/* Calculate Clock gate direct register address */
{
  Local0 = CCM_BASE + LPCG_REG_OFFSET
  Local1 = LPCG_REG_STEP * Arg0
  Local2 = Local1 + Local0
  Return ((Local2))
}

Method (CGST, 1, Serialized)
/* Read clock gate status
 * Arg0 - LpcgIndex
 */
{
  Local0 = CGDA(Arg0) // Calculate address of LPCGa_STATUS0
  OperationRegion (ICB1, SystemMemory, (Local0), 0x24)
  Field (ICB1, DWordAcc, NoLock, Preserve) 
  {
    DAT0,   32,
  }

  Return ((DAT0 & LPCG_STATUS0_ON_MASK)) // 0 - LPCG is OFF, 1 - LPCG is ON.
}

Method (CGON, 1, Serialized)
/* Enable clock gate
 * Arg0 - LpcgIndex
 */
{
  Local0 = CGDA(Arg0)
  OperationRegion (ICB1, SystemMemory, (Local0), 0x04)
  Field (ICB1, DWordAcc, NoLock, Preserve)
  {
    LPCG,   32,
  }
  LPCG = 0x1U // CCM_LPCG_DIRECT_REG(CCM_CTRL_BASE_PTR, LpcgIndex) = 0x1U;
}

Method (CGOF, 1, Serialized)
/* Disable clock gate
 * Arg0 - LpcgIndex
 */
{
  Local0 = CGDA(Arg0)
  OperationRegion (ICB1, SystemMemory, (Local0), 0x04)
  Field (ICB1, DWordAcc, NoLock, Preserve)
  {
    LPCG,   32,
  }
  LPCG = 0x0U // CCM_LPCG_DIRECT_REG(CCM_CTRL_BASE_PTR, LpcgIndex) = 0x0U;
}

/* Clock Root Control */

#define CLOCK_ROOT_REG_OFFSET 0x0
#define CLOCK_ROOT_REG_STEP 0x80

Method (CRCA, 1, Serialized)
/* Get Clock Root Control register address
 * Arg0 - Clock Root index
 */
{
  Local0 = CCM_BASE + CLOCK_ROOT_REG_OFFSET
  Local1 = CLOCK_ROOT_REG_STEP * Arg0
  Local2 = Local1 + Local0
  Return ((Local2))
}

Method (CRCS, 3, Serialized)
/* Set Clock Root Control register
 * Arg0 - CLOCK_ROOT_CONTROL index
 * Arg1 - MUX
 * Arg2 - DIV
 */
{
  Local0 = CRCA(Arg0) // Calculate register address
  OperationRegion (ICB1, SystemMemory, (Local0), 0x40)
  Field (ICB1, DWordAcc, NoLock, Preserve)
  {
    DAT0, 32,
  }
  
  Local0 = Local0 + 0x20 // Calculate CLOCK_ROOTa_STATUS0 address
  OperationRegion (ICB2, SystemMemory, (Local0), 0x40)
  Field (ICB2, DWordAcc, NoLock, Preserve)
  {
    DAT1, 32,
  }
  
  Local2 = ((Arg2 - 1) & 0xFF) | ((Arg1 << 8 ) & 0x0300) // CLOCK_ROOT_CONTROL = ((DIV - 1) & 0xFF) | ((MUX << 8 ) & 0x0300)
  DAT0 = Local2
  
  Local1 = ROOT_STATUS0_CHANGE_WAIT_TIMES
  While ((Local1 > Zero))
  {
    Local1 = (Local1 - One)
    Local2 = DAT1 // CLOCK_ROOT_STATUS0
    Local2 = (DAT1 & ROOT_STATUS0_CHANGING_MASK)
    If ((Local2 == 0x0))
    {
      Break
    }
    Sleep (0x32) // Sleep (MilliSeconds)
  }
}

Method (CRSE, 4, Serialized)
/* Set Clock Root Control register and enable LPCG
 * Arg0 - LpcgIndex
 * Arg1 - ClkRootIndex
 * Arg2 - Clk mux
 * Arg3 - Clk div
 */
{
  CGOF(Arg0) // Disable clock gate
  CRCS(Arg1, Arg2, Arg3) // Set Clock Root Control
  CGON(Arg0) // Enable clock gate
}


