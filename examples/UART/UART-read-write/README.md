# UART read-write communication sample

### Description
**UART-read-write.exe** is an example command line application that shows how to communicate both ways using UART device interface. Source code file is located in [UART-read-write-source](../UART-read-write/UART-read-write-source) folder.

Application uses WinBase.h library to perform the communication.

## Prerequisites

### Software
* **UART-read-write.exe**
### Hardware
* board with preinstalled Windows and peripherals 
* micro-USB cable or loopback wire

## Before running the app
 1. Check if all the UART instances that you want to use are enabled **"Return(0xF)"** in ACPI table located at _<bsp_dir>/mu_platform_nxp/NXP/BOARD_NAME/AcpiTables/Dsdt-Uart.asl_. \
 If not, enable the instances.
 2. Check if all the UART instances that you want to use do **not have commented** lines containing **"FixedDMA (SDMA_REQ_UARTx_RX, 4, Width8Bit, )"** (DMA mode) in ACPI table located at _<bsp_dir>/mu_platform_nxp/NXP/BOARD_NAME/AcpiTables/Dsdt-Uart.asl_.
 If not, uncomment lines for the instances.
 3. If ACPI tables were changed build new firmware and deploy it to the board according to User's Guide.
 4. **UART1** instance might be allocated for WinDbg, to check whether this instance is avialable check Device Manager. Allocated UART instance will show an exclamation mark and will inform about allocation in Device Status. To free this instance in UART devices,execute ```bcdedit /debug off``` in an **elevated** command line and then restart the board.
 5. Upload the UART-read-write.exe to the board.
 6. Use **UART1** (MQ, MP) or **UART3** instances for loopback communication by manually connecting TXD and RXD pins with wire.
 7. Use **UART2** or **UART4** instances for communication by connecting micro-USB cable to PC, open terminal and connect to COM port with correct parameters (baudrate, parity, databits, stop).

## Run the app
 1. Build the application using configuration for ARM64
 2. Copy **UART-read-write.exe** to the board
 3. Execute the application with this command:
 ```
 UART-read-write.exe [-list] device_path [parity=<P>] [data=<D>] [stop=<S>] [xon={on|off}] [odsr={on|off}] [octs={on|off}] [dtr={on|off|hs}] [rts={on|off|hs|tg}] [idsr={on|off}] [-help]
 ```
 4. Set the desired baudrate. Then write a message to be sent.
 5. If you are using **UART1** or **UART3**, the same message you wrote will be read.
 6. If you are using **UART2** or **UART4**, the message that you wrote will be displayed in the terminal window on your PC. Write something in the terminal window and the UART instance will read it.
 7. Press Ctrl+C to end the communication.