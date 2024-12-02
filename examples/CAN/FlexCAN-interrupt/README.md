# FlexCAN interrupt communication sample

### Description
**FlexCAN_interrupt.exe** is an example command line application. It shows how to perform a non-blocking CAN communication. Source code file is located in [FlexCAN-interrupt-source](../FlexCAN-interrupt/FlexCAN-interrupt-source) folder.

Application communicates through IOCTL interface [can.h](../../../imx-windows-iot/driver/can/imxcan_mc/can.h).

## Prerequisites

### Software:
* **FlexCAN_interrupt.exe**
### Hardware:
 * board with preinstalled Windows and peripherals and connected CAN connectors 

## Run the app
 1. Build the application using configuration for ARM64
 2. Copy **FlexCAN_interrupt.exe** to the board
 3. Open cmd window and start **FlexCAN_interrupt.exe**
 4. Follow the displayed text, select first CAN as node B
 5. Open new instance of cmd window and start **FlexCAN_interrupt.exe**
 6. Follow the displayed text, select second CAN (different then the first one) as node A
 7. Set clock source and desired bitrate (same for both nodes)
 8. Select data bytes to transmit and start the transmision by pressing any key in cmd window of node A