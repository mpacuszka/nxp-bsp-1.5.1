# RGB LED handling sample
### Description
RGBLed.exe is a utility to set RGB values of colors for RGB LED from the command line.
Source code file is located in [RGBLed source](../RGBLed/RGBLed-source) folder.

## Prerequisites
### Software:
 * [VC_redist.arm64.exe](../RGBLed/VC_redist.arm64.exe) installed on board
 * [RGBLed.exe](../RGBLed/RGBLed.exe)
### Hardware:
 * I.MX93 board with preinstalled Windows and peripherals

 ## Run the app

 1. Upload the [RGBLed.exe](../RGBLed/RGBLed.exe) to the board
 2. Open cmd in that directory and use ```RGBLed.exe -h``` to display usage of the app 
 3. Run the app with your specification of PWM controllers and pins over which you wish to communicate, set cotroller frequency and RGB values of the colors

For I.MX93, the parameters are:
```
RGBLed.exe R:PWM_4:2 G:PWM_3:0 B:PWM_3:2
```