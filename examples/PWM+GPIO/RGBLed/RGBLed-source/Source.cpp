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

//
//  RGB LED handling - example app
//
//  Utility to set colors for RGB LED from the command line.
//

#include <ppltasks.h>
#include <collection.h>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <cwctype>
#include <Windows.Devices.h>


class wexception {
public:
    explicit wexception(const std::wstring& msg) : msg_(msg) { }
    virtual ~wexception() { /*empty*/ }

    virtual const wchar_t* wwhat() const {
        return msg_.c_str();
    }

private:
    std::wstring msg_;
};


//
// Find and list all available PWM controllers
//
void ListPwmControllers() {
    using namespace Windows::Devices::Enumeration;
    using namespace Platform::Collections;

    std::wcout << L"Finding PwmControllers\n";

    //
    // Find available PWM controllers
    // 
    // Create empty vector for schematic names of the controllers.
    // Create task to find and get all available controllers, using the schematic name as a selector.
    // If no PWM controllers were found, return.
    //
    Platform::String^ friendlyNameProperty = L"System.Devices.SchematicName";

    auto properties = ref new Vector<Platform::String^>();
    properties->Append(friendlyNameProperty);
    
    auto dis = concurrency::create_task(DeviceInformation::FindAllAsync(Windows::Devices::Pwm::PwmController::GetDeviceSelector(), properties)).get();
    
    if (dis->Size < 1) {
        std::wcout << L"There are no pwm controllers on this system.\n";
        return;
    }

    std::wcout << L"Found " << dis->Size << L" PwmControllers\n";

    std::wcout << L"  SchematicName Id pincount minFrequency maxFrequency\n";

    //
    // Get information about all found controllers
    //
    // Create a cycle to go through each controller.
    // Get id and friendly name of the controller, then create task to get device from the id. Get pin count and frequency range form the device. 
    //
    for (const auto& di : dis) {
        Platform::String^ id = di->Id;
        Platform::String^ friendlyName = L"<null>";

        auto prop = di->Properties->Lookup(friendlyNameProperty);
        if (prop != nullptr) {
            friendlyName = prop->ToString();
        }

        std::wcout << L"  " << friendlyName->Data() << L" " << id->Data() << L" ";

        auto device = concurrency::create_task(Windows::Devices::Pwm::PwmController::FromIdAsync(id)).get();

        std::wcout << device->PinCount << L" " << device->MinFrequency << L" " << device->MaxFrequency << L"\n";
    }
}


//
// Make device of selected PWM controller, using friendly name of the controller, return the device
//
Windows::Devices::Pwm::PwmController^ MakeDevice(_In_opt_ Platform::String^ friendlyName) {
    using namespace Windows::Devices::Enumeration;

    Platform::String^ aqs;
    Platform::String^ id;

    //
    // Find the controller to make device of
    //
    // If the friendly name is specified, get the Advanced Query Syntax (AQS) string for the specified PWM controller. 
    // If not, get the AGS string for all the PWM controllers.
    // Create task to get device information about the controller(s) stored in aqs.
    // If there are more controllers in aqs, work with the first one.
    // Get id of the controller.
    // Throw exception if the id of the controller can't be found.
    //
    if (friendlyName) {
        aqs = Windows::Devices::Pwm::PwmController::GetDeviceSelector(friendlyName);
    }
    else {
        aqs = Windows::Devices::Pwm::PwmController::GetDeviceSelector();
    }

    auto dis = concurrency::create_task(DeviceInformation::FindAllAsync(aqs)).get();
    if (dis->Size > 0) {
        id = dis->GetAt(0)->Id;
    }
    else if (friendlyName->Length() >= 2 && friendlyName->ToString()->Data()[0] == L'\\' && friendlyName->ToString()->Data()[1] == L'\\') {
        id = friendlyName;
    }
    else {
        throw wexception(L"Pwm controller not found.");
    }

    //
    // Create task to get device based on the id and return the device. If an exception occurs, return null pointer.
    //
    try {
        auto device = concurrency::create_task(Windows::Devices::Pwm::PwmController::FromIdAsync(
            id)).get();
        return device;
    }
    catch (Platform::Exception^ ex) {
        return nullptr;
    }
}


void PrintHelp() {
    wprintf(
        L"\n"
        L"HELP:\n"
        L"frequency             set desired PWM controller frequency (Hz),\n"
        L"                      expecting integer in range <minimum frequency; maximum frequency>\n"
        L"\n"
        L"RED/GREEN/BLUE        set RGB values for each color (dutycycle of the LED),\n"    
        L"                      expecting integer in range <0;255>\n"
        L"\n");
}


//
// Return desired frequency, set by user
//
double SetFrequency() {
    double frequency;

    //
    // Create a cycle that stops after correct user input
    // 
    // Read the user input. 
    // Then quit, print help or return desired frequency.
    //
    while (1) {
        std::wcout << L"frequency: ";

        std::wstring line;
        if (!std::getline(std::wcin, line)) {
            return -1;
        }
        std::wistringstream linestream(line);

        std::wstring freq_string;
        linestream >> freq_string;

        if ((freq_string == L"q") || (freq_string == L"quit")) {
            return -1;
        }
        else if ((freq_string == L"h") || (freq_string == L"help")) {
            PrintHelp();
            continue;
        }
        else if (freq_string.empty()) {         // if no frequency is given, set it to 500
            frequency = 500;
            return frequency;
        }

        // convert frequency to double
        try {
            frequency = stod(freq_string);
            return frequency;
        }
        catch (std::exception& ex) {
            std::wcout << L"Expecting integer.\n";
            continue;
        }
    }
}


//
// Open red color pin, set dutycycle and return pin
// Parameters: PWM device and pin id
//
Windows::Devices::Pwm::PwmPin^ SetColorRed(Windows::Devices::Pwm::PwmController^ deviceRed, unsigned int pinIdRed) {
    Windows::Devices::Pwm::PwmPin^ pinRed;
    unsigned int value;

    //
    // Create a cycle that stops after correct user input
    //
    // Read the user input.
    // Then quit, print help or store desired value of the red color.
    //
    while (1) {
        std::wcout << L"RED: ";

        std::wstring line;
        if (!std::getline(std::wcin, line)) {
            return nullptr;
        }
        std::wistringstream linestream(line);

        std::wstring value_string;
        linestream >> value_string;

        // if no value was given, set it to 0
        if (value_string.empty()) {
            value = 0;
            break;
        }
        else if ((value_string == L"q") || (value_string == L"quit")) {
            return nullptr;
        }
        else if ((value_string == L"h") || (value_string == L"help")) {
            PrintHelp();
            continue;
        }
        else {
            // convert value to integer
            try {
                value = stoi(value_string);
            }
            catch (std::exception& ex) {
                std::wcout << L"Expecting integer.\n";
                continue;
            }

            // the number must be RGB value <0;255>
            if ((value < 0) || (value > 255)) {
                std::wcout << L"Expecting value in range <0;255>.\n";
                continue;
            }

            break;
        }
    }

    //
    // Set and return pin
    // 
	// Open pin on the device.
    // Then convert rgb value to dutycycle percentage, set the dutycycle of the pin and return the pin.
    //
	pinRed = deviceRed->OpenPin(pinIdRed);

	double duty = value / 255.0;
	pinRed->SetActiveDutyCyclePercentage(duty);

	return pinRed;
}


//
// Open green color pin, set dutycycle and return pin
// Parameters: pwm device and pin id
//
Windows::Devices::Pwm::PwmPin^ SetColorGreen(Windows::Devices::Pwm::PwmController ^ deviceGreen, unsigned int pinIdGreen) {
    Windows::Devices::Pwm::PwmPin^ pinGreen;
    unsigned int value;

    //
    // Create a cycle that stops after correct user input
    //
    // Read the user input.
    // Then quit, print help or store desired value of the red color.
    //
	while (1) {
		std::wcout << L"GREEN: ";

		std::wstring line;
		if (!std::getline(std::wcin, line)) {
			return nullptr;
		}
		std::wistringstream linestream(line);

		std::wstring value_string;
		linestream >> value_string;

        // if no value was given, set it to 0
        if (value_string.empty()) {
            value = 0;
            break;
        }
        else if ((value_string == L"q") || (value_string == L"quit")) {
            return nullptr;
        }
        else if ((value_string == L"h") || (value_string == L"help")) {
            PrintHelp();
            continue;
        }
        else {
            // convert value to integer
            try {
                value = stoi(value_string);
            }
            catch (std::exception& ex) {
                std::wcout << L"Expecting integer\n";
                continue;
            }

            // the number must be RGB value <0;255>
            if ((value < 0) || (value > 255)) {
                std::wcout << L"Expecting value in range <0;255>.\n";
                continue;
            }

            break;
        }
	}

    //
    // Set and return pin
    // 
    // Open pin on the device.
    // Then convert rgb value to dutycycle percentage, set the dutycycle of the pin and return the pin.
    //
	pinGreen = deviceGreen->OpenPin(pinIdGreen);

	double duty = value / 255.0;
	pinGreen->SetActiveDutyCyclePercentage(duty);

	return pinGreen;
}


//
// Open blue color pin, set dutycycle and return pin
// Parameters: pwm device and pin id
//
Windows::Devices::Pwm::PwmPin^ SetColorBlue(Windows::Devices::Pwm::PwmController^ deviceBlue, unsigned int pinIdBlue) {
    Windows::Devices::Pwm::PwmPin^ pinBlue;
    unsigned int value;

    //
    // Create a cycle that stops after correct user input
    //
    // Read the user input.
    // Then quit, print help or store desired value of the red color.
    //
    while (1) {
        std::wcout << L"BLUE: ";

        std::wstring line;
        if (!std::getline(std::wcin, line)) {
            return nullptr;
        }
        std::wistringstream linestream(line);

        std::wstring value_string;
        linestream >> value_string;

        // if no value was given, set it to 0
        if (value_string.empty()) {
            value = 0;
            break;
        }
        else if ((value_string == L"q") || (value_string == L"quit")) {
            return nullptr;
        }
        else if ((value_string == L"h") || (value_string == L"help")) {
            PrintHelp();
            continue;
        }
        else {
            // convert value to integer
            try {
                value = stoi(value_string);
            }
            catch (std::exception& ex) {
                std::wcout << L"Expecting integer\n";
                continue;
            }

            // the number must be RGB value <0;255>
            if ((value < 0) || (value > 255)) {
                std::wcout << L"Expecting value in range <0;255>.\n";
                continue;
            }

            break;
        }
    }

    //
    // Set and return pin
    // 
    // Open pin on the device.
    // Then convert rgb value to dutycycle percentage, set the dutycycle of the pin and return the pin.
    //
	pinBlue = deviceBlue->OpenPin(pinIdBlue);

	double duty = value / 255.0;
	pinBlue->SetActiveDutyCyclePercentage(duty);

	return pinBlue;
}


void PrintUsage(PCWSTR name) {
    wprintf(
        L"RGBLed: Command line RGB led handling example app\n"
        L"Usage: %s [-list] [R:FriendlyNameR:PinNumberR] [G:FriendlyNameG:PinNumberG] [B:FriendlyNameB:PinNumberB]\n"
        L"\n"
        L"  -list                   List available PWM controllers\n"
        L"                          (friendly name, name, number of pins, minimum frequency and maximum frequency) and exit.\n"
        L"  FriendlyName(R|G|B)     The friendly name of the PWM controller over\n"
        L"                          which you wish to communicate for each color.\n"
        L"  PinNumber(R|G|B)        The number of the GPIO pin which you wish to open for each color.\n"
        L"\n"
        L"  Different order of parameters R:, G:, B: is permitted. At least one of these parameters is required.\n"
        L"\n"
        L"Examples:\n"
        L"  List available PWM controllers and exit:\n"
        L"    %s -list\n"
        L"\n"
        L"  Start setting RBG colors, red color communicates over PWM 4 - pin 2, green color over PWM 3 - pin 0 and blue color over PWM 3 - pin 2:\n"
        L"    %s R:PWM_4:2 G:PWM_3:0 B:PWM_3:2\n",
        name,
        name,
        name);
}


Platform::String^ convertFromString(const std::string& input) {
    std::wstring w_str = std::wstring(input.begin(), input.end());
    const wchar_t* w_chars = w_str.c_str();

    return (ref new Platform::String(w_chars, w_str.length()));
}


int main(Platform::Array<Platform::String^>^ args) {
	PCWSTR arg = args->get(1)->Data();
	if (!_wcsicmp(arg, L"-h") || !_wcsicmp(arg, L"/h") || !_wcsicmp(arg, L"-?") || !_wcsicmp(arg, L"/?")) {
		PrintUsage(args->get(0)->Data());
		return 0;
	}

	if (!_wcsicmp(arg, L"-l") || !_wcsicmp(arg, L"-list")) {
		ListPwmControllers();
		return 0;
	}

    unsigned int optind = 1;
    if (optind >= args->Length) {
        std::wcerr << L"Missing required command line parameter.\n\n";
        PrintUsage(args->get(0)->Data());
        return 1;
    }

    std::string friendlyNameRed_s; 
    std::string friendlyNameGreen_s;
    std::string friendlyNameBlue_s;

    Platform::String^ friendlyNameRed;
    Platform::String^ friendlyNameGreen;
    Platform::String^ friendlyNameBlue;

    std::string pinIdRed_s;
    std::string pinIdGreen_s;
    std::string pinIdBlue_s;

    unsigned int pinIdRed = 0;
    unsigned int pinIdGreen = 0;
    unsigned int pinIdBlue = 0;

    std::string delimiter = ":";

    //
    // Parse command line arguments to get friendly names of PWM controllers and pin numbers
    //
    while (optind < args->Length) {
        Platform::String^ parameter = args->get(optind++);
        const wchar_t *parameter_s = parameter->ToString()->Data();

        if (parameter_s[0] == L'R') {
            std::wstring ws(parameter_s);
            std::string string(ws.begin(), ws.end());
            string.erase(0, string.find(delimiter) + delimiter.length());
            friendlyNameRed_s = string.substr(0, string.find(delimiter));

            if (friendlyNameRed_s == "" || string.find(delimiter) == -1) {
                std::wcerr << L"Missing PWM controller or pin number in command line arguments.\n\n";
                PrintUsage(args->get(0)->Data());
                return 1;
            }

            friendlyNameRed = convertFromString(friendlyNameRed_s);

            string.erase(0, string.find(delimiter) + delimiter.length());
            pinIdRed_s = string.substr(0, string.find(delimiter));

            if (pinIdRed_s == "") {
                std::wcerr << L"Missing pin number in command line arguments.\n\n";
                PrintUsage(args->get(0)->Data());
                return 1;
            }
            
            try {
                pinIdRed = stoi(pinIdRed_s);
            }
            catch (std::exception& ex) {
                std::wcout << L"Pin number must be an integer.\n";
                continue;
            }

            continue;
        }
        else if (parameter_s[0] == L'G') {
            std::wstring ws(parameter_s);
            std::string string(ws.begin(), ws.end());
            string.erase(0, string.find(delimiter) + delimiter.length());
            friendlyNameGreen_s = string.substr(0, string.find(delimiter));

            if (friendlyNameGreen_s == "" || string.find(delimiter) == -1) {
                std::wcerr << L"Missing PWM controller or pin number in command line arguments.\n\n";
                PrintUsage(args->get(0)->Data());
                return 1;
            }

            friendlyNameGreen = convertFromString(friendlyNameGreen_s);

            string.erase(0, string.find(delimiter) + delimiter.length());
            pinIdGreen_s = string.substr(0, string.find(delimiter));

            if (pinIdGreen_s == "") {
                std::wcerr << L"Missing pin number in command line arguments.\n\n";
                PrintUsage(args->get(0)->Data());
                return 1;
            }
            
            try {
                pinIdGreen = stoi(pinIdGreen_s);
            }
            catch (std::exception& ex) {
                std::wcout << L"Pin number must be an integer.\n";
                continue;
            }

            continue;
        }
        else if (parameter_s[0] == L'B') {
            std::wstring ws(parameter_s);
            std::string string(ws.begin(), ws.end());
            string.erase(0, string.find(delimiter) + delimiter.length());
            friendlyNameBlue_s = string.substr(0, string.find(delimiter));

            if (friendlyNameBlue_s == "" || string.find(delimiter) == -1) {
                std::wcerr << L"Missing PWM controller or pin number in command line arguments.\n\n";
                PrintUsage(args->get(0)->Data());
                return 1;
            }

            friendlyNameBlue = convertFromString(friendlyNameBlue_s);

            string.erase(0, string.find(delimiter) + delimiter.length());
            pinIdBlue_s = string.substr(0, string.find(delimiter));

            if (pinIdBlue_s == "") {
                std::wcerr << L"Missing pin number in command line arguments.\n\n";
                PrintUsage(args->get(0)->Data());
                return 1;
            }
            
            try {
                pinIdBlue = stoi(pinIdBlue_s);
            }
            catch (std::exception& ex) {
                std::wcout << L"Pin number must be an integer.\n";
                return 1;
            }

            continue;
        }
        else {
            std::wcerr << L"Invalid command line parameter.\n\n";
            PrintUsage(args->get(0)->Data());
            return 1;
        }
    }

    std::wcout << L"Type 'q'/'quit' to quit, 'h'/'help' to display help. \n\n";

    //
    // Make devices of the selected pwm controllers
    // 
    // If the friendly name of the controller is set, make a device of the controller.
    // If the device for one of the colors is already in use, the function MakeDevice() will return a null pointer. Then more colors will use the same device.
    //
    try {
        Windows::Devices::Pwm::PwmController^ deviceRed;
        Windows::Devices::Pwm::PwmController^ deviceGreen;
        Windows::Devices::Pwm::PwmController^ deviceBlue;

        if (friendlyNameRed) {
            deviceRed = MakeDevice(friendlyNameRed);
        }
        if (friendlyNameGreen) {
            deviceGreen = MakeDevice(friendlyNameGreen);
        }
        if (friendlyNameBlue) {
            deviceBlue = MakeDevice(friendlyNameBlue);
        }


        if (deviceRed == nullptr || deviceGreen == nullptr || deviceBlue == nullptr) {
            if (!(Platform::String::CompareOrdinal(friendlyNameRed, friendlyNameGreen))) {
                deviceGreen = deviceRed;
            }
            else if (!(Platform::String::CompareOrdinal(friendlyNameRed, friendlyNameBlue))) {
                deviceBlue = deviceRed;
            }
            else if (!(Platform::String::CompareOrdinal(friendlyNameGreen, friendlyNameBlue))) {
                deviceBlue = deviceGreen;
            }
        }

        //
        // The main cycle to set frequency and colors
        // 
        // Get frequency for all connected PWM controllers.
        // Then for each color (for each created device):
        //      - set the frequency of the device
        //      - set the color
        //      - start the pin
        // Then quit or continue setting new colors, based on the user input.
        //
        while (std::wcin) {
            double frequency = SetFrequency();
            if (frequency == -1) {
                return 0;
            }
            std::wcout << L"\n";

            Windows::Devices::Pwm::PwmPin^ pinRed;
            Windows::Devices::Pwm::PwmPin^ pinGreen;
            Windows::Devices::Pwm::PwmPin^ pinBlue;

            if (deviceRed) {
                deviceRed->SetDesiredFrequency(frequency);

                pinRed = SetColorRed(deviceRed, pinIdRed);
                if (pinRed == nullptr) {
                    return 0;
                }
                else {
                    pinRed->Start();
                }
            }
            if (deviceGreen) {
                deviceGreen->SetDesiredFrequency(frequency);

                pinGreen = SetColorGreen(deviceGreen, pinIdGreen);
                if (pinGreen == nullptr) {
                    return 0;
                }
                else {
                    pinGreen->Start();
                }
            }
            if (deviceBlue) {
                deviceBlue->SetDesiredFrequency(frequency);

                pinBlue = SetColorBlue(deviceBlue, pinIdBlue);
                if (pinBlue == nullptr) {
                    return 0;
                }
                else {
                    pinBlue->Start();
                }
            }

            std::wcout << L"Do you wish to quit or continue? Type 'q'/'quit' or 'c'/'continue': ";

            std::wstring line;
            if (!std::getline(std::wcin, line)) {
                return 1;
            }
            std::wistringstream linestream(line);

            std::wstring command;
            linestream >> command;
            if ((command == L"q") || (command == L"quit")) {
                return 0;
            }
            else if ((command == L"c") || (command == L"continue")) {
                std::wcout << L"\n";
                if (pinRed) pinRed->Stop();
                if (pinGreen) pinGreen->Stop();
                if (pinBlue) pinBlue->Stop();
                continue;
            }
            else {
                return 0;
            }
        }
    }
    catch (const wexception& ex) {
        std::wcerr << L"Error: " << ex.wwhat() << L"\n";
        return 1;
    }
    catch (Platform::Exception^ ex) {
        std::wcerr << L"Error: " << ex->Message->Data() << L"\n";
        return 1;
    }

    return 0;
}
