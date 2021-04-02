////////////////////////////////////////////////////////////////////////////////
//
//  Yet Another Motor Drive
//
//  MIT License
//
//  Copyright (c) 2021 Michael F. Kaufman
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
//
//  Defines the interface to the processor's discrete GPIOs
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "hw_definitions.h"
#include "processor.h"

using namespace GPIODefinitions;

class GPIOInterface
{
public:
	GPIOInterface();
	~GPIOInterface() {};

	//
	//  This prepares the GPIO interface for operation.
	//
	void initializeGPIOInterface();

	//
	//  These enable/disable the bridge driver.  If the bridge driver faults
	//  disabling it will reset any fault condition.
	//
	static void disableBridgeOutput ()
	{
        LL_GPIO_ResetOutputPin(bridgeDriverControlPort1, phaseAControlPin);
        LL_GPIO_ResetOutputPin(bridgeDriverControlPort1, phaseBControlPin);
        LL_GPIO_ResetOutputPin(bridgeDriverControlPort2, phaseCControlPin);
	}

    static void enableBridgeOutput ()
    {
        LL_GPIO_SetOutputPin(bridgeDriverControlPort1, phaseAControlPin);
        LL_GPIO_SetOutputPin(bridgeDriverControlPort1, phaseBControlPin);
        LL_GPIO_SetOutputPin(bridgeDriverControlPort2, phaseCControlPin);
    }

	//
	//  These provide the present state of the physical push-buttons
	//
    static bool onBoardButtonIsPressed()
        { return (LL_GPIO_IsInputPinSet(onBoardPushButtonPort, onBoardpushButtonPin) == 1U); }
    static bool externalButtonIsPressed()
        { return (LL_GPIO_IsInputPinSet(externalPushButtonPort, externalPushButtonPin) == 0U); }

	//
	//  These control the LEDs.
	//
	static void turnStatusLEDOff()  { LL_GPIO_ResetOutputPin(statusLEDPort, statusLEDPin ); }
	static void turnStatusLEDOn()   { LL_GPIO_SetOutputPin(statusLEDPort, statusLEDPin ); }
	static void toggleStatusLED()   { LL_GPIO_TogglePin(statusLEDPort, statusLEDPin ); }
    static void turnTimingLEDOff()  { LL_GPIO_ResetOutputPin(timingLEDPort, timingLEDPin ); }
    static void turnTimingLEDOn()   { LL_GPIO_SetOutputPin(timingLEDPort, timingLEDPin ); }
    static void toggleTimingLED()   { LL_GPIO_TogglePin(timingLEDPort, timingLEDPin ); }
	static void turnTriggerLEDOff() { LL_GPIO_ResetOutputPin(triggerLEDPort, triggerLEDPin ); }
	static void turnTriggerLEDOn()  { LL_GPIO_SetOutputPin(triggerLEDPort, triggerLEDPin ); }
	static void toggleTriggerLED()  { LL_GPIO_TogglePin(triggerLEDPort, triggerLEDPin ); }

private:

	//
	//  These initialize the various types of GPIOs.
	//
	void initializeDigitalOutputs();
	void initializeDigitalInputs();
	void initializeAnalogPins();
	void initializeTimerPins();
	void initializeUARTPins();
};

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern GPIOInterface theGPIOInterface;
