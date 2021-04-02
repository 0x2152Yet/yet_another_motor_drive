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
//  Defines the interfaces to physical processor input controls.  This include
//  items such as push-buttons and a speed control knob.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "debouncer.h"
#include "global_definitions.h"

class PhysicalInputs
{
public:
    PhysicalInputs();
    ~PhysicalInputs() {};

    //
    //  This prepares the various physical input devices for operation.
    //
    void initializePhysicalInputs();

    //
    //  This updates the push-button states.  It should be called at a periodic rate
    //  so that it may debounce the buttons.
    //
    void updatePushButtons();

    //
    //  These indicate if a push-button input is pressed.  This will be true
    //  only on the first call after updatePushButton determines that the button is
    //  pressed.  Note that a button press is declared when the button is released.
    //  If the button is pushed and held, this will not return true.  In button-hold
    //  cases, calls to pushButtonIsHeld eventually will return true.
    //
    bool onboardPushButtonIsPressed()  { return onboardButtonState.buttonIsPressed(); };
    bool externalPushButtonIsPressed() { return externalButtonState.buttonIsPressed(); };

    //
    //  These indicate if a push button is held.  Again, this will be true on only
    //  the first call after updatePushButton determines that the button is
    //  held.
    //
    bool onboardPushButtonIsHeld()  { return onboardButtonState.buttonIsHeld(); };
    bool externalPushButtonIsHeld() { return externalButtonState.buttonIsHeld(); };

    //
    //  This resets the speed command reported counts back to zero.
    //
    void resetSpeedCommand();

    //
    //  This provides the present speed command input, in unit-less signed ticks.
    //
    int32_t getPresentSpeedCommandCounts();

private:

    //
    //  These items are used to interface with on-board and external push-buttons.
    //
    debouncer onboardButtonState;
    debouncer externalButtonState;

    //
    //  Our encoder timer wraps when the counter hits its limits.  We don't
    //  want to propagate that behavior, so we keep a logical counter that
    //  we return to the consumer which we clamp.
    //
    int32_t encoderCommandCounts;
    int32_t lastEncoderCounts;
};

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern PhysicalInputs thePhysicalInputs;
