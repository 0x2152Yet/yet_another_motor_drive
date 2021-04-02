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
//  This class provides a simple button debouncer.  This should be invoked
//  at a periodic rate with the present button input state.  The button state
//  will be debounced and transformed into one of several logical button states.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "global_definitions.h"

class debouncer
{
public:
    //
    //  A filter coefficient may be optionally provided to the constructor.
    //
    debouncer();
    ~debouncer() {};

    //
    //  This should be called periodically with the latest sampled button
    //  state.
    //
    void updateDebouncer(const bool buttonIsActive);

    //
    //  These indicate if a push-button input is pressed.  This will be true
    //  only on the first call after updateDebouncer determines that the button is
    //  pressed.  Note that a button press is declared when the button is released.
    //  If the button is pushed and held, this will not return true.  In button-hold
    //  cases, calls to pushButtonIsHeld eventually will return true.
    //
    bool buttonIsPressed();

    //
    //  This indicate if a button is held.  Again, this will be true on only
    //  the first call after updateDebouncer determines that the button is
    //  held.
    //
    bool buttonIsHeld();

private:

    enum pushButtonStates
    {
        idle,
        active,
        held,
        waitingForIdle
    };

    pushButtonStates buttonState;
    tickTime_ms      buttonStateStartTime;
    uint32_t         buttonDebounceCount;
    bool             buttonPressed;
    bool             buttonHeld;
};
