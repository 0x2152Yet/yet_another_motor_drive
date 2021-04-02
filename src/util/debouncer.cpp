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
//  This class implements a simple button debouncing state machine
//
////////////////////////////////////////////////////////////////////////////////

#include "debouncer.h"
#include "global_constants.h"
#include "global_definitions.h"
#include "tick_timers.h"

//
//  These are constants related to button debounce.
//
namespace ButtonConsts
{
    //
    //  We clear "stale" button state if it has not been acted on in a while.
    //
    const tickTime_ms timeToClearButtonState = 500U;

    //
    //  These are the number of push button updates required to consider a button
    //  state change valid.
    //
    const uint32_t countsToDebounceButton = 5U;

    //
    //  This is the physical time required to declare a button held.
    //
    const tickTime_ms timeForButtonHold = 2000U;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
debouncer::debouncer() :
    buttonState(waitingForIdle),
    buttonStateStartTime (0U),
    buttonDebounceCount (0U),
    buttonPressed (false),
    buttonHeld (false)
{
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method updates the state of the push buttons.  A simple state machine
//  is used to debounce the buttons and support the notion of multiple button states
//
////////////////////////////////////////////////////////////////////////////////
void debouncer::updateDebouncer(const bool buttonIsActive)
{
    using namespace ButtonConsts;

    //
    //  The button state machine operates using both update counts and physical
    //  time.
    //
    const tickTime_ms timeSinceStateStart =
        theTimers.getMedResDelta(buttonStateStartTime);

    pushButtonStates newState = buttonState;

    switch (buttonState)
    {
        case idle:
            //
            //  We clear any stale button state if they have not been acted upon
            //  in a while.
            //
            if (timeSinceStateStart > timeToClearButtonState)
            {
                buttonPressed = false;
                buttonHeld = false;
            }

            //
            //  We check to see if the button has been active long enough to
            //  consider it pressed or held.
            //
            if (buttonIsActive)
            {
                buttonDebounceCount++;
                if (buttonDebounceCount > countsToDebounceButton)
                {
                    newState = active;
                }
            }
            break;

        case active:
            if (buttonIsActive)
            {
                //
                //  If the button is held-active, we check to see if enough time
                //  has elapsed to declare it held.
                //
                buttonDebounceCount = 0U;
                if (timeSinceStateStart > timeForButtonHold)
                {
                    newState = held;
                    buttonHeld = true;
                }
            }
            else
            {
                //
                //  When a non-held button is released, we consider it pressed.
                //
                buttonDebounceCount++;
                if (buttonDebounceCount > countsToDebounceButton)
                {
                    newState = idle;
                    buttonPressed = true;
                }
            }
            break;

        case held:
            //
            //  We stay in the held state until the button is released.
            //
            if (!buttonIsActive)
            {
                buttonDebounceCount++;
                if (buttonDebounceCount > countsToDebounceButton)
                {
                    newState = idle;
                }
            }
            else
            {
                buttonDebounceCount = 0U;
            }
            break;

        case waitingForIdle:
            //
            //  This state is only entered at startup to prevent any actions
            //  if a button is stuck active.
            //
            buttonPressed = false;
            buttonHeld    = false;

            if (!buttonIsActive)
            {
                buttonDebounceCount++;
                if (buttonDebounceCount > countsToDebounceButton)
                {
                    newState = idle;
                }
            }
            break;


        default:
            //
            //  Oops...we have a bad state.  We try to get back in sync.
            //
            newState      = waitingForIdle;
            buttonPressed = false;
            buttonHeld    = false;
            buttonDebounceCount = 0U;
            buttonStateStartTime = theTimers.getMedResTime();
            break;
    }

    //
    //  The state machine has transitioned, we reset the items used for the
    //  debounce and event timing.
    //
    if (newState != buttonState)
    {
        buttonDebounceCount = 0U;
        buttonStateStartTime = theTimers.getMedResTime();
        buttonState = newState;
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method indicates if the push-button has been pressed.  It will
//  clear the button-pressed status when it is called
//
////////////////////////////////////////////////////////////////////////////////
bool debouncer::buttonIsPressed()
{
    bool returnValue = buttonPressed;
    buttonPressed = false;
    return returnValue;
}

////////////////////////////////////////////////////////////////////////////////
//
//  This method indicates if the push-button is being held.  It will
//  clear the button-pressed status when it is called
//
////////////////////////////////////////////////////////////////////////////////
bool debouncer::buttonIsHeld()
{
    bool returnValue = buttonHeld;
    buttonHeld = false;
    return returnValue;
}
