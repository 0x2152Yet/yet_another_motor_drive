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
//  This class provides a midpoint clamp type zero sequence modulator (ZSM).  
//  The zero sequence modulator accepts duty cycle inputs for each of the 
//  three motor phases (A, B, and C).  The modulator shifts these duty cycles 
//  in unison as the motor commutates so that they make full use of the 
//  available motor DC bus voltage and fall in the input range required by the 
//  physical PWM interface.
//
//  Note that the midpoint clamp type zero sequence modulation produces the same
//  result as conventional space vector pulse width modulation (CSVPWM) often 
//  referred to more simply as space vector modulation (SVM).
//
//  An excellent description of zero sequence modulation, its possible variants 
//  and why one would choose the midpoint clamp type may be found at:
//    https://microchipdeveloper.com/mct5001:start
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "data_logger.h"
#include "global_constants.h"
#include "global_definitions.h"

class zeroSequenceModulator
{
public:

    zeroSequenceModulator();
    ~zeroSequenceModulator() {};

    //
    //  This method performs the zero sequence modulation.  The inputs are
    //  duty cycles that should nominally be in the range of -1.0 to 1.0 but 
    //  may extend slighly beyond to take advantage of some overmodulation. 
    //  The outputs are duty cycles in a form appropriate to be sent to the 
    //  PWM controller hardware.
    //
    void performZeroSequenceModulation(
        const signedDutyCycle inputA,
        const signedDutyCycle inputB,
        const signedDutyCycle inputC,
        dutyCycle_pct &resultDutyCycleA,
        dutyCycle_pct &resultDutyCycleB,
        dutyCycle_pct &resultDutyCycleC);

private:

    //
    //  This scales the duty cycles into the range required for the
    //  zero sequence modulation.
    //
    void scaleDutyCycles (
        const signedDutyCycle inputA,
        const signedDutyCycle inputB,
        const signedDutyCycle inputC,
        signedDutyCycle &resultA,
        signedDutyCycle &resultB,
        signedDutyCycle &resultC);

    //
    //  We make the data logger a friend class so that it can access
    //  private members for logging.
    //
    friend class dataLogger;
};


