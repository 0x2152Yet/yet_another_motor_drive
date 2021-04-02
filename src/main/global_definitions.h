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
//  Provides type definitions common to the entire application
//
////////////////////////////////////////////////////////////////////////////////
#pragma once
#include <stdint.h>

//
//  These define floating point types.  For the present processor, single
//  precision float is preferred as the processor can perform basic floating
//  point operations (add, multiply, etc.) in a single processor cycle.  Double
//  precision requires software float point libraries.
//
typedef float float32_t;
typedef double float64_t;

//
//  These type defines define basic engineering units.  These are all defined
//  such that a value of 1.0 refers to 1.0 units.
//
typedef float32_t volts;
typedef float32_t amperes;
typedef float32_t resistance_Ohms;
typedef float32_t inductance_H;

//
//  This is used to define motor back EMF coefficients.
//
typedef float32_t voltsPer1000MechanicalRPM;

//
//  These items define angles and angular rates.
//
typedef float32_t electricalAngle_rad;
typedef float32_t electricalAngle_deg;
typedef float32_t electricalSpeed_RPS;
typedef float32_t electricalSpeed_RPM;
typedef float32_t acceleration_RotationsPerSec2;

//
//  Current controller output commands are expressed as duty cycles.  These
//  express what percentage of the available DC bus should be used to control
//  the motor.  Initially, the duty cycles are signed values that range
//  nominally from -1.0 to 1.0.  In this form a value of zero indicates that
//  none of the bus should be used, a value of -1 indicates that the bus should
//  be driven in its full negative direction, and a value of +1 indicates that the
//  bus should be driven in its full positive direction.
//
typedef float32_t signedDutyCycle;

//
//  The final form of the current controller output command is an unsigned
//  duty cycle percentage that ranges from 0.0 to 1.0.  This commands the actual
//  duty cycle of each PWM signal that is sent to the bridge driver.  A value
//  of zero commands a 0% duty cycle while a value of 1.0 commands a 100%
//  duty cycle.  Note that limits are usually placed on the final commands that
//  disallow 0% or 100% duty cycles.
//
typedef float32_t dutyCycle_pct;

//
//  These are for time values.  These include frequencies, "real" time periods
//  (engineering  unit times), and tick times.  Tick times are integer
//  values that start at zero and increment at the particular time period.
//  If the system runs for a long enough period, the tick timers will roll-
//  over.
//
typedef float32_t frequency_Hz;
typedef float32_t time_s;
typedef float32_t time_ms;
typedef float32_t time_us;

typedef uint32_t tickTime_s;
typedef uint32_t tickTime_ms;
typedef uint32_t tickTime_us;

//
//  These define types of other items.
//
typedef uint32_t bits;
