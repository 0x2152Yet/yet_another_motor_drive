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
//  Lowest priority application task
//
//  This task performs background activities that do not have any particular
//  timing requirement.  The only thing lower priority is the OS idle task.
//
////////////////////////////////////////////////////////////////////////////////
#pragma once

class BackgroundTask
{
public:
    BackgroundTask();
    ~BackgroundTask() {};

    //
    //  This performs initializations required before the scheduler is
    //  started.
    //
    void initBackgroundTask();

    //
    //  This is a static function intended to be provided to the OS at
    //  initialization.
    //
    static void backgroundTaskEntryPoint(void * const thisPtr);

private:

    //
    //  This is the task's main loop.
    //
    void BackgroundTaskLoop();

    //
    //  This prints build information at startup.
    //
    void printBuildInfo();

};

//
//  We create a single instance of this class that may be referenced
//  globally.
//
extern BackgroundTask theBackgroundTask;
