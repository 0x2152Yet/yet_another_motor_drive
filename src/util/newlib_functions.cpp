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
//  Provides functions required by the compiler's "newlib".
//
//  Most of these are stubs (items for file I/O, etc.)  However support
//  does exist for printf and a very basic malloc,
//
////////////////////////////////////////////////////////////////////////////////

#include <errno.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <stdint.h>

#include "global_definitions.h"
#include "uart_interface.h"

#ifdef __cplusplus
extern "C"
{
#endif

//
//  These are linker defined symbols that provide the location of the heap
//  in the processor's memory space.
//
extern uint32_t _heap_start;
extern uint32_t _heap_end;

//
//  These are the prototypes for the items newlib requires.  These must be
//  defined in order for the build to link.
//

//
//  _sbrk is required by malloc.
//
caddr_t _sbrk(const int32_t Increment);

//
//  _write, _isatty, _close, and _fstat are required by printf.
//
int _write(int file,
           char *ptr,
           int len);
int _isatty(int file);
int _close(int file);
int _fstat(int file,
           struct stat *st);

//
//  The rest are stubs.
//

int _getpid();
int _kill(int pid,
          int sig);
int _read(int file,
          char *ptr,
          int len);
int _lseek(int file,
           int ptr,
           int dir);

////////////////////////////////////////////////////////////////////////////////
//
//  Memory pool manager for malloc
//
//  This function provides memory to malloc from a pool created by the linker.
//  We do not support freeing memory in this application.
//
////////////////////////////////////////////////////////////////////////////////
caddr_t _sbrk(const int32_t Increment)
{
    static caddr_t theHeap   = reinterpret_cast<caddr_t>(&_heap_start);
    static bool outOfHeap    = false;
    caddr_t returnHeap;

    if (outOfHeap)
    {
        returnHeap = NULL;
    }
    else
    {
        //
        //  "theHeap" always points to the next free block;
        //
        returnHeap = theHeap;

        //
        //  We move the free-heap pointer to the next 8 byte boundary past
        //  the desired increment.
        //
        theHeap = reinterpret_cast<caddr_t>(
            (reinterpret_cast<uint32_t>(theHeap + Increment) + 7U) & ~7U);

        //
        //  We check to see if we've run out of heap.
        //
        if (theHeap >= reinterpret_cast<caddr_t>(&_heap_end))
        {
            outOfHeap  = true;
            returnHeap = NULL;
        }
    }

    return reinterpret_cast<caddr_t>(returnHeap);
}

////////////////////////////////////////////////////////////////////////////////
//
//  Outputs characters for printf
//
//  This is called by printf once it has formatted its output string.  We
//  pass the data to the USART driver.
//
////////////////////////////////////////////////////////////////////////////////
int _write(int file, char *ptr, int len)
{
    theUARTInterface.sendToUART(
        reinterpret_cast<uint8_t *const>(ptr),
        static_cast<uint32_t>(len));

    return len;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Stub functions for printf
//
//  These must be present for printf to link, but we do not require their
//  functionality.
//
////////////////////////////////////////////////////////////////////////////////
int _isatty(int file)
{
    return 1;
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Stubs for other functions required by the newlib.
//  All of these that return, return an "error" status.
//
////////////////////////////////////////////////////////////////////////////////
int _lseek(int file, int ptr, int dir)
{
    return -1;
}

void _exit(int status)
{
    //
    //  There is no exit...
    //
    while(true)
    {
    }
}

int _close(int file)
{
    return -1;
}

int _getpid()
{
    return -1;
}

int _kill(int pid, int sig)
{
    return -1;
}

int _read(int file, char *ptr, int len)
{
    return -1;
}

#ifdef __cplusplus
}
#endif /* extern "C" */
