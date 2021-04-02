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
//  Provides an interface to the processor's UART
//
//  The UART is used to sent byte data to/from the processor.  At present,
//  this is used to send the processor commands and for it to output status
//  and data.
//
////////////////////////////////////////////////////////////////////////////////
#include "global_definitions.h"
#include "hw_definitions.h"
#include "processor.h"
#include "tick_timers.h"
#include "uart_interface.h"

#include <string.h>

//
//  These are physical constants related to the UART interface.
//
namespace UARTConstants
{
    //
    //  These define the sizes of the input and output data buffers.
    //
const uint32_t bytesInInputBuffer  = 1024U;
const uint32_t bytesInOutputBuffer = 1024U;
}

//
//  These are the buffers to used store incoming and outgoing data.  They
//  are accessed by the DMA controller so they must be in memory it can see.
//
static uint8_t outgoingDataBuffer[UARTConstants::bytesInOutputBuffer]
    __attribute__((aligned(0x10), section(".dma_memory")));
static uint8_t incomingDataBuffer[UARTConstants::bytesInInputBuffer]
    __attribute__((aligned(0x10), section(".dma_memory")));


////////////////////////////////////////////////////////////////////////////////
//
//  Class constructor
//
////////////////////////////////////////////////////////////////////////////////
UARTInterface::UARTInterface()
{
    memset(outgoingDataBuffer, 0U, sizeof(outgoingDataBuffer));
    memset(incomingDataBuffer, 0U, sizeof(incomingDataBuffer));
}

////////////////////////////////////////////////////////////////////////////////
//
//  Initializes the UART interface.
//
////////////////////////////////////////////////////////////////////////////////
void UARTInterface::initializeUARTInterface()
{
    //
    //  We configure the UART for basic asynchronous serial communications.
    //  The part that is tricky is that we use the DMA controller to actually
    //  move the data to/from the UART.  This makes sending and receiving
    //  serial data very "cheap" for the processor (no interrupts, etc.)
    //  Before the general initializations, we make sure the one-bit sample
    //  mode is off (this is the default).  We must do this before the baud
    //  rate is set.
    //
    LL_USART_DisableOneBitSamp(UARTDefinitions::testSupportUART);

    LL_USART_InitTypeDef uartSettings;
    LL_USART_StructInit(&uartSettings);
    uartSettings.BaudRate            = UARTDefinitions::testSupportBaudRate;
    uartSettings.DataWidth           = LL_USART_DATAWIDTH_8B;
    uartSettings.StopBits            = LL_USART_STOPBITS_1;
    uartSettings.Parity              = LL_USART_PARITY_NONE;
    uartSettings.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    uartSettings.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    uartSettings.OverSampling        = LL_USART_OVERSAMPLING_8;

    LL_USART_Init(UARTDefinitions::testSupportUART, &uartSettings);

    //
    //  We do not use the UART's clock feature.
    //
    LL_USART_ClockInitTypeDef uartClockSettings;
    LL_USART_ClockStructInit(&uartClockSettings);
    LL_USART_ClockInit(UARTDefinitions::testSupportUART, &uartClockSettings);

    //
    //  We use the DMA controller to move data between the processor and the
    //  UART.  The DMA controller supports a circular buffer mode, so we use
    //  that for data receipt.  We must configure  DMA stream for both RX and TX
    //
    LL_DMA_InitTypeDef dmaSettings;
    LL_DMA_StructInit(&dmaSettings);
    dmaSettings.PeriphOrM2MSrcAddress  = reinterpret_cast<uint32_t>(&(UARTDefinitions::testSupportUART->DR));
    dmaSettings.MemoryOrM2MDstAddress  = reinterpret_cast<uint32_t>(incomingDataBuffer);
    dmaSettings.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dmaSettings.Mode                   = LL_DMA_MODE_CIRCULAR;
    dmaSettings.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    dmaSettings.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    dmaSettings.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dmaSettings.MemoryOrM2MDstDataSize = LL_DMA_PDATAALIGN_BYTE;
    dmaSettings.NbData                 = UARTConstants::bytesInInputBuffer;
    dmaSettings.Channel                = DMADefinitions::UARTRxDMAChannel;
    dmaSettings.Priority               = LL_DMA_PRIORITY_LOW;
    dmaSettings.FIFOMode               = LL_DMA_FIFOMODE_DISABLE;
    dmaSettings.FIFOThreshold          = LL_DMA_FIFOTHRESHOLD_1_2;
    dmaSettings.MemBurst               = LL_DMA_MBURST_SINGLE;
    dmaSettings.PeriphBurst            = LL_DMA_PBURST_SINGLE;

    LL_DMA_Init(DMADefinitions::UARTDMA, DMADefinitions::UARTRxDMAStream, &dmaSettings);

    dmaSettings.PeriphOrM2MSrcAddress  = reinterpret_cast<uint32_t>(&(UARTDefinitions::testSupportUART->DR));
    dmaSettings.MemoryOrM2MDstAddress  = reinterpret_cast<uint32_t>(outgoingDataBuffer);
    dmaSettings.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dmaSettings.Mode                   = LL_DMA_MODE_NORMAL;
    dmaSettings.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    dmaSettings.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    dmaSettings.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dmaSettings.MemoryOrM2MDstDataSize = LL_DMA_PDATAALIGN_BYTE;
    dmaSettings.NbData                 = 0U;
    dmaSettings.Channel                = DMADefinitions::UARTTxDMAChannel;
    dmaSettings.Priority               = LL_DMA_PRIORITY_LOW;
    dmaSettings.FIFOMode               = LL_DMA_FIFOMODE_DISABLE;
    dmaSettings.FIFOThreshold          = LL_DMA_FIFOTHRESHOLD_1_2;
    dmaSettings.MemBurst               = LL_DMA_MBURST_SINGLE;
    dmaSettings.PeriphBurst            = LL_DMA_PBURST_SINGLE;

    LL_DMA_Init(DMADefinitions::UARTDMA, DMADefinitions::UARTTxDMAStream, &dmaSettings);

    //
    //  We start the RX DMA and the UART.
    //
    LL_DMA_EnableStream(DMADefinitions::UARTDMA, DMADefinitions::UARTRxDMAStream);
    LL_USART_EnableDMAReq_RX(UARTDefinitions::testSupportUART);
    LL_USART_Enable(UARTDefinitions::testSupportUART);
}

////////////////////////////////////////////////////////////////////////////////
//
//  This queues data for output to the UART.  The data is sent via the DMA
//  controller so this will return before the data is actually sent.
//
////////////////////////////////////////////////////////////////////////////////
void UARTInterface::sendToUART(uint8_t *const dataToSend, uint32_t numberBytesToSend)
{
    if ((dataToSend != nullptr) && (numberBytesToSend > 0U))
    {
        //
        //  If there is a previous transfer in process, this will wait for it to
        //  complete.
        //
        waitForLastTransferToComplete();

        //
        //  Now we set up the DMA to send the data.  We start by disabling the
        //  DMA transfer stream and the UART's TX.
        //
        LL_DMA_DisableStream(DMADefinitions::UARTDMA, DMADefinitions::UARTTxDMAStream);
        LL_USART_DisableDMAReq_TX(UARTDefinitions::testSupportUART);

        //
        //  We copy the outgoing data to our local buffer.
        //
        uint32_t adjustedBytesToSend;
        if (numberBytesToSend < UARTConstants::bytesInOutputBuffer)
        {
            adjustedBytesToSend = numberBytesToSend;
        }
        else
        {
            adjustedBytesToSend = UARTConstants::bytesInOutputBuffer;
        }

        memcpy(outgoingDataBuffer, dataToSend, adjustedBytesToSend);

        //
        //  We clear the prior status.
        //
        LL_DMA_ClearFlag_TC3(DMADefinitions::UARTDMA);
        LL_USART_ClearFlag_TC(UARTDefinitions::testSupportUART);

        //
        //  Now we update the DMA settings and start everything off.
        //
        LL_DMA_SetDataLength(
            DMADefinitions::UARTDMA,
            DMADefinitions::UARTTxDMAStream,
            adjustedBytesToSend);

        LL_USART_EnableDMAReq_TX(UARTDefinitions::testSupportUART);
        LL_DMA_EnableStream(DMADefinitions::UARTDMA, DMADefinitions::UARTTxDMAStream);
    }
}

////////////////////////////////////////////////////////////////////////////////
//
//  This waits for any prior transmit to complete.
//
////////////////////////////////////////////////////////////////////////////////
void UARTInterface::waitForLastTransferToComplete()
{
    //
    //  We do not wait forever.
    //
    const tickTime_us maxWaitTime = 1000000U;
    const tickTime_us waitStartTime = theTimers.getHighResTime();
    tickTime_us deltaTime;

    bool isDone;
    do
    {
        isDone = true;

        if (LL_DMA_IsEnabledStream(DMADefinitions::UARTDMA, DMADefinitions::UARTTxDMAStream))
        {
            if (LL_DMA_IsActiveFlag_TC3(DMADefinitions::UARTDMA) == 0U)

            {
                isDone = false;
            }
        }

        //
        //  Even if the DMA is done, we must make sure that the last character
        //  has gone out.
        //
        if (isDone)
        {
            if (LL_USART_IsActiveFlag_TC(UARTDefinitions::testSupportUART) == 0U)
            {
                isDone = false;
            }
        }

        deltaTime = theTimers.getHighResDelta(waitStartTime);
    } while ((!isDone) && (deltaTime < maxWaitTime));

}

