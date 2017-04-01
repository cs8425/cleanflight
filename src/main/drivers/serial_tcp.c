/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Authors:
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "io/serial.h"
#include "serial_tcp.h"

static uartPort_t tcpSerialPorts[SERIAL_PORT_COUNT];
const struct serialPortVTable uartVTable[]; // Forward

static void tcpReconfigure(uartPort_t *uartPort)
{

}

serialPort_t *uartOpen(USART_TypeDef *USARTx, serialReceiveCallbackPtr rxCallback, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s = NULL;

    if (false) {
#ifdef USE_UART1
    } else if (USARTx == USART1) {
        s = &tcpSerialPorts[0];

#endif
#ifdef USE_UART2
    } else if (USARTx == USART2) {
        s = &tcpSerialPorts[1];
#endif
#ifdef USE_UART3
    } else if (USARTx == USART3) {
        s = &tcpSerialPorts[2];
#endif
#ifdef USE_UART4
    } else if (USARTx == UART4) {
        s = &tcpSerialPorts[3];
#endif
#ifdef USE_UART5
    } else if (USARTx == UART5) {
        s = &tcpSerialPorts[4];
#endif
#ifdef USE_UART6
    } else if (USARTx == USART6) {
        s = &tcpSerialPorts[5];
#endif
#ifdef USE_UART7
    } else if (USARTx == USART7) {
        s = &tcpSerialPorts[6];
#endif
#ifdef USE_UART8
    } else if (USARTx == USART8) {
        s = &tcpSerialPorts[7];
#endif
    } else {
        return (serialPort_t *)s;
    }

    s->port.vTable = uartVTable;
    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    // callback works for IRQ-based RX ONLY
    s->port.rxCallback = rxCallback;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    tcpReconfigure(s);

    return (serialPort_t *)s;
}

uint32_t tcpTotalRxBytesWaiting(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t*)instance;
    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        return s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        return s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
}

uint32_t tcpTotalTxBytesFree(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t*)instance;

    uint32_t bytesUsed;

    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }

    return (s->port.txBufferSize - 1) - bytesUsed;
}

bool isTcpTransmitBufferEmpty(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t *)instance;
    return s->port.txBufferTail == s->port.txBufferHead;
}

uint8_t tcpRead(serialPort_t *instance)
{
    uint8_t ch;
    uartPort_t *s = (uartPort_t *)instance;

    ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }

    return ch;
}

void tcpWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *s = (uartPort_t *)instance;
    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }

	// TODO: trigger TX
}

const struct serialPortVTable uartVTable[] = {
    {
        .serialWrite = tcpWrite,
        .serialTotalRxWaiting = tcpTotalRxBytesWaiting,
        .serialTotalTxFree = tcpTotalTxBytesFree,
        .serialRead = tcpRead,
        .serialSetBaudRate = NULL,
        .isSerialTransmitBufferEmpty = isTcpTransmitBufferEmpty,
        .setMode = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
    }
};
