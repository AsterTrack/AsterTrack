/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef UART_H
#define UART_H

#include <string>
#include <cstdint>

void* uart_init(std::string serial);

void uart_deinit(void *port);

bool uart_start(void *port);

void uart_stop(void *port);

int uart_wait(void *port, uint32_t timeoutUS);

int uart_read(void *port, uint8_t *buf, uint32_t len);

int uart_write(void *port, const uint8_t *buf, uint32_t len);

void uart_submit(void *port);

void uart_flush(void *port);

int uart_getTXQueue(void *port);

float uart_getBitsPerUS(void *port);

void uart_configure(void *port, uint32_t rate);

#endif // UART_H