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

#include "uart.hpp"
#include "comm/uart.h"

//#include <asm/termios.h> // Clashes with ioctl.h
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <unistd.h>
#include <cstring>

// Set the properties needed for the UART port
static void configSerialPort(int uartFD, uint32_t baud, bool markErrors)
{
	struct termios2 tty = {};

	// NOTE: Change uart_getBytesPerUS when bits get changed
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
	//tty.c_cflag |= PARENB; // Set parity bit, enabling parity checks
	//tty.c_cflag &= ~PARODD; // Use even parity
	//tty.c_cflag &= ~CSTOPB; // Clear stop bit, only one stop bit used in communication
	tty.c_cflag |= CSTOPB; // Add stop bit, two stop bits used in communication
	tty.c_cflag |= CS8; // 8 bits per byte
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON; // Disable canonical mode, where data is read line by line
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|IGNPAR|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	if (markErrors)
		tty.c_iflag |= PARMRK | INPCK; // Enable marking of bytes with framing/parity errors and breaks with 0xFF00 (0xFF is 0xFFFF)

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

	// Set minimum bytes to wait for and timeout. Currently set to completely non-blocking
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be
	tty.c_cflag &= ~CBAUD; // Unset current baud rate
	tty.c_cflag |= BOTHER; // Allow custom baud rate
	tty.c_ispeed = baud; // Set custom input baud
	tty.c_ospeed = baud; // Set custom output baud

	// Submit config
 	ioctl(uartFD, TCSETSF2, &tty);
}

struct UartPort 
{
	int fd;
	std::string port;
	int baudrate;
};

void* uart_init(std::string serial)
{
	UartPort *port = new UartPort();
	port->fd = -1;
	port->port = serial;
	port->baudrate = UART_BAUD_RATE_SAFE;
	return port;
}

void uart_deinit(void **portPtr)
{
	if (portPtr == NULL || *portPtr == NULL)
		return;
	delete (UartPort*)*portPtr;
	*portPtr = NULL;
}

bool uart_start(void *port)
{
	UartPort &uart = *((UartPort*)port);
	uart.fd = open(uart.port.c_str(), O_RDWR | O_NOCTTY);
	if (uart.fd < 0)
	{
		printf("Error: Can't open UART port %s! (%i: %s)\n", uart.port.c_str(), errno, strerror(errno));
		return false;
	}
	configSerialPort(uart.fd, uart.baudrate, true);
	return true;
}

void uart_stop(void *port)
{
	if (port == NULL)
		return;
	UartPort &uart = *((UartPort*)port);
	if (uart.fd != 0)
		close(uart.fd);
}

int uart_wait(void *port, uint32_t timeoutUS)
{
	UartPort &uart = *((UartPort*)port);
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = timeoutUS;

	// Return values for select indicating events in camera file descriptor
	fd_set readFD;
	FD_ZERO(&readFD);
	FD_SET(uart.fd, &readFD);

	// Wait for changes from camera file descriptor
	int status = select(uart.fd + 1, &readFD, nullptr, nullptr, &timeout);
	if (status < 0) return status;
	return FD_ISSET(uart.fd, &readFD);
}

int uart_read(void *port, uint8_t *buf, uint32_t len)
{
	UartPort &uart = *((UartPort*)port);
	return read(uart.fd, buf, len);
}

int uart_write(void *port, const uint8_t *buf, uint32_t len)
{
	UartPort &uart = *((UartPort*)port);
	int wr = write(uart.fd, buf, len);
	return wr;
}

void uart_submit(void *port)
{
	UartPort &uart = *((UartPort*)port);
	// Nothing to do, uart settings are set to automatically send packet
}

void uart_flush(void *port)
{
	UartPort &uart = *((UartPort*)port);
	ioctl(uart.fd, TCSBRK, 1);
}

int uart_getTXQueue(void *port)
{
	int txQueue;
	UartPort &uart = *((UartPort*)port);
	ioctl(uart.fd, TIOCOUTQ, &txQueue);
	return txQueue;
}

void uart_configure(void *port, uint32_t rate)
{
	UartPort &uart = *((UartPort*)port);
	uart.baudrate = rate;
}

float uart_getBytesPerUS(void *port)
{
	UartPort &uart = *((UartPort*)port);
	// 1 Start, 8 Data, 2 Stop, 1 Parity
	int bitsPerByte = 1+8+2;//+1;
	return (float)uart.baudrate / 1000000 / bitsPerByte;
}