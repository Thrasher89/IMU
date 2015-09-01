/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file spi.h
 *
 * Base class for devices connected via UART.
 */


/**
 *
 * Stephan Zuercher for HSLU Horw
 * 10.10.2014 init
 *
 */

#ifndef _DEVICE_UART_H
#define _DEVICE_UART_H

#include "device.h"


namespace device __EXPORT
{

/**
 * Abstract class for character device on SPI
 */
class __EXPORT UART : public CDev
{
protected:
	/**
	 * Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 * @param baudRate	Baud rate
	 * @param irq		Interrupt assigned to the device (or zero if none)
	 */
	UART(const char *name,
	    const char *devname,
	    const char *serialPortdevname,
	    unsigned int baudrate,
	    int irq = 0);
	virtual ~UART();

	virtual int	init();



private:
	/* file descriptor */
	int _fd;
	const char *_name;
	const char *_devname;
	const char *_serialPortdevname;
	unsigned int _baudrate;

	/* this class does not allow copying */
	UART(const UART&);
	UART operator=(const UART&);

protected:
	int	_read(uint8_t *recv, unsigned len);
	int 	transfer(uint8_t *send, uint8_t *recv, unsigned len);
public:
	int	get(uint8_t *recv, unsigned len);
	int	put(uint8_t *trans, unsigned len);
	void printHex(unsigned len);

};

} // namespace device

#endif /* _DEVICE_UART_H */
