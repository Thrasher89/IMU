/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * Author: Stephan Zuercher stephan.zuercher@stud.hslu.ch
 */


#include <stdio.h>
#include <stdlib.h>



#include <nuttx/arch.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


#include "uart.h"

#ifndef CONFIG_SPI_EXCHANGE
# error This driver requires CONFIG_SPI_EXCHANGE
#endif

namespace device
{

UART::UART(const char *name,
	 const char *devname,
	 const char *serialPortdevname,
	 unsigned int baudrate,
	 int irq) :
	// base class
	CDev(name, devname, irq)
{
	_fd=-1;
	_name=name;
	_devname=devname;
	_serialPortdevname=serialPortdevname;
	_baudrate=baudrate;

	if(CDev::init() != OK){
		debug("gyro init failed");
		exit(-1);
	}

}

UART::~UART(){}

int
UART::init()
{
	int fd;
	struct termios  options;
	struct termios gOriginalTTYAttrs;



	if(_fd!=-1)
		::close(_fd);
	_fd=-1;
	fd = ::open(_serialPortdevname,O_RDONLY | O_NOCTTY);
	if(fd==-1){
		warnx("Faild to open Serial port: %s, errorNo:%d",_serialPortdevname);
		return -1;
	}

	if (tcgetattr(fd, &gOriginalTTYAttrs) < 0){ //get org. data
		warnx("getting tty attributes");    
		return -1;
	}

	options = gOriginalTTYAttrs;

	if (cfsetispeed(&options, _baudrate) < 0 || cfsetospeed(&options, _baudrate) < 0) {
		warnx("set serial speed");   
		return -1;
	}   

	if (tcsetattr(fd, TCSANOW, &options) < 0){
			warnx("update setting tty attributes");
			return -1;
	}
	_fd=fd;
	debug("serial port: %s opened",_serialPortdevname); 

	return OK;
}

int
UART::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	int result = 0;

	if ((send == nullptr) && (recv == nullptr))
		return -EINVAL;

	/*
	LockMode mode = up_interrupt_context() ? LOCK_NONE : locking_mode;

	switch (mode) {
	default:
	case LOCK_PREEMPTION:
		{
			irqstate_t state = irqsave();
			result = _transfer(send, recv, len);
			irqrestore(state);
		}
		break;
	case LOCK_THREADS:
		SPI_LOCK(_dev, true);
		result = _transfer(send, recv, len);
		SPI_LOCK(_dev, false);
		break;
	case LOCK_NONE:
		result = _transfer(send, recv, len);
		break;
	}
	return result;
	*/
	return result;
}

int
UART::put(uint8_t *trans, unsigned len){
	return ::write(_fd,trans,len); 
}

int
UART::get(uint8_t *recv, unsigned len){
	return _read(recv, len);
}
		
int
UART::_read(uint8_t *recv, unsigned len)
{
	ssize_t bytesRead;
	unsigned bytesTotalRead;

	uint8_t* dataPtr = recv;

	bytesTotalRead=0;

	do{ 
		bytesRead=::read(_fd, (dataPtr+bytesTotalRead), len-bytesTotalRead); 
		if(bytesRead==-1){
			printf("Error reading Serial data. ErrorNo: %d\n",errno);
			exit(-1);
		}
		bytesTotalRead+=bytesRead;
	}while(bytesTotalRead<len);
	if(bytesTotalRead!=len){
		printf("Error reading Serial data. Size didnt match");
		exit(-1);

	}

	return (int)bytesRead;

}
void
UART::printHex(unsigned len){
	uint8_t data[len];
	get(data,len);
	printf("\n reading %d bites\n",len);
	for(unsigned i=0;i<len;i++)
		//printf("%02d: %02x\n",i,data[i]);
		printf("%02x ",data[i]);

}
} // namespace device
