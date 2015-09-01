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
 * @Stephan Zuercher stephan.zuercher@stud.hslu.ch
 */

#ifndef _STIM300CTL_H_H
#define _STIM300CTL_H_H
#include <drivers/device/uart.h>

#define SYNC_TIMEOUT_CNT 2000

enum dataGramms {
	PART_NO		= 0xB3, 
	SERIAL_NO	= 0xB7, 
	CONFIG		= 0xBD, 
	NORMAL_MODE_93	= 0x93
};

struct s_measure_result{
	float accelX;
	float accelY;
	float accelZ;
	float gyroX;
	float gyroY;
	float gyroZ;
} typedef meas_result;

class STIM300CTL{
	public:
		STIM300CTL();
		STIM300CTL(device::UART* serial);
		~STIM300CTL();

		void sync2CR_LF(void);
		meas_result getSensorData();
		void streamSensorData2fd(int i);
		static float conv24bit2float(uint8_t msb, uint8_t midb, uint8_t lsb, unsigned int divideBy); //reading 3bytes
	protected:
		device::UART* s;
		int init();
		unsigned read=0;
		unsigned faild=0;
		uint8_t lastCnt=0;

	private:
		unsigned cnt=0;
		int _isInit=false;
		void* getData(dataGramms);
		uint8_t buffer[50];
		int checkData(dataGramms type, uint8_t* data);
		unsigned getStructSize(dataGramms type);
};


struct s_datagram_partNo{
	uint8_t id;
	uint8_t partNo1[3];
	uint8_t delimiter1;
	uint8_t partNo2[3];
	uint8_t delimiter2;
	uint8_t partNo3[2];
	uint8_t noUsed[4];
	uint8_t revision;
	uint8_t crc[4];
	uint8_t cr;
	uint8_t lf;
};

struct s_datagram_serialNo{
	uint8_t id;
	uint8_t data[15];
	uint8_t crc[4];
	uint8_t cr; 
	uint8_t lf; 
};

struct s_datagram_config{
	uint8_t id;
	uint8_t data[21];
	uint8_t crc[4];
	uint8_t cr; 
	uint8_t lf; 
};

struct s_datagram_normalMode_93{
	uint8_t id; 
	uint8_t gyroX[3];
	uint8_t gyroY[3];
	uint8_t gyroZ[3];
	uint8_t gyroStat;

	uint8_t accX[3];
	uint8_t accY[3];
	uint8_t accZ[3];
	uint8_t accStat;

	uint8_t incX[3];
	uint8_t incY[3];
	uint8_t incZ[3];
	uint8_t incStat;

	uint8_t cnt;
	uint16_t lat;
	uint8_t crc[4];
	uint8_t cr; 
	uint8_t lf; 
};



#endif
