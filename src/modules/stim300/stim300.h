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


#ifndef _STIM300_H_
#define _STIM300_H_

#define STREAM_MPU600_no

#ifdef STREAM_MPU600
#include <uORB/uORB.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <fcntl.h>
#endif

//GPS stuff
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_gps_heading.h>

#include "drivers/gps/novatel.h"
#include <uORB/topics/novatel_bestpos.h>


#include <sched.h>
#include "serial.h"

namespace stim300
{

	/**
	 *Standard gravity constant
	 */
	#define STIM300_ONE_G	9.80665f

	/**
	 * STIM States Init => starting up; NORMAL => data streaming; Service => listen for configuration  input
	 */
	enum STIM300_MODE{NOT_CONNECTED=0, INIT, NORMAL, SERVICE};

	enum dataGramms {
		PART_NO		= 0xB3, 
		SERIAL_NO	= 0xB7, 
		CONFIG		= 0xBD, 
		NORMAL_MODE_93	= 0x93
	};

	enum stimState{
		uninitialized = 0,
		startingUp,
		idle,
		streaming,
	};

	class Stim300 : protected Serial
	{
		public:
			/**
			  *Singelton Class
			  */
			static Stim300* Instance();
			static void Start();

			void readThread();
			static int readThreadTrampolin(const char**);
			void getInfo();

			static int32_t get32bit(uint8_t msb, uint8_t midsb, uint8_t lsb);
			static float conv24bit2float(uint8_t msb, uint8_t midb, uint8_t lsb, unsigned int divideBy);
			static char* getState();

		private:
			void startUp();
			stimState state = uninitialized;
			/**
			  *Constructor
			  */
			Stim300();
			/**
			  *Deconstructor
			  */
			~Stim300();
			Stim300(Stim300 const&);
			Stim300 operator=(Stim300 const&);
			uint8_t* getDatagramm(dataGramms); 
			unsigned getDatagrammSize(dataGramms type);

			unsigned datagrammReadCnt=0;
			unsigned datagrammReadErrCnt=0;
			unsigned datagrammCRCErrCnt=0;

			uint8_t dataBuffer[60];


		protected:
			struct p_int24_t {
				signed int:24;
			}__attribute__ ((packed)); // packed: dont allow to make a 32bit pack (padding) on 32bit system. Its 24bit

			struct p_int16_t {
				signed int:16;
			}__attribute__ ((packed));
	
			void putCmd(char *);

			/**
			  * Serialport speed
			  */
			int baudrate; 

			/**
			  * Sensors sampling frequenz
			  */
			int sampling_rate;

			/**
			  * Sensors state
			  */
			STIM300_MODE mode;

			Serial out;

#ifdef STREAM_MPU600 
			void board_accel_init(void);
#endif
	




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
	uint8_t lat[2];
	uint8_t crc[4];
	uint8_t cr; 
	uint8_t lf; 
};




}//end namespace

#endif
