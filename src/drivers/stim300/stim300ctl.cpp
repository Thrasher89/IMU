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

#include <stdio.h>
#include <stdlib.h> 
#include <systemlib/err.h>
//#include <crc32.h>

#include <string.h> //for strlen
#include <fcntl.h> //temp open()

#include "stim300ctl.h"


STIM300CTL::STIM300CTL(device::UART* serial)
{
	s=serial;
}
STIM300CTL::~STIM300CTL()
{}
STIM300CTL::STIM300CTL()
{}

void
STIM300CTL::sync2CR_LF(void){
	ssize_t numBytes;
        unsigned numBytesRead;

	numBytesRead=0;

	while(numBytesRead<SYNC_TIMEOUT_CNT){
		numBytes = s->get(&buffer[1], 1); 
		if(numBytes==-1){
			warnx("Error reading Serial data"); exit(-1);
		}

		numBytesRead+=numBytes;
		if(numBytes==1){
			if( buffer[0]==0x0d && buffer[1]==0x0a){ //found CR _ LF 
				//printf("found sync after %lu bits\n",numBytesRead);
				return;
			}   
		}   
		else{warnx("Error reading while syncing");exit(-1); }

		buffer[0]=buffer[1];

	}   

	warnx("Error no CR LF sync found"); exit(-1);


}
int
STIM300CTL::init(){
	if(_isInit)
		return OK;
	buffer[0]=-1;

	/*first byte of sensor ist 0x00*/
	s->get(&buffer[0],1);
	if(buffer[0]!=0x00){
		warnx("STIM300 not connected");
		exit(3);
	}
	getData(PART_NO);
	getData(SERIAL_NO);
	getData(CONFIG);
	getSensorData();
	getSensorData();
	printf("\nIniti 300 ctl OK\n");
	return OK;
}

unsigned
STIM300CTL::getStructSize(dataGramms type)
{
	switch(type)
		case PART_NO:
			return sizeof(struct s_datagram_partNo);
		case SERIAL_NO:
			return sizeof(struct s_datagram_serialNo);
		case CONFIG:
			return sizeof(struct s_datagram_config);
		case NORMAL_MODE_93:
			return sizeof(struct s_datagram_normalMode_93);
		default:
			warnx("STIM300CTL::getStructSize type not found");
			exit(4);
	}
}


int 
STIM300CTL::checkData(dataGramms type, uint8_t* data)
{

	cnt++;
	if(cnt>=10000){
		printf("read:%d, error:%d\n",read,faild);
		cnt=0;
	}

	if(data[0]!=(uint8_t)type){
		faild++;
		//warnx("datagram ID wrong\n");
		sync2CR_LF();
		//exit(5);
	}


	if(data[getStructSize(type)-2]!=0x0d || data[getStructSize(type)-1]!=0x0a){
		//warnx("datagram LF CR wrong\n");
		//exit(5);
		faild++;
		sync2CR_LF();

	}
	else{
		//data pack ok
		read++;
		//return OK;
	}
	//printf("Read datapack%x OK\n",type);
}
void*
STIM300CTL::getData(dataGramms type){
	s->get(buffer,getStructSize(type));
	checkData(type,buffer);
}
meas_result
STIM300CTL::getSensorData(){
	meas_result ret;
	getData(NORMAL_MODE_93);
	//TODO buffer used in many function (not privat)
	struct s_datagram_normalMode_93* res = (struct s_datagram_normalMode_93 *)buffer;
	ret.accelX=conv24bit2float(res->accX[0],res->accX[1],res->accX[2],524288);
	ret.accelY=conv24bit2float(res->accY[0],res->accY[1],res->accY[2],524288);
	ret.accelZ=conv24bit2float(res->accZ[0],res->accZ[1],res->accZ[2],524288);

	ret.gyroX=conv24bit2float(res->gyroX[0],res->gyroX[1],res->gyroX[2],2097152);
	ret.gyroY=conv24bit2float(res->gyroY[0],res->gyroY[1],res->gyroY[2],2097152);
	ret.gyroZ=conv24bit2float(res->gyroZ[0],res->gyroZ[1],res->gyroZ[2],2097152);

	/*
	if(res->cnt!=++lastCnt)
		printf("cnt error\n");
	*/

	return ret;
}

float STIM300CTL::conv24bit2float(uint8_t msb, uint8_t midb, uint8_t lsb, unsigned int divideBy){
	        if((msb & 0b10000000) >0 )    
			return (float)((int32_t)((int32_t)0xFF << 24 |((uint32_t)msb<<16) | ((uint32_t)midb<<8) | ((uint32_t)lsb)))/(float)divideBy;
		else
			return (float)(((uint32_t)msb<<16) | ((uint32_t)midb<<8) | ((uint32_t)lsb))/(float)divideBy;
}


void 
STIM300CTL::streamSensorData2fd(int i){
	printf("%s","starting streaming");
	char  out[128];
	uint8_t nl='\n';
	int k = ::open("/dev/ttyACM0",O_WRONLY);
	i=k;
	while(1){
		getData(NORMAL_MODE_93);
		meas_result ret = getSensorData();
		sprintf(out,"%2.5f\t %2.5f\t %2.5f\t %2.5f\t %2.5f\t %2.5f\t\0",(double)ret.accelX,(double)ret.accelY,(double)ret.accelZ,
				(double)ret.gyroX,(double)ret.gyroY,(double)ret.gyroZ);
		::write(i,(uint8_t *)out,strlen(out));
		::write(i,&nl,1);
	}
}
