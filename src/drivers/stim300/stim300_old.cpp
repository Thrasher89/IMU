
/**
 * @file stim300.cpp
 *
 * Driver for the sensonor stim300 connected via UART.
 *
 * @author Stephan Zuercher
 */
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "stim300.h"
#include "sdLoger.h"



#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>


stim300::stim300(const char *name,
		const char *devname,
		unsigned int baudrate,
		int irq): device::UART(name,devname,baudrate,irq)
{
	printf("Class created");
	device::UART::init();
	init();

}
stim300::~stim300(){
	delete _partNo;
}


int
stim300::init()
{
	//getStartupDatagrams();
	return OK;
}
void 
stim300::getStartupDatagrams()
{
	/*
	struct s_datagram_partNo dPartNo;
	uint8_t buff;
	char* partNoStr= new char[25];

	//first data bite is 0x00
	get(&buff,1);
	if(buff!=0x00){
		printf("first bite wasnt 0x00");
		exit(-1);
	}
	get((uint8_t*)&dPartNo,sizeof(dPartNo));
	if(dPartNo.id!=0xB3){
		printf("Error reading PartNo, wrong datagramID");
		exit(-1);
	}
	int i=0;
	partNoStr[i++]=dPartNo.partNo1[0];
	partNoStr[i++]=dPartNo.partNo1[1];
	partNoStr[i++]=dPartNo.partNo1[2];
	partNoStr[i++]=dPartNo.delimiter1;
	partNoStr[i++]=dPartNo.partNo2[0];
	partNoStr[i++]=dPartNo.partNo2[0];
	partNoStr[i++]=dPartNo.partNo2[1];
	partNoStr[i++]=dPartNo.partNo2[1];
	partNoStr[i++]=dPartNo.partNo2[2];
	partNoStr[i++]=dPartNo.partNo2[2];
	partNoStr[i++]=dPartNo.delimiter2;
	partNoStr[i++]=dPartNo.partNo3[0];
	partNoStr[i++]=dPartNo.partNo3[0];
	partNoStr[i++]=dPartNo.partNo3[1];

	partNoStr[i++]=' ';
	partNoStr[i++]='r';
	partNoStr[i++]='e';
	partNoStr[i++]='v';
	partNoStr[i++]='.';
	partNoStr[i++]=' ';
	partNoStr[i++]=dPartNo.revision;
	partNoStr[i++]='\0';
	if(i<sizeof(partNoStr)){
		printf("partNo str initialization to short");
		exit(-1);
	}

	
	printf("partNo: %s",partNoStr);

	_partNo= partNoStr;
	*/

}


extern "C" { __EXPORT int stim300_main(int argc, char *argv[]); }
int stim300_main(int argc, char *argv[]){
	//sdLoger log;
	//stim300 x("stim300", "/dev/ttyS2", 921600);



	return 0;
}
