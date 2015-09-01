#ifndef STIM300_H_
#define STIM300_H_

#include <drivers/device/uart.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>


class stim300 : public device::UART
{
	public:
		stim300(const char *name,
			const char *devname,
			unsigned int baudrate,
			int irq=0);
		virtual ~stim300();
	protected:
		virtual int init(void);
	private:	
		const char* _partNo=NULL;
		void getStartupDatagrams(void);

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
	uint8_t CRC[4];
	uint8_t CR; 
	uint8_t LF; 
};

#endif
