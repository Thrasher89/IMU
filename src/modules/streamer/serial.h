#ifndef _SERIAL_H_
#define _SERIAL_H_


#define SYNC_TIMEOUT_CNT 200 
class Serial
{
	private:
		int fd = -1;
		const char* devName="/dev/null";
		int serialSetSpeed(int speed);

	public:
		Serial();
		int isDataAvailable();
		int serialOpen(bool write=false);
		void serialClose();
		int getData(uint8_t *recv, unsigned len);
		int putData(uint8_t *trans, unsigned len);
		void printHex(unsigned len);
		void printAscii(unsigned len);
		int init(const char *serialPortdevname, int speed);
		void sync2CR_LF(void);
		void sync(void);
};

#endif
