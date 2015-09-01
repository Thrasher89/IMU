#include <drivers/drv_gps.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

__EXPORT int s_main(int argc, char *argv[]);
int s_main (int argc, char **argv){
	char* devName="/dev/ttyS3";
	//int speed = 921600;
	int speed = 9600;

	int fd  = open(devName,O_RDWR);


	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(fd, &uart_config)) < 0) {
		warnx("ERR GET CONF %s: %d\n", devName, termios_state);
		close(fd);
		return -1; 
	}   

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		warnx("ERR SET BAUD %s: %d\n", devName, termios_state);
		close(fd);
		return -1; 
	}   

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", devName);
		close(fd);
		return -1;
	}






	while(1)
		write(fd,"test\n",5);
		sleep(3);
	return 0;
}
