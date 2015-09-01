#include <drivers/drv_gps.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h> //atoi
#include <stdio.h>
#include <poll.h>

#ifndef STDIN_FILENO
#  define STDIN_FILENO 0
#endif

__EXPORT int uart_main(int argc, char *argv[]);
int uart_main (int argc, char **argv){

	if(argc<3){
		printf("\n not enought parameter enter: uart <deviceName> <speed>\n");
		return -1;
	}

	char* devName=argv[1];
	//int speed = 921600;
	int speed = atoi(argv[2]);
	if(speed==0){
		printf("\nspeed was not a number\n");
		return -1;
	
	}

	int fd  = open(devName,O_RDWR);


	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(fd, &uart_config)) < 0) {
		printf("ERR GET CONF %s: %d\n", devName, termios_state);
		close(fd);
		return -1; 
	}   

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		printf("ERR SET BAUD %s: %d\n", devName, termios_state);
		close(fd);
		return -1; 
	}   

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		printf("ERR SET CONF %s\n", devName);
		close(fd);
		return -1;
	}


	printf("\nport opened and speed set\n");


	/*
	struct pollfd fds[2];
	fds[0].fd = fd;
	fds[0].events = POLLIN;

	uint8_t buffer;
	*/


	while(1){
		write(fd,"Test\n",5);
		sleep(1);
		/*
		int ret = poll(fds,1, 200);

		if(read(0,&buffer,1)>0){
			write(fds[0].fd,&buffer,1);
			printf("c",buffer);
		}

		if (ret < 0) {
			printf("Error in polling\n");
			return -3;
		
		}
		else if (ret == 0){
			//Timeout
			continue;
		}

		if (fds[0].revents & POLLIN) {
			while(read(fds[0].fd,&buffer,1)>0)
				printf("%c",buffer);
			fflush(stdout);

		}
		*/




	}
	return 0;
}
