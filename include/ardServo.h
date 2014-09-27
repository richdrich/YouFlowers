/*
 * ardServo.h
 *
 *  Created on: Dec 15, 2012
 *      Author: richardparratt
 */

#ifndef ARDSERVO_H_
#define ARDSERVO_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <glob.h>

static int tty_fd;
static fd_set read_fds, write_fds, except_fds;

bool initServo() {
#ifdef MAC
#define PORT "/dev/tty.usbmodem*"
#define PORT_WILD true
#else
#define PORT "/dev/ttyO1"
#define PORT_WILD false
#endif

	char serFile[100];
	if(PORT_WILD) {
		glob_t glob_info;
		glob(PORT, 0, NULL, &glob_info);

		if(glob_info.gl_pathc==0) {
			fprintf(stderr, "No matches for port %s\n", PORT);
			return false;
		}

		strncpy(serFile, glob_info.gl_pathv[0], 99);

		fprintf(stdout, "Servo on port %s\n", serFile);
		globfree(&glob_info);
	}
	else {
		strncpy(serFile, PORT, 99);
	}

	struct termios tio;
    memset(&tio,0,sizeof(tio));
    tio.c_iflag=0;
    tio.c_oflag=0;
    tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
    tio.c_lflag=0;
    tio.c_cc[VMIN]=1;
    tio.c_cc[VTIME]=5;

    tty_fd=open(serFile, O_RDWR | O_NONBLOCK);
    if(tty_fd < 0) {
    	fprintf(stdout, "Servo port %s open failed\n", serFile);
    	return false;
    }

    cfsetospeed(&tio,B9600);
    cfsetispeed(&tio,B9600);

    tcsetattr(tty_fd,TCSANOW,&tio);

    // Initialize file descriptor sets
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    FD_ZERO(&except_fds);
    FD_SET(tty_fd, &read_fds);

    return true;
}

void centreServo() {
	write(tty_fd, "SX\n", 3);
}

void sendServo(int servoNum, int angle) {
	char cmd[7];

	snprintf(cmd, 7, "S%01d%03d\n", servoNum, angle);

	fprintf(stderr, "Send: [%s]", cmd);

	write(tty_fd, cmd, strlen(cmd));

    // Set timeout to 1.0 seconds
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    // Wait for prompt
    fprintf(stderr, "prompt? ");
	char bu[2];
    do {
    	if (select(tty_fd + 1, &read_fds, &write_fds, &except_fds, &timeout) == 1)
    	{
    	    read(tty_fd, bu, 1);
    	    bu[1]=0;
    	    fputs(bu, stderr);
    	}
    	else
    	{
    		fprintf(stderr, "Read reponse from servo controller timeout\n");
    	}
    } while(*bu != '>');
    fputs("\n", stderr);
}


#endif /* ARDSERVO_H_ */
