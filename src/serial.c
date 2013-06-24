/*
 * Serial communication utilities.
 * Author: Fernando Soto
 * Code extracted from the following sources:
 * [1] 	"The Linux Documentation Project - Serial Programming HowTo"
 * [2] 	http://isa.uniovi.es/~ialvarez/Curso/Mecatronica/C3-MontajeDelPrototipo/index.htm
 */


#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include "serial.h"

int SERIALDATARECEIVED = 0;


/*
 * Serial initialization
 * Configuration such that read operations are not blocking
 * and data reception is indicated through SIGIO signal handler
 */

int initSerial(const char *serialDevice,int baudRate, char parity, int dataBits,int stopBits)

{
	struct termios oldtio, newtio;
	struct sigaction saio; /* definition of signal action */
	int fd;

	// Open port
	/* open the device to be non−blocking (read will return immediately) */

	fd = open(serialDevice, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0)
		return fd;

	/* install the signal handler before making the device asynchronous */

	saio.sa_handler = signal_handler_IO;
	//saio.sa_mask = 0;
	saio.sa_flags = 0;
	saio.sa_restorer = NULL;
	sigaction(SIGIO,&saio,NULL);


	/* allow the process to receive SIGIO */

	fcntl(fd, F_SETOWN, getpid());

	/* Make the file descriptor asynchronous (the manual page says only
	O_APPEND and O_NONBLOCK, will work with F_SETFL...) */

	//fcntl(fd, F_SETFL, FASYNC);
	// Fernando: Added flag FNDELAY as seen in http://ulisse.elettra.trieste.it/services/doc/serial/
	fcntl(fd, F_SETFL, FASYNC | FNDELAY);


	tcgetattr(fd,&oldtio); /* save current port settings */

	// 	Set baudrate

	switch (baudRate)
	{
		case 9600:
			newtio.c_cflag = B9600;			break;
		case 19200:
			newtio.c_cflag = B19200;		break;
		case 38400:
			newtio.c_cflag = B38400;		break;
		case 57600:
			newtio.c_cflag = B57600;		break;
		case 115200:
			newtio.c_cflag = B115200;		break;
		default:
			close(fd);
			return -1;
	}

	// Set data bits

	switch (dataBits)
	{
		case 5:
			newtio.c_cflag |= CS5 | CLOCAL | CREAD;			break;
		case 6:
			newtio.c_cflag |= CS6 | CLOCAL | CREAD;			break;
		case 7:
			newtio.c_cflag |= CS7 | CLOCAL | CREAD;			break;
		case 8:
			newtio.c_cflag |= CS8 | CLOCAL | CREAD;			break;
		default:
			close(fd);
			return -2;
	}

	//	Set Parity

	switch (parity)
	{
		case 'N':
			newtio.c_iflag = IGNPAR ;			break;
		case 'P':
  			newtio.c_cflag |= PARENB;  			break;
  		case 'I':
  			newtio.c_cflag |= PARENB | PARODD;	break;
		default:
			close(fd);
			return -3;
	}

	//	Set Stop bits

	switch (stopBits)
	{
		case 1:
			break;
		case 2:
  			newtio.c_cflag |= CSTOPB;  			break;
		default:
			close(fd);
			return -4;
	}

	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VMIN] = 9;
	newtio.c_cc[VTIME] = 5;
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	return fd;
}


/*
 *
 */

void signal_handler_IO (int status)
{
	//printf("received SIGIO signal.\n");
	SERIALDATARECEIVED = 1;

}

/*
 * Function to determine whether there is new data available for reading
 */
int serialDataReceived() {

	if (SERIALDATARECEIVED) {
		SERIALDATARECEIVED = 0;
		return 1;
	} else {
		return 0;
	}
}


void int2littleEndianStr(unsigned int operand, unsigned char *str) {

	str[0] = (unsigned char)(operand>>24);
	str[1] = (unsigned char)(operand>>16);
	str[2] = (unsigned char)(operand>>8);
	str[3] = (unsigned char)operand;

}

unsigned int littleEndianStr2int(unsigned char *str) {

	int value = (unsigned char)str[0]<<24 | (unsigned char)str[1]<<16 |(unsigned char)str[2]<<8 | (unsigned char)str[3];

	return value;

}


/* Cola */


/*
 * Enqueue a character
 */

void RX_enqueue(queue* q, unsigned char c) {

	if (q->numElem < QUEUESIZE) {
		q->head++;
		if (q->head >= QUEUESIZE) {
			q->head = 0;
		}
		q->buff[q->head] = c;
		q->numElem++;
	} else {
		printf("Queue Overflow\n");
	}

}

/*
 * Dequeue a character
 */

unsigned char RX_dequeue(queue* q) {

	char c = 0;
	int tail;

	if (q->numElem > 0) {
		tail = q->head - q->numElem+1;
		if (tail < 0) tail += QUEUESIZE;
		c = q->buff[tail];
		q->numElem--;
	}

	return c;

}

char* RX_toStr(queue* q) {

	int i;
	int len = q->numElem;
	char* str = malloc(len+1);

	if(str == NULL) return NULL;

	for (i=0;i<len;i++) {
		str[i] = RX_dequeue(q);
	}
	str[len] = '\0';

	return str;
}
