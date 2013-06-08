/*************************************************************************
*****    MACHINE-DEPENDENT SERIAL SUPPORT INCLUDE FILE WSERIAL.C     *****
*****                             (LINUX)                            *****
*****                                                                *****
*****               (C)1999, Directed Perception, Inc.               *****
*****                     All Rights Reserved.                       *****
*****                                                                *****
*****   Licensed users may freely distribute compiled code including *****
*****   this code and data. Source data and code may NOT be          *****
*****   distributed without the prior written consent from           *****
*****   Directed Perception, Inc.                                    *****
*****	      Directed Perception, Inc. reserves the right to make   *****
*****   changes without further notice to any content herein to      *****
*****   improve reliability, function or design. Directed Perception *****
*****   shall not assume any liability arising from the application  *****
*****   or use of this code, data or function.                       *****
*****                                                                *****
**************************************************************************

CHANGE HISTORY:
   1/21/98: v1.0d.  First release of Linux serial port drivers developed
                    and tested under RedHat Linux v5.1.

**************************************************************************/

#include <termios.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include "linuxser.h"
#include <sys/select.h>

#define DEBUG 0

//***************************************************************
//*****                LOCAL STATIC STATE                   *****
//***************************************************************
static int err;

static unsigned char peeked_char, peeked_char_avail = FALSE;

//********************************************************************
//*****               PORTED ROUTINES                            *****
//********************************************************************

portstream_fd openserial(char *portname)
{
	int pt_fd;
	struct termios info;

	fprintf(stderr, "openserial: opening device %s\n", portname);
	if ((pt_fd=open(portname, O_RDWR|O_NONBLOCK)) < 0)
	{
		perror("Error opening serial port");
		return PORT_NOT_OPENED;
	}

	if (tcgetattr(pt_fd, &info) < 0)
	{
		perror("Error using TCGETS in ioctl.");
		close(pt_fd);
		return PORT_NOT_OPENED;
	}

	/* restore old values to unhang the PTU, if hung */
	info.c_iflag=1280;
	info.c_oflag=5;
	info.c_oflag=5;
	info.c_cflag=3261;
	info.c_lflag=35387;

	if (tcsetattr(pt_fd, TCSANOW, &info) < 0)
	{
		perror("Error using TCSETS in ioctl.");
		close(pt_fd);
		return PORT_NOT_OPENED;
	}
	close(pt_fd);

	if ((pt_fd=open(portname, O_RDWR)) < 0)
	{
		perror("Error opening serial port");
		return PORT_NOT_OPENED;
	}

	if (tcgetattr(pt_fd, &info) < 0)
	{
		perror("Error using TCGETS in ioctl");
		close(pt_fd);
		return PORT_NOT_OPENED;
	}

	info.c_iflag = IGNBRK | IGNPAR;
	info.c_iflag &= ~(INLCR | ICRNL | IUCLC | ISTRIP | IXON | BRKINT);
	info.c_oflag &= ~OPOST;
	info.c_lflag &= ~(ICANON | ISIG | ECHO);
	info.c_cflag = B9600 | CS8 | CREAD;

	if (tcsetattr(pt_fd, TCSANOW, &info) < 0)
	{
		perror("Error using TCSETS in ioctl");
		close(pt_fd);
		return PORT_NOT_OPENED;
	}

	return pt_fd;
}

char closeserial(portstream_fd pt_fd)
{
	if (pt_fd != PORT_NOT_OPENED) {
		close(pt_fd);
	}
	return TRUE;
}

/* returns TRUE if all is OK */
char SerialBytesOut(portstream_fd pt_fd, unsigned char *buffer,
		            int nBytes)
{
	register int nout;
	nout = write(pt_fd, buffer, nBytes);      /* command byte(s) */

	if (nout == nBytes)
		return TRUE;
	else
		return FALSE;
}

/* returns TRUE if executed correctly */
char SerialBytesIn (portstream_fd pt_fd, unsigned char *buffer,
 		            unsigned int nBytes, long timeoutVal)
{
	fd_set         readfds; /* list of fds for select to listen to */
	register int   i,noconn=0,n,bytesrec=0;
	char           inchars[256],*vptr;
	struct timeval timeout={((timeoutVal > 0) ? timeoutVal : 10),0};

	if (pt_fd == PORT_NOT_OPENED)
	   return FALSE;

	vptr=buffer;

	FD_ZERO(&readfds); FD_SET(pt_fd,&readfds);
	/* put only <fd> into the list of file descriptors we want to listen to */

	for (i=0; (i<nBytes) && (noconn==0); i++) {
		if (select(FD_SETSIZE,&readfds,(fd_set *)NULL,
			(fd_set *)NULL,&timeout)>0) {
				n=read(pt_fd,inchars,1); /* read one character at a time */
				*vptr++ = inchars[0];
				bytesrec++;
		} else {  /* if timeout occurred */
			noconn=1;
			fprintf(stderr,"SerialBytesIn: timeout during data receive\n");
		}
	}
#if DEBUG
	if (bytesrec == 1)
		fprintf(stderr,"SerialBytesIn: received <%u>\n", *buffer);
	else
		fprintf(stderr,"SerialBytesIn: received <%s> %d bytes - req. %d bytes\n", buffer, bytesrec, nBytes);
#endif
	if (bytesrec == nBytes) {
		return TRUE;
	} else {
		return FALSE;
	}
}

// returns TRUE if BYTE available to peek at; otherwise, FALSE
char PeekByte(portstream_fd portstream, unsigned char *peekedByte)
{
   return FALSE;
}


char FlushInputBuffer(portstream_fd pt_fd) {
	return TRUE;
}


/* send a string to the PTU on file descriptor <fd> */
char SerialStringOut(portstream_fd pt_fd, unsigned char *buffer)
{
	char s[256];
	register int len,nout;

	sprintf(s,"%s",buffer);  /* make a proper string out of buffer, just in case
								buffer doesn't end with a '\0' -- this is
								unnecessary if the '\0' is ensured to be there
								already */
	len=strlen(s);

	nout = write(pt_fd,s,len);       /* switch code and message */
	nout+= write(pt_fd," ",1);       /* terminator */

	return TRUE;
}

// if string successfully read, returns TRUE; otherwise, the negative error
// code. The number of characters read is returned in charsRead.
char ReadSerialLine(portstream_fd pt_fd, unsigned char *strbuffer,
			long timeoutVal, int *charsRead)
{
	char           out[2],*c,*sb;
	fd_set         readfds;
	int            n;
	unsigned char  done=FALSE;
	struct timeval timeout={((timeoutVal > 0) ? timeoutVal : 10),0};

	sprintf(strbuffer,"");

	FD_ZERO(&readfds); FD_SET(pt_fd,&readfds);
	c=out;
	sb=strbuffer;

	while (!done && select(FD_SETSIZE,&readfds,(fd_set *)NULL,
                         (fd_set *)NULL,&timeout)>0) {
		n=read(pt_fd,c,1);
		if (*c == '\n') {  /* end of message.  Terminate string and loop */
			*sb = '\0';
			done=TRUE;
		} else if (*c == '\r') {
			/* do nothing, if controller sends \r\n */
		} else {
			/* put current character into return string */
			*sb++ = *c;
		}
		*charsRead += n;
	}

	if (!done) {  /* timeout occurred */
		return(TIMEOUT_CHAR_READ);
	}

	if (strbuffer[0] == '!' || strbuffer[1] == '!') {
		/* if message from PTU begins with a !, then it is an error.  If
		 * echoing mode is on, then the ! won't be the first character so this
		 * may need to be changed.  I never use echoing though... */

		fprintf(stderr, "ReadSerialLine: Error <%s>\n", strbuffer);
		return(FALSE);
	} else {
#if DEBUG
		fprintf(stderr, "ReadSerialLine: got string <%s>\n", strbuffer);
#endif
		return(TRUE);
	}
}

void do_delay(long millisec)
{
	struct timeval tv;
	double elapsedTime, startTime;
    double delayTime = millisec/1000.0;

	gettimeofday(&tv, NULL);
    startTime = tv.tv_sec + tv.tv_usec/1000000.0;
	do {
		gettimeofday(&tv, NULL);
        elapsedTime = (tv.tv_sec + tv.tv_usec/1000000.0) - startTime;
    } while (elapsedTime < delayTime);
}

// Whether you define this depends upon your particular machine.
// PCs reverse integer byte order, so this define is required.
// For almost all other machines, you would omit this define.
//
// If your machine has 2 byte signed and unsigned integers, and
// 4 byte signed integers, then you won't have to port the below code...
#define	INT_REVERSED


// 2 byte signed short int
char GetSignedShort(portstream_fd portstream, signed short *SHORTval, long
		    timeout) {
#ifdef INT_REVERSED
	SerialBytesIn( portstream, (((unsigned char *) SHORTval)+1), 1,
		       timeout);
	SerialBytesIn( portstream, ( (unsigned char *) SHORTval), 1, timeout);
#if DEBUG
	fprintf(stderr,"GetSignedShort: received <%d>\n", *SHORTval);
#endif
#else
	SerialBytesIn( portstream, ( (unsigned char *) SHORTval), 1, timeout);
	SerialBytesIn( portstream, (((unsigned char *) SHORTval)++), 1,
		       timeout);
#endif
	return TRUE;
}

// 2 byte signed short int
char PutSignedShort(portstream_fd portstream, signed short *SHORTval) {
#ifdef INT_REVERSED
	SerialBytesOut( portstream, (((unsigned char *) SHORTval)+1), 1);
	SerialBytesOut( portstream, ( (unsigned char *) SHORTval), 1);
#else
	SerialBytesOut( portstream, ( (unsigned char *) SHORTval), 1);
	SerialBytesOut( portstream, (((unsigned char *) SHORTval)++), 1);
#endif
	return TRUE;
}

// 2 byte usigned short int
char GetUnsignedShort(portstream_fd portstream, unsigned short
		      *USHORTval, long timeout) {
#ifdef INT_REVERSED
	SerialBytesIn( portstream, (((unsigned char *) USHORTval)+1), 1,
		       timeout);
	SerialBytesIn( portstream, ( (unsigned char *) USHORTval), 1,
		       timeout);
#if DEBUG
	fprintf(stderr,"GetUnsignedShort: received <%d>\n", *USHORTval);
#endif
#else
	SerialBytesIn( portstream, ( (unsigned char *) USHORTval), 1, timeout);
	SerialBytesIn( portstream, (((unsigned char *) USHORTval)++), 1,
		       timeout);
#endif
	return TRUE;
}

// 2 byte unsigned short int
char PutUnsignedShort(portstream_fd portstream, unsigned short *USHORTval) {

#ifdef INT_REVERSED
	SerialBytesOut( portstream, (((unsigned char *) USHORTval)+1), 1);
	SerialBytesOut( portstream, ( (unsigned char *) USHORTval), 1);
#else
	SerialBytesOut( portstream, ( (unsigned char *) USHORTval),    1);
	SerialBytesOut( portstream, (((unsigned char *) USHORTval)++), 1);
#endif
	return TRUE;
}

// 4 byte signed short int
char GetSignedLong(portstream_fd portstream, signed long *LONGval, long
		   timeout) {
    long i, incr = 1;

#ifdef INT_REVERSED
	LONGval = (signed long *) (((unsigned char *) LONGval) + 3);
	incr = -1;
#endif
	for (i=0; i<4; i++) {
		SerialBytesIn( portstream, ((unsigned char *) LONGval), 1,
			       timeout);
		LONGval = (signed long *) (((unsigned char *) LONGval) + incr);
	}
	return TRUE;
}

// 4 byte signed short int
char PutSignedLong(portstream_fd portstream, signed long *LONGval) {

#ifdef INT_REVERSED
	SerialBytesOut( portstream, ((unsigned char *) LONGval)+3, 1);
	SerialBytesOut( portstream, ((unsigned char *) LONGval)+2, 1);
	SerialBytesOut( portstream, ((unsigned char *) LONGval)+1, 1);
	SerialBytesOut( portstream, ((unsigned char *) LONGval),   1);
#else
	SerialBytesOut( portstream, ((unsigned char *) LONGval),4);
#endif
	return TRUE;
}
