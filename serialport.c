#include <stdio.h>
#include <string.h>
#include <signal.h>

#include <errno.h> // Error integer and strerror() function
#include <unistd.h> // write(), read(), close()
#include <fcntl.h> // Contains file control like O_RDER
#include <termios.h> // Contain POISX terminal control definitions

#include "serialport.h"

/*----------------------------------------------------------*/
/*                          DEFINE                          */
/*----------------------------------------------------------*/
/* TTY */
#define TTYNAME_LEN		256

/* PARITY */
#define P_NONE			0x0000
#define P_ODD			0x0001
#define P_EVEN			0x0002

/* STOP BIT(s) */
#define S_ONE			0x0001
#define S_TWO			0x0002

/*----------------------------------------------------------*/
/*                         VARIABLES                        */
/*----------------------------------------------------------*/
static int				tty_fd					= -1;
static char				tty_fname[TTYNAME_LEN]	= {0};

static int				nBaudrate				= 9600;
static int				nDatabit				= 8;

static speed_t			tty_baudrate			= B9600;
static tcflag_t			tty_databit				= CS8;
static int				tty_parity				= P_NONE;
static int				tty_stopbit				= S_ONE;

static struct termios	oldOption				= {0};

/*----------------------------------------------------------*/
/*                                                          */
/*----------------------------------------------------------*/
static void ttySignalHandler(int signal)
{
	fprintf(stderr, "TTY:- Recieve signal %3d: %s\n", signal, strsignal(signal));
	tty_close();
}

/*----------------------------------------------------------*/
/*                                                          */
/*----------------------------------------------------------*/
void tty_setPortName(char *portname)
{
	snprintf(tty_fname, TTYNAME_LEN, portname);
}

void tty_setBaudrate(int baudrate)
{
	nBaudrate = 0;

	switch(baudrate) {
		case 50:		tty_baudrate = B50;		break;
		case 75:		tty_baudrate = B75;		break;
		case 110:		tty_baudrate = B110;	break;
		case 134:		tty_baudrate = B134;	break;
		case 150:		tty_baudrate = B150;	break;
		case 200:		tty_baudrate = B200;	break;
		case 300:		tty_baudrate = B300;	break;
		case 600:		tty_baudrate = B600;	break;
		case 1200:		tty_baudrate = B1200;	break;
		case 1800:		tty_baudrate = B1800;	break;
		case 2400:		tty_baudrate = B2400;	break;
		case 4800:		tty_baudrate = B4800;	break;
		case 9600:		tty_baudrate = B9600;	break;
		case 19200:		tty_baudrate = B19200;	break;
		case 38400:		tty_baudrate = B38400;	break;
		case 57600:		tty_baudrate = B57600;	break;
		case 115200:	tty_baudrate = B115200;	break;
		case 230400:	tty_baudrate = B230400;	break;
		case 460800:	tty_baudrate = B460800;	break;
		default:		tty_baudrate = B0;		return;
	}

	nBaudrate = baudrate;
}

void tty_setParity(int parity)
{
	switch(parity) {
		case 0:			tty_parity = P_NONE;	break;
		case 1:			tty_parity = P_ODD;		break;
		case 2:			tty_parity = P_EVEN;	break;
		default:		tty_parity = P_NONE;	break;
	}
}

void tty_setDatabits(int bits)
{
	nDatabit = 8;

	switch(bits) {
		case 5:			tty_databit = CS5;		break;
		case 6:			tty_databit = CS6;		break;
		case 7:			tty_databit = CS7;		break;
		case 8:
		default:		tty_databit = CS8;		return;
	}

	nDatabit = bits;
}

void tty_setStopbits(int bits)
{
	switch(bits) {
		case 1:			tty_stopbit = S_ONE;	break;
		case 2:			tty_stopbit = S_TWO;	break;
		default:		tty_stopbit = S_ONE;	break;
	}
}

/*----------------------------------------------------------*/
/*                                                          */
/*----------------------------------------------------------*/
int tty_getSignalPerSecond()
{
	int sumBits = 0;
	int sps;

	/* START BIT */
	sumBits += 1;
	
	/* DATA BITS */
	sumBits += nDatabit;

	/* PARITY BIT*/
	sumBits += (tty_parity == P_NONE)? 0: 1;

	/* STOP BIT */
	sumBits += 1;

	if(sumBits == 0) {
		sps = 0;
	} else {
		sps = nBaudrate / sumBits;
	}

	return sps;
}

/*----------------------------------------------------------*/
/*                                                          */
/*----------------------------------------------------------*/
int tty_read(char *buffer, int buffer_size, ssize_t *read_bytes)
{
	if(tty_fd < 0) {return TTY_RES_ERR;}

	ssize_t nbytes;

	memset(buffer, 0, sizeof(char) * buffer_size);

	errno = 0;
	nbytes = read(tty_fd, buffer, buffer_size);
	*read_bytes = nbytes;

	if(nbytes < 0) {
		if(errno == EAGAIN) {
			return TTY_RES_NONE;
		} else {
			fprintf(stderr, "TTY:- %d: %s\n", errno, strerror(errno));
			return TTY_RES_ERR;
		}
	} else if(nbytes > 0) {
		// data
	} else {
		// connection error
		return TTY_RES_ERR;
	}

	return TTY_RES_OK;
}

int tty_write(char *buffer, int buffer_size, ssize_t *write_bytes)
{	
	if(tty_fd < 0) {return TTY_RES_ERR;}

	ssize_t nbytes;

	errno = 0;
	nbytes = write(tty_fd, buffer, buffer_size);
	*write_bytes = nbytes;

	if(nbytes < 0) {
		fprintf(stderr, "TTY:- %d: %s\n", errno, strerror(errno));
		return TTY_RES_ERR;
	}

	return TTY_RES_OK;
}

/*----------------------------------------------------------*/
/*                                                          */
/*----------------------------------------------------------*/
int tty_open()
{
	struct termios ttyOption;

	struct sigaction act;
	sigset_t sigset;

	sigemptyset(&sigset);									// initialize and empty a signal set

	/* ADD SIGNAL */
	sigaddset(&sigset, SIGINT);								// interrupt from keyboard
	sigaddset(&sigset, SIGALRM);							// timer signal from alarm
	sigaddset(&sigset, SIGTERM);							// termination signal
	sigaddset(&sigset, SIGBUS);								// bus error (bad memory access)

	act.sa_handler = ttySignalHandler;
	act.sa_mask = sigset;
	act.sa_flags |= SA_RESTART;

	/* EXAMINE AND CHANGE A SIGNAL ACTION */
	sigaction(SIGINT, &act, NULL);
	sigaction(SIGALRM, &act, NULL);
	sigaction(SIGTERM, &act, NULL);
	sigaction(SIGBUS, &act, NULL);

	/* TTY */
	if(tty_fd > 0) {
		tty_close();
	}

	if(tty_fname == NULL) { 
		snprintf(tty_fname, TTYNAME_LEN, "/dev/ttyUSB0");
	}


	/**
	 *
	 * O_RDWR => Open for reading and writing. 
	 * O_NOCTTY => 
	 *
	 */
	
	tty_fd = open(tty_fname, O_RDWR | O_NOCTTY | O_NDELAY);
	if(tty_fd < 0) {
		fprintf(stderr, "TTY:- Unable to open %s\n", tty_fname);
		return TTY_RES_ERR;
	}

	fcntl(tty_fd, F_SETFL, FNDELAY);

	/* KEEP OLD OPTIONS */
	if(tcgetattr(tty_fd, &oldOption) != 0) {
		fprintf(stderr, "TTY:- %d: %s\n", errno, strerror(errno));
		return TTY_RES_ERR;
	}

	/* RESET */
	bzero(&ttyOption, sizeof(ttyOption));
	cfmakeraw(&ttyOption);

	/* BAUDRATE */
	cfsetspeed(&ttyOption, tty_baudrate);

	/* PARITY */
	switch(tty_parity) {
		case P_ODD:
			ttyOption.c_cflag |= PARENB;
			ttyOption.c_cflag |= PARODD;
			break;
		case P_EVEN:
			ttyOption.c_cflag |= PARENB;
			ttyOption.c_cflag &= ~PARODD;
			break;
		case P_NONE:
		default:
			ttyOption.c_cflag &= ~PARENB;
			ttyOption.c_cflag &= ~PARODD;
			break;
	}

	/* STOPBITS */
	switch(tty_stopbit) {
		case S_TWO:
			ttyOption.c_cflag |= CSTOPB;
			break;
		case S_ONE:
		default:
			ttyOption.c_cflag &= ~CSTOPB;
			break;
	}

	/* DATABITS */
	ttyOption.c_cflag &= ~CSIZE;							// clear all the size bits, then set data bits
	ttyOption.c_cflag |= tty_databit;

	/* OTHERS */
	ttyOption.c_cflag |= (CLOCAL | CREAD);					// CLOCAL: ignore modem control lines, CREAD: enable receiver
	ttyOption.c_lflag &= ~ICANON;							// disable canonical mode
	ttyOption.c_lflag &= ~ECHO;								// disable echo
	ttyOption.c_lflag &= ~ECHOE;							// disable erasure
	// ttyOption.c_lflag &= ~ECHONL;							// disable new-line echo
	ttyOption.c_lflag &= ~ISIG;								// disable interpretation of INTR, QUIT and SUSP

	ttyOption.c_oflag &= ~OPOST;							// disable implementation-defined output processing

	tcsetattr(tty_fd, TCSANOW, &ttyOption);

	return TTY_RES_OK;
}

void tty_close()
{
	if(tty_fd < 0) {return;}

	tcsetattr(tty_fd, TCSANOW, &oldOption);
	close(tty_fd);
	tty_fd = -1;
}