#ifndef TTYSERIAL_H
#define TTYSERIAL_H

/*----------------------------------------------------------*/
/*                          DEFINE                          */
/*----------------------------------------------------------*/
#define TTY_RES_NONE	0x0000
#define TTY_RES_OK		0x0001
#define TTY_RES_ERR		0x8000

/*----------------------------------------------------------*/
/*                                                          */
/*----------------------------------------------------------*/
extern void tty_setPortName(char *portname);
extern void tty_setBaudrate(int baudrate);
extern void tty_setParity(int parity);
extern void tty_setDatabits(int bits);
extern void tty_setStopbits(int bits);

extern int tty_getSignalPerSecond();

extern int tty_open();
extern void tty_close();

extern int tty_read(char *buffer, int buffer_size, ssize_t	*read_bytes);
extern int tty_write(char *buffer, int buffer_size, ssize_t *write_bytes);

#endif
