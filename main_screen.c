#include <stdio.h>
#include <stdlib.h>

#include <lite/lite.h>
#include <lite/window.h>

#include <leck/textbutton.h>

#include "serialport.h"

#define PATH "/home/camtac/Documents/directfb_arduino/button/"

/*==========================================
=           Setup serial port            =
==========================================*/

#define PORTNAME "/dev/ttyACM0"
#define BAUDRATE 9600
#define PARITY 0
#define DATABITS 8
#define STOPBITS 1 

#define WRITEBUF_SIZE 256


int   res;
ssize_t nbytes;
char  writeStr[WRITEBUF_SIZE+1] = {0};

static void tty_setting()
{
    tty_setPortName ( PORTNAME );
    tty_setBaudrate ( BAUDRATE );
    tty_setParity   ( PARITY );
    tty_setDatabits ( DATABITS );
    tty_setStopbits ( STOPBITS );
}


static void
button_pressed( LiteTextButton *button, void *ctx )
{
    memset(writeStr, 0, sizeof(char) * WRITEBUF_SIZE);
    snprintf(writeStr, read + 1, "%s\n", line);
    res = tty_write(writeStr, WRITEBUF_SIZE, &nbytes);
}

int main(int argc, char *argv[])
{
	LiteWindow				*window;
	LiteTextButton 			*button;
  LiteTextButtonTheme 	*textButtonTheme;
	DFBRectangle    		rect;
  DFBResult       		res;


	if (lite_open(&argc, &argv) && tty_open())
		return 1;

	/* Create a window */
	rect.x = LITE_CENTER_HORIZONTALLY;
    rect.y = LITE_CENTER_VERTICALLY;
    rect.w = 512;
    rect.h = 360;

    res = lite_new_window( 	NULL, 
                            &rect,
                            DWCAPS_ALPHACHANNEL, 
                            liteDefaultWindowTheme,
                            "Arduino & DiractFB",
                            &window );

    res = lite_new_text_button_theme(
          PATH "btn_img.png",
          &textButtonTheme);

    if( res != DFB_OK )
          return res;

     /* setup the button */
     rect.x = 30; rect.y = 30; rect.w = 150; rect.h = 50;
     res = lite_new_text_button(LITE_BOX(window), &rect, "Click", textButtonTheme, &button);

     //lite_on_text_button_press( button, button_pressed, window );

    /* show the window */
    lite_set_window_opacity( window, liteFullWindowOpacity );

    /* run the default event loop */
     lite_window_event_loop( window, 0 );

     /* destroy the window with all this children and resources */
     lite_destroy_window( window );

     lite_destroy_text_button_theme( textButtonTheme );


     /* deinitialize */
     lite_close();
     tty_close();


	return 0;
}