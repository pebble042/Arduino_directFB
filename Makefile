app					= 			main_screen
serial_port 		=			serialport

LIBS	=	-ldirect -ldirectfb -llite -lleck -lpthread -lm

INCLUDES = -I /usr/local/include/directfb/

STATIC = -static
CFLAGS = -Wall

.PHONY: default

default:
	gcc $(app).c $(serial_port).c -o $(app) $(INCLUDES) $(LIBS)

clean:
	@rm -f *.o
	@rm -f *.a
	@rm -f $(app)