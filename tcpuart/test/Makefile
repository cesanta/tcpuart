
.PHONY: all echo_server

MONGOOSE?=../../../mongoose
CC?=cc
PORT?=/dev/ttyUSB0
SPEED?=3000000

all: echo_test serial_echo serial_reader

echo_test: echo_test.c mongoose.c mongoose.h Makefile
	$(CC) -std=c99 -o $@ -I. -DMG_ENABLE_SSL -DCS_LOG_TS_DIFF \
	  echo_test.c mongoose.c -lcrypto -lssl

dumb_uart_echo:
	(stty $(SPEED) raw crtscts -echo && socat PIPE STDIO) <$(PORT) >$(PORT)

uart_echo: serial_echo
	(stty $(SPEED) raw crtscts -echo && ./serial_echo) <$(PORT) >$(PORT)

uart_null:
	(stty $(SPEED) raw crtscts && socat PIPE STDIO) <$(PORT) >/dev/null

serial_echo: serial_echo.c
	$(CC) -std=c99 -o $@ serial_echo.c

serial_reader: serial_reader.c
	$(CC) -std=c99 -o $@ serial_reader.c

clean:
	rm -f echo_test serial_echo serial_reader
