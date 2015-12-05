# cygwin makefile for FleaFPGA-JTAG utility

CC		= i686-pc-mingw32-gcc
CFLAGS	= -O2 -g -Wall -Wshadow -Wextra
# CFLAGS += -DVME_DEBUG
LFLAGS	= -L.
LIBS	=  ftd2xx.lib -lwinmm -lcomdlg32

INC = hardware.h vmopcode.h ftd2xx.h Makefile
SRC = ispvm_ui.c ivm_core.c hardware.c
OBJ = $(SRC:.c=.o) icon.res

.PHONY: all clean

FleaFPGA-JTAG: $(OBJ)
	$(CC) $(LFLAGS) $^ $(LIBS) -o $@
	strip -S $@

$(OBJ): $(INC)

icon.res: flea_logo.ico icon.rc
	windres icon.rc -O coff -F pe-i386 -o icon.res

all: clean FleaFPGA-JTAG

clean:
	rm -f ./FleaFPGA-JTAG *.o *.res
