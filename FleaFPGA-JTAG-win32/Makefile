# Cygwin makefile for FleaFPGA-JTAG utility

PREFIX	= i686-w64-mingw32
CC		= $(PREFIX)-gcc
STRIP	= $(PREFIX)-strip
WINDRES	= $(PREFIX)-windres
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
	$(STRIP) -S $@

$(OBJ): $(INC)

icon.res: flea_logo.ico icon.rc
	$(WINDRES) icon.rc -O coff -F pe-i386 -o icon.res

all: clean FleaFPGA-JTAG

clean:
	rm -f ./FleaFPGA-JTAG *.o *.res
