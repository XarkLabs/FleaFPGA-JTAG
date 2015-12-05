# cygwin makefile for FleaFPGA-JTAG utility

CC	= gcc
CFLAGS	= -m32 -O2 -g -Wall -Wshadow -Wextra `pkg-config --libs --cflags libftdi1`
#CFLAGS += -DVME_DEBUG 

INC = hardware.h vmopcode.h Makefile
SRC = ispvm_ui.c ivm_core.c hardware.c
OBJ = $(SRC:.c=.o)

.PHONY: all clean

FleaFPGA-JTAG: $(OBJ)
	$(CC) $^ -o $@ $(CFLAGS)

#	strip -S $@

$(OBJ): $(INC)

all: clean FleaFPGA-JTAG

clean:
	rm -f ./FleaFPGA-JTAG *.o
