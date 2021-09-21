# Makefile for rgbyuv

TARGET=rgbyuv
OBJS=imgio.o bmark.o
CFLAGS=-O2 -D_GNU_SOURCE -std=c99 -g 
LIBS=-lm -lpthread
CC=clang

all: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LIBS) -o $(TARGET)

%.o : %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.o $(TARGET) *comp.ppm yuvout.ppm
