# Makefile for rgbyuv

TARGET=rgbyuv
OBJS=imgio.o bmark.o
CFLAGS=-D_GNU_SOURCE -std=c99 -g  -mrtm -fopenmp=libiomp5 -pthread  -O2
LIBS=-lm
CC=clang

all: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LIBS) -o $(TARGET)

%.o : %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.o $(TARGET) *comp.ppm yuvout.ppm
