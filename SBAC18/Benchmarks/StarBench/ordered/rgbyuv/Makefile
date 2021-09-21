
CC = clang
OPT = opt
CFLAGS  = -O2 -D_GNU_SOURCE -std=c99 -fopenmp
EXECUTABLE = rgbyuv
LDFLAGS = -lm

all: $(EXECUTABLE)
 
$(EXECUTABLE): bmark.o imgio.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

bmark.o: bmark.bc
	$(CC) $(CFLAGS) -c $^ -o $@

bmark.bc: bmark.ll
	$(OPT) -use-transforms $^ -o $@

bmark.ll: bmark.c
	$(CC) $(CFLAGS) -S -emit-llvm $^ -o $@

clean:
	rm -rf *.o *.ll *.bc  $(EXECUTABLE) *comp.ppm yuvout.ppm
