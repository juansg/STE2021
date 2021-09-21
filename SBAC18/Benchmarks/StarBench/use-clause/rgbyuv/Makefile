
CC = clang
OPT = opt
CFLAGS  = -O3 -D_GNU_SOURCE -std=c99
LDFLAGS = -lm

all: serial.out ordered.out bdx.out tls.out
 
bdx.out: bmark.c imgio.c
	$(CC) $(CFLAGS) -fopenmp -fopenmp-use-dependence-analysis -D__USE_BDX__ $^ -o $@ $(LDFLAGS)

tls.out: bmark.c imgio.c
	$(CC) $(CFLAGS) -fopenmp -D__USE_TLS__ -mrtm $^ -o $@ $(LDFLAGS)

ordered.out: bmark.c imgio.c
	$(CC) $(CFLAGS) -fopenmp -D__ORDERED__ $^ -o $@ $(LDFLAGS)

serial.out: bmark.c imgio.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

run-serial: serial.out
	cp serial.out a.out;
	./run.sh serial;
	@echo "Finished!"

run-ordered: ordered.out
	cp ordered.out a.out;
	./run.sh ordered;
	@echo "Finished!"

run-bdx: bdx.out
	cp bdx.out a.out;
	./run.sh bdx;
	@echo "Finished!"

run-tls: tls.out
	cp tls.out a.out;
	./run.sh tls;
	@echo "Finished!"

clean:
	rm -rf *.o *.ll *.bc *.out *.ppm
