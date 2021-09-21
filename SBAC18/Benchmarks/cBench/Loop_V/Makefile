
CC = clang
OPT = opt
CFLAGS = -Wall -O3
LDFLAGS = -lm

all: serial.out ordered.out parallel.out bdx.out tls.out

serial.out: susan.c loop-wrap.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

parallel.out: susan.c loop-wrap.c
	$(CC) $(CFLAGS) -fopenmp -DPARALLEL $^ -o $@ $(LDFLAGS)

ordered.out: susan.c loop-wrap.c
	$(CC) $(CFLAGS) -fopenmp -DORDERED $^ -o $@ $(LDFLAGS)

bdx.out: susan.c loop-wrap.c
	$(CC) $(CFLAGS) -fopenmp -fopenmp_use=bdx -DUSE_BDX $^ -o $@ $(LDFLAGS)

tls.out: susan.c loop-wrap.c
	$(CC) $(CFLAGS) -fopenmp -DUSE_TLS -mrtm $^ -o $@ $(LDFLAGS)

run-serial: serial.out
	cp serial.out a.out;
	./__run 1;
	mv output_large.corners.pgm output_large.corners-serial.pgm;
	@echo "Finished!"

run-parallel: parallel.out
	cp parallel.out a.out;
	./__run 1;
	mv output_large.corners.pgm output_large.corners-parallel.pgm;
	@echo "Finished!"

run-ordered: ordered.out
	cp ordered.out a.out;
	./__run 1;
	mv output_large.corners.pgm output_large.corners-ordered.pgm;
	diff -q -s output_large.corners-ordered.pgm output_large.corners-serial.pgm;
	@echo "Finished!"

run-bdx: bdx.out
	cp bdx.out a.out;
	./__run 1;
	mv output_large.corners.pgm output_large.corners-bdx.pgm;
	diff -q -s output_large.corners-bdx.pgm output_large.corners-serial.pgm;
	@echo "Finished!"

run-tls: tls.out
	cp tls.out a.out;
	./__run 1;
	mv output_large.corners.pgm output_large.corners-tls.pgm;
	diff -q -s output_large.corners-tls.pgm output_large.corners-serial.pgm;
	@echo "Finished!"

clean:
	rm -f *.out *.o *.ll *.bc output_large.corners*.pgm ftmp_out*
