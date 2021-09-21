
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
	mv output_large.edges.pgm output_large.edges-serial.pgm;
	@echo "Finished!"

run-parallel: parallel.out
	cp parallel.out a.out;
	./__run 1;
	mv output_large.edges.pgm output_large.edges-parallel.pgm;
	@echo "Finished!"

run-ordered: ordered.out
	cp ordered.out a.out;
	./__run 1;
	mv output_large.edges.pgm output_large.edges-ordered.pgm;
	diff -q -s output_large.edges-ordered.pgm output_large.edges-serial.pgm;
	@echo "Finished!"

run-bdx: bdx.out
	cp bdx.out a.out;
	./__run 1;
	mv output_large.edges.pgm output_large.edges-bdx.pgm;
	diff -q -s output_large.edges-bdx.pgm output_large.edges-serial.pgm;
	@echo "Finished!"

run-tls: tls.out
	cp tls.out a.out;
	./__run 1;
	mv output_large.edges.pgm output_large.edges-tls.pgm;
	diff -q -s output_large.edges-tls.pgm output_large.edges-serial.pgm;
	@echo "Finished!"

clean:
	rm -f *.out *.o *.ll *.bc output_large.edges*.pgm ftmp_out*
