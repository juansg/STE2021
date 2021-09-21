
CC = clang
CFLAGS = -Wall -O3 -fopenmp
LDFLAGS = -lm -lomp

all: serial.out ordered.out parallel.out bdx.out

serial.out: susan-serial.c loop-wrap.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

parallel.out: susan-parallel.c loop-wrap.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

ordered.out: susan-ordered.c loop-wrap.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

bdx.out: susan-bdx.c loop-wrap.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

run-serial: serial.out
	cp serial.out a.out;
	./__run 1;
	mv output_large.corners.pgm output_large.corners-serial.pgm;
	echo "Finished!"

run-parallel: parallel.out
	cp parallel.out a.out;
	./__run 1;
	mv output_large.corners.pgm output_large.corners-parallel.pgm;
	echo "Finished!"

run-ordered: ordered.out
	cp ordered.out a.out;
	./__run 1;
	mv output_large.corners.pgm output_large.corners-ordered.pgm;
	diff output_large.corners-ordered.pgm output_large.corners-serial.pgm;
	echo "Finished!"

run-bdx: bdx.out
	cp bdx.out a.out;
	./__run 1;
	mv output_large.corners.pgm output_large.corners-bdx.pgm;
	diff output_large.corners-bdx.pgm output_large.corners-serial.pgm;
	echo "Finished!"

clean:
	rm -f *.out *.ll *.opt output_large.corners*.pgm ftmp_out*
