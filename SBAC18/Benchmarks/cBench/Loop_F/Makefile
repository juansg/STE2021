CC = clang
CFLAGS = -Wall -O3 -fopenmp
LDFLAGS = -lm

all: serial.out ordered.out parallel.out bdx.out

bdx.out: bitcnts-bdx.c bitarray.c bitcnt_*.c bitfiles.c bitstrng.c bstr_i.c loop-wrap.c 
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

serial.out: bitcnts-serial.c bitarray.c bitcnt_*.c bitfiles.c bitstrng.c bstr_i.c loop-wrap.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

ordered.out: bitcnts-ordered.c bitarray.c bitcnt_*.c bitfiles.c bitstrng.c bstr_i.c loop-wrap.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

parallel.out: bitcnts-parallel.c bitarray.c bitcnt_*.c bitfiles.c bitstrng.c bstr_i.c loop-wrap.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

run-serial: serial.out
	cp serial.out a.out;
	./__run 1;
	mv ftmp_out ftmp_out-serial;
	echo "Finished!"

run-ordered: ordered.out
	cp ordered.out a.out;
	./__run 1;
	mv ftmp_out ftmp_out-ordered;
	diff ftmp_out-ordered ftmp_out-serial;
	echo "Finished!"

run-bdx: bdx.out
	cp bdx.out a.out;
	./__run 1;
	mv ftmp_out ftmp_out-bdx;
	diff ftmp_out-bdx ftmp_out-serial;
	echo "Finished!"

run-parallel: parallel.out
	cp parallel.out a.out;
	./__run 1;
	mv ftmp_out
	ftmp_out-parallel;
	diff ftmp_out-parallel ftmp_out-serial;
	echo "Finished!"

clean:
	rm -f *.out *.ll *.opt ftmp_out*
