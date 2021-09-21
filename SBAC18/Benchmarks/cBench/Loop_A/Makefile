CC = clang
OPT = opt
CFLAGS = -Wall -O3
LDFLAGS = -lm

all: serial.out ordered.out parallel.out bdx.out tls.out

bdx.out: bitcnts.c bitarray.c bitcnt_*.c bitfiles.c bitstrng.c bstr_i.c loop-wrap.c 
	$(CC) $(CFLAGS) -fopenmp -fopenmp_use=bdx -DUSE_BDX $^ -o $@ $(LDFLAGS)

serial.out: bitcnts.c bitarray.c bitcnt_*.c bitfiles.c bitstrng.c bstr_i.c loop-wrap.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

ordered.out: bitcnts.c bitarray.c bitcnt_*.c bitfiles.c bitstrng.c bstr_i.c loop-wrap.c
	$(CC) $(CFLAGS) -fopenmp -DORDERED $^ -o $@ $(LDFLAGS)

tls.out: bitcnts.c bitarray.c bitcnt_*.c bitfiles.c bitstrng.c bstr_i.c loop-wrap.c
	$(CC) $(CFLAGS) -fopenmp -DUSE_TLS -mrtm $^ -o $@ $(LDFLAGS)

parallel.out: bitcnts.c bitarray.c bitcnt_*.c bitfiles.c bitstrng.c bstr_i.c loop-wrap.c
	$(CC) $(CFLAGS) -fopenmp -DPARALLEL $^ -o $@ $(LDFLAGS)

run-serial: serial.out
	cp serial.out a.out;
	./__run 1;
	mv ftmp_out ftmp_out-serial;
	@echo "Finished!"

run-ordered: ordered.out
	cp ordered.out a.out;
	./__run 1;
	mv ftmp_out ftmp_out-ordered;
	diff -q -s ftmp_out-ordered ftmp_out-serial;
	@echo "Finished!"

run-bdx: bdx.out
	cp bdx.out a.out;
	./__run 1;
	mv ftmp_out ftmp_out-bdx;
	diff -q -s ftmp_out-bdx ftmp_out-serial;
	@echo "Finished!"

run-tls: tls.out
	cp tls.out a.out;
	./__run 1;
	mv ftmp_out ftmp_out-tls;
	diff -q -s ftmp_out-tls ftmp_out-serial;
	@echo "Finished!"

run-parallel: parallel.out
	cp parallel.out a.out;
	./__run 1;
	mv ftmp_out ftmp_out-parallel;
	diff -q -s ftmp_out-parallel ftmp_out-serial;
	@echo "Finished!"

clean:
	rm -f *.o *.ll *.bc *.out ftmp_out*
