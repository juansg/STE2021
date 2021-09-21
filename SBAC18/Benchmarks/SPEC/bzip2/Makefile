# select which compiler to use
CC = clang
OPT = opt

# select which libraries should be linked in
LDFLAGS = -lm

# Compilation flags to use
CFLAGS = -O3 -DSPEC_CPU=1 -DDEBUG_DUMP=1

#######################
##### TARGETS
#######################
all: serial.out bdx.out tls.out

bdx.out: bzip2.c blocksort.c bzlib.c compress.c crctable.c \
					 decompress.c huffman.c randtable.c spec.c
	$(CC) -g $(CFLAGS) -fopenmp -fopenmp_use=bdx -DUSE_BDX $^ -o $@ $(LDFLAGS)

tls.out: bzip2.c blocksort.c bzlib.c compress.c crctable.c \
					 decompress.c huffman.c randtable.c spec.c
	$(CC) -g $(CFLAGS) -fopenmp -DUSE_TLS -mrtm $^ -o $@ $(LDFLAGS)

serial.out: bzip2.c blocksort.c bzlib.c compress.c crctable.c \
					 decompress.c huffman.c randtable.c spec.c
	$(CC) -g $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	bash -c "$(RM) *.o *.ll *.bc *.out *.compress.{5,7,9} *.uncompress.{5,7,9} *.uncompressed"

run-small-bdx: bdx.out
	bash -c "time ./bdx.out liberty.jpg 30"
	@bash -c 'for i in 5 7 9; do eval mv out.compress.\$$i bdx.compress.\$$i ; done'
	@bash -c 'for i in 5 7 9; do eval mv out.uncompress.\$$i bdx.uncompress.\$$i ; done'
	@bash -c 'mv out.uncompressed bdx.uncompressed'
	bash -c 'for i in 5 7 9; do eval diff -q -s serial.compress.\$$i bdx.compress.\$$i ; done'
	bash -c 'for i in 5 7 9; do eval diff -q -s serial.uncompress.\$$i bdx.uncompress.\$$i ; done'
	bash -c 'diff -q -s serial.uncompressed bdx.uncompressed'

run-small-tls: tls.out
	bash -c "time ./tls.out liberty.jpg 30"
	@bash -c 'for i in 5 7 9; do eval mv out.compress.\$$i tls.compress.\$$i ; done'
	@bash -c 'for i in 5 7 9; do eval mv out.uncompress.\$$i tls.uncompress.\$$i ; done'
	@bash -c 'mv out.uncompressed tls.uncompressed'
	bash -c 'for i in 5 7 9; do eval diff -q -s serial.compress.\$$i tls.compress.\$$i ; done'
	bash -c 'for i in 5 7 9; do eval diff -q -s serial.uncompress.\$$i tls.uncompress.\$$i ; done'
	bash -c 'diff -q -s serial.uncompressed tls.uncompressed'

run-small-serial: serial.out
	bash -c "time ./serial.out liberty.jpg 30"
	@bash -c 'for i in 5 7 9; do eval mv out.compress.\$$i serial.compress.\$$i ; done'
	@bash -c 'for i in 5 7 9; do eval mv out.uncompress.\$$i serial.uncompress.\$$i ; done'
	@bash -c 'mv out.uncompressed serial.uncompressed'

run-medium-bdx: bdx.out
	bash -c "time ./bdx.out text.html 280"

run-medium-tls: tls.out
	bash -c "time ./tls.out text.html 280"

run-medium-serial: serial.out
	bash -c "time ./serial.out text.html 280"

run-large-bdx: bdx.out
	bash -c "time ./bdx.out input.source 280"

run-large-tls: tls.out
	bash -c "time ./tls.out input.source 280"

run-large-serial: serial.out
	bash -c "time ./serial.out input.source 280"
