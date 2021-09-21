# select which compiler to use
CC = clang
OPT = opt

# select which libraries should be linked in
LDFLAGS = -lm

# Compilation flags to use
CFLAGS = -O3 -DSPEC_CPU=1

#######################
##### TARGETS
#######################
all: serial.out bdx.out tls.out

bdx.out: pbeampp.c implicit.c mcf.c mcfutil.c output.c \
         pbla.c pflowup.c psimplex.c pstart.c readmin.c treeup.c
	$(CC) $(CFLAGS) -fopenmp -fopenmp_use=bdx -DUSE_BDX $^ -o $@ $(LDFLAGS)

tls.out: pbeampp.c implicit.c mcf.c mcfutil.c output.c \
         pbla.c pflowup.c psimplex.c pstart.c readmin.c treeup.c
	$(CC) $(CFLAGS) -fopenmp -DUSE_TLS -mrtm $^ -o $@ $(LDFLAGS)

ordered.out: pbeampp.c implicit.c mcf.c mcfutil.c output.c \
         pbla.c pflowup.c psimplex.c pstart.c readmin.c treeup.c
	$(CC) $(CFLAGS) -fopenmp -DORDERED $^ -o $@ $(LDFLAGS)

serial.out: pbeampp.c implicit.c mcf.c mcfutil.c output.c \
         pbla.c pflowup.c psimplex.c pstart.c readmin.c treeup.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	$(RM) *.o *.ll *.bc *.out

run-small-bdx: bdx.out
	bash -c "time ./bdx.out inputs/test.in"

run-small-tls: tls.out
	bash -c "time ./tls.out inputs/test.in"

run-small-ordered: ordered.out
	bash -c "time ./ordered.out inputs/test.in"

run-small-serial: serial.out
	bash -c "time ./serial.out inputs/test.in"

run-medium-bdx: bdx.out
	bash -c "time ./bdx.out inputs/train.in"

run-medium-tls: tls.out
	bash -c "time ./tls.out inputs/train.in"

run-medium-ordered: ordered.out
	bash -c "time ./ordered.out inputs/train.in"

run-medium-serial: serial.out
	bash -c "time ./serial.out inputs/train.in"

run-large-bdx: bdx.out
	bash -c "time ./bdx.out inputs/ref.in"

run-large-tls: tls.out
	bash -c "time ./tls.out inputs/ref.in"

run-large-ordered: ordered.out
	bash -c "time ./ordered.out inputs/ref.in"

run-large-serial: serial.out
	bash -c "time ./serial.out inputs/ref.in"
