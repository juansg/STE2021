# select which compiler to use
CC = clang
OPT = opt

# select which libraries should be linked in
LDFLAGS = -lm

# Compilation flags to use
CFLAGS = -O3 -DSPEC_CPU=1
OTHER_SRCS = a2m.c aligneval.c alignio.c alphabet.c \
         clustal.c cluster.c core_algorithms.c dayhoff.c debug.c  \
				 display.c emit.c emulation.c eps.c file.c getopt.c gki.c \
				 gsi.c histogram.c hmmcalibrate.c hmmio.c hmmsearch.c hsregex.c \
				 iupac.c masks.c mathsupport.c misc.c modelmakers.c msa.c msf.c \
				 phylip.c plan7.c plan9.c postprob.c prior.c revcomp.c rk.c     \
				 selex.c seqencode.c shuffle.c sqerror.c sqio.c squidcore.c     \
				 sre_ctype.c sre_math.c sre_random.c sre_string.c ssi.c stack.c \
				 stockholm.c tophits.c trace.c translate.c types.c ucbqsort.c   \
				 vectorops.c weight.c

#######################
##### TARGETS
#######################
all: serial.out bdx.out tls.out

bdx.out: fast_algorithms.c $(OTHER_SRCS)
	$(CC) -g $(CFLAGS) -fopenmp -fopenmp_use=bdx -DUSE_BDX $^ -o $@ $(LDFLAGS)

tls.out: fast_algorithms.c $(OTHER_SRCS)
	$(CC) -g $(CFLAGS) -fopenmp -DUSE_TLS -mrtm $^ -o $@ $(LDFLAGS)

ordered.out: fast_algorithms.c $(OTHER_SRCS)
	$(CC) -g $(CFLAGS) -fopenmp -DORDERED $^ -o $@ $(LDFLAGS)

serial.out: fast_algorithms.c $(OTHER_SRCS)
	$(CC) -g $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	$(RM) *.o *.ll *.bc *.out

run-small-serial: serial.out
	bash -c "time ./serial.out --fixed 0 --mean 325 --num 45000 --sd 200 --seed 0 inputs/bombesin.hmm"

run-small-bdx: bdx.out
	bash -c "time ./bdx.out --fixed 0 --mean 325 --num 45000 --sd 200 --seed 0 inputs/bombesin.hmm"

run-small-tls: tls.out
	bash -c "time ./tls.out --fixed 0 --mean 325 --num 45000 --sd 200 --seed 0 inputs/bombesin.hmm"

run-small-ordered: ordered.out
	bash -c "time ./ordered.out --fixed 0 --mean 325 --num 45000 --sd 200 --seed 0 inputs/bombesin.hmm"

run-medium-serial: serial.out
	bash -c "time ./serial.out --fixed 0 --mean 425 --num 85000 --sd 300 --seed 0 inputs/leng100.hmm"

run-medium-bdx: bdx.out
	bash -c "time ./bdx.out --fixed 0 --mean 425 --num 85000 --sd 300 --seed 0 inputs/leng100.hmm"

run-medium-tls: tls.out
	bash -c "time ./tls.out --fixed 0 --mean 425 --num 85000 --sd 300 --seed 0 inputs/leng100.hmm"

run-medium-ordered: ordered.out
	bash -c "time ./ordered.out --fixed 0 --mean 425 --num 85000 --sd 300 --seed 0 inputs/leng100.hmm"

run-large-serial: serial.out
	bash -c "time ./serial.out inputs/nph3.hmm inputs/swiss41"

run-large-bdx: bdx.out
	bash -c "time ./bdx.out inputs/nph3.hmm inputs/swiss41"	

run-large-tls: tls.out
	bash -c "time ./tls.out inputs/nph3.hmm inputs/swiss41"	

run-large-ordered: ordered.out
	bash -c "time ./ordered.out inputs/nph3.hmm inputs/swiss41"
