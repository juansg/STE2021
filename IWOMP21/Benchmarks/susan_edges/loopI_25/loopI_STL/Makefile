# ************ CCC - passing optimization flags ************ 
ifeq ($(CCC_OPTS),)
 CCC_OPTS =
endif
ifeq ($(CCC_OPTS_ADD),)
 CCC_OPTS_ADD =
endif

# Declarations
ifeq ($(ZCC),) 
 ZCC = clang-12
endif
ifeq ($(LDCC),)
 LDCC = clang-12
endif
ifeq ($(LD_OPTS),)
 LD_OPTS = -o asusan_e
endif

# Actions
all:		
#	$(ZCC) $(CCC_OPTS) $(CCC_OPTS_ADD) -c *.c -O3
	$(LDCC) $(LD_OPTS) $(CCC_OPTS_ADD) *.c -fopenmp=libiomp5  -lm -O3 -mrtm 

clean:
	rm -f a.out *.o *.a *.s *.i *.I
