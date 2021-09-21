/* +++Date last modified: 05-Jul-1997 */

/*
**  BITCNTS.C - Test program for bit counting functions
**
**  public domain by Bob Stout & Auke Reitsma
*/

#include <stdio.h>
#include <stdlib.h>
#include "conio.h"
#include <limits.h>
#include <time.h>
#include <float.h>
#include "bitops.h"

#define FUNCS  7
double tiempo_promedio=0;

static int CDECL bit_shifter(long int x);

int __bdx_cond__(int flag) {return 1;}
void __bdx_stage_begin__(void) {}
void __bdx_stage_end__(void) {}

int main1(int argc, char *argv[], int print)
{
struct timeval start_time, end_time;
double run_time;
//	int print = 1;
/*   clock_t start, stop; */
  /*  double ct = 0.0; */
/*     double cmin = DBL_MAX, cmax = 0;  */
    long i;
/*     int cminix, cmaxix; */
  long j, n, seed;
  int iterations;
  static int (* CDECL pBitCntFunc[FUNCS])(long) = {
    bit_count,
    bitcount,
    ntbl_bitcnt,
    ntbl_bitcount,
    /*            btbl_bitcnt, DOESNT WORK*/
    BW_btbl_bitcount,
    AR_btbl_bitcount,
    bit_shifter
  };
  static char *text[FUNCS] = {
    "Optimized 1 bit/loop counter",
    "Ratko's mystery algorithm",
    "Recursive bit count by nybbles",
    "Non-recursive bit count by nybbles",
    /*            "Recursive bit count by bytes",*/
    "Non-recursive bit count by bytes (BW)",
    "Non-recursive bit count by bytes (AR)",
    "Shift and count bits"
  };
  if (argc<2) {
    fprintf(stderr,"Usage: bitcnts <iterations>\n");
    exit(EXIT_FAILURE);
	}
  iterations=atoi(argv[1]);
  
  if (print)
      puts("Bit counter algorithm benchmark\n");
  gettimeofday(&start_time, NULL);
//	#pragma omp parallel for ordered(1) private(seed, n, j) use(psbdx, (long)7)
  for (i = 0; i < FUNCS; i++) {
	unsigned int rSeed = i;
/*FGG    
    start = clock();
    
    for (j = n = 0, seed = rand(); j < iterations; j++, seed += 13)
*/
	int random = rand_r(&rSeed);
	n = 0;
    for (j = 0; j < iterations; j++) {
		int temp;
		seed = 13*j + random;
		switch(i) {
			case 0:
				temp = bit_count(seed);
				break;
			case 1:
				temp = bitcount(seed);
				break;
			case 2:
				temp = ntbl_bitcnt(seed);
				break;
			case 3:
				temp = ntbl_bitcount(seed);
				break;
			case 4:
				temp = BW_btbl_bitcount(seed);
				break;
			case 5:
				temp = AR_btbl_bitcount(seed);
				break;
			case 6:
				temp = bit_shifter(seed);
				break;
		}

		n += temp;
	}
    
/*FGG
    stop = clock();
    ct = (stop - start) / (double)CLOCKS_PER_SEC;
    if (ct < cmin) {
	 cmin = ct;
	 cminix = i;
    }
    if (ct > cmax) {
	 cmax = ct;
	 cmaxix = i;
    }

    printf("%-38s> Time: %7.3f sec.; Bits: %ld\n", text[i], ct, n);
*/

//	#pragma omp ordered depend(sink: i-1)
    if (print)
        printf("%-38s> Bits: %ld\n", text[i], n);
//	#pragma omp ordered depend(source)
  }
gettimeofday(&end_time, NULL);
    
run_time = ((end_time.tv_sec  - start_time.tv_sec) * 1000000u + end_time.tv_usec - start_time.tv_usec) / 1.e6;
tiempo_promedio+=run_time;
printf("Tiempo promedio %lf\n",tiempo_promedio);  
/*FGG
  printf("\nBest  > %s\n", text[cminix]);
  printf("Worst > %s\n", text[cmaxix]);
*/
  return 0;
}

static int CDECL bit_shifter(long int x)
{
  int i, n;
  
  for (i = n = 0; x && (i < (sizeof(long) * CHAR_BIT)); ++i, x >>= 1)
    n += (int)(x & 1L);
  return n;
}