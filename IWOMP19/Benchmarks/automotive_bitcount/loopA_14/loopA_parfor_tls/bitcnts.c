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

#include <sys/time.h>

#include <omp.h>

#define __USE_TLS__
#include <ompuseclause.h>

#define FUNCS  7

static int CDECL bit_shifter(long int x);

double tiempo_promedio=0;

int main1(int argc, char *argv[], int print)
{
/*   clock_t start, stop; */
  /*  double ct = 0.0; */
/*     double cmin = DBL_MAX, cmax = 0;  */
    int i;
    long nL;
    struct timeval start_time, end_time;
    double run_time;    
/*     int cminix, cmaxix; */
  long j,n,seed;
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
  
  for (i = 0; i < FUNCS; i++) {
/*FGG    
    start = clock();
    
    for (j = n = 0, seed = rand(); j < iterations; j++, seed += 13)
*/
    gettimeofday(&start_time, NULL);   
    
    n=0;
    //seed=1;
    #pragma omp parallel for ordered(1) use(tls,502) firstprivate(i,pBitCntFunc,iterations) private(seed) shared(n) num_threads(4) default(none)  //linear(seed:13)  
    for (j = 0; j < iterations; j++ ){
	 seed=1+j*13;
	 n += pBitCntFunc[i](seed);
    }
    
    gettimeofday(&end_time, NULL);
    run_time = ((end_time.tv_sec  - start_time.tv_sec) * 1000000u + end_time.tv_usec - start_time.tv_usec) / 1.e6;
    //printf("Timer Loop %lf\n",run_time);  
    tiempo_promedio+=run_time;
    printf("Tiempo promedio %lf\n",tiempo_promedio);     
	 
    
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
    if (print)
        printf("%-38s> Bits: %ld\n", text[i], n);
  }
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
