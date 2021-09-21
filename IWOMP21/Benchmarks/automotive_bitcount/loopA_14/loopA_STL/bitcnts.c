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
#include <immintrin.h>


#define ALLOC(vol,tipo,x)   vol tipo x __attribute__((aligned(64)));

#define STRIP_SIZE 5020

#define FUNCS  7


static int CDECL bit_shifter(long int x);


double tiempo_promedio=0;

long next_iter_commit __attribute__((aligned(64)));

long n __attribute__((aligned(64)));


int main1(int argc, char *argv[], int print)
{
/*   clock_t start, stop; */
  /*  double ct = 0.0; */
/*     double cmin = DBL_MAX, cmax = 0;  */
    int i;
    
    struct timeval start_time, end_time;
    double run_time;
    long js;
    //long seeds;
    long nL;
        
/*     int cminix, cmaxix; */
  long j,seed;
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
  
    #pragma omp parallel num_threads(4)
    #pragma omp master

  for (i = 0; i < FUNCS; i++) {
/*FGG    
    start = clock();
    
    for (j = n = 0, seed = rand(); j < iterations; j++, seed += 13)
*/
    next_iter_commit=0;

    gettimeofday(&start_time, NULL);   
    
    n=0;
    //seed=1;

    #pragma omp taskloop grainsize(1)default(none) firstprivate(iterations,i,pBitCntFunc) private(seed,j,nL)  shared(next_iter_commit,n)  //reduction(+:n) //linear(seed:13)  //spec(n)
    for (js = 0; js < iterations; js+=STRIP_SIZE){
        char execute_in_htm;
        unsigned status;
        
        
        Retry:
		if ((next_iter_commit)!=js)
		{
		            execute_in_htm=1;
		            status = _xbegin();
		            if (status!=_XBEGIN_STARTED)
		                goto Retry;
		}
		else{
		            execute_in_htm=0;
		}
		
		//seed=1+js*13;
              nL=0;	

		for (j=js; ((j-js)< STRIP_SIZE) && (j<(iterations)); j++){
	        seed=1+j*13;
	        nL += pBitCntFunc[i](seed);
	        //seed+=13;
	       }
	    
	    if (execute_in_htm)
           {
                //__builtin_tsuspend ();
                //while((*next_itr_commit)!=original_arc);
                //__builtin_tresume ();

                if ((next_iter_commit)!=js)
                    _xabort(0xff);

                _xend();

           }

        n+=nL;
        (next_iter_commit)+=STRIP_SIZE;     
         
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
