#ifndef BDX_H_
#define BDX_H_

#include <pthread.h>

/* Use this to control thread-affinity. If 1 enabled, 0 disabled. */
#define ENABLE_THREAD_AFFINITY		0

/* The size of the iteration batch. */
#define BDX_BATCH_SIZE		        2

/* To prevent GCC (and ARM) from moving things around. */
#ifdef __arm__
	static __inline__ void __DMB(void) { __asm__ volatile("dmb"); }
#else
	static __inline__ void __DMB(void) { __asm__ volatile("": : :"memory"); }
#endif

/* This is the structure of the bdx buffer. */
/// there is only one loop-independent variable/dependence in this loop.
/// Therefore the type of the buffer is the type of that bariable - no need for struct.

const int bdx_NumThreads = 4;

// An structure of the size of a cache line
typedef struct _bdx_c_line_  {
    unsigned long long int value;
    unsigned char padding[56];
} bdx_cache_line;

/* These are the variables storing pointers to stage 2 and 3 threads. */
pthread_t bdx_thread2;
pthread_t bdx_thread3;
pthread_t bdx_thread4;

/* Used to synchronize execution of the second stage threads. */
volatile bdx_cache_line bdx_live[4];
volatile bdx_cache_line bdx_stage_flag[4];

/* These are shared variables. */
volatile int bdx_i;



// Function prototypes are below

/* This is used to stick a thread to a arbitrary core. */
extern int stick_this_thread_to_core(int core_id);

/* Source code for BDX Threads */
extern void * BDX_Thread_Source(void*);


#endif
