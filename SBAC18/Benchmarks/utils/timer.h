#include <time.h>

// call this function to start a nanosecond-resolution timer
extern struct timespec* timer_start() ;

// call this function to end a nanosecond-resolution timer
double timer_end(struct timespec*) ;
