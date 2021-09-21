#include "time.h"
#include <stdlib.h>

// call this function to start a nanosecond-resolution timer
struct timespec* timer_start() {
    struct timespec* start_time = (struct timespec*) malloc(sizeof(struct timespec));
    clock_gettime(CLOCK_MONOTONIC, start_time);
    return start_time;
}

// call this function to end a nanosecond-resolution timer
double timer_end(struct timespec* start) {
	struct timespec* temp = (struct timespec*) malloc(sizeof(struct timespec));
	struct timespec end;

	clock_gettime(CLOCK_MONOTONIC, &end);

	if ((end.tv_nsec-start->tv_nsec)<0) {
		temp->tv_sec = end.tv_sec-start->tv_sec-1;
		temp->tv_nsec = 1000000000+end.tv_nsec-start->tv_nsec;
	} 
	else {
		temp->tv_sec = end.tv_sec-start->tv_sec;
		temp->tv_nsec = end.tv_nsec-start->tv_nsec;
	}

	return temp->tv_sec + (temp->tv_nsec / 1e9);
}
