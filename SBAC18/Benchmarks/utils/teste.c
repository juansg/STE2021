#include <unistd.h>
#include <stdio.h>
#include "timer.h"

int main() {

	struct timespec* start = timer_start();

	sleep(1);

	printf("Timer %.9lf\n", timer_end(start));

	return 0;
}
