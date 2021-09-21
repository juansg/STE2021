#include <stdio.h>
#include <stdlib.h>

int main1(int argc, char *argv[], int print, double* run_time_acc);

#define FUNCS 7

int main(int argc, char* argv[])
{
  FILE* loop_wrap=NULL;
  long loop_wrap1, loop_wrap2;
  
  if ((loop_wrap=fopen("_finfo_dataset","rt"))==NULL)
  {
    fprintf(stderr,"\nError: Can't find dataset!\n");
    return 1;
  }

  fscanf(loop_wrap, "%ld", &loop_wrap2);
  fclose(loop_wrap);

	double run_time_acc = 0.0;
	int iterations = atoi(argv[1]);
  			  
  for (loop_wrap1=0; loop_wrap1<loop_wrap2; loop_wrap1++)
  {
      main1(argc, argv, loop_wrap1+1 == loop_wrap2, &run_time_acc);
  }

	fprintf(stderr, "iterations: %d\n", iterations);
	fprintf(stderr, "FUNCS: %d\n", FUNCS);
	fprintf(stderr, "Total execution time: %lf (s)\n", run_time_acc/1.0e6);
	fprintf(stderr, "Loop mean execution time: %lf (ms)\n",
		run_time_acc/(double)(FUNCS*loop_wrap2)/1.0e3);

  return 0;
}
