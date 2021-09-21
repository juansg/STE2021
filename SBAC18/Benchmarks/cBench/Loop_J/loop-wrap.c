#include <stdio.h>

void main1(int argc, char* argv[]);

double run_time_acc = 0.0;
int global_x_size = 0, global_y_size = 0, global_mask_size = 0;

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
  			  
  for (loop_wrap1=0; loop_wrap1<loop_wrap2; loop_wrap1++)
  {
    main1(argc, argv);
  }

	int dy = global_y_size - global_mask_size;
	fprintf(stderr, "y_size: %d (dy: %d)\n",global_y_size, dy);
	fprintf(stderr, "mask_size: %d\n",global_mask_size);

	fprintf(stderr, "Total execution time: %lf (s)\n", run_time_acc/1.0e6);
	fprintf(stderr, "Loop mean execution time: %lf (ms)\n",
		run_time_acc/(double)(loop_wrap2)/1.0e3);

  return 0;
}
