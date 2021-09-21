/*
 * Copyright (C) 2013 Michael Andersch <michael.andersch@mailbox.tu-berlin.de>
 *
 * This file is part of Starbench.
 *
 * Starbench is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Starbench is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Starbench.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#include "rgbyuv.h"
#include "imgio.h"

typedef struct timeval timer;

#define TIME(x) gettimeofday(&x, NULL)

/*
*   This program is a benchmark kernel written as part of the StarBench benchmark suite.
*   It is a fully parallelized RGB to YUV color converter, processing .ppm format pictures.
*   This application is a common one for embedded domain multiprocessors such as DSPs.
*/

/* Function declarations */
int initialize(rgbyuv_args_t* a, char* infile);
int finalize(rgbyuv_args_t* a);
void statsme();
long timediff(timer* starttime, timer* finishtime);
int processImage(rgbyuv_args_t* args);
int writeComponents(rgbyuv_args_t* args);

/* Global data */
static char* usage =    "Usage: %s <options>\n"
                        "-i infile          give input file path\n"
                        "-c iterations      specify number of iterations\n"
                        "-h                 this help text\n";
int iterations = 1;

char srcfiles[100][100];
int qtdFiles = 0;

/* For profiling */
timer launch, compute, disk;

/*
*   Function: initialize
*   --------------------
*   Load image information and color data and allocate output buffer.
*   Thread setup is NOT performed here in order to get accurate threading
*   overhead timing.
*/
int initialize(rgbyuv_args_t* a, char* infile) {
    int maxcolor = 0;
    int depth = 0;
    int width = 0;
    int height = 0;
    // Get the image from disk
    a->in_img = loadPPMFile(infile, &width, &height, &maxcolor, &depth);

    // Check for correct image type
    if (depth != IN_DEPTH) {
        fprintf(stderr, "Problems reading from %s\n\n", infile);
        exit(-1);
    }

    // Compute image params
    a->width = width;
    a->height = height;
    a->pixels = a->height * a->width;
    a->pY = calloc(a->pixels, sizeof(uint8_t));
    a->pU = calloc(a->pixels, sizeof(uint8_t));
    a->pV = calloc(a->pixels, sizeof(uint8_t));


    // Trap bad memory allocation
    if(a->in_img == NULL || a->pY == NULL || a->pU == NULL || a->pV == NULL )
        return -1;

    return 0;
}

/*
*   Function: finalize
*   ------------------
*   Releases memory after the processing is finished.
*/
int finalize(rgbyuv_args_t* a) {
    if(a->in_img)
        free(a->in_img);
    if(a->pY)
        free(a->pY);
    if(a->pU)
        free(a->pU);
    if(a->pY)
        free(a->pV);
    return 0;
}

/*
*   Function: timediff
*   ------------------
*   Compute the difference between timers starttime and finishtime in msecs.
*/
long timediff(timer* starttime, timer* finishtime)
{
    long msec;
    msec=(finishtime->tv_sec-starttime->tv_sec)*1000;
    msec+=(finishtime->tv_usec-starttime->tv_usec)/1000;
    return msec;
}

/*
*   Function: processImage
*   ----------------------
*   Called to perform the actual image conversion and disk output. Controls iterations.
*/
int processImage(rgbyuv_args_t* args) {

    uint8_t R,G,B,Y,U,V;
#ifdef VERBOSE
    fprintf(stderr, "Beginning conversion ... \n");
#endif

    for(int i = 0; i < iterations; i++) {
      uint8_t* in = args->in_img;
      uint8_t* pY = args->pY;
      uint8_t* pU = args->pU;
      uint8_t* pV = args->pV;

      for(int j = 0; j < args->pixels; j++) {
        R = *in++;
        G = *in++;
        B = *in++;

        Y = round(0.256788*R+0.504129*G+0.097906*B) + 16;
        U = round(-0.148223*R-0.290993*G+0.439216*B) + 128;
        V = round(0.439216*R-0.367788*G-0.071427*B) + 128;

        *pY++ = Y;
        *pU++ = U;
        *pV++ = V;
      }

#ifdef VERBOSE
      fprintf(stderr, "Finished conversion, writing to disk ... \n");
#endif

    }
    return 0;
}

/*
*   Function: writeComponents
*   -------------------------
*   Receives the buffer containing the converted image and separately writes Y, U and V component
*   to a disk image file. Also writes YUV image as an RGB image to disk.
*/
int writeComponents(rgbyuv_args_t* args) {

    FILE* fp;
    char yuvheader[256];
    char planeheader[256];
    char* targets[] = { "ycomp.ppm", "ucomp.ppm", "vcomp.ppm" };
    uint8_t* pY = args->pY;
    uint8_t* pU = args->pU;
    uint8_t* pV = args->pV;
    int pels = args->pixels;

    // Write headers
    snprintf(yuvheader, (size_t)255, "P6\n%d %d\n%d\n", args->width, args->height, MAXCOLOR);
    snprintf(planeheader, (size_t)255, "P5\n%d %d\n%d\n", args->width, args->height, MAXCOLOR);
#ifdef VERBOSE
    fprintf(stderr, "Created .ppm header: \n%s\n\n%s\n", yuvheader, planeheader);
#endif

    // Y component
#ifdef VERBOSE
    fprintf(stderr, "Writing to file %s\n", targets[0]);
#endif
    fp = fopen(targets[0], "w");
    fprintf(fp, "%s", planeheader);
    fwrite(pY, sizeof(uint8_t), pels, fp);
    fclose(fp);

    // For U component

#ifdef VERBOSE
    fprintf(stderr, "Writing to file %s\n", targets[1]);
#endif
    fp = fopen(targets[1], "w");

    if (fp == NULL)
        return -1;

    fprintf(fp, "%s", planeheader);
    fwrite(pU, sizeof(uint8_t), pels, fp);
    fclose(fp);

    // For V component

#ifdef VERBOSE
    fprintf(stderr, "Writing to file %s\n", targets[1]);
#endif
    fp = fopen(targets[2], "w");

    if (fp == NULL)
        return -1;

    fprintf(fp, "%s", planeheader);
    fwrite(pV, sizeof(uint8_t), pels, fp);
    fclose(fp);

    return 0;
}

__attribute__ ((noinline)) int __bdx_cond__(int flag) {__asm__(""); return 1;}
__attribute__ ((noinline)) void __bdx_stage_begin__(volatile int flag) {__asm__("");}
__attribute__ ((noinline)) void __bdx_stage_end__(void) {__asm__("");}


/** MAIN */
int main(int argc, char** argv) {

    int opt;
    extern char* optarg;
    extern int optind;
    char* infiles;

    timer io_start, b_start, b_end;

    // Who we are
    fprintf(stderr, "StarBench - RGBYUV Kernel\n");

    // Parse command line options
    while ( (opt=getopt(argc,argv,"i:c:h")) != EOF) {
        switch (opt) {
            case 'i':
                infiles = optarg;
                break;
            case 'c':
                iterations = atoi(optarg);
                break;
            case 'h':
                fprintf(stderr, usage, argv[0]);
                return 0;
                break;
            default:
                fprintf(stderr, usage, argv[0]);
                return 0;
                break;
        }
    }

    if (infiles == NULL || iterations < 1) {
        fprintf(stderr, "Illegal argument given, exiting\n");
        return -1;
    }

    FILE* ifp = fopen(infiles, "r");
    char iname[256], *ptr;

    while(fscanf(ifp, " %s", srcfiles[qtdFiles]) != EOF) {
        printf("File to read %s\n", srcfiles[qtdFiles]);
        qtdFiles++;
    }

    fclose(ifp);

	timer loop_time_start, loop_time_end;
	int exec = 1;
	int i;

	#pragma omp parallel for ordered(1) use(psbdx)
    for (i=0; i<qtdFiles; i++) {
		rgbyuv_args_t args;
		rgbyuv_args_t *argsP = &args;
		//rgbyuv_args_t *argsP = (rgbyuv_args_t *)malloc(sizeof(rgbyuv_args_t));
//	TIME(loop_time_start);
		#pragma omp ordered depend(sink: i-1)
		if(exec) {
		    

		    if(initialize(argsP, srcfiles[i])) {
		        fprintf(stderr, "Could Not Initialize Kernel Data\n");
		        exec = 0;
				continue;
		    }
		}
		#pragma omp ordered depend(source)

		if(exec) {
		    processImage(argsP);

		    writeComponents(argsP);

		    if(finalize(argsP)) {
		        fprintf(stderr, "Could Not Free Allocated Memory\n");
		        exec = 0;
				continue;
		    }
		}
//	TIME(loop_time_end);

//	printf("Loop time: %lfs\n", (double)timediff(&loop_time_start, &loop_time_end)/1000);
    }

    return 0;
}
