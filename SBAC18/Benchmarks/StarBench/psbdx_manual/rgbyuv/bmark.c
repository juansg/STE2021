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
#include <string.h>

#include "rgbyuv.h"
#include "imgio.h"
#include "bdx.h"

typedef struct timeval timer;

#define TIME(x) gettimeofday(&x, NULL)

char srcfiles[100][100];
int qtdFiles = 0;

int bdx_t1_id = 1;
int bdx_t2_id = 2;
int bdx_t3_id = 3;

/*
*   This program is a benchmark kernel written as part of the StarBench benchmark suite.
*   It is a fully parallelized RGB to YUV color converter, processing .ppm format pictures.
*   This application is a common one for embedded domain multiprocessors such as DSPs.
*/

/* Function declarations */
int initialize(rgbyuv_args_t* a, char* str);
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
    if(a->in_img == NULL || a->pY == NULL || a->pU == NULL || a->pV == NULL ) {
        fprintf(stderr, "Problems reading from %s\n", infile);
        return -1;
    }

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
*   Function: statsme
*   -----------------
*   Print out input file which will be read from, thread count, and specified iterations.
*/
void statsme() {
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

    // Y component
    fp = fopen(targets[0], "w");
    fprintf(fp, "%s", planeheader);
    fwrite(pY, sizeof(uint8_t), pels, fp);
    fclose(fp);

    // For U component
    fp = fopen(targets[1], "w");

    if (fp == NULL)
        return -1;

    fprintf(fp, "%s", planeheader);
    fwrite(pU, sizeof(uint8_t), pels, fp);
    fclose(fp);

    // For V component
    fp = fopen(targets[2], "w");

    if (fp == NULL)
        return -1;

    fprintf(fp, "%s", planeheader);
    fwrite(pV, sizeof(uint8_t), pels, fp);
    fclose(fp);

    return 0;
}


int stick_this_thread_to_core(int core_id) {
#if ENABLE_THREAD_AFFINITY == 1
	int num_cores = sysconf(_SC_NPROCESSORS_ONLN);

	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(core_id, &cpuset);

	pthread_t current_thread = pthread_self();

	return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
#else
	return -1;
#endif
}

void* BDX_Thread_Source(void* _param) {
	rgbyuv_args_t bdx_buffer1[BDX_BATCH_SIZE];
	int MyId = *(int *) _param;

	//////////////////////////////////////////
	// mantem a thread do consumidor viva para
	// não precisa ficar recriando-a.
	//
	// bswp_live_flag==0, aguardando.
	// bswp_live_flag==1, itera uma vez.
	// bswp_live_flag==2, encerra thread.
	while (1) {
		while (bdx_live[MyId].value == 0);
		if (bdx_live[MyId].value == 2) break;

		///////////////////////////////////////////////////////////////////////
		// controla uma execução do loop original,
		// na forma de batch
		while (1) {
		    int stop = 0;
			int bdx_index1 = 0; 
			int bdx_index2 = 0;
			int bdx_index3 = 0;
			int bdx_index4 = 0;

           /** Begin Stage 1 **/
            while (bdx_stage_flag[0].value != MyId);

            register int local_i = bdx_i;

            while (bdx_index1 < BDX_BATCH_SIZE && local_i < qtdFiles) {
                if (initialize(&bdx_buffer1[bdx_index1], srcfiles[local_i])) {
                    fprintf(stderr, "Could Not Initialize Kernel Data, for file %s\n", srcfiles[local_i]);
                    return NULL;
                }

                local_i++;
                bdx_index1++;
            }
            stop = (local_i == qtdFiles);
            bdx_i = local_i;

            bdx_stage_flag[0].value = (bdx_stage_flag[0].value + 1) % bdx_NumThreads;
            /** End of Stage 1 **/


            /** Begin Stage 2, 3 and 4 **/

            while (bdx_index2 < bdx_index1) {
                processImage(&bdx_buffer1[bdx_index2]);

                writeComponents(&bdx_buffer1[bdx_index2]);

                if (finalize(&bdx_buffer1[bdx_index2])) {
                    fprintf(stderr, "Could Not Free Allocated Memory\n");
                    return NULL;
                }

                bdx_index2++;
            }

            /** End of Stage 2, 3 and 4 **/



            ///////////////////////////////////////////////////////////////////////
			// verifica se deve parar todas iteracoes
			if (stop) { bdx_live[MyId].value = 0; break; }
		}
	}

	return NULL;
}

void bdx_prologue() {
	/* Stick this thread to core */
	stick_this_thread_to_core(0);

	/* These barriers are used to synchronize the iteration of the loops. */
	bdx_live[0].value = 0;
	bdx_live[1].value = 0;
	bdx_live[2].value = 0;
	bdx_live[3].value = 0;

    bdx_stage_flag[0].value = 0;
    bdx_stage_flag[1].value = 0;
    bdx_stage_flag[2].value = 0;
    bdx_stage_flag[3].value = 0;

	/* Create BDX threads */
	pthread_create(&bdx_thread2, NULL, BDX_Thread_Source, &bdx_t1_id);
	pthread_create(&bdx_thread3, NULL, BDX_Thread_Source, &bdx_t2_id);
	pthread_create(&bdx_thread4, NULL, BDX_Thread_Source, &bdx_t3_id);
}

void bdx_epilogue() {
	bdx_live[0].value = 2;
	bdx_live[1].value = 2;
	bdx_live[2].value = 2;
	bdx_live[3].value = 2;

	pthread_join(bdx_thread2, NULL);
	pthread_join(bdx_thread3, NULL);
	pthread_join(bdx_thread4, NULL);

	printf("All BDX thread finished successfully.\n");
}



/** MAIN */
int main(int argc, char** argv) {

    int opt;
    extern char* optarg;
    extern int optind;
    char* infiles;

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

    bdx_prologue();

	rgbyuv_args_t bdx_buffer1[BDX_BATCH_SIZE];

    bdx_live[0].value = 1;
    bdx_live[1].value = 1;
    bdx_live[2].value = 1;
    bdx_live[3].value = 1;

    while (1) {
        int stop = 0;
        int bdx_index1 = 0;
        int bdx_index2 = 0;
        int bdx_index3 = 0;
        int bdx_index4 = 0;

        /** Begin Stage 1 **/
		while (bdx_stage_flag[0].value != 0);

		register int local_i = bdx_i;

        while (bdx_index1 < BDX_BATCH_SIZE && local_i < qtdFiles) {
            if (initialize(&bdx_buffer1[bdx_index1], srcfiles[local_i])) {
                fprintf(stderr, "Could Not Initialize Kernel Data, for file %s\n", srcfiles[local_i]);
                return -1;
            }

            local_i++;
            bdx_index1++;
        }
        stop = (local_i == qtdFiles);
        bdx_i = local_i;

        bdx_stage_flag[0].value = (bdx_stage_flag[0].value + 1) % bdx_NumThreads;
        /** End of Stage 1 **/


        /** Begin Stage 2, 3 and 4 **/

        while (bdx_index2 < bdx_index1) {
            processImage(&bdx_buffer1[bdx_index2]);

            writeComponents(&bdx_buffer1[bdx_index2]);

            if (finalize(&bdx_buffer1[bdx_index2])) {
                fprintf(stderr, "Could Not Free Allocated Memory\n");
                return -1;
            }

            bdx_index2++;
        }

        /** End of Stage 2, 3 and 4 **/


        /** Have reached the end of all iterations? **/
		if (stop) { break; }
    }

	/* Wait confirmation that they have finished. */
	while (bdx_live[1].value != 0 || bdx_live[2].value != 0 || bdx_live[3].value != 0);

    bdx_epilogue();

    return 0;
}
