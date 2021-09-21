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


/**********************************************************************************
				INCLUDES & DEFINES
*************************************************************************************/
#include <sys/time.h>
#include <stdlib.h>
#include "rotation_engine.h"
#include "ray_engine.h"
#include "bdx.h"

#define BAD_EXIT -1;
#define TIME(x) gettimeofday(&x,NULL)

typedef struct timeval timer;
using namespace std;

string srcfiles[100], destfiles[100];
string ifiles, ofiles;
unsigned int angle;
unsigned int xres = 1024; 
unsigned int yres = 768;
unsigned int rpp = 1;
int qtdFiles = 0;

int bdx_t1_id = 1;
int bdx_t2_id = 2;
int bdx_t3_id = 3;
int bdx_t4_id = 4;


/**********************************************************************************
				FUNCTION PROTOTYPES
*************************************************************************************/
static long timevaldiff(timer* start, timer* finish);
string* convertToString(char **in, size_t size);
bool parseArgs(string* args, unsigned int &angle, unsigned int& xres, unsigned int& yres, unsigned int& rpp, string &inname, string &outname);

/* GLOBAL VARIABLES */
string usage =  "Usage: ./ray-rot <infile> <outfile> <angle> <xres> <yres> <RPP>\n\n"
                "infile:      input file\n"
                "outfile:     output file\n"
                "angle:       angle to be rotated\n"
                "xres:        horizontal resolution\n"
                "yres:        vertical resolution\n"
                "RPP:        rays shot per pixel\n";

string p_name = "--- StarBENCH - ray-rot Workload ---\n";

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
	RotateEngine* bdx_buffer_rot[BDX_BATCH_SIZE];
	RayEngine* bdx_buffer_ray[BDX_BATCH_SIZE];
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
            int bdx_index5 = 0;

            /** Begin Stage 1 **/
            while (bdx_stage_flag[0].value != MyId);

            register int local_i = bdx_i;

            while (bdx_index1 < BDX_BATCH_SIZE && local_i < qtdFiles) {
                bdx_buffer_rot[bdx_index1] = new RotateEngine;
                bdx_buffer_ray[bdx_index1] = new RayEngine;

                if (!bdx_buffer_ray[bdx_index1]->init(srcfiles[local_i], xres, yres, rpp)) {
                    cerr << "Raytracing Kernel Init failed!" << endl;
                    exit(1);
                }

                if (!bdx_buffer_rot[bdx_index1]->init(bdx_buffer_ray[bdx_index1]->getOutputImage(), angle, destfiles[local_i])) {
                    cerr << "Rotation Kernel Init failed!" << endl;
                    exit(1);
                }

                local_i++;
                bdx_index1++;
            }
            stop = (local_i == qtdFiles);
            bdx_i = local_i;

            bdx_stage_flag[0].value = (bdx_stage_flag[0].value + 1) % bdx_NumThreads;
            /** End of Stage 1 **/

            /** Begin Stage 2, 3, 4 and 5 **/

            while (bdx_index2 < bdx_index1) {
                bdx_buffer_ray[bdx_index2]->printRaytracingState();
                bdx_buffer_rot[bdx_index2]->printRotationState();

                bdx_buffer_ray[bdx_index2]->run();
                bdx_buffer_rot[bdx_index2]->run();

                bdx_buffer_ray[bdx_index2]->finish();
                bdx_buffer_rot[bdx_index2]->finish();

                free(bdx_buffer_ray[bdx_index2]);
                free(bdx_buffer_rot[bdx_index2]);

                bdx_index2++;
            }

            /** End of Stage 2, 3, 4 and 5 **/


            /** Have reached the end of all iterations? **/
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
	bdx_live[4].value = 0;

    bdx_stage_flag[0].value = 0;
    bdx_stage_flag[1].value = 0;
    bdx_stage_flag[2].value = 0;
    bdx_stage_flag[3].value = 0;
    bdx_stage_flag[4].value = 0;


	/* Create BDX threads */
	pthread_create(&bdx_thread2, NULL, BDX_Thread_Source, &bdx_t1_id);
	pthread_create(&bdx_thread3, NULL, BDX_Thread_Source, &bdx_t2_id);
	pthread_create(&bdx_thread4, NULL, BDX_Thread_Source, &bdx_t3_id);
	pthread_create(&bdx_thread5, NULL, BDX_Thread_Source, &bdx_t4_id);
}

void bdx_epilogue() {
	bdx_live[0].value = 2;
	bdx_live[1].value = 2;
	bdx_live[2].value = 2;
	bdx_live[3].value = 2;
	bdx_live[4].value = 2;

	pthread_join(bdx_thread2, NULL);
	pthread_join(bdx_thread3, NULL);
	pthread_join(bdx_thread4, NULL);
	pthread_join(bdx_thread5, NULL);

	printf("All BDX thread finished successfully.\n");
}



/*
*	Function: main
*	--------------
*	The program main function.
*/
int main(int argc, char* argv[]) {
    cout << p_name;

    if(argc != 7) {
		cerr << usage;
		return BAD_EXIT;
    }


    string *args = convertToString(argv, argc);
    if(!parseArgs(args, angle, xres, yres, rpp, ifiles, ofiles)) {
        cerr << usage;
        return BAD_EXIT;
    }
	delete [] args;

    FILE* ifp = fopen(ifiles.c_str(), "r");
    FILE* ofp = fopen(ofiles.c_str(), "r");
    char iname[256], oname[256], *ptr;

    while(fscanf(ifp, " %s", iname) != EOF) {
        int tmp = fscanf(ofp, " %s", oname);

        srcfiles[qtdFiles] = iname;
        destfiles[qtdFiles] = oname;
        qtdFiles++;
        printf("%s and %s\n", iname, oname);
    }

    fclose(ifp);
    fclose(ofp);
  
    bdx_prologue();

    int MyId = 0;
	RotateEngine* bdx_buffer_rot[BDX_BATCH_SIZE];
	RayEngine* bdx_buffer_ray[BDX_BATCH_SIZE];

    bdx_live[0].value = 1;
    bdx_live[1].value = 1;
    bdx_live[2].value = 1;
    bdx_live[3].value = 1;
    bdx_live[4].value = 1;

    while (1) {
        int stop = 0;
        int bdx_index1 = 0;
        int bdx_index2 = 0;
        int bdx_index3 = 0;
        int bdx_index4 = 0;
        int bdx_index5 = 0;

        /** Begin Stage 1 **/
		while (bdx_stage_flag[0].value != 0);

		register int local_i = bdx_i;

        while (bdx_index1 < BDX_BATCH_SIZE && local_i < qtdFiles) {
            bdx_buffer_rot[bdx_index1] = new RotateEngine;
            bdx_buffer_ray[bdx_index1] = new RayEngine;

            if (!bdx_buffer_ray[bdx_index1]->init(srcfiles[local_i], xres, yres, rpp)) {
                cerr << "Raytracing Kernel Init failed!" << endl;
                return BAD_EXIT;
            }

            if (!bdx_buffer_rot[bdx_index1]->init(bdx_buffer_ray[bdx_index1]->getOutputImage(), angle, destfiles[local_i])) {
                cerr << "Rotation Kernel Init failed!" << endl;
                return BAD_EXIT;
            }

            local_i++;
            bdx_index1++;
        }
        stop = (local_i == qtdFiles);
        bdx_i = local_i;

        bdx_stage_flag[0].value = (bdx_stage_flag[0].value + 1) % bdx_NumThreads;
        /** End of Stage 1 **/


        /** Begin Stage 2, 3, 4 and 5 **/
        while (bdx_index2 < bdx_index1) {
            bdx_buffer_ray[bdx_index2]->printRaytracingState();
            bdx_buffer_rot[bdx_index2]->printRotationState();

            bdx_buffer_ray[bdx_index2]->run();
            bdx_buffer_rot[bdx_index2]->run();

            bdx_buffer_ray[bdx_index2]->finish();
            bdx_buffer_rot[bdx_index2]->finish();

            free(bdx_buffer_ray[bdx_index2]);
            free(bdx_buffer_rot[bdx_index2]);

            bdx_index2++;
        }
        /** End of Stage 2, 3, 4 and 5 **/


        /** Have reached the end of all iterations? **/
		if (stop) { break; }
    }

	/* Wait confirmation that they have finished. */
	while (bdx_live[1].value != 0 || bdx_live[2].value != 0 || bdx_live[3].value != 0 || bdx_live[4].value != 0);

    bdx_epilogue();

    return 0;
}

/*
*   Function: convertToString
*   -------------------------
*   Converts the c-string program arguments into c++-strings and returns
*   a pointer to an array of such strings.
*/
string* convertToString(char** in, size_t size) {
    string* args = new string[size];
    for(size_t i = 0; i < size; i++) {
       args[i] = in[i];
    }
    return args;
}

/*
*   Function: parseArgs
*   -------------------
*   Extracts the rotation angle as well as the in- and output file names
*   from the string array args, storing them in the specified variables.
*/
bool parseArgs(string* args, unsigned int &angle, unsigned int& xres, unsigned int& yres, unsigned int& rpp, string &inname, string &outname) {
    const char *tmp = args[3].c_str();

    angle = atoi(tmp) % 360;
    xres = atoi(args[4].c_str());
    yres = atoi(args[5].c_str());
    rpp = atoi(args[6].c_str());

    if (angle < 0 || xres <= 0 || yres <= 0 || rpp < 1) {
        cerr << "Bad arguments, exiting" << endl;
        exit(-1);
    }

    inname = args[1];
    outname = args[2];
    return true;
}

/*
*   Function: timevaldiff
*   ---------------------
*   Provides a millisecond-resolution timer, computing the elapsed time
*   in between the two given timeval structures.
*/
static long timevaldiff(timer* start, timer* finish){
	long msec;
	msec = (finish->tv_sec - start->tv_sec)*1000;
	msec += (finish->tv_usec - start->tv_usec)/1000;
	return msec;
}
