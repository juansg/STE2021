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

#define BAD_EXIT -1;
#define TIME(x) gettimeofday(&x,NULL)

typedef struct timeval timer;
using namespace std;

string srcfiles[100], destfiles[100];
string ifiles, ofiles;
unsigned int angle;
unsigned int qtdFiles;


/**********************************************************************************
				FUNCTION PROTOTYPES
*************************************************************************************/
static long timevaldiff(timer* start, timer* finish);
bool parseArgs(char* argv[], int argc, unsigned int &angle, string &inname, string &outname);

/* GLOBAL VARIABLES */
string usage =  "Usage: ./rot <infile> <outfile> <angle>\n\n"
                "infile:      input file\n"
                "outfile:     output file\n"
                "angle:       angle to be rotated\n";
string p_name = "--- StarBENCH - rotate Kernel ---\n";

timer a,b;

__attribute__ ((noinline)) int __bdx_cond__(int flag) {asm(""); return 1;}
__attribute__ ((noinline)) void __bdx_stage_begin__(volatile int flag) {asm("");}
__attribute__ ((noinline)) void __bdx_stage_end__(void) {asm("");}

/*
*	Function: main
*	--------------
*	The program main function.
*/
int main(int argc, char* argv[]) {
    cout << p_name;

    if(argc != 4) {
		cerr << usage;
		return BAD_EXIT;
    }

    if (!parseArgs(argv, argc, angle, ifiles, ofiles)) {
        cerr << usage;
        return BAD_EXIT;
    }

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

	timer loop_time_start, loop_time_end;
	int exec = 1;
	int i;

	#pragma omp parallel for ordered(1) use(psbdx)
    for (i=0; i<qtdFiles; i++) {
//	TIME(loop_time_start);
		#pragma omp ordered depend(sink: i-1)
		RotateEngine *re = new RotateEngine;
		if(exec) {
		    if (!re->init(srcfiles[i], destfiles[i], angle)) {
				exec = 0;
				continue;
			}
		}
		#pragma omp ordered depend(source)

		RotateEngine *re2 = re;

		if(exec) {
		    re2->run();
		    re2->finish();
		}
//	TIME(loop_time_end);

//	cout << "Loop time: " << (double)timevaldiff(&loop_time_start, &loop_time_end)/1000 << "s" << endl;
    }

	if(exec == 0) return BAD_EXIT;


    return 0;
}

/*
*   Function: parseArgs
*   -------------------
*   Extracts the rotation angle as well as the in- and output file names
*   from the string array args, storing them in the specified variables.
*/
bool parseArgs(char* argv[], int argc, unsigned int &angle, string &inname, string &outname) {
    if (argc !=4)
        return false;

    const char *tmp = argv[3];
    angle = atoi(tmp) % 360;

    if (angle < 0) {
        cerr << "Bad arguments, exiting" << endl;
        exit(-1);
    }

    inname = argv[1];
    outname = argv[2];
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
