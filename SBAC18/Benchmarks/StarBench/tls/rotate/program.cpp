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

#include <omp.h>
#include <immintrin.h>
#include <pthread.h>

#define STRIP_SIZE 2

#define BAD_EXIT -1;
#define TIME(x) gettimeofday(&x,NULL)

typedef struct timeval timer;
using namespace std;

string srcfiles[100], destfiles[100];
string ifiles, ofiles;
unsigned int angle;
unsigned int qtdFiles;

int next_iter_commit;
char exec=1;

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

/*
*	Function: main
*	--------------
*	The program main function.
*/
int main(int argc, char* argv[]) {
    cout << p_name;

    int i,is;

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


    next_iter_commit=0;

    //cout<<"iteraciones"<< qtdFiles<<endl;

    TIME(loop_time_start);

    #pragma omp parallel for schedule(static,1)  private (i) firstprivate(srcfiles,angle,destfiles) num_threads(4) //reduction(+:n) //linear(seed:13)  //spec(n)
    for (is = 0; (is < qtdFiles) ; is+=STRIP_SIZE){
        char execute_in_htm;
        unsigned status;
        
        Retry:
		if (is!=next_iter_commit)
		{
		            execute_in_htm=1;
		            status = _xbegin();
		            if (status!=_XBEGIN_STARTED)
		                goto Retry;
		}
		else{
		            execute_in_htm=0;
		}
		
	    

	       for (i=is; (i-is<STRIP_SIZE) && (i<qtdFiles) ; i++) {

			 RotateEngine re; 

			 if (exec){
				 if (!re.init(srcfiles[i], destfiles[i], angle)) {
				 	exec=0;
					continue;
				 }

				 re.run();

				 re.finish();
			 }
	       }

		if (execute_in_htm)
		{
		         //__builtin_tsuspend ();
		         //while((*next_itr_commit)!=original_arc);
		         //__builtin_tresume ();

		         if ((next_iter_commit)!=is)
		             _xabort(0xff);

		         _xend();

		}

		(next_iter_commit)+=STRIP_SIZE;      
    }

    TIME(loop_time_end);

    cout << "Loop time: " << (double)timevaldiff(&loop_time_start, &loop_time_end)/1000 << "s" << endl;


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
