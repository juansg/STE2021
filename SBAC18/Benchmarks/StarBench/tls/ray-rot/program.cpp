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

#include <omp.h>
#include <immintrin.h>

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


void _xabort2 ( char n)
{    //printf("entre 0");

	_xabort(0xff);

}



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

/*
*	Function: main
*	--------------
*	The program main function.
*/
int main(int argc, char* argv[]) {
    cout << p_name;

    int i;

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

    timer loop_time_start, loop_time_end;

    char exec=1;

    //TIME(loop_time_start);

    //cout<<"iteraciones"<<qtdFiles<<endl;

   
    #pragma omp parallel for ordered(1) use(tls,6) firstprivate(srcfiles,xres,yres,rpp,angle,destfiles) num_threads(4) //reduction(+:n) //linear(seed:13)  //spec(n)

    for (i=0; i<qtdFiles; i++) {
		RotateEngine re;
	       RayEngine ra ;

		if(exec) {
		    if (!ra.init(srcfiles[i], xres, yres, rpp)) {
		        cerr << "Raytracing Kernel Init failed!" << endl;
		        exec = 0;
				continue;
		    }

		    if(!re.init(ra.getOutputImage(), angle, destfiles[i])) {
		        cerr << "Rotation Kernel Init failed!" << endl;
				exec = 0;
				continue;
		    }
		}
//		#pragma omp ordered depend(source)

		if(exec) {
		    ra.printRaytracingState();
		    re.printRotationState();
		    ra.run();
		    re.run();
		    ra.finish();
		    re.finish();
		}

    }


    //TIME(loop_time_end);

    //cout << "Loop time: " << (double)timevaldiff(&loop_time_start, &loop_time_end)/1000 << "s" << endl;


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
