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
#include "benchmark_engine.h"
#include "convert_engine.h"
#include "rotation_engine.h"
#include <stdlib.h>
#include <sys/time.h>

#include <omp.h>
#include <ompuseclause.h>

#define BAD_EXIT -1;
#define TIME(x) gettimeofday(&x, NULL)

typedef struct timeval timer;
using namespace std;
string srcfiles[100], destfiles[100];
string ifiles, ofiles;
unsigned int angle;
unsigned int qtdFiles;

/**********************************************************************************
                                FUNCTION PROTOTYPES
*************************************************************************************/
static double timevaldiff(timer *start, timer *finish);
string *convertToString(char **in, size_t size);
bool parseArgs(char *args[], int argc, unsigned int &angle, string &inname,
               string &outname);

/* GLOBAL VARIABLES */
string usage = "Usage: ./rot-cc <infile> <outfile> <angle>\n\n"
               "infile:      input file\n"
               "outfile:     output file\n"
               "angle:       angle to be rotated\n";
string p_name = "--- StarBENCH - rot-cc Workload ---\n";

timer a, b, c, d;

/*
*	Function: main
*	--------------
*	The program main function.
*/
int main(int argc, char *argv[]) {
  cout << p_name;

  struct timeval start_time, end_time;
  double run_time;

  if (argc != 4) {
    cerr << usage;
    return BAD_EXIT;
  }

  if (!parseArgs(argv, argc, angle, ifiles, ofiles)) {
    cerr << usage;
    return BAD_EXIT;
  }

  FILE *ifp = fopen(ifiles.c_str(), "r");
  FILE *ofp = fopen(ofiles.c_str(), "r");
  char iname[256], oname[256], *ptr;

  while (fscanf(ifp, " %s", iname) != EOF) {
    int tmp = fscanf(ofp, " %s", oname);

    srcfiles[qtdFiles] = iname;
    destfiles[qtdFiles] = oname;
    qtdFiles++;

    printf("%s and %s\n", iname, oname);
  }

  fclose(ifp);
  fclose(ofp);

  int exec = 1;
  int i;
  timer loop_time_start, loop_time_end;

	TIME(loop_time_start);
#if defined(__USE_TLS__)
#pragma omp parallel for ordered(1) use(tls,2) schedule(static, 1) \
	private(i) firstprivate(srcfiles,angle,destfiles) num_threads(4) 
#endif /* __USE_TLS__ */
#if defined(__USE_BDX__)
#pragma omp parallel for ordered(1) use(psbdx)
#endif /* __USE_BDX__ */
#if defined(__ORDERED__)
#pragma omp parallel for ordered(1)
#endif /* __ORDERED__ */
  for (i = 0; i < qtdFiles; i++) {
#if defined(__USE_BDX__) || defined(__ORDERED__)
#pragma omp ordered depend(sink : i - 1)
#endif /* __USE_BDX__ || __ORDERED__ */
    BenchmarkEngine *be = new BenchmarkEngine;

    if (exec) {
      if (!be->init(srcfiles[i], destfiles[i], angle)) {
        exec = 0;
        continue;
      }
    }
#if defined(__USE_BDX__) || defined(__ORDERED__)
#pragma omp ordered depend(source)
#endif /* __USE_BDX__ || __ORDERED__ */
    if (exec) {
      be->run();
      be->finish();
    }
  }
	TIME(loop_time_end);
	double t = timevaldiff(&loop_time_start, &loop_time_end);
	fprintf(stderr, "Total execution time: %lf (s)\n", t/1.0e3);

  if (!exec)
    return BAD_EXIT;

  return 0;
}

/*
*   Function: convertToString
*   -------------------------
*   Converts the c-string program arguments into c++-strings and returns
*   a pointer to an array of such strings.
*/
string *convertToString(char **in, size_t size) {
  string *args = new string[size];
  for (size_t i = 0; i < size; i++) {
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
bool parseArgs(char *args[], int argc, unsigned int &angle, string &inname,
               string &outname) {
  const char *tmp = args[3];
  angle = atoi(tmp) % 360;

  if (angle < 0) {
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
static double timevaldiff(timer *start, timer *finish) {
  double msec;
  msec = (finish->tv_sec - start->tv_sec) * 1.0e3;
  msec += (finish->tv_usec - start->tv_usec) / 1.0e3;
  return msec;
}
