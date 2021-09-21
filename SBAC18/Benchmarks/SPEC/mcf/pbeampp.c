/**************************************************************************
PBEAMPP.C of ZIB optimizer MCF, SPEC version

This software was developed at ZIB Berlin. Maintenance and revisions
solely on responsibility of Andreas Loebel

Dr. Andreas Loebel
Ortlerweg 29b, 12207 Berlin

Konrad-Zuse-Zentrum fuer Informationstechnik Berlin (ZIB)
Scientific Computing - Optimization
Takustr. 7, 14195 Berlin-Dahlem

Copyright (c) 1998-2000 ZIB.
Copyright (c) 2000-2002 ZIB & Loebel.
Copyright (c) 2003-2005 Andreas Loebel.
**************************************************************************/
/*  LAST EDIT: Sun Nov 21 16:22:04 2004 by Andreas Loebel (boss.local.de)  */
/*  $Id: pbeampp.c,v 1.10 2005/02/17 19:42:32 bzfloebe Exp $  */
#include "pbeampp.h"
#include <sys/time.h>

#if defined(USE_BDX)
__attribute__((noinline)) int __bdx_cond__(int flag) { asm(""); return 1; }
__attribute__((noinline)) void __bdx_stage_begin__(volatile int flag) { asm(""); }
__attribute__((noinline)) void __bdx_stage_end__(void) { asm(""); }
#endif /* USE_BDX */

#if defined(USE_TLS)
#include <immintrin.h>
void _xabort2(char n __attribute((unused))) { _xabort(0xff); }
#endif /* USE_TLS */

extern double run_time_acc;
extern int global_n;

#include <sys/time.h>
#define TIME(x) gettimeofday(&x, NULL)
typedef struct timeval timer;
static double timevaldiff(timer *start, timer *finish) {
  double msec;
  msec = (finish->tv_sec - start->tv_sec) * 1.0e6;
  msec += (finish->tv_usec - start->tv_usec);
  return msec;
}

#define K 300
#define B 50


#ifdef _PROTO_
int bea_is_dual_infeasible(arc_t *arc, cost_t red_cost)
#else
int bea_is_dual_infeasible(arc, red_cost) arc_t *arc;
cost_t red_cost;
#endif
{
  return ((red_cost < 0 && arc->ident == AT_LOWER) ||
          (red_cost > 0 && arc->ident == AT_UPPER));
}

typedef struct basket {
  arc_t *a;
  cost_t cost;
  cost_t abs_cost;
} BASKET;

static long basket_size;
static BASKET basket[B + K + 1];
static BASKET *perm[B + K + 1];

#ifdef _PROTO_
void sort_basket(long min, long max)
#else
void sort_basket(min, max) long min, max;
#endif
{
  long l, r;
  cost_t cut;
  BASKET *xchange;

  l = min;
  r = max;

  cut = perm[(long)((l + r) / 2)]->abs_cost;

  do {
    while (perm[l]->abs_cost > cut)
      l++;
    while (cut > perm[r]->abs_cost)
      r--;

    if (l < r) {
      xchange = perm[l];
      perm[l] = perm[r];
      perm[r] = xchange;
    }
    if (l <= r) {
      l++;
      r--;
    }

  } while (l <= r);

  if (min < r)
    sort_basket(min, r);
  if (l < max && l <= B)
    sort_basket(l, max);
}

static long nr_group;
static long group_pos;
static long initialize = 1;

#ifdef _PROTO_
arc_t *primal_bea_mpp(long m, arc_t *arcs, arc_t *stop_arcs,
                      cost_t *red_cost_of_bea)
#else
arc_t *primal_bea_mpp(m, arcs, stop_arcs, red_cost_of_bea) long m;
arc_t *arcs;
arc_t *stop_arcs;
cost_t *red_cost_of_bea;
#endif
{
  long i, next, old_group_pos;
  arc_t *arc;
  cost_t red_cost;

  struct timeval start_time, end_time;
  double run_time;

  if (initialize) {
    for (i = 1; i < K + B + 1; i++)
      perm[i] = &(basket[i]);

    nr_group = ((m - 1) / K) + 1;
    group_pos = 0;
    basket_size = 0;
    initialize = 0;
  } else {
    for (i = 2, next = 0; i <= B && i <= basket_size; i++) {
      arc = perm[i]->a;
      red_cost = arc->cost - arc->tail->potential + arc->head->potential;

      if ((red_cost < 0 && arc->ident == AT_LOWER) ||
          (red_cost > 0 && arc->ident == AT_UPPER)) {
        next++;
        perm[next]->a = arc;
        perm[next]->cost = red_cost;
        perm[next]->abs_cost = ABS(red_cost);
      }
    }

    basket_size = next;
  }

  old_group_pos = group_pos;

NEXT:
  /// Cesar: Loop below was changed to always iterate 299 times.
  ///	   The original loop iterated usually 299 and 300 times
  ///	   over a linked list.
  {

    int cnt = 0, n = 0;
    arc = arcs + group_pos;
    for (; arc < stop_arcs; arc += nr_group)
      n++;

		// Publish n
		global_n = n;

    arc = arcs + group_pos;


  	timer loop_time_start, loop_time_end;

		TIME(loop_time_start);
#if defined(USE_TLS)
#pragma omp parallel for ordered(1) use(tls)
#endif /* USE_TLS */
#if defined(USE_BDX)
#pragma omp parallel for ordered(1) use(psbdx)
#endif /* USE_BDX */
#if defined(ORDERED)
#pragma omp parallel for ordered(1)
#endif /* ORDERED */
    for (cnt = 0; cnt < n; cnt++) {
#if defined(USE_BDX) || defined(ORDERED)
#pragma omp ordered depend(sink : cnt - 1)
#endif /* USE_BDX || ORDERED */
      int cond1 = 0; // = arc->ident > BASIC;
      int cond2 = 0;
      cost_t red_cost = 0;
      arc_t *arc2;
      arc2 = arc;
      arc += nr_group;
#if defined(USE_BDX) || defined(ORDERED)
#pragma omp ordered depend(source)
#endif /* USE_BDX || ORDERED */

      cond1 = arc2->ident > BASIC;

      if (cond1) {
        red_cost = arc2->cost - arc2->tail->potential + arc2->head->potential;
        cond2 = bea_is_dual_infeasible(arc2, red_cost);
      }

#if defined(USE_BDX) || defined(ORDERED)
#pragma omp ordered depend(sink : cnt - 1)
#endif /* USE_BDX || ORDERED */
      if (cond1 && cond2) {
        basket_size++;
        perm[basket_size]->a = arc2;
        perm[basket_size]->cost = red_cost;
        perm[basket_size]->abs_cost = ABS(red_cost);
      }

#if defined(USE_BDX) || defined(ORDERED)
#pragma omp ordered depend(source)
#endif /* USE_BDX || ORDERED */
    }
		TIME(loop_time_end);
		double t = timevaldiff(&loop_time_start, &loop_time_end);
		run_time_acc += t;
  }

  if (++group_pos == nr_group)
    group_pos = 0;

  if (basket_size < B && group_pos != old_group_pos)
    goto NEXT;

  if (basket_size == 0) {
    initialize = 1;
    *red_cost_of_bea = 0;
    return NULL;
  }

  sort_basket(1, basket_size);

  *red_cost_of_bea = perm[1]->cost;
  return (perm[1]->a);
}
