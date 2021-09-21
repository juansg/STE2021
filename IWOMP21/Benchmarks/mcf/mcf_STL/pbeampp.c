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


#include <omp.h>
#include <immintrin.h>
#include <sys/time.h>


#define K 300
#define B  50



#include "pbeampp.h"


#define STRIP_SIZE 75


#ifdef _PROTO_
int bea_is_dual_infeasible( arc_t *arc, cost_t red_cost )
#else
int bea_is_dual_infeasible( arc, red_cost )
    arc_t *arc;
    cost_t red_cost;
#endif
{
    return(    (red_cost < 0 && arc->ident == AT_LOWER)
            || (red_cost > 0 && arc->ident == AT_UPPER) );
}







typedef struct basket
{
    arc_t *a;
    cost_t cost;
    cost_t abs_cost;
} BASKET;

static long basket_size;
static BASKET basket[B+K+1];
static BASKET *perm[B+K+1];



#ifdef _PROTO_
void sort_basket( long min, long max )
#else
void sort_basket( min, max )
    long min, max;
#endif
{
    long l, r;
    cost_t cut;
    BASKET *xchange;

    l = min; r = max;

    cut = perm[ (long)( (l+r) / 2 ) ]->abs_cost;

    do
    {
        while( perm[l]->abs_cost > cut )
            l++;
        while( cut > perm[r]->abs_cost )
            r--;
            
        if( l < r )
        {
            xchange = perm[l];
            perm[l] = perm[r];
            perm[r] = xchange;
        }
        if( l <= r )
        {
            l++; r--;
        }

    }
    while( l <= r );

    if( min < r )
        sort_basket( min, r );
    if( l < max && l <= B )
        sort_basket( l, max ); 
}






static long nr_group;
static long group_pos;


static long initialize = 1;

//long long count1 =0;
//long long count2=0;
//long long count3=0;

double tiempo_promedio=0;

arc_t * next_iter_commit; // __attribute__((aligned(64)));


#ifdef _PROTO_
arc_t *primal_bea_mpp( long m,  arc_t *arcs, arc_t *stop_arcs, 
                              cost_t *red_cost_of_bea )
#else
arc_t *primal_bea_mpp( m, arcs, stop_arcs, red_cost_of_bea )
    long m;
    arc_t *arcs;
    arc_t *stop_arcs;
    cost_t *red_cost_of_bea;
#endif
{
    long i, next, old_group_pos;
    arc_t *arc,*arc_strip,*arc2;
    cost_t red_cost;
    
    struct timeval start_time, end_time;
    
    //count1++;

    if( initialize )
    {
        for( i=1; i < K+B+1; i++ )
            perm[i] = &(basket[i]);
        nr_group = ( (m-1) / K ) + 1;
        group_pos = 0;
        basket_size = 0;
        initialize = 0;
    }
    else
    {
        for( i = 2, next = 0; i <= B && i <= basket_size; i++ )
        {
            arc = perm[i]->a;
            red_cost = arc->cost - arc->tail->potential + arc->head->potential;
            if( (red_cost < 0 && arc->ident == AT_LOWER)
                || (red_cost > 0 && arc->ident == AT_UPPER) )
            {
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
    /* price next group */
        

    arc2 = arcs + group_pos;
    next_iter_commit=arc2;


    #pragma omp parallel num_threads(4)
    {
    #pragma omp single
    {
          gettimeofday(&start_time, NULL);
    #pragma omp taskloop grainsize(1) default(none)  firstprivate(stop_arcs,arc2) shared(next_iter_commit,basket_size,nr_group,perm)  private(red_cost,arc) 
    for(arc_strip =arc2  ; arc_strip < stop_arcs; arc_strip += nr_group*STRIP_SIZE)
    {
        //count2++;
        char execute_in_htm;
        unsigned status;
        long basket_sizeL;
        char flag;

	 Retry:
	 if ((next_iter_commit)!=arc_strip)
	 {
		         execute_in_htm=1;
		         status = _xbegin();
		         if (status!=_XBEGIN_STARTED)
		             goto Retry;
        } 
        else{
		         execute_in_htm=0;
	 }
    
       flag=0;
       for (arc=arc_strip; ((arc-arc_strip)< STRIP_SIZE*nr_group) && (arc < stop_arcs); arc+=nr_group){
		 
		 if( arc->ident > BASIC )
		 {
		     /* red_cost = bea_compute_red_cost( arc ); */
		     red_cost = arc->cost - arc->tail->potential + arc->head->potential;
		     if( bea_is_dual_infeasible( arc, red_cost ) )
		     {   //count3++;
		         if(!flag){
                           flag=1;
                           basket_sizeL=basket_size;
                       }
                       basket_sizeL++;
		         perm[basket_sizeL]->a = arc;
		         perm[basket_sizeL]->cost = red_cost;
		         perm[basket_sizeL]->abs_cost = ABS(red_cost);
		     }
		 }
       } 

      if (execute_in_htm)
      {
                //__builtin_tsuspend ();
                //while((*next_itr_commit)!=original_arc);
                //__builtin_tresume ();

                if ((next_iter_commit)!=arc_strip)
                    _xabort(0xff);

                _xend();

      }

      if (flag) basket_size=basket_sizeL;  
      next_iter_commit+=nr_group*STRIP_SIZE;
    }
    
    gettimeofday(&end_time, NULL);
    }
    }
    double run_time = ((end_time.tv_sec  - start_time.tv_sec) * 1000000u + end_time.tv_usec - start_time.tv_usec) / 1.e6;
    //printf("Timer Loop %lf\n",run_time);  
    tiempo_promedio+=run_time;
    printf("Tiempo promedio %lf\n",tiempo_promedio); 
    
    //printf("count1 %lld\n",count1);
    
    //printf("count2 %lld\n",count2);
    
    //printf("count3 %lld\n",count3);


    if( ++group_pos == nr_group )
        group_pos = 0;

    if( basket_size < B && group_pos != old_group_pos )
        goto NEXT;

    if( basket_size == 0 )
    {
        initialize = 1;
        *red_cost_of_bea = 0; 
        return NULL;
    }
    
    sort_basket( 1, basket_size );
    *red_cost_of_bea = perm[1]->cost;
    return( perm[1]->a );
}










