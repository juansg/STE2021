#!/bin/bash

ITERATIONS=10

version=$1
prefix='../../inputs/rotate_rgbyuv_workloads/libppm/Berlin_Botanischer-Garten_HB_02'

############# SEQUENTIAL 
for i in `seq 1 $ITERATIONS` ; do
    echo "Iterando $i de $ITERATIONS"
    ./a.out ifiles.txt ofiles.txt 90 1> /dev/null
    for j in 1 2 3 4 5 6 7 8 9; do
			mv ${prefix}.out${j}.ppm ${prefix}-$version-${j}.ppm
      diff -q -s ${prefix}-serial-${j}.ppm ${prefix}-${version}-${j}.ppm
    done
done

