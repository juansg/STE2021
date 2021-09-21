#!/bin/bash

ITERATIONS=10

version=$1
prefix='../../inputs/c-ray/scene_out'

############# SEQUENTIAL 
for i in `seq 1 $ITERATIONS` ; do
    echo "Iterando $i de $ITERATIONS"
    ./a.out ifiles.txt ofiles.txt 90 200 200 5 1>/dev/null
    for j in $(seq -w 1 24); do
			mv ${prefix}.${j} ${prefix}-$version-${j}
      diff -q -s ${prefix}-serial-${j} ${prefix}-${version}-${j}
    done
done

