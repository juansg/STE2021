#!/bin/bash

ITERATIONS=10

version=$1
prefix='../../inputs/rotate_rgbyuv_workloads/libppm/Berlin_Botanischer-Garten_HB_02'

############# SEQUENTIAL 
for i in `seq 1 $ITERATIONS` ; do
    echo "Iterando $i de $ITERATIONS"
    ./a.out -i ifiles.txt -c 5 1>/dev/null
		mv ycomp.ppm ycomp-${version}.ppm
		mv ucomp.ppm ucomp-${version}.ppm
		mv vcomp.ppm vcomp-${version}.ppm
		diff -q -s ycomp-serial.ppm ycomp-${version}.ppm
		diff -q -s ucomp-serial.ppm ucomp-${version}.ppm
		diff -q -s vcomp-serial.ppm vcomp-${version}.ppm
done

