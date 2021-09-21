#!/bin/bash

export TIMEFORMAT="%3R"

ITERATIONS=10

mv times.seq times.seq.backup

############# SEQUENTIAL 
for i in `seq 1 $ITERATIONS` ; do
    tempo=$( time ( ./rgbyuv -i ifiles.txt -c 5 2>/dev/null 1>&2 ) 2>&1 )
    echo $tempo >> times.seq
    echo "Iterando SEQ $i de $ITERATIONS -> $tempo"
done

