#!/bin/bash

FILES_EXT="out sol warn.log lp log"

dt=$(date '+%d%m%Y%H%M%S');
FOLDER="results-experiment_$dt"

mkdir $FOLDER

for EXT in $FILES_EXT; do
    mv *.$EXT $FOLDER -u -n
done