#!/bin/bash
DAY="17.03.22"
DATA="arabidopsis_square"
mkdir -p data/$DAY/$DATA/pcd
mkdir -p data/$DAY/$DATA/pics_pcd
for i in {0..99}
do
   ./build/cloudgen $DAY $DATA $i
done

