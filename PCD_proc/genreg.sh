#!/bin/bash
DATA="N100_c"
mkdir -p data/17.03.17/$DATA/transf
mkdir -p data/17.03.17/$DATA/pics_transf
mkdir -p data/17.03.17/$DATA/pcd
mkdir -p data/17.03.17/$DATA/pics_pcd
for i in {0..99}
do
   ./build/cloudgen $DATA $i
done
./build/cloudreg $DATA 0 100 1


