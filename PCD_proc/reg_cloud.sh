#!/bin/bash
DAY="17.03.22"
DATA="arabidopsis_square"
mkdir -p data/$DAY/$DATA/transf
mkdir -p data/$DAY/$DATA/pics_transf
./pcl/build/cloudreg $DAY $DATA 0 100 2 >data/$DAY/icp_${DATA}_2.txt
./pcl/build/cloudreg $DAY $DATA 0 100 3 >data/$DAY/icp_${DATA}_3.txt
./pcl/build/cloudreg $DAY $DATA 0 100 4 >data/$DAY/icp_${DATA}_4.txt
