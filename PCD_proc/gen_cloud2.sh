#!/bin/bash
DAY="17.03.17"
DATA="N100_c"
for i in {0..99}
do
   ./pcl/build/cloudgen2 $DAY $DATA $i
done

