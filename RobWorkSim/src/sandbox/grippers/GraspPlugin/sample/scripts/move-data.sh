#!/bin/bash

SCENES=("rotor" "dolt" "cyl")

# remove eroor files
for d in ${SCENES[@]}; do
	rm $d/*.e*
done

# move data
for i in ${SCENES[@]}
do
  echo "Moving data from ${i} experiment..."
  mkdir ../data/${i}/out
  mv ${i}/*.o* ../data/${i}/out
done
