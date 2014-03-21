#!/bin/bash

## Usage:
# ---

# parameters map
PARAMS=(1 2 3 4 5 7 8 10 11 12 13)
NAMES=("_" "length" "width" "depth" "chfdepth" "chfangle" "_" "cutdepth" "cutangle" "_" "tcpoff" "jawdist" "opening" "force")

# extract raw quality values for each of the changed parameters
# (number of samples assumed to be 100 (1-100)
# number of parameters is 12 (1-12)

# this is to find max q values in batch
rm .max.txt
touch .max.txt

for p in "${PARAMS[@]}"
do
	echo "Extracting data for ${NAMES[${p}]}..."

	# create raw data file
	rm .raw_${p}.txt
	touch .raw_${p}.txt
	
	# extract raw data
	for s in {1..100}
	do
		RAW_SUCCESS=`cat *_${p}_${s}.grp.xml | grep "<SuccessRatio>" | sed "s/\(<SuccessRatio>\|<\/SuccessRatio>\)//g"`
		RAW_COVERAGE=`cat *_${p}_${s}.grp.xml | grep "<Coverage>" | sed "s/\(<Coverage>\|<\/Coverage>\)//g"`
		RAW_WRENCH=`cat *_${p}_${s}.grp.xml | grep "<Wrench>" | sed "s/\(<Wrench>\|<\/Wrench>\)//g"`
		RAW_TOPWRENCH=`cat *_${p}_${s}.grp.xml | grep "<TopWrench>" | sed "s/\(<TopWrench>\|<\/TopWrench>\)//g"`
		
		#echo ${s} ${RAW_SUCCESS} ${RAW_COVERAGE} ${RAW_WRENCH} ${RAW_TOPWRENCH}
		echo ${s} ${RAW_SUCCESS} ${RAW_COVERAGE} ${RAW_WRENCH} ${RAW_TOPWRENCH} >> .raw_${p}.txt
	done
	
	# find temporary max values for each quality
	TMAX_SUCCESS=`sort -g -k 2 .raw_${p}.txt | tail -n 1 | awk '{print $2}'`
	TMAX_COVERAGE=`sort -g -k 3 .raw_${p}.txt | tail -n 1 | awk '{print $3}'`
	TMAX_WRENCH=`sort -g -k 4 .raw_${p}.txt | tail -n 1 | awk '{print $4}'`
	TMAX_TOPWRENCH=`sort -g -k 5 .raw_${p}.txt | tail -n 1 | awk '{print $5}'`
	
	echo "${TMAX_SUCCESS} ${TMAX_COVERAGE} ${TMAX_WRENCH} ${TMAX_TOPWRENCH}" >> .max.txt
done

# find absolute maximum values for qualities in batch
MAX_SUCCESS=`sort -g -k 1 .max.txt | tail -n 1 | awk '{print $1}'`
MAX_COVERAGE=`sort -g -k 2 .max.txt | tail -n 1 | awk '{print $2}'`
MAX_WRENCH=`sort -g -k 3 .max.txt | tail -n 1 | awk '{print $3}'`
MAX_TOPWRENCH=`sort -g -k 4 .max.txt | tail -n 1 | awk '{print $4}'`

echo "Max values: ${MAX_SUCCESS} ${MAX_COVERAGE} ${MAX_WRENCH} ${MAX_TOPWRENCH}"
	
# do the normalization
if [ ! -d "data" ]; then
	mkdir data
fi

for p in "${PARAMS[@]}"
do
	echo "Normalizing data for ${NAMES[${p}]}..."
	
	awk -v ms=$MAX_SUCCESS -v mc=$MAX_COVERAGE -v mt=$MAX_TOPWRENCH '{print $1, $2/ms, $3/mc, $4/mt, $5/mt}' .raw_${p}.txt >  data/${NAMES[${p}]}.txt
done
