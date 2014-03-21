#!/bin/bash

for gripper in `ls ../in`; do qsub evaluate.sh -v SCENE=$1,GRIPPER=`echo $gripper | sed 's/.grp.xml//g'` ; done
#for gripper in `ls ../in`; do qsub evaluate.sh -v SCENE="dolt",GRIPPER=`echo $gripper | sed 's/.grp.xml//g'` ; done
#for gripper in `ls ../in`; do qsub evaluate.sh -v SCENE="rotor",GRIPPER=`echo $gripper | sed 's/.grp.xml//g'` ; done

