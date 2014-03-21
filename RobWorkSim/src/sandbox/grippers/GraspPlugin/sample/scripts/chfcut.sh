#!/bin/sh

#These commands set up the Grid Environment for your job:
#PBS -N RotorChfCut
#PBS -l nodes=1:ppn=2,walltime=5:00:00
#PBS -t 0-9

SCENE="rotor"
GRIPPER="chamferedcut"

# Move to the desired working directory e.g. /home/<ldap-user> or /home/<ldap-user>/my/work/directory
cd /home/awolniakowski/experiments/final/
ulimit -s 80000

cp grippers/${GRIPPER}.grp.xml data/${SCENE}/${GRIPPER}_${PBS_ARRAYID}.grp.xml

EXE=/home/awolniakowski/robwork/trunk/RobWorkSim/src/sandbox/grippers/GraspPlugin/bin/evaluate-gripper
DWC=scenes/${SCENE}/Scene.dwc.xml
TD=scenes/${SCENE}/task_hints.td.xml
GRP=data/${SCENE}/${GRIPPER}_${PBS_ARRAYID}.grp.xml
OUT=data/${SCENE}/${GRIPPER}_${PBS_ARRAYID}.tasks.xml

$EXE --dwc $DWC --td $TD --gripper $GRP -o $OUT -n 50000

