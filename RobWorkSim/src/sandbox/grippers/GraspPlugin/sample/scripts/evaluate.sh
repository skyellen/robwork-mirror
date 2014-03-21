#!/bin/sh

#These commands set up the Grid Environment for your job:
#PBS -N LandscapeEvaluation
#PBS -l nodes=1:ppn=2,walltime=5:00:00

# Move to the desired working directory e.g. /home/<ldap-user> or /home/<ldap-user>/my/work/directory
cd /home/awolniakowski/current/gripper-landscape
ulimit -s 80000

cp in/${GRIPPER}.grp.xml out/${SCENE}/${GRIPPER}.grp.xml

EXE=/home/awolniakowski/robwork/trunk/RobWorkSim/src/sandbox/grippers/GraspPlugin/bin/evaluate-gripper
DWC=scenes/${SCENE}/Scene.dwc.xml
TD=scenes/${SCENE}/task_hints.td.xml
GRP=out/${SCENE}/${GRIPPER}.grp.xml
OUT=out/${SCENE}/${GRIPPER}.tasks.xml

$EXE --dwc $DWC --td $TD --gripper $GRP -o $OUT -n 1000
