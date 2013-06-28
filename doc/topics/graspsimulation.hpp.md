Grasp Simulation {#page_rw_graspsimulation}
================

[TOC]

# Introduction # {#sec_rw_gsim_intro}

# Grasp Database Generation #

## Using GUI program ##

## Using console program ##
Console programs enable easy remote execution of large simulation batches. This is especially usefull
when one wants to do simulation on several computers in parallel.

### Script with integrated generic sampling ###
This script use the semi-uniform sampling (TODO: insert ref) and can be used for typical grippers. 
Take a look in the script and change parameters such as approach, retract and opening and closing of gripper.

> ./SimulateTaskLB --dwc=[dwc_file] --output=[out_dir] --gripper=[GS20|GS20_WIDE] --object=object --gentask=true

The above command line will continue sampling and simulating grasps until a 100000 grasps has been generated. This include
grasps that fails or are in collisions. The grasps will be located in multiple files and each file will have up to 5000 
grasps. If you want to filter and only keep the successfull grasps then it can be done with gtmerge:

> ./gtmerge --output=[out_file] --oformat=[RWTASK|UIBK|Text] --include=Success --include=ObjectSlipped --input=[input_dir]

where [input_dir] is the same as [out_dir] in the first script execution.  

### Script without sampling ###
This script simulates all grasps that has been specified in a grasp task file.

> ./SimulateTask --dwc=[dwc_file] --output=[output_task_result] --input=[input_task]



# Grasp Quality Computations #

## Wrench Space Metrics ## 
TODO: describe the wrench based measures

## Robustness Quality Measure ##
Tutorial to calculating quality on previously generated grasps (by simulation or something else)
The tutorial will be using 4 programs from RobWorkSim/src/rwsimlibs/tools:
- gtmerge
- UniformSelectGrasps
- GraspTaskQualityCalc
- SimulateTask

### Input ###
The input to the quality calculation is a set of grasps in the GraspTask rw format. If the input is
generated using random sampling and dynamic simulation then it might be good to set the target pose
to the pose after grasping. This can be done using gtmerge

> ./gtmerge --input=<input_file> --output=<output_file> --oformat=RWTASK --include=Success --include=ObjectSlipped --useGraspTarget=true
	
The include argument makes sure that only successes and objectslipped (which are also successes) are
copied. The useGraspTarget make sure that the target pose is set to the objectTgrasp pose eg. The pose
after grasping.

### Quality sampling ###
The quality estimate is based on sampling new grasps normally distributed around each grasp that
needs to be evaluated. This is done with SimulateTask.

> ./SimulateTask --perturbe=true --pertubations=100 --dwc=<dynamic_workcell_file> --input=<task_input> --output=<out_prefix> --sigma_a=8 –sigma_p=0.003

This will start 100 new simulations for each grasp in <task_input>. The output will be several files
which are prefixed with <out_prefix> and with up to 6000 grasp in each. The normal distribution in 6D
is controlled with two standard deviations, one for the angular part sigma_a in degree and one for the
positional part sigma_p in meters. Hence the example will create a normal distribution with 8 degree
angular standard deviation and 3mm positional standard deviation.

### Quality calculation ###
The files generated in the previos section are used to calculate the final quality estimate. For this
GraspTaskQualityCalcis used.

> ./GraspTaskQualityCalc --input=<input_task_file> --output=<odir>/<out_task_file> --perturbe=true --pertubations=100

The input is one of the files generated in “Quality sampling”. The nr of grasps in the output should be
1/100 of the input.

### Merging result ###
In the end we merge all results generated in “Quality calculation” using gtmerge and the directory were
the results are located.

> ./gtmerge --input=<input_dir> --output=<output_file> --oformat=RWTASK

## Other Grasp Metrics ##
 
