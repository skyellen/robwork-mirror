# Grasp Simulation # {#page_rw_graspsimulation}

# Introduction # {#sec_rw_gsim_intro}

# Grasp Database Generation #

## Using console program ##


### Script med integreret sampling ###
This script use the semi-uniform sampling (TODO: insert ref) and can be used for typical grippers. 
Take a look in the script 
You can 


dette script benytter den semi-uniforme sampling og kan bruges til de gribere vi har i learnbip.  Du kan evt. 
kigge i scriptet og ændre parametre sådan som approach, åbne og lukke konfiguration.

~~~{.cpp}
./SimulateTaskLB --dwc=<dwc_file> --output=<out_dir> --gripper=<GS20|GS20_WIDE> --object=object --gentask=true
~~~

Denne bliver ved med at simulere indtil 100000 greb er simuleret (inklusive kolisioner). grebne ligger i filer af 5000 greb i hver.
hvis du vil filtrere grebne og kun havde de successfulde ud så kan du filtrere og merge filerne således:

	./gtmerge --output=<out_file> --oformat=<RWTASK|UIBK|Text> --include=Success --include=ObjectSlipped --input=<input_dir>

hvor <input_dir> er der samme som <out_dir> i førse script eksekvering.


******************** Script til eksekvering af allerede genereret tasks

dette script vil eksekvere greb beskrevet i rwtask formatet

	./SimulateTask --dwc=<dwc_file> --output=<output_task_result> --input=<input_task>


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

	./gtmerge --input=<input_file> --output=<output_file> --oformat=RWTASK –include=Success –include=ObjectSlipped --useGraspTarget=true
	
The include argument makes sure that only successes and objectslipped (which are also successes) are
copied. The useGraspTarget make sure that the target pose is set to the objectTgrasp pose eg. The pose
after grasping.

### Quality sampling ###
The quality estimate is based on sampling new grasps normally distributed around each grasp that
needs to be evaluated. This is done with SimulateTask.

	./SimulateTask --perturbe=true --pertubations=100 --dwc=<dynamic_workcell_file> --input=<task_input> --output=<out_prefix> --sigma_a=8 –sigma_p=0.003

This will start 100 new simulations for each grasp in <task_input>. The output will be several files
which are prefixed with <out_prefix> and with up to 6000 grasp in each. The normal distribution in 6D
is controlled with two standard deviations, one for the angular part sigma_a in degree and one for the
positional part sigma_p in meters. Hence the example will create a normal distribution with 8 degree
angular standard deviation and 3mm positional standard deviation.

### Quality calculation ###
The files generated in the previos section are used to calculate the final quality estimate. For this
GraspTaskQualityCalcis used.

	./GraspTaskQualityCalc --input=<input_task_file> --output=<odir>/<out_task_file> --perturbe=true –pertubations=100

The input is one of the files generated in “Quality sampling”. The nr of grasps in the output should be
1/100 of the input.

### Merging result ###
In the end we merge all results generated in “Quality calculation” using gtmerge and the directory were
the results are located.

	./gtmerge --input=<input_dir> --output=<output_file> --oformat=RWTASK

## Other Grasp Metrics ##
 