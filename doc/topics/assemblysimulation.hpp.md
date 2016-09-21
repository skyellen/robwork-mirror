Assembly Simulation {#page_rw_assemblysimulation}
================

[TOC]

# Introduction # {#sec_rw_asim_intro}
RobWork includes a framework for working with assembly operations.
This framework covers for instance peg-in-hole, hole-on-peg and screwing types of operations.

Examples of uses are:
 - Basic specification of a task, and the results of execution of the task on either real hardware or in simulation.
 - Development and testing of robust control strategies that can utilize force & torque sensors and controllers.
 - Performing experiments and learning in simulation.
 - Visualization of real or simulated results in RobWorkStudio.

# Task & Result Definition # {#sec_rw_asim_task_result}
The rwlibs::assembly library provides an abstract definition of assembly tasks, results and control strategies.
Furthermore it also includes some predefined control strategies, that can be used right away.

The rwlibs::assembly::AssemblyTask format requires only few parameters:
 - The name of the male object (for instance a peg or a screw)
 - The name of the female object (typically a hole)
 - The relative transformation of the male object with respect to the female object when assembled
 - The control strategy to use. The control strategy and parameterization is described in detail in the section \ref sec_rw_asim_task_strategy .

Furthermore there are optional parameters for context and metadata. If the task needs to be simulated, extra parameters might also be required in the
task specification.

The rwlibs::assembly::AssemblyResult format has only two mandatory values. Either the result is a success or a failure, and the final relative
transformation between the two objects must be saved. The format allows saving detailed data of the complete trajectory of the two objects. This
is especially useful for visualization, which is described in detail in the section \ref sec_rw_asim_task_visualization .

# Control Strategy Interface # {#sec_rw_asim_task_strategy}
An example of implementation of a control strategy is given in the class rwlibs::assembly::CircularPiHControlStrategy.

# Task Generation # {#sec_rw_asim_task_generation}
An example of generation of tasks that uses the CircularPiHControlStrategy can be found in:

 RobWorkSim/example/tools/src/GenerateCircularPiHTasks.cpp
 
The program can for instance be called like this:

 ./GenerateCircularPiHTasks -p UR1.Peg -h Hole -w assembly.wc.xml --hrad 0.03 --hlen 0.05 --prad 0.0294 --plen 0.08 --pegController UR1Controller --pegTCP UR1.PegTCP --pegFTSensor FTSensor

# Simulation # {#sec_rw_asim_task_simulation}
To simulate tasks, the rwsim::simulator::AssemblySimulator can be used. An example of the usage can be found in:

 RobWorkSim/example/tools/src/SimulateAssembly.cpp

The program can be called like this:

 ./SimulateAssembly -d assembly.dwc.xml -i tasks.assembly.xml -o results.assembly.xml

# Visualization # {#sec_rw_asim_task_visualization}
To visualize the results of a assembly operations, the rwslibs::ATaskVisPlugin RobWorkStudio plugin can be used.
Typically it will be the result of simulations that will be visualized this way.
On the \ref page_rw_gettingstarted page it is described how a plugin can be loaded in RobWorkStudio.
To load the ATaskVisPlugin, the RobWorkStudio.ini file should contain something similar to:

\verbatim
[Plugins]
ATaskVisPlugin\DockArea=2
ATaskVisPlugin\Filename=librws_atask
ATaskVisPlugin\Path=../../libs/debug
ATaskVisPlugin\Visible=true
\endverbatim