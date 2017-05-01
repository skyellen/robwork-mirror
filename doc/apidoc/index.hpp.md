RobWork   {#mainpage}
========
		
[TOC]

# Introduction # {#sec_index_intro}
*RobWork* is a collection of C++ libraries for simulation and control
of robot systems. RobWork is used for research and education as well
as for practical robot applications. Features of the library include:

- Kinematic modeling of various types of industrial manipulators, serial-, tree-, and parallel structures.
- General path-planning, path optimization, collision detection and inverse kinematics algorithms.
- Both kinematic and dynamic simulation of manipulators, controllers and sensor.
- Plugins for grasp simulation of suction cups, parallel- and dexterous grippers.
- Simple and extendible GUI and plugin system for integrating user algorithms and visualizations
- A script interface based on SWIG which extends RobWork to multiple script languages such as Python, Lua, Java...

Besides the core part, RobWork has a number of add-ons including
- RobWorkStudio which provides a graphical user interface
- RobWorkSim which is a simulator.
- RobWorkHardware which is a collection of hardware drivers that connect into RobWork

Target audience of RobWork is:
- Researchers who needs a common framework for experimental robotics
- Students who wants to experiment with concepts of robotics
- Implementers of robot applications

RobWork is developed by <a
href="http://www.sdu.dk/en/Om_SDU/Institutter_centre/SDURobotics">SDURobotics</a> at the <a href="http://www.sdu.dk">University of Southern
Denmark</a>. The focus is on cognitive and applied robotics.

# Manuals and tutorials # {#sec_index_manual}
- [Getting Started](@ref page_rw_gettingstarted)
- [Installation (Ubuntu)](@ref page_rw_installation_ubuntu)
- [Installation (CentOS)](@ref page_rw_installation_centos)
- [Installation (Windows)](@ref page_rw_installation_windows)
- [Manual (C++)](@ref page_rw_manual)
- [Tutorials](@ref page_rw_tutorials)

Other topics include
- [Motion Planning](@ref page_rw_motionplanning)
- [Online Control](@ref page_rw_onlinecontrol)
- [Grasp Simulation](@ref page_rw_graspsimulation)
- [Assembly Simulation](@ref page_rw_assemblysimulation)
- [Plugins](@ref page_rw_plugins)
- [Script Interface](@ref page_rw_scriptinterface)
- [Codeguidelines](@ref page_coding_guidelines)
- [Softbody Simulation](@ref page_rw_softbodysimulation)

# Download # {#sec_index_download}
- *RobWork* can be fetched from the SVN repository <a href="https://svnsrv.sdu.dk/svn/RobWork/trunk">https://svnsrv.sdu.dk/svn/RobWork/trunk</a>

Use the following credentials:


- Username: 'Guest'
- Password: ''

# Installation # {#sec_index_installation}
- Installation instructions are included in \ref page_rw_installation_ubuntu, \ref page_rw_installation_centos and \ref page_rw_installation_windows. 
An old and deprecated installation guide can still be found at \ref page_rw_installation. 

# License # {#sec_index_license}
RobWork is distributed under the "Apache License, Version 2.0", [See here](@ref rw_license). For convenience, a number of
open-source libraries are distributed together with RobWork; the
RobWork license does not apply to these libraries.
<!-- Adding a list of these libraries with links would be nice -->

# References #
<!-- Here we should add reference papers for RobWorkSim and RobWorkHardware - since these were not part of the initial release -->
If you use RobWork in your research then please support us by referencing to one of the following papers:

    @ARTICLE{5756879,
     author={Ellekilde, Lars-Peter and Jorgensen, Jimmy A.},
     journal={Robotics (ISR), 2010 41st International Symposium on and 2010 6th German Conference on Robotics (ROBOTIK)},
     title={RobWork: A Flexible Toolbox for Robotics Research and Education},
     year={2010},
     month={june},
     pages={1 -7},
    } 


    @inproceedings{joergensen10,
     author = {J{\o}rgensen, Jimmy A. and Ellekilde, Lars-Peter and Petersen, Henrik G.},  
     title = {{RobWorkSim - an Open Simulator for Sensor based Grasping - Conference papers - VDE Publishing House}},
     booktitle = {ISR/ROBOTIK 2010 - ISR 2010 (41st International Symposium on Robotics) and ROBOTIK 2010 (6th German Conference on Robotics)},
     citeulike-article-id = {8807064},
     citeulike-linkout-0 = {http://www.vde-verlag.de/proceedings-en/453273198.html},
     day = {7-9},
     month = jun,
     posted-at = {2011-02-11 12:00:13},
     priority = {2},
     publisher = {VDE-Verlag},
     url = {http://www.vde-verlag.de/proceedings-en/453273198.html},
     year = {2010}
    }

# Publications #
<!-- This section should be updated or simply refer to a google scholar/web of science page --> 
Development of RobWork started summer 2006. The dynamic simulation was added in 2008 and was and still is based on the Open Dynamics Engine.
The framework has been and is currently used by SDU in several large national and European projects: LearnBip, IntellAct, Xperience, Tailorcrete, MoveBots, Handyman,
RobWork is being used and developed in cooperation with industrial partners such as Scape Technologies and the Danish Technological Institute.

- At SDU RobWork is used in the mobile robotics department for kinematic simulation and visualization of mobile robot
- in [1] RobWork was used to simulate the dynamic movements of a iron bar, including verification of simulation to reality
- In [2] RobWork was used as grasp simulator where the use of tactile sensors in quality metrics was investigated
- in [3] an early RobWork was used to investigate reactive grasping using tactile array sensors
- in [4] RobWork is used for kinematic calculations and visualization
- and other works in which it was used [5-13]


[1] **Simulating Robot Handling of Large Scale Deformable Objects : Manufacturing of Unique Concrete Reinforcement Structures.** / Cortsen, Jens; Jørgensen, Jimmy Alison; Sølvason, Dorthe; Petersen, Henrik Gordon .I : I E E E International Conference on Robotics and Automation , 2012.

[2] **Grasp Synthesis for Dextrous Hands Optimised for Tactile Manipulation.** / Jørgensen, Jimmy Alison; Petersen, Henrik Gordon .2010. Paper presented at Joint 41st International Symposium on Robotics (ISR 2010), Munich, Tyskland.

[3] **Usage of simulations to plan stable grasping of unknown objects with a 3-fingered Schunk hand.** / Jørgensen, Jimmy Alison; Petersen, Henrik Gordon .2008. Paper presented at IEEE/RSJ International Conference on Intelligent Robots and Systems, Nice, Frankrig.

[4] **Advanced Off-line Simulation Framework with Deformation Compensation for High Speed Machining with Robot Manipulators.** / Cortsen, Jens; Petersen, Henrik Gordon .I : I E E E - A S M E Transactions on Mechatronics , 14.06.2012.

[5] **Ring on the hook: placing a ring on a moving and pendulating hook based on visual input.** / Kjær-Nielsen, Anders; Buch, Anders Glent; Jensen, Andreas Kryger; Møller, Bent; Kraft, Dirk; Krüger, Norbert; Petersen, Henrik Gordon; Ellekilde, Lars-Peter . Industrial Robot , Vol. 38, Nr. 3, 2011, s. 301-314.

[6]  **Optimizing visual appearance used for refinement of object poses.** / Holm, Preben Hagh Strunge; Buch, Anders Glent; Petersen, Henrik Gordon .I: Proceedings of the IASTED International Conference on Robotics and Applications. ACTA Press, 2011.

[7] **Assessing Grasp Stability Based on Learning and Haptic Data.** / Bekiroglu, Yasemin ; Laaksonen, Janne ; Jørgensen, Jimmy Alison ; Kyrki, Ville ; Kragic, Danica. I : I E E E Transactions on Robotics , Vol. 27, Nr. 3, 06.2011, s. 616-629.

[8] **Usage and verification of grasp simulation for industrial automation.** / Ellekilde, Lars-Peter; Jørgensen, Jimmy Alison .I: Automate 2011. Proceedings. AUTOMATE, 2011.

[9] **Sensor and Sampling-based Motion Planning for  Minimally Invasive Robotic Exploration of Osteolytic Lesions.** / Wen P. Liu Blake C. Lucas Kelleher Guerin Erion Plaku, IROS 2010

[10] **Inverse kinematics by numerical and analytical cyclic coordinate descent.** / Anders Lau Olsen  and Henrik Gordon Petersen, Robotica (2011), 29 : pp 619-626

[11] **RobWorkSim - an Open Simulator for Sensor based Grasping.** / Jørgensen, Jimmy Alison; Ellekilde, Lars-Peter; Petersen, Henrik Gordon. 2010. Paper presented at Joint 41st International Symposium on Robotics (ISR 2010), Tyskland.

[12] **Grasping Unknown Objects Using an Early Cognitive Vision System for General Scene Understanding.** / Mila Popovi?, Gert Kootstra, Jimmy Alison Jørgensen, Danica Kragic, Norbert Krüger (2011), In: Proceedings of the International Conference on Intelligent RObots and Systems (IROS), September 25-30, 2011, San Francisco, CA

[13] **Accelerated Hierarchical Collision Detection for Simulation using CUDA.** / Jørgensen, Jimmy Alison; Fugl, Andreas Rune; Petersen, Henrik Gordon. 2010.
