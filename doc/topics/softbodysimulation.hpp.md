Softbody Simulation {#page_rw_softbodysimulation}
================

This page describes the \a rwlibs/softbody RobWork module. 

[TOC]

# Introduction #

The softbody module implements a non-linear beam model supporting very large deformations and Laying-Down placement operations. The beam has been described in 

-Modeling and Simulation of Grasping of Deformable Objects (2012)

-An Adaptable Robot Vision System Performing Manipulation Actions with Flexible Objects (2013)

## Supplementary material ##

A short presentation and some videos of an application using the softbody module can be found at https://svnsrv.sdu.dk/svn/RobWorkApp/SoftBeamPlugin/presentation/

# Compiling #
## Dependencies ##

The softbody module is currently configured to use the IPOPT https://projects.coin-or.org/Ipopt non-linear optimization library. It is currently linked 
statically and must be present at compile time.

IPOPT should be compiled separately and either placed in the system-wide include/library folders or pointed to by setting IPOPT_HOME accordingly.

IPOPT requires a compatible linear algebra solver library. It supports a variety of routines, e.g. Harwell routines, Pardiso, WSMP and MUMPS (see the options reference at http://www.coin-or.org/Ipopt/documentation/node50.html ).
Currently the build system in RobWork is configured to use MUMPS due to it having the most permissive license.

## CMake settings ##

Make sure to have set the environment variable \b IPOPT_HOME to the root of your IPOPT installation, e.g. with a line in .bashrc:

> export IPOPT_HOME=/home/arf/Documents/Ipopt-3.10.3

After this, enable the flags \b BUILD_rw_softbody and \b RW_BUILD_SOFTBODY.

## Notes ##

The relevant FIND_PACKAGE CMake commands are located in \a RobWork/cmake/RobWorkSetup.cmake


