Softbody Simulation {#page_rw_softbodysimulation}
================

This page describes the \a rwlibs/softbody RobWork module. 

(Please note that this module is not enabled by default as it requires 3rd party libraries.)

[TOC]

# Introduction #

The softbody module implements a non-linear beam model supporting large deformations and Laying-Down placement operations. The beam has been described in 

-Modeling and Simulation of Grasping of Deformable Objects (2012)

-An Adaptable Robot Vision System Performing Manipulation Actions with Flexible Objects (2013)

## Features ##

The current implementation supports cuboid geometries and non-homogeneous materials under non-penetration constraints.

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

# Usage #

Here is a code example

        new BeamGeometryCuboid (
            get_dx(),
            get_dy(),
            get_dz(),
            getExvec(),
            getvxvec(),
            getRhovec(),
            Transform3D<>::identity(),
            Vector3D<> ( 0.0, 9.82, 0.0 )
        )

        new BeamObstaclePlane (
            Plane (
                Vector3D<> ( 0.0, 1.0, 0.0 ), 0.0
            ),
            Transform3D<>::identity()
        )

        new ModRusselBeamIpopt (
            _sBeamGeomPtr,
            _sBeamObstaclePtr,
            get_m()
        )

# Future work #

## Performance ##

-Improvement of the hessian approximation. 

Currently we use IPOPT's built-in numerical approximation which does not fully exploit sparsity.

-Evaluation of other linear algebra solver routines. HSL MA57 should provide a higher theoretical performance than MUMPS for moderately-sized problems, but has stricter licensing.

## Supported constraints ##

-Add support for more types of constraints

The beam implements non-penetration constraints by the means of IPOPT inequality constraints. These are enabled/disabled for each cross section of the beam by the user passing a list of active constraints. A more flexible interface supporting different types of constraints, e.g. ones fixing the beam in space should be made. 

## Supported geometry types ##

-Add support for more geometry types

The abstract base class \a BeamGeometry defines the integrals that should be evaluated upon geometry cross sections. This is implemented for cuboid cross
sections in \a BeamGeomtryCuboid. If for instance it should be wished to support spherical cross-sections, it is a matter of evaluating the same integrals but on
spherical domains.
