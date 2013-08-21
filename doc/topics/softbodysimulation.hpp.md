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

## Units ##

-Dimensions/deformation result: mm

-Young's modulus: MPa

-Poisson's ratio: (unitless, between 0.0 and 0.5)

-Mass density: kg/mm^3

## Initilization ##

    boost::scoped_ptr< rwlibs::softbody::ModRusselBeamIpopt > beamPtr;
    boost::shared_ptr< rwlibs::softbody::BeamGeometryCuboid > beamGeomPtr;
  	boost::shared_ptr< rwlibs::softbody::BeamObstaclePlane> beamObstaclePtr;

	double dx = 110.0; 	// length of beam
	double dy = 7.0; 	// thickness of beam
	double dz = 57.0;	// width of beam

	int M = 32; 		// number of cross sections in beam

	std::vector<double> Exvec = getExvec(M);	// Young's modulus for each cross section
	std::vector<double> vxvec = getvxvec(M);	// Poisson's ratio for each cross section
	std::vector<double> Rhovec = getRhovec(M);	// Mass density for each cross section

    Transform3D<> beamGeomTrans = Transform3D<>::identity(); // transform of the beam base frame at x=0

	Vector3D<> G = Vector3D<> ( 0.0, 9.82, 0.0 ); // gravity vector

    beamGeomPtr.reset (
		new BeamGeometryCuboid (
		    dx,
		    dy,
		    dz,
		    Exvec,
		    vxvec,
		    Rhovec,
			beamGeomTrans,
		    G
		)
	);

	Vector3D<> obstacleNormal = Vector3D<> ( 0.0, 1.0, 0.0 ); 	// normal of the plane obstacle
	Transform3D<> obstacleTrans = Transform3D<>::identity(); 	// transform of the plane obstacle

    beamObstaclePtr.reset (
		new BeamObstaclePlane (
		    rw::geometry::Plane (
		        obstacleNormal, 0.0
		    ),
			obstacleTrans
		)
	);

	/* instantiate the beam, passing the geometry, obstacle and beam size */
    beamPtr.reset (
		new ModRusselBeamIpopt (
		    BeamGeomPtr,
		    BeamObstaclePtr,
		    M
		)
	);

## Usage ##

    /* vectors for storing the solver results */
    boost::numeric::ublas::vector<double> avec(M); 	// angle of each cross section in radians
    boost::numeric::ublas::vector<double> Uvec(M); 	// x-deformation of each cross section in mm
    boost::numeric::ublas::vector<double> Vvec(M); 	// y-deformation of each cross section in mm

	/* 
	 * Optional: Provide starting guess for optimization by setting avec, Uvec and Vvec appropiately
	 * a starting guess of zero (which we effectively do right here) usually works fine
	 */

    beamPtr->setAccuracy ( 1.0e-3 );					// accuracy goal for optimization
    beamPtr->setIntegralIndices ( integralIndices );	// vector containing the indices of constraints that should be active
	beamPtr->solve ( avec, Uvec, Vvec );				// invoke solver, result will be stored in the passed vectors

## Setting frames ##

A small non-working code example from a RobWorkStudio plugin where the solution stored in avec, Uvec, Vvec is used to set frames in a workcell.

	void SoftBeamPlugin::setRwFrames ( const boost::numeric::ublas::vector< double >& U, const boost::numeric::ublas::vector< double >& V, const boost::numeric::ublas::vector< double >& avec ) {
		_pluginState = _wc->getStateStructure()->getDefaultState();

		for ( int i = 1; i < ( int ) _framePtrList.size(); i++ ) {
		    MovableFrame *frame = _framePtrList[i];

		    Transform3D<> trans = _beamFrameBase->getTransform ( _pluginState );

		    trans.P() [0] = U[i] * 1.0e-3;
		    trans.P() [1] = V[i] * 1.0e-3;

		    EAA<>  rotEAA ( 0.0, 0.0, avec[i] );
		    trans.R() = rotEAA.toRotation3D();

		    frame->setTransform ( trans, _pluginState );
		}

		_wc->getStateStructure()->setDefaultState ( _pluginState );
		getRobWorkStudio()->setState ( _pluginState );
	}

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
