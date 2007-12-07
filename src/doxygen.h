/**

@mainpage RobWork

@section intro_sec Introduction

RobWork is a framework for simulation and control of robotics with emphasis on
industrial robotics and their applications.

Major goals of this framework:

- Provide a single framework for offline and online robot programming including
  modelling, simulation and (realtime)control of robotics
.

Target audience:
- Researchers who needs a common framework for experimental robotics
- Students who wants to experiment with concepts of robotics
- Implementers of robot applications
.

This framework uses the following 3rd party libraries:
- stl (C++ Standard Template Library)
- boost (Peer-reviewed C++ libraries)
- boost::ublas, for basic linear algebra
- boost::spirit, for datafile parsing
- boost::graph, for probabilistic roadmaps
- LAPACK, for numerical computations
- OpenGL, for visualization
.

And the following optional libraries:
- opcode, for collision detection
- PQP, for collision detection
- RAPID, for collision detection
.

Since this library uses boost, and other modern frameworks, a modern (2000+) C++
compiler like GCC 3 or Visual C++ 7 is required. Not all files are expected to
compile without problems on ancient C++ compilers.

This library currently fails to compile with gcc 4.1.2 see
http://gcc.gnu.org/bugzilla/show_bug.cgi?id=28016 for details

@defgroup rw RW
@{
    @brief Core classes and interfaces for RobWork

    @defgroup math Math
    @{
        @brief Classes and interfaces for performing basic linear algebra
    @}

    @defgroup common Common
    @{
        @brief Various utilities and definitions of general use
    @}

    @defgroup models Models
    @{
        @brief Classes and interfaces for modelling robot 
    @}

    @defgroup kinematics Kinematics
    @{
        @brief Classes and interfaces for kinematics
    @}

    @defgroup InverseKinematics InverseKinematics
    @{
        @brief Classes and interfaces for invers kinematics algorithms
    @}

    @defgroup iksolvers IKSolvers
    @{
        @brief Collection of inverse kinematics solvers
    @}

    @defgroup pathplanning PathPlanning
    @{
        @brief Classes and interfaces for performing (collisionfree) motionplanning
        of manipulators
    @}

    @defgroup proximity Proximity
    @{
        @brief Classes and interfaces for proximity checking/calculation
    @}

    @defgroup geometry Geometry
    @{
        @brief Classes and interfaces for for loading and storing geometry
    @}

    @defgroup interpolator Interpolator
    @{
        @brief A collection of classes for performing interpolation and
        blending. This design is NOT STABLE
    @}

    @defgroup loaders Loaders
    @{
        @brief WorkCell loaders and other loaders
    @}

    @defgroup sensor Sensor
    @{
        @brief Sensor interfaces
    @}

    @defgroup 
    @{
        @brief Ignores this group. The group has been added to fix a glitch in
        the Doxygen output.
    @}
@}

@defgroup rwlibs RWLibs
@{
    @brief RobWork libraries

    @defgroup algorithms Algorithms
    @{
        @brief Collection of algorithms
    @}

    @defgroup proximitystrategies ProximityStrategies
    @{
        @brief Collection of proxiity strategies
    @}

    @defgroup devices Devices
    @{
        @brief Collection of device drivers
    @}

    @defgroup drawable Drawable
    @{
        @brief Classes and interfaces for loading drawable files and drawing workcells
    @}

    @defgroup io I/O
    @{
        @brief A collection of classes for performing device I/O
    @}

    @defgroup pathplanners PathPlanners
    @{
        @brief Collection of path planners
    @}

    @defgroup sensors Sensors
    @{
        @brief Collection of sensors
    @}

    @defgroup 
    @{
        @brief Ignores this group. The group has been added to fix a glitch in
        the Doxygen output.
    @}

@}

*/

/**
 * @brief RobWork core library. Collection of core functionality and interfaces.
 */
namespace rw {
    
}

/**
 * @brief RobWork libraries. 
 */
namespace rwlibs {
    
}
