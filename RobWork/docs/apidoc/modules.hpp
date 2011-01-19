//----------------------------------------------------------------------
// Below we have descriptions for the different namespaces. We basically have a
// namespace for each module.

/**
   @brief RobWork library.

   All classes and interfaces in this group only have dependencies on boost and
   STL. It is considered the core of the RobWork project.
 */
namespace rw {
	/**
	 * @brief Matrices, vectors, configurations, and more.
	 */
	namespace math {}
	/**
	 * @brief Various utilities and definitions of general use
	 */
	namespace common {}
	/**
	 * @brief Workcell and device models
	 */
	namespace models {}
	/**
	 * @brief Kinematic modelling
	 */
	namespace kinematics {}

    /**
     * @brief Inverse kinematics interfaces and iksolver classes
     */
	namespace invkin {}

	/**
	 * @brief Path-planning for devices
	 */
	namespace pathplanning {}

	/**
	 * @brief Interfaces for collision checking and distance calculation.
	 */
	namespace proximity {}

	/**
	 * @brief Loading and storing of CAD models
	 */
	namespace geometry {}

	/**
	 * @brief Workcell loaders and other loaders
	 */
	namespace loaders {}

	/**
	 * @brief Sensor interfaces
	 */
	namespace sensor {}

	/**
	 *  @brief Trajectory, path, interpolation and blending.
	 */
	namespace trajectory {}

	/**
	 * @brief Task descriptions
	 */
	namespace task {}
}

/**
  @brief Extension libraries for RobWork.

  Classes in this group can have specific dependencies on external libraries.
 */
namespace rwlibs {
	/**
	 * @brief Various algorithms
	 */
	namespace algorithms {}

	/**
	 * @brief Dynamic library loading
	 */
	namespace dll {}

	/**
	 * @brief OpenGL drawing of workcells and geometries
	 */
	namespace opengl {}

	/**
	 * @brief A Lua interface to RobWork
	 */
	namespace lua {}

	/**
	 * @brief A collection of OS specific include configurations
	 */
	namespace os {}

	/**
	 * @brief A collection of pathoptimization algorihms
	 */
	namespace pathoptimization {}

	/**
	 * @brief Path planners
	 */
	namespace pathplanners{}

	/**
	 * @brief Proximity strategies
	 */
	namespace proximitystrategies {}

	/**
       @brief Simulation of cameras and other I/O units.
    */
	namespace simulation {}
}

/**

@defgroup rw rw
@{
    @copydoc rw

    @defgroup math math
    @{
        @copydoc rw::math
    @}

    @defgroup common common
    @{
        @copydoc rw::common
    @}

    @defgroup models models
    @{
        @copydoc rw::models
    @}

    @defgroup kinematics kinematics
    @{
        @copydoc rw::kinematics
    @}

    @defgroup invkin invkin
    @{
        @copydoc rw::invkin
    @}

    @defgroup pathplanning pathplanning
    @{
        @copydoc rw::pathplanning
    @}

    @defgroup proximity proximity
    @{
        @copydoc rw::proximity
    @}

    @defgroup geometry geometry
    @{
        @copydoc rw::geometry
    @}

    @defgroup loaders loaders
    @{
        @copydoc rw::loaders
    @}

    @defgroup sensor sensor
    @{
        @copydoc rw::sensor
    @}

    @defgroup trajectory trajectory
    @{
        @copydoc rw::trajectory
    @}

    @defgroup task task
    @{
        @copydoc rw::task
    @}

    @defgroup
    @{
        @brief Ignore this group. The group has been added to fix a glitch in
        the Doxygen output.
    @}
@}

@defgroup rwlibs rwlibs
@{
    @copydoc rwlibs

    @defgroup algorithms algorithms
    @{
        @copydoc rwlibs::algorithms
    @}

    @defgroup proximitystrategies proximitystrategies
    @{
        @copydoc rwlibs::proximitystrategies
    @}

    @defgroup dll dll
    @{
        @copydoc rwlibs::dll
    @}

    @defgroup drawable drawable
    @{
        @copydoc rwlibs::drawable
    @}

    @defgroup pathplanners pathplanners
    @{
        @copydoc rwlibs::pathplanners
    @}

    @defgroup lua lua
    @{
        @copydoc rwlibs::lua
    @}

    @defgroup os os
    @{
        @copydoc rwlibs::os
    @}

    @defgroup pathoptimization pathoptimization
    @{
        @copydoc rwlibs::pathoptimization
    @}

    @defgroup simulation simulation
    @{
        @copydoc rwlibs::simulation
    @}

    @defgroup
    @{
        @brief Ignore this group. The group has been added to fix a glitch in
        the Doxygen output.
    @}
@}

*/
