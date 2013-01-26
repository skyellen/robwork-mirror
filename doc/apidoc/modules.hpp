//----------------------------------------------------------------------
// Below we have descriptions for the different namespaces. We basically have a
// namespace for each module.

/**
   @brief RobWork library.

   All classes and interfaces in this group only have dependencies on boost, xerces and
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
     * @brief Generic scene graph related stuff.
     */
    namespace graphics {}

    /**
     * @brief The plugin infrastructure, including extension and extension point mechanism
     */
    namespace plugin {}

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
 * @brief RobWorkStudio is the visualization framework of RobWork. It depends on Qt and the core RobWork
 */
namespace rws {



}

/**
 * @brief RobWorkStudio extension libraries.
 *
 * A collection of extensions to RobWorkStudio which enables the use of RobWork functionality
 * through a GUI.
 */
namespace rwslibs {



}


/**
   @brief RobWorkSim is the dynamic simulation framework of RobWork.

   It adds no extra dependencies
   beside RobWork and RobWorkStudio. However, for dynamic simulation, it is strongly recommended to use the ode extension
   from rwsiml, which depend on the Open Dynamics Engine (ODE).
*/
namespace rwsim {



}

/**
 * @brief RobWorkHardware is a collection of extensions to RobWork which allow RobWork to
 * connect to real hardware.
 *
 * As such each extension may have several dependencies.
 */
namespace rwhw {



}

/**
@defgroup rw RobWork
@{
    @copydoc rw

    @defgroup common Common utils
    @{
        @copydoc rw::common
    @}

    @defgroup math Math
    @{
        @copydoc rw::math
    @}

    @defgroup kinematics Kinematics
    @{
        @copydoc rw::kinematics
    @}

    @defgroup models Models
    @{
        @copydoc rw::models
    @}

    @defgroup invkin Inverse Kinematics
    @{
        @copydoc rw::invkin
    @}

    @defgroup pathplanning Pathplanning
    @{
        @copydoc rw::pathplanning
    @}

    @defgroup geometry Geometry
    @{
        @copydoc rw::geometry
    @}

    @defgroup proximity Proximity
    @{
        @copydoc rw::proximity
    @}

    @defgroup graphics Graphics
    @{
        @copydoc rw::graphics
    @}

    @defgroup loaders Loaders
    @{
        @copydoc rw::loaders
    @}

    @defgroup sensor Sensors
    @{
        @copydoc rw::sensor
    @}

    @defgroup plugin Plugin
    @{
        @copydoc rw::plugin
    @}

    @defgroup trajectory trajectory
    @{
        @copydoc rw::trajectory
    @}

    @defgroup task task
    @{
        @copydoc rw::task
    @}

    @defgroup serialization Serialization
    @{
        @brief serialization framework
    @}

    @defgroup
    @{
        @brief Ignore this group. The group has been added to fix a glitch in
        the Doxygen output.
    @}

@}

@defgroup rwlibs RobWork Extension Libraries
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

@defgroup rws RobWorkStudio
@{
    @copydoc rws
@}

@defgroup rwsl RobWorkStudio Extension Libraries
@{
    @copydoc rws
@}


@defgroup rwsim RobWorkSim
@{
    @brief Physics based Simulation and Control of robots and sensors.

    @defgroup rwsim_control Controllers
    @{
        @copydoc rwsim::control
    @}

    @defgroup rwsim_dynamics Dynamics
    @{
        @copydoc rwsim::dynamics
    @}

    @defgroup rwsim_simulator Simulator
    @{
        @copydoc rwsim::simulator
    @}

    @defgroup rwsim_sensor Sensors
    @{
        @copydoc rwsim::sensor
    @}

    @defgroup rwsim_drawable Drawable
    @{
        @copydoc rwsim::drawable
    @}

    @defgroup rwsim_rwphysics rwphysics
    @{
        @copydoc rwsim::drawable
    @}

@}

@defgroup rwsiml RobWorkSim Extension Libraries
@{
    @brief This group should contain all extension points of RobWork.
@}

@defgroup extensionpoints Extension Points
@{
    @brief This group should contain all extension points of RobWork.
@}


*/
