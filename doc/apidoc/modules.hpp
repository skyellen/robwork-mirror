//----------------------------------------------------------------------
// Below we have descriptions for the different namespaces. We basically have a
// namespace for each module.

//-------------------------------------------------------------------------------------------------------
//											 ROBWORK
//-------------------------------------------------------------------------------------------------------


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
	 * @brief Assembly strategies and task & result formats for assembly operations.
	 */
	namespace assembly {}

	//! @copydoc rwlibs::mathematica::Mathematica
	namespace mathematica {}

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
	 *   @brief Simulation of cameras and other I/O units.
     */
	namespace simulation {}

	/**
     * @brief Simulation of beam models and other soft bodies
	 * 
	 * 
	 * A collection of various soft body simulations, in particular beam models.
	 * This package depends on the optimization package IPOPT (https://projects.coin-or.org/Ipopt).
	 * 
	 * Currently it implements the non-linear 'Modified Russel' beam model, described in the Ph.D. dissertation 'Modeling and Simulation of Grasping of Deformable Objects' by Andreas Rune Fugl (2012).
   	 */
	namespace softbody {}

	/**
     * @brief SWIG generated interface for Robwork. The supported generated interface languages are: Lua, Python and Java.
	 *
	 * The SWIG interface to RobWork is very similar to the RobWork c++ interface. However, due to
	 * limitations in other languages, then the interfaces cannot match the c++ interface exactly.
	 * Class's in this module are documented as c++ classes with c++ functions, but they really
	 * only define the classes and functions that was wrapped and available through SWIG.
   	 */
	namespace swig {}


	namespace calibration {}
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

    @defgroup trajectory Trajectory
    @{
        @copydoc rw::trajectory
    @}

    @defgroup task Task
    @{
        @copydoc rw::task
    @}

    @defgroup serialization Serialization
    @{
        @brief serialization framework
    @}

@}

@defgroup rwlibs RobWork Extension Libraries
@{
    @copydoc rwlibs

    @defgroup algorithms algorithms
    @{
        @copydoc rwlibs::algorithms
    @}

    @defgroup assembly assembly
    @{
        @copydoc rwlibs::assembly
    @}

    @defgroup proximitystrategies proximitystrategies
    @{
        @copydoc rwlibs::proximitystrategies
    @}

    @defgroup dll dll
    @{
        @copydoc rwlibs::dll
    @}

    @defgroup opengl OpenGL
    @{
        @copydoc rwlibs::opengl
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

    @defgroup softbody softbody
    @{
        @copydoc rwlibs::softbody
    @}

    @defgroup swig swig
    @{
        @copydoc rwlibs::swig
    @}

    @defgroup assembly assembly
    @{
        @copydoc rwlibs::assembly
    @}
    @defgroup calibration calibration
    @{
        @copydoc rwlibs::calibration
    @}

@}
*/

//-------------------------------------------------------------------------------------------------------
//											 ROBWORKSTUDIO
//-------------------------------------------------------------------------------------------------------


/**
 * @brief RobWorkStudio is the visualization framework of RobWork.
 *
 * It depends on Qt and the core RobWork.
 *
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
@defgroup rws RobWorkStudio
@{
    @copydoc rws
@}

@defgroup rwslibs RobWorkStudio Extension Libraries
@{
    @copydoc rwslibs
@}
*/



//-------------------------------------------------------------------------------------------------------
//											 ROBWORKSIM
//-------------------------------------------------------------------------------------------------------


/**
   @brief RobWorkSim is the dynamic simulation framework of RobWork.

   It adds no extra dependencies
   beside RobWork and RobWorkStudio. However, for dynamic simulation, it is strongly recommended to use the ode extension
   from rwsiml, which depend on the Open Dynamics Engine (ODE).
*/
namespace rwsim {

/**
   @brief Contact Detection
 */
namespace contacts {}

//! @brief Logging system for dynamic simulation
namespace log {}

/**
   @brief Dynamic simulation
 */
namespace simulator {}

}


//! @brief RobWorkSim libraries: Physics engines, plugins, and script interface.
namespace rwsimlibs {

//! @brief The Bullet engine implementation.
namespace bullet {}

//! @brief GUI elements related to RobWorkSim.
namespace gui {}

//! @brief RobWorkStudio plugins providing RobWorkSim functionality.
namespace plugins {}

//! @brief The RobWorkPhysicsEngine implementation.
namespace rwpe {}

/**
 * @brief Test case library for RobWorkSim.
 *
 * Library that defines a standard set of tests that are more or less generic to all engines.
 * The purpose of the library is to make one unified interface for tests, that can be used
 * for multiple purposes:
 *
 * - Interactive testing in RobWorkStudio. Allows detailed debugging of the internal workings of
 * an engine.
 * - Automated testing in unit tests. Makes code changes easier, and makes sure that the engines
 * keep functioning as they are supposed to.
 * - Data output. Makes it possible to automatically generate graphs for documentation purposes.
 *
 * All of these can be based on one common test definition. In the case of test failures, debugging is
 * much easier as the exact same test can be repeated in interactive mode with advanced debugging facilities.
 *
 * The library defines an extension point, that makes it possible for individual engines to provide
 * their own engine-specific tests. Users can also define specific tests that fits specific applications.
 */
namespace test {}

}

/**

@defgroup rwsim RobWorkSim
@{
    @brief Physics based Simulation and Control of robots and sensors.

    @defgroup rwsim_contacts Contacts
    @{
        @copydoc rwsim::contacts
    @}

    @defgroup rwsim_control Controllers
    @{
        @copydoc rwsim::control
    @}

    @defgroup rwsim_dynamics Dynamics
    @{
        @copydoc rwsim::dynamics
    @}

    @defgroup rwsim_log Log
    @{
        @copydoc rwsim::log
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

@defgroup rwsimlibs RobWorkSim libraries
@{
    @brief Physics engines, GUI & Plugins, and script interface.

    @defgroup rwsimlibs_bullet Bullet
    @{
        @copydoc rwsimlibs::bullet
    @}

    @defgroup rwsimlibs_gui RobWorkSim GUI
    @{
        @copydoc rwsimlibs::gui
    @}

    @defgroup rwsimlibs_plugins RobWorkSim Plugins
    @{
        @copydoc rwsimlibs::plugins
    @}

    @defgroup rwsimlibs_rwpe RobWorkPhysicsEngine
    @{
        @copydoc rwsimlibs::rwpe
    @}

    @defgroup rwsimlibs_test RobWorkSim Test Case Library
    @{
        @copydoc rwsimlibs::test
    @}

@}

@defgroup rwsiml RobWorkSim Extension Libraries
@{
    @brief This group should contain all extension points of RobWork.
@}
*/

//-------------------------------------------------------------------------------------------------------
//											 ROBWORKHARDWARE
//-------------------------------------------------------------------------------------------------------


/**
 * @brief RobWorkHardware is a collection of extensions to RobWork which allow RobWork to
 * connect to real hardware.
 *
 * As such each extension may have several dependencies.
 */
namespace rwhw {
    /**
     * @brief Camera drivers
     */
    namespace camera {}
}

/**

@defgroup rwhw RobWorkHardware
@{
    @copydoc rwhw

    @defgroup camera camera
    @{
        @brief Camera driver wrappers.
    @}

    @defgroup can can
    @{
        @brief CAN bus driver wrappers.
    @}

    @defgroup crsA465 crsA465
    @{
        @brief driver wrapper for the crsA465 robot.
    @}

    @defgroup Fanuc Fanuc
    @{
        @brief Interface for communicating with Fanuc controller through proprietary software
        from Gibotech A/S.
    @}

    @defgroup katana katana
    @{
        @brief driver wrapper for katana robot.
    @}

    @defgroup MotomanIA20 MotomanIA20
    @{
        @brief Driver for motoman IA20
    @}

    @defgroup dockwelder dockwelder
    @{
        @brief Driver interface for the dockwelder robot based on ethernet.
    @}

    @defgroup PowerCube PowerCube
    @{
        @brief Driver interface through RS232 and CAN bus. Works for eqrlier versions of the
        PowerCube interface.
    @}

    @defgroup sdh sdh
    @{
        @brief Wrapper for SDH the 3-finger dextrous hand from schunk.
    @}

    @defgroup serialport serialport
    @{
        @brief multi platform serial port interface. Rather simplistic but usefull for most stuff.
    @}

    @defgroup sick sick
    @{
        @brief driver interface for sick scanner
    @}

    @defgroup swissranger swissranger
    @{
        @brief Driver wrapper for SwissRanger (early version)
    @}

    @defgroup tactile tactile
    @{
        @brief Driver wrapper for Weiss tactile array sensors (early version only)
    @}

@}

*/



/**

@defgroup extensionpoints Extension Points
@{
    @brief This group should contain all extension points of RobWork.
@}

@defgroup sandbox Sandbox
@{
    @brief compilable stuff under development should go into the sandbox group.
@}


*/
