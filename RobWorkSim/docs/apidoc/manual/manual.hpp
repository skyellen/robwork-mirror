/**

\page page_rwsim_manual RobWorkSim manual

- \ref sec_rwsim_manual_intro
- \ref sec_rwsim_install_and_use
- \ref sec_rwsim_consepts
- \ref sec_rwsim_dynamic_workcell
- \ref sec_rwsim_simulator
- \ref sec_rwsim_body
- \ref sec_rwsim_dynamic_device
- \ref sec_rwsim_xml_fileformat
- \ref sec_rwsim_plugins

\section sec_rwsim_manual_intro Introduction
This manual will present RobWorkSim and how to use it. 

\subsection sec_rw_manual_notation Notation

\subsection sec_namespaces Namespaces

The header files of RobWorkSim are distributed across a number of
directories each having its own namespace. The structure of namespaces
reflects the directory containing the code. For example

\code
// Include header files:
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/control/PDController.hpp>

using namespace rwsim::dynamics;
using namespace rwsim::PDController;
\endcode

This structure is the same as RobWork and RobWorkStudio.

\subsection sec_libraries Libraries


\section sec_rwsim_install_and_use Install and Use

Functionality in RobWorkSim depends heavilly on RobWork and RobworkStudio for GUI and specific plugins.
As such, it is recommended to install these before installing RobWorkSim.

\section sec_rwsim_consepts Concepts and Overview

The primary use of RobWorkSim evolves around specifying a DynamicWorkCell (scene with
dynamic information) from which a Simulator instance is created which then is used
to do the actual simulation.

The DynamicWorkCell is conceptually the same as the RobWork WorkCell class and extends
the WorkCell description
with focus on describing the dynamic properties of the scene. It is basically a container
that includes a hierarchy description of the scene including: bodies, obstacles, frames,
devices, controllers, sensors and their mutual attachment to each other.

The DynamicWorkCell is "stateless" in the same sense that the WorkCell is stateless, which
means that typical state values such as force of a rigid body are saved in a state structure
and not in the actual object. The following code snippet exemplifies this:

\code
RigidBody *b1 = getBody1(); // illustrative function "getBody1()"
State stateA = getState();
State stateB = getState();
b1->setForce( Vector3D<>(0,0,1), stateA );
b1->setForce( Vector3D<>(2,2,2), stateB );
std::cout << b1->getForce(stateA); // prints (0,0,1)
std::cout << b1->getForce(stateB); // prints (2,2,2)
\endcode

Not all variables of our "stateless" objects are saved in the state structure since
they are considered to change infrequently. An example of this is getMass() on RigidBody.
As such a rule of thumb is that frequently changing variables such as position, velocity
and force will allways be saved in the state structure. Infrequently changing variables
will be saved in the object instance, e.g. mass, material info, geometry, nr of joints,
position limits, force limits and so on.

The stateless nature of DynamicWorkCell makes it possible to use it in multiple threads
or methods at the same time and without bothering with cloning and copying of the
DynamicWorkCell. However, one should be carefull to change the "static" variables when
using multiple threads since these changes will influence all uses of the variable. For
more indepth description of the StateStructure the reader is directed to the RobWork manual.

Now the DynamicWorkCell can be constructed in c++ or as is done more often through the
XML based DynamicWorkCell file format described in section \ref sec_rwsim_xml_fileformat.
A Simulator is created with an instance of the DynamicWorkCell and is then ready for use.
A typical use is exemplified below:
\code
// create and initialize simulator
DynamicWorkCell::Ptr dwc = getDynamicWorkCell();
DynamicSimulator *sim = makeSimulator( );
sim->initPhysics( dwc );
// set the current state
sim->resetState( initState );
// now do a simulation
while( someStopCriteria ){
  // apply forces/velocities to bodies and devices using controllers
  sim->step( 0.01, state);
  // monitor contacts and states using sensors or the State
}
// do something usefull with "resting" state
\endcode

The Simulator is not stateless and to do simulations in parallel you should create
multiple instances of the simulator.

The simulation is run one step at the time using relatively small timesteps e.g.
[0.001s;0.01s]. The "best" timestep depends on the underlying physics engine,
the current scene, and the application. Please look at section \ref sec_rwsim_simulator
for more information.

There are two constructs designed for getting feedback and influencing the simulation.
These are the SimulatedSensor and the SimulatedController. The controller enables
"control" of bodies and devices or other states in the simulation, where as the sensor
enables getting appropriate feedback, e.g. tactile, visual or other states. Typically
used methods such as applying forces to bodies or setting the velocity of a device
are available on the Body/Device interface and does not require controllers or sensors.

\section sec_rwsim_dynamic_workcell DynamicWorkCell

\section sec_rwsim_simulator The DynamicSimulator
Timestep, ThreadSimulator, PhysicsEngine, PhysicsEngineFactory, EnableBody,

The simulation loop

\section sec_rwsim_body Body


\section sec_rwsim_dynamic_device Devices and controllers

\section sec_rwsim_xml_fileformat DynamicWorkcell Scene File

\subsection Material identifier
Per body/object

\subsection Friction data specification

\subsection Contact data specification



\section sec_rwsim_plugins Plugins

*/


