/**

\page page_rw_manual RobWork manual

- \ref sec_rw_manual_intro
	- \ref sec_rw_manual_notation
- \ref sec_namespaces
- \ref sec_libraries
- \ref sec_rw_manual_workcells
    - \ref sec_rw_manual_load_workcell
    - \ref sec_rw_manual_traverse_devices
    .
- \ref sec_rw_manual_states
    - \ref sec_rw_manual_FKTable
    - \ref sec_rw_manual_FKRange
    - \ref sec_rw_manual_dafs
    .
- \ref sec_rw_manual_device_configurations
- \ref sec_rw_manual_metrics
- \ref sec_rw_manual_collisions
- \ref sec_rw_manual_constraints
- \ref sec_rw_manual_sampling
- \ref sec_rw_manual_pathplanning
- \ref sec_rw_manual_invkin
- \ref sec_rw_manual_pointer_conventions
- \ref sec_rw_manual_task

- \subpage page_xml_workcell_format


\section sec_rw_manual_intro Introduction

All code examples of this manual are self-contained in the sense that
they will compile if placed in a C++ file of their own. The examples
are found in the \c RobWork/docs directory. 

The workcell \b workcell.xml described 

\subsection sec_rw_manual_notation Notation

In general a diagonal notation form will be used to describe the relation
of vectors, rotation matrixes, homogenous transform, velocity screw,
and so on.

<table>
<tr>
<td>@f$ \robax{a}{\mathbf{P}} @f$ </td>
<td>Vector P seen in frame \b a</td>
</tr>
<tr>
<td>@f$ \robabx{a}{b}{\mathbf{P}} @f$ </td>
<td>Translation of frame \b b seen in frame \b a</td>
</tr>
<tr>
<td>@f$ \robabx{a}{b}{\mathbf{R}} @f$ </td>
<td>Rotation of frame \b b seen in frame \b a</td>
</tr>
<tr>
<td>@f$ \robabx{a}{b}{\mathbf{T}} @f$ </td>
<td>Homogenous transform of frame \b b seen in frame \b a</td>
</tr>
<tr>
<td>@f$ \robabcdx{a}{b}{c}{d}{\mathbf{T}_v} @f$ </td>
<td>Velocity transform that transforms the reference frame from
\b b to \b a and the velocity reference point from \b c to \b d</td>
</tr>
<tr>
<td>@f$ \robabcdx{a}{b}{c}{d}{\mathbf{T}_f} @f$ </td>
<td>Force transform that transforms the reference frame from
\b b to \b a and the force reference point from \b c to \b d</td>
</tr>
<tr>
<td>@f$ \robabx{a}{b}{\mathbf{J}} @f$ </td>
<td>A jacobian matrix defined from reference frame \b a to frame \b b</td>
</tr>

</table>




\section sec_namespaces Namespaces

The header files of RobWork are distributed across a number of
directories each having its own namespace. The structure of namespaces reflects the directory containing the code. For example

\code
// Include header files:
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Frame.hpp>

using namespace rw::models; //Namespace for WorkCell included by #include<rw/models/WorkCell.hpp>
using namespace rw::kinematics; //Namespace for Frame included by #include <rw/kinematics/Frame.hpp>

\endcode

All classes related to the RobWorkStudio package are placed in a namespace rws. All classes related to RobWorkHardware are in the namespace rwhw;

\section sec_libraries Libraries

All classes of the \b rw directory are provided in a single library
named \b rw.

The subdirectories of the \b rwlibs directory each correspond to a
different library. The subdirectory \b rwlibs/xyz corresponds to the
library named \b rw_xyz and contains the objects in the namespace rwlibs::xyz. For example, suppose your program contains
the following include statement:

\code
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
\endcode

To build this program, you should link with \b rw_pathplanners.

\section sec_rw_manual_workcells Workcells

\subsection sec_rw_manual_load_workcell Loading a workcell

RobWork support workcells described in an XML format.

The below program loads a workcell from the file named on the command
line. If the loading of the workcell fails, the
rw::loaders::WorkCellLoader::load() function will throw an exception,
and the program will abort with an error message.

\include ex-load-workcell.cpp

\subsection sec_rw_manual_traverse_devices Traversing the devices of a workcell

A workcell contains a number of devices (rw::models::Device). You can
for example traverse the devices stored in a workcell and print their
names like this:

\include ex-print-devices.cpp

A device of a specific name can be retrieved from a workcell with
rw::models::WorkCell::findDevice().

\section sec_rw_manual_states Kinematics trees and states

The kinematic structure of the work cell is represented by a tree of
frames (see rw::kinematics::Frame). The root of the kinematic tree is
called the \e world \e frame (rw::models::WorkCell::getWorldFrame()).
Each frame has a transformation (see rw::math::Transform3D) relative
to its parent frame and this transformation may change in response to
values assigned for the frame. A revolute joint of a device (see
rw::models::RevoluteJoint) is for example implemented as a frame that
has a single value that rotates the frame relative to its parent. 
Besides revolute joints RobWork also supports prismatic joints and dependent
joints for which the value may depend on other joints.

It is important in RobWork to note that the values for the frames are
not stored \e within the frames, but are instead stored explicitly in
a value of type rw::kinematics::State. Given a state for the workcell,
the transform of a frame relative to its parent can be calculated with
rw::kinematics::Frame::getTransform().

The frames of the workcell are always organized in a tree, but for
certain frames the parent that they are connected to can dynamically
be changed. These frames are called \e dynamically \e attachable \e
frames or \e DAFs for short. The parent that a DAF is attached to is
not stored within the DAF itself, but is instead stored externally in
a State value. Different state values can thus correspond to different
structures of the tree. Given a state for the workcell the parent and
children of a frame can be retrieved with
rw::kinematics::Frame::getParent() and
rw::kinematics::Frame::getChildren().

Because the values of the frames and the attachments of DAFs are
stored outside of the workcell, we say that the workcell is \e
stateless. This enables a workcell and the associated data to be used 
concurrently in multiple threads as well as to easily communicate the 
entire state of a workcell.

To illustrate these important ideas, this example shows how to print
the structure of the kinematic tree of the workcell and for each frame
print also the position of the frame in space:

\include ex-print-kinematic-tree.cpp

We see from this example that given a state, it is straight-forward to
compute the transform of every single frame in the workcell. RobWork
has some utilities to make calculation of forward kinematics
convenient in the day to day work, such a rw::kinematics::FKTable and rw::kinematics::FKRange described below.

\subsection sec_rw_manual_FKTable World transforms for a set of frames

rw::kinematics::FKTable computes the forward kinematics for a number
of frames for a common state. The results of the forward kinematics
are stored in the FKTable object so that the transform for a frame is
not computed over and over again. This example shows how the transform
for a sequence of frames can be efficiently computed:

\include ex-world-transforms.cpp

\subsection sec_rw_manual_FKRange Relative transforms for a pair of frames

rw::kinematics::FKRange computes the relative transform for a pair of
frames. To efficiently compute the relative transform for a pair of
frames the path in the kinematic tree that connects the frames must be
computed. Knowing the path and the relative transform between adjacent
frames of the path (rw::kinematics::Frame::getTransform()) the full
transform from start to end of the path can be computed. This example
shows the use of rw::kinematics::FKRange:

\include ex-frame-to-frame-transform.cpp

If you repeatedly compute the forward kinematics for the same pair of
frames and the same parent-child structure of the tree, you can reuse
the rw::kinematics::FKRange object so that e.g. the path connecting
the frames need not be recomputed. For example, given a pair of frames
and a set of states the relative transforms that relate the frames can
be computed efficiently as follows:

\include ex-frame-to-frame-transforms.cpp

The frameToFrameTransform() utility function is available as
rw::kinematics::Kinematics::frameTframe().

\subsection sec_rw_manual_dafs Dynamically attachable frames and movable frames

A \e dynamically \e attachable \e frame (DAF) is a frame for which the
parent frame can be changed. We say that the frame is attached to a
new parent frame (rw::kinematics::Frame::attachFrame()). A DAF can be
attached to any frame of the workcell except itself. You should avoid
attaching a DAF to a child of its subtree as this will create a cycle
in the kinematic structure. Frames of any type can be a DAF. You can
check if a frame is a DAF like this:

\include ex-is-daf.cpp

DAFs are used for example to simulate the picking up of an item by a
gripper. The item is represented by a DAF and is initially attached to
some frame of the workcell. When the gripper is closed, the picking up
of the item is simulated by attaching the item frame to the gripper
frame.

If the parent frame of a DAF is changed, the world transform of the
DAF will generally change also. When simulating the picking up of an
item, you do not want the item to instantly change position in space.
Therefore a DAF is often a \e movable \e frame
(rw::kinematics::MovableFrame) also. A movable frame is a frame for
which an arbitrary transform can be set for the transform of the frame
relative to its parent (rw::kinematics::MovableFrame::setTransform()).
To simulate the gripping of the item, the parent of the frame is set
to the frame of the gripper, and at the same time the relative
transform of the item is assigned a value that equals the transform
from gripper to item. This procedure is carried out as follows:

\include ex-grip-frame.cpp

The function receives the current state of the workcell as input and
updates this state to reflect the gripping of the item. Recall that
the frames themselves are stateless: The attachment of the DAF and its
change of relative transform is stored entirely within the state.

RobWork provides utilities for the above in the form of the
rw::kinematics::Kinematics::gripFrame() and
rw::kinematics::Kinematics::gripMovableFrame() collection of
functions.

\section sec_rw_manual_device_configurations Devices and configurations

Algorithms for workcells often do not operate on the level of frames
and the values for frames. Instead they operate on \e devices
(rw::models::Device) and \e configurations (rw::math::Q) for devices.

A device controls a subset of frames of the workcell. Different
devices may overlap in the frames that they control and one device may
contain one or more other devices (rw::models::CompositeDevice). A
workcell for a factory application can for example have one device for
a 6-axis industrial robot and another 2-axis device that controls the
position of the base of the robot. These two device may be combined
into one large 8-axis device (rw::models::CompositeDevice).

A configuration is an vector of values for the frames of a device.
Configurations support standard vector operations such as addition,
scalar multiplication, inner product, etc. The \e configuration \e
space of a device is the set of valid configurations of a device. For
the rw::models::Device type, the configuration space is always box
shaped and described by a tuple containing the lower and upper corner
(see rw::models::Device::QBox and rw::models::Device::getBounds()).

Algorithms for devices often assume that only the configuration for
the device is changed while the state (rw::kinematics::State) of the
rest of the workcell stays fixed. A path-planner may for example
return a path in the form of a sequence of configurations together
with the common workcell state for which the planning was done. When
writing or using such algorithms you will often have translate from a
configuration for the device to a state of the workcell. This is
accomplished by the methods rw::models::Device::setQ() and
rw::models::Device::getQ(). This is example shows to convert a
sequence of configurations for a common state into a sequence of
states:

\include ex-get-state-path.cpp

This utility function is also available as
rw::models::Models::getStatePath().

Note that rw::models::Device::setQ() and rw::models::Device::getQ() do
not store a configuration within the device: The configuration is read
from and written to a state value. The device itself is stateless.

\section sec_rw_manual_metrics Configuration space metrics and other metrics

rw::math::Metric<\e X> is the general interface for measuring a
distance between a pair of values of type \e X. Path planning
algorithms, for example, often require a metric for measuring the
distance between configurations.

Metrics available in RobWork include:

- Manhattan metric (rw::math::MetricFactory::makeManhattan(),
  rw::math::MetricFactory::makeWeightedManhattan())

- Euclidean metric (rw::math::MetricFactory::makeEuclidean(),
  rw::math::makeWeightedEuclidean())

- Infinity metric (rw::math::MetricFactory::makeInfinity(),
rw::math::MetricFactory::makeWeightedInfinity())

These build-in metrics can be instantiated for configuration types
(rw::math::Q) and other vector types such as rw::math::Vector3D and
std::vector<double>. This program shows instantiation and expected output for
3 different metrics:

\include ex-metrics.cpp


\section sec_rw_manual_collisions Collision checking

Workcells loaded with rw::loaders::WorkCellLoader contain a default
collision setup possibly specified via a CollisionSetup XML file.

For each frame of the workcell zero or more geometries can be
associated. The collision setup essentially specifies for what pairs
of frames of the workcell that collision checking between geometries
should be done.

Classes and interfaces relevant to collision checking include:

- rw::proximity::CollisionStrategy: Collision checking for pairs of
  frames of the workcell.

- rw::proximity::CollisionSetup: Setup read from a file describing
  what pairs of frames to check for collisions.

- rw::proximity::CollisionDetector: Collision checking for an entire
  workcell according to a collision setup
  (rw::proximity::CollisionSetup) and a primitive collision strategy
  (rw::proximity::CollisionStrategy).

Collision strategies are implemented via external libraries such as
Yaobi. Wrappers for the external libraries are provided with
the \b rw_proximitystrategies library of the \b rwlibs directory.

This program shows how to construct a collision detector for the
default collision setup of a workcell. The example program then calls
the collision detector to see if the workcell is in collision in its
initial state:

\include ex-collisions.cpp


\section sec_rw_manual_constraints Workcell and configuration space constraints

A collision detector (rw::proximity::CollisionDetector) is an example
of a constraint on the states of a workcell. Collision checking is but
one form of constraint, and applications may implement their
constraints in terms of other classes than
rw::proximity::CollisionDetector.

The general interface for a discrete constraint on states
(rw::kinematics::State) is rw::pathplanning::StateConstraint. The
method to call to check if a constraint is satisfied for a state is
rw::pathplanning::StateConstraint::inCollision(). The naming of the
method is only a convention. The constraint need not not be concerned
with actual collisions of the workcell. A user may inherit from the interface 
and implement any kind of constraint they desire.

Path planners and other planners often operate on configurations
(rw::math::Q) rather than workcell states (rw::kinematics::State). The
interface for a discrete constraint on the configuration space is
rw::pathplanning::QConstraint and the method to call to check if the
constraint is satisfied is
rw::pathplanning::QConstraint::inCollision().

rw::pathplanning::StateConstraint as well as
rw::pathplanning::QConstraint provide constructor functions and
functions for combining constraints.

A sampling based path planner typically calls a configuration
constraint (rw::pathplanning::QConstraint) to verify individual
configurations. The path planner connects individual configurations by
edges, and verifies if the device can follow the path represented by
the edge. The interface for verifying a configuration space path
connecting a pair of configurations is called
rw::pathplanning::QEdgeConstraint. The method on the interface to
verify the edge is rw::pathplanning::QEdgeConstraint::inCollision().

Given a configuration constraint (rw::pathplanning::QConstraint), a
constraint for an edge (rw::pathplanning::QEdgeConstraint) can be
implemented by discretely checking the edge for collisions. When
constructing such edge constraint (see
rw::pathplanning::QEdgeConstraint::make()) you can specify the
resolution and metric for the discrete verification of the edge, or a
default metric and resolution can be used.

A configuration constraint together with an edge constraint is named a
planner constraint (rw::pathplanning::PlannerConstraint).
rw::pathplanning::PlannerConstraint::make() utility functions are
provided to ease the construction of constraints for standard
collision detection.

This program constructs a collision detector and corresponding default
planner constraint for the first device of the workcell. The program
calls the planner constraint to check if the edge from the lower to
upper corner of the configuration space can be traversed:

\include ex-constraints.cpp

\section sec_rw_manual_sampling Configuration space sampling

Configuration space sampling is a useful tool for path planners and
various other planning algorithms.

The interface for a sampler of the configuration space is
rw::pathplanning::QSampler. The rw::pathplanning::QSampler interface
provides constructor functions, including:

- rw::pathplanning::QSampler::makeFinite(): Deterministic sampling from a finite
  sequence of configurations.

- rw::pathplanning::QSampler::makeUniform(): Configurations for a device sampled
  uniformly at random.

- rw::pathplanning::QSampler::makeConstrained(): A sampler filtered by a
  constraint.

This example shows the construction of a sampler of collision free
configurations. The sampler calls a randomized sampler of the
configuration space of the device, and filters these configurations by
the constraint that the configurations should be collision free.

\include ex-qsampler.cpp


\section sec_rw_manual_pathplanning Path planning

rw::pathplanning::PathPlanner<\e From, \e To, \e Path> is the general
interface for finding a path of type \e Path connecting a start
location of type \e From and an goal location of type \e To.

Important variations of this interface includes:

- rw::pathplanning::QToQPlanner: Standard planning of a configuration
  space path that connects a start configuration to a goal
  configuration.

- rw::pathplanning::QToTPlanner: Planning of a configuration space
  path connecting a start configuration to \e any end configuration
  for which a spatial constraint represented a value of type
  rw::math::Transform3D<> is satisfied. Typically, planners of this
  type find paths for devices such that the tool of the device ends up
  at the given goal transformation (in other words, the planner of
  type rw::pathplanning::QToTPlanner implicitly solves an inverse
  kinematics problem).

- rw::pathplanning::QToQSamplerPlanner: Planning of a configuration
  space path from a start configuration to any end configuration
  returned by the sampler (rw::pathplanning::QSampler) representing
  the goal region.

These 3 planners all represent the resulting path by a sequence of
configurations (rw::trajectory::QPath).

The path planners of RobWork are placed in the library \b
rw_pathplanners. The example below instantiates a path planner for the
first device of the workcell and plans a number of paths to random
collision free configurations of the workcell. The full configuration
space path mapped to the corresponding sequence of states
(rw::kinematics::State) and written to a file that can be loaded into
\b RobWorkStudio using the \b PlayBack plugin. The example makes use
of configuration space sampling and path planning constraints
described in these earlier sections:

- \ref sec_rw_manual_sampling
- \ref sec_rw_manual_constraints

\include ex-path-planning.cpp

The path planner of the above example is based on the SBL algorithm.
This example shows instantiation of some more of the available path
planners:

\include ex-get-path-planner.cpp

Variations of these constructor functions have options for example for
controlling the configuration space exploration of the planner.

\section sec_rw_manual_invkin Inverse kinematics

Module rw::invkin contains inverse kinematics (IK) solvers. The
primary types of IK solvers are:

- rw::invkin::IterativeIK: Iterative IK solvers.

- rw::invkin::ClosedFormIK: Analytical IK solvers.

Both types of IK solvers take a transform
(rw::math::Transform3D<>) as input and return configurations for a
device for which the transform from the base to the end of the device
(rw::models::Device) equals the given transform.

An iterative IK solver needs a start configuration from which to start
the iterative search. Depending on the start configuration and other
constraints, the IK solver may fail or succeed in finding a valid
configuration.

\include ex-ik.cpp

The solution of an iterative solver may depend on the starting configuration, provided in the
state given in the solve method. To search for all solution an iterative ik solver may 
be wrapped in the rw::invkin::IKMetaSolver which calls it a specified number of times with random 
start configurations. Many robots have ambiguities which can be resolved using the rw::invkin::AmbiguityResolver.

For use in planning the IK sampler interface (rw::pathplanning::QIKSampler) hides details
of selection of IK solver and start configurations for the solver. The
program below tests the default iterative IK solver for a device. The
program selects 10 random base to end transforms for a device using
the forward kinematics for the device. Using the default IK sampler,
the program then checks that an IK solution is found for all
transforms. Only a small number of start configurations are used for
each target transform, and therefore the IK sampler might not always
find an IK solution. If the IK sampler is constrained by the
requirement that the IK solutions must be collision free, then
solutions for only a subset of the target transforms are found.

\include ex-ik-reachable.cpp

\section sec_rw_manual_pointer_conventions C++ shared pointer conventions

The \b RobWork libraries make extensive use of non-copyable objects
(such as object referred to by interface) shared by pointer between
different algorithms. Ownership of objects is managed by the shared
pointer type rw::common::Ptr. If an object needs access to a
non-copyable object, the constructor of the object will conventionally
take a rw::common::Ptr type as parameter.

Classes that are commonly referred to by shared pointer define a
shortcut for this pointer type. If the class is named \e T, the name
of the pointer type will be \e TPtr, and the type of the pointer will
be rw::common::Ptr<T>:

\include ex-typedef-t-ptr.cpp

Here are some examples of these such pointer types:

- rw::math::QMetricPtr
- rw::models::WorkCellPtr
- rw::proximity::CollisionDetectorPtr
- rw::pathplanning::QSamplerPtr
- rw::pathplanning::QToQPlannerPtr

Here are some examples of constructor functions for such objects:

- rw::math::MetricFactory::makeEuclidean()
- rw::proximity::CollisionDetector::make()
- rw::pathplanning::QSampler::makeUniform()
- rwlibs::pathplanners::RRTPlanner::makeQToQPlanner()

The rw::common::Ptr type differs from standard shared pointer
implementations by allowing the pointer to refer to a stack allocated
object or another object for which an entity has already taken
the full ownership. To make the use of such objects easy, a
pointer to \e T can be implicitly converted to Ptr<T>, but the
implicitly constructed rw::common::Ptr type \e does \e not take
ownership of the object. If the rw::common::Ptr type should take
ownership of the entity, you must explicitly call the
rw::common::ownedPtr() function. This example illustrates the idiom:

\include ex-owned-ptr.cpp

In everyday programming, the construction of rw::common::Ptr types is
managed by the constructor functions for the various objects. Only if
you write your own extensions for interfaces in \b RobWork will you
need to explicitly call rw::common::ownedPtr().


\section sec_rw_manual_task RobWork Task Format
RobWork includes an abstract task format which can be used to represent, save and 
load tasks. The basic rwlibs::task::Task is templated and can either store 
rw::math::Q or rw::math::Transform3D as targets. 

A task in RobWork is basically a 2-tuple which can be described as
\code
Task={(Target)*, (Motion|Action|Task)*}
\endcode

The elements in a task are
- \b Target: Typically representing either a Cartesian pose or a robot configuration using rw::math::Transform3D and rw::math::Q, respectively.
- \b Motion: Describes a motion/transition between targets. A target may be shared among any number of motions.
- \b Action: Has no fixed interpretation and can be used to specify events such as open/close gripper, acquire image or as synchronization point.
- \b Task: Tasks are recursive. Subtasks may be shared among multiple tasks.


The example below illustrated how to construct a small task, prints out the task, saves it to file, reloads it and prints it once again.

\include ex-task.cpp

*/


