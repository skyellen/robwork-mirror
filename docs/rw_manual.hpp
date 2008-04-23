// -*- latex -*-

/* Manual meta-comments go here:

----------------------------------------------------------------------
Todo for rw:

general:

- Section on post installation and use: Libraries, include paths, more
  on namespace conventions, etc.

common:

- Finalize the log, assertion, warning, exception interface and show
  how to intercept those messages.

models:

- the most essential parts we have briefly discussed.

invkin:

- These solvers are not very robust, but we will show an example of
  something that mostly works and maybe IKMetaSolver also.

pathplanning:

- We don't have much within RobWork as such. Perhaps we should just
  finalize a simple interface, show the processing of a simple task,
  and show how a planner can be plugged into that interface.

proximity:

- Show how to construct a collision checker and what libraries to link
  to etc.

geometry:

- nothing to do here.

interpolator:

- this needs some rewriting before we can do any documenting.

loaders:

- We should show loading and storing of trajectories or paths that we
  can display in RobWorkStudio.

- tul format:

    - Example of simple, but complete TUL workcell.

    - Finalize the interface and show how to use user defined
      attributes with TUL files.

- xml format:

    - ...

sensor:

- nothing to do here.

task:

- The code needs to mature and integrate better with e.g. the
  interpolator classes and path planners.

----------------------------------------------------------------------
Todo for rwlibs:

algorithms:

- nothing to do here.

proximitystrategies:

- We need something else that Opcode here that we show how to use in
  section rw::proximity.

devices:

- We skip this for now.

drawable:

- Skip this. Not important for plain RobWorkStudio users.

io:

- Skip this.

pathplanners:

- Use a pathplanner in section rw::pathplanning.

sensors:

- Skip this.

lua:

- When the task data structures are mature, then show how to write
  task descriptions and more in Lua.

os:

- Nothing here.

pathoptimization:

- How well do these implementations work, and should we give an
  example of their use?

----------------------------------------------------------------------
*/

/**

\page page_rw_manual RobWork manual

- \ref sec_rw_manual_intro
- \ref sec_namespaces
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

- \ref page_rw_installation
- \ref page_tul
- \ref page_xml_workcell_format

\section sec_rw_manual_intro Introduction

All code examples of this manual are self-contained in the sense that
they will compile if placed in a C++ file of their own. The examples
are found in the \c RobWork/docs directory. See also the \c
CMakeLists.txt file of the \c RobWork/docs directory for the setup of
the compiler and linker flags.

\section sec_namespaces Namespaces

The header files of RobWork are distributed across a number of
directories each having its own namespace. You can import the
namespaces of the header files into a common namespace called
::robwork as follows:

\code
// Include header files:
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Frame.hpp>
// ...

// Use the robwork namespace:
#include <rw/use_robwork_namespace.hpp>
\endcode

You can then use <code>robwork::Frame</code> as an alias for
rw::kinematics::Frame, or you can open the entire ::robwork namespace
with

\code
using namespace robwork;
\endcode

We use this idiom throughout the manual: It is mightily convenient
compared to having to type in and remember the complete namespace
names.

Beware that you can not forward declare entities of ::robwork using the ::robwork
abbreviation, i.e. the following \e does \e not work:
\code
namespace robwork { class WorkCell; }
void f(const robwork::WorkCell& workcell);
\endcode

whereas this \e does work:

\code
#include <rw/models/WorkCell.hpp>
#include <rw/use_robwork_namespace.hpp>

void f(const robwork::WorkCell& workcell);
\endcode

\section sec_rw_manual_workcells Workcells

\subsection sec_rw_manual_load_workcell Loading a workcell

RobWork support workcells described in an XML format as well as in the
\ref page_tul ".wu and .dev tag file format" used by the TUL program.

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
stateless.

To illustrate these important ideas, this example shows how to print
the structure of the kinematic tree of the workcell and for each frame
print also the position of the frame in space:

\include ex-print-kinematic-tree.cpp

Here is an example of output produced by the
printDefaultWorkCellStructure() function for a CRSA465 workcell:

\verbatim
WorkCell[d:/src/RobWorkData/TULDevices/CRSA465/CRSA465.dev]
WORLD at Vector3D {0, 0, 0}
 base at Vector3D {0, 0, 0}
  led1 at Vector3D {0, 0, 0.33}
   led2 at Vector3D {0, 0, 0.33}
    led3 at Vector3D {0, 0, 0.635}
     led4 at Vector3D {0, 0, 0.968}
      led5 at Vector3D {0, 0, 0.968}
       led6 at Vector3D {0, 0, 1.044}
        TCP at Vector3D {0, 0, 1.044}
        Link6Geo at Vector3D {0, 0, 1.044}
       Link5Geo at Vector3D {0, 0, 0.968}
      Link4Geo at Vector3D {0, 0, 0.968}
     Link3Geo at Vector3D {0, 0, 0.635}
    Link2Geo at Vector3D {0, 0, 0.33}
   Link1Geo at Vector3D {0, 0, 0.33}
  Link0Geo at Vector3D {0, 0, 0}
\endverbatim

We see from this example that given a state, it is straight-forward to
compute the transform of every single frame in the workcell. RobWork
has some utilities to make calculation of forward kinematics
convenient in the day to day work.

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
scalar multiplication, inner product, etc.

Algorithms for devices may assume that except for the configuration of
the device, the state of the workcell stays fixed. A path-planner may
for example return a path in the form of a sequence of configurations
together with the common workcell state for which the planning was
done. When writing or using such algorithms you will often have
translate from a configuration for the device to a state of the
workcell. This is accomplished by the methods
rw::models::Device::setQ() and rw::models::Device::getQ(). This is
example shows to convert a sequence of configurations for a common
state into a sequence of states:

\include ex-get-state-path.cpp

Note that rw::models::Device::setQ() and rw::models::Device::getQ() do
not store a configuration within the device: The configuration is read
from and written to a state value. The device itself is stateless.

*/

/*
----------------------------------------------------------------------
Dead text

The work cell state contains for each frame a number of joint values
for the frame. You can read or write the joint values that belong to a
frame to or from a state with Frame::getQ() and Frame::setQ().

The work cell state contains also information about the structure of
the tree. By changing the work cell state one can release a frame from
the tree and attach it to a new parent frame. We give examples of
this in Section \ref sec_frame_attachments.

*/

/*

Here is how you can include example code in the manual:

\include ex-load-workcell.cpp

This is equivalent to the following:

\code
#include <string>

#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

... and so on ...
\endcode

*/
