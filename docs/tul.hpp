// -*- latex -*-

/**

\page page_tul Tag workcell setup format

- \ref sec_tul_intro
- \ref sec_tul_format
- \ref sec_tul_geometric_primitives
- \ref sec_tul_attributes
  - \ref sec_tul_ActiveJoint
  - \ref sec_tul_CollisionModelID
  - \ref sec_tul_CollisionSetup
  - \ref sec_tul_CompositeDevice
  - \ref sec_tul_DAF
  - \ref sec_tul_Device
  - \ref sec_tul_DrawableID
  - \ref sec_tul_Fixed
  - \ref sec_tul_GeoID
  - \ref sec_tul_GeoScale
  - \ref sec_tul_IJK
  - \ref sec_tul_JointAccLimit
  - \ref sec_tul_JointHomePos
  - \ref sec_tul_JointPosLimit
  - \ref sec_tul_JointVelLimit
  - \ref sec_tul_Movable
  - \ref sec_tul_PassivePrismatic
  - \ref sec_tul_PassiveRevolute
  - \ref sec_tul_Position
  - \ref sec_tul_Prismatic
  - \ref sec_tul_RPY
  - \ref sec_tul_ReferenceFrame
  - \ref sec_tul_Revolute
  .
.

\section sec_tul_intro Introduction

Workcell setup files in the tag format have suffix \c .wu or \c .dev.
Conventionally a \c .dev file is used for a single device
(rw::models::Device) whereas a \c .wu file is used for a complete
workcell (rw::models::WorkCell) containing the environment and a
number of imported devices.
\ref sec_rw_manual_load_workcell "Tag workcell files are loaded"
with rw::loaders::WorkCellLoader::load().

\section sec_tul_format Tag format

A \e tag file contains a sequence of \e tags. Every tag has a name and
each name must be unique. A tag contains zero or more attributes that
each take up one line. The first word on an attribute line is the name
of the attribute, and the elements following the name are the
attribute values. The following types of attribute values are
supported:

- Strings (<code>std::string</code>): For example \c "Hi"

- Numbers (<code>double</code>): For example \c 12 and \c 42.0.

- Configurations (rw::math::Q): For example (\c 12) and (\c 3.1, \c 4, \c 15).

The following is an example of a syntactically valid tag:

\verbatim
{ "demo tag" ! A tag named 'demo tag'

    ! Two attributes with mixed types of attribute values:
    StringsAndNumbers "one" 1 "two" 2 "three" 3
    DofsAndQs 1 (12) 2 (54, 12)

    ! An attribute with no attribute values:
    Hello
}
\endverbatim

In workcell descriptions based on the tag format each tag maps to a
frame (rw::kinematics::Frame) of the workcell. The name of the tag
maps (prefixed by the device name) to the name of the frame. Here is
an example of a typical tag for a frame:

\verbatim
{ "Obstacle" ! The name of the frame.

    ! The transform and parent of the frame:
    Position (0.8, -0.5, -0.4)
    RPY (0, 0, 0)
    ReferenceFrame "WORLD"

    ! CAD geometry for collision checking and display:
    GeoID "Geometry/obstacle"
}
\endverbatim

The name of the frame is \c "Obstacle". Its parent frame is the frame
named \c "WORLD" which is defined to be the world frame
(rw::models::WorkCell::getWorldFrame()). The position and orientation
of the frame relative to its parent is given by the \b Position and \b
RPY attributes. The contents of the CAD file \c "Geometry/obstacle" is
used for display and collision checking for the frame. The loader will
add a suitable suffix so that e.g. \c "Geometry/obstacle.stl" is used
for collision checking and \c "Geometry/obstacle.3ds" is used for
display.

\section sec_tul_geometric_primitives Geometric primitives

The tag attributes \ref sec_tul_GeoID, \ref
sec_tul_CollisionModelID, and \ref sec_tul_DrawableID each take an
identifier for a CAD geometry as argument. The identifier can either
be the name of the file of a CAD geometry, or it can be a string
describing a geometric primitive.

The supported geometric primitives are:

- \b \#Box \e dx \e dy \e dz
  \n\n
  A box of with x-, y-, z-dimensions \e dx, \e dy, and \e dz. The
  box is centered at the position of the frame.

- \b \#Cylinder \e radius \e height \e level
  \n\n
  A cylinder of the given radius and height and approximated by \e
  level faces. The cylinder is centered at the position of the frame.
  The axis of the cylinder is in the direction of the z-axis of the
  frame.
.

This is a complete workcell showing the use of geometric primitives:

\include geometric-primitives.wu

This is what the workcell looks like when loaded into RobWorkStudio:

<center>
<table>
  <tr>
    <td><img src="../geometric-primitives.png" alt="Workcell
    environment built from geometric primitives"></td>
  </tr>
  <tr>
    <td><center>Workcell environment built from geometric
    primitives</center></td>
  </tr>
</table>
</center>

\section sec_tul_attributes Built-in attributes

\subsection sec_tul_ActiveJoint ActiveJoint

- \b ActiveJoint
  \n\n
  The frame is a joint (rw::models::Joint). The actual subtype of
  the joint (rw::models::FixedJoint, rw::models::PrismaticJoint or
  rw::models::RevoluteJoint) is set by the \ref sec_tul_Fixed, \ref
  sec_tul_Revolute or \ref sec_tul_Prismatic attribute.

\subsection sec_tul_CollisionModelID CollisionModelID

- \b CollisionModelID \e id
  \n\n
  Geometric primitive or file name for a CAD geometry to use for collision checking
  exclusively (see also attributes \ref sec_tul_DrawableID and \ref
  sec_tul_GeoID and section \ref sec_tul_geometric_primitives).

\subsection sec_tul_CollisionSetup CollisionSetup

- \b CollisionSetup \e file-name
  \n\n
  Use file \e file-name as collision checking setup for the workcell
  (rw::proximity::CollisionSetup). If more than one \b CollisionSetup
  attribute is found in a workcell description, the contents of the
  files are merged into a single collision setup.

\subsection sec_tul_CompositeDevice CompositeDevice

- \b CompositeDevice \e dev1 \e dev2 ... \e devN
  \n\n
  Construct a composite device (rw::models::CompositeDevice) from
  devices the named \e dev1 \e dev2 ... \e devN. The \b
  CompositeDevice attribute may refer also to devices that are only
  loaded later in the workcell file. The base of the composite device
  is current frame and the end frame is the end frame of device \e dev1.
  See also attribute \ref sec_tul_Device.

\subsection sec_tul_DAF DAF

- \b DAF
  \n\n
  The frame is a \e dynamically \e attachable \e frame (DAF), meaning
  that the parent of the frame can be changed
  (rw::kinematics::Frame::attachFrame()).

\subsection sec_tul_Device Device

- \b Device \e device-file
  \n\n
  Load the device of file \e device-file and attach the base frame of
  the device to this frame. The name of the device
  (rw::models::Device::getName()) is the name of the frame. The frames
  names of the device are prefixed with the name of the device.

\subsection sec_tul_DeviceHomePos DeviceHomePos

- \b DeviceHomePos \e q
  \n\n
  The configuration of the device in the initial state of the workcell
  (rw::models::getDefaultState()). Attribute \b DeviceHomePos has
  precedence over attribute \ref sec_tul_JointHomePos. Later \b
  DeviceHomePos attributes take precedence over earlier \b
  DeviceHomePos attributes that assign values for the same joints. The
  attribute is meaningful only in the tag from which the device was
  loaded (see attribute \ref sec_tul_Device or \ref
  sec_tul_CompositeDevice).

\subsection sec_tul_DrawableID DrawableID

- \b DrawableID \e id
  \n\n
  Geometric primitive or file name for a CAD geometry to use for display exclusively (see
  also attributes \ref sec_tul_CollisionModelID and \ref
  sec_tul_GeoID and section \ref sec_tul_geometric_primitives).

\subsection sec_tul_Fixed Fixed

- \b Fixed
  \n\n
  The frame is a fixed joint (rw::models::FixedJoint). This
  attribute is used in combination with attribute \ref sec_tul_ActiveJoint.

\subsection sec_tul_GeoID GeoID

- \b GeoID \e id
  \n\n
  Geometric primitive or file name for a CAD geometry to use for display as well as
  collision checking (see also attributes \ref
  sec_tul_CollisionModelID and \ref sec_tul_DrawableID and section
  \ref sec_tul_geometric_primitives).

\subsection sec_tul_GeoScale GeoScale

- \b GeoScale \e scale-factor
  \n\n
  Factor by which to scale the CAD geometries loaded for the
  frame.

\subsection sec_tul_IJK I, J, K

- \b I (\e x, \e y, \e z)
- \b J (\e x, \e y, \e z)
- \b K (\e x, \e y, \e z)
  \n\n
  The attributes \b I, \b J and \b K together specify the rotation of
  the frame relative to its parent. The values for \b I, \b J and \b K
  are respectively the first, second and third column of the rotation
  matrix. If no rotation is specified for the frame (see also
  attribute \ref sec_tul_RPY) the relative rotation of the frame
  defaults to zero.

\subsection sec_tul_JointAccLimit JointAccLimit

- \b JointAccLimit \e max-limit
  \n\n
  The acceleration limit (rw::models::Joint::getMaxAcceleration()) for
  the joint (see attribute \ref sec_tul_ActiveJoint). For revolute
  joints (rw::models::RevoluteJoint) the acceleration limit is given in
  degrees per second per second.

\subsection sec_tul_JointHomePos JointHomePos

- \b JointHomePos \e value
  \n\n
  The value of the joint (see attribute \ref sec_tul_ActiveJoint) in the initial
  state of the workcell (rw::models::WorkCell::getDefaultState()).

\subsection sec_tul_JointPosLimit JointPosLimit

- \b JointPosLimit \e lower-limit \e upper-limit
  \n\n
  The upper and lower joint limits (rw::models::Joint::getBounds()) of
  the joint (see attribute \ref sec_tul_ActiveJoint). For revolute
  joints (rw::models::RevoluteJoint) the joint limits are given in
  degrees.

\subsection sec_tul_JointVelLimit JointVelLimit

- \b JointVelLimit \e max-limit
  \n\n
  The velocity limit (rw::models::Joint::getMaxVelocity()) for
  the joint (see attribute \ref sec_tul_ActiveJoint). For revolute
  joints (rw::models::RevoluteJoint) the velocity limit is given in
  degrees per second.

\subsection sec_tul_Movable Movable

- \b Movable
  \n\n
  The frame is of type MovableFrame (rw::kinematics::MovableFrame).

\subsection sec_tul_PassivePrismatic PassivePrismatic

- \b PassivePrismatic \e frame-name \e scale \e offset
  \n\n
  The frame is a passive prismatic frame
  (rw::models::PassivePrismaticFrame). The frame is controlled by
  frame \e frame-name. Frame \e frame-name must have been given
  previously in the workcell description. Scale and offset parameters
  for the passive frame are given by \e scale and \e offset.

\subsection sec_tul_PassiveRevolute PassiveRevolute

- \b PassiveRevolute \e frame-name \e scale \e offset
  \n\n
  The frame is a passive revolute frame
  (rw::models::PassiveRevoluteFrame). The frame is controlled by frame
  \e frame-name. Frame \e frame-name must have been given previously
  in the workcell description. Scale and offset parameters for the
  passive frame are given by \e scale and \e offset.

\subsection sec_tul_Position Position

- \b Position (\e x, \e y, \e z):
  \n\n
  The position of the frame relative to its parent. If no position is
  given the relative position of the frame defaults to zero.

\subsection sec_tul_RPY RPY

- \b RPY (\e roll, \e pitch, \e yaw)
  \n\n
  The rotation of the frame relative to its parent in (roll, pitch,
  yaw) angles (rw::math::RPY). Angles are given in degrees. If no
  rotation is specified for the frame (see also the \ref sec_tul_IJK
  attributes) the relative rotation of the frame defaults to zero.

\subsection sec_tul_ReferenceFrame ReferenceFrame

- \b ReferenceFrame \e frame-name
  \n\n
  The parent of the frame is \e frame-name
  (rw::kinematics::Frame::getParent()). The frame named \e frame-name
  must have been processed previously in the workcell description. If
  no \b ReferenceFrame attribute is given, the parent frame defaults
  to the world frame (rw::models::WorkCell::getWorldFrame()). If \e
  frame-name in a device file is the empty string \c "", the frame
  will be attached to the frame from which the device is loaded (see
  attribute \ref sec_tul_Device).

\subsection sec_tul_Revolute Revolute

- \b Revolute
  \n\n
  The frame is a revolute joint (rw::models::RevoluteJoint). This
  attribute is used in combination with attribute \ref sec_tul_ActiveJoint.

\subsection sec_tul_Prismatic Prismatic

- \b Prismatic
  \n\n
  The frame is a prismatic joint (rw::models::PrismaticJoint). This
  attribute is used in combination with attribute \ref sec_tul_ActiveJoint.

*/
