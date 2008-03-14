// -*- latex -*-

/**

\page page_tul Tag workcell format

Workcell description files in the tag format have suffix \c .wu or \c
.dev. Conventionally the suffix \c .dev is used for files of a single
device (rw::models::Device) whereas files with suffix \c .wu contain
the setup for a complete workcell (rw::models::WorkCell) containing
the environment and a number of imported devices. Tag workcell files
are loaded with rw::loaders::WorkCellLoader::load().

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

In workcell descriptions each tag maps to a frame
(rw::kinematics::Frame) of the workcell. The name of the tag maps
(prefixed by the device name) to the name of the frame. Here is an
example of a typical tag for a frame:

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

\section sec_tul_attributes Built-in attributes

List of built-in attributes in alphabetic order:

- \ref page_tul_ActiveJoint
- \ref page_tul_CollisionModelID
- \ref page_tul_CollisionSetup
- \ref page_tul_CompositeDevice
- \ref page_tul_DAF
- \ref page_tul_Device
- \ref page_tul_DrawableID
- \ref page_tul_Fixed
- \ref page_tul_GeoID
- \ref page_tul_GeoScale
- \ref page_tul_IJK
- \ref page_tul_JointAccLimit
- \ref page_tul_JointHomePos
- \ref page_tul_JointPosLimit
- \ref page_tul_JointVelLimit
- \ref page_tul_Movable
- \ref page_tul_PassivePrismatic
- \ref page_tul_PassiveRevolute
- \ref page_tul_Position
- \ref page_tul_Prismatic
- \ref page_tul_RPY
- \ref page_tul_ReferenceFrame
- \ref page_tul_Revolute
.

\subsection page_tul_ActiveJoint ActiveJoint

- \b ActiveJoint
  \n\n
  The frame is a joint (rw::models::Joint). The actual subtype of
  the joint (rw::models::FixedJoint, rw::models::PrismaticJoint or
  rw::models::RevoluteJoint) is set by the \ref page_tul_Fixed, \ref
  page_tul_Revolute or \ref page_tul_Prismatic attribute.

\subsection page_tul_CollisionModelID CollisionModelID

- \b CollisionModelID \e id
  \n\n
  ID or file name for a CAD geometry to use for collision checking
  exclusively (see also attributes \ref page_tul_DrawableID and \ref
  page_tul_GeoID).

\subsection page_tul_CollisionSetup CollisionSetup

- \b CollisionSetup \e file-name
  \n\n
  Use file \e file-name as collision checking setup for the workcell
  (rw::proximity::CollisionSetup). If more than one \b CollisionSetup
  attribute is found in a workcell description, the contents of the
  files are merged into a single collision setup.

\subsection page_tul_CompositeDevice CompositeDevice

- \b CompositeDevice \e dev1 \e dev2 ... \e devN
  \n\n
  Construct a composite device (rw::models::CompositeDevice) from
  devices the named \e dev1 \e dev2 ... \e devN. The \b
  CompositeDevice attribute may refer also to devices that are only
  loaded later in the workcell file. The base of the composite device
  is current frame and the end frame is the end frame of device \e dev1.
  See also attribute \ref page_tul_Device.

\subsection page_tul_DAF DAF

- \b DAF
  \n\n
  The frame is a \e dynamically \e attachable \e frame (DAF), meaning
  that the parent of the frame can be changed
  (rw::kinematics::Frame::attachFrame()).

\subsection page_tul_Device Device

- \b Device \e device-file
  \n\n
  Load the device of file \e device-file and attach the base frame of
  the device to this frame. The name of the device
  (rw::models::Device::getName()) is the name of the frame. The frames
  names of the device are prefixed with the name of the device.

\subsection page_tul_DrawableID DrawableID

- \b DrawableID \e id
  \n\n
  ID or file name for a CAD geometry to use for display exclusively (see
  also attributes \ref page_tul_CollisionModelID and \ref page_tul_GeoID).

\subsection page_tul_Fixed Fixed

- \b Fixed
  \n\n
  The frame is a fixed joint (rw::models::FixedJoint). This
  attribute is used in combination with attribute \ref page_tul_ActiveJoint.

\subsection page_tul_GeoID GeoID

- \b GeoID \e id
  \n\n
  ID or file name for a CAD geometry to use for display as well as
  collision checking (see also attributes \ref
  page_tul_CollisionModelID and \ref page_tul_DrawableID).

\subsection page_tul_GeoScale GeoScale

- \b GeoScale \e scale-factor
  \n\n
  Factor by which to scale the CAD geometries loaded for the
  frame.

\subsection page_tul_IJK I, J, K

- \b I (\e x, \e y, \e z)
- \b J (\e x, \e y, \e z)
- \b K (\e x, \e y, \e z)
  \n\n
  The attributes \b I, \b J and \b K together specify the rotation of
  the frame relative to its parent. The values for \b I, \b J and \b K
  are respectively the first, second and third column of the rotation
  matrix. If no rotation is specified for the frame (see also
  attribute \ref page_tul_RPY) the relative rotation of the frame
  defaults to zero.

\subsection page_tul_JointAccLimit JointAccLimit

- \b JointAccLimit \e max-limit
  \n\n
  The acceleration limit (rw::models::Joint::getMaxAcceleration()) for
  the joint (see attribute \ref page_tul_ActiveJoint). For revolute
  joints (rw::models::RevoluteJoint) the acceleration limit is given in
  degrees per second per second.

\subsection page_tul_JointHomePos JointHomePos

- \b JointHomePos \e value
  \n\n
  The value of the joint (see attribute \ref page_tul_ActiveJoint) in the initial
  state of the workcell (rw::models::WorkCell::getDefaultState()).

\subsection page_tul_JointPosLimit JointPosLimit

- \b JointPosLimit \e lower-limit \e upper-limit
  \n\n
  The upper and lower joint limits (rw::models::Joint::getBounds()) of
  the joint (see attribute \ref page_tul_ActiveJoint). For revolute
  joints (rw::models::RevoluteJoint) the joint limits are given in
  degrees.

\subsection page_tul_JointVelLimit JointVelLimit

- \b JointVelLimit \e max-limit
  \n\n
  The velocity limit (rw::models::Joint::getMaxVelocity()) for
  the joint (see attribute \ref page_tul_ActiveJoint). For revolute
  joints (rw::models::RevoluteJoint) the velocity limit is given in
  degrees per second.

\subsection page_tul_Movable Movable

- \b Movable
  \n\n
  The frame is of type MovableFrame (rw::kinematics::MovableFrame).

\subsection page_tul_PassivePrismatic PassivePrismatic

- \b PassivePrismatic \e frame-name \e scale \e offset
  \n\n
  The frame is a passive prismatic frame
  (rw::models::PassivePrismaticFrame). The frame is controlled by
  frame \e frame-name. Frame \e frame-name must have been given
  previously in the workcell description. Scale and offset parameters
  for the passive frame are given by \e scale and \e offset.

\subsection page_tul_PassiveRevolute PassiveRevolute

- \b PassiveRevolute \e frame-name \e scale \e offset
  \n\n
  The frame is a passive revolute frame
  (rw::models::PassiveRevoluteFrame). The frame is controlled by frame
  \e frame-name. Frame \e frame-name must have been given previously
  in the workcell description. Scale and offset parameters for the
  passive frame are given by \e scale and \e offset.

\subsection page_tul_Position Position

- \b Position (\e x, \e y, \e z):
  \n\n
  The position of the frame relative to its parent. If no position is
  given the relative position of the frame defaults to zero.

\subsection page_tul_RPY RPY

- \b RPY (\e roll, \e pitch, \e yaw)
  \n\n
  The rotation of the frame relative to its parent in (roll, pitch,
  yaw) angles (rw::math::RPY). Angles are given in degrees. If no
  rotation is specified for the frame (see also the \ref page_tul_IJK
  attributes) the relative rotation of the frame defaults to zero.

\subsection page_tul_ReferenceFrame ReferenceFrame

- \b ReferenceFrame \e frame-name
  \n\n
  The parent of the frame is \e frame-name
  (rw::kinematics::Frame::getParent()). The frame named \e frame-name
  must have been processed previously in the workcell description. If
  no \b ReferenceFrame attribute is given, the parent frame defaults
  to the world frame (rw::models::WorkCell::getWorldFrame()). If \e
  frame-name in a device file is the empty string \c "", the frame
  will be attached to the frame from which the device is loaded (see
  attribute \ref page_tul_Device).

\subsection page_tul_Revolute Revolute

- \b Revolute
  \n\n
  The frame is a revolute joint (rw::models::RevoluteJoint). This
  attribute is used in combination with attribute \ref page_tul_ActiveJoint.

\subsection page_tul_Prismatic Prismatic

- \b Prismatic
  \n\n
  The frame is a prismatic joint (rw::models::PrismaticJoint). This
  attribute is used in combination with attribute \ref page_tul_ActiveJoint.

*/

/*
  Things we should support later on:

  - DeviceHomePos

*/
