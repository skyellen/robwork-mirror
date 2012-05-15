// -*- latex -*-

/**

\page page_xml_workcell_format RW XML File Format

- \ref sec_rwxml_intro
- \ref sec_rwxml_format
- \ref sec_rwxml_elements
	- \ref sec_rwxml_workcell
	- \ref sec_rwxml_device
		- \ref sec_rwxml_serialdevice
		- \ref sec_rwxml_treedevice
		- \ref sec_rwxml_paralleldevice
		- \ref sec_rwxml_mobiledevice
	- \ref sec_rwxml_frame
		- \ref sec_rwxml_joint
		- \ref sec_rwxml_dhjoint
	- \ref sec_rwxml_drawable
	- \ref sec_rwxml_collisionmodel
	- \ref sec_rwxml_property
	- \ref sec_rwxml_polytope
	- \ref sec_rwxml_transform


\section sec_rwxml_intro Introduction
 The workcell XML file format have suffix \c .xml and follow the rules of standard XML.
\ref sec_rw_manual_load_workcell "Tag workcell files are loaded"
with rw::loaders::WorkCellLoader::load().

The workcell file describes workcell elements such as frames, joints, DAFs, devices and the
properties that relate to these elements such as name, parent name, joint limits,
position and orientation,
collision geometry, drawing geometry and even user defined properties.

 The basic structure of a workcell is frames. Frames are connected in a tree like fasion
 such that each frame has a parent and [0;many] children. The root of the frame tree is
 called the world frame and this frame is the only frame that has no parent. The world frame
 is named \b WORLD.

 Frames come in many different types: fixed frame, movable frame, prismatic joint, revolute
 joint, and so on. They can be freely defined and connected in the workcell file, though often
 frames will be grouped into devices.

 A device is a robot with one or more joints. It defines a scope where frames that belong to that
 device can be described. A device always has a base frame which is the frame belonging to the
 device that has a parent that does not belong to the device. That is, a device defines a sub tree
 in the frame tree. There exists severel device types: serial device, tree device, parallel device
 and so on, that allow different sort of connections
 between the joints of the device.

 User properties can be attached to any frame. A property has a name and a string value. The
 property is saved in the frame such that the user can retreve it and parse the string himself.

 A collision setup is a set of include or exclude rules that tell which frame pairs
 that should not or should be tested for collision. Ex. two neighboring links on a
 robot will usually collide in their attachment points, which is undesirable, the
 collision setup should in this case be used to exclude those frames from collision
 checking.

\section sec_rwxml_scopes Scopes

 An important thing to notice is that some xml elements are scope dependent. In
 general the \b refframe and \b refjoint attributes are optional and if not
 defined then scope rules will define them instead.

 Ex. when defining a frame in workcell or a device scope the frame will automaically be
 attached to the previusly defined frame.
 \verbatim
 <WorkCell name="testwc">
 ...
 <Frame name="A" refframe="WORLD" />

 <Frame name="B" />

 <Frame name="C" refframe="A" />

 ...
 </WorkCell>
 \endverbatim
 in the above example A attaches to world and B and C attaches to A.

 Another example illustrates how properties defined inside
 a frame can omit the refframe attribute.

 \verbatim
 <WorkCell name="testwc">
 ...
 <Frame name="A" refframe="WORLD">
     <Property name="A_Property"> some string value</Property>
 </Frame>

 <Property name="A_nother_Property" refframe="A">some string value</Property>
 ...
 </WorkCell>
 \endverbatim

\section sec_rwxml_format RW XML Format
 Before going deep into the grammar of the rw xml format some common structure need be
explained. The complete kinematic description is based on a construct named Frame.
 The Frame has a name and a transform relative to its parent frame or refframe.
This means that a kinematic
Frame tree can be described with multiple frames. The Frame can be of different
types the simplest being Fixed, which means that the frame transform is unchangeable.

 Another common structure is the Property. The property is linket to a frame and has a
name, description and a string value. Properties are used to link different
information to frames. F.ex. collision models and drawables are described using properties.
properties are also used to allow users to attach user specific information to frames.

 In the xml file format Frames are grouped logicly in container type elements. These elements
are WorkCell and device types. To avoid name clashes frames belonging to a container type will
have the container name prepended. Example: a frame named "base" specified in a device named
"PA10" in a workcell named "scene" will have the unique name "scene.PA10.base".

 The WorkCell element is the root element in the fileformat. It implicitly defines a Fixed frame
named World. This world frame is the root frame in the kinematic frame tree.

workcell
  *(frame
   | Drawable
   | SerialDevice
   | TreeDevice
   | ParallelDevice
   | MobileDevice
   | CollisionSetup )


\section sec_rwxml_elements XML Elements

\subsection sec_rwxml_workcell WorkCell

\b Element WorkCell

\b Attributes
- \b name: a string identifying the workcell.

\b Child elements:
The DeviceType is any of the device types that can be defined,
see section \ref sec_rwxml_device .

\verbatim
rule := *( DeviceType | Frame | Joint | DHJoint | CollisionModel | Drawable | CollisionSetup )
\endverbatim

\b Example
\verbatim
<WorkCell name="scene">
...
</WorkCell>
\endverbatim

\subsection sec_rwxml_device Device
 The different device types are much alike when considering the child elements
that they allow. Though they vary somehow in the implicit rules of frame
attachment.

 In general a device defines a scope. This scope has the same name as the
 device. Any frames defined inside the scope of a device gets the device
 name appended. Ex. given a device "dev" and a frame "base" in the device
 the complete frame name becomes: "dev.base"

\subsubsection sec_rwxml_serialdevice SerialDevice
The serial device only allows joints to be connected in a serial chain. And
it also only allows one single endeffector.

\b Element SerialDevice

\b Attributes
- \b name: a string identifying the device.

\b Child elements:
- \b Frame
- \b Joint
- \b DHJoint
- \b CollisionModel
- \b Drawable
- \b PosLimit
- \b VelLimit
- \b AccLimit
- \b Property
- \b SerialChain
- \b CollisionSetup
- \b Q

\b Example
\verbatim
<SerialDevice name="RobotArm">
...
</SerialDevice>
\endverbatim

\subsubsection sec_rwxml_treedevice TreeDevice
The tree device allows joints to be connected in a tree like structure. And
it also allows for multiple endeffectors.

\b Element TreeDevice

\b Attributes
- \b name: a string identifying the device.

\b Child elements:
- \b Frame
- \b Joint
- \b DHJoint
- \b CollisionModel
- \b Drawable
- \b PosLimit
- \b VelLimit
- \b AccLimit
- \b Property
- \b SerialChain
- \b CollisionSetup
- \b Q

\b Example
\verbatim
<TreeDevice name="RobotHand">
...
</TreeDevice>
\endverbatim

\subsubsection sec_rwxml_paralleldevice ParallelDevice
The parallel device is like a number of serial devices (with same base) with all endeffectors
rigidly connected together. The initial configuration of the robot is required
to make all endeffectors align in the same pose.

\b Attributes
- \b name: a string identifying the device.

\b Child elements:
- \b Frame
- \b Joint
- \b DHJoint
- \b CollisionModel
- \b Drawable
- \b PosLimit
- \b VelLimit
- \b AccLimit
- \b Property
- \b SerialChain
- \b CollisionSetup
- \b Q

\b Example
\verbatim
<ParallelDevice name="RobotHand">
...
</ParallelDevice>
\endverbatim

\subsubsection sec_rwxml_mobiledevice MobileDevice
The mobile device defines a two wheeled mobile robot where the two
wheels are on the same axel displaced from the center of the axel with
some width \b AxelWidth.

\b Attributes
- \b name: a string identifying the device.
- \b basename: name of the mobile device base.

\b Child elements:
- \b AxelWidth
- \b LeftWheel
- \b RightWheel

- \b Frame
- \b Joint
- \b DHJoint
- \b CollisionModel
- \b Drawable
- \b PosLimit
- \b VelLimit
- \b AccLimit
- \b Property
- \b SerialChain
- \b CollisionSetup
- \b Q

\b Example
\verbatim
<MobileDevice name="Pioneer" basename="Base">
...
</MobileDevice>
\endverbatim

\subsection sec_rwxml_frame Frame

\b Attributes
- \b name: a string identifying the frame.
- \b refframe: name of the parent frame (optional).
- \b type: a frame type identifier (optional).
- \b daf: boolean defining if the frame is a daf or not (optional)

\b Child elements

!((\b RPY >> \b Pos) | \b Transform) >> *(\b Property | \b CollisionModel | \b Drawable)

\b Example
\verbatim
<Frame name="myframe" refframe="WORLD">

</Frame>
\endverbatim

\subsubsection sec_rwxml_joint Joint

\b Attributes
- \b name: a string identifying the frame.
- \b refframe: name of the parent frame (optional).
- \b type: a joint type identifier (Prismatic|Revolute).
- \b state: joint state (optional)

\b Child elements

!((\b RPY >> \b Pos) | \b Transform)
>> *(\b PosLimit | \b VelLimit | \b AccLimit
| Depend
| \b Property | \b CollisionModel | \b Drawable)

\b Example

\subsubsection sec_rwxml_dhjoint DHJoint
A joint that is defined from the Denavit Hartenberg notation.
The Craig DH variant is used. This can only specify Revolute
or Prismatic joints

\b Attributes
- \b name: a string identifying the frame.
- \b alpha:
- \b a:
- \b d:
- \b theta:
- \b offset:
- \b state: joint state (optional)

\verbatim
rule := name >> alpha >> a >> (d >> offset)|(theta >> offset);
\endverbatim

\b Child elements

!((\b RPY >> \b Pos) | \b Transform)
>> *(\b PosLimit | \b VelLimit | \b AccLimit
| \b Depend
| \b Property | \b CollisionModel | \b Drawable)

\b Example

\subsection sec_rwxml_drawable Drawable

\b Attributes
- \b name: the name of the drawable
- \b refframe: the frame that the drawable is to be attached to.
- \b colmodel: (Enabled|Disabled) if enabled the drawable will also
be used as collision model.

\b Child Elements

!((\b RPY >> \b Pos) | \b Transform)
>> *(Polytope | Sphere | Box | Cone | Cylinder)

\b Example

\subsection sec_rwxml_collisionmodel CollisionModel

\b Attributes
- \b name:
- \b refframe:

\b Child Elements

!((\b RPY >> \b Pos) | \b Transform)
>> *(Polytope | Sphere | Box | Cone | Cylinder)

\b Example

\subsection sec_rwxml_property Property
\b Attributes
- \b name: name of the property
- \b reframe: frame to attach property to (optional)
- \b desc: description of the property (optional)

\b Child Elements
- string value

\b Example
A user property for defining a camera on a frame. The string value
can be parsed by the user to get image dimensions [640;480] and field
of view 40
\verbatim
<Property name="Camera" refframe="WORLD" desc="[width,hwight,fovy]">
    640 480 40
</Property>
\endverbatim

\subsection sec_rwxml_transform Transform
\b Attributes

\b Child Elements
All real values are parsed into a rotation matrix \b R and a position \b P
\verbatim
R00 >> R01 >> R02 >> P0 >>
R10 >> R11 >> R12 >> P1 >>
R01 >> R21 >> R22 >> P2
\endverbatim

\b Example
Loads the identity rotation and the (0.1,0.1,0.1) position. Values are
seperated by whitespace.

\verbatim
<Transform>
1 0 0 0.1
0 1 0 0.1
0 0 1 0.1
</Transform>
\endverbatim

\subsection sec_rwxml_rpy RPY
\b Attributes

\b Child Elements
Loads RPY values seperated by whitespace
\verbatim
roll >> pitch >> yaw
\endverbatim

\b Example
A rotation matrix with 90 degree rotation around z-axis

\verbatim
<RPY> 90 0 0 </RPY>
\endverbatim

\subsection sec_rwxml_pos Pos
\b Attributes

\b Child Elements
Loads pos values seperated by whitespace
\verbatim
x >> y >> z
\endverbatim

\b Example
\verbatim
<Pos> 0.1 0.1 0.2 </Pos>
\endverbatim


\subsection sec_rwxml_polytope Polytope
\b Attributes
- \b file: the geometry file

\b Example
\verbatim
<Polytope file="c:/geometry/object.stl" />
\endverbatim

\subsection sec_rwxml_sphere Sphere
\b Attributes
- \b radius: radius of the sphere in m

\b Example
\verbatim
<Sphere radius="0.05" />
\endverbatim

\subsection sec_rwxml_box Box
\b Attributes
- \b x: length in x-axis
- \b y: length in y-axis
- \b z: length in z-axis

\b Example
\verbatim
<Box x="0.1" y="0.1" z="0.1" />
\endverbatim


\subsection sec_rwxml_cone Cone
\b Attributes
- \b radius: radius of bottom circle of cone.
- \b z: height of cone

\b Example
\verbatim
<Cone radius="0.1" z="0.1" />
\endverbatim

\subsection sec_rwxml_cylinder Cylinder
\b Attributes
- \b radius: radius of the cylinder.
- \b z: length of zylinder

\b Example
\verbatim
<Cylinder radius="0.1" z="0.1" />
\endverbatim

\subsection sec_rwxml_collisionsetup CollisionSetup
\b Attributes
- \b file: the file where the collision setup is described

\b Example
\verbatim
<CollisionSetup file="../mydevice/colsetup.xml" />
\endverbatim

\subsection sec_rwxml_poslimit PosLimit
\b Attributes
- \b refjoint: the joint which the limit is valid for. (optional)
- \b min: the minimum joint value
- \b max: the maximum joint value

\b Example
\verbatim
<PosLimit refjoint="joint1" min="-90" max="90" />
\endverbatim

\subsection sec_rwxml_vellimit VelLimit
\b Attributes
- \b refjoint: the joint which the limit is valid for. (optional)
- \b max: the maximum joint velocity value

\b Example
\verbatim
<VelLimit refjoint="joint1" max="180" />
\endverbatim

\subsection sec_rwxml_acclimit AccLimit
\b Attributes
- \b refjoint: the joint which the limit is valid for. (optional)
- \b max: the maximum joint acceleration

\b Example
\verbatim
<AccLimit refjoint="joint1" max="180" />
\endverbatim


*/
