Dynamic WorkCell XML File Format  {#page_xml_dynamicworkcell_format}
==================

[TOC]

# Introduction # {#sec_rwsimxml_intro}
 The dynamic workcell (DWC) XML file format have suffix \c .dwc.xml and follow the rules of standard XML.
See also the \ref sec_rwsim_xml_fileformat "section about dynamic workcells" in the manual.
A dynamic workcell can be loaded with rwsim::loaders::DynamicWorkCellLoader::load() :

	DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load("DynamicWorkCellFile.dwc.xml");

The dynamic workcell file referes to an existing workcell file, and adds dynamic concepts
such as dynamic bodies, dynamic devices, mass, inertia, material properties, constraints,
gravity, controllers, and sensors.

The DynamicWorkCell element is the root element in the fileformat and refers to the underlying WorkCell file:

	<DynamicWorkcell workcell="pih.wc.xml">
		...
	</DynamicWorkcell>

# XML Elements # {#sec_rwsimxml_elements}
In the following the different possible elements in the DWC format will be described.

## DynamicWorkCell ## {#sec_rwsimxml_dynamicworkcell}

\b Element DynamicWorkCell

\b Attributes
- \b workcell: a string identifying the workcell file.

\b Child elements:
- \ref sec_rwsimxml_include
- \ref sec_rwsimxml_physicsengine
- \ref sec_rwsimxml_gravity
- \ref sec_rwsimxml_materialdata
- \ref sec_rwsimxml_frictionmap
- \ref sec_rwsimxml_objecttypedata
- \ref sec_rwsimxml_contactmap
- \ref sec_rwsimxml_fixedbody
- \ref sec_rwsimxml_kinematicbody
- \ref sec_rwsimxml_rigidbody
- \ref sec_rwsimxml_kinematicdevice
- \ref sec_rwsimxml_rigiddevice
- \ref sec_rwsimxml_suctioncup
- \ref sec_rwsimxml_constraints
- \ref sec_rwsimxml_springs
- \ref sec_rwsimxml_tactilearraysensor
- \ref sec_rwsimxml_bodycontactsensor
- \ref sec_rwsimxml_tactilemultiaxissensor
- \ref sec_rwsimxml_ftsensor
- \ref sec_rwsimxml_pddevicecontroller
- \ref sec_rwsimxml_posedevicecontroller
- \ref sec_rwsimxml_serialdevicecontroller
- \ref sec_rwsimxml_springjointcontroller

\b Example
\verbatim
<DynamicWorkcell workcell="pih.wc.xml">
	...
</DynamicWorkcell>
\endverbatim

## Include ##  {#sec_rwsimxml_include}
The dynamic workcell can be split into multiple files.
The extension of a included file should be .xml
(and not dwc.xml as it is not a valid dynamic workcell on its own).

\b Element Include

\b Attributes
- \b file: a string identifying the file to include.

\b Example
\verbatim
<Include file="include.xml" />
\endverbatim

The included file must have a root element called IncludeData: 
\verbatim
<IncludeData>
	...
</IncludeData>
\endverbatim

Everything inside the IncludeData element is interpreted as if it was inserted directly in the .dwc.xml
file instead of the Include tag.

## PhysicsEngine ##  {#sec_rwsimxml_physicsengine}
For dynamic simulation different Physics Engines can be used.
The PhysicsEngine element allows modification of parameters that are specific to the PhysicsEngine used.
See also rwsim::simulator::PhysicsEngine .

\b Element PhysicsEngine

\b Child elements:
- \b Property

<HR>

\b Child \b element Property

\b Attributes
- \b name: a string identifying the property.
- \b type: string (default), int, float or Q.

<HR>

\b Example
The following example shows an example with properties for the Open Dynamics Engine
(see rwsim::simulator::ODESimulator and ODE documentation for further information).
All properties are optional, and the example shows the default values used for ODE.
\verbatim
<PhysicsEngine>
	<Property name="StepMethod">WorldStep</Property> <!-- WorldQuickStep/WorldStep/WorldFast1 -->
    <Property name="WorldCFM" type="float">0.0000001</Property>
    <Property name="WorldERP" type="float">0.2</Property>
    <Property name="SpaceType">Simple</Property> <!-- QuadTree/Simple/HashTable -->
    <Property name="MaxIterations" type="int">20</Property>
    <Property name="ContactSurfaceLayer" type="float">0.0001</Property>
    <Property name="MaxSepDistance" type="float">0.0005</Property>
    <Property name="MaxPenetration" type="float">0.0005</Property>
    <Property name="MaxCorrectingVelocity" type="float">0.1</Property>
</PhysicsEngine>
\endverbatim

For similar options for the TNT engine, please see rwsimlibs::tntphysics::TNTIsland::getDefaultPropertyMap documentation.

## Gravity ##  {#sec_rwsimxml_gravity}
Set the gravity in world coordinates.

\b Element Gravity

\b Example
\verbatim
<Gravity>0 0 -9.82</Gravity>
\endverbatim

## Materials ##  {#sec_rwsimxml_materials}
Each body in a dynamic workcell must be associated to a material and object type.
These determine how bodies interact with each other by defining friction and restitution.
The material database is made of the following four elements: 
- \ref sec_rwsimxml_materialdata
- \ref sec_rwsimxml_frictionmap
- \ref sec_rwsimxml_objecttypedata
- \ref sec_rwsimxml_contactmap

In practice the material database can be big, and often the materials are defined in a separate file
(often named DynamicMaterialDataBase.xml) using the \ref sec_rwsimxml_include element.

### MaterialData ###  {#sec_rwsimxml_materialdata}
Defines names and an optional descriptions of all materials used in the dynamic workcell.
See also rwsim::dynamics::MaterialDataMap .

\b Element MaterialData

\b Child elements:
- \b Default: a string defining the default material for bodies where no material is defined explicitly.
- \b Material

<HR>

\b Child \b element Material

\b Attributes
- \b id: a string giving the name of the material.

\b Child elements:
- \b Description: (optional) a string with a description of the material.

<HR>

\b Example

The following example shows the definition of a single material.
\verbatim
<MaterialData>
	<Default>Plastic</Default>
	<Material id="Plastic">
		<Description>Optional description</Description>
	</Material>
</MaterialData>
\endverbatim

### FrictionMap ###  {#sec_rwsimxml_frictionmap}
Definition of friction between different materials defined in \ref sec_rwsimxml_materialdata .
See also rwsim::dynamics::FrictionData .

\b Element FrictionMap

\b Child elements:
- \b Pair

<HR>

\b Child \b element Pair

\b Attributes
- \b first: the material id of the first material.
- \b second: the material id of the second material.

\b Child elements:
- \b FrictionData

<HR>

\b Child \b element FrictionData

\b Attributes
- \b type: only Coulomb supported currently.

\b Child elements:
- \b Mu: a float giving the Coulomb friction coefficient.

<HR>

\b Example
\verbatim
<FrictionMap>
	<Pair first="Plastic" second="Plastic">
		<FrictionData type="Coulomb">
			<Mu>0.4</Mu>
		</FrictionData>
	</Pair>
</FrictionMap>
\endverbatim

### ObjectTypeData ###  {#sec_rwsimxml_objecttypedata}
Definition of different object types.
This is primarily used for defining restitution between objects.
See also rwsim::dynamics::ContactDataMap .

\b Element ObjectTypeData

\b Child elements:
- \b Default: a string defining default object type for objects where no object types are set explicitly.
- \b ObjectType

<HR>

\b Child \b element ObjectType

\b Attributes
- \b id: a string identifying the type of object.

\b Child elements:
- \b Description: (optional) a string with a description of the type.

<HR>

\b Example
\verbatim
<ObjectTypeData>
	<Default>hardObj</Default>
	<ObjectType id="hardObj">
		<Description>A hard object. with low elasticity</Description>
	</ObjectType>
</ObjectTypeData>
\endverbatim

### ContactMap ###  {#sec_rwsimxml_contactmap}
Definition of restitution coefficients between different object types
defined in \ref sec_rwsimxml_objecttypedata .
See also rwsim::dynamics::ContactDataMap::NewtonData .

\b Element ContactMap

\b Child elements:
- \b Pair

<HR>

\b Child \b element Pair

\b Attributes
- \b first: the type id of the first object type.
- \b second: the type id of the second object type.

\b Child elements:
- \b ContactData

<HR>

\b Element ContactData

\b Attributes
- \b type: only Newton supported currently.

\b Child elements:
- \b cr: a float giving the coefficient of restitution.

<HR>

\b Example
\verbatim
<ContactMap>
	<Pair first="hardObj" second="hardObj">
		<ContactData type="Newton">
			<cr>0.0</cr>
		</ContactData>
	</Pair>
</ContactMap>
\endverbatim

## Bodies ## {#sec_rwsimxml_bodies}
Bodies that should be a part of the simulation must be defined in the dynamic workcell.
There are three available types:
- \ref sec_rwsimxml_fixedbody :
The FixedBody specifies bodies that are static and does not move. Such a body influences the motion
of other bodies, but other bodies can not influence a FixedBody.
- \ref sec_rwsimxml_kinematicbody :
A KinematicBody can be moved during simulation, and can affect other bodies. Other bodies can
however not affect the velocity of the KinematicBody.
- \ref sec_rwsimxml_rigidbody :
Rigid bodies has mass and inertia and the motion is determined by the forces acting on the body.

### FixedBody ###  {#sec_rwsimxml_fixedbody}
Fixed bodies will typically be static environment, such as floors and walls.
See also rwsim::dynamics::FixedBody .

\b Element FixedBody

\b Attributes
- \b frame: a string associating the body to a Frame defined in the workcell.

\b Child elements:
- \b MaterialID: (optional) a string giving the name of a material defined in \ref sec_rwsimxml_materialdata .
- \b ObjectID: (optional) a string giving the name of a object type defined in \ref sec_rwsimxml_objecttypedata .
- \b Property: (optional) for additional data (Property tag defined in \ref sec_rwsimxml_physicsengine )

\b Example
\verbatim
<FixedBody frame="Floor" />
\endverbatim

### KinematicBody ###  {#sec_rwsimxml_kinematicbody}
Kinematic bodies can be controlled directly with velocities. See also rwsim::dynamics::KinematicBody .

\b Element KinematicBody

\b Attributes
- \b frame: a string associating the body to a \e MovableFrame defined in the workcell.

\b Child elements:
- \b MaterialID: (optional) a string giving the name of a material defined in \ref sec_rwsimxml_materialdata .
- \b ObjectID: (optional) a string giving the name of a object type defined in \ref sec_rwsimxml_objecttypedata .

\b Example
\verbatim
<KinematicBody frame="MovingBody">
	<MaterialID>Plastic</MaterialID>
</KinematicBody>
\endverbatim

### RigidBody ###  {#sec_rwsimxml_rigidbody}
Rigid bodies moves due to forces acting on the bodies. Hence they require specification
of dynamic parameters as mass and inertia. See also rwsim::dynamics::RigidBody .

\b Element RigidBody

\b Attributes
- \b frame: a string associating the body to a \e MovableFrame defined in the workcell.

\b Child elements:
- \b Mass: a float with the mass of the body.
- \b EstimateInertia: (optional) calculate Inertia and COG from geometry (geometry must be present in this case).
If COG is given this will be used when calulating the Inertia.
- \b COG: (required if EstimateInertia is not used, else optional) the center of gravity.
- \b Inertia: (required if EstimateInertia is not used) the 3x3 inertia matrix of the body.
- \b Integrator: the integrator used (not used in ODE).
- \b Associate: (optional) allows associating geometry that is not attached to the main body frame to this body.
- \b MaterialID: (optional) a string giving the name of a material defined in \ref sec_rwsimxml_materialdata .
- \b ObjectID: (optional) a string giving the name of a object type defined in \ref sec_rwsimxml_objecttypedata .
- \b Property: (optional) for additional data (Property tag defined in \ref sec_rwsimxml_physicsengine )

<HR>

\b Child \b element Associate

\b Attributes
- \b object: a string identifying a object in the workcell to associate to this body.
The geometry of the object is then added to this body.

<HR>

\b Example
\verbatim
<RigidBody frame="DynBodyFrame">
	<Mass>0.1</Mass>
	<EstimateInertia />
	<Integrator>Euler</Integrator>
	<Associate object="DynBodyGeometry" />
</RigidBody>
\endverbatim

## Devices ## {#sec_rwsimxml_devices}
Devices that should be a part of the simulation must be defined in the dynamic workcell.
There are two available types:
- \ref sec_rwsimxml_kinematicdevice :
The KinematicDevice specifies a device that is composed of kinematic bodies. If dynamic simulation
is not required this is the most efficient method to simulate a device.
- \ref sec_rwsimxml_rigiddevice :
A RigidDevice is composed of rigid bodies, which have their motion constrained.

Please note that a Body that is part of a device is called a Link. A Body in the dynamic workcell format
referes to a body that is not part of a device (it is free). A Link is part of a device and has its motion
constrained. Because of this, the syntax for a Link and a Body is equivalent in practice. 

### KinematicDevice ###  {#sec_rwsimxml_kinematicdevice}
See also rwsim::dynamics::KinematicDevice .

\b Element KinematicDevice

\b Attributes
- \b device: a string associating the dynamic device to a JointDevice defined in the workcell.

\b Child elements:
- \b FixedBase: (one base element required)
equivalent to \ref sec_rwsimxml_fixedbody.
- \b KinematicBase: (one base element required)
equivalent to \ref sec_rwsimxml_kinematicbody .
- \b RefBase: (one base element required) use an existing body as base.
- \b KinematicJoint/Link: equivalent to \ref sec_rwsimxml_rigidbody - note that mass parameters can just be set to zero.

<HR>

\b Child \b element RefBase

\b Attributes
- \b body: a string identifying an existing body.

<HR>

\b Example
\verbatim
 <KinematicDevice device="Robot">
	<KinematicBase frame="Base" />
	<KinematicJoint object="Joint0">
		<Mass>0</Mass> 
		<COG>0 0 0</COG>
		<Inertia>0 0 0 0 0 0 0 0 0</Inertia>
		<MaterialID>Plastic</MaterialID> 
	</KinematicJoint>
	...
 </KinematicDevice>
\endverbatim

### RigidDevice ###  {#sec_rwsimxml_rigiddevice}
See also rwsim::dynamics::RigidDevice .

\b Element RigidDevice

\b Attributes
- \b device: a string associating the dynamic device to a JointDevice defined in the workcell.

\b Child elements:
- \b FixedBase: (one base element required)
equivalent to \ref sec_rwsimxml_fixedbody.
- \b KinematicBase: (one base element required)
equivalent to \ref sec_rwsimxml_kinematicbody .
- \b RigidBase: (one base element required)
equivalent to \ref sec_rwsimxml_rigidbody .
- \b RefBase: (one base element required) use an existing body as base.
- \b RigidJoint/Link: equivalent to \ref sec_rwsimxml_rigidbody.
- \b ForceLimit: the force or torque applied by each motor (depending on the joint it refers to)

<HR>

\b Child \b element ForceLimit

\b Attributes
- \b joint: the joint to set limit for.

<HR>

\b Example
\verbatim
 <RigidDevice device="UR1">
 	<ForceLimit joint="Joint0">1000</ForceLimit>
	<FixedBase frame="Base">
		<MaterialID>Plastic</MaterialID> 
	</FixedBase> 
	<Link object="Joint0">
		<Mass>3.8</Mass>
		<EstimateInertia/>
		<MaterialID>Plastic</MaterialID> 
	</Link> 
	...
 </RigidDevice>
\endverbatim

## SuctionCup ##  {#sec_rwsimxml_suctioncup}
The SuctionCup element is not documented yet.

## Constraints ## {#sec_rwsimxml_constraints}
The only way to constraint bodies is to use devices. In some cases it might however be
useful to constrain arbitrary bodies without requiring that the bodies are part of the same
device. This could for instance be if one wants to model a spring.
The Constraint element is the equivalent of a rwsim::dynamics::Constraint objects.

\b Element Constraint

\b Attributes
- \b name: a unique name for this constraint.
- \b type: one of the ContraintType values defined in rwsim::dynamics::Constraint (Fixed, Prismatic, Revolute, Universal, Spherical, Piston, PrismaticRotoid, PrismaticUniversal, Free)
- \b parent: the parent body.
- \b child: the child body.

\b Child elements:
- \b Transform3D: (optional) where the constraint acts relative to the parent body frame.
- \b Spring: (optional) adds springs in the non-constrained directions (see \ref sec_rwsimxml_springs ).

Note the Spring element can be defined under the Constraint or after the Constraint with a reference to the name
of the constraint.

\b Example of a Fixed constraint:
\verbatim
<Constraint name="FixedConstraint" type="Fixed" parent="Parent" child="Child" />
\endverbatim

\b Example of a non-fixed constraint (the spring works for the one linear and two angular directions that are not constrained by the PrismaticUniversal constraint):
\verbatim
<Constraint name="ComplianceConstraint" type="PrismaticUniversal" parent="Parent" child="Child">
	<Transform3D>
		<Pos>0 0 0.01</Pos>
		<RPY>0 0 0</RPY>
	</Transform3D>
	<Spring>
		<Compliance>
			0.0005 0 0
			0 0.5 0
			0 0 0.5
		</Compliance>
		<Damping>
			50 0 0
			0 0.1 0
			0 0  0.1
		</Damping>
	</Spring>
</Constraint>
\endverbatim

## Springs ## {#sec_rwsimxml_springs}
It is possible to attach a spring to a constraint as described in \ref sec_rwsimxml_constraints .
If the spring is defined outside the Constraint tags, it must refer to a constraint by name.

\b Element Spring

\b Attributes
- \b constraint: the name of the constraint to attach the string to (the string works in the non-constrained directions).

\b Child elements:
- \b Compliance: n times n compliance matrix where n is the number of non-constrained dimensions (between 0 and 6 according to constraint type).
- \b Damping: matrix of same dimensions as the compliance matrix.

\b Example of a spring attached to a PrismaticUniversal constraint
(the spring works for the one linear and two angular directions that are not constrained by the PrismaticUniversal constraint):
\verbatim
<Spring constraint="ComplianceConstraint">
	<Compliance>
		0.0005 0 0
		0 0.5 0
		0 0 0.5
	</Compliance>
	<Damping>
		50 0 0
		0 0.1 0
		0 0  0.1
	</Damping>
</Spring>
\endverbatim

## Sensors ## {#sec_rwsimxml_sensors}
Not documented yet.
### TactileArraySensor ###  {#sec_rwsimxml_tactilearraysensor}
### BodyContactSensor ###  {#sec_rwsimxml_bodycontactsensor}
### TactileMultiAxisSensor ###  {#sec_rwsimxml_tactilemultiaxissensor}
### FTSensor ###  {#sec_rwsimxml_ftsensor}

## Controllers ## {#sec_rwsimxml_controllers}
Not documented yet.
### PDDeviceController ###  {#sec_rwsimxml_pddevicecontroller}
### PoseDeviceController ###  {#sec_rwsimxml_posedevicecontroller}
### SerialDeviceController ###  {#sec_rwsimxml_serialdevicecontroller}
### SpringJointController ###  {#sec_rwsimxml_springjointcontroller}