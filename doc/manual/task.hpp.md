Task descriptions {#page_task}
=================

[TOC]

# Task description format # {#sec_task_format}
In RobWork tasks are described in a structured format with multiple layers. 
RobWork is amongst other things a system for geometric modelling and task descriptions 
in robwork is designed primarilly with respect to solving geometric planning problems.  

In the following the syntax of the task description fileformat is presented.
It is focussed at describing movements of devices and their tools. 
In the notation a typename *Name* corresponds to am XML-element in the form
`<Name> ... </Name>`. An unstructured text string is denoted `<leaf>`.

##  Primitive types ## {#sec_task_primitive_types}

The primitive robwork types *Vector3D*, *Rotation3D*,
*Transform3D* and *Q* use the notation presented below. Frames
in the task format are identified with thier name (a string). A 
*Name-element* corresponds to a name of one sub blok of a task.
With *Define* it is possible to define an alias of either a
string or number value. To insert a defined value then the 
element *Use* is inserted where the alias value is needed.

	<string> ::= <char>*
	<number> ::= N <char>*
	<vector> ::= Vector3D <number> <number> <number>
	<rpy> ::= RPY <number> <number> <number>
	<rotation-matrix> ::=
		Rotation3D
			<number> <number> <number>
			<number> <number> <number>
			<number> <number> <number>
	<rotation> ::= <rotation-matrix> | <rpy>
	<transform> ::= Transform3D <vector> <rotation>
	<configuration> ::= Q <number>*
	<frame> ::= Frame <string>
	<name> ::= Name <string>


## Properties ## {#sec_task_properties}

Properties are in the task format described using a list of *(key,value)*
pairs. 

	<property-key> ::= Key <string>
	<property-description> ::= Description <string>
	<property-value> ::=
		Special  |
		<string> |
		<number> |
		<vector> |
		<rpy> |
		<rotation-matrix> |
		<transform> |
		<configuration>
	<property> ::= Property <property-key> <property-value>
	<property-map> ::= PropertyMap <property>*

The flag *Sepcial* signals that the properties must be interpreted by 
a task planner. The standard built in motion planner will therefore 
stop its planning with an error if the special type is in the property list.


## Task ## {#sec_task_task}
In the construction of a task it is convienient to add 
application specifik information. These informations are named
*properties* and are seldom used by the inbuilt RobWork rutines.
You can interprete your properties in the planner if it is 
developed directly for the application. You might also wait until
the RobWork planning is done and then edit the plan.
 
	<task> ::= Task <name>? <properties>? <workcell>? <action>*
	<action> ::= <trajectory> | <attach-frame>


## AttachFrame ## {#sec_task_action}

	<attach-frame> ::= AttachFrame <name>? <properties>? <item> <tcp>
	<item> ::= Item <string>
	<tcp> ::= TCP <string>


## Trajectory ## {#sec_task_trajectory}
A trajectory describe a desired movement for a device and its tool.
The description does not hold any information on the context in 
which the movement is to be executed and neither does it hold commands for 
changing the current context.

A trajectory is a list of *targets* and *links* or 
groups of targets or links. With all groupings flattened the trajectory
must contain an alternating sequence of target and link. For
each group you can define aliases for values. The scope of an alias is the 
group in which it was defined.


	<trajectory-element> ::= <target> | <link>
	<device> ::= Device <string>
	<tool> ::= TCP <string>
	<trajectory> ::=
		Trajectory <name>? <properties>? <device> <tool> <trajectory-element>*


## Target ## {#sec_task_targets}
A target describe the configuration of a device or
the transformation of its tool. You can only store the configuration 
of the current device and no configuration of the other parts of the workcell.
The transformation of the tool is relativ to a reference frame.
	
	<target-location> ::=
	  Tool <transform> <frame> |
	  Joint <configuration>
	
	<target> ::= Target <name>? <properties>? <target-location>

Properties of a target can be used in several ways. For eksample 
to define special rules for how to achive a target:

- target is a via-point meant to guide some movement, but the 
motion planner can choose another rute and ignore the via-point.
- target has a tolerance: The device does not need to exactly reach 
the target, but only reach inside a specific area of the target.
- target must be completed with a specific inverse kinematic (IK)
solution, e.g. IK solution closest to some stored joint configuration


## Link ## {#sec_task_links}
A link denote the movement between two targets. The movement
can be constrained. For example: the tool can be set to follow a line
or a circle curve with constant speed, or joints can be set to follow a
straight line in configuration space. For tool-constraints of the type
*LinearToolConstraint* and *CircularToolConstraint* the slerp-interpolation
for tool-orientation is implicit. Tool-constraints of the type 
*CircularToolConstraint* denote the angiver form of the cirkel-curve
using an extra point (in a certain frame) which lie between start and end target.


	<tool-speed> ::= Speed (Angular | Positional) <number>
	<constraint> ::=
	  LinearJointConstraint |
	  LinearToolConstraint <tool-speed> |
	  CircularToolConstraint <tool-speed> <vector> <frame>	
	<link> ::= Link <name>? <properties>? <constraint>?

Properties of a link can be used to define extra constraints on the movement.

# Task description for pick-and-place task # {#sec_task_example}

The robot of this example is a 6 DOF Fanuc manipulator equipped with a
9 DOF 3-fingered Schunk hand.

The robot starts in the home position, opens the hand and moves to the
position of the item to pick up. The hand is closed and the item is
moved to the position for the placement of the item. The hand opened
and the robot and hand is moved back to the home position.

The task description hardcodes the 9 DOF configurations for the hand
and the pick and place positions for the item.

~~~~{.xml}
<Task>
  <Name>Pick and place task</Name>

  <PropertyMap>
    <Property>
      <Key>IP</Key>
      <Description>IP number of server</Description>
      <S>127.0.0.1</S>
    </Property>
  </PropertyMap>

  <!-- Use this workcell. -->
  <WorkCell>d:/movebots/FanucSchunk/scene.wu</WorkCell>

  <!-- Open hand -->
  <Trajectory>
    <Name>Open hand</Name>
    <Device>Gripper.Composite</Device>
    <Target>
      <Joint>
        <Q>
          0 -0.7 0.4
          0 -0.7 0.5235
          0 -0.7 0.5235
        </Q>
      </Joint>
    </Target>
  </Trajectory>

  <!-- Move to target pick position -->
  <Trajectory>
    <Name>Pick target</Name>
    <Device>Robot</Device>
    <TCP>RobotTool</TCP>
    <Target>
      <Tool>
        <Transform3D>
          <Vector3D>0 0 0</Vector3D>
          <RPY>0 0 0</RPY>
        </Transform3D>
        <Frame>ItemStart</Frame>
      </Tool>
    </Target>
  </Trajectory>

  <!-- Close hand -->
  <Trajectory>
    <Name>Close hand</Name>
    <PropertyMap>
      <Property>
        <Key>Force</Key>
        <Description>Force (in Newton) to apply with gripper.</Description>
        <N>2.5</N>
      </Property>
      <Property>
        <Key>MaxGripTime</Key>
        <Description>Maximum time (in seconds) to make a grip.</Description>
        <N>10.5</N>
      </Property>
    </PropertyMap>
    <Device>Gripper.Composite</Device>
    <Target>
      <PropertyMap>
        <Property>
          <Key>A1</Key>
          <Description>Approach vector 1.</Description>
          <Vector3D>-1 -1 0</Vector3D>
        </Property>
        <Property>
          <Key>A2</Key>
          <Description>Approach vector 2.</Description>
          <Vector3D>1 1 0</Vector3D>
        </Property>
      </PropertyMap>
      <Joint>
        <Q>
          0 0.208 -0.088
          -0.5235 -0.109 0.151
          0.5235 -0.109 0.151
        </Q>
      </Joint>
    </Target>
  </Trajectory>

  <!-- Grip the item by attaching it to the gripper frame. -->
  <AttachFrame>
    <Name>Grip item</Name>
    <Item>Item</Item>
    <TCP>RobotTool</TCP>
  </AttachFrame>

  <!-- Move to target placement position -->
  <Trajectory>
    <Name>Place target</Name>
    <Device>Robot</Device>
    <TCP>RobotTool</TCP>
    <Target>
      <Tool>
        <Transform3D>
          <Vector3D>0 0 0</Vector3D>
          <RPY>0 0 0</RPY>
        </Transform3D>
        <Frame>ItemEnd</Frame>
      </Tool>
    </Target>
  </Trajectory>

  <!-- Open hand -->
  <Trajectory>
    <Name>Open hand</Name>
    <Device>Gripper.Composite</Device>
    <Target>
      <Joint>
        <Q>
          0 -0.7 0.4
          0 -0.7 0.5235
          0 -0.7 0.5235
        </Q>
      </Joint>
    </Target>
  </Trajectory>

  <!-- Release the item by attaching it to the WORLD frame. -->
  <AttachFrame>
    <Name>Release item</Name>
    <Item>Item</Item>
    <TCP>WORLD</TCP>
  </AttachFrame>

  <!-- Robot to home position -->
  <Trajectory>
    <Name>Robot to home</Name>
    <Device>Robot</Device>
    <TCP>RobotTool</TCP>
    <Target>
      <Joint>
        <Q>0 0 0 0 0 0</Q>
      </Joint>
    </Target>
  </Trajectory>

  <!-- Hand to home position -->
  <Trajectory>
    <Name>Hand to home</Name>
    <Device>Gripper.Composite</Device>
    <Target>
      <Joint>
        <Q>
          0 0 0
          0 0 0
          0 0 0
        </Q>
      </Joint>
    </Target>
  </Trajectory>

</Task>
~~~~

# Loading a task description # {#sec_task_loading}

To load a task file you must either explicitly provide the workcell
for which the task is meant, or your task description must name the
workcell file in the \c WorkCell tag. The facility to load a task
description is rw::loaders::TaskLoader::load(). The program below
shows the two ways of loading a task description.

\include snippets/ex-load-task.cpp

Running the program can for example look as follows:

~~~~
> ./rw_load_task pick-and-place.xml
Task Task[Pick and place task, WorkCell[d:/FanucSchunk/scene.wu]]
succesfully loaded. (1)

> ./rw_load_task d:/FanucSchunk/scene.wu pick-and-place.xml
Task Task[Pick and place task, WorkCell[d:/FanucSchunk/scene.wu]]
succesfully loaded. (2)
~~~~

If the workcell specified for the task description is not compatible
with the frame and device names listed in the task description, the
loading of the task description will throw an exception:

~~~~
> ./rw_load_task d:/src/RobWorkData/TULScenes/FanucCatcher/scene.wu pick-and-place.xml
Exception (D:\src\RobWork\src\rw\loaders\TaskLoader.cpp:258):
No device named 'Gripper.Composite' in workcell
WorkCell[d:/src/RobWorkData/TULScenes/FanucCatcher/scene.wu]
~~~~

# Traversing a task description # {#sec_task_traversal}

The contents of a task description file are stored in a task object
(rw::task::Task). The details of execution of a task object are
typically application dependent. You should therefore expect to have
to traverse the task object yourself and to perform an appropiate
action for each subpart of the task.

The program below shows how to traverse each action and the targets of
each trajectory. The program does not visit the links connecting the
targets.

\include ex-traverse-task.cpp

Calling visitTask() on the pick-and-place task description prints the
following summary of the task:

~~~~
Task Pick and place task
Properties: IP
  Trajectory Open hand
    Target
      Move device to Q of DOF 9
  Trajectory Pick target
    Target
      Move tool to Vector3D {0, 0, 0} relative to Frame[ItemStart]
  Trajectory Close hand
  Properties: Force MaxGripTime
    Target
    Properties: A1 A2
      Move device to Q of DOF 9
  Attach Item to RobotTool
  Trajectory Place target
    Target
      Move tool to Vector3D {0, 0, 0} relative to Frame[ItemEnd]
  Trajectory Open hand
    Target
      Move device to Q of DOF 9
  Attach Item to WORLD
  Trajectory Robot to home
    Target
      Move device to Q of DOF 6
  Trajectory Hand to home
    Target
      Move device to Q of DOF 9
~~~~

You can modify the program to execute the robot motions either in
simulation or for the real robot.
