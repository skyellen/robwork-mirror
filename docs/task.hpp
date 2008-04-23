// -*- latex -*-

/**

\page page_task Task descriptions

- \ref sec_task_format
  - \ref sec_task_primitive_types
  - \ref sec_task_properties
  - \ref sec_task_task
  - \ref sec_task_action
  - \ref sec_task_trajectory
  - \ref sec_task_targets
  - \ref sec_task_links
  .
- \ref sec_task_example
- \ref sec_task_loading
- \ref sec_task_traversal
.

\section sec_task_format Task description format

\subsection sec_task_primitive_types Primitive types

\verbatim
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
\endverbatim

\subsection sec_task_properties Properties

\verbatim
<property-key> ::= Key <string>

<property-description> ::= Description <string>

<property-value> ::=
    S <string> |
    <number> |
    <vector> |
    <rpy> |
    <rotation-matrix> |
    <transform> |
    <configuration>

<property> ::= Property <property-key> <property-value>

<property-map> ::= PropertyMap <property>*
\endverbatim

\subsection sec_task_task Task

\verbatim
<task> ::= Task <name>? <properties>? <workcell>? <action>*

<action> ::= <trajectory> | <attach-frame>
\endverbatim

\subsection sec_task_action AttachFrame

\verbatim
<attach-frame> ::= AttachFrame <name>? <properties>? <item> <tcp>

<item> ::= Item <string>

<tcp> ::= TCP <string>
\endverbatim

\subsection sec_task_trajectory Trajectory

\verbatim
<trajectory-element> ::= <target> | <link>

<device> ::= Device <string>

<tool> ::= TCP <string>

<trajectory> ::=
    Trajectory <name>? <properties>? <device> <tool> <trajectory-element>*
\endverbatim

\subsection sec_task_targets Targets

\verbatim
<target-location> ::=
  Tool <transform> <frame> |
  Joint <configuration>

<target> ::= Target <name>? <properties>? <target-location>
\endverbatim

\subsection sec_task_links Links

\verbatim
<tool-speed> ::= Speed (Angular | Positional) <number>

<motion-constraint> ::=
  LinearJointConstraint |
  LinearToolConstraint <tool-speed> |
  CircularToolConstraint <tool-speed> <vector> <frame>

<link> ::= Link <name>? <properties>? <motion-constraint>?
\endverbatim

\section sec_task_example Task description for pick-and-place task

The robot of this example is a 6 DOF Fanuc manipulator equipped with a
9 DOF 3-fingered Schunk hand.

The robot starts in the home position, opens the hand and moves to the
position of the item to pick up. The hand is closed and the item is
moved to the position for the placement of the item. The hand opened
and the robot and hand is moved back to the home position.

The task description hardcodes the 9 DOF configurations for the hand
and the pick and place positions for the item.

\verbatim
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
          <N>0</N><N>-0.7</N><N>0.4</N>
          <N>0</N><N>-0.7</N><N>0.5235</N>
          <N>0</N><N>-0.7</N><N>0.5235</N>
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
          <Vector3D><N>0</N><N>0</N><N>0</N></Vector3D>
          <RPY><N>0</N><N>0</N><N>0</N></RPY>
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
          <Vector3D><N>-1</N><N>-1</N><N>0</N></Vector3D>
        </Property>
        <Property>
          <Key>A2</Key>
          <Description>Approach vector 2.</Description>
          <Vector3D><N>1</N><N>1</N><N>0</N></Vector3D>
        </Property>
      </PropertyMap>
      <Joint>
        <Q>
          <N>0</N><N>0.208</N><N>-0.088</N>
          <N>-0.5235</N><N>-0.109</N><N>0.151</N>
          <N>0.5235</N><N>-0.109</N><N>0.151</N>
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
          <Vector3D><N>0</N><N>0</N><N>0</N></Vector3D>
          <RPY><N>0</N><N>0</N><N>0</N></RPY>
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
          <N>0</N><N>-0.7</N><N>0.4</N>
          <N>0</N><N>-0.7</N><N>0.5235</N>
          <N>0</N><N>-0.7</N><N>0.5235</N>
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
        <Q><N>0</N><N>0</N><N>0</N><N>0</N><N>0</N><N>0</N></Q>
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
          <N>0</N><N>0</N><N>0</N>
          <N>0</N><N>0</N><N>0</N>
          <N>0</N><N>0</N><N>0</N>
        </Q>
      </Joint>
    </Target>
  </Trajectory>

</Task>
\endverbatim

\section sec_task_loading Loading a task description

To load a task file you must either explicitly provide the workcell
for which the task is meant, or your task description must name the
workcell file in the \c WorkCell tag. The facility to load a task
description is rw::loaders::TaskLoader::load(). The program below
shows the two ways of loading a task description.

\include ex-load-task.cpp

Running the program can for example look as follows:

\verbatim
> ./rw_load_task pick-and-place.xml
Task Task[Pick and place task, WorkCell[d:/FanucSchunk/scene.wu]]
succesfully loaded. (1)

> ./rw_load_task d:/FanucSchunk/scene.wu pick-and-place.xml
Task Task[Pick and place task, WorkCell[d:/FanucSchunk/scene.wu]]
succesfully loaded. (2)
\endverbatim

If the workcell specified for the task description is not compatible
with the frame and device names listed in the task description, the
loading of the task description will throw an exception:

\verbatim
> ./rw_load_task d:/src/RobWorkData/TULScenes/FanucCatcher/scene.wu pick-and-place.xml
Exception (D:\src\RobWork\src\rw\loaders\TaskLoader.cpp:258):
No device named 'Gripper.Composite' in workcell
WorkCell[d:/src/RobWorkData/TULScenes/FanucCatcher/scene.wu]
\endverbatim

\section sec_task_traversal Traversing a task description

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

\verbatim
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
\endverbatim

You can modify the program to execute the robot motions either in
simulation or for the real robot.

*/

/*

Tasks i RobWork er beskrevet i et struktureret format med flere lag.
RobWork er et system til geometrisk modellering, og task-beskrivelser
i RobWork er struktureret med henblik på løsning af geometriske
planlægningsproblemer.

I det følgende er givet en oversigt over syntaksen over laget i
task-beskrivelsen, der beskriver ønskede bevægelser for devices og
deres tools. I notationen for syntaksen svarer et typenavn \c Name
til et XML-element på formen <code><Name> ... </Name></code>. En
ustruktureret streng af tegn betegnes med <code><leaf></code>.

--

De primitive RobWork-typer \c Vector3D, \c Rotation3D,
\c Transform3D og \c Q angives i task-formatet med notationen
nedenfor. Frames identificeres i task-formatet ved deres navn (en
string). Et \c Name-element angiver et navn for en delblok af en
task. Man kan med \c Define definere et alias for værdien af en
streng eller tal. Eventuelt kan man understøtte at \c Define
definerer et alias for et vilkårligt XML-udtryk. Man anvender
\c Use på det sted, hvor man ønsker værdien skal sættes ind.

--

I konstruktionen af en task kan det være bekvemt at kunne tilføje
applikationsspecifikke informationer. Disse informationer kaldes for
\e properties og bliver ikke brugt af de indbyggede
RobWork-rutiner. Man kan fortolke sine properties i en planner skrevet
specifikt til applikationen eller evt. først efter at al planlægningen
i RobWork er blevet udført.

Properties er i task-formatet beskrevet ved en liste af (\e key,
\e value)}-par. Syntaksen nedenfor angiver at værdien for en property
altid er af typen string. Afhængigt af hvor udbredt brugen af
properties vil blive kunne man understøtte andre typer af værdier,
f.eks. tal, device-konfigurationer og transformationsmatricer.

--

Flaget \c Special signalerer, at de angivne properties \e skal
fortolkes af en planner for task. En standard indbygget motion planner
stopper derfor planlægningen med en fejl, hvis den når en
property-liste af denne type.

--

En trajectory beskriver en ønsket bevægelse for et device og dets
tool. Beskrivelsen indeholder ingen oplysninger om den kontekst
bevægelsen vil blive udført i og ej heller (i første omgang)
kommandoer til ændring af konteksten.

En trajectory er en liste af \e targets og \e links eller
grupperinger af targets og links. Med alle grupperinger fladet ud skal
trajectory skiftevis indeholde et target og et link. Man kan for hver
gruppering definere aliaser for værdier. Scope for et alias er gruppen
selv.

--

Et target beskriver konfigurationen af et device eller
transformationen af dets tool. Man kan kun angive konfigurationen for
det aktuelle device og intet af konfigurationen for den øvrige
workcell. Transformationen af tool er givet relativt til en reference
frame.

**

Properties for et target kan tænkes at blive brugt til f.eks.
særregler for hvordan target skal nås:

- Target er et via-punkt beregnet til \e vejledning for en
  bevægelse, men en motion planner har lov at vælge en rute, der
  ignorerer target.

- Target har en tolerance: Device behøver ikke nå target eksakt,
  men skal blot være indenfor et nærmere angivet område.

- Target skal nås for en bestem type af invers kinematik (IK)
  løsning, f.eks. IK løsningen liggende nærmest en angivet joint
  konfiguration.
.

--

Et link angiver en bevægelse mellem targets. Bevægelsen kan være
underlagt constraints. Eksempelvis kan tool være sat til at følge en
ret linie eller cirkel-kurve (begge med konstant hastighed), eller
joints kan være sat til at følge en ret linie i konfigurationsrummet.
For tool-constraints af type \c LinearToolConstraint og
\c CircularToolConstraint er slerp-interpolering for
tool-orientering underforstået. Tool-constraints af typen
\c CircularToolConstraint angiver formen af cirkelbuen ved et
ekstra punkt (i en bestemt frame) liggende mellem start- og
slut-target.

**

Properties for et link kan blive brugt til f.eks. yderligere
constraints for bevægelsen.


*/
