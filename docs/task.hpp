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

Tasks i RobWork er beskrevet i et struktureret format med flere lag.
RobWork er et system til geometrisk modellering, og task-beskrivelser
i RobWork er struktureret med henblik p� l�sning af geometriske
planl�gningsproblemer.

I det f�lgende er givet en oversigt over syntaksen over laget i
task-beskrivelsen, der beskriver �nskede bev�gelser for devices og
deres tools. I notationen for syntaksen svarer et typenavn \c Name
til et XML-element p� formen <code><Name> ... </Name></code>. En
ustruktureret streng af tegn betegnes med <code><leaf></code>.

\subsection sec_task_primitive_types Primitive types

De primitive RobWork-typer \c Vector3D, \c Rotation3D,
\c Transform3D og \c Q angives i task-formatet med notationen
nedenfor. Frames identificeres i task-formatet ved deres navn (en
string). Et \c Name-element angiver et navn for en delblok af en
task. Man kan med \c Define definere et alias for v�rdien af en
streng eller tal. Eventuelt kan man underst�tte at \c Define
definerer et alias for et vilk�rligt XML-udtryk. Man anvender
\c Use p� det sted, hvor man �nsker v�rdien skal s�ttes ind.

\verbatim
<define> ::= Define <leaf> <leaf>

<string> ::= <leaf> | Use <leaf>

<number> ::= N <leaf> | Use <leaf>

<vector> ::= Vector3D <number> <number> <number>

<rpy> ::= RPY <number> <number> <number>

<matrix> ::=
    Rotation3D
        <number> <number> <number>
        <number> <number> <number>
        <number> <number> <number>

<rotation> ::= <matrix> | <rpy>

<transform> ::= Transform3D <vector> <rotation>

<configuration> ::= Q <number>*

<frame> ::= Frame <string>

<name> ::= Name <string>
\endverbatim

\subsection sec_task_properties Properties

I konstruktionen af en task kan det v�re bekvemt at kunne tilf�je
applikationsspecifikke informationer. Disse informationer kaldes for
\e properties og bliver ikke brugt af de indbyggede
RobWork-rutiner. Man kan fortolke sine properties i en planner skrevet
specifikt til applikationen eller evt. f�rst efter at al planl�gningen
i RobWork er blevet udf�rt.

Properties er i task-formatet beskrevet ved en liste af (\e key,
\e value)}-par. Syntaksen nedenfor angiver at v�rdien for en property
altid er af typen string. Afh�ngigt af hvor udbredt brugen af
properties vil blive kunne man underst�tte andre typer af v�rdier,
f.eks. tal, device-konfigurationer og transformationsmatricer.

\verbatim
<property-key> ::= Key <string>

<property-value> ::= Value <string>

<property> ::= Property <property-key> <property-value>

<properties> ::= Properties Special? <property>*
\endverbatim

Flaget \c Special signalerer, at de angivne properties \e skal
fortolkes af en planner for task. En standard indbygget motion planner
stopper derfor planl�gningen med en fejl, hvis den n�r en
property-liste af denne type.

\subsection sec_task_task Task

\verbatim
<task> ::= Task <name>? <workcell>? <task-element>*

<task-element> ::= <trajectory> | <action>
\endverbatim

\subsection sec_task_action Action

\verbatim
<action> ::= Action <name>? <action-type>

<action-type> ::= <attach-frame>

<attach-frame> ::= AttachFrame <item> <tcp>

<item> ::= Item <string>

<tcp> ::= TCP <string>
\endverbatim

\subsection sec_task_trajectory Trajectory

En trajectory beskriver en �nsket bev�gelse for et device og dets
tool. Beskrivelsen indeholder ingen oplysninger om den kontekst
bev�gelsen vil blive udf�rt i og ej heller (i f�rste omgang)
kommandoer til �ndring af konteksten.

En trajectory er en liste af \e targets og \e links eller
grupperinger af targets og links. Med alle grupperinger fladet ud skal
trajectory skiftevis indeholde et target og et link. Man kan for hver
gruppering definere aliaser for v�rdier. Scope for et alias er gruppen
selv.

\verbatim
<trajectory-element> ::= <group> | <target> | <link>

<group> ::= Group <define>* <trajectory-element>*

<device> ::= Device <string>

<tool> ::= TCP <string>

<trajectory> ::=
    Trajectory <device> <tool> <trajectory-element>* <properties>?
\endverbatim

\subsection sec_task_targets Targets

Et target beskriver konfigurationen af et device eller
transformationen af dets tool. Man kan kun angive konfigurationen for
det aktuelle device og intet af konfigurationen for den �vrige
workcell. Transformationen af tool er givet relativt til en reference
frame.

\verbatim
<target-location> ::=
  Tool <transform> <frame> |
  Joint <configuration>

<target> ::= Target <name>? <target-location> <properties>?
\endverbatim

Properties for et target kan t�nkes at blive brugt til f.eks.
s�rregler for hvordan target skal n�s:

- Target er et via-punkt beregnet til \e vejledning for en
  bev�gelse, men en motion planner har lov at v�lge en rute, der
  ignorerer target.

- Target har en tolerance: Device beh�ver ikke n� target eksakt,
  men skal blot v�re indenfor et n�rmere angivet omr�de.

- Target skal n�s for en bestem type af invers kinematik (IK)
  l�sning, f.eks. IK l�sningen liggende n�rmest en angivet joint
  konfiguration.
.

\subsection sec_task_links Links

Et link angiver en bev�gelse mellem targets. Bev�gelsen kan v�re
underlagt constraints. Eksempelvis kan tool v�re sat til at f�lge en
ret linie eller cirkel-kurve (begge med konstant hastighed), eller
joints kan v�re sat til at f�lge en ret linie i konfigurationsrummet.
For tool-constraints af type \c LinearToolConstraint og
\c CircularToolConstraint er slerp-interpolering for
tool-orientering underforst�et. Tool-constraints af typen
\c CircularToolConstraint angiver formen af cirkelbuen ved et
ekstra punkt (i en bestemt frame) liggende mellem start- og
slut-target.

\verbatim
<tool-speed> ::= Speed (Angular | Positional) <number>

<motion-constraint> ::=
  LinearJointConstraint |
  LinearToolConstraint <tool-speed> |
  CircularToolConstraint <tool-speed> <vector> <frame>

<link> ::= Link <name>? <motion-constraint>? <properties>?
\endverbatim

Properties for et link kan blive brugt til f.eks. yderligere
constraints for bev�gelsen.

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

  <!-- Use this workcell. -->
  <WorkCell>d:/FanucSchunk/scene.wu</WorkCell>

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
    <Device>Gripper.Composite</Device>
    <Target>
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
  <Action>
    <Name>Grip item</Name>
    <AttachFrame>
      <Item>Item</Item>
      <TCP>RobotTool</TCP>
    </AttachFrame>
  </Action>

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
  <Action>
    <Name>Release item</Name>
    <AttachFrame>
      <Item>Item</Item>
      <TCP>WORLD</TCP>
    </AttachFrame>
  </Action>

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

*/
