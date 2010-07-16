/**

\page page_rwsim_manual RobWorkSim manual

- \ref sec_rwsim_manual_intro
	- \ref sec_rwsim_manual_notation
- \ref sec_rwsim_namespaces
- \ref sec_rwsim_libraries
- \ref sec_rwsim_plugins
- \ref sec_rwsim_dynamic_workcell
    - \ref sec_rwsim_body_types
    - \ref sec_rwsim_device_types
    - \ref sec_rwsim_sensor_types
    - \ref sec_rwsim_actuator_types
    .
- \subpage page_rwsim_xml_workcell_format

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


*/


