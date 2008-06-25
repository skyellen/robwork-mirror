// -*- latex -*-

/** \mainpage RobWork

\section sec_index_intro Introduction

\b RobWork is a collection of C++ libraries for simulation and control
of robot systems. RobWork is used for research and education as well
as for practical robot applications. Features of the library include:

- Kinematic modeling of various types of industrial manipulators.

- Path-planning and inverse kinematics algorithms.

- Drivers for robots and sensors available in our lab.
.

Workcells and their operation are visualized in a seperate application
called \b RobWorkStudio.

RobWork is developed at the <a
href="http://www.mip.sdu.dk/robotics">robotics department</a> of the
<a href="http://www.sdu.dk/mmmi">Mærsk McKinney Møller Institute</a>
at the <a href="http://www.sdu.dk">University of Southern
Denmark</a>. The focus of the department is on industrial robots and
their applications.

\section sec_index_manual Manuals and tutorials

- \ref page_rw_manual

\section sec_index_download Download

Download \b RobWork and \b RobWorkStudio from the <a
href="http://www.robwork.dk">RobWork homepage</a>.

\section sec_index_installation Installation

- \ref page_rw_installation

\section sec_index_license License

RobWork is distributed under a
\ref page_license "BSD style license". For convenience, a number of
open-source libraries are distributed together with RobWork; the
RobWork license does not apply to these libraries.

\section sec_index_mailinglists Mailing lists

- <a href="http://mls.sdu.dk/mailman/listinfo/robwork">RobWork mailing list</a>

*/

/*

About this manual in relation to the homepage

Permanent stuff on the homepage: 

- Introduction (home page)
- Documents (link to this manual)
- Download (binary and source-code distributions)
- Mailing list (via mailman)
- License
- Links
- Forum
- Screenshots and demos

Permanent stuff in the manual:
- RW File format 
- Presentations 
- API doc
- Tutorials

*/

//----------------------------------------------------------------------
// Everything below are just various pieces of text that aren't part
// of the documentation.

/*

Math package based on Boost/Ublas with varius mathematical constructs
for calculating and describing 3d kinematic scenes and other robot
specific stuff.

Loading and visualising of devices with a variety of kinematic
structures (SerialDevice, TreeDevice, ParallelDevice, MobileDevice).

General iterative forward and inverse kinematics for all types of
devices.

----------------------------------------------------------------------

Major goals of this framework:

- Provide a single framework for offline and online robot programming including
  modelling, simulation and (realtime)control of robotics
.

The target audience for the library are:

- Researchers who needs a common framework for experimental robotics

- Students who wants to experiment with concepts of robotics

- Implementers of robot applications
.

Since this library uses boost, and other modern frameworks, a modern
(2000+) C++ compiler like GCC 3 or Visual C++ 7 is required. Not all
files are expected to compile without problems on ancient C++
compilers.

This library currently fails to compile with gcc 4.1.2 see
http://gcc.gnu.org/bugzilla/show_bug.cgi?id=28016 for details

Target audience:
- Researchers who needs a common framework for experimental robotics
- Students who wants to experiment with concepts of robotics
- Implementers of robot applications
.

This framework uses the following 3rd party libraries:
- stl (C++ Standard Template Library)
- boost (Peer-reviewed C++ libraries)
- boost::ublas, for basic linear algebra
- boost::spirit, for datafile parsing
- boost::graph, for probabilistic roadmaps
- LAPACK, for numerical computations
- OpenGL, for visualization
.

And the following optional libraries:
- opcode, for collision detection
- PQP, for collision detection
- RAPID, for collision detection
.

*/
