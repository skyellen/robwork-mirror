// -*- latex -*-

/** \mainpage RobWork

\section sec_index_intro Introduction

\b RobWork is a collection of C++ libraries for simulation and control
of robot systems. The library is used for research and education as
well as for practical robot applications. Features of the library
include:

- Kinematic modeling of various types of industrial manipulators.

- Path-planning and inverse kinematics algorithms.

- Drivers for robots and sensors available in our lab.
.

Workcells and their operation are visualized in a seperate application
called \b RobWorkStudio.

RobWork is developed at the robotics department of the <a
href="http://www.sdu.dk/mmmi">Mærsk McKinney Møller Institute</a> at
the <a href="http://www.sdu.dk/mmmi">University of Southern
Denmark</a>. The focus of the department is on industrial robots and
their applications.

\section sec_index_manual Manuals and tutorials

- \ref page_rw_manual

- \ref page_rwstudio_manual

\section sec_index_download Download

Download \b RobWork and \b RobWorkStudio from the <a
href="http://www.robwork.dk">RobWork homepage</a>.

\section sec_index_installation Installation

- \ref page_rw_installation

- \ref page_rwstudio_installation

\section License

RobWork is distributed under a
\ref page_license "BSD style license". For convenience, a number of
open-source libraries are distributed together with RobWork; the
RobWork license does not apply to these libraries.

\section sec_index_mailinglists Mailing lists

<a href="http://mls.sdu.dk/mailman/listinfo/robwork">RobWork mailing list</a>

\section sec_index_about OM DENNE MANUAL IFHT. HJEMMESIDEN

Af permanente ting vil jeg i Joomla placere:

- Introduktion (home page)
- Manual (link til her)
- Download (binære og source-code distributioner)
- Mailing list (via mailman)
- License
- Links
- Forum
- Screenshots and demos (dvs. noget visen frem af hvad RobWork kan)

Dette vil jeg slå ihjel på hjemmesiden:

- About RobWork (brug homepage til at fortælle dette)
- RobWork history (who cares?)
- Exercises (brug Blackboard til kursusrelaterede ting)
- Tutorials (ligger i manualen)
- RobWork api-doc (et link til manualen er nok)
- RW File format (ligger i manualen)
- Presentations (brug Screenshots and demos)

Jo mindre der ligger på hjemmesiden des bedre. Vi har ikke resurser
til at vedligeholde et kompliceret setup.

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
