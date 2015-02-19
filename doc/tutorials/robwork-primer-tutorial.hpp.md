RobWork - A Programming Primer  {#pageRobworkPrimer}
===============================

[TOC]

This collection of basic tutorials and exercises are provided to ease and aid the
introduction to programming applications using RobWork.

# Introduction # {#sec_rw_fformats_intro}

RobWork is a framework/library for simulation and control of robotics with emphasis
on industrial robotics and their applications.
The project was started in 2006 by Ph.D. students and master students at the Maersk Mc-Kinney Moeller
Institute, University of Southern Denmark.
Today the project is more mature and used by both researchers, Ph.D. students, master students and
robotics students. RobWork is mainly devided into 2 different parts. The basic
framework named RobWork consist of basic mathematics used for robots and algorithms — e.g.
motion planning and inverse kinematics.
The second part of the framework named RobWorkHardware contains drivers and code for
communicating with robots, cameras, canbus and others.
The major goal of the framework is to:

- Provide a single framework for offline and online robot programming including modelling,
 simulation and (realtime)control of robotics

The target users are:
 - Researchers who needs a common framework for experimental robotics
 - Students (you) who wants to experiment with the concepts of robotics
 - Implementers of robot applications

 RobWork is currently being used for research and student exercises at the University of Southern Denmark.
 You can read a lot more information at the RobWork website: [http://www.robwork.dk]
 The following tutorials/exercises assume a functional installation of both RobWork and RobWorkStudio.
 Please go to the homepage for installation tutorials.

### Exercises ### 
The exercises in the tutorials are often created with the intent of the user to retrieve
information/functionality by using the API-DOC. This should enable the user to become more
familiar with the base functionality of RobWork and also encurage the use of apidoc to find
functionality. The apidoc is available on the <a href="http://www.robwork.dk">homepage</a>.

## My first cmake project ## 
This tutorial will demonstrate how to create a simple RobWork application using cmake.
First create a directory for your project. Create a “CMakeLists.txt” file and a “HelloWorld.cpp” in
the directory. Insert the following cmake script into the CMakeLists.txt file:

~~~{.cmake}
 CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
 # The name of the project. (EDIT THIS)
 PROJECT(TutorialCollection)

 # Used to resolve absolute path names
 SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

 ##### IMPORTANT EDIT THESE PATHS TO REFLECT YOUR ROBWORK INSTALLATION #####
 # Now set the RW/RWS root (edit this if necessary)
 SET(RW_ROOT "${ROOT}/../../../RobWork")

 # set some robwork settings (EDIT THESE TO FIT YOUR ENVIRONMENT)
 SET(CMAKE_BUILD_TYPE Release)

 #Include default settings for constructing a robwork dependent project
 SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
 FIND_PACKAGE(RobWork)
 LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

 # if you have additional libraries or include dirs then add them here
 INCLUDE_DIRECTORIES(${ROOT} ${ROBWORK_INCLUDE_DIR})

 # And now we add any targets that we want
 add_executable(HelloWorld HelloWorld.cpp)
 target_link_libraries(HelloWorld ${ROBWORK_LIBRARIES})
~~~

 The script should be pretty much self explaining. However, it is important that the RW_ROOT is set to the
 correct paths of robwork.
 Now edit you “HelloWorld.cpp” file and type in a small main application.

~~~{.cpp}
 #include <rw/common/Log.hpp>
 using namespace rw::common;
 int main(int argc, char** argv) {
   Log::infoLog() << "Hey, we are printing to the RobWork log!\n";
   std::cout << "Which should just be standard out for now!" << std::endl;
 }
~~~

This is very basic but the reader should notice two important aspects. Firstly, the header file
rw/common/Log.hpp is included. This file includes only the Log class functionality which is the only class
used in the code. Secondly the “using namespace” clause is used such that we can call Log::infoLog()
instead of rw::common::Log::infoLog().
When many different namespaces and classes from robwork are used, it can be somewhat tedious to write
the “using namespace” and includes in every file. Instead a general header file rw/rw.hpp and a macro can
be used to assemble all classes into one namespace: “robwork”. We rewrite the code snippet from above:

~~~{.cpp}
 #include <rw/rw.hpp>
 USE_ROBWORK_NAMESPACE
 using namespace robwork;

 int main(int argc, char** argv) {
   Log::infoLog() << "The using namespace enables us to call Log directly!\n";
   rw::common::Log::infoLog() << "We can still use the native namespace!\n";
   robwork::Log::infoLog() << "but also the general namespace!\n";
 }
~~~


 Notice that when using this type friendly shortcut, the risk of name clashes between robwork classes and
 other libraries become much higher. Also, NEVER use “using namespace” in headerfiles, unless its within a
 function scope.


## Math jogling ## {#sec-rw-tut-math-jogling}

 This tutorial will demonstrate some of the basic math functionality available in RobWork. This is mostly
 related to homogenous transformations, rotations, convertions and so on.
 First add a new file “MathJogling.cpp” to your cmake project from tutorial 1. Make sure that the file is
 added as an executable in the end of the CMakeList.txt file.

~~~{.cmake}
 # add another executable
 add_executable(MathJogling MathJogling.cpp)
 target_link_libraries(MathJogling ${ROBWORK_LIBRARIES})
~~~

 Add the standard static main code body in the “MathJogling.cpp” and we are ready to play.

~~~{.cpp}
 #include <rw/rw.hpp>
 USE_ROBWORK_NAMESPACE
 using namespace robwork;
 int main(int argc, char** argv) {
 // main body, add your code here
 }
~~~

 The main use of the math package is homogenous transformations, rotations, vectors. However, before
 venturing into mathematical expressions we need to look at the different rotation representations. The
 most user friendly format is probably euler angles where RobWork use a fixed axis ZYX euler representation
 using the class “RPY” (Roll Pitch Yaw). The following snippet illustrates conversions between the rotation
 formats.

~~~{.cpp}
 RPY<> rpy(0, 0, 90*Deg2Rad); // 90 degree rotation around x-axis
 Rotation3D<> rot = rpy.toRotation3D(); // create Rotation3D matrix
 EAA<> eaa( rot ); // construct eaa form rotation3d
 Quaternion<> quat( rot ); // construct quaternion from rotation3d
 // there are streaming operators for all math types
 Log::infoLog() << rpy << std::endl;
 Log::infoLog() << rot << std::endl;
 Log::infoLog() << eaa << std::endl;
 Log::infoLog() << quat << std::endl;
~~~

 Operators are used throughout the math package to enable intuitive math expressions and all so streaming
 as shown above. Now lets look at some of the most used functions in the math package.

~~~{.cpp}
 // rotate a vector (0,1,0) 90 degrees around x-axis
 Log::infoLog() << rot*Vector3D<>(0,1,0) << std::endl;
 // transform a vector
 Transform3D<> t1( Vector3D<>(0,0,1), rot);
 Log::infoLog() << t1*Vector3D<>(0,1,0) << std::endl;
 // calcualte the inverse rotation
 Log::infoLog() << inverse( rot ) << std::endl;
 // calculate the inverse transform
 Log::infoLog() << inverse( t1 ) << std::endl;
 // get the rotation and translation part of a transform
 Log::infoLog() << t1.R() << t1.P() << std::endl;
~~~

tutorial/MathJoggling.cpp

### Exercise 1 - Transformations ### 
Try to set up two transformations T1 and T2. Set T1 with a position (x; y; z) = (1; 1; 1) and rotation (r;
p; y) = (90; 0; 0). Set T2 with a position (x; y; z) = (0; 0; 1) and rotation (r; p; y) = (0; 0; 0).

RobWork uses radians and not degrees. Take this into account.

Now calculate T1T2. What is the result? Construct a drawing of the frames (by hand)!

### Exercise 2 – Point transform ### 

Now consider that T1 and T2 are frames in a robotic system. T2 is described relative to T1 and T1 relative to
the world frame T0.

Using the two transformations from the previous exercise, calculate the position of pT2 = (0:5; 1; 0) with
respect to frame T1 and T0.

What is the result?

## WorkCell, Devices and forward kinematics ## 
Instead of hardcoding robotic descriptions and scene descriptions into C++ files, devices and scenes can be
described using an XML file.
This section with exercises will guide you through the usage of these files. First extend the cmake project
from tutorial 1 with a file “WorkCellAndDevices.cpp” and create the file with an empty main body.
Loading a !WorkCell in RobWork is fairly simple:

~~~{.cpp}
 WorkCell::Ptr wc = WorkCellLoader::Factory::load("SimpleWorkCell.wc.xml");
~~~

 The typical functions of the workcell is to create a default State and add/remove/find frames or devices. The
 templated find functions are especially usefull if specific frame or device types need to be found.

~~~{.cpp}
 Log::infoLog() << "Name of workcell: " << wc->getName() << std::endl;
 // get the default state
 State state = wc->getDefaultState();
 Frame* worldFrame = wc->getWorldFrame();
 // find a frame by name, remember NULL is a valid return
 Frame* frame = wc->findFrame("FixedFrameName");
 // find a frame by name, but with a specific frame type
 FixedFrame* fframe = wc->findFrame<FixedFrame>("FixedFrameName");
 MovableFrame* mframe = wc->findFrame<MovableFrame>("MovableFrameName");
 // find a device by name
 Device::Ptr wc->findDevice("SerialDeviceName");
 SerialDevice::Ptr wc->findDevice<SerialDevice>("SerialDeviceName");
~~~

 The basic building blocks of a workcell are Frames. These are ordered in a tree-structure where the ROOT
 node allways is the WORLD frame. All frames has a parent which thier position and orientation is relative to.
 Descending in this tree accumulating frame transformations is basically forward kinematics. The “Kinematics”
 class is a utility class for calculating forward kinematics, reattaching frames(gripping),

~~~{.cpp}
 // calculate the transform from one frame to another
 Transform3D<> fTmf = Kinematics::frameTframe(frame, mframe, state);
 // calculate the transform from world to frame
 Transform3D<> wTmf = Kinematics::worldTframe( mframe, state );
 // we can find the world to frame transform by a little jogling
 Transform3D<> wTf = wTmf * inverse(fTmf);
 // test if frame is a dynamic attachable frame
 if( Kinematics::isDAF( mframe ) ){
 	// attach mframe to end of serial device
 	Kinematics::gripFrame(state, mframe, sdevice->getEnd() );
 }
~~~

 The device class also define utility functions for calculating forward kinematics, at least those that relate to
 the device. Additionally the Device has functionality to compute the Device Jacobian, setting and getting
 the joint configurations and getting the joint limits.
~~~{.cpp}
 // get device base frame
 Frame *base = sdevice->getBase();
 // get device end effector
 Frame *end = sdevice->getEnd();
 // calculate base to end transform
 Transform3D<> bTe = sdevice->baseTend(state);
 // or just base to any frame
 Transform3D<> bTmf = sdevice->baseTframe(mframe, state);
 // get device name
 std::string sdevicename = sdevice->getName();
 // the degrees of freedom of this device
 int dof = sdevice->getDOF();
 // set the configuration of the device to zero
 sdevice->setQ( Q::zero(dof) , state );
~~~

### Exercise 1 – WorkCell ### 
 Construct code, that load a workcell into your software and print the workcell name. Use the workcell to
 retrieve all the devices in the scene. Print the name of each device to the standard output (terminal).
 To solve this exercise, you should consider the following classes: WorkCellLoader, WorkCell and Device.

### Exercise 2 – Forward kinematics ### 
 Using the code from the previous exercise, you have loaded a workcell into your program. Extract the first
 device of the workcell. Set the robot in a configuration of: _(0.451; 1.4; 0.976; 0.0; 0.76; 0.0)_. This
 configuration is given in radians.

 Calculate the forward kinematics of the robot in the configuration given. You should only consider the robot
 base to end effector for this calculation.

### Exercise 3 – More Forward kinematics ### 
 Now, you should consider, that the robot might not be placed in a non-zero position and orientation relative
 to the world frame.
 Calculate the transformation from world frame to end effector frame. Are the transformations equivalent?
 Why or why not?
 For this exercise you should consider the class Kinematics.


## Inverse Kinematics ## 

