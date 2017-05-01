Installation on Windows {#page_rw_installation_windows}
============ 
[TOC]

# Introduction # {#sec_rw_install_win_intro}
This guide shows the steps for building the RobWork packages on a Windows platform.
The guide is written based on a setup with Windows 7 and Visual Studio 2015 and the guide is last revised in May 2017.
If you have any suggestions or additions to the guide, please post them on the issue tracker at https://gitlab.com/caro-sdu/RobWork/issues .

RobWork is basically multiple projects:
- RobWork : is the core part including math, kinematics, planning and so on.
- RobWorkStudio : is the GUI which enable visualization and more userfriendly interfaces through gui plugins
- RobWorkSim : is an extension to the RobWork core functionality which adds dynamic simulation of bodies, devices and several tactile sensors.
- RobWorkHardware : is mostly drivers (with RobWork datatypes) for common hardware, or hardware on which RobWork platforms have been built eg. SDH, cameras, CAN-devices, the Universal robot arm, serial port...

Note that RobWork is needed to run RobWorkStudio, RobWorkSim and RobWorkHardware. Therefore it is not possible to use these, without having RobWork installed on the machine. 

# Installing dependencies # {#sec_win_dependencies}

RobWork depends on third-party software that must be installed prior to compilation. This includes both build tools and third-party libraries.
In Windows it can be a bit tedious to set up all the dependencies, but most packages are easily installed using standard installers.
Unfortunately, it is also necessary to compile some dependencies from scratch.

## ¤ Build Tools ## {#sec_win_dependencies_tools}

To be able to checkout code it is necessary to install some source code management (SCM) tools, such as Subversion, Git and Mercurial.
To be able to checkout the code from our own SVN repository, a SVN client is needed. The latest version of the Tortoise SVN client is recommended for this.
Tortoise SVN comes with a GUI that is easy to use and is nicely integrated with explorer.
Git and Mercurial clients are not strictly required, but depending on your needs it might be worthwhile to install them. The tools can be found on the following pages:

Tortoise SVN client: https://tortoisesvn.net

Git client: https://git-scm.com

TortoiseHg Mercurial client: https://www.mercurial-scm.org
 
Microsoft Visual Studio should be used to compile RobWork on Windows platforms. RobWork is expected to compile on Windows 7 or newer, using Visual Studio 2010 or newer.
Currently, RobWork is continuously tested on Windows 7 using Visual Studio 2015. 
If you are a student at University of Southern Denmark, please see the following page for information about access to Microsoft products:

http://www.sdu.dk/en/information_til/studerende_ved_sdu/campusguide/it/software

CMake must be used to generate projects for Visual Studio. A Windows installer can be downloaded from the CMake homepage at https://cmake.org .
The minimum CMake version for RobWork is currently 2.8.12, but choosing the latest version is always recommended.
Choosing older versions will mean that newer Visual Studio and Boost versions will not be supported.
If you already have an older version of CMake installed, please check that it is recent enough to support your setup: 

| CMake Version | Maximum Visual Studio Version Supported | Maximum Boost Version Supported |
|---------------|-----------------------------------------|---------------------------------|
| 3.8.0         | Visual Studio 15 2017                   | 1.64.0                          |
| 3.7.2         | Visual Studio 15 2017                   | 1.63.0                          |
| 3.7.0-3.7.1   | Visual Studio 15 2017                   | 1.62.0                          |
| 3.5.0-3.6.3   | Visual Studio 14 2015                   | 1.61.0                          |
| 3.4.0-3.4.3   | Visual Studio 14 2015                   | 1.59.0                          |
| 3.1.1-3.3.2   | Visual Studio 14 2015                   | 1.58.0                          |
| 3.1.0         | Visual Studio 14 2015                   | 1.56.0                          |
| 3.0.2         | Visual Studio 12 2013                   | 1.56.0                          |
| 2.8.12        | Visual Studio 12 2013                   | 1.56.0                          |

## ¤ RobWork Required Dependencies ## {#sec_win_dependencies_rw}
Boost is the most important dependency in RobWork, and it is recommended to always use the latest possible version of Boost when possible.
RobWork is also backwards compatible with older versions of Boost, mainly to support current Ubuntu LTS releases and CentOS 7. For this reason, it is strongly recommended to use at least Boost 1.53.
Boost precompiled libraries can be found at:

https://sourceforge.net/projects/boost/files/boost-binaries

It is also possible to compile the Boost libraries from source. From a command prompt run something similar to:

	bootstrap.bat
	b2 -j4 --with-filesystem --with-system --with-program_options --with-regex --with-serialization --with-thread --with-date_time --with-chrono --with-test --prefix=.\ address-model=64 link=shared install

Here -j gives the number of threads to use for compilation. Run with -help, -help-options or \-\-show-libraries to get more information about the various options.

Xerces is used some places in RobWork for opening XML files. It is still a requirement, but it is the intention that this dependency will one day become optional.
It is necessary to compile the Xerces library:
 -# Go to http://xerces.apache.org and download the latest source distribution.
 -# Unpack it where you want Xerces installed.
 -# Open xerces-c-3.1.4/projects/Win32/VCxx/xerces-all.sln in Visual Studio (substitute VCxx with your Visual Studio version - see https://en.wikipedia.org/wiki/Microsoft_Visual_Studio#History for overview).
 -# Choose 64-bit Release build configuration, and build the XercesLib target.

## ¤ RobWork Optional Dependencies ## {#sec_win_dependencies_rw_optional}

SWIG (optional) is a tool that makes it possible to generate a LUA script interface for RobWork.
Python and Java interfaces are also possible, but require that Python or Java SDK is installed as well. The SWIG tool is easily downloaded from:

https://sourceforge.net/projects/swig/files/swigwin

The tool needs no compilation. Simply extract the files from the zip-file where you want SWIG installed.

Google Test (optional) is used for unit tests in RobWork. If you are a developer and wants to develop code for the RobWork trunk, writing a GTest will be a requirement.
In this case, open a terminal, go to the folder where you want the code, and clone the Google Test code:

	git clone https://github.com/google/googletest.git

The Google Test code should not be compiled. It will be compiled as a part of the RobWork compilation when the source code is present.

## ¤ RobWorkStudio Dependencies ## {#sec_win_dependencies_rws}
RobWorkStudio requires Qt to be installed. Both Qt4 and Qt5 is supported, but on a fresh Qt install it is encouraged to choose the latest Qt5 version (for now, skip 5.8).
Download and install Qt from:

https://www.qt.io

You need to answer questions about your use of Qt. Qt is only free for open source projects.

<b>WARNING! Please avoid Qt 5.8</b>  ( see issue https://gitlab.com/caro-sdu/RobWork/issues/37 )

## ¤ RobWorkSim Dependencies ## {#sec_win_dependencies_rwsim}

If you need to do dynamic simulations, you will probably need the RobWorkSim package. If you are in doubt and just need RobWorkStudio, you can likely skip this.

Open Dynamics Engine (ODE) must be compiled from source. Use TortoiseHg (Mercurial) to download the source from bitbucket:

	https://bitbucket.org/odedevs/ode

Open a terminal and go to the build folder to run premake4:

	premake4.exe --only-double --only-shared --with-ou --with-builtin-threading-impl --os=windows --platform=x64 vs2010

This will make sure that ODE is built with double precision as a 64-bit shared library.
The \-\-with-builtin-threading-impl does not exist from version 0.15, as it is now default.
Unfortunately, Visual Studio 2010 is the latest supported version by the premake4 program.
When the ode.sln is opened, Visual Studio will upgrade to a newer format. Select 64-bit Release configuration and build the solution.

Bullet Physics must be compiled from source. Clone the source code with git:

	git clone https://github.com/bulletphysics/bullet3

Make a Build folder and run CMake to generate Visual Studio solutions. From within the Build folder, run in a terminal:

	cmake .. -G "Visual Studio 14 2015 Win64" -DUSE_DOUBLE_PRECISION=ON -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON -DBUILD_EXTRAS=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_CPU_DEMOS=OFF -DBUILD_OPENGL3_DEMOS=OFF -DBUILD_BULLET2_DEMOS=OFF -DINSTALL_LIBS=ON -DCMAKE_INSTALL_PREFIX:PATH=SOME_INSTALL_DIR

Choose the CMake generator that fits your Visual Studio version. Modify the options to suit your needs.
The shown options will make sure that Bullet is built with double precision, shared runtime and switch off building of things that are normally unnecessary when used in RobWorkSim.
In case you want to install Bullet after building it, set the INSTALL_LIBS variable to ON and set the CMAKE_INSTALL_PREFIX variable to the directory you want to install in.
Installing is optional. To build Bullet, open BULLET_PHYSICS.sln, choose the Release configuration and build the solutions. To install build the INSTALL target.

RobWork Physics Engine (RWPE) requires access to code that is not yet public. Request more information about this if you need it.

## ¤ RobWorkHardware Dependencies ##  {#sec_win_dependencies_rwhw}

RobWorkHardware compilation depends heavily on which hardware you need to use. It is not currently possible to give any general instructions for RobWorkHardware.

# Building RobWork # {#sec_win_build}

When the dependencies have been installed, RobWork is ready to be built. First, the source must be downloaded, followed by the build procedure.

## Getting RobWork source files from SVN ## {#sec_win_build_svn}
Make a new directory where you want to install RobWork. When the dependencies are installed, go ahead and download the newest version of RobWork from the SVN repository at: 

https://svnsrv.sdu.dk/svn/RobWork/trunk

Using: 

- Username: 'Guest'
- Password: ''

In the terminal, this is done as follows: (be sure that you are located in the directory where you want to install RobWork)

	svn co --username Guest --password '' https://svnsrv.sdu.dk/svn/RobWork/trunk/ .

There should now be RobWork, RobWorkStudio, RobWorkSim and RobWorkHardware folders. Alternatively, the individual projects can be downloaded directly:

	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWork
	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWorkStudio
	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWorkSim
	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWorkHardware

## Setup CMake Options & Environment ## {#sec_rw_install_win_environment}

Setting up the environment variables is known as one of the challenging tasks when compiling RobWork on a Windows platform. One thing is to install and compile all the needed
dependencies, another is to make sure that RobWork actually finds these dependencies. A good advice before building RobWork, is to actually read the CMake output carefully.
Running CMake will be discussed later, but the CMake output will typically reveal early in the process if a dependency was not found.
Building RobWork can take quite some time, and it is a pitty building everything, just to discover that some functionality was disabled due to a unmet dependency
(especially a problem for the optional dependencies).

There are overall two methods to let RobWork know where a dependency is installed. One is to set an environment variable, another is to set CMake options when running the CMake command.
Environment variables can be set up one time for all in the system, while CMake options has to be specified each time you need to rebuild RobWork from scratch.
The later does however give more fine-grained control, as it allows multiple versions of dependencies to be installed on the system.
The version to use is then selected explicitly when running CMake.

In [CMake Options & Environment](@ref page_rw_installation_cmake_options) we try to give an overview of the correct variables to set for the various dependencies.

## Compiling RobWork ## {#sec_win_build_compile}

Open a Visual Studio 64-bit command prompt and go to the directory where RobWork was checked out from SVN.
Add new build directories for the RobWork packages you want to compile, such as:

	mkdir Build
	mkdir Build\RW
	mkdir Build\RWS

Now we are ready to build RobWork. Choose the generator that matches your Visual Studio version. For Visual Studio 2015, use the following:

	cd Build\RW
	cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 14 2015 Win64" ../../RobWork

Look carefully through the CMake output and check that there is no errors, and that the required dependencies are correctly found.
Now that the CMake files has been built, we are ready to compile the project. In the build_rw folder there will now be a RobWork.sln solution that can be opened in Visual Studio.
Choose the correct configuration and build the solution.

For RobWorkStudio, the same procedure is repeated in the RWS build folder, and similar for RobWorkSim and RobWorkHardware if needed.