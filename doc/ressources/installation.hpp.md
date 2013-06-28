Installation {#page_rw_installation}
============ 
[TOC]

# Introduction # {#sec_rw_install_intro}
RobWork uses several external libraries such as
Boost, Xerces and for RobWorkStudio also Qt. Furthermore, RobWork is developed
for multiple platforms (Linux, Win7/xp MinGW, Mac, and Win7/xp Visual Studio) which
sometimes complicates the installation.

Here you find articles that describe how to install RobWork or compile RobWork
and the dependencies on several different platforms.


# Installing RobWork SDK # {#sec_rw_install_SDK}
The quickest installation obtained through the provided SDK packages for Ubuntu (Debian packages)
and Windows (NSIS installers). The Debian packages automatically install all dependencies and
the NSIS installers contain headers and precompiled libraries for all dependencies.
These SDKs are suitable for users who want to get started quickly with the development of
RobWork and RobWorkStudio plugins.

Please look at the [download](http://www.robwork.dk/jrobwork/index.php?option=com_jdownloads&view=viewcategory&catid=8&Itemid=83)_ section on the homepage for downloading the SDK packages.

# Installing RobWork from source # 		{#sec-rw-install-compile}
This section will explain how to install RobWork from source. Several platforms and compilers
are supported and one should look for the description including he's platform.

RobWork is basically multiple projects
- RobWork : is the core part including math, kinematics, planning and so on.
- RobWorkStudio : is the GUI which enable visualization and more userfriendly interfaces through gui plugins
- RobWorkSim : is an extension to the RobWork core functionality which adds dynamic simulation of bodies, devices and several tactile sensors.
- RobWorkHardware : is mostly drivers (with RobWork datatypes) for common hardware, or hardware on which RobWork platforms have been built eg. SDH, cameras, CAN-devices, the Universal robot arm, serial port...

Naturally all projects depend on the core functionality of RobWork, however if functioanlity in RobWorkSim or RobWorkStudio is not necesary
then one does not need to install/compile it. So one can download project sources individually and compile them or download all of it and
compile it all together. The latter is the easiest approach.

## Downloading the source ## {#sec-rw-install-download}
RobWork source is made available through half year releases and SVN. The source releases can be downloaded
from the RobWork homepage.

You can get the latest developer snapshot of the RobWork source code from SVN using this account:

	Username: Guest
	Password: 

For svn use the complete project:

Complete SVN repository: 
	https://svnsrv.sdu.dk/svn/RobWork/trunk

or one of the specific project folders:
 
	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWork
	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWorkStudio
	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWorkSim
	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWorkHardware

Since the trunk is actively developed there might be bugs or compile issues in the source. Therefore
check out the nightly builds on our cdash servers, these should indicate any problems there might be on
the different supported platforms.

The trunk is compiled nightly with several different compilers on both 32 and 64 bit platforms (Ubuntu Linux, Windows XP and Windows 7).
The status of these builds is submitted to the following dashboards which enable users to keep track of broken builds:

	http://www.robwork.dk/cdash/index.php?project=RobWork
	http://www.robwork.dk/cdash/index.php?project=RobWorkStudio
	http://www.robwork.dk/cdash/index.php?project=RobWorkSim

## Installation instructions for the complete project ## {#sec-rw-install-complete}

Download (from homepage or svn) the complete RobWork-project and place it in a directory of your choice.
Lets call this directory RWPROJECTROOT.

The build system in RobWork is based on cmake which enables the support for several platforms and compilers.
Pleace download and install cmake from <a href="http://www.cmake.org">CMake</a>. The
top cmake file is placed at "RWPROJECTROOT/CMakeLists.txt" and a common approach to compiling the project is:

~~~~{.bash}
cd RWPROJECTROOT
cmake -DCMAKE_BUILD_TYPE=Relase -G"MinGW Makefiles" .
~~~~

This will make an *in-source* build which is fine if you only need the project for a specific build type or compiler.
A slightly nicer approach is to create a subdirectory say "buildrelease" in which you compile the project

~~~
cd RWPROJECTROOT
mkdir buildrelease
cd buildrelease
cmake -DCMAKE_BUILD_TYPE=Relase -G"MinGW Makefiles" ..
~~~

now all files generated in the compilation will be contained in "buildrelease" directory.

You can customize the build using -DCMAKE_BUILD_TYPE=Relase or -DCMAKE_BUILD_TYPE=Debug, changing options in the gui with ccmake (linux) or
CMakeSetup.exe (windows), and you can edit the build configuration files

- RWPROJECTROOT/RobWork/RobWork.cmake
- RWPROJECTROOT/RobWorkStudio/RobWorkStudio.cmake
- RWPROJECTROOT/RobWorkSim/RobWorkSim.cmake
- RWPROJECTROOT/RobWorkHardware/RobWorkHardware.cmake

This however should not be necessary unless you want to specifically force disabling a component such as LUA or if you
need to specify a variable explicitly such as BOOST_ROOT.


## Installation instructions for specific projects ## {#sec_rw_install_specific}

Download and install the *RobWork* and *RobWorkStudio* packages and
place them both in single directory of your choice. Uncompress the
packages.

Building of *RobWork* and *RobWorkStudio* is supported on multiple
platforms thanks to the <a href="http://www.cmake.org">CMake</a> build
system which you should install.

To customize the build process add and edit the following two files:

- RobWork/RobWork.cmake
- RobWorkStudio/RobWorkStudio.cmake

Templates with suggested contents for the above two files are included
in the packages:

- RobWork/RobWork.cmake.template
- RobWorkStudio/RobWorkStudio.cmake.template

To construct a build setup for the compiler of your choice, you must
run CMake from the root of the \c RobWork and \c RobWorkStudio
directories. The CMake command will be of the form

~~~~{.cmake}
cmake -G <generator name>
~~~~

where \c <generator \c name> is the name of the compilation system for
which the build setup should be constructed. Examples of generated
build setups are makefiles for GCC and project files for Microsoft
Visual Studio. See the sections below for compiler specific examples.

Using the generated build setup to first build \b RobWork and then \b
RobWorkStudio. Executables and libraries are by default written to the
\c libs and \c bin subdirectories of the \c RobWork and \c
RobWorkStudio directories. To link your own programs with the \b
RobWork system, you must add the \c RobWork/libs and \c
RobWorkStudio/libs directories to the compiler linker path and the \c
RobWork/src and \c RobWorkStudio/src directories to the include path.

You can test that \b RobWork has been built correctly by running the
generated program named \c TestSuite from the root of the \c
RobWork/test directory.

The \b RobWorkStudio program looks for files named \c
RobWorkStudio.ini and \c CustomRobWorkStudio.ini in the current
working directory. A template file \c
RobWorkStudio/bin/RobWorkStudio.ini_template shows suggested contents
for the \c RobWorkStudio.ini file. Start the \b RobWorkStudio program
and try opening a workcell file like for example \c
RobWork/docs/workcell.wu.

# Dependencies of RobWork # {#sec_rw_dependencies}
RobWork uses several external libraries such as Boost, Xerces and for RobWorkStudio Qt.
Furthermore, RobWork is developed for multiple platforms (GCC, MinGW and Visual Studio)
which sometimes complicates the installation.

In this section you find guides to install these dependencies on your specific platform.

## Installing dependencies on linux (Ubuntu) ## {#subsec_rw_dependencies_linux}
For RobWork core, the following is needed:

- gcc (version 4.1 or above), g++, cmake, blas-atlas, lapack-atlas, libxerces-c3.1, libboost-dev,
libboost-date-time-dev, libboost-filesystem-dev, libboost-program-options-dev, libboost-regex-dev,
libboost-serialization-dev, libboost-system-dev, libboost-test-dev, libboost-thread-dev, swig (optional)

using apt-get this is:

	sudo apt-get install gcc g++ cmake libatlas-base-dev libxerces-c3.1 libxerces-c-dev libboost-dev libboost-date-time-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev libboost-serialization-dev libboost-system-dev libboost-test-dev libboost-thread-dev swig

For RobWorkStudio additional libraries are needed: qt4-dev-tools, libqt4-dev, qt4-designer (optional)

	sudo apt-get install qt4-dev-tools libqt4-dev qt4-designer

And if RobWorkSim is required then installing the Open Dynamics Engine is strongly recommended.

- libode-dev, libode1

	sudo apt-get install libode-dev libode1

RobWorkHardware is a bit more difficult because more or less all packages depend on some specific driver,
software library or platform.


## Installing dependencies on windows with Visual Studio ## {#subsec_rw_dependencies_winvc}
For \b RobWork core functionality the following is needed:

- cmake, xerces 3.1, boost, swig (optional)

Cmake can be downloaded and installed from <a href="http://www.cmake.org">CMake</a>.

Precompiled versions of Xerces can be downloaded from http://xerces.apache.org/xerces-c/download.cgi.
For Visual Studio 2005, 2008 and 2010 you need to select vc8, vc9 and vc10, respectively.
Remember to update the environment variable XERCES_ROOT to the Xerces-C++ root path containing the include and lib folders.

To make life easier, we have created precompiled versions of Boost for Visual Studio.
The compressed files contain precompiled libraries for both debug and release variants for all
supported Visual Studio compilers (2005, 2008 and 2010).

- <a href="http://www.robwork.dk/data/ext/boost-1.45.0-msvc-x86.zip"> boost-1.45.0-msvc-x86.zip </a>
- <a href="http://www.robwork.dk/data/ext/boost-1.45.0-msvc-amd64.zip"> boost-1.45.0-msvc-amd64.zip </a>

You may install Boost wherever you like. Create a new environment variable BOOST_ROOT containing the path to
the Boost installation and Boost should be ready to use.

A more complete binary installer for various boost versions can be found on BoostPro
http://www.boostpro.com/download . Make sure to choose the MultiThreaded libs on install.

Alternatively download Boost from http://www.boost.org/users/download/ and compile it your self. When
configuring make sure to enable all packages, something like:


	bjam --with-date_time --with-filesystem --with-program_options --with-regex --with-serialization
	--with-system --with-test --with-thread --prefix=.\ --layout=tagged toolset=YOUR_TOOLSET install

SWIG can be downloaded and installed from <a href="http://www.swig.org/">SWIG</a>. Please specify the path to
SWIG in the RobWork.cmake eg. SET(SWIG_EXECUTABLE path_to_swig_exe_including_file)

For \b RobWorkStudio additional libraries are needed
- qt4

\b Visual Studio 2008 32 bit <br>
The Qt libraries for Visual Studio 2008 32 bit are prebuilt and therefore no compilation is needed.
Download Qt libraries for Windows (VS 2008) from http://qt.nokia.com/downloads under the Framework Only section.
 The Visual Studio Add-in can be found on the same page under Other Downloads.

First install the Qt libraries and then the Visual Studio Add-in. When installed, create an environment variable
QTDIR with the value c:/qt/VERSION and add c:/qt/VERSION/bin to the PATH environment variable where c:/qt/VERSION is
the installation directory.

\b Visual Studio 2005-2010 x86|amd64 <br>
Unfortunately Qt does not have a binary distribution for Visual Studio 2005 and 2010. You therefore need to download the source and compile Qt yourself.
Go to the Qt download page at http://qt.nokia.com/downloads. 
Choose the open source LGPL version and download the source code (zip or tar.gz) under the 
Framework Only section and extract the archive. 
Assuming you have extracted the Qt source into c:/Qt/VERSION, follow these steps:
- Open a Visual Studio terminal one of the following ways:
    - In the Visual Studio start menu folder, go to "Visual Studio Tools" and open "Visual Studion 2005|2008|2010 Command Prompt".
    - Open a command promt and run vcvarsall.bat x86|amd64 (found in e.g. c:/Program Files (x86)/Microsoft Visual Studio 8|9|10.0\VC) to setup the Visual Studio toolset
- In the command prompt cd to the c:/Qt/VERSION folder
- Visual Studio 10 amd64 users only: due to a bug reported here, Qt will produce a runtime error with the default compiler settings. Therefore, you need to modify the compiler flags for this platform:
    - Open c:/Qt/VERSION/mkspecs/win32-msvc2010/qmake.conf in a text editor
    - Change the optimization flag in "QMAKE_CFLAGS_RELEASE" from "-O2" to "-O1"
- Run (VERSION dependent):
    - 2010.04 (4.6) and higher: configure -opensource -no-sql-sqlite -no-qt3support -no-webkit -no-scripttools -fast -platform win32-msvc2005|win32-msvc2008|win32-msvc2010
    - 2009.04: configure -opensource -no-sql-sqlite -no-qt3support -no-webkit -fast -platform win32-msvc2005|win32-msvc2008|win32-msvc2010
- It is important to specify -platform to tell which toolset to use. The remaining options might be choosen differently. If asked, accept the license
- Call nmake to build Qt for Visual Studio (this step requires the patience of a saint)
- Create an environment variable QTDIR with the value c:/Qt/VERSION and add c:/Qt/VERSION/bin to the PATH environment variable

Qt Visual Studio Add-in:
- Download and install the Qt Visual Studio Add-in
- Start Visual Studio
- Select the menu "Qt" -> "Qt Options" and make sure the Qt Version points to the right directory

You are now ready to build Qt applications with Visual Studio.
If you have problems compiling, please visit: http://dcsoft.com/community\_server/blogs/dcsoft/archive/2009/03/06/how-to-setup-qt-4-5-visual-studio-integration.aspx for further information.


\b RobWorkHardware is a bit more difficult because more or less all packages depend on some specific driver,
software library or platform.


## Installing dependencies on windows with MinGW ## {#subsec_rw_dependencies_winmingw}
TODO: add mingw installation description. NOT CURRENTLY SUPPORTED
