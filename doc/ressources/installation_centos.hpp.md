Installation on CentOS {#page_rw_installation_centos}
============ 
[TOC]

# Introduction # {#sec_rw_install_centos_intro}
RobWork needs to be built from source by the user. This guide shows the steps for doing this in CentOS 7. If you have any suggestions or additions to the guide, please post it on the issue tracker https://gitlab.com/caro-sdu/RobWork/issues .

The main motivation for supporting CentOS, is the potential use of RobWork on the SDU Abacus cluster. See also https://abacus.deic.dk . 

RobWork is basically multiple projects:
- RobWork : is the core part including math, kinematics, planning and so on.
- RobWorkStudio : is the GUI which enable visualization and more userfriendly interfaces through gui plugins
- RobWorkSim : is an extension to the RobWork core functionality which adds dynamic simulation of bodies, devices and several tactile sensors.
- RobWorkHardware : is mostly drivers (with RobWork datatypes) for common hardware, or hardware on which RobWork platforms have been built eg. SDH, cameras, CAN-devices, the Universal robot arm, serial port...

Note that RobWork is needed to run RobWorkStudio, RobWorkSim and RobWorkHardware. Therefore it is not possible to use these, without having RobWork installed on the machine. 

# Installing dependencies # {#sec_centos_dependencies}

RobWork depends on third-party software that must be installed prior to compilation. This includes both build tools and third-party libraries.
In Linux it is quite easy to set up the dependencies as these are available as packages in the systems package manager.
Unfortunately, in CentOS it is also necessary to compile some optional dependencies from scratch.

## ¤ Build Tools ## {#sec_centos_dependencies_tools}

To be able to checkout code it is necessary to install some source code management (SCM) tools, such as Subversion, Git and Mercurial.
To be able to checkout the code from our own SVN repository, a SVN client is needed.
Git and Mercurial clients are not strictly required, but depending on your needs it might be worthwhile to install them.

	sudo yum install subversion git mercurial

To compile the C++ code, the GCC compiler should be used on Ubuntu.
CMake must be used to prepare RobWork for compilation. The minimum CMake version for RobWork is currently 2.8.12, which is also available in CentOS 7.

	sudo yum install make gcc gcc-c++ cmake

## ¤ Extra Packages for Enterprise Linux (EPEL) ## {#sec_centos_dependencies_epel}

Some packages are only available when the "Extra Packages for Enterprise Linux" (EPEL) is used.
Add the packages with:

	sudo yum install epel-release

This is for instance needed for Assimp, GTest and Bullet and libdc1394.

## ¤ RobWork Required Dependencies ## {#sec_centos_dependencies_rw}
Start by installing the dependencies. This is done using the package manager by running the following commands in a terminal.
First, install OpenGL libraries:

	sudo yum install mesa-libGL-devel mesa-libGLU-devel

Then install Boost:

	sudo yum install boost-devel 

Finally, install Xerces:

	sudo yum install xerces-c xerces-c-devel

Xerces is used some places in RobWork for opening XML files. It is still a requirement, but it is the intention that this dependency will one day become optional.

## ¤ RobWork Optional Dependencies ## {#sec_centos_dependencies_rw_optional}

SWIG (optional) is a tool that makes it possible to generate a LUA script interface for RobWork.
Python and Java interfaces are also possible, but require that Python or Java SDK is installed as well.
All of these interfaces can be generated if you install the following packages:

	sudo yum install swig swig lua-devel python-devel java-1.8.0-openjdk-devel

Google Test (optional) is used for unit tests in RobWork. If you are a developer and wants to develop code for the RobWork trunk, writing a GTest will be a requirement:

	sudo yum install gtest-devel

## ¤ RobWorkStudio Dependencies ##  {#sec_centos_dependencies_rws}

RobWorkStudio requires Qt to be installed. Both Qt4 and Qt5 is supported, but on a fresh Qt install it is encouraged to choose the Qt5 version:

	sudo yum install qt5-qtbase-devel

## ¤ RobWorkSim Dependencies ##  {#sec_centos_dependencies_rwsim}
If you need to do dynamic simulations, you will probably need the RobWorkSim package. If you are in doubt and just need RobWorkStudio, you can likely skip this.

Open Dynamics Engine (ODE) is not available in the package manager. Instead, ODE must be compiled from source.
Use Mercurial to download the source from bitbucket:

	hg clone https://bitbucket.org/odedevs/ode

Open a terminal and run:

	./bootstrap
	./configure --enable-double-precision --enable-shared --enable-ou --enable-builtin-threading-impl --disable-demos --disable-asserts
	make -j4

This will make sure that ODE is built with 4 threads with double precision as a shared library.

Bullet Physics can be installed through the package manager:

	sudo yum install bullet-devel

It is also possible to compile Bullet Physics from source, if a specific version is needed. Clone the source code with git:

	git clone https://github.com/bulletphysics/bullet3

Make a Build folder and run CMake to configure the build. From within the Build folder, run in a terminal:

	cmake -DCMAKE_BUILD_TYPE=Release -DUSE_DOUBLE_PRECISION=ON -DBUILD_BULLET3=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX:PATH=$WORKSPACE/Release -DCMAKE_CXX_FLAGS="-fPIC" -DCMAKE_C_FLAGS="-fPIC" -DBUILD_EXTRAS=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_UNIT_TESTS=OFF -BUILD_CPU_DEMOS=OFF ..
	make -j4

Modify the options to suit your needs. The shown options will make sure that Bullet is built with double precision, required compile flags and switch off building of things that are normally unnecessary when used in RobWorkSim.

RobWork Physics Engine (RWPE) requires access to code that is not yet public. Request more information about this if you need it.

## ¤ RobWorkHardware Dependencies ##  {#sec_centos_dependencies_rwhw}

RobWorkHardware compilation depends heavily on which hardware you need to use. Install the following package: 

	sudo yum install libdc1394-22-dev

It is not currently possible to give any general instructions for RobWorkHardware.

# Building RobWork # {#sec_centos_build}

When the dependencies have been installed, RobWork is ready to be built. First, the source must be downloaded, followed by the build procedure.

## Getting source files from SVN ## {#sec_centos_build_svn}
Make a new directory where you want to install RobWork (in this guide, we will install in ~/RobWork): 

	mkdir RobWork
	cd RobWork


When the depencies are installed, go ahead and download the newest version of RobWork from the SVN repository at: 

https://svnsrv.sdu.dk/svn/RobWork/trunk/

Using: 

- Username: 'Guest'
- Password: ''

In the terminal, this is done as follows: (be sure that you are located in the directory where you want to install RobWork)

	svn co --username Guest --password '' https://svnsrv.sdu.dk/svn/RobWork/trunk/ .

## Setup CMake Options & Environment ## {#sec_rw_install_centos_environment}

Before running CMake to build RobWork, some environment variables might need to be set. This is generally not needed when installing dependencies through the package manager.
If one or more dependencies were compiled manually, one must be careful that CMake actually finds the dependency correctly.
A good advice before building RobWork, is to actually read the CMake output carefully.
Running CMake will be discussed later, but the CMake output will typically reveal early in the process if a dependency was not found.
Building RobWork can take quite some time, and it is a pitty building everything, just to discover that some functionality was disabled due to a unmet dependency
(especially a problem for the optional dependencies).

There are overall two methods to let RobWork know where a dependency is installed. One is to set an environment variable, another is to set CMake options when running the CMake command.
Environment variables can be set up one time for all in the users home folder in the .bashrc file, while CMake options has to be specified each time you need to rebuild RobWork from scratch.
The later does however give more fine-grained control, as it allows multiple versions of dependencies to be installed on the system.
The version to use is then selected explicitly when running CMake.

In [CMake Options & Environment](@ref page_rw_installation_cmake_options) we try to give an overview of the correct variables to set for the various dependencies.

## Compiling RobWork ## {#sec_centos_build_compile}

Add build directories for the projects you want to build:

	mkdir Build
	mkdir Build/RW
	mkdir Build/RWS

Now we are ready to build RobWork. Run CMake:

	cd Build/RW
	cmake -DCMAKE_BUILD_TYPE=Release ../../RobWork

Look carefully through the CMake output and check that there is no errors, and that the required dependencies are correctly found.
Now that the CMake files has been built, we are ready to compile the project. Using 4 cores/threads, this is done by: 

	make -j4

Note that you need at least 1 GB of memory per thread when building. Ie. building with 4 cores requires around 4 GB of RAM. 

For RobWorkStudio, the same procedure is repeated in the RWS build folder, and similar for RobWorkSim and RobWorkHardware if needed.

Finally, we need to add the following paths to ~/.bashrc:

~~~~{~/.bashrc}
#ROBWORK#
export RW_ROOT=~/RobWork/trunk/RobWork/
export RWS_ROOT=~/RobWork/trunk/RobWorkStudio/
export RWHW_ROOT=~/RobWork/trunk/RobWorkHardware/
export RWSIM_ROOT=~/RobWork/trunk/RobWorkSim/
~~~~

Remember to only add paths to the components you have actually installed. Ie. if you only installed RobWork and RobWorkStudio, the paths for RobWorkSim and RobWorkHardware should not be set. 




