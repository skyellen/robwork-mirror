Installation on Ubuntu {#page_rw_installation_ubuntu}
============ 
[TOC]

# Introduction # {#sec_rw_install_ubuntu_intro}
At the moment, no precompiled Debian packages are available for download. Therefore, RobWork needs to be built by the user. This guide shows the steps for doing this in Ubuntu 16.04 and 14.04. The guide has been tested on a fresh install of Ubuntu 14.04.5, 16.04.3, 17.04 and 17.10 in October 2017. If you have any suggestions or additions to the guide, please post it on the issue tracker https://gitlab.com/caro-sdu/RobWork/issues .

RobWork is basically multiple projects:
- RobWork : is the core part including math, kinematics, planning and so on.
- RobWorkStudio : is the GUI which enable visualization and more userfriendly interfaces through gui plugins
- RobWorkSim : is an extension to the RobWork core functionality which adds dynamic simulation of bodies, devices and several tactile sensors.
- RobWork Physics Engine (RWPE) : is an extension under RobWorkSim that requires access to code that is not yet public. Please request more information about this if you need it.
- RobWorkHardware : is mostly drivers (with RobWork datatypes) for common hardware, or hardware on which RobWork platforms have been built eg. SDH, cameras, CAN-devices, the Universal robot arm, serial port...

Note that RobWork is needed to run RobWorkStudio, RobWorkSim and RobWorkHardware. Therefore it is not possible to use these, without having RobWork installed on the machine. 

# Installing dependencies # {#sec_ubuntu_dependencies}

RobWork depends on third-party software that must be installed prior to compilation. This includes both build tools and third-party libraries.
In Linux it is quite easy to set up the dependencies as these are available as packages in the systems package manager.

## ¤ Build Tools ## {#sec_ubuntu_dependencies_tools}

To be able to checkout code it is necessary to install some source code management (SCM) tools, such as Subversion, Git and Mercurial.
To be able to checkout the code from our own SVN repository, a SVN client is needed.
Git and Mercurial clients are not strictly required, but depending on your needs it might be worthwhile to install them.

	sudo apt-get install subversion git mercurial

To compile the C++ code, the GCC compiler should be used on Ubuntu.
CMake must be used to prepare RobWork for compilation. The minimum CMake version for RobWork is currently 2.8.12, which is present in both Ubuntu 14.04 and 16.04.

	sudo apt-get install gcc g++ cmake

## ¤ RobWork Required Dependencies ## {#sec_ubuntu_dependencies_rw}
Start by installing the dependencies. This is done using the package manager by opening a terminal (Ctrl+Alt+T) and running the following commands.
First, install the boost libraries:
 
	sudo apt-get install libboost-dev libboost-date-time-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev libboost-serialization-dev libboost-system-dev libboost-test-dev libboost-thread-dev

## ¤ RobWork Optional Dependencies ## {#sec_ubuntu_dependencies_rw_optional}

Xerces can be used some places in RobWork for opening XML files. It is no longer a strict requirement, as RobWork is now able to use a Boost parser instead.
If you enable compilation of non-standard parts of RobWork, or need to compile old RobWork-dependent projects, it might be a good idea to compile Xerces:

	sudo apt-get install libxerces-c3.1 libxerces-c-dev

SWIG (optional) is a tool that makes it possible to generate a LUA script interface for RobWork.
Python and Java interfaces are also possible, but require that Python or Java SDK is installed as well.
All of these interfaces can be generated if you install the following packages (in Ubuntu 16.04 and newer you can consider using liblua5.3-dev instead):

	sudo apt-get install swig liblua5.2-dev python-dev default-jdk

Google Test (optional) is used for unit tests in RobWork. If you are a developer and wants to develop code for the RobWork trunk, writing a GTest will be a requirement:

	sudo apt-get install libgtest-dev

## ¤ RobWorkStudio Dependencies ##  {#sec_ubuntu_dependencies_rws}

RobWorkStudio requires Qt to be installed. Both Qt4 and Qt5 is supported, but on a fresh Qt install it is encouraged to choose the Qt5 version:

	sudo apt-get install qtbase5-dev

## ¤ RobWorkSim Dependencies ##  {#sec_ubuntu_dependencies_rwsim}

If you need to do dynamic simulations, you will probably need the RobWorkSim package. If you are in doubt and just need RobWorkStudio, you can likely skip this.

Open Dynamics Engine (ODE) can be installed through the package manager:

	sudo apt-get install libode-dev libodeX

Replace libodeX with the newest version available. Ubuntu 14.04 comes with ODE 0.11.1 (libode1), Ubuntu 16.04 with ODE 0.13.1 (libode4) and Ubuntu versions newer than 16.10 with ODE 0.14 (libode6).

Notice that the version from the package manager can sometimes be a bit outdated. If you want the latest version, Open Dynamics Engine (ODE) must be compiled from source.
Use Mercurial to download the source from bitbucket (use a dot '.' as the third argument to checkout directly to the current folder):

	hg clone https://bitbucket.org/odedevs/ode

Make sure that the required build tools are installed:

	sudo apt-get install automake libtool

Open a terminal, go to the ode folder (with the bootstrap file in it) and run:

	./bootstrap
	./configure --enable-double-precision --enable-shared --enable-ou --enable-builtin-threading-impl --disable-demos --disable-asserts
	make -j4

This will make sure that ODE is built with 4 threads with double precision as a shared library.

Bullet Physics can also be installed through the package manager. Ubuntu 14.04 comes with Bullet 2.81, Ubuntu 16.04 with 2.83.6, Ubuntu 17.04 with 2.83.7 and Ubuntu 17.10 with 2.86.1. The bullet packages can be installed with the following command:

	sudo apt-get install libbullet-dev libbullet-extras-dev

It is also possible to compile Bullet Physics from source, if a specific version is needed. Clone the source code with git (use a dot '.' as the third argument to checkout directly to the current folder):

	git clone https://github.com/bulletphysics/bullet3

Make a Build folder under the bullet3 folder and run CMake to configure the build. From within the Build folder, run in a terminal:

	cmake -DCMAKE_BUILD_TYPE=Release -DUSE_DOUBLE_PRECISION=ON -DBUILD_BULLET3=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX:PATH=$WORKSPACE/Release -DCMAKE_CXX_FLAGS="-fPIC" -DCMAKE_C_FLAGS="-fPIC" -DBUILD_EXTRAS=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_UNIT_TESTS=OFF -BUILD_CPU_DEMOS=OFF ..
	make -j4

Modify the options to suit your needs. The shown options will make sure that Bullet is built with double precision, required compile flags and switch off building of things that are normally unnecessary when used in RobWorkSim.

## ¤ RobWorkHardware Dependencies ##  {#sec_ubuntu_dependencies_rwhw}

RobWorkHardware compilation depends heavily on which hardware you need to use. Install the following package: 

	sudo apt-get install libdc1394-22-dev

It is not currently possible to give any general instructions for RobWorkHardware.

# Building RobWork # {#sec_ubuntu_build}

When the dependencies have been installed, RobWork is ready to be built. First, the source must be downloaded, followed by the build procedure.

## Getting source files from SVN ## {#sec_ubuntu_build_svn}
Make a new directory where you want to install RobWork (in this guide, we will install in ~/RobWork): 

	mkdir RobWork
	cd RobWork


When the dependencies are installed, go ahead and download the newest version of RobWork from the SVN repository at: 

https://svnsrv.sdu.dk/svn/RobWork/trunk/

Using: 

- Username: 'Guest'
- Password: ''

In the terminal, this is done as follows: (be sure that you are located in the directory where you want to install RobWork)

	svn co --username Guest --password '' https://svnsrv.sdu.dk/svn/RobWork/trunk/ .

## Setup CMake Options & Environment ## {#sec_rw_install_ubuntu_environment}

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

## Compiling RobWork ## {#sec_ubuntu_build_compile}

In the following it is assumed that RobWork was checked out to the folder ~/RobWork, and that this is the current directory.
Add a build directory for each of the projects you want to build:

	mkdir Build
	mkdir Build/RW
	mkdir Build/RWStudio
	mkdir Build/RWSim
	mkdir Build/RWHardware

Now we are ready to build RobWork. Run CMake in the newly created build directory for RobWork, and run make afterwards to build the project:

	cd ~/RobWork/Build/RW
	cmake -DCMAKE_BUILD_TYPE=Release ../../RobWork
	make -j4

Look carefully through the CMake output before running the make command. Check that there is no errors, and that the required dependencies are correctly found. 
The -j4 argument to make will build RobWork on 4 CPU cores. Note that you need at least 1 GB of memory per thread when building. Ie. building with 4 cores requires around 4 GB of RAM. 

For RobWorkStudio:

	cd ~/RobWork/Build/RWStudio
	cmake -DCMAKE_BUILD_TYPE=Release ../../RobWorkStudio
	make -j4

For RobWorkSim:

	cd ~/RobWork/Build/RWSim
	cmake -DCMAKE_BUILD_TYPE=Release ../../RobWorkSim
	make -j4

For RobWorkHardware:

	cd ~/RobWork/Build/RWHardware
	cmake -DCMAKE_BUILD_TYPE=Release ../../RobWorkHardware
	make -j4

Finally, we need to add the following paths to ~/.bashrc:

~~~~{.sh}
#ROBWORK#
export RW_ROOT=~/RobWork/RobWork/
export RWS_ROOT=~/RobWork/RobWorkStudio/
export RWHW_ROOT=~/RobWork/RobWorkHardware/
export RWSIM_ROOT=~/RobWork/RobWorkSim/
~~~~

Remember to only add paths to the components you have actually installed. Ie. if you only installed RobWork and RobWorkStudio, the paths for RobWorkSim and RobWorkHardware should not be set.

By setting these environment variables, it will be possible for other projects to find the RobWork projects. 
