Installation on Windows {#page_rw_installation_windows}
============ 
[TOC]

# Introduction # {#sec_rw_install_win_intro}
This guide shows the steps for building the RobWork packages on a Windows platform.
The guide is written based on a setup with Windows 10 and Visual Studio 2017 and the guide is last revised in March 2018.
If you have any suggestions or additions to the guide, please post them on the issue tracker at https://gitlab.com/caro-sdu/RobWork/issues .

RobWork is basically multiple projects:
- RobWork : is the core part including math, kinematics, planning and so on.
- RobWorkStudio : is the GUI which enable visualization and more userfriendly interfaces through gui plugins
- RobWorkSim : is an extension to the RobWork core functionality which adds dynamic simulation of bodies, devices and several tactile sensors.
- RobWorkHardware : is mostly drivers (with RobWork datatypes) for common hardware, or hardware on which RobWork platforms have been built eg. SDH, cameras, CAN-devices, the Universal robot arm, serial port...

Note that RobWork is needed to run RobWorkStudio, RobWorkSim and RobWorkHardware. Therefore it is not possible to use these, without having RobWork installed on the machine. 

# Requirements # {#sec_win_requirements}
It is expected that you have a system with:

- Windows 7, 8 or 10
- 64 bits
- 20GB available disk space (should be enough for build tools, dependencies and all the RobWork packages).

# Installing dependencies # {#sec_win_dependencies}

RobWork depends on third-party software that must be installed prior to compilation. This includes both build tools and third-party libraries.
In Windows it can be a bit tedious to set up all the dependencies, but most packages are easily installed using standard installers.
Unfortunately, it is also necessary to compile some dependencies from scratch.

The following dependencies will be described bellow. Please read the important documentation and installation notes under each dependency before installing, as some of the installations might be a little tricky.

\ref sec_win_dependencies_tools
- Tortoise SVN
- Git
- TortoiseHg Mercurial
- Microsoft Visual Studio
- CMake

\ref sec_win_dependencies_rw
- Boost

\ref sec_win_dependencies_rw_optional
- Xerces
- SWIG
- Google Test

\ref sec_win_dependencies_rws
- Qt

\ref sec_win_dependencies_rwsim
- Open Dynamics Engine (ODE)
- Bullet Physics

## ¤ Build Tools ## {#sec_win_dependencies_tools}

To be able to checkout code it is necessary to install some source code management (SCM) tools, such as Subversion, Git and Mercurial.
To be able to checkout the code from our own SVN repository, a SVN client is needed. The latest version of the Tortoise SVN client is recommended for this.
Tortoise SVN comes with a GUI that is easy to use and is nicely integrated with Explorer.
Git and Mercurial clients are not strictly required, but depending on your needs it might be worthwhile to install them.

The three SCM tools can be installed in only 20 minutes, and can be expected to use around 360 MB in total.

**Tortoise SVN client:**

Download from https://tortoisesvn.net

Installation of the Tortoise SVN client is straightforward. Expect to use 45 MB on the installation. During installation we recommend that you also install the "command line client tools" as shown below:

\image html installation/TortoiseSVN_addCLI.png "The TortoiseSVN installation options with CLI tools selected."

**Git client:**

Download from https://git-scm.com

Installation of Git requires a lot of choices during installation. We recommend to simply go with the preselected standard options, except you should add Git to your PATH environment. This makes it possible to use Git from a normal Windows Command Prompt: 

\image html installation/Git_PATH.png "Git installation where Git CLI can be used from ordinary Windows Command Prompt."

Git takes up more space than the other SCM tools. Expect to use around 225 MB.

**TortoiseHg Mercurial client:** 

Download from https://www.mercurial-scm.org

Installation of the TortoiseHg Mercurial client is straightforward. Expect to use 90 MB on the installation.

\image html installation/TortoiseHg_install.png "The TortoiseSVN installation options."

**SCM Tools in Windows Context Menu**

Once you have installed the three SCM tools, you will notice that the tools have been integrated in the Windows Explorer Context menu. A right-click on the desktop, or in a folder, will give the following options:

\image html installation/SCM_contextmenu.png "The Windows Explorer Context menu."

Typically, you will use "Git GUI Here" to checkout a Git project, "SVN Checkout" to checkout a SVN project, or "ToroiseHg-->Clone.." to checkout a Mercurial based project.
 
**Microsoft Visual Studio** should be used to compile RobWork (and dependencies) on Windows platforms. RobWork is expected to compile on Windows 7 or newer, using Visual Studio 2013 or newer. Currently, RobWork is continuously tested on Windows 7 using Visual Studio 2015. Notice that RobWork is now written using C++11 code, that is not expected to compile in Visual Studio 2012 or earlier versions.

Expect to use 45 minutes or more to install Visual Studio. It will use around 7 GB.

If you are a student at University of Southern Denmark, please see the following page for information about access to Microsoft products:

http://www.sdu.dk/en/information_til/studerende_ved_sdu/campusguide/it/software (under Microsoft see the sections about Dreamspark Standard or Premium).

It is possible to use both Community, Professional and Enterprise editions of Visual Studio.
The installation procedure is self-explanatory. You should select the option for C++ desktop development (here shown for the Enterprise edition):

\image html installation/VS17_installC++.png "A Visual Studio 2017 Enterprise installation. Be careful to select the \"Desktop devlopment with C++\"."

After the installation, you will see some new entries in the Windows start menu. Especially the the "x64 Native Tools Command Prompt" and "Visual Studio 2017" is important.

\image html installation/VS17_startmenu.png "The start menu entries after installation of Visual Studio."

The command prompt is used to run CMake for the projects that we will compile in the following sections. The command prompt sets up a development environment, such that CMake can detect the correct compiler. The Visual Studio IDE will require you to login when you start it. On the University of Southern Denmark you should be able to use your usual university login.
Once logged in, you should be able to open the Visual Studio IDE: 
\image html installation/VS17_IDE.png "The Visual Studio 2017 IDE."

Visual Studio uses a somewhat confusing versioning scheme between the Visual Studio IDE and the corresponding compiler versions.
The following table gives an overview of the version numbers for future reference:

| Visual Studio Name | Visual Studio Version | Visual C++ Compiler Toolset | Visual C/C++ Compiler Version |
|--------------------|-----------------------|-----------------------------|-------------------------------|
| Visual Studio 2017 | 15.6                  | 14.13                       | 19.13                         |
| Visual Studio 2017 | 15.5                  | 14.12                       | 19.12                         |
| Visual Studio 2017 | 15.3 & 15.4           | 14.11                       | 19.11                         |
| Visual Studio 2017 | 15.1 & 15.2           | 14.10                       | 19.10                         |
| Visual Studio 2015 | 14.0                  | 14.00                       | 19.00                         |
| Visual Studio 2013 | 12.0                  | 12.00                       | 18.00                         |

Notice that a given version of the Visual Studio IDE can in principle be used to compile with different toolset/compiler versions.
One can think of the toolset as a set of tools: the compiler, linker, C/C++ runtime libraries used etc.
The Visual C++ sompiler (often abbreviated as MSVC) is just one of the tools in the toolset.

**CMake** must be used to generate projects for Visual Studio. A Windows installer can be downloaded from the CMake homepage at https://cmake.org , and installation takes up 70 MB .
The minimum CMake version for RobWork on Windows is currently 3.1.
If you want to compile shared libraries, CMake version 3.4 or newer is needed.
Choosing the latest version is always recommended (except the release candidates).
Choosing older versions will mean that newer Visual Studio and Boost versions will not be supported.
If you already have an older version of CMake installed, please check that it is recent enough to support your setup:

| CMake Version | Maximum Visual Studio Version Supported | Maximum Boost Version Supported |
|---------------|-----------------------------------------|---------------------------------|
| 3.11.0        | Visual Studio 15 2017                   | 1.66.0                          |
| 3.10.2*       | Visual Studio 15 2017                   | 1.65.1                          |
| 3.9.3         | Visual Studio 15 2017                   | 1.65.1                          |
| 3.8.0         | Visual Studio 15 2017                   | 1.64.0                          |
| 3.7.2         | Visual Studio 15 2017                   | 1.63.0                          |
| 3.7.0-3.7.1   | Visual Studio 15 2017                   | 1.62.0                          |
| 3.5.0-3.6.3   | Visual Studio 14 2015                   | 1.61.0                          |
| 3.4.0-3.4.3   | Visual Studio 14 2015                   | 1.59.0                          |
| 3.1.1-3.3.2   | Visual Studio 14 2015                   | 1.58.0                          |
| 3.1.0         | Visual Studio 14 2015                   | 1.56.0                          |

* Newest at time of writing.

Installation is straightforward, and we recommend that you install CMake to the system PATH.
By installing to the system PATH, it will be possible to run CMake from a command line without specification of the entire path to the CMake executable:

\image html installation/CMake_PATH.png "CMake install, with addition to system-wide PATH environment."

## ¤ RobWork Required Dependencies ## {#sec_win_dependencies_rw}
**Boost** is the most important dependency in RobWork, and it is recommended to always use the latest possible version of Boost.
RobWork is also backwards compatible with older versions of Boost, mainly to support current Ubuntu LTS releases and CentOS 7.
On Windows, you should use at least Boost 1.55, as older releases is not expected to support the Visual Studio versions which are new enough to support C++11.
Boost precompiled libraries can be found at:

https://sourceforge.net/projects/boost/files/boost-binaries

Choose the newest Boost version that fits your CMake version, according to the table above.
Choose the newest precompiled library version, based on your Visual C++ toolset version below:

| Boost Version   | Maximum Visual C++ Toolset (Source) | Maximum Visual C++ Toolset (Precompiled) |
|-----------------|-------------------------------------|------------------------------------------|
| 1.66.0          | 14.11 (VS 15.4)                     | 14.1                                     |
| 1.65.1          | 14.11 (VS 15.3)                     | 14.1                                     |
| 1.64.0 - 1.65.0 | 14.10                               | 14.1                                     |
| 1.63.0          | 14.10                               | 14.0                                     |
| 1.59.0 - 1.62.0 | 14.00                               | 14.0                                     |
| 1.57.0 - 1.58.0 | 14.00                               | 12.0                                     |
| 1.55.0 - 1.56.0 | 12.00                               | 12.0                                     |

In this table, the "Maximum Visual C++ Toolset (Source)" version is the maximum supported version in the Boost source.
The newest Visual Studio versions will not be recognized as safe/tested versions by Boost, which means that Boost will issue a lot of warnings while compiling RobWork.
Usually, these warnings can simply be ignored, and things will work fine anyway. As shown in the table, the precompiled libraries for new Visual Studio versions, is built a while after they are introduced
in the code. If you want to use Visual Studio 2017, the table shows that you must choose Boost 1.64 if you want to use precompiled libraries (or 1.63 if you compile Boost yourself).

The file to download has a name with a format similar to "boost_1_66_0-msvc-14.1-64.exe".
Here 1_66_0 refers to Boost version 1.66.0, msvc-14.1 refers to the Visual C++ toolset version 14.1 (Visual Studio 2017), and 64 means the 64 bit version of Boost.

The Boost installer is straightforward, and we suggest to stick with the default choices during installation. After installation you should have a Boost installation with the following directory layout:

\image html installation/Boost_layout.png "The Boost precompiled installation layout."

Normally, Boost will have a lib folder. For the precompiled installation, this folder has been renamed to lib64-msvc-14.1. This makes it possible to install multiple configurations side by side, for the same Boost version. Note down the path to the Boost folder. Later we will refer to it as BOOST_ROOT. The path to the lib64-msvc-14.1 we will refer to as BOOST_LIBRARYDIR.

Boost installation can be done in 10 minutes and will take up roughly 3.35 GB disk space. 

To compile the Boost libraries from source, get the source and run something similar from a command prompt (only for expert users!):

	bootstrap.bat
	b2 -j4 --with-filesystem --with-system --with-program_options --with-regex --with-serialization --with-thread --with-date_time --with-chrono --with-test --prefix=.\ address-model=64 link=shared install

Here -j gives the number of threads to use for compilation. Run with -help, -help-options or \-\-show-libraries to get more information about the various options.

## ¤ RobWork Optional Dependencies ## {#sec_win_dependencies_rw_optional}

**Xerces** (optional) can be used some places in RobWork for opening XML files. It is no longer a strict requirement, as RobWork is now able to use a Boost parser instead.
If you enable compilation of non-standard parts of RobWork, or need to compile old RobWork-dependent projects, it might be a good idea to compile Xerces.

Go to http://xerces.apache.org (older versions can be found here: http://archive.apache.org/dist/xerces/c/3/sources) and download and unpack the source distribution.

Xerces 3.2 and newer are CMake based, and you can use the new procedure to compile it:

First, go to the unpacked Xerces folder and create two folders inside it, called build and xerces-install:
\image html installation/Xerces_createbuildfolder.png "The Xerces source. Create empty folder build and xerces-install manually."

Open a Visual Studio "x64 Native Tools Command Prompt", and go to the newly created build folder. Now run the following command:

	cmake -G "Visual Studio 15 2017 Win64" -DCMAKE_INSTALL_PREFIX:PATH="C:/some/path/to/xerces-install"

If CMake succeeds, go to the build folder, and open xerces-c.sln. Then chosse Release mode and 64 bit build as follows:

\image html installation/xerces_install_1.png "Choose the 'Release' configuration (alternatively choose 'Static Release' if you prefer static libraries)."
\image html installation/xerces_install_2.png "Choose 64 bit build."

Build the XercesLib target:

\image html installation/xerces_install_3.png "Right click XercesLib in the SolutionExplorer and click 'Build'."

Finally run build for the INSTALL target. This will populate the xerces-install folder with a bin, cmake, include, lib and share folder.
Note down the path to the xerces-install folder. We will use the name XERCESC_ROOT to refer to that directory path later when setting up the RobWork project.

Xerces will take up around 250 MB in total, and will take around 20 minutes to download and compile.

Old installation procedure (Xerces 3.1.4 and earlier):
 -# Go to http://xerces.apache.org (older versions can be found here: http://archive.apache.org/dist/xerces/c/3/sources) and download the source distribution.
 -# Unpack it where you want Xerces installed.
 -# Open xerces-c-3.1.4/projects/Win32/VCxx/xerces-all.sln in Visual Studio (substitute VCxx with your Visual Studio version - see https://en.wikipedia.org/wiki/Microsoft_Visual_Studio#History for overview).
 -# Choose 64-bit Release build configuration, and build the XercesLib target.

**SWIG** (optional) is a tool that makes it possible to generate a LUA script interface for RobWork.
Python and Java interfaces are also possible, but require that Python or Java SDK is installed as well. The SWIG tool is easily downloaded from:

https://sourceforge.net/projects/swig/files/swigwin

The tool needs no compilation. Simply extract the files from the zip-file where you want SWIG installed. Note down the path to the swig.exe executable. We will refer to this path later as SWIG_EXECUTABLE.

SWIG uses only 35 MB.

**Google Test** (optional) is used for unit tests in RobWork. If you are a developer and wants to develop code for the RobWork trunk, writing a GTest will be a requirement.

Go to the folder where you want to put the Google Test source. Right-click and click "Git GUI Here". Now insert https://github.com/google/googletest.git as the source location, and choose the target directory. The target directory must be an empty or non-existing directory. Finally, press clone to clone the Git repository.

\image html installation/GTest_clone.png "Cloning Google Test source with the Git GUI."

After cloning, you should see the following directory layout:

\image html installation/GTest_layout.png "Google Test directory layout."

Note down the path to the googletest folder. We will refer to this as GTEST_ROOT and GTEST_SOURCE later on.

The Google Test code should not be compiled. It will be compiled as a part of the RobWork compilation when the source code is present. The Google test repository uses up to 95 MB.

## ¤ RobWorkStudio Dependencies ## {#sec_win_dependencies_rws}
RobWorkStudio requires **Qt** to be installed. Both Qt4 and Qt5 is supported, but on a fresh Qt install it is encouraged to choose the latest Qt5 version (for now, skip 5.8).
Download and install Qt from:

https://www.qt.io

You need to choose the Open Source version. Notice that Qt is only free for open source projects. Also, you need to register to download Qt.

<b>WARNING! Please avoid Qt 5.8</b>  ( see issue https://gitlab.com/caro-sdu/RobWork/issues/37 )

Run the Online installer for Windows, and select the components you want. Simply select your Visual Studio version under the version of Qt you want to use.

\image html installation/Qt5_components.png "Choice of Qt components. It is enough to make a single selection with your Visual Studio version."

Qt installer might launch QtCreator at the end. Just close this program, as we intend to use Visual Studio instead.
Qt5 will use aroung 3.65 GB disk space.

After installation you should have a folder with the following layout:

\image html installation/Qt5_layout.png "Qt5 directory layout."

Note down the path to the Qt folder shown above, we will need that when setting up the RobWorkStudio project.

## ¤ RobWorkSim Dependencies ## {#sec_win_dependencies_rwsim}

If you need to do dynamic simulations, you will probably need the RobWorkSim package. If you are in doubt and just need RobWorkStudio, you can likely skip this section.

**Open Dynamics Engine (ODE)** must be compiled from source. Use **TortoiseHg (Mercurial)** to download the source from bitbucket: https://bitbucket.org/odedevs/ode

\image html installation/ODE_clone.png "Clone ODE with Mercurial client."

CMake is used by ODE 0.15.2 and newer. It takes 10 minutes to setup and compile, and takes up around 85 MB. This is the recommended procedure:

	mkdir rwode_build
	cd rwode_build
	cmake -G "Visual Studio 15 2017 Win64" -DBUILD_SHARED_LIBS=ON -DODE_DOUBLE_PRECISION=ON -DODE_WITH_OU=ON -DODE_WITH_TESTS=OFF -DODE_WITH_DEMOS=OFF -DCMAKE_INSTALL_PREFIX:PATH="C:\some\path\to\ode\install" ..

The directory layout will be as follows (we will later refer to the install folder as ODE_DIR):

\image html installation/ODE_layout.png "ODE directory layout."

Old procedure (0.15.1 and earlier):

Open a terminal and go to the build folder to run premake4:

	premake4.exe --only-double --only-shared --with-ou --with-builtin-threading-impl --os=windows --platform=x64 vs2010

This will make sure that ODE is built with double precision as a 64-bit shared library.
The \-\-with-builtin-threading-impl does not exist from version 0.15, as it is now default.
Unfortunately, Visual Studio 2010 is the latest supported version by the premake4 program.
When the ode.sln is opened, Visual Studio will upgrade to a newer format. Select 64-bit Release configuration and build the solution.

**Bullet Physics** must be compiled from source. Clone the source code with git from the source: https://github.com/bulletphysics/bullet3

Bullet takes up around 440 MB, and takes around 15 minutes to compile.

Make a Build folder and run CMake to generate a Visual Studio solution. From within the Build folder, run in a terminal:

	cmake .. -G "Visual Studio 15 2017 Win64" -DUSE_DOUBLE_PRECISION=ON -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON -DBUILD_EXTRAS=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_CPU_DEMOS=OFF -DBUILD_OPENGL3_DEMOS=OFF -DBUILD_BULLET2_DEMOS=OFF -DINSTALL_LIBS=ON -DCMAKE_INSTALL_PREFIX:PATH="C:\some\path\to\bullet3\install"

Choose the generator that fits your Visual Studio version with the -G option, and remember to replace "C:\some\path\to\bullet3\install" with the full path to the directory to install to.
Modify the options to suit your needs.
The shown options will make sure that Bullet is built with double precision, shared runtime and switch off building of things that are normally unnecessary when used in RobWorkSim.
To build Bullet, open BULLET_PHYSICS.sln, choose the Release configuration and build the solutions. To install, build the INSTALL target.

The directory layout is shown below. Note down the path to the install folder, which we will refer to as BULLET_ROOT later on.

\image html installation/Bullet_layout.png "Bullet directory layout."

**RobWork Physics Engine**

A third engine exists, but requires access to code that has not yet been released to the public. Request more information about this if you need it.

## ¤ RobWorkHardware Dependencies ##  {#sec_win_dependencies_rwhw}

RobWorkHardware compilation depends heavily on which hardware you need to use. It is not currently possible to give any general instructions for RobWorkHardware.

# Building RobWork # {#sec_win_build}

When the dependencies have been installed, RobWork is ready to be built. First, the source must be downloaded, followed by the build procedure.

Expect to use a total of 3 GB for the RobWork projects.

## Getting RobWork source files from SVN ## {#sec_win_build_svn}
Make a new directory where you want to install RobWork. When the dependencies are installed, go ahead and download the newest version of RobWork from the SVN repository at: 

https://svnsrv.sdu.dk/svn/RobWork/trunk

Right-click where you want to check out, choose "SVN Checkout.." and insert the resporitory URL as shown below. Also set the checkout directory (should be empty!), and press OK.

\image html installation/RW_checkout.png "The SVN checkout dialog for RobWork."

You will be asked for username and password: 

- Username: 'Guest'
- Password: ''

Alternatively, this can be done from the terminal: (be sure that you are located in the directory where you want to install RobWork)

	svn co --username Guest --password '' https://svnsrv.sdu.dk/svn/RobWork/trunk/ .

There should now be RobWork, RobWorkStudio, RobWorkSim and RobWorkHardware folders. It is also possible to check out the projecs individually:

	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWork
	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWorkStudio
	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWorkSim
	https://svnsrv.sdu.dk/svn/RobWork/trunk/RobWorkHardware

## Compiling RobWork ## {#sec_win_build_compile}

The challenging part when compiling RobWork on a Windows platform, is to get CMake to find the dependencies. One thing is to install and compile all the needed
dependencies, another is to make sure that RobWork actually finds these dependencies. A good advice before building RobWork, is to actually read the CMake output carefully.
The CMake output will typically reveal early in the process if a dependency was not found.
Building RobWork can take quite some time, and it is a pitty building everything, just to discover that some functionality was disabled due to a unmet dependency
(especially a problem for the optional dependencies).

To build RobWork, open a Visual Studio 64-bit command prompt and go to the directory where RobWork was checked out from SVN.
Add new build directories for the RobWork packages you want to compile, such as:

	mkdir Build
	mkdir Build\RW
	mkdir Build\RWS

Now we are ready to build RobWork. You need to choose the generator that matches your Visual Studio version, and adjust all the paths given to CMake:

	cd Build\RW
	cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 15 2017 Win64" ^
	      -DBOOST_ROOT="C:\Boost\boost_1_65_1" ^
	      -DBOOST_LIBRARYDIR="C:\Boost\boost_1_65_1\lib64-msvc-14.1" ^
	      -DGTEST_ROOT:PATH="C:\some\path\to\GTest\googletest" ^
	      -DGTEST_SOURCE:PATH="C:\some\path\to\GTest\googletest" ^
	      -DXERCESC_ROOT:PATH="C:\some\path\to\xerces-c-3.2.1\xerces-install" ^
	      -DSWIG_EXECUTABLE="C:\some\path\to\swigwin-3.0.12\swig.exe" ^
	      -DBULLET_ROOT:PATH="C:\some\path\to\bullet3\install" ^
	      ../../RobWork

The paths must be adjusted to your own compilation and installation of the dependencies. The images illustrating the directory layout for the relevant dependencies, will give a clue on which paths to pass to CMake. Notice that in practice you will not necessarily need to specify all of these paths. It depends on which dependencies you want to build with (except the mandatory Boost dependency of course).

Executing the CMake command will look like the following:

\image html installation/RW_cmakecmd.png "Running CMake for RobWork in a Visual Studio Prompt."

Look carefully through the CMake output and check that there is no errors, and that the required dependencies are correctly found.
Now that the CMake files has been built, we are ready to compile the project. In the Build\RW folder there will now be a RobWork.sln solution that can be opened in Visual Studio.
Choose the correct configuration (Release for instance) and build the solution.

If errors are encountered, try to decode them and adjust the paths if that is what is needed. CMake caches the result for the following runs of CMake. It is often a good idea to delete the CMakeCache.txt file to force CMake to run from scratch. The benefit of the cache is that you can run the CMake without specifying all the paths, as long as it has been run once before with the paths given. The disadvantage is that it might cause things to behave odd if you later change essential options CMake. If you update RobWork to a newer revision from the SVN server, it is also possible that options can change (even though it is rare). If you encounter problems after such an update, always try to remove the CMakeCache.txt, rerun CMake and try to compile again.

If you go to the RobWork folder, you will se the following directory layout:

\image html installation/RW_layout.png "Directory layout of RobWork. Layout is similar for RobWorkStudio, RobWorkSim and RobWorkHardware. Notice the bin and libs folders have been created and populated after the build in Visual Studio."

The bin folder will hold the compiled executables (for RobWork this will be unit test executables) and DLL's for shared libraries.

The libs folder will hold the .lib files used to link to shared libraries, static libraries, and DLL's for plugins. 

Each of these folders are divided into four subfolders. Here you have to choose the subfolder that matches your build configuration (such as bin/release or libs/release).

**RobWorkStudio**

Wait with compilation of RobWorkStudio until you have RobWork successfully compiled. Then the same procedure is repeated in the RWS build folder. In this case we need to specify the Qt directory (shown previously in the Qt layout image). This directory is added to the CMAKE_PREFIX_PATH variable:

	cd Build\RWS
	cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 15 2017 Win64" -DCMAKE_PREFIX_PATH="C:\Qt\5.10.1\msvc2017_64" ../../RobWorkStudio

**RobWorkSim**

Finally, RobWorkSim can be compiled (when both RobWork and RobWorkStudio has been successfully compiled). Once again, the almost same procedure is repeated. This time we must specify the Qt, Bullet and ODE directories:

	cd Build\RWS
	cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 15 2017 Win64" -DCMAKE_PREFIX_PATH="C:\Qt\5.10.1\msvc2017_64" ^
	      -DBULLET_ROOT:PATH="C:\some\path\to\bullet3\install" ^
	      -DODE_DIR:PATH="C:\some\path\to\ode\install" ^
	      ../../RobWorkSim

**How to run a program after compilation**

After you have compiled the different RobWork projects, you will want to launch a program, such as RobWorkStudio.exe which lies in RobWorkStudio/bin/release (or some other configuration than release).
Often you will encounter error messages about  DLL's that can not be found. In this case you need to set your PATH environment.
The folder that holds the given dll must be added to the PATH, and there are different ways to achieve this.

First, you can launch the program from a command prompt. Just before you launch the program, you can set the PATH with:

	SET PATH=C:\path\to\some\dir;C:\path\to\some\other\dir;%PATH%
	ECHO %PATH%

Which will tell Windows where to search for the runtime DLL's. The path will typically need to include directories for DLL's for Xerces, Qt, Boost, ODE, Bullet and other dependencies which might be dynamically linked. Some of these might also be statically linked, in which case it is not necessary to specify a path at runtime.

Another alternative is to set the PATH in Windows for either the entire system (all users) or your user. Go to the start menu and search for "environment", to launch the necessary dialogs.
To get directly to the dialogs, you can also run

	SystemPropertiesAdvanced.exe

for the system-wide settings (requires administration rights) or

	rundll32 sysdm.cpl,EditEnvironmentVariables

to edit the PATH for your own account only.

So which method is best?

For external depdendencies like Qt, Boost, Xerces and similar it will probably make sense to add the DLL's to the PATH environment variable system-wide or for your user account. But only if you only have one installation of the given dependencies. If you have multiple different versions or configurations in your system for a single dependency, it is recommended to not have these in the system/user PATH variable. This is because it can cause some confusion, and errors that are hard to debug. It becomes difficult to really understand what version you are using when it is in the system/user path. In this case the first method is better. Here you explicitly state what DLL's you want to use each time before you run the program. It is however a bit more difficult as it takes more work each time you want to launch the program. Here it is often useful to create a .bat script that set up the path and launches the program.

## CMake Options & Environment ## {#sec_rw_install_win_environment}
Above we used different CMake definitions to specify the paths to dependencies, choose the build configuration and the type of project to generate (Visual Studio projects).
Some of the paths might also be set up as environment variables in the system. This might be useful in some cases, and will make it easier to run CMake without specifying all the paths. Again, using environment variables can also make the CMake process a lot less transparent, and it becomes difficult to understand where the dependencies are found and why.

The RobWork CMake system has been around since ancient versions of CMake. It is possible to adjust a huge number of variables to adjust which parts of RobWork is built, which dependencies are used, and if libraries should be static or shared. This is also complex because of the large number of (optional) dependencies, of which we only show some in this guide. We are constantly trying to modernize the CMake system to utilize more recent features, and we hope that this will make it easier to understand and use the build system. Recently, multiple of our dependencies switched to CMake based build systems, which makes the overall build procedure a lot easier, as users are able to repeat the same procedure using only CMake as build system.

In the future, we plan to use the [CMake Options & Environment](@ref page_rw_installation_cmake_options) page to give an overview of how to customize RobWork through CMake.