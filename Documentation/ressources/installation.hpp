// -*- latex -*-

/**

\page page_rw_installation Installation
- \ref sec_rw_install_intro
- \ref sec_rw_install_SDK
- \ref sec_rw_install_compile
 - \ref sec_rw_install_download
 - \ref sec_rw_install_common
- \ref sec_rw_dependencies
- \ref sec_rw_install_common
 - \ref sec_rw_dependencies
- \ref sec_rw_install_linux
- \ref sec_rw_install_windows
 - \ref subsec_rw_install_windows_mingw
- \ref subsec_rw_install_windows_visual


\section sec_rw_install_intro Introduction
RobWork uses several external libraries such as
Boost, Xerces and for RobWorkStudio also Qt. Furthermore, RobWork is developed
for multiple platforms (Linux, Win7/xp MinGW, Mac, and Win7/xp Visual Studio) which
sometimes complicates the installation.

Here you find articles that describe how to install RobWork or compile RobWork
and the dependencies on several different platforms.


\section sec_rw_install_SDK Installing RobWork SDK
 The quickest installation obtained through the provided SDK packages for Ubuntu (Debian packages)
 and Windows (NSIS installers). The Debian packages automatically install all dependencies and
 the NSIS installers contain headers and precompiled libraries for all dependencies.
 These SDKs are suitable for users who want to get started quickly with the development of
 RobWork and RobWorkStudio plugins.

Please look at the <a href="http://www.robwork.dk/jrobwork/index.php?option=com_jdownloads&view=viewcategory&catid=8&Itemid=83">download</a>
section on the homepage for downloading the SDK packages.


\section sec_rw_install_compile Installing RobWork from source
This section will explain how to install RobWork from source. Several platforms and compilers
are supported and one should look for the description including he's platform.

\subsection sec_rw_install_download Downloading the source
RobWork source is made available through half year releases and SVN. The source releases can be downloaded
from the RobWork homepage.

You can get the latest developer snapshot of the RobWork source code from SVN using this account:

Username: RO-RobWork

Password: anonymous

SVN repository: https://svn.mip.sdu.dk/RobWork/trunk

Since the trunk is actively developed there might be bugs or compile issues in the source. Therefore
check out the nightly builds on our cdash servers, these should indicate any problems there might be on
the different supported platforms.

The trunk is compiled nightly with several different compilers on both 32 and 64 bit platforms (Ubuntu Linux, Windows XP and Windows 7).
The status of these builds is submitted to the following dashboards which enable users to keep track of broken builds:

http://www.robwork.dk/cdash/index.php?project=RobWork

http://www.robwork.dk/cdash/index.php?project=RobWorkStudio

\subsection sec_rw_install_common Installation instructions common for all platforms

Download and install the \b RobWork and \b RobWorkStudio packages and
place them both in single directory of your choice. Uncompress the
packages.

Building of \b RobWork and \b RobWorkStudio is supported on multiple
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

\code
cmake -G <generator name>
\endcode

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

\section sec_rw_dependencies Dependencies of RobWork




\section sec_rw_install_linux Installation on Linux

The following programs and libraries must be installed:

- gcc (version 4.1 or above)
- cmake
- blas-atlas
- lapack-atlas
- qt
.

The <a href="http://trolltech.com">Qt</a> library is needed only for
\b RobWorkStudio.

Uncompress the packages:

\code
tar xjf robwork.tar.bz2
tar xjf robworkstudio.tar.bz2
\endcode

Build the RobWork collection of libraries:

\code
cd RobWork
cmake .
make
\endcode

Build the RobWorkStudio program:

\code
cd RobWorkStudio
cmake .
make
\endcode

The make targets can be run also from <a
href="http://www.eclipse.org">Eclipse</a>.

\section sec_rw_install_windows Installation on Windows

RobWork can be build on the Windows operating system using either <a
href="http://www.mingw.org">MinGW</a> or <a
href="http://msdn.microsoft.com/vstudio">Microsoft Visual Studio</a>.
For either compilation system you must download and install <a
href="http://www.cmake.org">CMake</a> and <a
href="http://trolltech.com">QT4</a>. The Qt library is needed only for
\b RobWorkStudio.

Add the \c bin directories of CMake and Qt to the Windows \c Path
environment variable (see Control Panel : System : Advanced :
Environment Variables). You can check that CMake and Qt can both be
found by running the following two commands from a \c cmd window:

\code
cmake --version
moc --version
\endcode

\subsection subsec_rw_install_windows_mingw Using MinGW

Your installation of <a href="http://www.mingw.org">MinGW</a> must
include the following packages or libraries:

- gcc-core
- gcc-g77
- gcc-g++
- binutils
- w32api
- mingw-runtime

Build the RobWork collection of libraries:

\code
cmake -G "MinGW Makefiles"
mingw32-make
\endcode

Run the same commands in the \c RobWorkStudio directory to build the
\b RobWorkStudio program:

\code
cmake -G "MinGW Makefiles"
mingw32-make
\endcode

\subsection subsec_rw_install_windows_visual Using Visual Studio

Precompiled Lapack and Blas libraries are included in the RobWork
package in the directory RobWork\\ext\\libs_vs. You must either add
this directory to the Windows \c Path environment variable or copy the
DLL files of this directory to a directory already in the system path.

To see a list of all compilers supported by CMake, type \c cmake in a
\c cmd window. For this example, we assume the compiler used is Visual
Studio 8 2005.

In the \c RobWork directory, run the command:
\code
cmake -G "Visual Studio 8 2005"
\endcode

This CMake command generates a Visual Studio solution file named \c
RobWork.sln. Open the solution file with Visual Studio and build the
solution. Repeat the procedure for the \c RobWorkStudio directory.




*/
