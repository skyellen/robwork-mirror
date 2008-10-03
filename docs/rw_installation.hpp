// -*- latex -*-

/**

\page page_rw_installation RobWork and RobWorkStudio installation

- \ref sec_rw_install_common
- \ref sec_rw_install_linux
- \ref sec_rw_install_windows

\section sec_rw_install_common Installation instructions common for all platforms

Download and install the \b RobWork and \b RobWorkStudio packages and
place them both in single directory of your choice. Uncompress the
packages.

The building of \b RobWork and \b RobWorkStudio is supported on
multiple platforms thanks to the <a
href="http://www.cmake.org">CMake</a> build system.

To customize the build process add and edit the following two files:

- RobWork/RobWork.cmake
- RobWorkStudio/RobWorkStudio.cmake

Templates with suggested contents for the above two files are included
in the download:

- RobWork/RobWork.cmake.template
- RobWorkStudio/RobWorkStudio.cmake.template

To construct a build setup for the compiler of your choice, you must
run CMake from the root of the \c RobWork and \c RobWorkStudio
directories. The CMake command will be of the form

\code
cmake -G <generator name>
\endcode

where \c <generator \c name> is the name of the compiler system for
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
generated program \c TestSuite from within the \c RobWork/test
directory.

The \b RobWorkStudio program looks for files named \c
RobWorkStudio.ini and \c CustomRobWorkStudio.ini in the current
working directory. A template file \c
RobWorkStudio/bin/RobWorkStudio.ini_template shows the typical
contents for the \c RobWorkStudio.ini file. Start the \b RobWorkStudio
program and open a workcell file such as \c RobWork/docs/workcell.wu.

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

The make targets can be run also from Eclipse.

\section sec_rw_install_windows Installation on Windows

RobWork can be build on the Windows operating system using either <a
href="http://www.mingw.org">MinGW</a> or <a
href="http://msdn.microsoft.com/vstudio">Microsoft Visual Studio</a>.
For either compiler system you must download and install <a
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

To see the list of compilers supported by CMake type
\code
cmake
\endcode
in a \c cmd window. For this example, we assume the compiler
used is "Visual Studio 8 2005".

In the \c RobWork directory run the command:
\code
cmake -G "Visual Studio 8 2005"
\endcode

The CMake command generates a Visual Studio solution file named \c
RobWork.sln. Open the solution file with Visual Studio and build the
solution. Repeat the procedure for the \c RobWorkStudio directory.

*/

/*

----------------------------------------------------------------------
-- Killed text
----------------------------------------------------------------------

- Now open the Visual Studio solution and compile.
    - Notice: Newer versions of Visual Studio has many of the c-style
      functions (e.g. sscanf, fopen, etc.) as deprecated and gives
      warnings. Just ignore these.

----------------------------------------------------------------------

- Blas and lapack - libraries that typically exists on linux/unix
  platforms. We have supplied precompiled libraries for windows
  platforms in our download section.

- (Optional) Eclipse Europe - Eclipse Europe is a C++ programming
  framework. Eclipse project files are supplied with the RobWork
  source.

- Notice that If you choose to install mingw using the QT installer
  you will be missing the gcc-g77 package.

----------------------------------------------------------------------

- Check that the installers has made the correct setup of system variables
  - QTDIR set to your QT folder (e.g. c:\\local\\Qt\\4.3.1)
  - PATH contains you QT, MingQ and System32 folders (e.g.
  c:\\local\\Qt\\4.3.1; c:\\local\\MingW\\bin;
  c:\\Windows\\System32)
  - QMAKESPEC set to win32-g++

----------------------------------------------------------------------

The CMake build system can be used with different compilers. Currently
only the GCC compiler system of <a
href="http://www.mingw.org">MinGW</a> is fully supported, but if you
don't need RobWorkStudio then <a
href="http://msdn.microsoft.com/vstudio">Microsoft Visual Studio</a>
is usable also.

If MinGW if the only compiler on you system you can omit the -G "MinGW
Makefiles".

----------------------------------------------------------------------

By default the project is compiled without orocos. To use orocos WHAT
WAS IT WE NEEDED TO DO FOR THIS?

- Goto RobWork/ext/orocos/
- execute \code cmake .\endcode
- execute \code make\endcode and then \code make install \endcode
- liborocos-rtt.a will be installed to the rwroot/libs/ folder and all header files will be installed to rwroot/ext/orocos/include

Common pitfals:

- Problem - CMake does not find MinGW but instead locates another
  compiler (visual cl.exe or the likes).
    - Solution - Check if MinGW is in your system path. If you have
      more than one make system on your platform then CMake will
      automatically choose one. To force CMake to choose MinGW
      compiler use <cmake -G"MinGW Makefiles" .> instead of just
      <cmake .>

However, if you have and would like to explain how, please send us the
information.

----------------------------------------------------------------------
(Orocos has been removed.)

The Orocos system is included in the RobWork package. If you want to
use RobWork together with Orocos you can build the Orocos library as
follows:

\code
cd RobWork/ext/orocos
cmake .
make
make install
\endcode

Note that \c make \c install command will install Orocos locally.

----------------------------------------------------------------------
\subsection subsec_rw_install_linux_start_robworkstudio Starting RobWorkStudio

The RobWorkStudio program a \c RobWorkStudio.ini file from the current
working directory. To create the \c RobWorkStudio.ini file and start up
RobWorkStudio do as follows:

\code
cd RobWorkStudio/bin
cp RobWorkStudio.ini_template RobWorkStudio.ini
./RobWorkStudio
\endcode

\subsection subsec_rw_install_linux_dependencies Dependencies
\subsection subsec_rw_install_linux_unpack Downloading and uncompressing the software
\subsection subsec_rw_install_linux_compile Building RobWork and RobWorkStudio

The RobWorkStudio program is found in the \c RobWorkStudio\\bin
directory. Before running the program for the first time, you must
rename the \c RobWorkStudio.ini_template file found in the \c
RobWorkStudio\\bin directory to \c RobWorkStudio.ini.

*/
