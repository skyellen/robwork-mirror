// -*- latex -*-

/**

\page page_rw_installation RobWork and RobWorkStudio installation

- \ref sec_rw_install_linux
- \ref sec_rw_install_windows

\section sec_rw_install_linux Installation on Linux

\subsection subsec_rw_install_linux_dependencies Dependencies

The following programs and libraries should be available:

- gcc (version 4.1 or above)
- cmake
- blas-atlas
- lapack-atlas
- qt
.

The qt library is needed only you want to use RobWorkStudio.

The following libraries for cameras are optional:
- libdc1394
- libraw1394
.

\subsection subsec_rw_install_linux_unpack Downloading and uncompressing the software

<a href="http://www.robwork.dk">Download</a> the RobWork and
RobWorkStudio packages and place them in a directory of your choice.
The RobWork and RobWorkStudio packages can be uncompressed as follows:

\code
tar xjf robwork.tar.bz2
tar xjf robworkstudio.tar.bz2
\endcode

\subsection subsec_rw_install_linux_compile Building RobWork and RobWorkStudio

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

To build the RobWork collection of libraries do as follows:

\code
cd RobWork
cmake .
make
\endcode

(All make targets can also be run from eclipse. As an advanced linux
user you will know how this works.)

The RobWorkStudio program is build as follows:

\code
cd RobWorkStudio
cmake .
make
\endcode

\subsection subsec_rw_install_linux_start_robworkstudio Starting RobWorkStudio

The RobWorkStudio program a \c RobWorkStudio.ini file from the current
working directory. To create the \c RobWorkStudio.ini file and start up
RobWorkStudio do as follows:

\code
cd RobWorkStudio/bin
cp RobWorkStudio.ini_template RobWorkStudio.ini
./RobWorkStudio
\endcode

\section sec_rw_install_windows Installation on Windows

Building of RobWork on Windows is currently fully supported only for
<a href="http://www.mingw.org">MinGW</a>. If you don't need
RobWorkStudio then RobWork can be build using <a
href="http://msdn.microsoft.com/vstudio">Microsoft Visual Studio</a>
also.

\subsection subsec_rw_install_windows_mingw Using MingW

Dowload and install the following:

- <a href="http://www.cmake.org">CMake</a>.

- <a href="http://www.mingw.org">MinGW</a>. Your
  installation should include the following packages or libraries:
  - gcc-core
  - gcc-g77
  - gcc-g++
  - binutils
  - w32api
  - mingw-runtime
  - lapack
  - blas
  .

- <a href="http://trolltech.com">QT4</a>

The \c bin directories of CMake, MinGW, and Qt should be added the
Windows \c Path environment variable (see Control Panel : System :
Advanced : Environment Variables). You can check that the programs can
all be found by running these commands in the Windows \c cmd program:

\code
cmake --version
g++ --version
moc --version
\endcode

<a href="http://www.robwork.dk">Download</a> and uncompress the
RobWork and RobWorkStudio packages. In the RobWork directory run the
following command lines to build the RobWork collection of libraries:
\code
cmake -G "MinGW Makefiles"
mingw32-make
\endcode

Run the same commands in the \c RobWorkStudio directory to build the
RobWorkStudio program:

\code
cmake -G "MinGW Makefiles"
make
\endcode

The RobWorkStudio program is found in the \c RobWorkStudio\\bin
directory. Before running the program for the first time, you must
rename the \c RobWorkStudio.ini_template file found in the \c
RobWorkStudio\\bin directory to \c RobWorkStudio.ini.

\subsection subsec_rw_install_windows_visual Using Visual Studio

Building of RobWork has been tested for Visual Studio 8 2005.
RobWorkStudio can be build also, but only if you can find a proper
version of QT4 for Visual Studio.

Download and install <a href="http://www.cmake.org">CMake</a>.

To see the list of compilers supported by CMake type
\code
cmake
\endcode
in a \c cmd window. For this example we we assume the compiler used is
"Visual Studio 8 2005".

In the RobWork folder run
\code
cmake -G "Visual Studio 8 2005"
\endcode
This can be done also via the CMake GUI.

Running the CMake program generates a Visual Studio solution file
named \c RobWork. Open the solution file with Visual Studio and build
the solution. The libraries being build are placed in \c
RobWork\\libs\\Debug or \c RobWork\\libs\\Release depending on the
build type.

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

*/
