// -*- latex -*-

/**

\page page_rw_installation Installation of RobWork

\section sec_rw_install_linux Installation on Linux

\subsection subsec_rw_install_dependencies Dependencies

Make sure to have the following dependencies installed:

- gcc (version 4.1 or above is recommended)
- cmake
- blas-atlas
- lapack-atlas
- libdc1394
- libraw1394
- qt

The qt dependency is only needed if you want to use RobWorkStudio.


Now construct a working directory like
\code
mkdir working \n
cd working
\endcode
and download the software from the website to this location. Next unpack the software:
\code
tar xvjf robwork.tar.bz2 
tar xvjf robworkstudio.tar.bz2 
\endcode

\subsection subsec_rw_install_linux Compile on Linux
Now do the following:

\code
cd RobWork/ext/orocos
cmake .
make
make install
cd ../..
cmake .
make
\endcode
 

Notice that the make install of orocos will install locally - not globally. Therefore it's not necessary to have administrative rights. All make targets can also be run from eclipse. As an advanced linux user you will know how this works. Else refer to some of the other linux tutorials.

If everything compiles you've compiled RobWork successfully. Now to RobWorkStudio.
RobWorkStudio

Do the following:
\code
cmake .
make
\endcode
If everything compiled without errors, write the following
\code
cd bin
cp RobWorkStudio.ini_template RobWorkStudio.ini
./RobWorkStudio
\endcode
Now if the program is running - everything is successfull. 

\section sec_rw_install_windows Installation on Windows 

\subsection subsec_rw_windows_dependencies 3'rd party dependancies

- CMake - RobWork use the cmake build system. CMake can be downloaded from the cmake project site, please see the link section for details.
- Blas and lapack - libraries that typically exists on linux/unix platforms. We have supplied precompiled libraries for windows platforms in our download section.
- (Optional) Eclipse Europe - Eclipse Europe is a c++ programming framework. Eclipse project files are supplied with the RobWork source. 

\subsection subsec_rw_install_mingw On Windows using MingW
- Download and install cmake from www.cmake.org
- Download and install mingw from www.mingw.org . Make sure that you install the following packages: gcc-core, gcc-g77, gcc-g++, binutils, w32api and mingw-runtime
- Download and install QT4 from http://trolltech.com/ . Notice that If you choose to install mingw using the QT installer you will be missing the gcc-g77 package.
- Check that the installers has made the correct setup of system variables
          - QTDIR set to your QT folder (e.g. c:\\local\\Qt\\4.3.1)
          - PATH contains you QT, MingQ and System32 folders (e.g. c:\\local\\Qt\\4.3.1; c:\\local\\MingW\\bin; c:\\Windows\\System32)
          - QMAKESPEC s set to win32-g++
- Download the RobWork and RobWorkStudio source code
- Goto the RobWork folder, run \code cmake -G "MinGW Makefiles" .\endcode and then \code make \endcode If MinGW if the only compiler on you system you can omit the "-G "MinGW Makefiles".
- Goto the RobWorkStudio folder, run \code cmake -G "MinGW Makefiles" .\endcode and then \code make\endcode If MinGW is the only compiler on you system you can omit the "-G "MinGW Makefiles".
- Goto the RobWorkStudio\\bin folder and run \code RobWorkStudio.exe \endcode

As default the project is compiles without orocos. To use orocos WHAT WAS IT WE NEEDED TO DO FOR THIS?

- Goto rwroot/ext/orocos/
- execute \code cmake .\endcode
- execute \code make\endcode and then \code make install \endcode
- liborocos-rtt.a will be installed to the rwroot/libs/ folder and all header files will be installed to rwroot/ext/orocos/include 

Common pitfals:

- Problem - CMake does not find MinGW but instead locates another compiler (visual cl.exe or the likes).
    - Solution - Check if MinGW is in your system path. If you have more than one make system on your platform then CMake will automatically choose one. To force CMake to choose MinGW compiler use <cmake -G"MinGW Makefiles" .> instead of just <cmake .>
- Problem - 


\subsection subsec_rw_install_visual On Windows using Visual Studio
Only compilation of the RobWork core libraries are supported by Visual C++. We currently have not tried to compile RobWorkStudio with Visual C++. However, if you have and would like to explain how, please send us the information.

- Download and install cmake from www.cmake.org
- To see which versions of Visual Studio cmake support simple type \code cmake\endcode and look at the list generators
- Goto the RobWork folder, run \code cmake -G "Visual Studio 8 2005" .\endcode or whatever Visual Studio version you prefer.
    - Notice: Currently we have just tried with "Visual Studio 8 2005".
- Now open the Visual Studio solution and compile.
    - Notice: Newer versions of Visual Studio has many of the c-style functions (e.g. sscanf, fopen, etc.) as deprecated and gives warnings. Just ignore these.
- On compilation Visual Studio adds a Debug or Release folder in the RobWork\\libs directory and places the library files there.

*/
