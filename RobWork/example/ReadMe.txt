GENERAL
This folder contains example plugins for RobWork as well as example XML specifations of a number of workcell scenes and devices. If you have downloaded the RobWork installer, make sure to install CMake before trying to compile the plugins.

The automated .deb and .exe installers for Linux and Windows will place the RobWork installation in /usr/local or in the program files folder. Since normal users do not have write access to these folders, you need to copy the example plugins to your user folder or another folder to which you have write access before compilation.

The plugin folders all contain a CMakeLists.txt file which sets up the RobWork path RW_ROOT to the default installation path. If you have RobWork installed elsewhere, be sure to edit this file before compilation.

LINUX USERS
- In the plugin directory, call:

	cmake .
	make


MINGW USERS
- In the plugin directory, call:

	cmake -G "MinGW Makefiles" .
	mingw32-make


MSVC USERS
- Start up the Visual Studio Command Prompt, cd to the plugin directory and call (replacing PROJECT by the plugin name):

	- Visual Studio 2005:
	
		cmake -G "Visual Studio 8 2005" .
		msbuild PROJECT.sln /p:configuration=Release|Debug

	- Visual Studio 2008:
	
		cmake -G "Visual Studio 9 2008" .
		msbuild PROJECT.sln /p:configuration=Release|Debug

	- Visual Studio 2010:
	
		cmake -G "Visual Studio 10" .
		msbuild PROJECT.sln /p:configuration=Release|Debug