#####################################################
# Template for building application and libraries 
#
# Go through the file and change the values to whats 
# appropriate for your application
#####################################################


#
# Test CMake version
#
cmake_minimum_required(VERSION 2.4.4)


#
# The name of the project.
#
project(InterpolatorExample)


#
#Set RobWork Root
#
set(RW_ROOT ../../)
set(RWAPP_ROOT ../../../RobWorkApp)
#
# Setup which additional packages to use (0: Do not use, 1: Use)
#
set(UseOrocos 0)
set(UseOpenGL 0)
set(UseIPL98 0)

#
#Include default settings for constructing a RobWorkStudio Plugin
#
include(${RWAPP_ROOT}/RobWorkCommon.cmake)


#
# The shared library to build:
#
set(AppName InterpolatorTest)


#
# Source files
#
set(SrcFiles 
	main.cpp 	  
)



#
# Header files (optional)
#
set(HeaderFiles 
	HEADER_FILES
)

#
# Additional compile options goes here
#
if (WIN32)
	set(LibraryList ${LibraryList} kernel32 )
else ()
	set(LibraryList ${LibraryList} dl)
endif ()

#
# To create a static library use
#
#add_library(${TargetName} ${SrcFiles} )


#
# To create an application use
#
add_executable(${AppName} ${SrcFiles})
target_link_libraries(${AppName} ${LibraryList})

