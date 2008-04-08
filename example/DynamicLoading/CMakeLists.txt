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
project(LIBRARY_PROJECT_NAME)


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
set(AppName DynamicLoadingTest)

set(LibraryNameA PluginA)
set(LibraryNameB PluginB)

#
# Source files
#
set(AppSrcFiles 
	main.cpp 	  
)

set(PluginASrcFiles 
	PluginA.cpp 	  
)

set(PluginBSrcFiles 
	PluginB.cpp 	  
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
# To create a shared library use
#
add_library(${LibraryNameA} SHARED ${PluginASrcFiles} )
target_link_libraries(${LibraryNameA} ${LibraryList})

add_library(${LibraryNameB} SHARED ${PluginBSrcFiles} )
target_link_libraries(${LibraryNameB} ${LibraryList})


#
# To create an application use
#
add_executable(${AppName} ${AppSrcFiles})
target_link_libraries(${AppName} ${LibraryList})

