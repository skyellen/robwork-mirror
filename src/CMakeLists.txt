
# Subdirectories to process.
add_subdirectory(rw CMakeTmp/rw)
add_subdirectory(rwlibs CMakeTmp/rwlibs)

#----------------------------------------------------------------------
# Deprecated: The setup has been moved to the root CMakeLists.txt file.

# Test CMake version
# cmake_minimum_required(VERSION 2.4.4)

# Allow the syntax else (), endif (), etc.
# set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# The name of the project.
# project(RobWork)

# Load the optional RobWork.cmake file.
# include(RobWork.cmake OPTIONAL)

# It is safer to use absolute directory names.
# set(Root ${CMAKE_CURRENT_SOURCE_DIR})
# set(Ext ${Root}/../ext)

# Include directories.
# include_directories(${Ext})
# include_directories(${Ext}/boostbindings)
# include_directories(${Root})

# Output goes to bin/ and lib/ regardless of what is specified by the user.

# set(EXECUTABLE_OUTPUT_PATH ${Root}/../bin CACHE PATH "Bin directory" FORCE)
# set(LIBRARY_OUTPUT_PATH ${Root}/../libs CACHE PATH "Lib directory" FORCE)
# set(CMAKE_LIBRARY_PATH CMAKE_LIBRARY_PATH ${Root}/../libs)
