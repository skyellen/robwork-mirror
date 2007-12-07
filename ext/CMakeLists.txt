
# Subdirectories to process.
add_subdirectory(opcode CMakeTmp/opcode)
add_subdirectory(PQP CMakeTmp/pqp)
add_subdirectory(lua)
add_subdirectory(tolua)

# You have to install orocos yourself... 
# add_subdirectory(orocos CMakeTmp/orocos)

# A short alias for the current directory.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Copy libraries according to compiler (linux users compile library manually)
if (DEFINED WIN32)
  if (DEFINED MINGW)
    configure_file(${Root}/libs_mingw/libblas.a ${LIBRARY_OUTPUT_PATH} COPYONLY)
    configure_file(${Root}/libs_mingw/liblapack.a ${LIBRARY_OUTPUT_PATH} COPYONLY)
  else ()
    configure_file(${Root}/libs_vs/blas_win32.lib ${LIBRARY_OUTPUT_PATH} COPYONLY)
    configure_file(${Root}/libs_vs/lapack_win32.lib ${LIBRARY_OUTPUT_PATH} COPYONLY)
  endif ()
endif ()

#----------------------------------------------------------------------
# Deprecated: The setup has been moved to the root CMakeLists.txt file.

# Test CMake version
# cmake_minimum_required(VERSION 2.4.4)

# Allow the syntax else (), endif (), etc.
# set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Include directories.
# (There are no include directories to consider.)

# The name of the project.
# project(RobWorkExt)

# Load the optional RobWork.cmake file.
# include(RobWorkExt.cmake OPTIONAL)

# Output goes to bin/ and lib/ regardless of what is specified by the user.
#  set(EXECUTABLE_OUTPUT_PATH ${Root}/../bin CACHE PATH "Bin directory" FORCE)
#  set(LIBRARY_OUTPUT_PATH ${Root}/../libs CACHE PATH "Lib directory" FORCE)
