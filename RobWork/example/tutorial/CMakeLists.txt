
# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project. 
PROJECT(TutorialCollection)

SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
FIND_PACKAGE(RobWork REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# Set the output dir for generated libraries and binaries
IF(MSVC)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
ELSE()
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
ENDIF()

# And now we add any targets that we want
add_executable(HelloWorld HelloWorld.cpp)
target_link_libraries(HelloWorld ${ROBWORK_LIBRARIES})

add_executable(InverseKinematics InverseKinematics.cpp)
target_link_libraries(InverseKinematics ${ROBWORK_LIBRARIES})

add_executable(MathJogling MathJogling.cpp)
target_link_libraries(MathJogling ${ROBWORK_LIBRARIES})

add_executable(WorkCellAndDevices WorkCellAndDevices.cpp)
target_link_libraries(WorkCellAndDevices ${ROBWORK_LIBRARIES})

if ( CMAKE_COMPILER_IS_GNUCC )
    # Do not warn about unused variables. They are there for demonstration.
    target_compile_options(InverseKinematics PRIVATE "-Wno-unused-variable;-Wno-unused-but-set-variable")
    target_compile_options(WorkCellAndDevices PRIVATE "-Wno-unused-variable;-Wno-unused-but-set-variable")
endif()