
# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project. 
PROJECT(TutorialCollection)

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
FIND_PACKAGE(RobWork REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# And now we add any targets that we want
add_executable(HelloWorld HelloWorld.cpp)
target_link_libraries(HelloWorld ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

add_executable(MathJogling MathJogling.cpp)
target_link_libraries(MathJogling ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

add_executable(WorkCellAndDevices WorkCellAndDevices.cpp)
target_link_libraries(WorkCellAndDevices ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})
