CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
# The name of the project. (EDIT THIS)
PROJECT(TutorialCollection)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

##### IMPORTANT EDIT THESE PATHS TO REFLECT YOUR ROBWORK INSTALLATION #####
# Now set the RW/RWS root (edit this if necessary)
SET(RW_ROOT "${ROOT}/../../../RobWork")
SET(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")

# set some robwork settings (EDIT THESE TO FIT YOUR ENVIRONMENT)
SET(CMAKE_BUILD_TYPE Release)

#Include default settings for constructing a robwork dependent project
SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
FIND_PACKAGE(RobWork)
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# if you have additional libraries or include dirs then add them here
INCLUDE_DIRECTORIES(${ROOT} ${ROBWORK_INCLUDE_DIR})

# And now we add any targets that we want
add_executable(HelloWorld HelloWorld.cpp)
target_link_libraries(HelloWorld ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

add_executable(MathJogling MathJogling.cpp)
target_link_libraries(MathJogling ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

add_executable(WorkCellAndDevices WorkCellAndDevices.cpp)
target_link_libraries(WorkCellAndDevices ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})
