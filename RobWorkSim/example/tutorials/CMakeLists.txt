CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)
# The name of the project. (EDIT THIS)
PROJECT(RWSimTutorialCollection)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

##### IMPORTANT EDIT THESE PATHS TO REFLECT YOUR ROBWORK INSTALLATION #####
# Now set the RW/RWS root (edit this if necessary)
SET(RobWorkSim_DIR "${ROOT}/../../../RobWorkSim/cmake")

#Include default settings for constructing a robwork dependent project
FIND_PACKAGE(RobWorkSim REQUIRED)

# if you have additional libraries or include dirs then add them here
INCLUDE_DIRECTORIES(${ROBWORKSIM_INCLUDE_DIRS})
LINK_DIRECTORIES(${ROBWORKSIM_LIBRARY_DIRS})

# And now we add any targets that we want
add_executable(SimLoop src/SimLoop.cpp)
target_link_libraries(SimLoop ${ROBWORKSIM_LIBRARIES})

#add_executable(MathJogling MathJogling.cpp)
#target_link_libraries(MathJogling ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

#add_executable(WorkCellAndDevices WorkCellAndDevices.cpp)
#target_link_libraries(WorkCellAndDevices ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})
