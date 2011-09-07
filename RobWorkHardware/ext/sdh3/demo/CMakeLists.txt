
#add_executable(GraspTest grasptest.cpp)
#target_link_libraries(GraspTest rwsim ${LibraryList})

#add_executable(ForceClosureTest forceclosuretest.cpp)
#target_link_libraries(ForceClosureTest rwsim ${LibraryList})

FIND_PACKAGE(Boost COMPONENTS thread REQUIRED)    

LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

IF(WITH_ESD_CAN)
	SET(ESD_CAN_LIBS ntcan)
ENDIF()

IF(WITH_PEAK_CAN)
	SET(PEAK_CAN_LIBS pcan)
ENDIF()

add_executable(demo-simple demo-simple.cpp  sdhoptions.cpp )
target_link_libraries(demo-simple sdh ${ESD_CAN_LIBS} ${PEAK_CAN_LIBS})

add_executable(demo-simple2 demo-simple2.cpp sdhoptions.cpp )
target_link_libraries(demo-simple2 sdh ${ESD_CAN_LIBS} ${PEAK_CAN_LIBS})

add_executable(demo-simple3 demo-simple3.cpp sdhoptions.cpp)
target_link_libraries(demo-simple3 sdh ${ESD_CAN_LIBS} ${PEAK_CAN_LIBS})

add_executable(demo-dsa demo-dsa.cpp sdhoptions.cpp dsaboost.cpp )
target_link_libraries(demo-dsa sdh ${ESD_CAN_LIBS} ${PEAK_CAN_LIBS} ${Boost_LIBRARIES})

add_executable(demo-temperature demo-temperature.cpp sdhoptions.cpp )
target_link_libraries(demo-temperature sdh ${ESD_CAN_LIBS} ${PEAK_CAN_LIBS} ${Boost_LIBRARIES})

add_executable(demo-contact-grasping demo-contact-grasping.cpp dsaboost.cpp sdhoptions.cpp)
target_link_libraries(demo-contact-grasping sdh ${ESD_CAN_LIBS} ${PEAK_CAN_LIBS} ${Boost_LIBRARIES})

add_executable(demo-velocity-acceleration demo-velocity-acceleration.cpp sdhoptions.cpp )
target_link_libraries(demo-velocity-acceleration sdh ${ESD_CAN_LIBS} ${PEAK_CAN_LIBS} )

add_executable(demo-simple-withtiming demo-simple-withtiming.cpp sdhoptions.cpp )
target_link_libraries(demo-simple-withtiming sdh ${ESD_CAN_LIBS} ${PEAK_CAN_LIBS} )

add_executable(demo-griphand demo-griphand.cpp sdhoptions.cpp )
target_link_libraries(demo-griphand sdh ${ESD_CAN_LIBS} ${PEAK_CAN_LIBS} )

add_executable(demo-mimic demo-mimic.cpp sdhoptions.cpp )
target_link_libraries(demo-mimic sdh ${ESD_CAN_LIBS} ${PEAK_CAN_LIBS} )

