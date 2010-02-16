
#add_executable(GraspTest grasptest.cpp)
#target_link_libraries(GraspTest rwsim ${LibraryList})

#add_executable(ForceClosureTest forceclosuretest.cpp)
#target_link_libraries(ForceClosureTest rwsim ${LibraryList})

SET(BOOST_ROOT "C:/local/Boost-1.34.1")
FIND_PACKAGE(Boost COMPONENTS thread REQUIRED)    

LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

add_executable(demo-simple demo-simple.cpp  sdhoptions.cpp )
target_link_libraries(demo-simple sdh ntcan.lib)

add_executable(demo-simple2 demo-simple2.cpp sdhoptions.cpp )
target_link_libraries(demo-simple2 sdh ntcan.lib)

add_executable(demo-simple3 demo-simple3.cpp sdhoptions.cpp)
target_link_libraries(demo-simple3 sdh ntcan.lib)

add_executable(demo-dsa demo-dsa.cpp sdhoptions.cpp dsaboost.cpp )
target_link_libraries(demo-dsa sdh ntcan.lib ${Boost_LIBRARIES})

add_executable(demo-temperature demo-temperature.cpp sdhoptions.cpp )
target_link_libraries(demo-temperature sdh ntcan.lib ${Boost_LIBRARIES})

add_executable(demo-contact-grasping demo-contact-grasping.cpp dsaboost.cpp sdhoptions.cpp)
target_link_libraries(demo-contact-grasping sdh ntcan.lib ${Boost_LIBRARIES})

add_executable(demo-velocity-acceleration demo-velocity-acceleration.cpp sdhoptions.cpp )
target_link_libraries(demo-velocity-acceleration sdh ntcan.lib )

add_executable(demo-simple-withtiming demo-simple-withtiming.cpp sdhoptions.cpp )
target_link_libraries(demo-simple-withtiming sdh ntcan.lib )

add_executable(demo-griphand demo-griphand.cpp sdhoptions.cpp )
target_link_libraries(demo-griphand sdh ntcan.lib )

add_executable(demo-mimic demo-mimic.cpp sdhoptions.cpp )
target_link_libraries(demo-mimic sdh ntcan.lib )

