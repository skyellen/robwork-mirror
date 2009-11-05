
#add_executable(GraspTest grasptest.cpp)
#target_link_libraries(GraspTest rwsim ${LibraryList})

#add_executable(ForceClosureTest forceclosuretest.cpp)
#target_link_libraries(ForceClosureTest rwsim ${LibraryList})

add_executable(demo-simple demo-simple.cpp sdhoptions.cpp)
target_link_libraries(demo-simple sdh ntcan.lib)

add_executable(demo-simple2 demo-simple2.cpp sdhoptions.cpp)
target_link_libraries(demo-simple2 sdh ntcan.lib)

add_executable(demo-simple3 demo-simple3.cpp sdhoptions.cpp)
target_link_libraries(demo-simple3 sdh ntcan.lib)


add_executable(demo-dsa demo-dsa.cpp dsaoptions.cpp)
target_link_libraries(demo-dsa sdh ntcan.lib)
