
#add_executable(GraspTest grasptest.cpp)
#target_link_libraries(GraspTest rwsim ${LibraryList})

#add_executable(ForceClosureTest forceclosuretest.cpp)
#target_link_libraries(ForceClosureTest rwsim ${LibraryList})

LINK_DIRECTORIES( ${BulletLinkDir} ${OdeLinkDir} )
INCLUDE_DIRECTORIES( ${BulletIncludes} ${OdeIncludes} )

INCLUDE_DIRECTORIES("${RW_ROOT}/example/simplegui")

add_executable(ModelPropertyTest ModelPropertyTest.cpp)
target_link_libraries(ModelPropertyTest rwsim ${LibraryList})

add_executable(RWSimTest RWSimTest.cpp)
target_link_libraries(RWSimTest rwsim ${LibraryList})

add_executable(ModelStablePoseTest ModelStablePoseTest.cpp)
target_link_libraries(ModelStablePoseTest rwsim ${LibraryList})

