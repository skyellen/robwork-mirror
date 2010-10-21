
#add_executable(GraspTest grasptest.cpp)
#target_link_libraries(GraspTest rwsim ${LibraryList})

#add_executable(ForceClosureTest forceclosuretest.cpp)
#target_link_libraries(ForceClosureTest rwsim ${LibraryList})

LINK_DIRECTORIES( ${BulletLinkDir} ${OdeLinkDir} )
INCLUDE_DIRECTORIES( ${BulletIncludes} ${OdeIncludes} )

INCLUDE_DIRECTORIES("${RW_ROOT}/example/simplegui")

add_executable(ModelPropertyTest ModelPropertyTest.cpp)
target_link_libraries(ModelPropertyTest rwsim 
    ${ROBWORK_LIBRARIES})

add_executable(RWSimTest RWSimTest.cpp)
target_link_libraries(RWSimTest rwsim ${ROBWORK_LIBRARIES})

add_executable(ModelStablePoseTest ModelStablePoseTest.cpp)
target_link_libraries(ModelStablePoseTest rwsim ${ROBWORK_LIBRARIES})

add_executable(GraspTableConvert GraspTableConvert.cpp)
target_link_libraries(GraspTableConvert rwsim ${ROBWORK_LIBRARIES})

add_executable(LoadGraspTableKTH LoadGraspTableKTH.cpp)
target_link_libraries(LoadGraspTableKTH rwsim ${ROBWORK_LIBRARIES})

add_executable(ScaleWorkCell ScaleWorkCell.cpp)
target_link_libraries(ScaleWorkCell rwsim ${ROBWORK_LIBRARIES})

add_executable(PathFilter PathFilter.cpp)
target_link_libraries(PathFilter rwsim ${ROBWORK_LIBRARIES})

#add_executable(RestingPoseSimulation RestingPoseSimulation.cpp)
#target_link_libraries(RestingPoseSimulation rwsim ${ROBWORK_LIBRARIES})
