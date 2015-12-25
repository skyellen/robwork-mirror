# Each subdirectory is a library. Process them.

add_subdirectory(algorithms)
add_subdirectory(assembly)
add_subdirectory(calibration)
add_subdirectory(control)
add_subdirectory(opengl)
add_subdirectory(mathematica)
add_subdirectory(proximitystrategies)
add_subdirectory(pathoptimization)
add_subdirectory(pathplanners)
add_subdirectory(task)
add_subdirectory(simulation)
add_subdirectory(swig)
add_subdirectory(softbody)
add_subdirectory(geometry)
add_subdirectory(csg)

#add_subdirectory(grasper)

foreach (dir ${RWLIBS_USER_DIRECTORIES})
  add_subdirectory(${dir})
endforeach ()

# some dirs are too simple to add CMakelists in
#INSTALL(FILES "os/rwgl.hpp" DESTINATION "include/rwlibs/os")
#INSTALL(FILES "use_robwork_namespace.hpp" DESTINATION "${RW_HPP_INSTALL_DIR}/rwlibs")
