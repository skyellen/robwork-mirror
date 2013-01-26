# Each subdirectory is a library. Process them.

add_subdirectory(algorithms)
add_subdirectory(calibration)
add_subdirectory(control)
add_subdirectory(opengl)
add_subdirectory(proximitystrategies)
add_subdirectory(pathoptimization)
add_subdirectory(pathplanners)
add_subdirectory(task)
add_subdirectory(simulation)
add_subdirectory(swig)

foreach (dir ${RWLIBS_USER_DIRECTORIES})
  add_subdirectory(${dir})
endforeach ()

# some dirs are too simple to add CMakelists in
#INSTALL(FILES "os/rwgl.hpp" DESTINATION "include/rwlibs/os")
#INSTALL(FILES "use_robwork_namespace.hpp" DESTINATION "${RW_HPP_INSTALL_DIR}/rwlibs")
