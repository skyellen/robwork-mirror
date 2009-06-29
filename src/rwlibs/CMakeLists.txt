# Each subdirectory is a library. Process them.

add_subdirectory(algorithms)
add_subdirectory(proximitystrategies)
add_subdirectory(drawable)
add_subdirectory(pathplanners)
add_subdirectory(pathoptimization)
add_subdirectory(lua)
add_subdirectory(dll)
add_subdirectory(simulation)
add_subdirectory(task)

foreach (dir ${RWLIBS_USER_DIRECTORIES})
  add_subdirectory(${dir})
endforeach ()

# some dirs are too simple to add CMakelists in
#INSTALL(FILES "os/rwgl.hpp" DESTINATION "include/rwlibs/os")
INSTALL(FILES "use_robwork_namespace.hpp" DESTINATION "include/rwlibs")
