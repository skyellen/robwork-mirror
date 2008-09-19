# Each subdirectory is a library. Process them.

include(FindOpenGL)

add_subdirectory(algorithms)
add_subdirectory(proximitystrategies)
add_subdirectory(drawable)
add_subdirectory(pathplanners)
add_subdirectory(pathoptimization)
add_subdirectory(lua)
add_subdirectory(dll)

foreach (dir ${RWLIBS_USER_DIRECTORIES})
  add_subdirectory(${dir})
endforeach ()
