
# Subdirectories to process.
add_subdirectory(rw CMakeTmp/rw)
add_subdirectory(rwlibs CMakeTmp/rwlibs)
add_subdirectory(sandbox CMakeTmp/sandbox)

if (DEFINED COMPILE_SANDBOX)
  add_subdirectory(sandbox CMakeTmp/sandbox)
endif ()
