
# Subdirectories to process.
add_subdirectory(rw CMakeTmp/rw)
add_subdirectory(rwlibs CMakeTmp/rwlibs)

if (DEFINED COMPILE_SANDBOX)
  add_subdirectory(sandbox CMakeTmp/sandbox)
endif ()


if (DEFINED COMPILE_DEPRECATED)
	add_subdirectory(deprecated CMakeTmp/deprecated)
endif()