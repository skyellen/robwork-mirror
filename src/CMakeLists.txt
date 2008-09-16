
# Subdirectories to process.
add_subdirectory(rw)
add_subdirectory(rwlibs)

if (DEFINED COMPILE_SANDBOX)
  add_subdirectory(sandbox)
endif ()


if (DEFINED COMPILE_DEPRECATED)
	add_subdirectory(deprecated)
endif()