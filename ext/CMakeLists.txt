
# Subdirectories to process.
add_subdirectory(opcode CMakeTmp/opcode)
add_subdirectory(PQP CMakeTmp/pqp)
add_subdirectory(lua)
add_subdirectory(tolua)
add_subdirectory(yaobi CMakeTmp/yaobi)

# A short alias for the current directory.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Copy libraries according to compiler (linux users compile library manually)
if (DEFINED WIN32)
  if (DEFINED MINGW)
    configure_file(${Root}/libs_mingw/libblas.a ${LIBRARY_OUTPUT_PATH} COPYONLY)
    configure_file(${Root}/libs_mingw/liblapack.a ${LIBRARY_OUTPUT_PATH} COPYONLY)
  else ()

    configure_file(${Root}/libs_vs/blas_win32.lib ${LIBRARY_OUTPUT_PATH} COPYONLY)
    configure_file(${Root}/libs_vs/lapack_win32.lib ${LIBRARY_OUTPUT_PATH} COPYONLY)

  endif ()
endif ()
