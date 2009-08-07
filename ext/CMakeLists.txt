
# Subdirectories to process.
add_subdirectory(PQP)
add_subdirectory(yaobi)
add_subdirectory(lua)
add_subdirectory(tolua)

# A short alias for the current directory.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Copy libraries according to compiler (linux users compile library manually)
if (DEFINED MINGW)
  configure_file("${Root}/libs_mingw/libblas.a" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/libblas.a" COPYONLY)
  configure_file("${Root}/libs_mingw/liblapack.a" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/liblapack.a" COPYONLY)
elseif (DEFINED MSVC)
  configure_file("${Root}/libs_vs/blas_win32.lib" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/blas_win32.lib" COPYONLY)
  configure_file("${Root}/libs_vs/lapack_win32.lib" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/lapack_win32.lib" COPYONLY)
  configure_file("${Root}/libs_vs/blas_win32.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/blas_win32.dll" COPYONLY)
  configure_file("${Root}/libs_vs/lapack_win32.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lapack_win32.dll" COPYONLY)
endif ()
