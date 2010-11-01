
# Subdirectories to process.
IF(RW_ENABLE_INTERNAL_PQP_TARGET)
    add_subdirectory(PQP)
ENDIF()
IF(RW_ENABLE_INTERNAL_YAOBI_TARGET)
    add_subdirectory(yaobi)
ENDIF()
IF(RW_ENABLE_INTERNAL_LUA_TARGET)
    add_subdirectory(lua)
ENDIF()
IF(RW_ENABLE_INTERNAL_TOLUA_TARGET)
    add_subdirectory(tolua)
ENDIF()

# A short alias for the current directory.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Copy libraries according to compiler (linux users compile library manually)
if (DEFINED MINGW)
  configure_file("${Root}/libs_mingw/glut32.lib" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/glut32.lib" COPYONLY)
  configure_file("${Root}/libs_mingw/glut32.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/glut32.dll" COPYONLY)

  configure_file("${Root}/libs_vs/blas_win32.lib" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/blas_win32.lib" COPYONLY)
  configure_file("${Root}/libs_vs/lapack_win32.lib" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/lapack_win32.lib" COPYONLY)
  configure_file("${Root}/libs_vs/blas_win32.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/blas_win32.dll" COPYONLY)
  configure_file("${Root}/libs_vs/lapack_win32.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/lapack_win32.dll" COPYONLY)

  INSTALL(FILES 
      "${Root}/libs_mingw/glut32.lib"
      DESTINATION "${RW_LIB_INSTALL_DIR}")

  INSTALL(FILES "${Root}/libs_mingw/glut32.dll" DESTINATION "${RW_BIN_INSTALL_DIR}")

  INSTALL(FILES 
      "${Root}/libs_vs/blas_win32.lib" 
      "${Root}/libs_vs/lapack_win32.lib" 
      DESTINATION "${RW_LIB_INSTALL_DIR}")

  INSTALL(FILES 
      "${Root}/libs_vs/blas_win32.dll" 
      "${Root}/libs_vs/lapack_win32.dll" 
      DESTINATION "${RW_BIN_INSTALL_DIR}")

elseif (DEFINED MSVC)
  configure_file("${Root}/libs_vs/blas_win32.lib" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE}/blas_win32.lib" COPYONLY)
  configure_file("${Root}/libs_vs/lapack_win32.lib" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE}/lapack_win32.lib" COPYONLY)
  configure_file("${Root}/libs_vs/blas_win32.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE}/blas_win32.dll" COPYONLY)
  configure_file("${Root}/libs_vs/lapack_win32.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE}/lapack_win32.dll" COPYONLY)
  
  INSTALL(FILES 
      "${Root}/libs_vs/blas_win32.lib" 
      "${Root}/libs_vs/lapack_win32.lib" 
      DESTINATION "${RW_LIB_INSTALL_DIR}")

  INSTALL(FILES 
      "${Root}/libs_vs/blas_win32.dll" 
      "${Root}/libs_vs/lapack_win32.dll" 
      DESTINATION "${RW_BIN_INSTALL_DIR}")
            
endif ()
