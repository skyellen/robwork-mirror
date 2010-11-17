
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

# Copy libraries according to compiler (Linux users compile libraries manually)
IF(WIN32)
  # Set library suffix
  IF(AMD64)
    SET(BLAS_LAPACK_SUFFIX "_win64")
  ELSE()
    SET(BLAS_LAPACK_SUFFIX "_win32")
  ENDIF()
  
  # Set library destination and copy MinGW GLUT libraries
  IF(MINGW)
    configure_file("${Root}/libs_mingw/glut32.lib" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/glut32.lib" COPYONLY)
    configure_file("${Root}/libs_mingw/glut32.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/glut32.dll" COPYONLY)
    INSTALL(FILES "${Root}/libs_mingw/glut32.lib" DESTINATION "${RW_LIB_INSTALL_DIR}")
    INSTALL(FILES "${Root}/libs_mingw/glut32.dll" DESTINATION "${RW_BIN_INSTALL_DIR}")
    
    SET(BLAS_LAPACK_DEST "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
  ELSEIF(MSVC)
    SET(BLAS_LAPACK_DEST "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE}")    
  ENDIF()
  
  # Copy libraries
  configure_file("${Root}/libs_vs/blas${BLAS_LAPACK_SUFFIX}.lib"
                 "${BLAS_LAPACK_DEST}/blas${BLAS_LAPACK_SUFFIX}.lib"
                 COPYONLY)
  configure_file("${Root}/libs_vs/lapack${BLAS_LAPACK_SUFFIX}.lib"
                 "${BLAS_LAPACK_DEST}/lapack${BLAS_LAPACK_SUFFIX}.lib"
                 COPYONLY)
	configure_file("${Root}/libs_vs/libf2c${BLAS_LAPACK_SUFFIX}.lib"
                 "${BLAS_LAPACK_DEST}/libf2c${BLAS_LAPACK_SUFFIX}.lib"
                 COPYONLY)
  
  # Install libraries
  INSTALL(FILES "${Root}/libs_vs/blas${BLAS_LAPACK_SUFFIX}.lib" 
                "${Root}/libs_vs/lapack${BLAS_LAPACK_SUFFIX}.lib" 
                "${Root}/libs_vs/libf2c${BLAS_LAPACK_SUFFIX}.lib"
          DESTINATION "${RW_LIB_INSTALL_DIR}")
ENDIF()

