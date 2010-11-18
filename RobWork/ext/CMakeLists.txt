
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
  
  IF("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    SET(BLAS_LAPACK_SUFFIX "${BLAS_LAPACK_SUFFIX}d")   
  ENDIF()
  
  # Set library destination and copy MinGW GLUT libraries
  IF(MINGW)
    SET(BLAS_LAPACK_DIR "libs_mingw")
    SET(BLAS_LAPACK_EXTENSION "a")
    
    configure_file("${Root}/libs_mingw/glut32.lib" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/glut32.lib" COPYONLY)
    configure_file("${Root}/libs_mingw/glut32.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/glut32.dll" COPYONLY)
    INSTALL(FILES "${Root}/libs_mingw/glut32.lib" DESTINATION "${RW_LIB_INSTALL_DIR}")
    INSTALL(FILES "${Root}/libs_mingw/glut32.dll" DESTINATION "${RW_BIN_INSTALL_DIR}")
    
    SET(BLAS_LAPACK_DEST "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
  ELSEIF(MSVC)
    SET(BLAS_LAPACK_DIR "libs_vs")
    SET(BLAS_LAPACK_EXTENSION "lib")
    
    SET(BLAS_LAPACK_DEST "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE}")    
  ENDIF()
  
  # Copy libraries
  configure_file("${Root}/${BLAS_LAPACK_DIR}/blas${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
                 "${BLAS_LAPACK_DEST}/blas${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
                 COPYONLY)
  configure_file("${Root}/${BLAS_LAPACK_DIR}/lapack${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
                 "${BLAS_LAPACK_DEST}/lapack${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
                 COPYONLY)
  configure_file("${Root}/${BLAS_LAPACK_DIR}/libf2c${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
                 "${BLAS_LAPACK_DEST}/libf2c${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
                 COPYONLY)
  
  # Copy and install debugging symbols
  IF(MSVC AND "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    configure_file("${Root}/${BLAS_LAPACK_DIR}/blas${BLAS_LAPACK_SUFFIX}.pdb"
                   "${BLAS_LAPACK_DEST}/blas${BLAS_LAPACK_SUFFIX}.pdb"
                   COPYONLY)
    configure_file("${Root}/${BLAS_LAPACK_DIR}/lapack${BLAS_LAPACK_SUFFIX}.pdb"
                   "${BLAS_LAPACK_DEST}/lapack${BLAS_LAPACK_SUFFIX}.pdb"
                   COPYONLY)
    configure_file("${Root}/${BLAS_LAPACK_DIR}/libf2c${BLAS_LAPACK_SUFFIX}.pdb"
                   "${BLAS_LAPACK_DEST}/libf2c${BLAS_LAPACK_SUFFIX}.pdb"
                   COPYONLY)
                   
  INSTALL(FILES "${Root}/${BLAS_LAPACK_DIR}/blas${BLAS_LAPACK_SUFFIX}.pdb" 
                "${Root}/${BLAS_LAPACK_DIR}/lapack${BLAS_LAPACK_SUFFIX}.pdb" 
                "${Root}/${BLAS_LAPACK_DIR}/libf2c${BLAS_LAPACK_SUFFIX}.pdb"
          DESTINATION "${RW_LIB_INSTALL_DIR}")
  ENDIF()
  
  # Install libraries
  INSTALL(FILES "${Root}/${BLAS_LAPACK_DIR}/blas${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}" 
                "${Root}/${BLAS_LAPACK_DIR}/lapack${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}" 
                "${Root}/${BLAS_LAPACK_DIR}/libf2c${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
          DESTINATION "${RW_LIB_INSTALL_DIR}")
ENDIF()

