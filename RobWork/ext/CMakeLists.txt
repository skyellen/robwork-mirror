
#INCLUDE(ExternalProject)

# A short alias for the current directory.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

#SET(LIB_INSTALL_DIR ${RW_LIB_INSTALL_DIR})
#SET(SRC_INSTALL_DIR ${RW_EXT_INSTALL_DIR})

# Subdirectories to process.
IF(RW_ENABLE_INTERNAL_PQP_TARGET)
    #ExternalProject_Add(
    #   rwPQP
    #   SOURCE_DIR ${Root}/PQP
    #   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} 
    #              -DLIB_INSTALL_DIR=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    #              -DSRC_INSTALL_DIR=${RW_EXT_INSTALL_DIR}
    #   INSTALL_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    #)
    add_subdirectory(rwpqp)
ENDIF()

IF(RW_ENABLE_INTERNAL_YAOBI_TARGET)
    #ExternalProject_Add(
    #       rwyaobi
       
    #   SOURCE_DIR ${Root}/yaobi
     #  CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} 
     #             -DLIB_INSTALL_DIR=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    #              -DSRC_INSTALL_DIR=${RW_EXT_INSTALL_DIR}
    #   INSTALL_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    #)

    add_subdirectory(rwyaobi)
ENDIF()
IF(RW_ENABLE_INTERNAL_LUA_TARGET)
    #ExternalProject_Add(
    #   rwlua
    #   SOURCE_DIR ${Root}/lua
    #   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} 
    #              -DLIB_INSTALL_DIR=${RW_LIB_INSTALL_DIR}
    #              -DSRC_INSTALL_DIR=${RW_EXT_INSTALL_DIR}
    #   INSTALL_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    #)
    add_subdirectory(lua)
ENDIF()
#IF(RW_ENABLE_INTERNAL_TOLUA_TARGET)
#    add_subdirectory(tolua)
#ENDIF()
#IF(RW_ENABLE_INTERNAL_QHULL_TARGET)
#ExternalProject_Add(
#   rwqhull
#   SOURCE_DIR ${Root}/qhull
#   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
#   INSTALL_COMMAND ""
#)
add_subdirectory(qhull)
#ENDIF()

IF(RW_ENABLE_INTERNAL_ZLIB_TARGET)
	add_subdirectory(zlib)
ENDIF()

IF(RW_ENABLE_INTERNAL_MINIZIP_TARGET)
	add_subdirectory(unzip)
ENDIF()

IF(RW_ENABLE_INTERNAL_ASSIMP_TARGET)
	add_subdirectory(assimp)
ENDIF()

# Copy libraries according to compiler (Linux users compile libraries manually)
#IF(WIN32)
  # Set library suffix
#  IF(AMD64)
#    SET(BLAS_LAPACK_SUFFIX "_win64")
#  ELSE()
#    SET(BLAS_LAPACK_SUFFIX "_win32")
#  ENDIF()
  
#  IF("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
#    SET(BLAS_LAPACK_SUFFIX "${BLAS_LAPACK_SUFFIX}d")   
#  ENDIF()
  
  # Set library destination and copy MinGW GLUT libraries
#  IF(MINGW)
#    SET(BLAS_LAPACK_DIR "libs_mingw")
#    SET(BLAS_LAPACK_EXTENSION "a")
    
#    configure_file("${Root}/libs_mingw/glut32.lib" "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/glut32.lib" COPYONLY)
#    configure_file("${Root}/libs_mingw/glut32.dll" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/glut32.dll" COPYONLY)
#    INSTALL(FILES "${Root}/libs_mingw/glut32.lib" DESTINATION "${LIB_INSTALL_DIR}")
#    INSTALL(FILES "${Root}/libs_mingw/glut32.dll" DESTINATION "${BIN_INSTALL_DIR}")
    
#    SET(BLAS_LAPACK_DEST "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
#  ELSEIF(MSVC)
#    SET(BLAS_LAPACK_DIR "libs_vs")
#    SET(BLAS_LAPACK_EXTENSION "lib")
    
#    SET(BLAS_LAPACK_DEST "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE}")    
#  ENDIF()
  
  # Copy libraries
#  configure_file("${Root}/${BLAS_LAPACK_DIR}/blas${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
#                 "${BLAS_LAPACK_DEST}/blas${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
#                 COPYONLY)
#  configure_file("${Root}/${BLAS_LAPACK_DIR}/lapack${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
#                 "${BLAS_LAPACK_DEST}/lapack${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
#                 COPYONLY)
#  configure_file("${Root}/${BLAS_LAPACK_DIR}/libf2c${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
#                 "${BLAS_LAPACK_DEST}/libf2c${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
#                 COPYONLY)
  
  
  # Install libraries
#  INSTALL(FILES "${Root}/${BLAS_LAPACK_DIR}/blas${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}" 
#                "${Root}/${BLAS_LAPACK_DIR}/lapack${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}" 
#                "${Root}/${BLAS_LAPACK_DIR}/libf2c${BLAS_LAPACK_SUFFIX}.${BLAS_LAPACK_EXTENSION}"
#          DESTINATION "${LIB_INSTALL_DIR}")
#ENDIF()

add_subdirectory(csgjs)

