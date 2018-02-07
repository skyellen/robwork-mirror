
# Subdirectories to process.
add_subdirectory(rw)
add_subdirectory(rwlibs)

# Build sandbox if chosen
IF (NOT DEFINED RW_BUILD_SANDBOX)
  SET(RW_BUILD_SANDBOX OFF)
ENDIF()
OPTION(RW_BUILD_SANDBOX "Set when you want to build the sandbox library" ${RW_BUILD_SANDBOX})
IF ( RW_BUILD_SANDBOX )
  ADD_SUBDIRECTORY(sandbox)
  SET(RW_SANDBOX_LIB sandbox)
ENDIF ()

# Install all headerfiles
#INSTALL(DIRECTORY rw rwlibs ${RW_SANDBOX_LIB} DESTINATION ${INCLUDE_INSTALL_DIR} 
#    FILES_MATCHING 
#        PATTERN "*.h" 
#        PATTERN "*.hpp"
#        PATTERN ".svn" EXCLUDE
#)

