
# Subdirectories to process.
add_subdirectory(rw)
add_subdirectory(rwlibs)

OPTION(RW_BUILD_SANDBOX "Set when you want to build the sandbox library" ${RW_BUILD_SANDBOX})
IF ( DEFINED COMPILE_SANDBOX OR RW_BUILD_SANDBOX )
  ADD_SUBDIRECTORY(sandbox)
  SET(RW_SANDBOX_LIB sandbox)
ENDIF ()

INSTALL(DIRECTORY rw rwlibs ${RW_SANDBOX_LIB} DESTINATION ${RW_HPP_INSTALL_DIR} 
    FILES_MATCHING 
        PATTERN "*.h" 
        PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY dirs... DESTINATION <dir>
#          [FILE_PERMISSIONS permissions...]
#          [DIRECTORY_PERMISSIONS permissions...]
#          [USE_SOURCE_PERMISSIONS]
#          [CONFIGURATIONS [Debug|Release|...]]
#          [COMPONENT <component>] [FILES_MATCHING]
#          [[PATTERN <pattern> | REGEX <regex>]
#           [EXCLUDE] [PERMISSIONS permissions...]] [...])
