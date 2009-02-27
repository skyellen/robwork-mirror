
# Subdirectories to process.
add_subdirectory(rw)
add_subdirectory(rwlibs)

OPTION(RW_BUILD_SANDBOX "Set when you want to build the sandbox library" ${RW_BUILD_SANDBOX})
IF ( DEFINED COMPILE_SANDBOX OR BUILD_SANDBOX )
  ADD_SUBDIRECTORY(sandbox)
ENDIF ()


