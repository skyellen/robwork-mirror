#MESSAGE("ROBWORKSTUDIO LIBS: ${ROBWORKSTUDIO_LIBRARIES}")

# If we are using static linking then remember to 
IF (RWS_USE_STATIC_LINK_PLUGINS)
  MESSAGE(STATUS "RobWorkStudio: Using static linking of default plugins!")
ELSE ()
  MESSAGE(STATUS "RobWorkStudio: Using dynamic linking of default plugins!")
ENDIF ()

# First we compile the robworkstudio library
ADD_SUBDIRECTORY(rws)

# then the components 
SET(ENV{RWS_COMPONENT_LIBRARIES} "")
ADD_SUBDIRECTORY(components)

# then the plugins 
SET(ENV{RWS_PLUGIN_LIBRARIES} "")
ADD_SUBDIRECTORY(plugins)

# Then sandbox if choosen
IF ( RWS_BUILD_SANDBOX )
    ADD_SUBDIRECTORY(sandbox)
    SET(RWS_HAVE_SANDBOX ON CACHE BOOL "")
    SET(RWS_SANDBOX_LIB rwstudio_sandbox)
ENDIF ()

SET(PluginLibraries $ENV{RWS_PLUGIN_LIBRARIES} CACHE string "plugin libs")
SET(ComponentLibraries $ENV{RWS_COMPONENT_LIBRARIES} CACHE string "component libs")

# now create the actual RobWorkStudio executable

# Libraries that RobWorkStudio depends on
set(LibraryList
  ${PluginLibraries}
  ${ComponentLibraries}
  ${RWS_SANDBOX_LIB}
  ${ROBWORKSTUDIO_LIBRARIES}
  ${ROBWORK_LIBRARIES}
)

#MESSAGE(${LibraryList})

# Now the RobWorkStudio executable need to be build:
ADD_EXECUTABLE(RobWorkStudio main.cpp)
TARGET_LINK_LIBRARIES(RobWorkStudio ${LibraryList})

# Some install stuff
INSTALL(TARGETS RobWorkStudio DESTINATION bin)
INSTALL(FILES "${RWSTUDIO_ROOT}/bin/RobWorkStudio.ini_template" 
        DESTINATION bin
        RENAME "RobWorkStudio.ini"
)

# -------------------------------------------------------------------------
# this is used to set up installation of dlls/so's that RobWorkStudio 
# depends on
SET(DEPENDS_DIRS 
    "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" 
    "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" 
#    ${QT_LIBRARIES} 
     ${QT_BINARY_DIR}
     ${QT_PLUGINS_DIR}
)

SET(target "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/RobWorkStudio.exe")
SET(EXEPATH "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

INCLUDE(GetPrerequisites OPTIONAL RESULT_VARIABLE prereq_result)
IF( prereq_result ) 
    IF(WIN32)
        SET(RWS_DEPENDENCIES 
            "QtCore4.dll;QtGui4.dll;QtOpenGL4.dll;MINGWM10.dll"
            "MSVCRT.dll;GLU32.dll;libxerces-c2_8_0.dll"
        )
        
        FOREACH(item ${RWS_DEPENDENCIES})
            gp_resolve_item("${target}" "${item}" "${EXEPATH}" "${DEPENDS_DIRS}" resolved_item)
            INSTALL(FILES "${resolved_item}" DESTINATION bin)
            #MESSAGE("${resolved_item}")
        ENDFOREACH()
    ENDIF ()
ENDIF()
