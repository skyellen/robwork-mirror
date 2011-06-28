#MESSAGE("ROBWORKSTUDIO LIBS: ${ROBWORKSTUDIO_LIBRARIES}")


# First we compile the robworkstudio library
ADD_SUBDIRECTORY(rws)


# Then sandbox if choosen
OPTION(RWS_BUILD_SANDBOX "Set when you want to build the sandbox library" ${RWS_BUILD_SANDBOX})
IF ( RWS_BUILD_SANDBOX )
    ADD_SUBDIRECTORY(sandbox)
ENDIF ()
	
SET(RWS_PLUGIN_LIBRARIES $ENV{RWS_PLUGIN_LIBRARIES} CACHE string "plugin libs")
SET(RWS_COMPONENT_LIBRARIES $ENV{RWS_COMPONENT_LIBRARIES} CACHE string "component libs")

# now create the actual RobWorkStudio executable

# Now the RobWorkStudio executable need to be build:
ADD_EXECUTABLE(RobWorkStudio main.cpp ProgramOptions.cpp )
SET(LIBS_TMP ${RWS_PLUGIN_LIBRARIES} rwsim_gui RWSimPluginStatic rwsim_ode ode rwsim rwstudio ${RWS_COMPONENT_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})
TARGET_LINK_LIBRARIES(RobWorkStudio ${LIBS_TMP})
IF (RWS_USE_STATIC_LINK_PLUGINS)
    ADD_DEPENDENCIES(RobWorkStudio rwstudio $ENV{RWS_PLUGIN_LIBRARIES} $ENV{RWS_COMPONENT_LIBRARIES} )
ENDIF()

# Some install stuff
INSTALL(TARGETS RobWorkStudio DESTINATION ${RWS_BIN_INSTALL_DIR})
IF(RWS_USE_STATIC_LINK_PLUGINS)
    INSTALL(FILES "${RWSTUDIO_ROOT}/bin/RobWorkStudio.ini.template.static" 
	        DESTINATION ${RWS_BIN_INSTALL_DIR}
            RENAME "RobWorkStudio.ini.template" 
    )
    CONFIGURE_FILE("${RWSTUDIO_ROOT}/bin/RobWorkStudio.ini.template.static"
                   "${RWSTUDIO_ROOT}/bin/${CMAKE_BUILD_TYPE}/RobWorkStudio.ini.template"
                   COPYONLY
    )    
ELSE()
    INSTALL(FILES "${RWSTUDIO_ROOT}/bin/RobWorkStudio.ini.shared"
	        DESTINATION ${RWS_BIN_INSTALL_DIR}
	        RENAME "RobWorkStudio.ini"
    )
    CONFIGURE_FILE("${RWSTUDIO_ROOT}/bin/RobWorkStudio.ini.shared"
                   "${RWSTUDIO_ROOT}/bin/${CMAKE_BUILD_TYPE}/RobWorkStudio.ini"
                   COPYONLY
    )    
ENDIF()

# Install all headerfiles
INSTALL(DIRECTORY rws DESTINATION ${RWS_HPP_INSTALL_DIR} 
    FILES_MATCHING 
        PATTERN "*.h" 
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
)
