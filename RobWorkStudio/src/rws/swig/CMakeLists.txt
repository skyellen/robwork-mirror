
FIND_PACKAGE(SWIG 1.3 QUIET)
IF( SWIG_FOUND )	
    INCLUDE(UseSWIG)
    SET(CMAKE_SWIG_FLAGS "")
    SET(CMAKE_SWIG_OUTDIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    IF( RWS_HAVE_LUA )
       
    
    	SET_SOURCE_FILES_PROPERTIES(rws.i PROPERTIES CPLUSPLUS ON)
    	SET_SOURCE_FILES_PROPERTIES(rws.i PROPERTIES SWIG_FLAGS "-includeall")
    
        SWIG_ADD_MODULE(rws_lua lua rws.i LuaState.cpp ScriptTypes.cpp)
        SWIG_LINK_LIBRARIES(rws_lua rwstudio rwstudioapp
                    ${RWS_COMPONENT_LIBRARIES} 
                    ${RWS_PLUGIN_LIBRARIES} 
                    ${ROBWORKSTUDIO_LIBRARIES} 
                    ${ROBWORK_LIBRARIES}
                    ${BOOST_LIBRARIES})
        
        ADD_DEPENDENCIES(rws_lua LuaEditorWindow)
        
        ADD_LIBRARY(rwstudio_lua STATIC ${swig_generated_sources} ${swig_other_sources})
        LINK_LIBRARIES(rwstudio_lua rwstudio rwstudioapp ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})

        ADD_DEPENDENCIES(rwstudio_lua LuaEditorWindow)

        #MESSAGE("${ROBWORKSTUDIO_LIBRARIES} \n ROBWORK ${ROBWORK_LIBRARIES}" )
        # this is used to indicate static linking to Visual Studio or mingw 
	    	# this is used to indicate static linking to Visual Studio or mingw
    	IF (DEFINED MSVC) 
    	    SET_TARGET_PROPERTIES(rwstudio_lua PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
    	ELSE()
    	    SET_TARGET_PROPERTIES(rwstudio_lua PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
    	ENDIF()
    	
	ELSE()
	    MESSAGE(FATAL_ERROR "RobWorkStudio: SWIG not found, LUA disabled DISABLED!")
    ENDIF()

INCLUDE(RobWorkMacros)

FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)
INCLUDE(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(RWS_PYTHON_BINDINGS_ENABLED "Enable python bindings." 
                       ON "PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND" OFF)
IF( RWS_PYTHON_BINDINGS_ENABLED )
    FIND_PACKAGE(PythonInterp)
    FIND_PACKAGE(PythonLibs)
    IF( PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND )
        MESSAGE(STATUS "RobWorkStudio: Python bindings ENABLED!")
        INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
        SWIG_ADD_MODULE(rws python rws.i ScriptTypes.cpp)
        SWIG_LINK_LIBRARIES(rws rwstudio rwstudioapp ${RWS_COMPONENT_LIBRARIES} ${RWS_PLUGIN_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${PYTHON_LIBRARIES} )
        
        RW_TO_PYTHON_STR_LIST("${RWS_CXX_FLAGS};${RW_CXX_FLAGS}" RWS_SWIG_COMPILE_FLAGS)
        RW_TO_PYTHON_STR_LIST("${RWS_COMPONENT_LIBRARIES};${RWS_PLUGIN_LIBRARIES};${ROBWORKSTUDIO_LIBRARIES};${ROBWORK_LIBRARIES};${PYTHON_LIBRARIES}" RWS_SWIG_DEP_LIBS)
        RW_TO_PYTHON_STR_LIST("${ROBWORKSTUDIO_INCLUDE_DIR};${QT_INCLUDES};${PYTHON_INCLUDE_DIRS}" RWS_SWIG_DEP_INCLUDE_DIRS)
        RW_TO_PYTHON_STR_LIST("${ROBWORKSTUDIO_LIBRARY_DIRS}" RWS_SWIG_DEP_LIB_DIRS)
        RW_TO_PYTHON_STR_LIST("${swig_generated_sources}" RWS_SWIG_SOURCES)

        # create python stile lists
        #SET(RWS_SWIG_DEP_LIBS_TMP "${RWS_COMPONENT_LIBRARIES};${RWS_PLUGIN_LIBRARIES};${ROBWORKSTUDIO_LIBRARIES};${ROBWORK_LIBRARIES};${PYTHON_LIBRARIES}")
        #STRING(REPLACE ";" "," RWS_SWIG_DEP_LIBS ${RWS_SWIG_DEP_LIBS_TMP}) 
        # now add a costom command that compiles the
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in ${CMAKE_CURRENT_SOURCE_DIR}/setup.py)
        
        #ADD_CUSTOM_COMMAND(
            # Because we are building in a temporary directory, we have to use absolute
            # paths here.
            #OUTPUT
            #"${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}Stub.cpp"
            #"${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}Stub.hpp"
            
        #    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/setup.py build_ext --inplace
            #DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}.pkg"
        #)
        
        #LIST(APPEND SRC_FILES "${pkgname}.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}Stub.cpp" )
        #LIST(APPEND SRC_FILES_HPP "${pkgname}.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/${pkgname}Stub.cpp")
        
        
        
         
    ELSE()
        MESSAGE(STATUS "RobWorkStudio: Python bindings DISABLED!")
    ENDIF()
ELSE()
    MESSAGE(STATUS "RobWorkStudio: Python bindings DISABLED")
ENDIF()  
    
ENDIF()