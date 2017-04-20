# Setting CMP0046 to OLD, because the yaobi, pqp and assimp libraries in add_dependencies can point to non-existing targets when using the external libraries (e.g. '/usr/lib/x86_64-linux-gnu/libassimp.so' and not those distributed with RobWork 'assimp')
if(POLICY CMP0046)
  cmake_policy(SET CMP0046 OLD)
endif()

SET(SUBSYS_NAME rw_lua )
SET(SUBSYS_DESC "Interface for accessing RobWorkSim from lua." )
SET(SUBSYS_DEPS rw )

# make sure that SDH can be found
SET(build TRUE)

FIND_PACKAGE(SWIG 1.3 QUIET)
set(DEFAULT TRUE)
set(REASON )
IF( NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND( build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
   SET(RWSIM_HAVE_LUA TRUE CACHE INTERNAL "")
	#MESSAGE(STATUS "SWIG found adding swig modules!")
	INCLUDE(UseSWIG)
	
	SET(CMAKE_SWIG_FLAGS "")

	SET_SOURCE_FILES_PROPERTIES(../rw.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../rw.i PROPERTIES SWIG_FLAGS "-includeall")
	
	# lua interface generation
	SET(SWIG_MODULE_rw_lua_EXTRA_DEPS ../rwmath.i ../rwplanning.i)
	STRING(REPLACE "rw_lua" "" LIBS_WITHOUT_RW_LUA ${ROBWORK_LIBRARIES})
	IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
		SWIG_ADD_LIBRARY(rw_lua MODULE LANGUAGE lua SOURCES ../rw.i ../ScriptTypes.cpp Lua.cpp LuaState.cpp)
	ELSE()
		SWIG_ADD_MODULE(rw_lua lua ../rw.i ../ScriptTypes.cpp Lua.cpp LuaState.cpp)
	ENDIF()
	
	SWIG_LINK_LIBRARIES(rw_lua rw_algorithms
                              rw_pathplanners
                              rw_pathoptimization
                              rw_simulation
                              rw_opengl
                              rw_assembly
                              ${LUA_LIBRARIES}
                              rw_proximitystrategies
                              rw_task
                              ${YAOBI_LIBRARIES}
                              ${PQP_LIBRARIES}  
                              rw
                              ${OPENGL_LIBRARIES}
                              ${XERCESC_LIBRARIES}
                              ${ASSIMP_LIBRARIES}
                              ${ZLIB_LIBRARIES}
                              ${MINIZIP_LIBRARIES}
                              ${Boost_LIBRARIES}
                              ${LAPACK_LIBRARIES} 
                              ${BLAS_LIBRARIES}
                              rw_qhull)
    ADD_DEPENDENCIES(rw_lua rw_algorithms
                          rw_pathplanners
                          rw_pathoptimization
                          rw_simulation
                          rw_opengl
                          rw_proximitystrategies
                          rw_task
                          ${YAOBI_LIBRARIES}
                          ${PQP_LIBRARIES}  
                          rw
                          rw_qhull
                          ${ASSIMP_LIBRARIES}
                          ${ZLIB_LIBRARIES}
                          ${MINIZIP_LIBRARIES}
)
    
    ADD_LIBRARY(rw_lua_s STATIC Lua.cpp LuaState.cpp ${swig_generated_sources} ${swig_other_sources})
    LINK_LIBRARIES(rw_lua_s rw)

	# the dynamic version
    #ADD_LIBRARY(rw_lua DYNAMIC Lua.cpp ${swig_generated_sources} ${swig_other_sources})
    #LINK_LIBRARIES(rw_lua rw)
    
	# this is used to indicate static linking to Visual Studio or mingw
	IF (DEFINED MSVC) 
	    SET_TARGET_PROPERTIES(rw_lua_s PROPERTIES COMPILE_FLAGS "/DSTATIC_LINKED")
	ELSE()
	    SET_TARGET_PROPERTIES(rw_lua_s PROPERTIES COMPILE_FLAGS "-DSTATIC_LINKED")
	ENDIF()

    install(TARGETS rw_lua rw_lua_s 
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT swig
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT swig)
	
ENDIF()
