SET(SUBSYS_NAME rwsim_bullet)
set(SUBSYS_DESC "Bullet based physics engine plugin for RobWorkSim")
SET(SUBSYS_DEPS rwsim) # mband: The target 'rw' was not found, so removed it from this dependency list

SET(build FALSE)
set(DEFAULT ${RWSIM_HAVE_BULLET})

find_package(Bullet QUIET)
IF( NOT BULLET_FOUND )
    SET(DEFAULT FALSE)
    SET(REASON "Bullet not found!")
ENDIF()

RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)

    SET(SRC_FILES
    	BtUtil.cpp
    	BtBody.cpp
		BtDevice.cpp
		BtPositionDevice.cpp
		BtVelocityDevice.cpp
        BtSimulator.cpp
    	BtDebugRender.cpp
		BtConstraint.cpp
		BtTactileSensor.cpp
		BtMaterial.cpp
		BtContactStrategy.cpp
		BtRWCollisionAlgorithm.cpp
		BtRWCollisionConfiguration.cpp
    )
    
    SET(SRC_FILES_HPP
    	BtUtil.hpp
    	BtBody.hpp
		BtDevice.hpp
		BtPositionDevice.hpp
		BtVelocityDevice.hpp
        BtSimulator.hpp
    	BtDebugRender.hpp
		BtConstraint.hpp
		BtTactileSensor.hpp
		BtMaterial.hpp
		BtContactStrategy.hpp
		BtRWCollisionAlgorithm.hpp
		BtRWCollisionConfiguration.hpp
    )

	# Detect if Bullet is compiled in single or double precision by doing a test compilation 
	SET( BULLET_USE_SINGLE_PRECISION FALSE CACHE BOOL "Select to force compiling without -DBT_USE_DOUBLE_PRECISION." )
	SET( BULLET_USE_DOUBLE_PRECISION FALSE CACHE BOOL "Select to force compiling with -DBT_USE_DOUBLE_PRECISION." )
	IF(BULLET_USE_SINGLE_PRECISION)
    	MESSAGE( STATUS "Bullet will be compiled with single precision." )
    ELSEIF(BULLET_USE_DOUBLE_PRECISION)
    	MESSAGE( STATUS "Bullet will be compiled with double precision." )
    ELSE()
    	MESSAGE( STATUS "Testing Bullet for use of double precision..." )
	    SET(CMAKE_TRY_COMPILE_CONFIGURATION ${CMAKE_BUILD_TYPE})
	    TRY_COMPILE( COMPILE_SUCCESS ${CMAKE_BINARY_DIR}
	        ${CMAKE_CURRENT_SOURCE_DIR}/BtPrecisionTest.cpp
	        CMAKE_FLAGS
	            "-DINCLUDE_DIRECTORIES:string=${BULLET_INCLUDE_DIRS}"
	            "-DLINK_LIBRARIES:string=${BULLET_LIBRARIES}"
	        COMPILE_DEFINITIONS
	            "-DBT_USE_DOUBLE_PRECISION"
	        OUTPUT_VARIABLE OUT
	    )
	    IF(COMPILE_SUCCESS)
    		MESSAGE( STATUS "Bullet appears to be using double precision." )
			SET( BULLET_USE_SINGLE_PRECISION OFF CACHE BOOL "Select to force compiling without -DBT_USE_DOUBLE_PRECISION." FORCE )
			SET( BULLET_USE_DOUBLE_PRECISION ON CACHE BOOL "Select to force compiling with -DBT_USE_DOUBLE_PRECISION." FORCE )
    	ELSE()
    		MESSAGE( STATUS "Bullet appears to be using single precision." )
			SET( BULLET_USE_SINGLE_PRECISION ON CACHE BOOL "Select to force compiling without -DBT_USE_DOUBLE_PRECISION." FORCE )
			SET( BULLET_USE_DOUBLE_PRECISION OFF CACHE BOOL "Select to force compiling with -DBT_USE_DOUBLE_PRECISION." FORCE )
	    ENDIF()
    ENDIF()
    IF (BULLET_USE_DOUBLE_PRECISION)
		ADD_DEFINITIONS(-DBT_USE_DOUBLE_PRECISION)
	ENDIF()

    RW_ADD_LIBRARY(${SUBSYS_NAME} rwsim_bullet ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(rwsim_bullet "rwsimlibs/bullet" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    
    ADD_LIBRARY(bt_plugin.rwplugin MODULE BtPlugin.cpp BtPlugin.hpp)
    TARGET_LINK_LIBRARIES(bt_plugin.rwplugin rwsim_bullet ${ROBWORKSIM_LIBRARIES})

ENDIF()


