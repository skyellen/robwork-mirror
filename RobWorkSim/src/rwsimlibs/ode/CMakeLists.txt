set(Ext ${PROJECT_SOURCE_DIR}/ext)

IF (RWSIM_HAVE_ODE)

    SET(SRC_FILES
    	ODESimulator.cpp
    	ODEVelocityDevice.cpp
    	ODEKinematicDevice.cpp
    	ODESuctionCupDevice.cpp
    	ODEBody.cpp
    	ODEJoint.cpp
    	ODEUtil.cpp
    	ODEDebugRender.cpp
    	ODETactileSensor.cpp
    	ODEMaterialMap.cpp
    	ODESuctionCupDevice.cpp
    	ODEConstraint.cpp
    	ODEContactStrategy.cpp
    	ODEThreading.cpp
    )
    
    SET(SRC_FILES_HPP
    	ODESimulator.hpp
    	ODEVelocityDevice.hpp
    	ODEKinematicDevice.hpp
    	ODESuctionCupDevice.hpp
    	ODEBody.hpp
    	ODEJoint.hpp
    	ODEUtil.hpp
    	ODEDebugRender.hpp
    	ODETactileSensor.hpp
    	ODEMaterialMap.hpp
    	ODEConstraint.hpp
    	ODEContactStrategy.hpp
    	ODEThreading.hpp
    )
    
    # Detect if ODE comes with threading_impl.h file (version 0.13 and newer).
    IF(NOT DEFINED ODE_WITH_THREADING_IMPL)
    	MESSAGE( STATUS "Testing ODE for the threading_impl.h file..." )
    	IF(EXISTS "${ODE_INCLUDE_DIR}/ode/threading_impl.h")
    		MESSAGE( STATUS "ODE appears to have threading_impl.h file (version 0.13 or newer)." )
    		SET(ODE_WITH_THREADING_IMPL TRUE CACHE BOOL "Select if ODE has threading_impl.h file (version 0.13 and newer).")
    	ELSE()
	   		MESSAGE( STATUS "ODE does not appear to have threading_impl.h file (version 0.12 or earlier)." )
    		SET(ODE_WITH_THREADING_IMPL FALSE CACHE BOOL "Select if ODE does not have threading_impl.h file (version 0.12 or earlier).")
	   	ENDIF()
	ENDIF()
	IF(ODE_WITH_THREADING_IMPL)
    	MESSAGE( STATUS "ODE threading_impl.h detected - assuming version 0.13 and later." )
		SET_SOURCE_FILES_PROPERTIES(ODEThreading.cpp PROPERTIES COMPILE_DEFINITIONS ODE_WITH_THREADING_IMPL)
    ELSEIF(ODE_WITH_THREADING_IMPL)
    	MESSAGE( STATUS "ODE threading_impl.h NOT detected - assuming version 0.12 and earlier." )
    ENDIF()
    
    ADD_LIBRARY(rwsim_ode ${SRC_FILES} ${SRC_FILES_HPP})
    
    ADD_LIBRARY(ode_plugin.rwplugin MODULE ODEPlugin.cpp ODEPlugin.hpp)
    TARGET_LINK_LIBRARIES(ode_plugin.rwplugin rwsim_ode ${ROBWORKSIM_LIBRARIES})
    #INSTALL(TARGETS rwsim_ode DESTINATION ${RWSIM_LIB_INSTALL_DIR})
    #INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/simulation)

ENDIF()