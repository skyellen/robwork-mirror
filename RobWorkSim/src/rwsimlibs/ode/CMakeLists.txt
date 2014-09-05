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
    )
    
    ADD_LIBRARY(rwsim_ode ${SRC_FILES} ${SRC_FILES_HPP})
    
    ADD_LIBRARY(ode_plugin.rwplugin MODULE ODEPlugin.cpp ODEPlugin.hpp)
    TARGET_LINK_LIBRARIES(ode_plugin.rwplugin rwsim_ode ${ROBWORKSIM_LIBRARIES})
    #INSTALL(TARGETS rwsim_ode DESTINATION ${RWSIM_LIB_INSTALL_DIR})
    #INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/simulation)

ENDIF()