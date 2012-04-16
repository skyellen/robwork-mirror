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
    )
    
    ADD_LIBRARY(rwsim_ode ${SRC_FILES} ${SRC_FILES_HPP})
    
    #INSTALL(TARGETS rwsim_ode DESTINATION ${RWSIM_LIB_INSTALL_DIR})
    #INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/simulation)

ENDIF()