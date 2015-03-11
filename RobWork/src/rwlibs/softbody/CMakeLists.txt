SET(SUBSYS_NAME rw_softbody)
set(SUBSYS_DESC "Various simulations of soft bodies")
SET(SUBSYS_DEPS rw)

SET(build TRUE)
SET(DEFAULT FALSE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    SET(SRC_FILES
      ./numerics/TrapMethod.cpp
      ./numerics/FdUtil.cpp
      ./beam/BeamGeometry.cpp
      ./beam/BeamGeometryCuboid.cpp
      ./beam/ModRusselBeamBase.cpp
      ./beam/RusselIntegrand.cpp
      ./beam/EBBeam.cpp
      ./beam/BeamObstaclePlane.cpp 
      ./beam/BeamStartGuess.cpp 
      ./beam/ModRussel_NLP.cpp
      ./beam/ModRusselBeamIpopt.cpp
    )
    
    SET(SRC_FILES_HPP
      ./numerics/TrapMethod.hpp
      ./numerics/FdUtil.hpp
      ./beam/BeamGeometry.hpp
      ./beam/BeamGeometryCuboid.hpp
      ./beam/ModRusselBeamBase.hpp
      ./beam/RusselIntegrand.hpp
      ./beam/EBBeam.hpp
      ./beam/BeamObstaclePlane.hpp 
      ./beam/BeamStartGuess.hpp
      ./beam/ModRussel_NLP.hpp
      ./beam/ModRusselBeamIpopt.hpp
    )
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(softbody "rwlibs/softbody" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
endif()

