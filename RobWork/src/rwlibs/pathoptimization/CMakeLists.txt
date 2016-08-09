SET(SUBSYS_NAME rw_pathoptimization)
set(SUBSYS_DESC "Path optimization algorithms")
SET(SUBSYS_DEPS rw_proximitystrategies rw)

SET(build TRUE)
set(DEFAULT TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} )
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    SET(SRC_FILES
      ./clearance/ClearanceOptimizer.cpp
      ./clearance/ClearanceCalculator.cpp
      ./clearance/MinimumClearanceCalculator.cpp
      ./pathlength/PathLengthOptimizer.cpp
    )
     
    SET(SRC_FILES_HPP
      ./clearance/ClearanceOptimizer.hpp
      ./clearance/ClearanceCalculator.hpp
      ./clearance/MinimumClearanceCalculator.hpp
      ./pathlength/PathLengthOptimizer.hpp
    )
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} pathoptimization ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(pathoptimization "rwlibs/pathoptimization" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
