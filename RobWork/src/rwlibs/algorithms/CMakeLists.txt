SET(SUBSYS_NAME rw_algorithms)
SET(SUBSYS_DESC "A collection of varius algorithms!")
SET(SUBSYS_DEPS rw)

set(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    SET(SRC_FILES
      ${RW_ALGORITHMS_USER_FILES}
      ./kdtree/KDTree.cpp
      ./kdtree/KDTreeQ.cpp
      ./qpcontroller/IKQPSolver.cpp
      ./qpcontroller/QPController.cpp
      ./qpcontroller/QPSolver.cpp
      ./xqpcontroller/XQPController.cpp
      ./xqpcontroller/NullSpaceProjection.cpp
      ./gpm/BasicGPM.cpp
      BFGS.cpp
      
      MovingAverage.cpp
      #StablePlanePose.cpp
      RANSACModel.cpp
      PointModel.cpp
      LineModel.cpp
      StructuredLineModel.cpp
      PlaneModel.cpp
      StablePose1DModel.cpp
      StablePose0DModel.cpp
    )
    
    SET(SRC_FILES_HPP
      ./kdtree/KDTree.hpp
      ./kdtree/KDTreeQ.hpp
      ./qpcontroller/IKQPSolver.hpp
      ./qpcontroller/QPController.hpp
      ./qpcontroller/QPSolver.hpp
      ./xqpcontroller/XQPController.hpp
      ./xqpcontroller/NullSpaceProjection.hpp
      ./gpm/BasicGPM.hpp
	
	  BFGS.hpp
      MovingAverage.hpp
      #StablePlanePose.hpp
      
      RANSACModel.hpp
      PointModel.hpp
      LineModel.hpp
      StructuredLineModel.hpp
      PlaneModel.hpp
      StablePose1DModel.hpp
      StablePose0DModel.cpp
    )
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} algorithms ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(algorithms "rwlibs/algorithms" MovingAverage.hpp StablePlanePose.hpp)
    RW_ADD_INCLUDE_DIRS(algorithms "rwlibs/algorithms" kdtree qpcontroller xqpcontroller gpm) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
