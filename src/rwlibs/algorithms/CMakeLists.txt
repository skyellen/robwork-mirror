
SET(SRC_FILES
  ${RW_ALGORITHMS_USER_FILES}
  ./kdtree/KDTree.cpp
  ./qpcontroller/IKQPSolver.cpp
  ./qpcontroller/QPController.cpp
  ./qpcontroller/QPSolver.cpp
  ./xqpcontroller/XQPController.cpp
  ./xqpcontroller/NullSpaceProjection.cpp
  ./gpm/BasicGPM.cpp
)

SET(SRC_FILES_HPP
  ./kdtree/KDTree.hpp
  ./qpcontroller/IKQPSolver.hpp
  ./qpcontroller/QPController.hpp
  ./qpcontroller/QPSolver.hpp
  ./xqpcontroller/XQPController.hpp
  ./xqpcontroller/NullSpaceProjection.hpp
  ./gpm/BasicGPM.hpp
)

ADD_LIBRARY(rw_algorithms ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_algorithms DESTINATION lib)
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/algorithms)
  