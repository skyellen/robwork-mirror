
add_library(
  rw_algorithms ${RW_ALGORITHMS_USER_FILES}
  ./kdtree/KDTree.cpp
  ./qpcontroller/IKQPSolver.cpp
  ./qpcontroller/QPController.cpp
  ./qpcontroller/QPSolver.cpp
  ./xqpcontroller/XQPController.cpp
  ./xqpcontroller/NullSpaceProjection.cpp
  ./gpm/BasicGPM.cpp)
