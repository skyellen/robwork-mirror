
add_library(
  rw_algorithms
  ${RW_ALGORITHMS_USER_FILES}
  ./qpcontroller/IKQPSolver.cpp
  ./qpcontroller/QPController.cpp
  ./qpcontroller/QPSolver.cpp
  ./xqpcontroller/XQPController.cpp)
