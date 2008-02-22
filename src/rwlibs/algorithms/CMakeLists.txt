
add_library(
  rw_algorithms
  ${RW_ALGORITHMS_USER_FILES}
  ./jtag/ContourExtractor.cpp
  ./jtag/CornerExtractor.cpp
  ./jtag/JTagDetector.cpp
  ./jtag/JTagGenerator.cpp
  ./jtag/JTagMarker.cpp
  ./jtag/Line2D.cpp	
  ./kdtree/KDTree.cpp
  ./qpcontroller/IKQPSolver.cpp
  ./qpcontroller/QPController.cpp
  ./qpcontroller/QPSolver.cpp
  ./xqpcontroller/XQPController.cpp
  ./xqpcontroller/NullSpaceProjection.cpp
  ./gpm/BasicGPM.cpp)
