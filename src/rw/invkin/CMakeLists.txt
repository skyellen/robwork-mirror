SET(FILES_CPP
  ClosedFormIK.cpp
  IterativeIK.cpp
  IterativeMultiIK.cpp
  CCDSolver.cpp
  ParallelIKSolver.cpp
  PieperSolver.cpp
  ResolvedRateSolver.cpp
  SimpleSolver.cpp
  SimpleMultiSolver.cpp  
  IKMetaSolver.cpp
)

SET(FILES_HPP
  ClosedFormIK.hpp
  IterativeIK.hpp
  IterativeMultiIK.hpp
  CCDSolver.hpp
  ParallelIKSolver.hpp
  PieperSolver.hpp
  ResolvedRateSolver.hpp
  SimpleSolver.hpp
  SimpleMultiSolver.hpp  
  IKMetaSolver.hpp
)

SOURCE_GROUP(invkin FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${INVKIN_FILES_HPP} DESTINATION "include/rw/invkin")
