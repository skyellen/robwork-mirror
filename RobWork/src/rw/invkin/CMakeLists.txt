SET(FILES_CPP
  AmbiguityResolver.cpp
  ClosedFormIK.cpp
  IterativeIK.cpp
  IterativeMultiIK.cpp
  CCDSolver.cpp
  ParallelIKSolver.cpp
  PieperSolver.cpp
  #ResolvedRateSolver.cpp
  IKMetaSolver.cpp
  JacobianIKSolver.cpp
  JacobianIKSolverM.cpp
  ClosedFormIKSolverUR.cpp
  ClosedFormIKSolverKukaIIWA.cpp
)

SET(FILES_HPP
  AmbiguityResolver.hpp
  ClosedFormIK.hpp
  IterativeIK.hpp
  IterativeMultiIK.hpp
  CCDSolver.hpp
  ParallelIKSolver.hpp
  PieperSolver.hpp
  IKMetaSolver.hpp
  JacobianIKSolver.hpp
  JacobianIKSolverM.hpp
  ClosedFormIKSolverUR.hpp
  ClosedFormIKSolverKukaIIWA.hpp
)

SOURCE_GROUP(invkin FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${INVKIN_FILES_HPP} DESTINATION "include/rw/invkin")
