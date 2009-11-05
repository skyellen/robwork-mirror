SET(FILES_CPP
  PlannerConstraint.cpp
  QEdgeConstraint.cpp
  PathPlanner.cpp
  QToQPlanner.cpp
  QToTPlanner.cpp
  QToQSamplerPlanner.cpp
  PlannerUtil.cpp
  StopCriteria.cpp
  QConstraint.cpp
  QNormalizer.cpp
  QIKSampler.cpp
  QSampler.cpp
  StateConstraint.cpp
  PathAnalyzer.cpp
)

SET(FILES_HPP
  PlannerConstraint.hpp
  QEdgeConstraint.hpp
  PathPlanner.hpp
  QToQPlanner.hpp
  QToTPlanner.hpp
  QToQSamplerPlanner.hpp
  PlannerUtil.hpp
  StopCriteria.hpp
  QConstraint.hpp
  QNormalizer.hpp
  QIKSampler.hpp
  QSampler.hpp
  StateConstraint.hpp
  PathAnalyzer.hpp
)

SOURCE_GROUP(pathplanning FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${PATHPLANNING_FILES_HPP} DESTINATION "include/rw/pathplanning/")
