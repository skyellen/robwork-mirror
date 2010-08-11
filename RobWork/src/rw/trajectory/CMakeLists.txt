SET(FILES_CPP
  Blend.cpp
  CircularInterpolator.cpp
  CubicSplineFactory.cpp
  CubicSplineInterpolator.cpp
  Interpolator.cpp
  InterpolatorUtil.cpp
  LinearInterpolator.cpp
  LloydHaywardBlend.cpp
  ParabolicBlend.cpp
  Path.cpp
  Timed.cpp
  TimedUtil.cpp
  Trajectory.cpp
  TrajectoryFactory.cpp
  TrajectoryIterator.cpp
  InterpolatorTrajectory.cpp
  BlendedTrajectory.cpp
  TimeMetricUtil.cpp
)

SET(FILES_HPP
  Blend.hpp
  CircularInterpolator.hpp
  CubicSplineFactory.hpp
  CubicSplineInterpolator.hpp
  Interpolator.hpp
  InterpolatorUtil.hpp
  LinearInterpolator.hpp
  LloydHaywardBlend.hpp
  ParabolicBlend.hpp
  Path.hpp
  Timed.hpp
  TimedUtil.hpp
  Trajectory.hpp
  TrajectoryFactory.hpp
  TrajectoryIterator.hpp
  InterpolatorTrajectory.hpp
  BlendedTrajectory.hpp
  TimeMetricUtil.hpp
)

SOURCE_GROUP(trajectory FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${TRAJECTORY_FILES_HPP} DESTINATION "include/rw/trajectory/")
