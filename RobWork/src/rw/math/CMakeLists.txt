SET(FILES_CPP
  CameraMatrix.cpp
  EAA.cpp
  Jacobian.cpp
  Line2D.cpp
  LinearAlgebra.cpp
  InertiaMatrix.cpp
  Math.cpp
  Metric.cpp
  MetricFactory.cpp
  MetricUtil.cpp
  Polynomial.cpp
  PolynomialSolver.cpp
  Pose6D.cpp
  Q.cpp
  Quaternion.cpp
  Rotation2D.cpp
  Rotation3D.cpp
  Rotation3DVector.cpp
  RPY.cpp
  Statistics.cpp
  Transform2D.cpp
  Transform3D.cpp
  Vector2D.cpp
  Vector3D.cpp
  VectorND.cpp
  Vector.cpp
  VelocityScrew6D.cpp
  PerspectiveTransform2D.cpp
  EigenDecomposition.cpp
  Line2DPolar.cpp
  Pose2D.cpp
  Wrench6D.cpp
  ProjectionMatrix.cpp
  #serialization.cpp
)

SET(FILES_HPP
  CameraMatrix.hpp
  EAA.hpp
  Jacobian.hpp
  Line2D.hpp
  LinearAlgebra.hpp
  InertiaMatrix.hpp
  Math.hpp
  Metric.hpp
  MetricFactory.hpp
  MetricUtil.hpp
  Polynomial.hpp
  PolynomialSolver.hpp
  Pose6D.hpp
  Q.hpp
  Quaternion.hpp
  Rotation2D.hpp
  Rotation3D.hpp
  Rotation3DVector.hpp
  RPY.hpp
  #Statistics.hpp
  Transform2D.cpp
  Transform3D.hpp
  Vector2D.hpp
  Vector3D.hpp
  VelocityScrew6D.hpp
  Wrench6D.hpp
  PerspectiveTransform2D.hpp
  EigenDecomposition.hpp
  ProjectionMatrix.hpp
)

SOURCE_GROUP(math FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${MATH_FILES_HPP} DESTINATION "include/rw/math/")
