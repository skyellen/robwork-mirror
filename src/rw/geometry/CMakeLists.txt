SET(FILES_CPP
  FaceArrayFactory.cpp
  Geometry.cpp
  GeometryBox.cpp
  GeometryCylinder.cpp
  GeometryFactory.cpp
  GeometrySTL.cpp
  GeometryFace.cpp
)

SET(FILES_HPP
  FaceArrayFactory.hpp
  Geometry.hpp
  GeometryBox.hpp
  GeometryCylinder.hpp
  GeometryFactory.hpp
  GeometrySTL.hpp
  GeometryFace.hpp
)

SOURCE_GROUP(geometry FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${GEOMETRY_FILES_HPP} DESTINATION "include/rw/geometry")
