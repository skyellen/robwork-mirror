
SET(FILES_CPP
  ${RW_DRAWABLE_USER_FILES}
  Model3D.cpp
  DrawableGeometryNode.cpp
  ProjectionMatrix.cpp
  SceneGraph.cpp
  SceneNode.cpp
  DrawableNode.cpp
  DrawableNodeClone.cpp
  GroupNode.cpp
  SceneCamera.cpp
  WorkCellScene.cpp
  SceneViewer.cpp
  Render.cpp  
  SceneDescriptor.cpp
)

SET(FILES_HPP
  Model3D.hpp
  DrawableGeometryNode.hpp
  ProjectionMatrix.hpp
  SceneGraph.hpp
  SceneNode.hpp
  DrawableNode.hpp
  GroupNode.hpp
  SceneCamera.hpp
  WorkCellScene.hpp
  SceneViewer.hpp
  Render.hpp   
  DrawableNodeClone.hpp
)

SOURCE_GROUP(graphics FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")


