
SET(FILES_CPP
  ${RW_DRAWABLE_USER_FILES}
  ./ac3d/LoaderAC3D.cpp
  ./3ds/Loader3DS.cpp
  ./3ds/Model3DS.cpp
  #./ivg/LoaderIVG.cpp
  ./obj/LoaderOBJ.cpp
  ./tri/LoaderTRI.cpp

  Model3D.cpp
  Model3DFactory.cpp
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
  ProjectionMatrix.cpp
  Render.cpp  
  SceneDescriptor.cpp
)

SET(FILES_HPP
  Model3D.hpp
  Model3DFactory.hpp
  DrawableGeometryNode.hpp
  ProjectionMatrix.hpp
  SceneGraph.hpp
  SceneNode.hpp
  DrawableNode.hpp
  GroupNode.hpp
  SceneCamera.hpp
  WorkCellScene.hpp
  SceneViewer.hpp
  ProjectionMatrix.hpp
  Render.hpp   
  DrawableNodeClone.hpp
)

SOURCE_GROUP(graphics FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")


