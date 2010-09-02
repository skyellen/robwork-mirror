
if( OPENGL_FOUND AND OPENGL_GLU_FOUND )

SET(SRC_FILES
  ${RW_DRAWABLE_USER_FILES}
  ./Drawable.cpp
  ./DrawableFactory.cpp
  ./DrawableUtil.cpp
  ./Render.cpp  
  ./RenderFrame.cpp
  ./RenderGeometry.cpp
  ./RenderLines.cpp
  ./RenderMatrix.cpp
  ./WorkCellGLDrawer.cpp
  ./TactileArrayRender.cpp
  ./ac3d/LoaderAC3D.cpp
  ./3ds/Loader3DS.cpp
  ./3ds/Model3DS.cpp
  ./ivg/LoaderIVG.cpp
  ./obj/LoaderOBJ.cpp
  ./tri/LoaderTRI.cpp
  RWGLTexture.cpp
  RenderImage.cpp
  RenderModel3D.cpp
  Model3D.cpp
  RenderUtil.cpp
  RenderScan.cpp
  RenderPointCloud.cpp
    #./Render3DS.cpp
  #./RenderAC3D.cpp
  #./RenderSTL.cpp
  #./RenderOBJ.cpp
  #./RenderTriSoup.cpp
  #./RenderIVG.cpp
  #./IVGReader.cpp
  #./OBJReader.cpp
  #./Model_3DS.cpp
  
)

SET(SRC_FILES_HPP
  ./Drawable.hpp
  ./DrawableFactory.hpp
  ./DrawableUtil.hpp
  ./Render.hpp
  #./Render3DS.hpp
  #./RenderAC3D.hpp
  ./RenderFrame.hpp
  ./RenderGeometry.hpp
  ./RenderLines.hpp
  #./RenderSTL.hpp
  ./RenderMatrix.hpp
  #./RenderOBJ.hpp
  #./RenderTriSoup.hpp
  #./RenderIVG.hpp
  #./IVGReader.hpp
  #./OBJReader.hpp
  ./WorkCellGLDrawer.hpp
  RWGLTexture.hpp
  ./TactileArrayRender.hpp
  RenderUtil.hpp
  RenderImage.hpp
  RenderScan.hpp
  RenderPointCloud.hpp
)

ADD_LIBRARY(rw_drawable ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_drawable DESTINATION ${RW_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/drawable)

else ()
  message(SEND_WARNING "Opengl an Glu is needed for compiling the drawable library!")
endif()



