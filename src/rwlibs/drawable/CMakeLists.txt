
if( OPENGL_FOUND AND OPENGL_GLU_FOUND )

SET(SRC_FILES
  ${RW_DRAWABLE_USER_FILES}
  ./Drawable.cpp
  ./DrawableFactory.cpp
  ./DrawableUtil.cpp
  ./Render.cpp
  ./Render3DS.cpp
  ./RenderAC3D.cpp
  ./RenderFrame.cpp
  ./RenderGeometry.cpp
  ./RenderLines.cpp
  ./RenderSTL.cpp
  ./RenderMatrix.cpp
  ./RenderOBJ.cpp
  ./RenderTriSoup.cpp
  ./RenderIVG.cpp
  ./IVGReader.cpp
  ./OBJReader.cpp
  ./GLTexture.cpp
  ./Model_3DS.cpp
  ./WorkCellGLDrawer.cpp
  ./TactileArrayRender.cpp
  RWGLTexture.cpp
)

SET(SRC_FILES_HPP
  ./Drawable.hpp
  ./DrawableFactory.hpp
  ./DrawableUtil.hpp
  ./Render.hpp
  ./Render3DS.hpp
  ./RenderAC3D.hpp
  ./RenderFrame.hpp
  ./RenderGeometry.hpp
  ./RenderLines.hpp
  ./RenderSTL.hpp
  ./RenderMatrix.hpp
  ./RenderOBJ.hpp
  ./RenderTriSoup.hpp
  ./RenderIVG.hpp
  ./IVGReader.hpp
  ./OBJReader.hpp
  ./WorkCellGLDrawer.hpp
  RWGLTexture.hpp
  ./TactileArrayRender.hpp
)

ADD_LIBRARY(rw_drawable ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_drawable DESTINATION lib)
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/drawable)

else ()
  message(SEND_WARNING "Opengl an Glu is needed for compiling the drawable library!")
endif()



