
if( OPENGL_FOUND AND OPENGL_GLU_FOUND )

add_library(
  rw_drawable
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
  ./RenderTriSoup.cpp
  ./RenderIVG.cpp
  ./IVGReader.cpp
  ./GLTexture.cpp
  ./Model_3DS.cpp
  ./WorkCellGLDrawer.cpp)
  		
else ()
  message(SEND_WARNING "Opengl an Glu is needed for compiling the drawable library!")
endif()



