
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

