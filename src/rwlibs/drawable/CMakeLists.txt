
add_library(
  rw_drawable
  ${RW_DRAWABLE_USER_FILES}
  ./Drawable.cpp
  ./Drawable3DS.cpp
  ./DrawableAC3D.cpp
  ./DrawableFactory.cpp
  ./DrawableFrame.cpp
  ./DrawableGeometry.cpp
  ./DrawableLines.cpp
  ./DrawableSTL.cpp
  ./DrawableTriSoup.cpp
  ./GLTexture.cpp
  ./Model_3DS.cpp
  ./WorkCellGLDrawer.cpp)

