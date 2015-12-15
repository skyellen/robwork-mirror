SET(SUBSYS_NAME rw_opengl)
set(SUBSYS_DESC "Opengl visualization for RobWork")
SET(SUBSYS_DEPS rw)

if( OPENGL_FOUND AND OPENGL_GLU_FOUND )
    set(DEFAULT TRUE)
    set(REASON)
else()
    set(DEFAULT FALSE)
    set(REASON "OpenGl or Glu was not found.")
endif()

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS glu opengl)
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)

    SET(SRC_FILES
      ${RW_DRAWABLE_USER_FILES}
      Drawable.cpp
      RenderArrow.cpp
      RWGLFrameBuffer.cpp
      DrawableFactory.cpp
      DrawableUtil.cpp
      RenderFrame.cpp
      RenderGeometry.cpp
      RenderLines.cpp
      RenderMatrix.cpp
      SceneOpenGL.cpp
      TactileArrayRender.cpp
      RWGLTexture.cpp
      RenderImage.cpp
      RenderModel3D.cpp
      RenderUtil.cpp
      RenderScan.cpp
      RenderPointCloud.cpp
      RenderCameraFrustum.cpp
      DrawableGeometry.cpp
      RenderSmoothSkin.cpp
      RenderForceTorque.cpp
      RenderVelocity.cpp
    )
    
    SET(SRC_FILES_HPP
      Drawable.hpp
      RWGLFrameBuffer.hpp
      DrawableFactory.hpp
      DrawableUtil.hpp
      RenderFrame.hpp
      RenderGeometry.hpp
      RenderLines.hpp
      RenderMatrix.hpp
      SceneOpenGL.hpp
      TactileArrayRender.hpp
      RWGLTexture.hpp
      RenderImage.hpp
      RenderModel3D.hpp
      RenderUtil.hpp
      RenderScan.hpp
      RenderPointCloud.hpp
      RenderCameraFrustum.hpp
      DrawableGeometry.hpp
      RenderSmoothSkin.hpp
      RenderForceTorque.hpp
      RenderVelocity.hpp
    )
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} opengl ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(opengl "rwlibs/opengl" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
endif()
