
# Standard files to compile:
SET(SRC_FILES_CPP
  ArcBall.cpp
  Convert.cpp
  RobWorkStudio.cpp
  ViewGL.cpp
  Event.cpp
  RobWorkStudioPlugin.cpp)

SET(SRC_FILES_HPP
  ArcBall.hpp
  Convert.hpp
  RobWorkStudio.hpp
  ViewGL.hpp
  Event.hpp
  RobWorkStudioPlugin.hpp)

# Files to moc and compile:
SET(MocHeaderFiles
  RobWorkStudioPlugin.hpp
  RobWorkStudio.hpp
  ViewGL.hpp)

SET(MocSrcFiles)
QT4_WRAP_CPP(MocSrcFiles ${MocHeaderFiles})

# Resource files to rcc and compile:
IF(RWS_BUILD_SANDBOX)
    SET(SandboxResourceFiles rwstudio_sandbox_resources.qrc)
ENDIF ()
QT4_ADD_RESOURCES(ResourceFiles rwstudio_resources.qrc ${SandboxResourceFiles})

# We create a static library named rwstudio that all plugins should link to. Our
# RobWorkStudio executable links to this library also.
ADD_LIBRARY(rwstudio
  ${SRC_FILES_CPP}
  ${MocSrcFiles}
  ${ResourceFiles})

# Install stuff
INSTALL(TARGETS rwstudio DESTINATION lib)
INSTALL(FILES ${SRC_FILES_CPP} ${SRC_FILES_HPP} DESTINATION include/rws)
