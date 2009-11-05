#----------------------------------------------------------------------
# Define the following variables:

# Standard cpp files to compile:
SET(SrcFiles DeviceTab.cpp Jog.cpp)

# Header files to moc and compile:
QT4_WRAP_CPP(MocSrcFiles DeviceTab.hpp Jog.hpp)

# Rcc the files:
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

# Set extra linker flags for apple macos computers
SET(CMAKE_SHARED_LINKER_FLAGS "${ExtraLinkerFlags} ${CMAKE_SHARED_LINKER_FLAGS}")

# The library to build:
IF (RWS_USE_STATIC_LINK_PLUGINS)
  ADD_LIBRARY(rwstudio_jog STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
ELSE ()
  ADD_LIBRARY(rwstudio_jog SHARED ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
ENDIF ()

# Link the standard static libraries with with the shared library:
target_link_libraries(${TargetName} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
