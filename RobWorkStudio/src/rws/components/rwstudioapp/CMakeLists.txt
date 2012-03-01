IF( RWS_HAVE_LUA )

#----------------------------------------------------------------------
# Define the following variables:

SET(SrcFiles RobWorkStudioApp.cpp)
QT4_WRAP_CPP(MocSrcFiles RobWorkStudioApp.hpp)

# Rcc the files:
QT4_ADD_RESOURCES(RccSrcFiles )

#----------------------------------------------------------------------
# Everything below should be common for all components:

# The library to build:
ADD_LIBRARY(rwstudioapp STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
INSTALL(TARGETS rwstudioapp DESTINATION ${RWS_LIB_INSTALL_DIR})

ADD_DEPENDENCIES(rwstudioapp rwstudio)

SET(ENV{RWS_COMPONENT_LIBRARIES} "$ENV{RWS_COMPONENT_LIBRARIES}${TargetName};")

ENDIF()


