# if we want to use ui files add them here
SET(UIS_FILES SimTemplatePlugin.ui )

RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

QT4_WRAP_CPP(MocSrcFiles SimTemplatePlugin.hpp)
# Rcc the files:

QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
SET(SrcFiles SimTemplatePlugin.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(SimTemplatePlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SimTemplatePlugin bstrap ${ROBWORKSIM_LIBRARIES})

