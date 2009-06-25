#
# Source files
#
set(SrcFiles
	SimpleGLViewer.cpp
	EventListener.cpp
	Menu.cpp 
	MenuItem.cpp 
	ArcBall.cpp  	  
)

SET(TEST_LIBRARIES
  ${RW_DRAWABLE_LIBRARY_LIST}
  ${RW_PATHPLANNERS_LIBRARY_LIST}
  ${RW_PROXIMITYSTRATEGIES_LIBRARY_LIST}
  ${ROBWORK_LIBRARIES}
)

ADD_LIBRARY(rwtools_simpleviewer ${SrcFiles}) 

# To create an application use
ADD_EXECUTABLE(ViewerExample ViewerExample.cpp)
TARGET_LINK_LIBRARIES(ViewerExample rwtools_simpleviewer ${TEST_LIBRARIES} glut32.lib)

INSTALL(TARGETS ViewerExample DESTINATION bin)
