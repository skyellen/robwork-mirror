# This depends on glut32.lib so try and find it
FIND_PACKAGE(GLUT)

IF(GLUT_FOUND)
    MESSAGE(STATUS "GLUT found - SimpleViewer added!")    
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
    
    SET(LIBRARIES
      ${RW_DRAWABLE_LIBRARY_LIST}
      ${RW_PATHPLANNERS_LIBRARY_LIST}
      ${RW_PROXIMITYSTRATEGIES_LIBRARY_LIST}
      ${ROBWORK_LIBRARIES}
      ${GLUT_LIBRARIES}
    )
    
    ADD_LIBRARY(rwtools_simpleviewer ${SrcFiles}) 
    
    # To create an application use
    ADD_EXECUTABLE(ViewerExample ViewerExample.cpp)
    TARGET_LINK_LIBRARIES(ViewerExample rwtools_simpleviewer ${LIBRARIES})
    
    INSTALL(TARGETS ViewerExample DESTINATION bin)
ELSE()
    MESSAGE(STATUS "GLUT not found - SimpleViewer is not added!")
ENDIF()