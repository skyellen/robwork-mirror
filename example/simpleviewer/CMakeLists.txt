#
# Source files
#
set(SrcFiles
    ViewerExample.cpp
	SimpleGLViewer.cpp
	EventListener.cpp
	Menu.cpp 
	MenuItem.cpp 
	ArcBall.cpp  	  
)


# To create an application use
add_executable(SimpleViewer ${SrcFiles})
target_link_libraries(SimpleViewer ${RW_LIBRARIES})
