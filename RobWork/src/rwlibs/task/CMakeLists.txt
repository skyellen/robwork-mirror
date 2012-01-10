set(Ext ${PROJECT_SOURCE_DIR}/ext)

# The camera file and include path to use.

SET(SRC_FILES
  ./Action.cpp
  ./Entity.cpp
  ./Motion.cpp
  ./Target.cpp
  ./Task.cpp
  ./TaskUtils.cpp
  ./TypeRepository.cpp
  ./GraspTask.cpp
)


SET(SRC_FILES_HPP
  ./Action.hpp
  ./Entity.hpp
  ./Motion.hpp
  ./Target.hpp
  ./Task.hpp
  ./TaskUtils.hpp
  ./TypeRepository.hpp
  ./GraspTask.hpp
)


IF (RW_HAVE_XERCES) 
	set(SRC_FILES ${SRC_FILES}
	  ./loader/XMLTaskFormat.cpp
  	  ./loader/XMLTaskLoader.cpp
  	  ./loader/XMLTaskSaver.cpp
	)
	set(SRC_FILES_HPP ${SRC_FILES_HPP}
	  ./loader/XMLTaskFormat.hpp
  	  ./loader/XMLTaskLoader.hpp
  	  ./loader/XMLTaskSaver.hpp
   )
ENDIF()   
  	  

ADD_LIBRARY(rw_task ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_task DESTINATION ${RW_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/task)
