SET(SUBSYS_NAME rw_task)
set(SUBSYS_DESC "Generic Task description")
SET(SUBSYS_DEPS rw)

SET(DEFAULT TRUE)
SET(REASON)

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
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
      ./GraspTarget.cpp
      ./GraspResult.cpp
      ./GraspSubTask.cpp
      ./loader/DOMTaskFormat.cpp
      ./loader/DOMTaskLoader.cpp
      ./loader/DOMTaskSaver.cpp
      ./loader/TaskLoader.cpp
      ./loader/TaskSaver.cpp
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
      ./GraspTarget.hpp
      ./GraspResult.hpp
      ./GraspSubTask.hpp
      ./loader/DOMTaskFormat.hpp
      ./loader/DOMTaskLoader.hpp
      ./loader/DOMTaskSaver.hpp
      ./loader/TaskLoader.hpp
      ./loader/TaskSaver.hpp
    )
    
    IF(RW_HAVE_XERCES)
	set(SRC_FILES ${SRC_FILES}
	  ./loader/XMLTaskFormat.cpp
  	  ./loader/XMLTaskLoader.cpp
  	  ./loader/XMLTaskSaver.cpp
	)
	set(LOADER_SRC_FILES_HPP 
	  ./loader/XMLTaskFormat.hpp
  	  ./loader/XMLTaskLoader.hpp
  	  ./loader/XMLTaskSaver.hpp
   )
   ENDIF()
      	  
    RW_ADD_LIBRARY(${SUBSYS_NAME} task ${SRC_FILES} ${SRC_FILES_HPP} ${LOADER_SRC_FILES_HPP})
    RW_ADD_INCLUDES(task "rwlibs/task" ${SRC_FILES_HPP})     
    RW_ADD_INCLUDE_DIRS(task "rwlibs/task" loader)
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()