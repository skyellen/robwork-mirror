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
  ./loader/XMLTaskFormat.cpp
  ./loader/XMLTaskLoader.cpp
  ./loader/XMLTaskSaver.cpp
)

SET(SRC_FILES_HPP
  ./Action.hpp
  ./Entity.hpp
  ./Motion.hpp
  ./Target.hpp
  ./Task.hpp
  ./TaskUtils.hpp
  ./TypeRepository.hpp
  ./loader/XMLTaskFormat.hpp
  ./loader/XMLTaskLoader.hpp
  ./loader/XMLTaskSaver.hpp
)

ADD_LIBRARY(rw_task ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_task DESTINATION lib)
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/task)
