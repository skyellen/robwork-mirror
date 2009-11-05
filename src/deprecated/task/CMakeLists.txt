SET(FILES_CPP
  AttachFrame.cpp
  ToolLocation.cpp
  Task.cpp
  Task_Trajectory.cpp
  Link.cpp
  Target.cpp
)

SET(FILES_HPP
  AttachFrame.hpp
  ToolLocation.hpp
  Task.hpp
  Task_Trajectory.hpp
  Link.hpp
  Target.hpp
)

SOURCE_GROUP(task FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${TASK_FILES_HPP} DESTINATION "include/rw/task/")
