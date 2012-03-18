SET(SRC_FILES
    Dummy.cpp
)

SET(SRC_FILES_HPP
    Dummy.hpp
)


RW_ADD_LIBRARY(rwstudio_sandbox sandbox ${SRC_FILES} ${SRC_FILES_HPP})
INSTALL(FILES ${SRC_FILES_HPP} DESTINATION ${INCLUDE_INSTALL_DIR}/sandbox/rws )
