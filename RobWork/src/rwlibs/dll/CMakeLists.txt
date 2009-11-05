
SET(SRC_FILES
  DynamicLibraryLoader.cpp
  DynamicLibraryLoaderBase.cpp    
)

SET(SRC_FILES_HPP
  DynamicLibraryLoader.hpp
  DynamicLibraryLoaderBase.hpp    
)

ADD_LIBRARY(rw_dll ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_dll DESTINATION ${RW_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION "${RW_HPP_INSTALL_DIR}/rwlibs/dll")
