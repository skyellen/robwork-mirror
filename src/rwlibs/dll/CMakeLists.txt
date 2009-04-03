
SET(SRC_FILES
  DynamicLibraryLoader.cpp
  DynamicLibraryLoaderBase.cpp    
)

SET(SRC_FILES_HPP
  DynamicLibraryLoader.hpp
  DynamicLibraryLoaderBase.hpp    
)

ADD_LIBRARY(rw_dll ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_dll DESTINATION lib)
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/dll)
