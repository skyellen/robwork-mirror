SET(FILES_CPP
  DynamicLibraryLoader.cpp
  DynamicLibraryLoaderBase.cpp
  PluginFactory.cpp
  PluginFactoryBase.cpp
  PluginRepository.cpp
)

SET(FILES_HPP
  DynamicLibraryLoader.hpp
  DynamicLibraryLoaderBase.hpp
  PluginFactory.hpp
  PluginFactoryBase.hpp
  PluginRepository.hpp
  FactoryMacro.hpp
)

SOURCE_GROUP(proximity FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${PROXIMITY_FILES_HPP} DESTINATION "include/rw/proximity/")
