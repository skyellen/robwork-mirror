SET(FILES_CPP
  Common.cpp
  ConcatVectorIterator.cpp
  IOUtil.cpp
  Log.cpp
  LogWriter.cpp
  LogBufferedChar.cpp
  LogBufferedMsg.cpp
  LogStreamWriter.cpp
  LogMultiWriter.cpp
  LogFileWriter.cpp
  LogMultiWriter.cpp
  Message.cpp
  Property.cpp
  PropertyBase.cpp
  PropertyMap.cpp
  PropertyType.cpp
  StringUtil.cpp
  Timer.cpp
  TimerUtil.cpp
  VectorIterator.cpp
  Cache.cpp
  FileCache.cpp
  PairMap.cpp
  Exception.cpp
  Event.cpp
  ProgramOptions.cpp
  ThreadPool.cpp
  ThreadSafeQueue.cpp
  ThreadSafeStack.cpp
  ThreadSafeVariable.cpp
  ThreadTask.cpp
  
  Archive.cpp
  BoostXMLParser.cpp
  DOMElem.cpp
  DOMParser.cpp
  InputArchive.cpp
  OutputArchive.cpp
  INIArchive.cpp
  BINArchive.cpp
  Serializable.cpp
  
  Extension.cpp
  ExtensionPoint.cpp
  ExtensionRegistry.cpp
  Plugin.cpp

)

SET(FILES_HPP
  AnyPtr.hpp 
  Common.hpp
  ConcatVectorIterator.hpp
  IOUtil.hpp
  Log.hpp
  LogWriter.hpp
  LogBufferedChar.hpp
  LogBufferedMsg.hpp
  LogStreamWriter.hpp
  LogMultiWriter.hpp
  LogFileWriter.hpp
  LogMultiWriter.hpp
  Message.hpp
  Property.hpp
  PropertyBase.hpp
  PropertyMap.hpp
  PropertyType.hpp
  StringUtil.hpp
  Timer.hpp
  TimerUtil.hpp
  VectorIterator.hpp
  Cache.hpp
  FileCache.hpp
  PairMap.hpp
  Exception.hpp
  ScopedTimer.hpp
  Event.hpp
  ProgramOptions.hpp
  ThreadPool.hpp
  ThreadSafeQueue.hpp
  ThreadSafeStack.hpp
  ThreadSafeVariable.hpp
  ThreadTask.hpp
  
  Archive.hpp
  BoostXMLParser.hpp
  DOMElem.hpp
  DOMParser.hpp
  InputArchive.hpp
  OutputArchive.hpp
  INIArchive.hpp
  BINArchive.hpp
  Serializable.hpp

  Extension.hpp
  ExtensionPoint.hpp
  ExtensionRegistry.hpp
  Plugin.hpp
  
)

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${COMMON_FILES_HPP} DESTINATION "include/rw/common/")
