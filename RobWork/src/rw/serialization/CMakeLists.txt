SET(FILES_CPP
Archive.cpp
INIArchive.cpp
InputArchive.cpp
OutputArchive.cpp
serializeutil.cpp
)

SET(FILES_HPP
Archive.hpp
INIArchive.hpp
InputArchive.hpp
OutputArchive.hpp
serializeutil.hpp
)

SOURCE_GROUP(serialization FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

