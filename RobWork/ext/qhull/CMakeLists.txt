cmake_minimum_required(VERSION 2.6)

if(POLICY CMP0003)
  cmake_policy(SET CMP0003 NEW)
endif()

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

project(qhull)

add_subdirectory(src)

