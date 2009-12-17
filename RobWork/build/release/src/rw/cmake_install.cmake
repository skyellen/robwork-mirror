# Install script for directory: /home/lpe/workspace/RobWork/src/rw

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")
  IF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/libs/Release" TYPE STATIC_LIBRARY FILES "/home/lpe/workspace/RobWork/libs/Release/librw.a")
  ENDIF("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/rw" TYPE FILE FILES
    "/home/lpe/workspace/RobWork/src/rw/robwork.hpp"
    "/home/lpe/workspace/RobWork/src/rw/use_robwork_namespace.hpp"
    "/home/lpe/workspace/RobWork/src/rw/common.hpp"
    "/home/lpe/workspace/RobWork/src/rw/geometry.hpp"
    "/home/lpe/workspace/RobWork/src/rw/invkin.hpp"
    "/home/lpe/workspace/RobWork/src/rw/kinematics.hpp"
    "/home/lpe/workspace/RobWork/src/rw/loaders.hpp"
    "/home/lpe/workspace/RobWork/src/rw/math.hpp"
    "/home/lpe/workspace/RobWork/src/rw/models.hpp"
    "/home/lpe/workspace/RobWork/src/rw/pathplanning.hpp"
    "/home/lpe/workspace/RobWork/src/rw/sensor.hpp"
    "/home/lpe/workspace/RobWork/src/rw/trajectory.hpp"
    "/home/lpe/workspace/RobWork/src/rw/proximity.hpp"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" MATCHES "^(Unspecified)$")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/common/cmake_install.cmake")
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/geometry/cmake_install.cmake")
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/invkin/cmake_install.cmake")
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/kinematics/cmake_install.cmake")
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/loaders/cmake_install.cmake")
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/math/cmake_install.cmake")
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/models/cmake_install.cmake")
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/pathplanning/cmake_install.cmake")
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/proximity/cmake_install.cmake")
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/sensor/cmake_install.cmake")
  INCLUDE("/home/lpe/workspace/RobWork/build/release/src/rw/trajectory/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

