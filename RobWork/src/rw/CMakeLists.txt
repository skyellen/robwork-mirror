SET(SUBSYS_NAME rw)
set(SUBSYS_DESC "The core RobWork library!")
SET(SUBSYS_DEPS )

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME})

IF(build)

FILE(GLOB SRC_FILES_HPP robwork.hpp)
set(ENV{RW_LIB_FILES_CPP} "${RW_USER_FILES}")
set(ENV{RW_LIB_FILES_HPP} "${SRC_FILES_HPP};${RW_USER_FILES_HPP};")
 
ADD_SUBDIRECTORY(common)
ADD_SUBDIRECTORY(geometry)
ADD_SUBDIRECTORY(invkin)
ADD_SUBDIRECTORY(kinematics)
ADD_SUBDIRECTORY(loaders)
ADD_SUBDIRECTORY(math)
ADD_SUBDIRECTORY(models)
ADD_SUBDIRECTORY(pathplanning)
ADD_SUBDIRECTORY(graspplanning)
ADD_SUBDIRECTORY(proximity)
ADD_SUBDIRECTORY(sensor)
ADD_SUBDIRECTORY(trajectory)
ADD_SUBDIRECTORY(plugin)
ADD_SUBDIRECTORY(graphics)

#Add the RobWork files
SET(FILES_CPP
    RobWork.cpp
)

SET(FILES_HPP
    rw.hpp 
    common.hpp
    geometry.hpp
    invkin.hpp
    kinematics.hpp
    loaders.hpp
    math.hpp
    models.hpp
    pathplanning.hpp
    sensor.hpp
    trajectory.hpp
    proximity.hpp
	RobWork.hpp
)

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

RW_ADD_LIBRARY(rw rw $ENV{RW_LIB_FILES_CPP} $ENV{RW_LIB_FILES_HPP})
RW_ADD_INCLUDES(rw "rw" ${FILES_HPP} )
RW_ADD_INCLUDE_DIRS(rw "rw" 
    common geometry invkin kinematics loaders math models pathplanning 
    proximity sensor trajectory plugin graphics constraints)

ENDIF()
