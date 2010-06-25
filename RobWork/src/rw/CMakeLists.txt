FILE(GLOB SRC_FILES_HPP robwork.hpp use_robwork_namespace.hpp)
set(ENV{RW_LIB_FILES_CPP} "RobWork.cpp;${RW_USER_FILES}")
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


#Add the RobWork files
SET(FILES_CPP
    RobWork.cpp
)

SET(FILES_HPP
	RobWork.hpp
)

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")


ADD_LIBRARY(rw $ENV{RW_LIB_FILES_CPP} $ENV{RW_LIB_FILES_HPP})

INSTALL(TARGETS rw DESTINATION ${RW_LIB_INSTALL_DIR})
INSTALL(FILES 
    rw.hpp 
    use_robwork_namespace.hpp 
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
    DESTINATION "${RW_HPP_INSTALL_DIR}/rw")

