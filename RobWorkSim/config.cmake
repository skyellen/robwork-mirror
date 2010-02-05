# -*- cmake -*-

#
#Set RobWork, RobWorkStudio and RobWorkApp ROOTs
# 
set(RW_ROOT ${ROOT}/../RobWork/)
set(RWSTUDIO_ROOT ${ROOT}/../RobWorkStudio)

# add a path to the boost thread library both link directory and include path
#set(LibraryList ${LibraryList} -lboost_thread-mt)

# Include global compiler settings.
include(${RW_ROOT}/config.cmake)

set(RWSIM_HAVE_RWPHYS TRUE)

#
# Bullet and Ode include,link and libs
# 
#set(BulletIncludes 
#	"c:/Program files/BULLET_PHYSICS/include"
#	${RWAPP_ROOT}/ext/bullet/Extras/GIMPACT/include
#	${RWAPP_ROOT}/ext/bullet/Extras
#	${RWAPP_ROOT}/ext/bullet/Demos
#)
#set(BulletLinkDir "c:/Program files/BULLET_PHYSICS/lib")
#set(BulletLibs 	
#    LibGIMPACTUtils 
#	LibGIMPACT
#	LibConvexDecomposition  
#	LibOpenGLSupport 
#	LibBulletDynamics 
#	LibBulletCollision 
#	LibLinearMath
#)

SET(USE_OPENCV OFF)
#SET(OpenCV_ROOT_DIR "C:/Program Files/OpenCV/")
 

#set(OdeIncludes	${RWAPP_ROOT}/ext/ode/include)
#set(OdeLinkDir ${RWAPP_ROOT}/ext/ode/lib/ReleaseSingleLib)
#set(OdeLibs ode_single)

#set(OdeIncludes	"D:/sim/ode-0.11/include")
#set(OdeLinkDir "D:/sim/ode-0.11/lib/ReleaseSingleLib")
#set(OdeLibs "ode_single")

#set(OdeLinkDir "D:/sim/ode-0.11/lib/DebugSingleLib")
#set(OdeLibs "ode_singled")

SET(GRASPING_INCLUDE_DIRS "${RWAPP_ROOT}/userprojects/GraspPlanning/src")
SET(GRASPING_LIBRARY_DIRS "${RWAPP_ROOT}/userprojects/GraspPlanning/libs/Release")
SET(GRASPING_LIBRARIES "grasping")


set(OdeIncludes	"C:/jimalilocal/ode-0.11.1/include")
set(OdeLinkDir "C:/jimalilocal/ode-0.11.1/lib/ReleaseSingleDll")
add_definitions(-DdSINGLE)
set(OdeLibs "ode_single.lib")
