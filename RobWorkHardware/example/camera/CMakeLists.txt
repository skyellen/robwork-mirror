
# check compiler/operating system
set(Ext ${PROJECT_SOURCE_DIR}/ext)
# The camera file and include path to use.

if (DEFINED WIN32)
  FIND_PACKAGE(CMU1394C)

   if( DEFINED MINGW AND CMU1394C_FOUND)
        
       link_directories(${Ext}/dcam_c)
	   #find_library(CMU1394 ${Ext}/dcam_c/1394Camera_c.lib ) #  1394camera.dll 1394camera.dll 1394camera.dll
	   #if (CMU1394)
	     message(STATUS "CMU1394C enabled! 1394camera_c.lib - found!")
	     
	     include_directories( ${Ext}/dcam_c/ )
	     include_directories(${Ext}/dcam/win/cmu1394/)
	     link_libraries(1394Camera_c.lib)
	     
	     add_executable(CMU1394_CcameraExample CMU1394_CcameraExample.cpp)
		    target_link_libraries(CMU1394_CcameraExample 
		    	rwhw_camera 
		    	${ROBWORK_LIBRARIES}
		    	1394camera_c.lib
		   )
	     
	   #else ()
	   #  message(STATUS "CMU1394C disabled! 1394camera_c.lib - not found!")
	   #endif ()
    else( ) 
	   find_library(CMU1394 1394camera.lib ) #  1394camera.dll 1394camera.dll 1394camera.dll
	   if (CMU1394)
	     message(STATUS "CMU1394 enabled! 1394camera.lib - found!")
	     set(CameraFile ./CMU1394Camera.cpp)
	     include_directories(${Ext}/dcam/win/cmu1394)
	   else ()
	     message(STATUS "CMU1394 disabled! 1394camera.lib - not found!")
	   endif ()


       # then its msvc
	  #include_directories(${Ext}/dcam/win/cmu1394)
	  #set(CameraFile ./camera/CMU1394Camera.cpp)
	endif()
	
else ()
  find_library(DC1394 dc1394_control)
  find_library(RAW1394 raw1394)
  if (DC1394 AND RAW1394)
    message("-- Looking for libdc1394 and libraw1394 - found")
    set(CameraFile DC1394cameraExample.cpp)
    include_directories(${Ext}/dcam/linux)
  else ()
    message("-- Looking for libdc1394 and libraw1394 - not found")
  endif ()
endif ()

# Add additional packages that are required by your project here
IF( DEFINED USE_OPENCV AND DEFINED OpenCV_ROOT_DIR)
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${RWHW_ROOT}/build/)

    SET(OpenCV_FIND_REQUIRED_COMPONENTS CV CXCORE HIGHGUI CVCAM)
    FIND_PACKAGE( OpenCV REQUIRED "CV CXCORE HIGHGUI CVCAM")
    INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIR} )
    #SET(LibraryList ${OPENCV_LIBRARIES} ${LibraryList})
       
    add_executable(OpenCVCameraExample OpenCVcameraExample.cpp)
    target_link_libraries(OpenCVCameraExample 
    	rwhw_camera 
    	${ROBWORK_LIBRARIES}
    	${OPENCV_LIBRARIES}
    )
ENDIF()

