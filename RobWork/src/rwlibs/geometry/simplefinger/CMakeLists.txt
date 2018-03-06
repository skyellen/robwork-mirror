RW_ADD_LIBRARY(SimpleFinger SimpleFinger SimpleFinger.cpp SimpleFinger.hpp)
TARGET_LINK_LIBRARIES(SimpleFinger PRIVATE rw_csg PUBLIC rw)

RW_ADD_LIBRARY(simplefinger.rwplugin SimpleFinger MODULE SimpleFingerPlugin.cpp SimpleFingerPlugin.hpp)
TARGET_LINK_LIBRARIES(simplefinger.rwplugin PRIVATE SimpleFinger PUBLIC rw)

IF(CMAKE_VERSION VERSION_GREATER 3.3)
	SET_TARGET_PROPERTIES(SimpleFinger PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
ENDIF()